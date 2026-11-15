"""정지 상태 IMU 점검 (모터를 돌리지 않음, EEPROM 을 바꾸지 않음).

    python -m rl.imu_check --port COM4            진자를 손대지 말고 60 s
    python -m rl.imu_check --port COM4 --secs 120
    python -m rl.imu_check --fake --fast           가짜 보드로 동작 확인

보내는 명령: HELLO, GET, STOP, STREAM <div> 뿐 (REF·RUN·U·CAL·ZERO·BETA 없음).
RL 펌웨어(AEROPEND_RL)와 실험용 펌웨어(AEROPEND_LAB) 모두 됩니다(같은 명령·D 줄 형식).
모터가 꺼진 채 D 줄을 모아 다음을 계산합니다.
- 잔여 자이로 편향 = 정지 중 ω 평균 [°/s] (보드가 저장된 gyroBias 를 뺀 뒤의 값)
- θ 표류율 = θ 의 기울기 [°/s], θ − θacc 간극의 처음·끝, θacc 기울기(가속도계는 표류하지 않아야 함)
- Madgwick 보정 한계 ≈ 2β [rad/s] → °/s (β 는 쿼터니언 변화율에 걸리고 각속도는 그 2배). 정규화된 기울기 방향으로
  일정 크기만 보정하므로, 이 예산을 수평 두 축(진자 축과 그에 수직인 수평 축)의 자이로 편향이 나눠 씁니다.
  진자 축 편향만 보이는 이 점검으로는 한계를 정확히 알 수 없어서, 판정은 실제 θ 표류(θ − θacc 간극 변화)로 합니다.
판정: θ 가 떠내려가면 이상(정지 자이로 보정 CAL 또는 β 상향), 안 떠내려가도 편향이 크면 평형 오프셋이 생기므로 주의.
(11-15: 처음 판정식은 한계를 β 로 잡아 절반으로 과소평가했음.)
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np


def run(link, secs: float = 60.0, div: int = 2, log=print) -> dict:
    lines = []
    link.open()
    try:
        fw = link.hello(sink=lambda m: lines.append(m), require_rl=False)   # 실험용 펌웨어에서도 됨
        m = link.request('STOP', 'EVT', timeout=2.0, sink=lambda m: lines.append(m))
        cfgm = link.request('GET', 'CFG', timeout=2.0, sink=lambda m: lines.append(m))
        kv = dict(cfgm.kv or {})
        cal = None
        t_end = link.now() + 1.5
        while link.now() < t_end:                       # GET 뒤 CALVAL·RLCFG 까지
            m = link.recv(0.2)
            if m is None:
                continue
            if m.raw.startswith('CALVAL'):
                cal = m.raw
            if m.kind == 'RLCFG':
                break
        for x in lines:
            if x.raw.startswith('CALVAL'):
                cal = cal or x.raw
        link.request(f'STREAM {div}', 'CFG', timeout=2.0)
        t, th, om, tha, u = [], [], [], [], []
        t_end = link.now() + secs
        while link.now() < t_end:
            m = link.recv(0.2)
            if m is None or m.kind != 'D' or len(m.f) < 6:
                continue
            f = m.f
            t.append(f[0] / 1000.0)
            u.append(f[1])
            th.append(f[2] * 0.01)
            om.append(f[3] * 0.01)
            tha.append(f[4] * 0.01)
        try:
            link.request('STOP', 'EVT', timeout=2.0)
        except Exception:
            pass
    finally:
        link.close()
    t, th, om, tha, u = map(np.array, (t, th, om, tha, u))
    if len(t) < 50:
        raise SystemExit(f'D 줄이 너무 적습니다 ({len(t)})')
    t = t - t[0]
    beta = float(kv.get('beta', 0)) / 1000.0                       # CFG beta = β×1000
    cap = 2.0 * beta * 180.0 / math.pi                             # 각속도 보정 한계 ≈ 2β (수평 두 축이 나눠 씀)
    bias = float(om.mean())
    w_sd = float(om.std())
    th_rate = float(np.polyfit(t, th, 1)[0])
    tha_rate = float(np.polyfit(t, tha, 1)[0])
    n5 = max(1, int(len(t) * 0.1))
    gap0 = float(np.mean(th[:n5] - tha[:n5]))
    gap1 = float(np.mean(th[-n5:] - tha[-n5:]))
    drift = abs(th_rate - tha_rate) > 0.05 or abs(gap1 - gap0) > 2.0
    moving = w_sd > 1.0 or float(np.std(tha)) > 1.5
    if moving:
        verdict = '움직임이 있음 (ω σ 또는 θacc σ 가 큼) — 진자를 멈춘 채 다시 재세요'
    elif drift:
        verdict = ('이상: 정지 중에도 θ 가 θacc 에서 떠내려감 (편향이 Madgwick 보정 예산을 넘음) — '
                   '정지 자이로 보정(CAL) 또는 β 상향')
    elif abs(bias) < 0.1:
        verdict = '정상: 잔여 편향이 작고 θ 가 떠내려가지 않음'
    elif abs(bias) < 0.5 * cap:
        verdict = ('주의: θ 는 떠내려가지 않지만 편향이 있어 θ 가 θacc 와 조금 어긋난 채 유지됨 — '
                   '모터 진동이 편향을 더하면 넘칠 수 있음')
    else:
        verdict = ('경고: 아직 떠내려가지는 않지만 편향이 보정 예산에 가까움 — 온도·진동으로 조금만 더 커지면 표류. '
                   'β 상향 또는 운전 온도에서 CAL 권장')
    r = dict(fw=fw, samples=len(t), secs=round(float(t[-1]), 1), motor_off=bool((u == 0).all()), beta=beta,
             cap_dps=round(cap, 3), bias_dps=round(bias, 4), omega_sd=round(w_sd, 3), theta_rate_dps=round(th_rate, 4),
             theta_acc_rate_dps=round(tha_rate, 4), gap_start=round(gap0, 2), gap_end=round(gap1, 2),
             theta_mean=round(float(th.mean()), 2), theta_acc_mean=round(float(tha.mean()), 2),
             theta_acc_sd=round(float(np.std(tha)), 3), calval=cal, verdict=verdict)
    log(f'펌웨어 {fw}, D {len(t)} 줄 / {t[-1]:.0f} s, 모터 꺼짐 {r["motor_off"]}')
    log(f'저장된 보정: {cal}')
    log(f'Madgwick β {beta:g} → 각속도 보정 예산 ≈ 2β = {cap:.3f} °/s (수평 두 축 공유)')
    log(f'잔여 자이로 편향 {bias:+.3f} °/s (ω σ {w_sd:.3f}), θ 기울기 {th_rate:+.3f} °/s, θacc 기울기 {tha_rate:+.3f} °/s')
    log(f'θ − θacc: 처음 {gap0:+.2f}° → 끝 {gap1:+.2f}°  (θ 평균 {r["theta_mean"]:.2f}°, θacc 평균 {r["theta_acc_mean"]:.2f}°)')
    log(f'판정: {verdict}')
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description='정지 상태 IMU 점검 (모터 안 돌림, EEPROM 안 바꿈)')
    ap.add_argument('--port')
    ap.add_argument('--fake', action='store_true')
    ap.add_argument('--fast', action='store_true', help='가짜 보드 가상시간')
    ap.add_argument('--secs', type=float, default=60.0)
    ap.add_argument('--div', type=int, default=2, help='STREAM 분주 (2 = 10 ms, 학습기와 같음)')
    args = ap.parse_args(argv)
    if args.fake:
        from .config import PlantCfg
        from .fake_board import FakeBoard, FakePort
        from .link import SerialLink, VirtualLink
        board = FakeBoard(PlantCfg(), seed=3)
        link = VirtualLink(board) if args.fast else SerialLink(FakePort(board), name='fake')
    elif args.port:
        from .link import SerialLink
        link = SerialLink(args.port)
    else:
        ap.error('--port COMx 또는 --fake')
    run(link, args.secs, args.div)
    return 0


if __name__ == '__main__':
    sys.exit(main())
