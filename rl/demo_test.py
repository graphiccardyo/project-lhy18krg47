"""시연용 펌웨어(AEROPEND_DEMO 1.0) 벤치 시험 — 모터는 돌지 않습니다(BENCH 1, 모터 핀 LOW).

    .venv\\Scripts\\python.exe -m rl.demo_test --port COM4            (프로토콜 + RL 끝-끝 비교)
    .venv\\Scripts\\python.exe -m rl.demo_test --port COM4 --nn 200   (+ NN 명령 정책 계산 비교: Build-Demo.ps1 -NnTest 빌드)

시험 내용
1. 연결·식별·설정 줄, 명령 오류 처리, PID/NOTCH/LIM/MODE, 하트비트 정지(LINK_TIMEOUT).
2. (--nn N) 리플레이 관측 N 개를 `NN` 으로 보내 보드의 정수 누산값이 rl/demo_policy.QuantPolicy 와 정확히 같은지.
3. RL 끝-끝: BENCH 1 에서 현재 각도로 REF → MODE RL → 정착 1 s 뒤 EVT RL ON → 기준을 몇 번 바꿈.
   보드의 D 줄(θ·ω·ref 는 보드가 관측에 쓴 것과 같은 cdeg 값)로
   (a) 펌웨어 RL 계산을 float32 로 옮긴 BoardEmu 를 돌려 결정마다 PWM·행동 a 가 보드와 같은지(누락은 D 줄로 추론해
       S 줄 miss 수와 맞춰 봄), (b) 같은 줄을 학습기 코드 rl/env.py EpisodeEnv 에 넣어 만든 관측 8개가 BoardEmu 의
       관측과 같은지 비교합니다. (a)+(b) = 보드가 학습 때와 같은 관측으로 같은 정책을 계산한다는 확인.
   (float64 학습기와 float32 보드는 관측 정수가 가끔 1 LSB 달라 행동이 0.01 정도 다를 수 있어서 직접 비교하지 않음)
   (이 비교는 D 줄 10 ms(OUT 1)가 필요 — 직렬 출력이 늘어 누락이 생길 수 있어 참고로만 봄)
4. 시연 설정(OUT 2, D 줄 20 ms)으로 같은 RL 시험을 다시 해서 누락 0·추론 시간·틱 시간을 확인합니다.
EEPROM 은 바꾸지 않습니다(CAL/ZERO/BETA 를 보내지 않음). 끝나면 STOP, BENCH 0.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

from .config import load_run_config
from .demo_policy import build, quant_x
from .env import EpisodeEnv

RL8_CONFIG = os.path.join('data', '20261115_rl8', 'config.json')


class Board:
    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 250000, timeout=0.05)
        self.buf = b''
        self.lines = []          # (시각, 줄) 전부
        self.last_ping = 0.0

    def send(self, cmd):
        self.s.write((cmd + '\n').encode())

    def poll(self, ping=True):
        now = time.time()
        if ping and now - self.last_ping > 0.25:
            self.send('PING')
            self.last_ping = now
        data = self.s.read(self.s.in_waiting or 1)
        out = []
        if data:
            self.buf += data
            *done, self.buf = self.buf.split(b'\n')
            for ln in done:
                t = ln.decode(errors='replace').strip()
                if t:
                    out.append(t)
                    self.lines.append((now, t))
        return out

    def wait_for(self, pred, timeout=2.0, ping=True):
        t0 = time.time()
        got = []
        while time.time() - t0 < timeout:
            for ln in self.poll(ping):
                got.append(ln)
                if pred(ln):
                    return ln, got
        return None, got

    def cmd(self, c, expect, timeout=1.5):
        self.send(c)
        ln, got = self.wait_for(lambda l: any(l.startswith(e) for e in expect), timeout)
        return ln


class Checker:
    def __init__(self):
        self.ok = 0
        self.fail = []

    def check(self, name, cond, info=''):
        if cond:
            self.ok += 1
            print(f'  통과  {name}' + (f'  ({info})' if info else ''))
        else:
            self.fail.append(name)
            print(f'  실패  {name}' + (f'  ({info})' if info else ''))


def parse_d(ln):
    if not ln.startswith('D,'):
        return None
    try:
        return [int(x) for x in ln[2:].split(',')]
    except ValueError:
        return None


def run_rl(b, dur):
    """현재 각도로 REF → MODE RL → RL ON → 기준 세 번 변경 → MODE PID → STOP. (D 줄, S 줄, EVT) 를 돌려줌."""
    b.lines.clear()
    b.wait_for(lambda l: False, 0.5)
    th = [x[2] for x in (parse_d(l) for _, l in b.lines) if x]
    home = int(round(np.median(th) / 10.0)) * 10 if th else 0
    b.cmd(f'REF {home}', ['EVT REF'])
    b.cmd('MODE RL', ['MODE'])
    b.wait_for(lambda l: l == 'EVT RL ON', 3.0)
    t0 = time.time()
    steps = [(0.25, home + 300), (0.5, home - 200), (0.75, home + 100)]
    si = 0
    while time.time() - t0 < dur:
        frac = (time.time() - t0) / dur
        if si < len(steps) and frac >= steps[si][0]:
            b.send(f'REF {steps[si][1]}')
            si += 1
        b.poll()
    b.cmd('MODE PID', ['MODE'])
    b.wait_for(lambda l: False, 1.2)
    b.send('STOP')
    b.wait_for(lambda l: l == 'EVT STOP', 1.0, ping=False)
    rows = [x for x in (parse_d(l) for _, l in b.lines) if x and len(x) == 8]
    # S 줄의 miss 는 RL 시작 때만 0 이 되므로 EVT RL ON 뒤 ~ RL OFF 전 것만
    # RL 이 끝난 뒤 첫 S 줄의 miss = 그 RL 구간의 최종 누락 수 (다음 RL 시작 때까지 유지됨)
    s_lines, on, final_miss = [], False, None
    for _, l in b.lines:
        if l == 'EVT RL ON':
            on = True
        elif l.startswith('EVT RL OFF') or l.startswith('EVT FALLBACK'):
            on = False
        elif l.startswith('S,'):
            if on:
                s_lines.append(l)
            elif s_lines and final_miss is None:
                final_miss = int(l.split(',')[11])
    if final_miss is not None:
        s_lines.append(s_lines[-1].rsplit(',', 2)[0] + f',{final_miss},' + s_lines[-1].rsplit(',', 1)[1])
    return rows, s_lines, [l for _, l in b.lines if l.startswith('EVT')]


class BoardEmu:
    """펌웨어 rlStep/nnFinish 를 float32 로 그대로 옮긴 것. 보드 D 줄(10 ms)로 한 줄씩 돌리며 보드와 비교.

    보드가 결정 k 의 추론을 제때 못 끝내면(miss) 결과를 버리므로, 결정 k 의 마무리(prevDu·prevA·나머지)는
    적용 샘플 k+2 줄을 보고 정합니다: PWM 과 a_milli 가 '적용됨'과 맞으면 정상, '그대로'와 맞으면 누락."""
    F = np.float32
    AW, AI, KIX = F(0.46651200), F(0.00332779), F(0.012)
    DU_NONE = -32768

    def __init__(self, qp, u0, tol=0.0015):
        F = self.F
        self.qp, self.tol = qp, tol
        self.rlU = max(40, min(255, u0))
        self.first, self.n = True, 0
        self.resid = self.ei = self.wf = F(0)
        self.prevA = self.prevXi = self.prevTh = self.lastA = F(0)
        self.prevDu, self.prevU = 0, 0
        self.pend = None            # 결정 k: (du, a, 나머지(마무리 후), 나머지(전))
        self.misses = self.bad_u = self.bad_a = 0
        self.decisions = []         # (줄 번호, 관측 float32 8개, du, a)

    @staticmethod
    def _round(x):                  # lround: 0.5 는 0 에서 먼 쪽
        return int(math.floor(abs(x) + 0.5)) * (1 if x >= 0 else -1)

    def _qx(self, v):
        return max(-32767, min(32767, self._round(float(self.F(v) * self.F(2048)))))

    def line(self, k, r):
        F = self.F
        _, pwm_b, thC, omC, _, _, refC, am = r
        a_b = am / 1000.0
        th, om = F(thC) * F(0.01), F(omC) * F(0.01)
        missed = None
        if self.n == 0 and not self.first:          # 적용 시점: 결정 k−2 마무리
            du, a, res_after, res_before = self.pend
            u_n = max(40, min(255, self.rlU + du))
            ok_n = u_n == pwm_b and abs(float(a) - a_b) <= self.tol
            ok_m = self.rlU == pwm_b and abs(float(self.lastA) - a_b) <= self.tol
            if ok_n or not ok_m:
                if not ok_n:
                    self.bad_u += u_n != pwm_b
                    self.bad_a += abs(float(a) - a_b) > self.tol
                self.rlU = pwm_b                    # 어긋나도 보드 값으로 맞춰 이어 감
                self.prevDu, self.prevA, self.resid, self.lastA = du, a, res_after, a
                missed = False
            else:
                self.misses += 1
                self.prevDu, self.resid = self.DU_NONE, res_before
                missed = True
        e = F(refC) * F(0.01) - th
        self.wf = om if self.first else F(self.wf + self.AW * (om - self.wf))
        self.ei = F(self.ei + self.AI * (e - self.ei))
        if self.n == 0:
            aS, dth = F(0), F(0)
            if not self.first:
                d_eff = self.rlU - self.prevU
                if d_eff == self.prevDu:
                    aS = self.prevA
                else:
                    xp = min(F(1), max(F(-1), F(F(d_eff) / F(10)) - self.prevXi))
                    aS = -F(np.sqrt(-xp)) if xp < 0 else F(np.sqrt(xp))
                dth = min(F(5), max(F(-5), th - self.prevTh))
            o = [e / F(10), (th - F(25)) / F(10), self.wf / F(100), dth, (F(self.rlU) - F(125)) / F(85), aS,
                 self.ei / F(2), F(np.tanh(e / F(0.5)))]
            xq = np.array([self._qx(v) for v in o], np.int64)
            xi = self.KIX * e
            self.prevXi, self.prevU, self.prevTh = xi, self.rlU, th
            mu = self.qp.mu_int(xq)[0]
            a = min(F(1), max(F(-1), F(np.tanh(F(mu)))))
            res0 = self.resid
            res = F(res0 + (a * abs(a) + xi) * F(10))
            du = max(-10, min(10, self._round(float(res))))
            res = min(F(1), max(F(-1), F(res - F(du))))
            self.pend = (du, a, res, res0)
            self.decisions.append((k, np.array(o, np.float64), du, float(a)))
        self.first = False
        self.n = (self.n + 1) % 2
        return missed


def compare_rl(C, qp, rows, evts, board_miss):
    """(1) 보드 = float32 에뮬레이터 (정확히), (2) 에뮬레이터 관측 = 학습기 rl/env.py EpisodeEnv 관측."""
    C.check('정착 1 s 뒤 EVT RL ON', 'EVT RL ON' in evts)
    i0 = next((i for i, r in enumerate(rows) if r[5] == 3), None)
    if i0 is None or i0 == 0:
        C.check('RL 샘플', False, '없음')
        return
    seg = []
    for r in rows[i0:]:
        if r[5] != 3:
            break
        seg.append(r)
    C.check('MODE PID → EVT RL OFF, mode 1', 'EVT RL OFF' in evts and rows[i0 + len(seg)][5] == 1, ', '.join(evts[-4:]))
    gaps = np.diff([r[0] for r in seg])
    C.check('D 줄 빠짐 없음 (간격 ≤ 13 ms)', len(gaps) > 0 and gaps.max() <= 13 and gaps.min() >= 7,
            f'{len(seg)}줄, 간격 {gaps.min()}–{gaps.max()} ms')
    emu = BoardEmu(qp, rows[i0 - 1][1])        # 시작 PWM = RL 이 시작된 샘플의 PID 출력
    cfg = load_run_config(RL8_CONFIG)
    env = EpisodeEnv(cfg)
    env.arm(lambda o: qp.act(o), cfg.rl.dec, cfg.rl.du_max)
    d_obs = []
    for k, r in enumerate(seg[:-2]):
        missed = emu.line(k, r)
        # 학습기 상태를 보드 상태에 맞춤 (누락·float32 차이로 갈라지지 않게) → 관측 만드는 식만 비교
        if missed is not None:
            env.resid = float(emu.resid)
            env.prev_du = emu.prevDu if not missed else 10 ** 6
            env.prev_apol = float(emu.prevA)
        m = type('M', (), {})()
        m.f = [r[0], r[1], r[2], r[3], r[4], 1, r[6], 0]
        m.info = None
        env.on_D(m)
        if m.info is not None and emu.decisions and emu.decisions[-1][0] == k:
            d_obs.append(np.abs(m.info['o'].astype(np.float64) - emu.decisions[-1][1]).max())
    nd = len(emu.decisions)
    C.check('보드 PWM = 에뮬레이터 (모든 결정)', nd > 100 and emu.bad_u == 0, f'{nd}결정, 불일치 {emu.bad_u}')
    C.check('보드 행동 a = 에뮬레이터 (±0.0015)', emu.bad_a == 0, f'불일치 {emu.bad_a}')
    C.check('추론한 누락 수 = 보드 S 줄 miss', emu.misses == board_miss, f'{emu.misses} / {board_miss}')
    C.check('관측 8개: 에뮬레이터 = rl/env.py (|Δ| < 1e-4)', len(d_obs) > 100 and max(d_obs) < 1e-4,
            f'{len(d_obs)}결정, 최대 |Δo| {max(d_obs):.2e}')
    pwm = [r[1] for r in seg]
    C.check('PWM 범위 [40, 255]', min(pwm) >= 40 and max(pwm) <= 255, f'{min(pwm)}–{max(pwm)}')


def timing(C, s_lines, tag, strict):
    if not s_lines:
        C.check('S 줄', False)
        return
    f = [[int(x) for x in l[2:].split(',')] for l in s_lines]
    nn_us = max(x[9] for x in f)
    miss = max(x[10] for x in f)
    mt = max(x[4] for x in f)
    ov = max(x[5] for x in f) - min(x[5] for x in f)
    load = max(x[11] for x in f) if len(f[0]) > 11 else -1
    if strict:
        C.check(f'누락 miss 0 {tag}', miss == 0, f'{miss}')
        C.check(f'추론 벽시계 < 20 ms {tag}', 0 < nn_us < 20000, f'최대 {nn_us / 1000:.1f} ms')
        C.check(f'틱 maxTick < 5000 µs, 오버런 0 {tag}', mt < 5000 and ov == 0, f'maxTick {mt} µs, 오버런 +{ov}, 틱 부하 {load}%')
    else:
        print(f'  참고  {tag}: miss {miss}, 추론 벽시계 최대 {nn_us / 1000:.1f} ms, maxTick {mt} µs, 틱 부하 {load}%')


def main(argv=None):
    ap = argparse.ArgumentParser(description='시연용 펌웨어 벤치 시험 (모터 안 돎)')
    ap.add_argument('--port', default='COM4')
    ap.add_argument('--nn', type=int, default=0, help='NN 명령으로 비교할 관측 수 (NN_TEST 빌드 필요)')
    ap.add_argument('--rl-s', type=float, default=10.0, help='RL 끝-끝 시험 길이 [s]')
    args = ap.parse_args(argv)

    print('정책 양자화 (리플레이 버퍼 읽는 중)…', flush=True)
    net, qp, O = build()
    C = Checker()
    b = Board(args.port)
    time.sleep(2.0)                      # 포트 열면 우노가 재부팅
    b.poll(False)
    try:
        print('[1] 연결·명령')
        ln = b.cmd('HELLO', ['READY'])
        C.check('HELLO → READY AEROPEND_DEMO', ln is not None and 'AEROPEND_DEMO' in ln, ln or '응답 없음')
        _, got = b.wait_for(lambda l: l.startswith('MODE'), 1.0)
        cfg_ln = next((l for _, l in b.lines[::-1] if l.startswith('CFG ')), '')
        C.check('CFG 줄 (eeprom=1, beta=10)', 'eeprom=1' in cfg_ln and 'beta=10' in cfg_ln, cfg_ln)
        imu = any(l.startswith('IMU READY') for _, l in b.lines)
        C.check('IMU READY', imu)
        ln = b.cmd('BENCH 1', ['CFG', 'ERR'])
        C.check('BENCH 1', ln is not None and 'bench=1' in ln, ln)
        C.check('알 수 없는 명령 → ERR UNKNOWN_COMMAND', b.cmd('FOO', ['ERR']) == 'ERR UNKNOWN_COMMAND')
        C.check('PID 범위 밖 → ERR BAD_ARGS', b.cmd('PID -1 0 0 150', ['ERR', 'PIDCFG']) == 'ERR BAD_ARGS')
        ln = b.cmd('PID 900 7000 20 140 40', ['PIDCFG', 'ERR'])
        C.check('PID 900 7000 20 140 40', ln == 'PIDCFG kp=0.900 ki=7.000 kd=0.020 ff=140 notch=235 zz=150 zp=700 umin=40', ln)
        ln = b.cmd('NOTCH 0', ['PIDCFG', 'ERR'])
        C.check('NOTCH 0', ln is not None and 'notch=0' in ln, ln)
        ln = b.cmd('NOTCH 235 150 700', ['PIDCFG', 'ERR'])
        C.check('NOTCH 235 150 700', ln is not None and 'notch=235 zz=150 zp=700' in ln, ln)
        ln = b.cmd('PID 4300 13300 0 150 40', ['PIDCFG', 'ERR'])
        C.check('PID 기본값 복원', ln == 'PIDCFG kp=4.300 ki=13.300 kd=0.000 ff=150 notch=235 zz=150 zp=700 umin=40', ln)
        ln = b.cmd('LIM 255 10000', ['CFG', 'ERR'])
        C.check('LIM 255 10000', ln is not None and 'umax=255 alim=10000' in ln, ln)
        C.check('REF 범위 밖 → ERR BAD_ARGS', b.cmd('REF 9500', ['ERR', 'EVT']) == 'ERR BAD_ARGS')
        C.check('MODE RL → MODE RL', b.cmd('MODE RL', ['MODE', 'ERR']) == 'MODE RL')
        C.check('MODE PID → MODE PID', b.cmd('MODE PID', ['MODE', 'ERR']) == 'MODE PID')

        # 하트비트: REF 로 유지 시작 → PING 끊기 → 1 s 뒤 LINK_TIMEOUT
        ln = b.cmd('REF 0', ['EVT REF', 'ERR'])
        C.check('REF 0 → EVT REF 0', ln == 'EVT REF 0', ln)
        ln, got = b.wait_for(lambda l: l.startswith('EVT ABORT'), 2.5, ping=False)
        C.check('하트비트 끊김 → EVT ABORT LINK_TIMEOUT', ln == 'EVT ABORT LINK_TIMEOUT', ln)
        got += b.wait_for(lambda l: False, 0.3, ping=False)[1]
        d = [parse_d(l) for l in got if l.startswith('D,')]
        modes = {x[5] for x in d if x and len(x) == 8}
        C.check('D 줄 9 필드, 유지 중 mode 1 → 정지 뒤 0 (OUT 2 = 20 ms)', all(x and len(x) == 8 for x in d) and 1 in modes and 0 in modes,
                f'modes {sorted(modes)}')

        # ---- 2. NN
        if args.nn:
            print(f'[2] NN 정책 계산 {args.nn}개')
            rng = np.random.default_rng(1)
            idx = rng.integers(0, len(O), args.nn)
            bad = 0
            us = []
            dmu = []
            for o in O[idx]:
                xq = quant_x(o[None, :])[0]
                ln = b.cmd('NN ' + ' '.join(str(int(v)) for v in xq), ['NN ', 'ERR'], 2.0)
                if not ln or not ln.startswith('NN '):
                    C.check('NN 응답', False, ln or '응답 없음 (NN_TEST 빌드인지 확인)')
                    break
                acc3, mu6, t_us = (int(v) for v in ln[3:].split())
                exp = int(qp.acc3(xq)[0])
                bad += acc3 != exp
                us.append(t_us)
                dmu.append(abs(mu6 * 1e-6 - float(qp.mu_int(xq)[0])))
            else:
                C.check('보드 정수 누산값 = QuantPolicy (정확히)', bad == 0, f'불일치 {bad}/{args.nn}')
                C.check('μ float 마무리 |Δ| < 1e-5', max(dmu) < 1e-5, f'최대 {max(dmu):.2e}')
                C.check('한 번 추론 시간 (틱 없이)', True, f'{min(us)}–{max(us)} µs')

        # ---- 3. RL 끝-끝 (OUT 1: D 줄 10 ms — 모든 샘플이 있어야 ω 필터를 다시 계산할 수 있음)
        C.check('OUT 1 → out_ms=10', 'out_ms=10' in (b.cmd('OUT 1', ['CFG', 'ERR']) or ''))
        print(f'[3] RL 끝-끝 ({args.rl_s:g} s, 벤치, D 줄 10 ms)')
        rows, s_lines, evts = run_rl(b, args.rl_s)
        compare_rl(C, qp, rows, evts, max([int(l.split(',')[11]) for l in s_lines] or [0]))
        timing(C, s_lines, '(OUT 1, 참고)', strict=False)
        # ---- 4. 시연 설정(OUT 2)에서 누락·시간
        C.check('OUT 2 → out_ms=20', 'out_ms=20' in (b.cmd('OUT 2', ['CFG', 'ERR']) or ''))
        print(f'[4] RL 시간 여유 ({args.rl_s:g} s, 벤치, 시연 설정 D 줄 20 ms)')
        rows, s_lines, evts = run_rl(b, args.rl_s)
        C.check('RL 구간 D 줄 20 ms 간격', len(rows) > 10 and max(np.diff([r[0] for r in rows if r[5] == 3])) <= 21)
        timing(C, s_lines, '(OUT 2)', strict=True)
    finally:
        try:
            b.send('STOP')
            time.sleep(0.2)
            b.send('BENCH 0')
            time.sleep(0.2)
            b.s.close()
        except Exception:
            pass
    print(f'\n결과: 통과 {C.ok}, 실패 {len(C.fail)}' + (f' — {", ".join(C.fail)}' if C.fail else ''))
    return 1 if C.fail else 0


if __name__ == '__main__':
    sys.exit(main())
