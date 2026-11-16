"""시연용 펌웨어(firmware/aeropendulum_demo)에 넣을 고정 RL 정책 만들기 — 정수 양자화 + 보드 계산 재현.

    .venv\\Scripts\\python.exe -m rl.demo_policy                                  (기본 rl5 ep 81 → policy.h + JS, 오차 보고)
    .venv\\Scripts\\python.exe -m rl.demo_policy --run data/20261117_rl4 --snapshot best
    .venv\\Scripts\\python.exe -m rl.demo_policy --vectors 20                     (보드 `NN` 시험용 입력·기대값 출력)

대상은 **잔차 RL** 실행입니다(11-16 모터 교체 뒤, `rl.res_max` > 0, RL 펌웨어 1.2 와 같은 구조): 보드가 매 샘플
PI + 노치를 계산하고 정책 잔차 r = res·a|a| 를 노치 앞에 더합니다. 실행 폴더의 config.json 에서 잔차 크기·PWM 범위·
관측 정규화·학습 때 보드 PID 를 읽어 헤더 상수로 넣으므로, 펌웨어는 그 실행과 같은 조건으로 정책을 돌립니다.

정책: SAC 액터 8 → H → H → (μ, log σ) (H = 16, 11-15 rl8 은 64). 결정적 행동 a = tanh(μ) 만 쓰므로 μ 한 줄만 넣습니다.
Uno 는 float 곱셈이 느려서(약 8 µs) 정수로 계산합니다.

    입력   x_q[i]  = round(o[i]·2^11), ±32767 로 자름                              (int16, 해상도 0.0005)
    1층    h1[j]   = clamp((Σ W1q[j,i]·x_q[i] + B1[j]) >> S1[j], 0, 32767)          (W1q 14비트, 행마다 시프트)
    2층    h2[j]   = clamp((Σ W2q[j,k]·h1[k] + B2[j]) >> S2[j], 0, 32767)           (W2q int8, 행마다 시프트)
    3층    μ       = (Σ W3q[j]·h2[j])·M3 + C3                                       (W3q 10비트, float 한 번)

- ReLU 는 양의 배율에 대해 동차라서, 1층 뉴런 j 를 c_j 배 하고 2층 열 j 를 1/c_j 배 해도 망은 같습니다.
  c_j 를 골라 리플레이 관측에서 h1 의 최댓값이 모두 H1_TARGET 이 되게 맞춥니다(활성값 해상도를 뉴런마다 고르게).
  2층 행 스케일(int8)과 시프트는 3층 열로 옮깁니다. 그래서 뉴런마다 곱셈 없이 시프트 한 번으로 재양자화합니다.
- 2층 int8 반올림의 평균 오차는 리플레이 관측으로 편향 보정(B2 에 더함)합니다.
- 오버플로 없음 (H ≤ 64): |x_q| ≤ 32767, |W1q| ≤ 8191, 8 항 → < 2^31. h1 ≤ 32767, |W2q| ≤ 127, 64 항 → 2.7e8.
  h2 ≤ 32767, |W3q| ≤ 1023, 64 항 → 2.15e9 < 2^31.
- AVR 의 `>>` 는 산술 시프트(내림). 반올림용 2^(S−1) 은 B 에 미리 더해 둡니다.
- 보드 속도: 1층은 16×16 비트 어셈블리 곱셈(AVR201 방식), 2층은 ReLU 로 0 이 된 h1 을 건너뛰고 두 행을 한 번에
  계산합니다. 그래서 W2 는 행 두 개씩 묶어 [쌍][k][2] 순서로 저장합니다(NN_W2P).

`QuantPolicy.mu_int(x_q)` 는 보드와 같은 정수 계산(int64 로 계산해도 범위 안이라 int32 와 같음)과 마지막 float32 한 번을
재현합니다. 보드 `NN` 명령의 출력과 비교해 펌웨어 구현을 확인합니다(rl/demo_test.py).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

XS_BITS = 11                 # 입력 고정소수점 (o·2^11)
X_MAX = 32767
W1_MAX = 8191                # 14비트
W2_MAX = 127                 # int8
W3_MAX = 1023                # 10비트
H_MAX = 32767
H1_TARGET = 12000            # 리플레이 관측의 h1 최댓값 → 이 정수 (범위 밖 관측에 2.7배 여유)
H2_TARGET = 12000

DEFAULT_RUN = os.path.join('data', '20261117_rl5')        # 잔차 RL, 펌웨어 1.2 (11-17)
DEFAULT_SNAPSHOT = 'manual_00081_041012'               # ep 81 = 확인 시험한 정책 (RL_HISTORY D13)
DEFAULT_HEADER = os.path.join('firmware', 'aeropendulum_demo', 'policy.h')
DEFAULT_JS = os.path.join('lab', 'demo_policy.js')        # 시연 UI 시뮬레이터용 (같은 정수 망 + 설정)


def load_actor(path: str):
    """actor.pt → float64 (W1, b1, W2, b2, w3, b3) — μ 한 줄만."""
    import torch
    p = os.path.join(path, 'actor.pt') if os.path.isdir(path) else path
    sd = torch.load(p, weights_only=False, map_location='cpu')['actor']
    g = lambda k: sd[k].detach().numpy().astype(np.float64)  # noqa: E731
    return g('l1.weight'), g('l1.bias'), g('l2.weight'), g('l2.bias'), g('l3.weight')[0], float(g('l3.bias')[0])


def float_mu(net, O):
    W1, b1, W2, b2, w3, b3 = net
    h1 = np.maximum(O @ W1.T + b1, 0.0)
    h2 = np.maximum(h1 @ W2.T + b2, 0.0)
    return h2 @ w3 + b3, h1, h2


def quant_x(O):
    return np.clip(np.round(np.asarray(O, np.float64) * (1 << XS_BITS)), -X_MAX, X_MAX).astype(np.int64)


def rl_constants(cfg) -> dict:
    """실행 설정 → 펌웨어가 쓰는 상수. 시연 펌웨어가 따르는 구조(잔차 모드, dec = lag = 2, 행동 곡선 a|a|, 관측 8개)와
    다르면 멈춥니다."""
    e, r = cfg.env, cfg.rl
    bad = []
    if not getattr(r, 'res_max', 0):
        bad.append('잔차 모드가 아님 (rl.res_max = 0) — 11-15 rl8 같은 증분 정책은 커밋 a35dfe4 의 시연 펌웨어를 쓰세요')
    if (r.dec, r.lag) != (2, 2):
        bad.append(f'dec/lag = {r.dec}/{r.lag} (2/2 만 지원)')
    if abs(cfg.ts_s - 0.01) > 1e-9:
        bad.append(f'Ts = {cfg.ts_s} s (0.01 만 지원)')
    if abs(e.act_pow - 2.0) > 1e-9:
        bad.append(f'act_pow = {e.act_pow} (2 만 지원)')
    if not (e.obs_eint and e.obs_efine):
        bad.append('관측 8개(obs_eint, obs_efine 켬)만 지원')
    if bad:
        raise SystemExit('시연 펌웨어와 맞지 않는 실행입니다:\n  ' + '\n  '.join(bad))
    p = cfg.pid
    return dict(
        res=int(r.res_max), u_lo=int(r.u_lo), u_hi=int(r.u_hi), umax=int(cfg.lim.pwm_max),
        th_c=e.th_center, th_s=e.th_scale, w_s=e.w_scale, u_c=e.u_center, u_s=e.u_scale,
        ei_s=e.eint_scale, ef_s=e.efine_scale,
        aw=1.0 - math.exp(-2 * math.pi * e.w_lpf_hz * cfg.ts_s), ai=1.0 - math.exp(-cfg.ts_s / max(1e-3, e.eint_tau_s)),
        safe_lo=int(cfg.safe.th_lo_cdeg), safe_hi=int(cfg.safe.th_hi_cdeg), safe_w=cfg.safe.w_max_cdps / 100.0,
        pid=dict(kp=p.kp, ki=p.ki, kd=p.kd, ff=int(p.ff), umin=int(p.umin), nfc=int(p.notch_cHz),
                 nzz=int(p.notch_zz), nzp=int(p.notch_zp)))


class QuantPolicy:
    """정수 양자화된 μ 망 (보드와 같은 계산)."""

    def __init__(self, net, O):
        W1, b1, W2, b2, w3, b3 = net
        H = W1.shape[0]
        assert H % 2 == 0 and H <= 64 and W1.shape[1] == 8, W1.shape
        self.H = H
        _, h1, h2 = float_mu(net, O)
        # ---- 1층: 뉴런 배율 c_j (h1 최댓값 → H1_TARGET). 죽은 뉴런은 망 전체 최댓값 기준
        m1 = h1.max(0)
        c = H1_TARGET / np.where(m1 > 1e-3 * m1.max(), m1, m1.max())
        U = c[:, None] * W1 / (1 << XS_BITS)          # 정수 입력 1 당 h1 정수 증가량
        S1 = np.zeros(H, np.int64)
        for j in range(H):
            mx = np.abs(U[j]).max()
            s = int(math.floor(math.log2(W1_MAX / mx))) if mx > 0 else 30
            S1[j] = max(0, min(30, s))
        self.W1q = np.round(U * (2.0 ** S1)[:, None]).astype(np.int64)
        assert np.abs(self.W1q).max() <= W1_MAX
        self.B1 = np.round(c * b1 * 2.0 ** S1).astype(np.int64) + np.where(S1 > 0, 1 << np.maximum(S1 - 1, 0), 0)
        self.S1 = S1
        self.c = c
        # ---- 2층: 열을 1/c 로, 행마다 int8
        V = W2 / c[None, :]
        s2 = np.abs(V).max(1) / W2_MAX
        self.W2q = np.round(V / s2[:, None]).astype(np.int64)
        assert np.abs(self.W2q).max() <= W2_MAX
        # 편향 보정: 정수 1층 출력으로 본 2층 사전 활성값의 평균 오차 (s2 단위)
        h1q = self._l1(quant_x(O))
        err = ((h1q @ self.W2q.T) * s2 - h1q @ V.T).mean(0)          # float 단위
        b2c = b2 - err
        # h2 정수 = relu(acc + b2c/s2) >> S2 ;  h2_real = h2q · s2 · 2^S2
        m2 = h2.max(0)
        S2 = np.zeros(H, np.int64)
        for j in range(H):
            if m2[j] <= 0:
                continue
            # h2q_max = m2/(s2·2^S2) ≤ H2_TARGET 가 되는 가장 작은 S2
            S2[j] = max(0, int(math.ceil(math.log2(m2[j] / (s2[j] * H2_TARGET)))))
        self.S2 = S2
        self.B2 = np.round(b2c / s2).astype(np.int64) + np.where(S2 > 0, 1 << np.maximum(S2 - 1, 0), 0)
        # ---- 3층: w3' = w3 · s2 · 2^S2, 전체 한 스케일 10비트
        w3p = w3 * s2 * 2.0 ** S2
        self.M3 = float(np.float32(np.abs(w3p).max() / W3_MAX))
        self.W3q = np.round(w3p / self.M3).astype(np.int64)
        self.C3 = float(np.float32(b3))               # 보드는 float32

    def _l1(self, xq):
        acc = xq @ self.W1q.T + self.B1
        return np.clip(acc >> self.S1, 0, H_MAX)

    def _l2(self, h1):
        acc = h1 @ self.W2q.T + self.B2
        return np.clip(acc >> self.S2, 0, H_MAX)

    def acc3(self, xq):
        xq = np.atleast_2d(np.asarray(xq, np.int64))
        return self._l2(self._l1(xq)) @ self.W3q

    def mu_int(self, xq):
        a = self.acc3(xq).astype(np.float32)
        return a * np.float32(self.M3) + np.float32(self.C3)

    def mu(self, O):
        return self.mu_int(quant_x(O)).astype(np.float64)

    def act(self, o):
        """학습기 정책처럼 쓰는 결정적 행동 (관측 1개)."""
        return float(np.tanh(self.mu(np.asarray(o, np.float64)[None, :])[0]))

    def check_ranges(self, O):
        xq = quant_x(O)
        a1 = xq @ self.W1q.T + self.B1
        h1 = self._l1(xq)
        a2 = h1 @ self.W2q.T + self.B2
        h2 = self._l2(h1)
        a3 = h2 @ self.W3q
        return dict(acc1=int(np.abs(a1).max()), acc2=int(np.abs(a2).max()), acc3=int(np.abs(a3).max()),
                    h1_max=int(h1.max()), h2_max=int(h2.max()), nz_max=int((h1 > 0).sum(1).max()))

    # ---------------------------------------------------------------- C 헤더
    def header(self, tag: str, stats: str, k: dict) -> str:
        H = self.H

        def arr(name, ctype, v, per=16):
            v = [int(x) for x in np.asarray(v).reshape(-1)]
            rows = [', '.join(str(x) for x in v[i:i + per]) for i in range(0, len(v), per)]
            return f'const {ctype} {name}[{len(v)}] PROGMEM = {{\n  ' + ',\n  '.join(rows) + '\n};\n'

        f = lambda x: f'{float(np.float32(x))!r}f'  # noqa: E731
        p = k['pid']
        out = [
            '// 자동 생성: python -m rl.demo_policy  (손으로 고치지 말 것)\n',
            f'// 정책: {tag}\n',
            '// 잔차 RL: 보드 PI + 노치에 r = RL_RES·a|a| 를 노치 앞에서 더함 (RL 펌웨어 1.2 와 같음), a = tanh(mu).\n',
            '// 관측 8개는 rl/env.py 와 같은 순서·정규화. 계산 방식은 rl/demo_policy.py 머리말.\n',
            ''.join(f'// {s}\n' for s in stats.splitlines()),
            '#pragma once\n#include <avr/pgmspace.h>\n\n',
            f'#define RL_TAG "{tag}"\n',
            f'#define NN_IN 8\n#define NN_H {H}\n#define NN_XS_BITS {XS_BITS}\n',
            f'const float NN_M3 = {f(self.M3)};\nconst float NN_C3 = {f(self.C3)};\n\n',
            '// ---- 실행 설정 (config.json) — 학습 때와 같은 조건으로 돌리기 위한 값\n',
            f'const int16_t RL_RES = {k["res"]};             // 잔차 최대 [PWM]\n',
            f'const int16_t RL_ULO = {k["u_lo"]}, RL_UHI = {k["u_hi"]};   // RL 중 출력 범위 [PWM]\n',
            f'const int16_t DEMO_UMAX = {k["umax"]};          // 기본 최대 PWM (LIM)\n',
            f'const int16_t SAFE_THLO = {k["safe_lo"]}, SAFE_THHI = {k["safe_hi"]};   // [cdeg] 벗어나면 PID 복귀\n',
            f'const float SAFE_WMAX = {f(k["safe_w"])};          // [deg/s]\n',
            f'const float RL_TH_C = {f(k["th_c"])}, RL_TH_S = {f(k["th_s"])}, RL_W_S = {f(k["w_s"])};\n',
            f'const float RL_U_C = {f(k["u_c"])}, RL_U_S = {f(k["u_s"])}, RL_EI_S = {f(k["ei_s"])}, RL_EF_S = {f(k["ef_s"])};\n',
            f'const float RL_AW = {f(k["aw"])};   // 1 − exp(−2π·f_ω·Ts)  ω 저역통과\n',
            f'const float RL_AI = {f(k["ai"])};   // 1 − exp(−Ts/τ)       오차 느린 평균\n',
            '// 학습 때 보드 PID (RL 중에는 사용자 이득 대신 이것을 씀)\n',
            f'const float RLPID_KP = {f(p["kp"])}, RLPID_KI = {f(p["ki"])}, RLPID_KD = {f(p["kd"])};\n',
            f'const int16_t RLPID_FF = {p["ff"]};\nconst uint8_t RLPID_UMIN = {p["umin"]};\n',
            f'const uint16_t RLPID_NFC = {p["nfc"]}, RLPID_NZZ = {p["nzz"]}, RLPID_NZP = {p["nzp"]};\n\n',
            arr('NN_W1', 'int16_t', self.W1q, 8),
            arr('NN_B1', 'int32_t', self.B1, 8),
            arr('NN_S1', 'uint8_t', self.S1),
            arr('NN_W2P', 'int8_t', self.W2q.reshape(H // 2, 2, H).transpose(0, 2, 1), 16),
            arr('NN_B2', 'int32_t', self.B2, 8),
            arr('NN_S2', 'uint8_t', self.S2),
            arr('NN_W3', 'int16_t', self.W3q, 16),
        ]
        return ''.join(out)

    def js(self, tag: str, k: dict) -> str:
        """시연 UI 시뮬레이터(lab/demo_sim.js)가 쓰는 같은 정수 망과 설정 (W2 는 행 우선 H×H)."""
        d = dict(src=tag, H=self.H, xsBits=XS_BITS, M3=self.M3, C3=self.C3,
                 W1=self.W1q.reshape(-1).tolist(), B1=self.B1.tolist(), S1=self.S1.tolist(),
                 W2=self.W2q.reshape(-1).tolist(), B2=self.B2.tolist(), S2=self.S2.tolist(), W3=self.W3q.tolist(),
                 cfg=k)
        return ('// 자동 생성: python -m rl.demo_policy  (손으로 고치지 말 것) — firmware/aeropendulum_demo/policy.h 와 같은 망'
                + chr(10) + 'window.DEMO_POLICY = ' + json.dumps(d, separators=(',', ':')) + ';' + chr(10))


def error_report(net, qp: QuantPolicy, O, res: int) -> str:
    mu, _, _ = float_mu(net, O)
    a = np.tanh(mu)
    aq = np.tanh(qp.mu(O))
    d = aq - a
    r, rq = res * a * np.abs(a), res * aq * np.abs(aq)     # 잔차 [PWM] (행동 곡선 a|a|)
    ss = np.abs(O[:, 0]) < 0.1                          # |e| < 1°
    g = qp.check_ranges(O)
    return '\n'.join([
        f'리플레이 관측 {len(O)}개: |Δa| 최대 {np.abs(d).max():.4f}, RMS {np.sqrt((d ** 2).mean()):.5f}, 평균 {d.mean():+.5f}',
        f'잔차 r = {res}·a|a|: |Δr| 최대 {np.abs(rq - r).max():.3f} PWM, RMS {np.sqrt(((rq - r) ** 2).mean()):.4f} PWM; '
        f'|e| < 1° ({ss.mean() * 100:.0f}%) 평균 Δr {(rq - r)[ss].mean():+.4f} PWM',
        f'범위: acc1 {g["acc1"]:.3g}, acc2 {g["acc2"]:.3g}, acc3 {g["acc3"]:.3g} (< 2.1e9), h1 최대 {g["h1_max"]}, '
        f'h2 최대 {g["h2_max"]}, 0 이 아닌 h1 최대 {g["nz_max"]}/{qp.H}',
    ])


def build(run=DEFAULT_RUN, snapshot=DEFAULT_SNAPSHOT, buffer=None):
    """(net, QuantPolicy, 리플레이 관측, 상수, 태그, cfg)."""
    from .config import load_run_config
    snap = os.path.join(run, 'checkpoints', snapshot)
    buf = buffer or os.path.join(run, 'checkpoint', 'buffer.npz')
    for pth in (os.path.join(snap, 'actor.pt'), buf, os.path.join(run, 'config.json')):
        if not os.path.exists(pth):
            raise SystemExit(f'없습니다: {pth} (체크포인트·버퍼는 git 에 없음 — 학습한 PC 에서 실행)')
    cfg = load_run_config(os.path.join(run, 'config.json'))
    k = rl_constants(cfg)
    net = load_actor(snap)
    O = np.load(buf)['o'].astype(np.float64)
    meta = {}
    if os.path.exists(os.path.join(snap, 'meta.json')):
        with open(os.path.join(snap, 'meta.json'), encoding='utf-8') as f:
            meta = json.load(f)
    tag = f'{os.path.basename(os.path.normpath(run))}/{snapshot}'
    if meta.get('episode') is not None:
        tag += f' ep{meta["episode"]}'
    return net, QuantPolicy(net, O), O, k, tag, cfg


def main(argv=None):
    ap = argparse.ArgumentParser(description='시연용 펌웨어 정책 헤더 만들기 (잔차 RL, 정수 양자화)')
    ap.add_argument('--run', default=DEFAULT_RUN, help='실행 폴더 (config.json, checkpoints/, checkpoint/buffer.npz)')
    ap.add_argument('--snapshot', default=DEFAULT_SNAPSHOT, help='checkpoints/ 아래 스냅샷 이름 (best, ep00050, …)')
    ap.add_argument('--buffer', help='활성값 범위·편향 보정용 리플레이 (기본 <run>/checkpoint/buffer.npz)')
    ap.add_argument('--out', default=DEFAULT_HEADER)
    ap.add_argument('--js', default=DEFAULT_JS, help='시연 UI 시뮬레이터용 JS (빈 문자열이면 안 씀)')
    ap.add_argument('--vectors', type=int, default=0, help='보드 NN 시험용 무작위 리플레이 관측 N개 출력 (헤더는 안 씀)')
    args = ap.parse_args(argv)
    net, qp, O, k, tag, _ = build(args.run, args.snapshot, args.buffer)
    if args.vectors:
        rng = np.random.default_rng(0)
        for o in O[rng.integers(0, len(O), args.vectors)]:
            xq = quant_x(o[None, :])[0]
            print(' '.join(str(int(v)) for v in xq), int(qp.acc3(xq)[0]), f'{float(qp.mu_int(xq)[0]):.6f}')
        return 0
    rep = error_report(net, qp, O, k['res'])
    print(f'정책: {tag} (망 8-{qp.H}-{qp.H}, 잔차 ±{k["res"]} PWM, RL 출력 {k["u_lo"]}–{k["u_hi"]}, '
          f'PID {k["pid"]["kp"]}/{k["pid"]["ki"]}/{k["pid"]["kd"]} ff {k["pid"]["ff"]} 노치 {k["pid"]["nfc"]})')
    print(rep)
    with open(args.out, 'w', encoding='utf-8', newline='\n') as f:
        f.write(qp.header(tag, rep, k))
    print(f'헤더: {args.out}')
    if args.js:
        with open(args.js, 'w', encoding='utf-8', newline='\n') as f:
            f.write(qp.js(tag, k))
        print(f'JS: {args.js}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
