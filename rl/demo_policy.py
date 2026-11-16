"""시연용 펌웨어(firmware/aeropendulum_demo)에 넣을 고정 RL 정책 만들기 — 정수 양자화 + 보드 계산 재현.

    .venv\\Scripts\\python.exe -m rl.demo_policy                       (rl8 best → policy.h, 오차 보고)
    .venv\\Scripts\\python.exe -m rl.demo_policy --vectors 20          (보드 `NN` 시험용 입력·기대값 출력)

정책: SAC 액터 8 → 64 → 64 → (μ, log σ). 시연에서는 결정적 행동 a = tanh(μ) 만 쓰므로 μ 한 줄만 넣습니다.
Uno 는 float 곱셈이 느려서(약 8 µs) 4 672 번의 곱을 20 ms 안에 할 수 없으므로 정수로 계산합니다.

    입력   x_q[i]  = round(o[i]·2^11), ±32767 로 자름                              (int16, 해상도 0.0005)
    1층    h1[j]   = clamp((Σ W1q[j,i]·x_q[i] + B1[j]) >> S1[j], 0, 32767)          (W1q 14비트, 행마다 시프트)
    2층    h2[j]   = clamp((Σ W2q[j,k]·h1[k] + B2[j]) >> S2[j], 0, 32767)           (W2q int8, 행마다 시프트)
    3층    μ       = (Σ W3q[j]·h2[j])·M3 + C3                                       (W3q 10비트, float 한 번)

- ReLU 는 양의 배율에 대해 동차라서, 1층 뉴런 j 를 c_j 배 하고 2층 열 j 를 1/c_j 배 해도 망은 같습니다.
  c_j 를 골라 리플레이 관측에서 h1 의 최댓값이 모두 H1_TARGET 이 되게 맞춥니다(활성값 해상도를 뉴런마다 고르게).
  2층 행 스케일(int8)과 시프트는 3층 열로 옮깁니다. 그래서 뉴런마다 곱셈 없이 시프트 한 번으로 재양자화합니다.
- 2층 int8 반올림의 평균 오차는 리플레이 관측으로 편향 보정(B2 에 더함)합니다.
- 오버플로 없음: |x_q| ≤ 32767, |W1q| ≤ 8191, 8 항 → < 2^31. h1 ≤ 32767, |W2q| ≤ 127, 64 항 → 2.7e8.
  h2 ≤ 32767, |W3q| ≤ 1023, 64 항 → 2.15e9 < 2^31.
- AVR 의 `>>` 는 산술 시프트(내림). 반올림용 2^(S−1) 은 B 에 미리 더해 둡니다.
- 보드 속도: 1층은 16×16 비트 어셈블리 곱셈(AVR201 방식), 2층은 ReLU 로 0 이 된 h1 을 건너뛰고(리플레이에서 0 이 아닌 것
  최대 39/64, 평균 29) 두 행을 한 번에 계산합니다. 그래서 W2 는 행 두 개씩 묶어 [쌍][k][2] 순서로 저장합니다(NN_W2P).

`QuantPolicy.mu_int(x_q)` 는 보드와 같은 정수 계산(int64 로 계산해도 범위 안이라 int32 와 같음)과 마지막 float32 한 번을
재현합니다. 보드 `NN` 명령의 출력과 비교해 펌웨어 구현을 확인합니다(rl/demo_test.py).
"""
from __future__ import annotations

import argparse
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

DEFAULT_SNAPSHOT = os.path.join('data', '20261115_rl8', 'checkpoints', 'best')
DEFAULT_BUFFER = os.path.join('data', '20261115_rl8', 'checkpoint', 'buffer.npz')
DEFAULT_HEADER = os.path.join('firmware', 'aeropendulum_demo', 'policy.h')
DEFAULT_JS = os.path.join('lab', 'demo_policy.js')        # 시연 UI 시뮬레이터용 (같은 정수 망)


def load_actor(path: str):
    """actor.pt → float64 (W1, b1, W2, b2, w3, b3) — μ 한 줄만."""
    import torch
    p = os.path.join(path, 'actor.pt') if os.path.isdir(path) else path
    sd = torch.load(p, weights_only=False, map_location='cpu')['actor']
    g = lambda k: sd[k].detach().numpy().astype(np.float64)
    return g('l1.weight'), g('l1.bias'), g('l2.weight'), g('l2.bias'), g('l3.weight')[0], float(g('l3.bias')[0])


def float_mu(net, O):
    W1, b1, W2, b2, w3, b3 = net
    h1 = np.maximum(O @ W1.T + b1, 0.0)
    h2 = np.maximum(h1 @ W2.T + b2, 0.0)
    return h2 @ w3 + b3, h1, h2


def quant_x(O):
    return np.clip(np.round(np.asarray(O, np.float64) * (1 << XS_BITS)), -X_MAX, X_MAX).astype(np.int64)


class QuantPolicy:
    """정수 양자화된 μ 망 (보드와 같은 계산)."""

    def __init__(self, net, O):
        W1, b1, W2, b2, w3, b3 = net
        _, h1, h2 = float_mu(net, O)
        # ---- 1층: 뉴런 배율 c_j (h1 최댓값 → H1_TARGET). 죽은 뉴런은 망 전체 최댓값 기준
        m1 = h1.max(0)
        c = H1_TARGET / np.where(m1 > 1e-3 * m1.max(), m1, m1.max())
        U = c[:, None] * W1 / (1 << XS_BITS)          # 정수 입력 1 당 h1 정수 증가량
        S1 = np.zeros(64, np.int64)
        for j in range(64):
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
        S2 = np.zeros(64, np.int64)
        for j in range(64):
            if m2[j] <= 0:
                S2[j] = 0
                continue
            # h2q_max = m2/(s2·2^S2) ≤ H2_TARGET 가 되는 가장 작은 S2
            S2[j] = max(0, int(math.ceil(math.log2(m2[j] / (s2[j] * H2_TARGET)))))
        self.S2 = S2
        self.B2 = np.round(b2c / s2).astype(np.int64) + np.where(S2 > 0, 1 << np.maximum(S2 - 1, 0), 0)
        # ---- 3층: w3' = w3 · s2 · 2^S2, 전체 한 스케일 10비트
        w3p = w3 * s2 * 2.0 ** S2
        self.M3 = float(np.abs(w3p).max() / W3_MAX)
        self.W3q = np.round(w3p / self.M3).astype(np.int64)
        self.C3 = float(b3)
        # 보드는 float32
        self.M3 = float(np.float32(self.M3))
        self.C3 = float(np.float32(self.C3))

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
                    h1_max=int(h1.max()), h2_max=int(h2.max()), h1_clip=float((h1 >= H_MAX).mean()),
                    h2_clip=float((h2 >= H_MAX).mean()))

    # ---------------------------------------------------------------- C 헤더
    def header(self, src: str, stats: str) -> str:
        def arr(name, ctype, v, per=16):
            v = [int(x) for x in np.asarray(v).reshape(-1)]
            rows = [', '.join(str(x) for x in v[i:i + per]) for i in range(0, len(v), per)]
            return f'const {ctype} {name}[{len(v)}] PROGMEM = {{\n  ' + ',\n  '.join(rows) + '\n};\n'
        out = [
            '// 자동 생성: python -m rl.demo_policy  (손으로 고치지 말 것)\n',
            f'// 정책: {src}\n',
            '// 결정적 행동 a = tanh(mu), 관측 8개 (rl/env.py 와 같은 순서·정규화). 계산 방식은 rl/demo_policy.py 머리말.\n',
            ''.join(f'// {s}\n' for s in stats.splitlines()),
            '#pragma once\n#include <avr/pgmspace.h>\n\n',
            f'#define NN_IN 8\n#define NN_H 64\n#define NN_XS_BITS {XS_BITS}\n',
            f'const float NN_M3 = {self.M3!r}f;\nconst float NN_C3 = {self.C3!r}f;\n\n',
            arr('NN_W1', 'int16_t', self.W1q, 8),
            arr('NN_B1', 'int32_t', self.B1, 8),
            arr('NN_S1', 'uint8_t', self.S1),
            arr('NN_W2P', 'int8_t', self.W2q.reshape(32, 2, 64).transpose(0, 2, 1), 16),
            arr('NN_B2', 'int32_t', self.B2, 8),
            arr('NN_S2', 'uint8_t', self.S2),
            arr('NN_W3', 'int16_t', self.W3q, 16),
        ]
        return ''.join(out)

    def js(self, src: str) -> str:
        """시연 UI 시뮬레이터(lab/demo_sim.js)가 쓰는 같은 정수 망 (W2 는 행 우선 64×64)."""
        import json
        d = dict(src=src, xsBits=XS_BITS, M3=self.M3, C3=self.C3,
                 W1=self.W1q.reshape(-1).tolist(), B1=self.B1.tolist(), S1=self.S1.tolist(),
                 W2=self.W2q.reshape(-1).tolist(), B2=self.B2.tolist(), S2=self.S2.tolist(), W3=self.W3q.tolist())
        return ('// 자동 생성: python -m rl.demo_policy  (손으로 고치지 말 것) — firmware/aeropendulum_demo/policy.h 와 같은 망' + chr(10) +
                'window.DEMO_POLICY = ' + json.dumps(d, separators=(',', ':')) + ';' + chr(10))


def error_report(net, qp: QuantPolicy, O) -> str:
    mu, _, _ = float_mu(net, O)
    a = np.tanh(mu)
    aq = np.tanh(qp.mu(O))
    d = aq - a
    x, xq = np.sign(a) * a * a, np.sign(aq) * aq * aq   # 행동 곡선 (act_pow 2) → du/du_max
    ss = np.abs(O[:, 0]) < 0.1                          # |e| < 1°
    r = qp.check_ranges(O)
    lines = [
        f'리플레이 관측 {len(O)}개: |Δa| 최대 {np.abs(d).max():.4f}, RMS {np.sqrt((d ** 2).mean()):.5f}, 평균 {d.mean():+.5f}',
        f'|e| < 1° ({ss.mean() * 100:.0f}%): 평균 Δdu {10 * (xq - x)[ss].mean():+.4f} PWM/결정, RMS Δdu {10 * np.sqrt(((xq - x)[ss] ** 2).mean()):.4f} PWM',
        f'범위: acc1 {r["acc1"]:.3g}, acc2 {r["acc2"]:.3g}, acc3 {r["acc3"]:.3g} (< 2.1e9), h1 최대 {r["h1_max"]}, h2 최대 {r["h2_max"]}',
    ]
    return '\n'.join(lines)


def build(snapshot=DEFAULT_SNAPSHOT, buffer=DEFAULT_BUFFER):
    net = load_actor(snapshot)
    O = np.load(buffer)['o'].astype(np.float64)
    return net, QuantPolicy(net, O), O


def main(argv=None):
    ap = argparse.ArgumentParser(description='시연용 펌웨어 정책 헤더 만들기 (정수 양자화)')
    ap.add_argument('--snapshot', default=DEFAULT_SNAPSHOT, help='actor.pt 가 있는 스냅샷 폴더')
    ap.add_argument('--buffer', default=DEFAULT_BUFFER, help='활성값 범위·편향 보정용 리플레이 (buffer.npz)')
    ap.add_argument('--out', default=DEFAULT_HEADER)
    ap.add_argument('--js', default=DEFAULT_JS, help='시연 UI 시뮬레이터용 JS (빈 문자열이면 안 씀)')
    ap.add_argument('--vectors', type=int, default=0, help='보드 NN 시험용 무작위 리플레이 관측 N개 출력 (헤더는 안 씀)')
    args = ap.parse_args(argv)
    if not os.path.exists(args.buffer):
        raise SystemExit(f'리플레이 버퍼가 없습니다: {args.buffer} (체크포인트는 git 에 없음 — 학습한 PC 에서 실행)')
    net, qp, O = build(args.snapshot, args.buffer)
    if args.vectors:
        rng = np.random.default_rng(0)
        for o in O[rng.integers(0, len(O), args.vectors)]:
            xq = quant_x(o[None, :])[0]
            print(' '.join(str(int(v)) for v in xq), int(qp.acc3(xq)[0]), f'{float(qp.mu_int(xq)[0]):.6f}')
        return 0
    rep = error_report(net, qp, O)
    print(rep)
    src = os.path.relpath(os.path.abspath(args.snapshot)).replace('\\', '/')
    with open(args.out, 'w', encoding='utf-8', newline='\n') as f:
        f.write(qp.header(src, rep))
    print(f'헤더: {args.out}')
    if args.js:
        with open(args.js, 'w', encoding='utf-8', newline='\n') as f:
            f.write(qp.js(src))
        print(f'JS: {args.js}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
