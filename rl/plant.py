"""가짜 보드용 플랜트 이산 시뮬레이션 (1 ms 내부 스텝, 상수는 전부 config.PlantCfg).

    u(PWM) ─지연 L─▶ 모터(저속 정지·재기동 지연) ─▶ 정적 지도 map(u) (+오프셋)
        ├─ 빠른 경로  f·(K/K_nom)·map  ─ 1/(T s+1) ─ 공진 ωr²/(s²+2ζωr s+ωr²) ─┐
        └─ 느린 경로 (1−f)·(Ks/Ks_nom)·map ─ 1/(Tc s+1) ─────────────────────┴─▶ 마찰(백래시) ─▶ θ
    측정: θ + 잡음, ω = dθ/dt + 잡음 + 프로펠러 진동, θacc = θ + 큰 잡음
    선택: 케이블 충격(공진 모드 각속도 충격, 포아송), 도메인 랜덤화(randomize()).

analysis/pid_design.py 의 모델을 참고했지만 import 하지 않고 자체 구현합니다.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
from scipy.linalg import expm

from .config import PlantCfg

DT = 0.001   # 내부 스텝 [s]


class Plant:
    def __init__(self, cfg: PlantCfg, rng: np.random.Generator | None = None):
        self.cfg = cfg
        self.rng = rng or np.random.default_rng(cfg.seed)
        self.p = {k: getattr(cfg, k) for k in ('K', 'Ks', 'T', 'Tc', 'delay_ms', 'fr_hz', 'zr', 'offset_deg',
                                                'shock_rate', 'noise_deg')}
        m = np.asarray(cfg.static_map, float)
        self.map_u, self.map_y = m[:, 0], m[:, 1]
        self.x1 = 0.0      # 빠른 경로 1차 지연 출력
        self.xs = 0.0      # 느린 경로
        self.y = 0.0       # 공진 출력
        self.yd = 0.0      # 공진 출력 미분
        self.out = 0.0     # 마찰 후 진자 각 (참값)
        self.omega = 0.0   # 참 각속도
        self.motor_on = False
        self.start_t = 0.0
        self.t = 0.0
        self._ma, self._mv = -1.0, 0.0   # 정적 지도 캐시
        self.rest_off = 0.0              # 현재 정지 위치 오프셋
        self._prepare()
        self.buf = deque([0.0] * self.nd, maxlen=self.nd) if self.nd else None

    # ---- 계수
    def _prepare(self):
        p, c = self.p, self.cfg
        self.a1 = 1.0 - math.exp(-DT / p['T'])
        self.as_ = 1.0 - math.exp(-DT / p['Tc'])
        wr = 2 * math.pi * p['fr_hz']
        A = np.array([[0.0, 1.0, 0.0], [-wr * wr, -2 * p['zr'] * wr, wr * wr], [0.0, 0.0, 0.0]])
        E = expm(A * DT)
        self.phi = tuple(float(x) for x in E[:2, :2].ravel())   # 파이썬 float 로 (스칼라 연산이 빠름)
        self.gam = (float(E[0, 2]), float(E[1, 2]))
        f = c.K_nom / (c.K_nom + c.Ks_nom)
        self.gf = f * p['K'] / c.K_nom
        self.gs = (1 - f) * (p['Ks'] / c.Ks_nom if c.Ks_nom > 0 else 0.0)
        self.nd = max(0, int(round(p['delay_ms'])))

    def thetass(self, u: float) -> float:
        """정적 지도 (오프셋 제외, 공칭 이득)."""
        return float(np.interp(min(255.0, abs(u)), self.map_u, self.map_y))

    def randomize(self):
        """도메인 랜덤화: cfg.dr 범위에서 파라미터를 새로 뽑음 (상태는 유지 → 실제 이득 변화처럼 과도가 생김)."""
        for k, spec in self.cfg.dr.items():
            lo, hi = float(spec[0]), float(spec[1])
            if len(spec) > 2 and spec[2] == 'log' and lo > 0:
                v = math.exp(self.rng.uniform(math.log(lo), math.log(hi)))
            else:
                v = self.rng.uniform(lo, hi)
            self.p[k] = v
        old_nd = self.nd
        self._prepare()
        if self.nd != old_nd:
            last = self.buf[-1] if self.buf else 0.0
            self.buf = deque([last] * self.nd, maxlen=self.nd) if self.nd else None

    # ---- 시뮬레이션
    def step(self, u: float, n: int = 5):
        """같은 입력 u 로 1 ms × n 진행."""
        c, p = self.cfg, self.p
        out0 = self.out
        ph, g = self.phi, self.gam
        half_w = 0.5 * c.friction_deg
        shock_p = p['shock_rate'] * DT
        for _ in range(n):
            if self.buf is not None:
                ud = self.buf[0]
                self.buf.append(u)
            else:
                ud = u
            a = abs(ud)
            if self.motor_on:
                if a < c.stall_pwm:
                    self.motor_on = False
                    self.rest_off = self.rng.uniform(-c.rest_offset_deg, c.rest_offset_deg)
            elif a >= c.start_pwm:
                self.start_t += DT
                if self.start_t >= c.start_delay:
                    self.motor_on = True
                    self.start_t = 0.0
            else:
                self.start_t = 0.0
            if self.motor_on:
                if a != self._ma:
                    self._ma = a
                    self._mv = float(np.interp(min(a, 255.0), self.map_u, self.map_y))
                m = self._mv
                tf = self.gf * m + p['offset_deg']
                ts_ = self.gs * m
            else:
                tf = self.rest_off     # 정지 위치 (0° 가 아닐 수 있음)
                ts_ = 0.0
            self.x1 += self.a1 * (tf - self.x1)
            self.xs += self.as_ * (ts_ - self.xs)
            y, yd = self.y, self.yd
            self.y = ph[0] * y + ph[1] * yd + g[0] * self.x1
            self.yd = ph[2] * y + ph[3] * yd + g[1] * self.x1
            if shock_p > 0 and self.rng.random() < shock_p:
                w = self.rng.uniform(*c.shock_w)
                self.yd += w if self.rng.random() < 0.5 else -w
            raw = self.y + self.xs
            if raw > self.out + half_w:
                self.out = raw - half_w
            elif raw < self.out - half_w:
                self.out = raw + half_w
        self.t += n * DT
        self.omega = (self.out - out0) / (n * DT)

    def measure(self, running: bool):
        """(θ, ω, θacc) [deg, deg/s, deg] 측정값."""
        c, r = self.cfg, self.rng
        th = self.out + r.normal() * self.p['noise_deg']
        om = self.omega + r.normal() * c.omega_noise
        if running:
            om += c.vib_amp * math.sin(2 * math.pi * c.vib_hz * self.t)
        tha = self.out + r.normal() * (c.acc_noise_deg if running else 0.3)
        return th, om, tha
