"""`D` 줄 스트림 → 관측·행동·보상·전이 (에피소드 한 개).

시간 구조 (docs/RL_PROTOCOL.md): 결정 샘플 k (에피소드 샘플 번호 n 이 dec 의 배수)의 D 줄을 받으면
관측 o_k 로 행동 a_k 를 골라 `A du` 를 보내고, 보드는 n+lag 에서 u ← clamp(u + du) 를 적용합니다.
lag ≤ dec 이므로 다음 결정 샘플 D 줄의 pwm 에는 a_k 가 이미 반영돼 있습니다 → 실제 증분 Δu = u_{k+1} − u_k 를
보낸 du 와 비교해, 같으면 정책이 고른 연속 행동 a_k 를, 다르면(누락·범위 자르기) Δu 를 정책 공간으로 되돌린 값을
리플레이에 넣습니다.

행동 → 정수 증분: x = sign(a)|a|^p (p = act_pow), du = 시그마-델타 양자화((x + x_I)·du_max). 반올림만 하면
|x·du_max| < 0.5 가 불감대가 되어 느린 경로(Tc 6 s) 때문에 남는 작은 정상상태 오차를 못 없앱니다.

고정 적분 보조 (env.ki_assist = k_I [PWM/(°·s)], 보드 PID 의 ki 와 같은 단위):
    x_I = k_I · (dec·Ts) · e / du_max     →   u = u_0 + Σ du_max·x_policy + k_I·Σ e·dt   (속도형 적분)
- 정책 출력 0 의 의미가 "PWM 유지"에서 "오차 적분"으로 바뀝니다. 11-14 rl3 에서는 정책이 e ≈ 1.4° 에서 du ≈ 0 으로
  멈춰 있었고, 모터·드라이버 과열로 이득이 변하자 편향이 남았습니다. 정책은 이 적분 루프를 포함한 환경을 학습합니다
  (잔차 RL). 적분 상태는 u 에 들어 있어 관측에 이미 있으므로 마르코프성은 그대로입니다.
- 과도 구간(큰 e)에서는 정책 증분(±du_max)이 적분 증분보다 훨씬 커서 정책이 지배하고, 정상상태 근처에서는 행동 벌점
  때문에 정책 출력이 작아져 적분이 지배합니다. 엄격한 역할 분리는 아닙니다(정책이 −x_I 를 내면 상쇄 가능).
- 보드가 u 를 [u_lo, u_hi] 로 자르므로 적분 와인드업이 없습니다.
- 행동 벌점(w_du, w_ddu)과 리플레이 행동은 정책 몫만: 보낸 그대로 적용됐으면 정책이 고른 연속 증분, 누락·자르기면
  실제 증분 x_eff 에서 직전 적분 몫 x_I 를 뺀 값.
  적분 몫까지 벌하면 정책이 적분 움직임을 상쇄하도록 배웁니다. k_I = 0 이면 예전과 완전히 같습니다.

마르코프성: 지연(지연 21 ms + lag) 때문에 θ 만으로는 상태가 모자라서 관측에 현재 PWM 과 직전 행동,
걸러진 각속도, 직전 결정 이후 각 변화를 넣습니다.

    o = [ (r−θ)/s_θ, (θ−θc)/s_θ, ω_f/s_ω, Δθ/1°, (u−uc)/s_u, a_prev, ē/s_ē, tanh(e/s_f) ]
        ē = 오차의 3 s 느린 평균 (obs_eint), tanh(e/s_f) = 미세 오차 (obs_efine, s_f 0.5°: 첫 항만으로는 0.5° 가 0.05)

보상 (전이 k→k+1, 오차는 k+1 의 것):
    r = scale · ( −min((e/s_e)², cap) − w_abs·min(|e|/s_e, cap) − w_fine·(1 − exp(−(e/s_p)²))
                  − w_du·a² − w_ddu·(a − a_prev)² )
    정밀 항(w_fine, s_p 0.5°)은 0.2–1° 에서 기울기가 커서 작은 정상상태 오차를 과도 응답 벌점에 묻히지 않게 합니다
    (11-14 rl1: 0.5° 의 벌점이 스텝 직후 8° 의 1/60 이라 정상상태 개선 신호가 Q 오차에 묻힘).
FALLBACK/ABORT 는 종단(done=1, 벌점), 시간 만료(DONE)는 절단(마지막 전이를 버리고 부트스트랩 유지).

이 클래스의 on_D 는 link.fast_path 로 읽기 스레드에서 불립니다(가볍게 유지).
"""
from __future__ import annotations

import math

import numpy as np

from .config import Config

OBS_DIM = 6      # 기본 관측 차원 (obs_eint, obs_efine 마다 +1) — obs_dim(cfg) 를 쓰세요
ACT_DIM = 1


def obs_dim(cfg: Config) -> int:
    return OBS_DIM + (1 if cfg.env.obs_eint else 0) + (1 if cfg.env.obs_efine else 0)


def reward_from_obs(cfg: Config, o, a, o2):
    """리플레이 전이 (o, a, o2) 에서 보상을 다시 계산 (벡터, 종단 전이 제외 — 그건 벌점 상수).
    EpisodeEnv._reward 와 같은 식: 오차·각속도 = 다음 결정의 관측 o2[:,0]·th_scale, o2[:,2]·w_scale,
    정책 몫 증분 x = sign(a)|a|^p, 직전 증분 = 관측의 직전 행동 o[:,5] 를 같은 곡선으로.
    k_I > 0 실행에서는 보낸 그대로 적용된 결정의 벌점이 정확히 이 값이고, 누락·범위 자르기 결정만 근사입니다
    (k_I = 0 실행은 실제 정수 증분을 썼으므로 근사). rl.fork 의 보상 재라벨에 씀."""
    c = cfg.env
    p = c.act_pow
    o, a, o2 = np.asarray(o, np.float64), np.asarray(a, np.float64).reshape(-1), np.asarray(o2, np.float64)
    e = o2[:, 0] * c.th_scale
    wf = o2[:, 2] * c.w_scale
    x = np.sign(a) * np.abs(a) ** p
    xp = np.sign(o[:, 5]) * np.abs(o[:, 5]) ** p
    ax = np.abs(e) / c.err_scale
    r = -np.minimum(ax * ax, c.err_cap) - c.w_abs * np.minimum(ax, c.err_cap) - c.w_du * x * x - c.w_ddu * (x - xp) ** 2
    if c.w_fine:
        r -= c.w_fine * (1.0 - np.exp(-(e / c.fine_scale) ** 2))
    if c.w_omega:
        r -= c.w_omega * np.exp(-(e / c.omega_gate) ** 2) * np.minimum((wf / c.omega_scale) ** 2, c.omega_cap)
    return c.reward_scale * r


class EpisodeEnv:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.active = False
        self.policy = None
        self.n = 0
        self.reset_stats()

    def reset_stats(self):
        self.rows = []          # 기록용: (t_ms, pwm, th, om, tha, phase, ref, miss, du, reward)
        self.trans = []         # (o, a, r, o2, done)
        self.decisions = 0
        self.ret = 0.0
        self.se = 0.0           # 오차 제곱합 (모든 RL 샘플)
        self.cnt = 0
        self.pwm_sum = 0.0
        self.miss = 0
        self.du_abs = 0.0
        self.di_abs = 0.0       # 적분 보조 증분 |x_I|·du_max 합 [PWM]
        self.fast_errors = 0

    def arm(self, policy, dec: int, du_max: int):
        """policy(obs) -> a ∈ [−1, 1]. 다음 9필드 D 줄을 n = 0 으로 셉니다."""
        e = self.cfg.env
        self.policy = policy
        self.dec = dec
        self.du_max = du_max
        self.n = 0
        self.t0_ms = None
        self.lost_rows = 0      # 빠진 D 줄 수 (t_ms 로 추정)
        self.prev_o = None
        self.prev_a = 0.0       # 직전 결정의 실제 증분 중 정책 몫 (물리, du/du_max)
        self.prev_xi = 0.0      # 직전 결정의 적분 보조 몫 (정규화, 연속값)
        self.ki_x = e.ki_assist * dec * self.cfg.ts_s / du_max   # 오차 1° 당 결정 1회 적분 증분 (정규화)
        self.ki_band = e.ki_band_deg                             # 0 = 적분 분리 없음
        self.prev_apol = 0.0    # 직전 결정에서 정책이 고른 행동 (정책 공간 [−1, 1])
        self.prev_du = 0        # 직전 결정에서 보낸 정수 증분
        self.prev_u = None
        self.prev_th = None
        self.wf = None
        self.resid = 0.0
        self.ei = 0.0
        self.alpha_w = 1.0 - math.exp(-2 * math.pi * e.w_lpf_hz * self.cfg.ts_s)
        self.alpha_i = 1.0 - math.exp(-self.cfg.ts_s / max(1e-3, e.eint_tau_s))
        self.reset_stats()
        self.active = True

    def _reward(self, e_deg: float, a: float, a_prev: float, w_f: float = 0.0) -> float:
        c = self.cfg.env
        x = abs(e_deg) / c.err_scale
        r = -min(x * x, c.err_cap) - c.w_abs * min(x, c.err_cap) - c.w_du * a * a - c.w_ddu * (a - a_prev) ** 2
        if c.w_fine:
            r -= c.w_fine * (1.0 - math.exp(-(e_deg / c.fine_scale) ** 2))
        if c.w_omega:
            r -= c.w_omega * math.exp(-(e_deg / c.omega_gate) ** 2) * min((w_f / c.omega_scale) ** 2, c.omega_cap)
        return c.reward_scale * r

    def on_D(self, msg):
        """모든 D 줄에 대해 호출. 결정 샘플이면 보낼 'A du' 를 돌려주고 msg.info 에 결정 정보를 남김."""
        f = msg.f
        if not self.active or len(f) != 8 or f[5] != 1:
            return None
        c = self.cfg.env
        t_ms, u, th, om, tha, ph, ref, miss = f
        # D 줄이 깨지거나 빠져도 결정 위상이 어긋나지 않게, 줄 수가 아니라 t_ms 로 샘플 번호를 맞춤
        if self.t0_ms is None:
            self.t0_ms = t_ms
        n_t = int(round((t_ms - self.t0_ms) / (self.cfg.ts_s * 1000.0)))
        if n_t > self.n:
            self.lost_rows += n_t - self.n
            self.n = n_t
        n = self.n
        self.n += 1
        th *= 0.01
        om *= 0.01
        ref *= 0.01
        self.wf = om if self.wf is None else self.wf + self.alpha_w * (om - self.wf)
        e_deg = ref - th
        self.ei += self.alpha_i * (e_deg - self.ei)
        self.se += e_deg * e_deg
        self.cnt += 1
        self.pwm_sum += u
        self.miss = miss
        row = [t_ms, u, th, om, tha * 0.01, ph, ref, miss, None, None]
        self.rows.append(row)
        if n % self.dec:
            return None
        # ---- 결정 샘플
        p = c.act_pow
        if self.prev_u is None:
            x_pol, a_store = 0.0, 0.0
        else:
            d_eff = u - self.prev_u
            # 실제 적용된 증분에서 적분 보조 몫을 뺀 정책 몫 (물리, 정규화). k_I = 0 이면 실제 증분 그대로
            x_pol = max(-1.0, min(1.0, d_eff / self.du_max - self.prev_xi))
            if d_eff == self.prev_du:
                a_store = self.prev_apol                          # 보낸 그대로 적용됨 → 정책이 고른 연속값
                if self.ki_x:
                    # 정수 du − 연속 x_I 에는 양자화 잡음이 섞이므로 벌점은 정책이 고른 연속 증분으로
                    # (k_I = 0 이면 예전처럼 실제 정수 증분 그대로 → 옛 실행 보상 불변)
                    x_pol = math.copysign(abs(a_store) ** p, a_store)
            else:                                                 # 누락·범위 자르기 → 실제 정책 몫을 정책 공간으로
                a_store = math.copysign(abs(x_pol) ** (1.0 / p), x_pol)
        dth = 0.0 if self.prev_th is None else max(-5.0, min(5.0, th - self.prev_th))
        ov = [e_deg / c.th_scale, (th - c.th_center) / c.th_scale, self.wf / c.w_scale, dth,
              (u - c.u_center) / c.u_scale, a_store]
        if c.obs_eint:
            ov.append(self.ei / c.eint_scale)
        if c.obs_efine:
            ov.append(math.tanh(e_deg / c.efine_scale))
        o = np.array(ov, dtype=np.float32)
        info = {'n': n, 'o': o}
        if self.prev_o is not None:
            r = self._reward(e_deg, x_pol, self.prev_a, self.wf)
            self.trans.append((self.prev_o, a_store, r, o, 0.0))
            self.ret += r
            row[9] = r
            info['r'] = r
        a = max(-1.0, min(1.0, float(self.policy(o))))
        x_cmd = math.copysign(abs(a) ** p, a)      # 행동 곡선: p > 1 이면 0 근처를 잘게 (작은 정상상태 보정)
        x_i = self.ki_x * e_deg                    # 고정 적분 보조 (정책 몫과 더한 뒤 양자화)
        if self.ki_band:                           # 적분 분리: 기준 근처에서만 (과도 구간 누적 → 오버슈트 방지)
            x_i *= math.exp(-(e_deg / self.ki_band) ** 2)
        x_tot = x_cmd + x_i
        if c.sigma_delta:
            # 시그마-델타 양자화: 반올림 나머지를 다음 결정으로 넘겨 1 PWM 보다 작은 평균 증분도 실현
            # (반올림만 하면 |x| < 0.5/du_max 가 불감대가 되어 작은 정상상태 오차를 못 없앰)
            self.resid += x_tot * self.du_max
            du = max(-self.du_max, min(self.du_max, int(round(self.resid))))
            self.resid = max(-1.0, min(1.0, self.resid - du))
        else:
            du = max(-self.du_max, min(self.du_max, int(round(x_tot * self.du_max))))
        self.di_abs += abs(x_i) * self.du_max
        row[8] = du
        self.du_abs += abs(du)
        self.decisions += 1
        self.prev_o = o
        self.prev_a = x_pol
        self.prev_xi = x_i
        self.prev_apol = a
        self.prev_du = du
        self.prev_u = u
        self.prev_th = th
        info['du'] = du
        msg.info = info
        return f'A {du}'

    def terminal(self, kind: str):
        """FALLBACK / ABORT: 마지막 결정에서 종단 전이 (주 스레드에서 호출)."""
        self.active = False
        if self.prev_o is None:
            return None
        c = self.cfg.env
        pen = c.fallback_penalty if kind == 'FALLBACK' else c.abort_penalty
        r = -c.reward_scale * pen
        self.trans.append((self.prev_o, self.prev_apol, r, self.prev_o, 1.0))
        self.ret += r
        self.prev_o = None
        return r

    def finish(self):
        """DONE(절단) 또는 중단: 대기 중인 마지막 결정은 전이로 만들지 않음."""
        self.active = False

    def summary(self) -> dict:
        n = max(1, self.cnt)
        return dict(samples=self.cnt, decisions=self.decisions, ret=self.ret, rms=math.sqrt(self.se / n),
                    mean_pwm=self.pwm_sum / n, miss=self.miss,
                    miss_frac=self.miss / max(1, self.decisions), mean_abs_du=self.du_abs / max(1, self.decisions),
                    mean_abs_di=self.di_abs / max(1, self.decisions))
