"""설정 한 곳: 플랜트·PID·안전·RL·에피소드·휴식·SAC·정지 조건.

- 모든 숫자는 여기 dataclass 기본값에 있고, JSON 으로 저장/읽기 합니다(`load_config`, `save_config`).
- JSON 에는 바꾸고 싶은 항목만 적어도 됩니다(나머지는 기본값). 모르는 키는 오류로 알립니다.
- "재식별" 표시가 붙은 값은 플랜트를 다시 식별하면 바꿔 끼울 값입니다.
  현재 기본값 = 2026-11-13 기구 수정 후 모델 (docs/RESULTS.md §한눈에·§3, data/20261113_run3/analysis/REPORT.md §6).
"""
from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass, field
from typing import Any

# 11-13 계단 2회 평균 정적 특성 (PWM → θ [deg]). lab/sim.js 의 MAP 과 같은 값.   [재식별]
STATIC_MAP_1113 = [
    [0, 0.0], [30, 0.0], [50, 1.95], [60, 3.0], [70, 4.8], [80, 6.75], [90, 8.7], [100, 11.35], [110, 13.55],
    [120, 15.3], [130, 17.1], [140, 18.9], [150, 20.8], [160, 22.9], [170, 25.6], [180, 27.6], [190, 29.95],
    [200, 32.25], [210, 35.2], [220, 38.3], [230, 42.1], [240, 47.25], [250, 53.3], [255, 56.5],
]


@dataclass
class PlantCfg:
    """가짜 보드의 플랜트 (rl/plant.py).

    G(s) = [ K/(T s + 1) · ωr²/(s² + 2ζωr s + ωr²) + Ks/(Tc s + 1) ] · e^(−L s)   [deg/PWM]
    큰 신호는 정적 지도로: 빠른 경로 입력 = (K/K_nom)·f·map(u), 느린 경로 = (Ks/Ks_nom)·(1−f)·map(u),
    f = K_nom/(K_nom+Ks_nom). 그래서 운전점에 따라 국소 이득이 바뀝니다(20° 0.15 → 30° 0.24 경향).
    """
    static_map: list = field(default_factory=lambda: [list(p) for p in STATIC_MAP_1113])  # [재식별]
    K: float = 0.14          # 빠른 이득 [deg/PWM]                                    [재식별]
    Ks: float = 0.05         # 느린 경로 이득 (케이블 점탄성·피벗 크리프)             [재식별]
    K_nom: float = 0.14      # 정적 지도가 나타내는 공칭 K (지도를 바꾸면 같이)       [재식별]
    Ks_nom: float = 0.05     # 정적 지도가 나타내는 공칭 Ks                           [재식별]
    T: float = 0.319         # 1차 지연 시정수 [s]                                    [재식별]
    Tc: float = 6.0          # 느린 경로 시정수 [s]                                   [재식별]
    delay_ms: float = 21.0   # 입력 지연 [ms] (DLPF 5 기준)                           [재식별]
    fr_hz: float = 2.84      # 구조 공진 [Hz]                                         [재식별]
    zr: float = 0.136        # 공진 감쇠비                                            [재식별]
    offset_deg: float = 0.0  # 정적 지도 오프셋 [deg] (기록마다 ±3° 차이)
    rest_offset_deg: float = 3.0  # 모터 정지 시 정지 위치가 0° 로 안 돌아옴 (케이블 간섭): 멈출 때마다 ±이 값에서 뽑음
    friction_deg: float = 1.0  # 피벗 마찰 히스테리시스 폭 [deg] (백래시 모델: 정지 위치가 접근 방향에 따라 ±폭/2)
    noise_deg: float = 0.04    # θ 측정 잡음 σ [deg]
    omega_noise: float = 0.5   # ω 백색 잡음 σ [deg/s]
    vib_amp: float = 9.0       # 프로펠러 진동이 자이로에 실리는 진폭 [deg/s] (11-13: σ 6–7)
    vib_hz: float = 46.3
    acc_noise_deg: float = 4.0  # θacc 잡음 σ (모터 가동 중)
    stall_pwm: float = 30.0     # 이 아래면 모터 정지
    start_pwm: float = 45.0     # 정지 뒤 이 이상에서
    start_delay: float = 0.15   # 이 시간 지나야 다시 추력 [s]
    # 케이블 충격 외란 (모델에 없는 실측 현상: 15–25° 튐 + 2.7 Hz 울림). 0 = 끔
    shock_rate: float = 0.0          # 평균 발생 횟수 [1/s]
    shock_w: list = field(default_factory=lambda: [100.0, 300.0])   # 공진 모드 각속도 충격 [deg/s] (부호 무작위)
    # 도메인 랜덤화: True 면 RUN RL 마다 아래 범위에서 새로 뽑음 (가짜 보드 전용)
    randomize: bool = False
    dr: dict = field(default_factory=lambda: {
        'K': [0.11, 0.26, 'log'], 'Ks': [0.0, 0.08], 'Tc': [3.0, 10.0], 'T': [0.27, 0.38],
        'delay_ms': [15.0, 30.0], 'fr_hz': [2.5, 3.1], 'zr': [0.09, 0.18], 'offset_deg': [-3.0, 3.0],
        'shock_rate': [0.0, 0.2], 'noise_deg': [0.02, 0.08],
    })
    seed: int = 1


@dataclass
class PidCfg:
    """보드 PID (복귀·안전 복귀·기준선). 현재 사용자 손튜닝 값 (run3)."""
    kp: float = 0.9
    ki: float = 7.0
    kd: float = 0.02
    ff: int = 140
    umin: int = 40
    notch_cHz: int = 0        # 0 = 노치 끔
    notch_zz: int = 150
    notch_zp: int = 700


@dataclass
class LimCfg:
    pwm_max: int = 255        # PID 시험과 같은 상한 (11-14 rl1 은 210 이라 33° 에서 포화)
    angle_cdeg: int = 10000   # 각도 한계 100°
    rev: int = 0


@dataclass
class RLBoardCfg:
    """`RL <dec> <lag> <du_max> <u_lo> <u_hi>`"""
    dec: int = 2       # 결정 주기 [샘플] → 10 ms × 2 = 20 ms (50 Hz)
    lag: int = 2       # 적용 지연 [샘플]. 실측 왕복 7–9 ms → lag 1(10 ms 창)은 누락, lag 2(20 ms)는 누락 0 (11-14 벤치)
    # 보드 재부팅 기본값은 lag=1 이므로 학습기는 항상 RL 명령으로 명시 설정
    du_max: int = 10   # 한 번에 바꿀 수 있는 최대 PWM (최대 변화율 500 PWM/s)
    u_lo: int = 40     # 정책 PWM 범위 (모터 정지 방지 umin 과 같게)
    u_hi: int = 255    # = LIM 최대 PWM


@dataclass
class SafeCfg:
    """`SAFE <th_lo> <th_hi> <w_max>` — 에피소드 안전 범위(벗어나면 PID 복귀). 케이블 충격 튐(15–25°) 여유 포함."""
    th_lo_cdeg: int = -1000    # −10°
    th_hi_cdeg: int = 7000     # 70°
    w_max_cdps: int = 60000    # 600 °/s (충격 300 °/s + 진동)


@dataclass
class EpisodeCfg:
    dur_s: float = 20.0             # 에피소드 길이 (11-15: 10 → 20 s. 구간 7–13 s 라 느린 경로 Tc 6 s 까지 담기고
                                    # 샘플의 ~80% 가 정상상태 근처 → 보상이 과도 응답에 덜 치우침)
    ref_lo: float = 15.0            # 기준 범위 [deg]                                 [재식별]
    ref_hi: float = 30.0            # 11-14 rl1 (35°) 은 31° 이상에서 포화·정착 실패가 잦았음
    home_deg: float | None = None   # None = 에피소드마다 기준 범위에서 무작위
    step_frac: list = field(default_factory=lambda: [0.35, 0.65])  # 중간 기준 스텝 시각 (에피소드 비율)
    min_step_deg: float = 4.0       # 기준 변경 최소 크기 (home→r, r→r2)
    settle_tol_deg: float = 2.0     # 복귀 정착 판정 |θ−home|
    settle_hold_s: float = 1.0      # 이 시간 동안 정착 유지
    settle_timeout_s: float = 15.0  # 넘으면 그냥 시작(경고 기록)
    home_min_s: float = 2.0         # 복귀 최소 시간


@dataclass
class EnvCfg:
    """관측 정규화·보상 (rl/env.py)."""
    th_center: float = 25.0     # 각도 정규화 중심 [deg]
    th_scale: float = 10.0      # 각도·오차 스케일 [deg]
    w_scale: float = 100.0      # 각속도 스케일 [deg/s]
    w_lpf_hz: float = 10.0      # ω 저역통과 (진동 제거, PID 와 같은 10 Hz)
    u_center: float = 125.0     # PWM 정규화 중심
    u_scale: float = 85.0       # PWM 정규화 스케일
    err_scale: float = 2.0      # 보상 오차 스케일 [deg]: r_err = −min((e/err_scale)², err_cap)
    err_cap: float = 16.0
    w_abs: float = 1.0          # |e|/err_scale 항 (정상상태 정밀도용, 제곱만 쓰면 작은 오차의 기울기가 0)
    w_du: float = 0.05          # 행동(PWM 변화) 벌점: w_du·(du/du_max)²
    w_ddu: float = 0.1          # 행동 변화 벌점: w_ddu·((du−du_prev)/du_max)² (rl1 0.02 → 정착 구간 du ±3–4 떨림)
    w_fine: float = 2.0         # 정밀 항: w_fine·(1 − exp(−(e/fine_scale)²)) — 작은 오차(0.2–1°)에서 기울기가 큼
    fine_scale: float = 0.5     # [deg]
    fallback_penalty: float = 30.0   # FALLBACK 종단 벌점
    abort_penalty: float = 60.0      # ABORT 종단 벌점
    reward_scale: float = 0.03       # 전체 보상 배율 (Q 크기 조절)
    sigma_delta: bool = True         # 행동 → 정수 du 양자화에 시그마-델타 (반올림 불감대 제거)
    obs_eint: bool = True            # 관측에 오차의 느린 평균 (시정수 eint_tau_s) 추가 → 느린 경로 상태의 단서
    eint_tau_s: float = 3.0
    eint_scale: float = 2.0          # [deg]
    act_pow: float = 2.0            # 행동 곡선 du = du_max·sign(a)|a|^p (p > 1 이면 0 근처 해상도↑)
    obs_efine: bool = True           # 관측에 미세 오차 tanh(e/efine_scale) 추가 (e/th_scale 만으로는 0.5° 가 0.05)
    efine_scale: float = 0.5         # [deg]
    ki_assist: float = 6.0           # 고정 적분 보조 k_I [PWM/(°·s)] (보드 PID ki 와 같은 단위, 0 = 끔): du += k_I·dt·e.
    # 정책 출력 0 이 "유지"가 아니라 "오차 적분"이 되게 함 (rl3: 과열로 이득이 변하자 e ≈ 1.4° 에서 du ≈ 0 으로 멈춤).
    # 평형 편향 e* = −(정책 몫)/(k_I·dt): 11-15 rl1(k_I 3)은 정책 출력의 영점 오프셋 −0.06 PWM/결정 때문에 e* ≈ 1°,
    # 6 이면 같은 오프셋에 ≈ 0.5°. 크게 할수록 과도 구간에 쌓여 오버슈트↑ (가짜 보드 공칭, 20 s, 2 시드 60 ep:
    # k_I 3 / 6 / 10 → 편향 0.31 / 0.25 / 0.21°, OS 7.9 / 5.9 / 13.9%, 정착 4.1 / 3.5 / 4.0 s; PID 0.19° · 5.9% · 2.3 s)
    ki_band_deg: float = 0.0         # 적분 분리 폭 [deg]: x_I 에 exp(−(e/폭)²) 를 곱해 기준 근처에서만 적분 (0 = 끔).
    # 속도형이라도 과도 구간 동안 쌓이는 양은 큼: k_I 10, 10° 스텝 상승 ~1 s (∫e ≈ 3 °·s) → 30 PWM ≈ 4° 오버슈트.
    # 가짜 보드에서 k_I 10 + 폭 1.5/3° 는 OS 는 10% 로 줄었지만 편향 0.29–0.31° 로 이점이 사라져 기본은 끔


@dataclass
class RestCfg:
    """모터 과열 방지 휴식: 모터 가동 누적이 max_on_s 를 넘으면 STOP 하고 rest_s 쉼."""
    max_on_s: float = 300.0
    rest_s: float = 90.0


@dataclass
class SacCfg:
    hidden: int = 64            # 관측 7차원·행동 1차원이라 작은 망으로 충분, 갱신 1회 ≈ 5 ms (CPU)
    batch: int = 256
    lr: float = 3e-4
    gamma: float = 0.99         # 결정 20 ms → 유효 지평 약 2 s (느린 경로·마찰 때문에 작은 보정의 효과가 늦게 나타남)
    tau: float = 0.005
    buffer_size: int = 500_000  # 50 Hz × 2.8 h
    warmup_steps: int = 2500    # 균일 무작위 행동 결정 수 (≈ 5 에피소드)
    utd: float = 1.0            # 결정 1개당 경사 갱신 수 (에피소드 사이에 몰아서)
    init_alpha: float = 0.2
    target_entropy: float = -1.0
    log_std_min: float = -6.0
    log_std_max: float = 1.0
    seed: int = 0


@dataclass
class MonitorCfg:
    """자동 일시정지 조건."""
    max_overrun_delta: int = 5     # S 줄 overruns(누적) 증가가 한 줄 사이에 이보다 크면
    sat_max_per_s: int = 50        # S 줄 sat 이 이보다 크면
    sat_consec: int = 3            # sat > 0 인 S 줄이 연속 이만큼이면
    max_miss_frac: float = 0.2     # 한 에피소드 miss 비율이 이보다 크면
    link_timeout_s: float = 2.0    # 이 시간 동안 아무 줄도 안 오면 연결 끊김으로 봄
    rest_check_deg: float = 5.0    # 휴식 끝 정지 확인: 마지막 1 s 평균 |θ| 가 이보다 크면 일시정지 (정지 위치는 0° 가 아닐 수 있어 너그럽게; ZERO 는 절대 자동으로 보내지 않음)
    hb_watchdog_s: float = 20.0   # 학습기 주 스레드가 이 시간 동안 멈추면 하트비트를 끊어 보드가 정지하게 함
    # 자세 추정 감시: 모터 가동 중 θ − θacc 의 att_gap_win_s 이동평균이 att_gap_max_deg 를 넘으면 자동 일시정지 (0 = 끔).
    # 11-15 rl3: 진동이 커져 자이로가 모터 가동 중에만 편향(진동 정류) → Madgwick β 0.004(각속도 보정 예산 ≈ 2β = 0.46 °/s, 수평 두 축 공유)를 넘어
    # θ 가 θacc 에서 2–4° → 25–29° 까지 떠내려감. 정상 실행(11-14 rl1·rl3, 11-15 rl1)의 10 s 평균 최대는 1.8–2.7°,
    # rl3 은 02:15 (간극이 커지기 시작한 때) 에 3° 를 넘음. 짧은 창(2 s)은 무작위 워밍업의 과도 지연으로 3.7° 까지 나와 오경보
    att_gap_max_deg: float = 3.0
    att_gap_warn_deg: float = 2.0  # 넘으면 경고 이벤트 (60 s 에 한 번)
    att_gap_win_s: float = 10.0


@dataclass
class StopCfg:
    max_hours: float = 3.0         # 총 실행 시간 (휴식 포함, 보드 시계)
    max_episodes: int = 3000       # 학습 에피소드 수
    max_fallbacks: int = 150       # FALLBACK 누적


@dataclass
class RunCfg:
    eval_every: int = 10           # 학습 에피소드 N 개마다 평가
    eval_refs: list = field(default_factory=lambda: [[20.0, 30.0, 22.0], [30.0, 18.0, 28.0], [25.0, 30.0, 16.0]])
    # 평가 시나리오 [home, r1, r2]: r1 은 시작부터, r2 는 에피소드 절반에서
    ss_win_s: float = 3.0          # 정상상태 지표 창: 각 기준 구간(스텝 전·에피소드 끝)의 마지막 이 시간
    settle_band_deg: float = 0.5   # 정착 시간 판정 띠 |e| (0.2 s 이동평균, 진동·잡음 제거)
    baseline_every: int = 5        # 정기 평가 N 회마다 PID 기준선 다시 (플랜트가 세션 중 흘러감). 0 = 시작 때만
    checkpoint_every: int = 10     # 에피소드마다
    snapshot_every: int = 50       # 학습 에피소드 N 개마다 정책 스냅샷 checkpoints/epNNNNN/ (0 = 끔). 옛 config.json 에 없으면 이 값
    baseline_at_start: bool = True
    save_episode_csv: bool = True
    keep_episode_series: int = 3   # status() 에 넣을 최근 에피소드 시계열 개수
    bench: bool = False            # True 면 BENCH 1 (드라이런: 모터 핀 LOW) 로 학습 루프만 시험


@dataclass
class Config:
    plant: PlantCfg = field(default_factory=PlantCfg)
    pid: PidCfg = field(default_factory=PidCfg)
    lim: LimCfg = field(default_factory=LimCfg)
    rl: RLBoardCfg = field(default_factory=RLBoardCfg)
    safe: SafeCfg = field(default_factory=SafeCfg)
    episode: EpisodeCfg = field(default_factory=EpisodeCfg)
    env: EnvCfg = field(default_factory=EnvCfg)
    rest: RestCfg = field(default_factory=RestCfg)
    sac: SacCfg = field(default_factory=SacCfg)
    monitor: MonitorCfg = field(default_factory=MonitorCfg)
    stop: StopCfg = field(default_factory=StopCfg)
    run: RunCfg = field(default_factory=RunCfg)
    ts_s: float = 0.010            # 보드 샘플 주기 (STREAM 2). 실제 값은 CFG ts_ms 로 확인


def to_dict(cfg: Config) -> dict:
    return dataclasses.asdict(cfg)


def _merge(obj: Any, d: dict, path: str):
    names = {f.name: f for f in dataclasses.fields(obj)}
    for k, v in d.items():
        if k not in names:
            raise KeyError(f'알 수 없는 설정 키: {path}{k}')
        cur = getattr(obj, k)
        if dataclasses.is_dataclass(cur):
            if not isinstance(v, dict):
                raise TypeError(f'{path}{k} 는 객체여야 합니다')
            _merge(cur, v, f'{path}{k}.')
        else:
            setattr(obj, k, v)


# 실행 폴더의 config.json(전체 설정을 저장)에 이 키가 없으면 그 실행은 이 기능 전에 시작한 것 →
# 새 기본값 대신 옛 동작 값을 씀 (관측 차원·보상이 바뀌면 옛 체크포인트를 이어 하거나 정책을 시험할 수 없음)
LEGACY_DEFAULTS = {
    'env': {'obs_efine': False, 'w_fine': 0.0, 'ki_assist': 0.0},
    'run': {'baseline_every': 0},
}


def from_dict(d: dict, legacy: bool = False) -> Config:
    """legacy=True: 실행 폴더 config.json 읽기 — 없는 키 중 LEGACY_DEFAULTS 에 있는 것은 옛 동작 값."""
    cfg = Config()
    d = d or {}
    if legacy:
        for sec, kv in LEGACY_DEFAULTS.items():
            have = d.get(sec, {})
            for k, v in kv.items():
                if k not in have:
                    setattr(getattr(cfg, sec), k, v)
    _merge(cfg, d, '')
    return cfg


def load_config(path: str | None) -> Config:
    """--config 용 (바꿀 항목만 적은 JSON, 나머지는 현재 기본값)."""
    if not path:
        return Config()
    with open(path, encoding='utf-8') as f:
        return from_dict(json.load(f))


def load_run_config(path: str) -> Config:
    """실행 폴더의 config.json (--resume, rl.play). 옛 실행은 LEGACY_DEFAULTS 로 옛 동작 유지."""
    with open(path, encoding='utf-8') as f:
        return from_dict(json.load(f), legacy=True)


def save_config(cfg: Config, path: str):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(to_dict(cfg), f, ensure_ascii=False, indent=2)
