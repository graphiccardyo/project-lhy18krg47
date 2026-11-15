"""학습 상태 기계 (스레드). 실기·가짜 보드(실시간/가상시간)에서 같은 코드로 돕니다.

흐름
    연결·설정(HELLO, STOP, BENCH, LIM, PID, NOTCH, RL, SAFE, GET)
    → [처음 한 번] PID 기준선: 평가 시나리오를 PID 만으로 (REF home → REF r1 → 중간에 REF r2)
    → 반복:  (모터 가동 누적 ≥ max_on_s 이면 휴식: STOP, rest_s 대기)
             (eval_every 마다 평가: 결정적 정책으로 평가 시나리오 전부)
             복귀(REF home, 정착 대기) → 학습 에피소드(RUN RL, 결정 샘플마다 A, 중간에 REF r2)
             → 경사 갱신(에피소드 사이, 보드는 PID 로 home 유지) → 주기적 체크포인트·스냅샷
    스냅샷 (정책만, <run_dir>/checkpoints/<이름>/actor.pt + meta.json; checkpoint/ 는 이어 하기용으로 따로 덮어씀)
        epNNNNN            학습 run.snapshot_every 에피소드마다 (0 = 끔)
        best               평가 라운드 평균 RMS 가 지금까지 스냅샷한 최고보다 작을 때 (덮어씀)
        manual_NNNNN_HHMMSS  command('snapshot')
    자동 일시정지(모터 STOP, 사람이 resume): EVT ABORT, 연결 끊김·포트 오류, S 줄 overruns 급증·sat 이상·IMU 오류,
        에피소드 miss 비율 초과, 예상치 못한 EVT STOP.
    자동 종료: 총 시간(max_hours), 학습 에피소드 수(max_episodes), FALLBACK 누적(max_fallbacks), command('stop').
    어떤 예외로 끝나더라도 STOP 을 보내고 체크포인트를 남깁니다.

외부(대시보드) 인터페이스 — 모두 스레드 안전
---------------------------------------------
    tr = Trainer(cfg, link_factory, run_dir, resume=False, autostart=False)
    tr.start()                       # threading.Thread. autostart=False 면 command('start') 를 기다림
    tr.command(cmd) -> (ok: bool, msg: str)
        'start'   대기 중이면 시작
        'pause'   즉시 모터 STOP 하고 일시정지 (진행 중 에피소드는 버림)
        'resume'  일시정지에서 재개 (포트 오류였으면 다시 연결·설정)
        'stop'    STOP·체크포인트 후 종료 (대기·일시정지 중에도 가능)
        'snapshot' 정책 스냅샷 manual_<에피소드 5자리>_<HHMMSS>. 제어 루프를 방해하지 않게 다음 에피소드 경계
                  (또는 휴식 중)에서 처리, 대기·일시정지 중이면 즉시
        'eval'    다음 에피소드 경계에서 평가 라운드를 한 번 (정기 평가와 같게 기록·갱신, 정기 일정은 그대로).
                  일시정지·대기 중에 받으면 재개·시작 뒤 첫 경계에서
    tr.status() -> dict (JSON 직렬화 가능, 매번 새 사본)
        state            'waiting' | 'connecting' | 'baseline' | 'homing' | 'episode' | 'eval' | 'updating'
                         | 'resting' | 'paused' | 'stopping' | 'done' | 'error'
        detail           사람이 읽을 현재 동작 설명 (str)
        paused_reason    일시정지 이유 (str | None)
        link             연결 이름 ('COM4', 'fake-realtime', 'fake-virtual'), fw: 펌웨어 식별 문자열
        run_dir          실행 폴더 경로
        episode          끝난 학습 에피소드 수 (int)
        eval_rounds      끝난 평가 회차 수
        decisions        누적 결정(전이) 수, updates: 누적 경사 갱신 수, buffer: 리플레이 크기
        warmup_left      남은 무작위 워밍업 결정 수
        current          진행 중 상황 {kind, home, r1, r2, n, total, ref, theta, pwm} (없으면 None)
        recent_episodes  최근 에피소드 시계열 목록 (최대 keep_episode_series + 최근 평가 1 + 기준선 1):
                         [{episode, kind('train'|'eval'|'baseline'), scenario, status, rms,
                           t: [s], theta: [deg], ref: [deg], pwm: [PWM]}]  (2샘플마다 1점)
        learning_curve   에피소드마다 [{episode, kind, status, ret, rms, miss_frac, mean_pwm, alpha, time_s}]
                         (time_s = 실행 시작 이후 보드 시계, 최근 2000개)
        eval_scores      [{episode, rms_mean, per_scenario: [rms...], ss_mean, ss_per_scenario: [|e|...],
                           bias_mean, ripple_mean, settle_mean, os_mean, bias_per_scenario, settle_per_scenario,
                           time_s, manual(수동 평가면 True)}]   (steady_metrics: ss = 평균 |e|, bias = |창 평균 e|,
                           ripple = 창 안 σ(e), settle = ±band 정착 시간 [s], os = 오버슈트 [%])
        snapshots        [{name, kind('periodic'|'best'|'manual'), episode, updates, eval_rms, recent_eval_rms, time}]
                         (에피소드·시각 순. eval_rms = 바로 그 정책의 평가 RMS(모르면 None),
                          recent_eval_rms = 저장 시점의 최근 평가 RMS, time = 'YYYY-MM-DDTHH:MM:SS')
        best_rms         best 스냅샷의 평가 RMS | None
        pending          처리 대기 중인 명령 ['snapshot', 'eval']
        baseline         {rms_mean, per_scenario, ss_mean, ss_per_scenario, bias_mean, ripple_mean, settle_mean,
                          os_mean, bias_per_scenario, settle_per_scenario, episode, time_s} | None
                         (가장 최근 PID 기준선, 같은 평가 시나리오. run.baseline_every 평가마다 다시)
        baseline_history [baseline...] (기준선을 돌릴 때마다)
        counts           {fallbacks, aborts, misses, pauses, train_episodes, eval_episodes}
        mean_pwm         직전 에피소드 평균 PWM
        motor_on_s       마지막 휴식 이후 모터 가동 누적 [s], rest_left_s: 휴식 중 남은 시간 [s] | None
        elapsed_s        실행 시간 [s] (이어 하기 포함), remaining_s: max_hours 까지 남은 시간
        episodes_left    max_episodes 까지 남은 학습 에피소드 수
        sac              {alpha, q_loss, pi_loss, entropy, q_mean}
        last_s           최근 S 줄 {state, maxTick_us, overruns, gate_pct, sat}
        att_gap_deg      모터 가동 중 θ − θacc 의 monitor.att_gap_win_s 이동평균 [deg] | None (+ att_gap_max/warn_deg)
        last_error       마지막 오류 메시지 | None
        log              최근 이벤트 문자열 50개
"""
from __future__ import annotations

import json
import math
import os
import queue
import threading
import time
import traceback
from collections import deque

import numpy as np

from .config import Config, save_config, to_dict
from .env import ACT_DIM, EpisodeEnv, obs_dim
from .link import LinkError
from .record import Recorder


class Pause(Exception):
    pass


class StopRun(Exception):
    pass


SNAP_DIR = 'checkpoints'
SNAP_META = 'meta.json'


def steady_metrics(rows, step_n: int, win_n: int, band: float = 0.5, ts: float = 0.01, smooth_s: float = 0.2) -> dict:
    """구간 지표. rows: (t_ms, pwm, th, om, tha, phase, ref, ...), 기준 스텝이 샘플 step_n 에서 (구간 1 = [0, step_n),
    구간 2 = [step_n, 끝)). 정상상태 창 = 각 구간의 마지막 win_n 샘플 (일찍 끝났으면 있는 것만).

    ss_e1/ss_e2: 창 평균 e = ref − θ (부호 있음)
    ss_abs:      두 창의 평균 |e| (= 편향 + 리플이 섞인 값, 11-14 부터 쓰던 지표)
    ss_bias:     창 평균 e 의 크기 |ē_w| 의 평균 — 오래 남는 오프셋 (적분·정책 영점 문제)
    ss_ripple:   창 안 e 의 표준편차 (창마다 평균을 뺀 합동) — 공진·케이블 울림·잡음
    ss_pwm_sd:   창 안 PWM 표준편차 (떨림)
    settle1/2:   기준 변경 뒤 |e| 의 smooth_s 이동평균이 band 안에 끝까지 머물기 시작한 시각 [s].
                 끝까지 못 들어오면 그 구간 길이 (검열값, 보수적)
    os1/os2:     1° 이상 기준 변경마다 새 기준을 넘어간 최대량 / 스텝 크기 [%] (구간 1 의 시작 = 첫 샘플 θ)
    settle_s:    settle1·2 평균, os_pct: os1·2 최대"""
    keys = ('ss_e1', 'ss_e2', 'ss_abs', 'ss_bias', 'ss_ripple', 'ss_pwm_sd', 'settle1', 'settle2', 'os1', 'os2',
            'settle_s', 'os_pct')
    out = dict.fromkeys(keys)
    if not rows or win_n <= 0:
        return out
    n = len(rows)
    segs = [(1, 0, min(step_n, n))]
    if n > step_n:
        segs.append((2, step_n, n))
    th = np.array([x[2] for x in rows], float)
    ref = np.array([x[6] for x in rows], float)
    u = np.array([x[1] for x in rows], float)
    e = ref - th
    k = max(1, int(round(smooth_s / ts)))
    ae, cnt, ess, uss, biases, settles, oss = 0.0, 0, 0.0, 0.0, [], [], []
    prev_ref = th[0]
    for idx, a, b in segs:
        if b <= a:
            continue
        r = ref[a]
        # 정상상태 창 (구간 2 는 반 창 이상 있어야)
        if idx == 1 or b - a >= win_n // 2:
            w = slice(max(a, b - win_n), b)
            ew, uw = e[w], u[w]
            m = float(ew.mean())
            out[f'ss_e{idx}'] = round(m, 4)
            biases.append(abs(m))
            ae += float(np.abs(ew).sum())
            ess += float(((ew - m) ** 2).sum())
            uss += float(((uw - uw.mean()) ** 2).sum())
            cnt += len(ew)
        # 정착 시간·오버슈트
        step = r - prev_ref
        if abs(step) >= 1.0:
            seg = e[a:b]          # 구간 안에서만 뒤쪽(인과) 이동평균 — 구간 경계를 넘어 섞지 않음
            cs = np.cumsum(np.insert(seg, 0, 0.0))
            idx_ = np.arange(1, len(seg) + 1)
            lo = np.maximum(0, idx_ - k)
            es = (cs[idx_] - cs[lo]) / (idx_ - lo)
            out_band = np.nonzero(np.abs(es) > band)[0]
            if len(out_band) == 0:
                st = 0.0
            elif out_band[-1] >= b - a - 1:
                st = (b - a) * ts                      # 끝까지 못 들어옴 → 구간 길이
            else:
                st = float(out_band[-1] + 1) * ts
            ov = max(0.0, float(np.max((th[a:b] - r) * math.copysign(1.0, step))))
            out[f'settle{idx}'] = round(float(st), 3)
            out[f'os{idx}'] = round(float(100.0 * ov / abs(step)), 2)
            settles.append(st)
            oss.append(out[f'os{idx}'])
        prev_ref = r
    if cnt:
        out['ss_abs'] = round(ae / cnt, 4)
        out['ss_bias'] = round(float(np.mean(biases)), 4)
        out['ss_ripple'] = round(math.sqrt(ess / cnt), 4)
        out['ss_pwm_sd'] = round(math.sqrt(uss / cnt), 3)
    if settles:
        out['settle_s'] = round(float(np.mean(settles)), 3)
        out['os_pct'] = round(float(np.max(oss)), 2)
    return out


def _mean_or_none(xs):
    v = [x for x in xs if x is not None]
    return round(float(np.mean(v)), 4) if v else None


def _fmt(v, d=3):
    return '–' if v is None else f'{v:.{d}f}'


def _seg_summary(sss: list) -> dict:
    """시나리오별 steady_metrics 목록 → 평가·기준선 점수 필드 (실패한 시나리오는 {} → None)."""
    g = lambda k: [s.get(k) for s in sss]
    return dict(ss_mean=_mean_or_none(g('ss_abs')), ss_per_scenario=g('ss_abs'),
                bias_mean=_mean_or_none(g('ss_bias')), bias_per_scenario=g('ss_bias'),
                ripple_mean=_mean_or_none(g('ss_ripple')),
                settle_mean=_mean_or_none(g('settle_s')), settle_per_scenario=g('settle_s'),
                os_mean=_mean_or_none(g('os_pct')))


def _seg_text(d: dict) -> str:
    return (f'편향 {_fmt(d.get("bias_mean"))}° 리플 {_fmt(d.get("ripple_mean"))}° '
            f'정착 {_fmt(d.get("settle_mean"), 2)} s OS {_fmt(d.get("os_mean"), 1)}%')


def list_snapshots(run_dir: str) -> list:
    """<run_dir>/checkpoints/*/meta.json 목록 (에피소드·시각 순)."""
    out = []
    base = os.path.join(run_dir, SNAP_DIR)
    if not os.path.isdir(base):
        return out
    for name in os.listdir(base):
        p = os.path.join(base, name, SNAP_META)
        try:
            with open(p, encoding='utf-8') as f:
                m = json.load(f)
        except (OSError, ValueError):
            continue
        m['name'] = name
        out.append(m)
    out.sort(key=lambda m: (m.get('episode', 0), m.get('time', ''), m['name']))
    return out


class Trainer(threading.Thread):
    RECORD_LOG = 'train_log.csv'      # 하위 클래스(rl.play)가 바꿈
    RECORD_CHECKPOINT = True

    def __init__(self, cfg: Config, link_factory, run_dir: str, resume: bool = False, autostart: bool = False,
                 log=print, sim: bool = False):
        super().__init__(name='rl-trainer', daemon=True)
        from .sac import SAC   # torch 는 여기서만 필요
        self.cfg = cfg
        self.link_factory = link_factory
        self.link = None
        self.run_dir = run_dir
        self.autostart = autostart
        self.sim = sim
        self._print = log or (lambda s: None)
        self.rec = Recorder(run_dir, self.RECORD_LOG, self.RECORD_CHECKPOINT)
        save_config(cfg, os.path.join(run_dir, 'config.json'))
        self.sac = SAC(obs_dim(cfg), ACT_DIM, cfg.sac)
        self.env = EpisodeEnv(cfg)
        self.rng = np.random.default_rng(cfg.sac.seed + 100)
        self._cmdq = queue.Queue()
        self._lock = threading.Lock()
        self.ts = cfg.ts_s
        # 진행 상태 (체크포인트에 저장)
        self.train_episodes = 0
        self.eval_rounds = 0
        self.decisions = 0
        self.fallbacks = 0
        self.aborts = 0
        self.misses = 0
        self.pauses = 0
        self.eval_episodes = 0
        self.elapsed_prev = 0.0
        self.baseline = None
        self.baseline_history = []
        self.eval_scores = []
        self.curve = deque(maxlen=2000)
        self.series = deque(maxlen=max(1, cfg.run.keep_episode_series))
        self.series_eval = None
        self.series_base = None
        self.ep_index = 0              # CSV 이름용 전체 에피소드 번호
        # 실행 중 상태
        self.state = 'waiting'
        self.detail = ''
        self.paused_reason = None
        self.last_error = None
        self.current = None
        self.motor_on_s = 0.0
        self.rest_left = None
        self.mean_pwm = 0.0
        self.last_s = None
        self.meta = {}
        self.logbuf = deque(maxlen=50)
        self._t_start = None
        self._run_started = False
        self._last_rx = 0.0
        self._prev_overruns = None
        self._sat_run = 0
        self._gap_q = deque()          # 모터 가동 중 θ − θacc [deg] (att_gap_win_s 창)
        self._gap_sum = 0.0
        self.att_gap = None            # 최근 창 평균 θ − θacc [deg]
        self._gap_warn_t = None
        self._in_episode = False
        self._abort_flag = None
        self._finished = threading.Event()
        self.manual_evals = 0          # 수동 평가 횟수 (정기 일정 계산에서 뺌; 옛 체크포인트엔 없음 → 0)
        self._pending = deque()        # 에피소드 경계에서 처리할 'snapshot' / 'eval'
        self.snapshots = []
        self.best_rms = None
        self._last_eval = None         # (평가한 정책의 갱신 수, RMS)
        if resume:
            self._load_checkpoint()
        self._scan_snapshots()

    # ================================================================== 외부 인터페이스
    COMMANDS = ('start', 'pause', 'resume', 'stop', 'snapshot', 'eval')

    def command(self, cmd: str):
        cmd = (cmd or '').strip().lower()
        if cmd not in self.COMMANDS:
            return False, f'알 수 없는 명령: {cmd}'
        if self._finished.is_set():
            return False, '이미 끝났습니다'
        self._cmdq.put(cmd)
        if cmd == 'snapshot':
            return True, 'snapshot 요청됨 (다음 에피소드 경계에서, 대기·일시정지 중이면 바로 저장)'
        if cmd == 'eval':
            return True, 'eval 요청됨 (다음 에피소드 경계에서 평가 라운드)'
        return True, f'{cmd} 요청됨'

    def status(self) -> dict:
        c = self.cfg
        with self._lock:
            elapsed = self._elapsed()
            return dict(
                state=self.state, detail=self.detail, paused_reason=self.paused_reason,
                link=getattr(self.link, 'name', None), fw=getattr(self.link, 'fw', None), run_dir=self.run_dir,
                episode=self.train_episodes, eval_rounds=self.eval_rounds, decisions=self.decisions,
                updates=self.sac.updates, buffer=self.sac.buffer.n,
                warmup_left=max(0, c.sac.warmup_steps - self.decisions),
                current=dict(self.current) if self.current else None,
                recent_episodes=[s for s in (list(self.series) + [self.series_eval, self.series_base]) if s],
                learning_curve=list(self.curve), eval_scores=list(self.eval_scores),
                baseline=dict(self.baseline) if self.baseline else None,
                baseline_history=list(self.baseline_history),
                counts=dict(fallbacks=self.fallbacks, aborts=self.aborts, misses=self.misses, pauses=self.pauses,
                            train_episodes=self.train_episodes, eval_episodes=self.eval_episodes),
                mean_pwm=self.mean_pwm, motor_on_s=self.motor_on_s, rest_left_s=self.rest_left,
                att_gap_deg=self.att_gap, att_gap_max_deg=c.monitor.att_gap_max_deg,
                att_gap_warn_deg=c.monitor.att_gap_warn_deg,
                elapsed_s=elapsed, remaining_s=max(0.0, c.stop.max_hours * 3600 - elapsed),
                episodes_left=max(0, c.stop.max_episodes - self.train_episodes),
                sac={k: (None if isinstance(v, float) and math.isnan(v) else v) for k, v in self.sac.last.items()},
                last_s=dict(self.last_s) if self.last_s else None, last_error=self.last_error,
                snapshots=[dict(name=m['name'], kind=m.get('kind'), episode=m.get('episode'),
                                updates=m.get('updates'), eval_rms=m.get('eval_rms'),
                                recent_eval_rms=m.get('recent_eval_rms'), time=m.get('time'))
                           for m in self.snapshots],
                best_rms=self.best_rms, pending=list(self._pending),
                log=list(self.logbuf))

    def wait(self, timeout=None):
        return self._finished.wait(timeout)

    # ================================================================== 내부 공통
    def _elapsed(self):
        if self._t_start is None or self.link is None:
            return self.elapsed_prev
        return self.elapsed_prev + (self.link.now() - self._t_start)

    def _set(self, state=None, detail=None):
        with self._lock:
            if state is not None:
                self.state = state
            if detail is not None:
                self.detail = detail

    def _event(self, text: str):
        line = f'[{self._elapsed():8.1f}s] {text}'
        with self._lock:
            self.logbuf.append(line)
        self.rec.event(text)
        self._print(line)

    def _check_commands(self):
        while True:
            try:
                cmd = self._cmdq.get_nowait()
            except queue.Empty:
                return
            if cmd == 'pause':
                raise Pause('사용자 일시정지')
            if cmd == 'stop':
                raise StopRun('사용자 정지')
            if cmd in ('snapshot', 'eval'):
                self._defer(cmd)
            # start/resume 은 실행 중에는 의미 없음

    def _defer(self, cmd):
        with self._lock:
            if cmd not in self._pending:
                self._pending.append(cmd)

    def _take(self, cmd) -> bool:
        with self._lock:
            if cmd in self._pending:
                self._pending.remove(cmd)
                return True
        return False

    def _gap_reset(self):
        self._gap_q.clear()
        self._gap_sum = 0.0

    def _gap_check(self, g: float):
        """모터 가동 중 θ − θacc 이동평균 감시 (진동에 의한 자이로 편향 → 자세 추정 표류)."""
        mc = self.cfg.monitor
        n = max(1, int(round(mc.att_gap_win_s / self.ts)))
        self._gap_q.append(g)
        self._gap_sum += g
        while len(self._gap_q) > n:
            self._gap_sum -= self._gap_q.popleft()
        if len(self._gap_q) < n:
            return
        mean = self._gap_sum / n
        self.att_gap = round(mean, 3)
        if mc.att_gap_max_deg > 0 and abs(mean) > mc.att_gap_max_deg:
            self._gap_reset()
            raise Pause(f'자세 추정 이상: θ − θacc {mean:+.1f}° ({mc.att_gap_win_s:g} s 평균) > {mc.att_gap_max_deg:g}° '
                        f'— 진동으로 자이로가 편향된 것으로 의심 (프로펠러·모터·마운트 확인, 쉬면 회복될 수 있음)')
        if mc.att_gap_warn_deg > 0 and abs(mean) > mc.att_gap_warn_deg:
            now = self._elapsed()
            if self._gap_warn_t is None or now - self._gap_warn_t > 60.0:
                self._gap_warn_t = now
                self._event(f'경고: θ − θacc {mean:+.1f}° ({mc.att_gap_win_s:g} s 평균) > {mc.att_gap_warn_deg:g}°')

    def _monitor(self, m):
        """모든 수신 줄 공통 감시. 일시정지가 필요하면 Pause."""
        k = m.kind
        if k == 'D':
            f = m.f
            if f[1] != 0:
                self.motor_on_s += self.ts
                if len(f) >= 5:
                    self._gap_check((f[2] - f[4]) * 0.01)
            else:
                self._gap_reset()
            cur = self.current
            if cur is not None:
                cur['theta'] = f[2] * 0.01
                cur['pwm'] = f[1]
                if len(f) >= 7:
                    cur['ref'] = f[6] * 0.01
        elif k == 'S':
            f = m.f
            if len(f) >= 9:
                with self._lock:
                    self.last_s = dict(state=f[0], maxTick_us=f[4], overruns=f[5], gate_pct=f[7], sat=f[8])
                mc = self.cfg.monitor
                if f[4] >= 5000:
                    self._event(f'경고: maxTick {f[4]} µs ≥ 5000')
                prev = self._prev_overruns
                self._prev_overruns = f[5]
                if prev is not None and 0 <= f[5] - prev and f[5] - prev > mc.max_overrun_delta:
                    raise Pause(f'S 줄 overruns 급증 ({prev} → {f[5]})')
                self._sat_run = self._sat_run + 1 if f[8] > 0 else 0
                if f[8] > mc.sat_max_per_s:
                    raise Pause(f'센서 포화 sat={f[8]}')
                if self._sat_run >= mc.sat_consec:
                    raise Pause(f'센서 포화가 {self._sat_run} 초 연속')
                if f[1] == 0 and not self.cfg.run.bench:
                    raise Pause('IMU 오류 (S 줄 imu=0)')
        elif k == 'EVT':
            w = m.words
            if w and w[0] == 'ABORT':
                self.aborts += 1
                self._event('EVT ABORT ' + ' '.join(w[1:]))
                if self._in_episode:
                    self._abort_flag = ' '.join(w[1:])
                else:
                    raise Pause('EVT ABORT ' + ' '.join(w[1:]))
        elif k == 'ERR':
            self._event('보드 ' + m.raw)
        elif k == 'LINKERR':
            raise Pause(f'포트 오류: {m.raw}')

    def _recv(self, timeout: float):
        link = self.link
        link.kick()
        self._check_commands()
        m = link.recv(timeout)
        now = link.now()
        if m is None:
            if now - self._last_rx > self.cfg.monitor.link_timeout_s:
                raise Pause(f'연결 끊김: {now - self._last_rx:.1f} s 동안 수신 없음')
            return None
        self._last_rx = now
        self._monitor(m)
        return m

    def _request(self, cmd, kinds, timeout=2.0):
        m = self.link.request(cmd, kinds, timeout=timeout, sink=self._monitor)
        self._last_rx = self.link.now()
        return m

    def _wait(self, seconds: float, on_msg=None):
        """seconds 동안 줄을 받아 감시 (on_msg(m) 이 True 를 돌려주면 일찍 끝)."""
        t_end = self.link.now() + seconds
        while True:
            left = t_end - self.link.now()
            if left <= 0:
                return False
            m = self._recv(min(0.5, left))
            if m is not None and on_msg is not None and on_msg(m):
                return True

    # ================================================================== 연결·설정
    def _connect(self):
        self._set('connecting', '보드 연결')
        if self._t_start is not None and self.link is not None:
            self.elapsed_prev = self._elapsed()      # 새 연결은 시계가 0 부터라 지금까지를 누적
            self._t_start = None
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                pass
        self.link = self.link_factory()
        self.link.hb_enabled = True
        if hasattr(self.link, 'watchdog_s'):
            self.link.watchdog_s = self.cfg.monitor.hb_watchdog_s
        self.link.open()
        fw = self.link.hello(sink=None)
        self._last_rx = self.link.now()
        if self._run_started:
            self._t_start = self.link.now()
        self._event(f'연결: {self.link.name}, 펌웨어 {fw}')
        self._configure()

    def _configure(self):
        c = self.cfg
        expect = []

        def req(cmd, kind, check=None):
            m = self._request(cmd, kind)
            if m.kind == 'ERR':
                raise RuntimeError(f'설정 실패: {cmd} → {m.raw}')
            if check and not check(m.kv):
                raise RuntimeError(f'설정 확인 실패: {cmd} → {m.raw}')
            return m

        self.link.drain(self._monitor)
        self._request('STOP', 'EVT')
        req(f'BENCH {1 if c.run.bench else 0}', 'CFG', lambda kv: kv.get('bench') == str(1 if c.run.bench else 0))
        div = int(round(c.ts_s / 0.005))
        req(f'STREAM {div}', 'CFG', lambda kv: kv.get('div') == str(div))
        req(f'LIM {c.lim.pwm_max} {c.lim.angle_cdeg} {c.lim.rev}', 'CFG',
            lambda kv: kv.get('umax') == str(c.lim.pwm_max) and kv.get('alim') == str(c.lim.angle_cdeg))
        p = c.pid
        req(f'PID {round(p.kp * 1000)} {round(p.ki * 1000)} {round(p.kd * 1000)} {p.ff} {p.umin}', 'PIDCFG',
            lambda kv: kv.get('ff') == str(p.ff) and kv.get('umin') == str(p.umin))
        req(f'NOTCH {p.notch_cHz} {p.notch_zz} {p.notch_zp}' if p.notch_cHz else 'NOTCH 0', 'PIDCFG')
        r = c.rl
        req(f'RL {r.dec} {r.lag} {r.du_max} {r.u_lo} {r.u_hi}', 'RLCFG',
            lambda kv: (kv.get('dec'), kv.get('lag'), kv.get('dumax'), kv.get('ulo'), kv.get('uhi'))
            == tuple(map(str, (r.dec, r.lag, r.du_max, r.u_lo, r.u_hi))))
        s = c.safe
        req(f'SAFE {s.th_lo_cdeg} {s.th_hi_cdeg} {s.w_max_cdps}', 'RLCFG',
            lambda kv: (kv.get('thlo'), kv.get('thhi'), kv.get('wmax'))
            == tuple(map(str, (s.th_lo_cdeg, s.th_hi_cdeg, s.w_max_cdps))))
        m = req('GET', 'CFG')
        kv = m.kv
        self.ts = int(kv.get('ts_ms', 10)) / 1000.0
        self.meta = dict(axis=kv.get('axis', '?') + kv.get('sign', ''), beta=int(kv.get('beta', 0)) / 1000,
                         dlpf=kv.get('dlpf'), cal=kv.get('cal'), zero=kv.get('zero'), bench=kv.get('bench'),
                         sim=1 if self.sim else 0, fw=self.link.fw)
        if not self.sim and kv.get('cal') != '1':
            self._event('경고: 보드 보정(cal) 없음')
        self.link.drain(self._monitor)
        self._prev_overruns = None
        self._sat_run = 0
        self._event(f'설정 완료: RL {r.dec}/{r.lag}/{r.du_max}/{r.u_lo}-{r.u_hi}, SAFE {s.th_lo_cdeg}..{s.th_hi_cdeg}, '
                    f'PID {p.kp}/{p.ki}/{p.kd} ff {p.ff}, LIM {c.lim.pwm_max}/{c.lim.angle_cdeg}')

    # ================================================================== 에피소드 구성 요소
    def _home(self, home: float):
        """REF home → 정착 대기."""
        e = self.cfg.episode
        self._set(detail=f'복귀 {home:.1f}°')
        m = self._request(f'REF {round(home * 100)}', 'EVT')
        if m.kind == 'ERR':
            raise Pause(f'REF 실패: {m.raw}')
        t0 = self.link.now()
        hold = [0.0]

        def on(m):
            if m.kind == 'D':
                th = m.f[2] * 0.01
                hold[0] = hold[0] + self.ts if abs(th - home) < e.settle_tol_deg else 0.0
                return hold[0] >= e.settle_hold_s and self.link.now() - t0 >= e.home_min_s
            return False

        if not self._wait(e.settle_timeout_s, on):
            self._event(f'경고: {home:.1f}° 정착 시간 초과 (그대로 시작)')

    def _sample_refs(self):
        e = self.cfg.episode
        lo, hi = e.ref_lo, e.ref_hi

        def pick(away):
            for _ in range(100):
                x = self.rng.uniform(lo, hi)
                if away is None or abs(x - away) >= e.min_step_deg:
                    return x
            return x

        home = e.home_deg if e.home_deg is not None else pick(None)
        r1 = pick(home)
        r2 = pick(r1)
        frac = self.rng.uniform(*e.step_frac)
        return round(home, 2), round(r1, 2), round(r2, 2), frac

    def _policy_fn(self, mode: str):
        pol = self.sac.policy
        if mode == 'random':
            rng = np.random.default_rng(int(self.rng.integers(1 << 31)))
            return lambda o: rng.uniform(-1.0, 1.0)
        if mode == 'deterministic':
            return lambda o: pol.act(o, True)[0]
        return lambda o: pol.act(o)[0]

    def _series(self, rows, kind, scenario, status, rms, episode):
        r = rows[::2]
        t0 = rows[0][0] if rows else 0
        return dict(episode=episode, kind=kind, scenario=scenario, status=status, rms=rms,
                    t=[round((x[0] - t0) / 1000, 3) for x in r], theta=[round(x[2], 2) for x in r],
                    ref=[round(x[6], 2) for x in r], pwm=[x[1] for x in r])

    def _finish_episode(self, kind, scenario, status, rows, summ, home, r1, r2, step_s, extra_rms=None, ss=None):
        """기록·상태 갱신 공통."""
        self.ep_index += 1
        name = f'ep{self.ep_index:05d}_{kind}'
        c = self.cfg
        if c.run.save_episode_csv and rows:
            meta = dict(self.meta, status=status, start=time.strftime('%Y-%m-%dT%H:%M:%S'),
                        params=dict(mode=kind, home=home, r1=r1, r2=r2, step_s=f'{step_s:.2f}', dur_s=c.episode.dur_s,
                                    scenario=scenario))
            self.rec.episode_csv(name, kind, meta, to_dict(c)['pid'], rows, self.ts)
        sl = self.sac.last
        row = dict(episode=self.train_episodes, kind=kind, scenario=scenario, status=status,
                   wall_s=round(time.time(), 1), board_s=round(self._elapsed(), 2), home=home, r1=r1, r2=r2,
                   step_s=round(step_s, 2), samples=summ['samples'], decisions=summ['decisions'],
                   ret=round(summ['ret'], 4), rms=round(summ['rms'], 4),
                   rms_after_step=round(extra_rms, 4) if extra_rms is not None else '',
                   mean_pwm=round(summ['mean_pwm'], 1), miss=summ['miss'], miss_frac=round(summ['miss_frac'], 4),
                   mean_abs_du=round(summ['mean_abs_du'], 3), mean_abs_di=round(summ.get('mean_abs_di', 0.0), 3),
                   fallbacks_total=self.fallbacks,
                   updates=self.sac.updates, buffer=self.sac.buffer.n, alpha=round(sl['alpha'], 5),
                   q_loss=round(sl['q_loss'], 5) if not math.isnan(sl['q_loss']) else '',
                   pi_loss=round(sl['pi_loss'], 5) if not math.isnan(sl['pi_loss']) else '',
                   entropy=round(sl['entropy'], 4) if not math.isnan(sl['entropy']) else '',
                   motor_on_s=round(self.motor_on_s, 1))
        ss = ss or {}
        on = [x[2] - x[4] for x in rows if x[1]]
        att_gap = round(sum(on) / len(on), 3) if on else None
        row['att_gap'] = '' if att_gap is None else att_gap
        row.update({k: ('' if ss.get(k) is None else ss[k])
                    for k in ('ss_e1', 'ss_e2', 'ss_abs', 'ss_pwm_sd', 'ss_bias', 'ss_ripple', 'settle_s', 'os_pct')})
        row['return'] = row.pop('ret')
        self.rec.log_row(row)
        ser = self._series(rows, kind, scenario, status, round(summ['rms'], 3), self.train_episodes)
        with self._lock:
            self.mean_pwm = summ['mean_pwm']
            self.curve.append(dict(episode=self.train_episodes, kind=kind, status=status, ret=round(summ['ret'], 4),
                                   rms=round(summ['rms'], 3), miss_frac=round(summ['miss_frac'], 4),
                                   mean_pwm=round(summ['mean_pwm'], 1), alpha=round(sl['alpha'], 5),
                                   ss_abs=ss.get('ss_abs'), ss_pwm_sd=ss.get('ss_pwm_sd'), ss_bias=ss.get('ss_bias'),
                                   ss_ripple=ss.get('ss_ripple'), settle_s=ss.get('settle_s'), os_pct=ss.get('os_pct'),
                                   att_gap=att_gap,
                                   time_s=round(self._elapsed(), 1)))
            if kind == 'train':
                self.series.append(ser)
            elif kind == 'eval':
                self.series_eval = ser
            else:
                self.series_base = ser

    def _rl_episode(self, kind: str, home, r1, r2, frac, mode: str, scenario=''):
        """RUN RL 에피소드 하나. 전이는 리플레이에 넣고 요약을 돌려줌."""
        c = self.cfg
        dur_ms = int(round(c.episode.dur_s * 1000))
        total = int(round(dur_ms / (self.ts * 1000)))
        step_n = int(frac * total)
        env = self.env
        env.arm(self._policy_fn(mode), c.rl.dec, c.rl.du_max)
        self._abort_flag = None
        self._in_episode = True
        with self._lock:
            self.current = dict(kind=kind, home=home, r1=r1, r2=r2, n=0, total=total, ref=r1, theta=None, pwm=None)
        self._set(detail=f'{kind} 에피소드 {r1:.1f}° → {r2:.1f}° (home {home:.1f}°)')
        status = 'stopped'
        self.link.fast_path = env.on_D
        try:
            m = self._request(f'RUN RL {round(r1 * 100)} {dur_ms} {round(home * 100)}', 'EVT')
            if m.kind == 'ERR' or not m.words or m.words[0] != 'RUN':
                raise Pause(f'RUN RL 실패: {m.raw}')
            sent2 = False
            while True:
                m = self._recv(0.5)
                if self._abort_flag is not None:
                    env.terminal('ABORT')
                    status = 'abort'
                    break
                if m is None:
                    continue
                if m.kind == 'D':
                    if m.info is not None:
                        if 'fast_error' in m.info:
                            raise RuntimeError('정책 계산 오류: ' + m.info['fast_error'])
                        n = m.info['n']
                        self.current['n'] = n
                        if not sent2 and n >= step_n:
                            self.link.send(f'REF {round(r2 * 100)}')
                            sent2 = True
                elif m.kind == 'EVT':
                    w0 = m.words[0] if m.words else ''
                    if w0 == 'DONE':
                        env.finish()
                        status = 'done'
                        break
                    if w0 == 'FALLBACK':
                        env.terminal('FALLBACK')
                        status = 'fallback'
                        self.fallbacks += 1
                        self._event(f'FALLBACK {" ".join(m.words[1:])} ({kind}, {r1:.1f}→{r2:.1f}°)')
                        break
                    if w0 == 'STOP':
                        env.finish()
                        status = 'stopped'
                        break
        finally:
            self.link.fast_path = None
            env.active = False
            self._in_episode = False
            with self._lock:
                self.current = None
        # 전이 → 리플레이
        if kind in ('train', 'eval'):
            for o, a, r, o2, d in env.trans:
                self.sac.buffer.add(o, a, r, o2, d)
            self.decisions += len(env.trans)
        summ = env.summary()
        self.misses += summ['miss']
        rows = env.rows
        ras = None
        if rows:
            seg = rows[step_n:] if len(rows) > step_n else rows
            ras = math.sqrt(sum((x[6] - x[2]) ** 2 for x in seg) / len(seg))
        ss = steady_metrics(rows, step_n, int(round(c.run.ss_win_s / self.ts)), c.run.settle_band_deg, self.ts)
        summ['ss'] = ss
        self._finish_episode(kind, scenario, status, rows, summ, home, r1, r2, step_n * self.ts, ras, ss)
        if status == 'abort':
            raise Pause(f'EVT ABORT {self._abort_flag}')
        if status == 'stopped':
            raise Pause('에피소드 중 예상치 못한 EVT STOP')
        if summ['decisions'] > 20 and summ['miss_frac'] > c.monitor.max_miss_frac:
            raise Pause(f'miss 비율 {summ["miss_frac"]:.1%} > {c.monitor.max_miss_frac:.0%}')
        return summ, status, len(env.trans)

    def _pid_episode(self, home, r1, r2, frac, scenario='', kind='baseline'):
        """PID 기준선: 같은 기준 스텝을 REF 로."""
        c = self.cfg
        total = int(round(c.episode.dur_s / self.ts))
        step_n = int(frac * total)
        with self._lock:
            self.current = dict(kind=kind, home=home, r1=r1, r2=r2, n=0, total=total, ref=r1, theta=None, pwm=None)
        self._set(detail=f'PID 기준선 {r1:.1f}° → {r2:.1f}°')
        rows = []
        m = self._request(f'REF {round(r1 * 100)}', 'EVT')
        if m.kind == 'ERR':
            raise Pause(f'REF 실패: {m.raw}')
        sent2 = False
        se, pw = 0.0, 0.0
        try:
            while len(rows) < total:
                m = self._recv(0.5)
                if m is None or m.kind != 'D' or len(m.f) != 7:
                    continue
                t_ms, u, th, om, tha, ph, ref = m.f
                th, ref = th * 0.01, ref * 0.01
                rows.append([t_ms, u, th, om * 0.01, tha * 0.01, ph, ref, None, None, None])
                se += (ref - th) ** 2
                pw += u
                self.current['n'] = len(rows)
                if not sent2 and len(rows) >= step_n:
                    self.link.send(f'REF {round(r2 * 100)}')
                    sent2 = True
        finally:
            with self._lock:
                self.current = None
        n = max(1, len(rows))
        seg = rows[step_n:]
        ras = math.sqrt(sum((x[6] - x[2]) ** 2 for x in seg) / max(1, len(seg)))
        ss = steady_metrics(rows, step_n, int(round(c.run.ss_win_s / self.ts)), c.run.settle_band_deg, self.ts)
        summ = dict(samples=len(rows), decisions=0, ret=0.0, rms=math.sqrt(se / n), mean_pwm=pw / n, miss=0,
                    miss_frac=0.0, mean_abs_du=0.0, ss=ss)
        self._finish_episode(kind, scenario, 'done', rows, summ, home, r1, r2, step_n * self.ts, ras, ss)
        return summ

    # ================================================================== 단계
    def _baseline_round(self):
        self._set('baseline')
        per, sss = [], []
        for i, (home, r1, r2) in enumerate(self.cfg.run.eval_refs):
            self._home(home)
            s = self._pid_episode(home, r1, r2, 0.5, scenario=str(i))
            per.append(round(s['rms'], 4))
            sss.append(s['ss'])
        b = dict(rms_mean=round(float(np.mean(per)), 4), per_scenario=per, **_seg_summary(sss),
                 episode=self.train_episodes, time_s=round(self._elapsed(), 1))
        ssm, pss = b['ss_mean'], b['ss_per_scenario']
        with self._lock:
            self.baseline = b
            self.baseline_history.append(b)
        self._event(f'PID 기준선 (학습 {self.train_episodes} ep): RMS {b["rms_mean"]:.3f}° {per}, '
                    f'정상상태 |e| {_fmt(ssm)}° {pss}, {_seg_text(b)}')

    def _eval_round(self, manual: bool = False):
        self._set('eval')
        per, sss = [], []
        for i, (home, r1, r2) in enumerate(self.cfg.run.eval_refs):
            self._home(home)
            s, status, _ = self._rl_episode('eval', home, r1, r2, 0.5, 'deterministic', scenario=str(i))
            self.eval_episodes += 1
            per.append(round(s['rms'], 4) if status == 'done' else round(s['rms'], 4) + 100.0)
            sss.append(s['ss'] if status == 'done' else {})
        self.eval_rounds += 1
        if manual:
            self.manual_evals += 1
        sc = dict(episode=self.train_episodes, rms_mean=round(float(np.mean(per)), 4), per_scenario=per,
                  **_seg_summary(sss), time_s=round(self._elapsed(), 1))
        pss = sc['ss_per_scenario']
        if manual:
            sc['manual'] = True
        with self._lock:
            self.eval_scores.append(sc)
        base = f' (PID {self.baseline["rms_mean"]:.3f}°)' if self.baseline else ''
        bss = f' (PID {_fmt(self.baseline.get("ss_mean"))}°)' if self.baseline else ''
        bseg = f' | PID {_seg_text(self.baseline)}' if self.baseline else ''
        self._event(f'{"수동 " if manual else ""}평가 #{self.eval_rounds} (학습 {self.train_episodes} ep): '
                    f'RMS {sc["rms_mean"]:.3f}°{base} {per}, 정상상태 |e| {_fmt(sc["ss_mean"])}°{bss} {pss}, '
                    f'{_seg_text(sc)}{bseg}')
        # 같은 정책(갱신 수가 같음)의 스냅샷에 평가 결과를 적고, 최고면 best 스냅샷
        self._last_eval = (self.sac.updates, sc['rms_mean'])
        self._annotate_snapshots(sc['rms_mean'])
        if self.best_rms is None or sc['rms_mean'] < self.best_rms:
            prev = self.best_rms
            self._save_snapshot('best', 'best', eval_rms=sc['rms_mean'])
            self._event(f'best 스냅샷 갱신: RMS {sc["rms_mean"]:.3f}°'
                        + (f' (이전 {prev:.3f}°)' if prev is not None else ''))

    def _eval_due(self) -> bool:
        """정기 평가 차례인가 (수동 평가는 세지 않음)."""
        c = self.cfg.run
        regular = self.eval_rounds - self.manual_evals
        return self.train_episodes > 0 and self.train_episodes % c.eval_every == 0 \
            and regular < self.train_episodes // c.eval_every

    def _eval_and_update(self, manual=False):
        c = self.cfg
        self._eval_round(manual=manual)
        regular = self.eval_rounds - self.manual_evals
        just_measured = self.baseline is not None and self.baseline.get('episode') == self.train_episodes
        if not manual and c.run.baseline_every > 0 and regular % c.run.baseline_every == 0 and not just_measured:
            self._baseline_round()     # 플랜트가 세션 중 흘러가므로 비교 기준도 다시 잼 (갱신 없음, 버퍼에 안 넣음)
        self._updates(len(c.run.eval_refs) * int(c.episode.dur_s / (self.ts * c.rl.dec)))

    # ================================================================== 스냅샷
    def _scan_snapshots(self):
        snaps = list_snapshots(self.run_dir)
        best = [m for m in snaps if m['name'] == 'best']
        with self._lock:
            self.snapshots = snaps
            self.best_rms = best[0].get('eval_rms') if best else None

    def _save_snapshot(self, name: str, kind: str, eval_rms=None):
        """정책 스냅샷: checkpoints/<name>/actor.pt + meta.json. 실패해도 학습은 계속."""
        try:
            d = os.path.join(self.run_dir, SNAP_DIR, name)
            self.sac.save_policy(d)
            recent = self.eval_scores[-1]['rms_mean'] if self.eval_scores else None
            meta = dict(name=name, kind=kind, episode=self.train_episodes, decisions=self.decisions,
                        updates=self.sac.updates, eval_rms=eval_rms, recent_eval_rms=recent,
                        time=time.strftime('%Y-%m-%dT%H:%M:%S'), elapsed_s=round(self._elapsed(), 1),
                        obs_dim=self.sac.obs_dim, alpha=round(self.sac.alpha, 6))
            self._write_meta(d, meta)
            with self._lock:
                self.snapshots = [m for m in self.snapshots if m['name'] != name] + [meta]
                self.snapshots.sort(key=lambda m: (m.get('episode', 0), m.get('time', ''), m['name']))
                if name == 'best':
                    self.best_rms = eval_rms
            if kind != 'best':
                self._event(f'스냅샷 {name} 저장 (학습 {self.train_episodes} ep, 갱신 {self.sac.updates})')
            return meta
        except Exception as e:
            self._event(f'스냅샷 {name} 저장 실패: {e}')
            return None

    @staticmethod
    def _write_meta(d, meta):
        tmp = os.path.join(d, SNAP_META + '.tmp')
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)
        os.replace(tmp, os.path.join(d, SNAP_META))

    def _annotate_snapshots(self, rms):
        for m in list(self.snapshots):
            if m.get('updates') == self.sac.updates and m.get('eval_rms') is None:
                with self._lock:
                    m['eval_rms'] = rms
                try:
                    self._write_meta(os.path.join(self.run_dir, SNAP_DIR, m['name']), m)
                except OSError:
                    pass

    def _manual_snapshot(self):
        name = f'manual_{self.train_episodes:05d}_{time.strftime("%H%M%S")}'
        base, k = name, 2
        while os.path.exists(os.path.join(self.run_dir, SNAP_DIR, name)):
            name = f'{base}_{k}'
            k += 1
        le = self._last_eval
        ev = le[1] if le is not None and le[0] == self.sac.updates else None
        return self._save_snapshot(name, 'manual', eval_rms=ev)

    def _process_pending(self, allow_eval: bool = True):
        """에피소드 경계: 쌓인 명령 처리 (모터는 PID 로 home 유지 중이거나 정지)."""
        self._check_commands()
        if self._take('snapshot'):
            self._manual_snapshot()
        if allow_eval and self._take('eval'):
            self._eval_and_update(manual=True)

    def _train_episode(self):
        home, r1, r2, frac = self._sample_refs()
        self._set('homing')
        self._home(home)
        self._set('episode')
        mode = 'random' if self.decisions < self.cfg.sac.warmup_steps else 'stochastic'
        summ, status, n_tr = self._rl_episode('train', home, r1, r2, frac, mode)
        self.train_episodes += 1
        return n_tr

    def _updates(self, n_new: int):
        c = self.cfg.sac
        if self.decisions < c.warmup_steps or self.sac.buffer.n < c.batch:
            return
        n = int(round(c.utd * n_new))
        if n <= 0:
            return
        self._set('updating', f'경사 갱신 {n}회')
        done = 0
        while done < n:
            k = min(50, n - done)
            self.sac.update(k)
            done += k
            self.link.kick()
            self._check_commands()
            self.link.drain(self._monitor)
        self.sac.sync_policy()

    def _maybe_rest(self):
        r = self.cfg.rest
        if self.motor_on_s < r.max_on_s:
            return
        self._set('resting', f'휴식 {r.rest_s:.0f} s')
        self._request('STOP', 'EVT')
        self._event(f'휴식 시작 (모터 가동 {self.motor_on_s:.0f} s)')
        t_end = self.link.now() + r.rest_s
        last = deque(maxlen=max(1, int(1.0 / self.ts)))
        while True:
            left = t_end - self.link.now()
            with self._lock:
                self.rest_left = max(0.0, left)
            if left <= 0:
                break
            m = self._recv(min(0.5, left))
            if m is not None and m.kind == 'D':
                last.append(m.f[2] * 0.01)
            if self._take('snapshot'):       # 모터가 멈춰 있으니 스냅샷은 휴식 중에도 바로
                self._manual_snapshot()
        with self._lock:
            self.rest_left = None
        self.motor_on_s = 0.0
        # 정지 확인: 케이블 간섭으로 정지 위치가 1–3° 남을 수 있으므로 너그럽게
        lim = self.cfg.monitor.rest_check_deg
        if last and not self.cfg.run.bench:
            th = float(np.mean(last))
            if abs(th) > lim:
                raise Pause(f'휴식 끝 정지 각도 {th:.1f}° (|θ| > {lim}°): 진자·센서를 확인하세요')
            self._event(f'휴식 끝, 정지 각도 {th:.1f}°')

    def _check_stop(self):
        s = self.cfg.stop
        if self._elapsed() >= s.max_hours * 3600:
            raise StopRun(f'총 시간 {s.max_hours} h 도달')
        if self.train_episodes >= s.max_episodes:
            raise StopRun(f'학습 에피소드 {s.max_episodes} 도달')
        if self.fallbacks >= s.max_fallbacks:
            raise StopRun(f'FALLBACK 누적 {self.fallbacks} ≥ {s.max_fallbacks}')

    def _pause(self, reason: str):
        try:                       # 모터 정지를 먼저 (상태 표시보다)
            if self.link is not None and not self.link.error:
                self.link.send('STOP')
        except Exception:
            pass
        with self._lock:
            self.paused_reason = reason
            self.current = None
        self.pauses += 1
        self._set('paused', '재개(resume) 대기')
        self._event(f'일시정지: {reason}')
        self._save_checkpoint()
        if self._take('snapshot'):
            self._manual_snapshot()
        while True:
            try:
                cmd = self._cmdq.get(timeout=0.2)
            except queue.Empty:
                cmd = None
                try:
                    if self.link is not None and not self.link.error and not self.link.is_virtual:
                        self.link.drain()
                except Exception:
                    pass
            if cmd == 'stop':
                raise StopRun('사용자 정지 (일시정지 중)')
            if cmd == 'resume':
                break
            if cmd == 'snapshot':
                self._manual_snapshot()        # 모터 정지 상태 → 바로
            elif cmd == 'eval':
                self._defer('eval')            # 재개 뒤 첫 경계에서
                self._event('eval 요청: 재개 뒤 실행')
        self._event('재개')
        with self._lock:
            self.paused_reason = None
        if self.link is None or self.link.error:
            self._connect()
        else:
            self.link.drain()
            self._last_rx = self.link.now()
            try:
                self._configure()
            except (LinkError, RuntimeError) as e:
                self._event(f'재설정 실패({e}), 다시 연결')
                self._connect()
        self.motor_on_s = 0.0      # 일시정지 동안 식었다고 봄
        self._gap_reset()

    # ================================================================== 체크포인트
    def _state_dict(self):
        return dict(train_episodes=self.train_episodes, eval_rounds=self.eval_rounds, decisions=self.decisions,
                    fallbacks=self.fallbacks, aborts=self.aborts, misses=self.misses, pauses=self.pauses,
                    eval_episodes=self.eval_episodes, elapsed=self._elapsed(), baseline=self.baseline,
                    baseline_history=self.baseline_history,
                    eval_scores=self.eval_scores, curve=list(self.curve), ep_index=self.ep_index,
                    manual_evals=self.manual_evals, rng=self.rng.bit_generator.state)

    def _save_checkpoint(self):
        try:
            d = os.path.join(self.run_dir, 'checkpoint')
            st = self._state_dict()
            self.sac.save(d, extra=st)
            js = {k: v for k, v in st.items() if k not in ('curve', 'rng')}
            with open(os.path.join(d, 'trainer.json'), 'w', encoding='utf-8') as f:
                json.dump(js, f, ensure_ascii=False, indent=1)
        except Exception as e:
            self._print(f'체크포인트 저장 실패: {e}')

    def _load_checkpoint(self):
        d = os.path.join(self.run_dir, 'checkpoint')
        st = self.sac.load(d)
        for k in ('train_episodes', 'eval_rounds', 'decisions', 'fallbacks', 'aborts', 'misses', 'pauses',
                  'eval_episodes', 'baseline', 'eval_scores', 'ep_index', 'manual_evals',
                  'baseline_history'):   # 없는 키는 기본값 유지
            if k in st:
                setattr(self, k, st[k])
        self.elapsed_prev = st.get('elapsed', 0.0)
        if not self.baseline_history and self.baseline:
            self.baseline_history = [self.baseline]
        self.curve.extend(st.get('curve', []))
        if 'rng' in st:
            self.rng.bit_generator.state = st['rng']
        self.rec.event(f'이어 하기: 학습 {self.train_episodes} ep, 결정 {self.decisions}, 갱신 {self.sac.updates}')

    # ================================================================== 스레드 본체
    def run(self):
        final = 'done'
        try:
            if not self.autostart:
                while True:
                    cmd = self._cmdq.get()
                    if cmd == 'start':
                        break
                    if cmd == 'stop':
                        raise StopRun('시작 전 정지')
                    if cmd == 'snapshot':
                        self._manual_snapshot()
                    elif cmd == 'eval':
                        self._defer('eval')
            while True:   # 연결 (실패하면 일시정지 → resume 으로 재시도)
                try:
                    self._connect()
                    break
                except (LinkError, OSError) as e:
                    self.last_error = str(e)
                    self._pause_before_connect(str(e))
            self._t_start = self.link.now()
            self._run_started = True
            c = self.cfg
            while True:
                try:
                    if c.run.baseline_at_start and self.baseline is None:
                        self._baseline_round()
                    self._check_stop()
                    self._maybe_rest()
                    self._process_pending()          # 수동 snapshot / eval
                    if self._eval_due():
                        self._eval_and_update()
                    n_new = self._train_episode()
                    self._updates(n_new)
                    if self.train_episodes % c.run.checkpoint_every == 0:
                        self._save_checkpoint()
                    if c.run.snapshot_every > 0 and self.train_episodes % c.run.snapshot_every == 0:
                        self._save_snapshot(f'ep{self.train_episodes:05d}', 'periodic')
                    if self.train_episodes % 10 == 0:
                        lc = [x for x in self.curve if x['kind'] == 'train'][-10:]
                        self._event(f'학습 {self.train_episodes} ep: 최근 10 RMS '
                                    f'{np.mean([x["rms"] for x in lc]):.2f}°, return {np.mean([x["ret"] for x in lc]):.2f}, '
                                    f'α {self.sac.alpha:.3f}, 버퍼 {self.sac.buffer.n}, FALLBACK {self.fallbacks}')
                except Pause as p:
                    self._pause(str(p))
        except StopRun as s:
            self._event(f'종료: {s}')
        except Exception as e:
            final = 'error'
            self.last_error = f'{type(e).__name__}: {e}'
            self._event('오류로 종료: ' + self.last_error)
            self.rec.event(traceback.format_exc())
        finally:
            self._set('stopping', '정지·저장')
            try:
                if self.link is not None and not self.link.error:
                    self.link.send('STOP')
                    if not self.link.is_virtual:
                        time.sleep(0.1)
            except Exception:
                pass
            self._save_checkpoint()
            try:
                if self.link is not None:
                    self.link.close()
            except Exception:
                pass
            self.rec.close()
            self._set(final, '끝')
            self._finished.set()

    def _pause_before_connect(self, reason):
        with self._lock:
            self.paused_reason = f'연결 실패: {reason}'
        self._set('paused', '재개(resume) 대기')
        self._event(f'연결 실패: {reason}')
        while True:
            cmd = self._cmdq.get()
            if cmd == 'stop':
                raise StopRun('사용자 정지 (연결 전)')
            if cmd == 'resume':
                return
