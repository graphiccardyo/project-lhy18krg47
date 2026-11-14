"""학습 없이 저장된 정책 시험 (기준 각도 유지·추종).

    python -m rl.play --port COM4 --ckpt data/20261114_rl1 --hold 25          (모터가 돕니다)
    python -m rl.play --port COM4 --ckpt data/20261114_rl1 --snapshot best --refs 20 30 22 --pid
    python -m rl.play --fake --fast --ckpt <run_dir> --hold 25 --out <폴더>     (가짜 보드, 가상시간)

--ckpt 로 받는 것
    실행 폴더            → checkpoint/sac.pt (가장 최근 이어 하기 체크포인트)
    실행 폴더 + --snapshot 이름 → checkpoints/<이름>/actor.pt
    스냅샷 폴더(actor.pt), checkpoint 폴더(sac.pt), 또는 .pt 파일
설정은 그 실행 폴더의 config.json 을 씁니다(정책 폴더에서 위로 올라가며 찾음). 정책은 결정적(평균 행동)이고
경사 갱신·리플레이 버퍼는 쓰지 않습니다. 학습기가 돌고 있을 때는 포트를 같이 열 수 없으니 실행하지 마세요.

시나리오: --hold θ 는 [θ, θ, θ] (한 각도 유지), --refs home r1 r2 [home r1 r2 ...] 는 학습기 평가와 같은 의미
(home 에서 정착 → r1 로 시작 → 에피소드 절반에서 r2). 둘 다 없으면 config 의 평가 시나리오 전부.

절차·안전은 학습기(trainer.Trainer)와 같은 코드: 연결·설정(BENCH, STREAM, LIM, PID, NOTCH, RL, SAFE, GET)
→ [REF home 정착 → RUN RL 에피소드] (→ --pid 면 같은 시나리오를 REF 로 보드 PID) → STOP.
EVT ABORT·연결 오류·miss 초과·Ctrl+C 어떤 경우에도 STOP 을 보내고 끝냅니다. ZERO 는 보내지 않습니다.

출력: 표(전체 RMS, 정착 구간 평균 |e|·σ(e), 오버슈트, 평균 PWM, PWM σ, miss, FALLBACK)와
<run_dir>/play/<YYYYMMDD_HHMMSS>/ (또는 --out) 에 episodes/*.csv(학습기와 같은 형식), play_log.csv, events.log,
config.json, summary.json, play.png.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback

import numpy as np

from .config import load_run_config
from .link import LinkError
from .trainer import Pause, StopRun, Trainer


# ====================================================================== 경로
def resolve_policy(ckpt: str, snapshot: str | None = None):
    """--ckpt/--snapshot → (정책 경로, 실행 폴더, 스냅샷 meta | None)."""
    p = os.path.abspath(ckpt)
    if not os.path.exists(p):
        raise SystemExit(f'--ckpt 경로가 없습니다: {p}')
    if snapshot:
        if not os.path.isdir(p):
            raise SystemExit('--snapshot 은 --ckpt 가 실행 폴더일 때만 씁니다')
        pol = os.path.join(p, 'checkpoints', snapshot)
        if not os.path.exists(os.path.join(pol, 'actor.pt')):
            have = sorted(os.listdir(os.path.join(p, 'checkpoints'))) if os.path.isdir(os.path.join(p, 'checkpoints')) else []
            raise SystemExit(f'스냅샷이 없습니다: {pol}\n  있는 스냅샷: {", ".join(have) or "(없음)"}')
    elif os.path.isfile(p):
        pol = p
    elif os.path.exists(os.path.join(p, 'actor.pt')) or os.path.exists(os.path.join(p, 'sac.pt')):
        pol = p
    elif os.path.exists(os.path.join(p, 'checkpoint', 'sac.pt')):
        pol = os.path.join(p, 'checkpoint')
    else:
        raise SystemExit(f'{p} 에서 정책(actor.pt / sac.pt / checkpoint/sac.pt)을 찾지 못했습니다')
    d = pol if os.path.isdir(pol) else os.path.dirname(pol)
    run_dir = None
    cur = d
    for _ in range(5):
        if os.path.exists(os.path.join(cur, 'config.json')):
            run_dir = cur
            break
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    if run_dir is None:
        raise SystemExit(f'{d} 위쪽에서 config.json 을 찾지 못했습니다')
    meta = None
    mp = os.path.join(d, 'meta.json')
    if os.path.exists(mp):
        with open(mp, encoding='utf-8') as f:
            meta = json.load(f)
    return pol, run_dir, meta


# ====================================================================== 지표
def episode_metrics(rows, home: float, ts: float, settle_s: float = 3.0) -> dict:
    """rows: (t_ms, pwm, th, om, tha, phase, ref, ...). 기준이 바뀐 구간마다 앞 settle_s 를 빼고 정착 구간으로 봄.

    rms: 전체 RMS(r−θ), settled_mean_abs_e / settled_std_e: 정착 구간 평균 |e|, e 의 표준편차,
    overshoot_deg/pct: 1° 이상 기준 변경마다 새 기준을 넘어간 최대량(최대 % 인 것), 처음 구간의 이전 기준 = home,
    mean_pwm: 전체 평균, pwm_std_settled: 정착 구간 안 PWM 표준편차(구간마다 평균을 뺀 합동 값)."""
    if not rows:
        return dict(samples=0)
    th = np.array([r[2] for r in rows], float)
    ref = np.array([r[6] for r in rows], float)
    u = np.array([r[1] for r in rows], float)
    e = ref - th
    n = len(rows)
    cuts = [0] + [i for i in range(1, n) if abs(ref[i] - ref[i - 1]) > 0.05] + [n]
    settle_n = int(round(settle_s / ts))
    sel = np.zeros(n, bool)
    os_deg, os_pct = None, None
    prev = home
    u_ss, u_n = 0.0, 0          # PWM 분산은 구간 안에서만 (기준마다 PWM 수준이 다르므로)
    for a, b in zip(cuts[:-1], cuts[1:]):
        if b <= a:
            continue
        r = ref[a]
        k = settle_n if b - a > settle_n + 10 else (b - a) // 2
        sel[a + k:b] = True
        us_seg = u[a + k:b]
        if len(us_seg):
            u_ss += float(np.sum((us_seg - us_seg.mean()) ** 2))
            u_n += len(us_seg)
        step = r - prev
        if abs(step) >= 1.0:
            ov = max(0.0, float(np.max((th[a:b] - r) * math.copysign(1.0, step))))
            pct = 100.0 * ov / abs(step)
            if os_deg is None or pct > os_pct:
                os_deg, os_pct = ov, pct
        prev = r
    es = e[sel]
    return dict(samples=n, rms=float(np.sqrt(np.mean(e ** 2))),
                settled_mean_abs_e=float(np.mean(np.abs(es))) if len(es) else None,
                settled_std_e=float(np.std(es)) if len(es) else None,
                settled_mean_e=float(np.mean(es)) if len(es) else None,
                overshoot_deg=os_deg, overshoot_pct=os_pct,
                mean_pwm=float(np.mean(u)), pwm_std_settled=math.sqrt(u_ss / u_n) if u_n else None,
                mean_abs_dpwm=float(np.mean(np.abs(np.diff(u)))) if n > 1 else 0.0)


# ====================================================================== 실행기
class Player(Trainer):
    """Trainer 의 연결·설정·복귀·에피소드 코드를 그대로 쓰고, 학습(갱신·버퍼·체크포인트)만 뺀 실행기."""
    RECORD_LOG = 'play_log.csv'
    RECORD_CHECKPOINT = False

    def __init__(self, cfg, link_factory, out_dir, policy_path, scenarios, repeat=1, pid=False, log=print,
                 sim=False, settle_s=3.0):
        super().__init__(cfg, link_factory, out_dir, resume=False, autostart=True, log=log, sim=sim)
        self.policy_info = self.sac.load_policy(policy_path)
        self.scenarios = scenarios
        self.repeat = repeat
        self.pid = pid
        self.settle_s = settle_s
        self.results = []
        self.end_reason = None
        self._rep = 0

    def _finish_episode(self, kind, scenario, status, rows, summ, home, r1, r2, step_s, extra_rms=None, ss=None):
        super()._finish_episode(kind, scenario, status, rows, summ, home, r1, r2, step_s, extra_rms, ss)
        m = episode_metrics(rows, home, self.ts, self.settle_s)
        self.results.append(dict(run=self._rep, kind='rl' if kind == 'play' else kind, scenario=int(scenario),
                                 status=status, home=home, r1=r1, r2=r2, miss=summ['miss'],
                                 fallback=status == 'fallback', csv=f'ep{self.ep_index:05d}_{kind}.csv',
                                 metrics=m, rows=rows))

    def _save_checkpoint(self):       # 재생은 체크포인트를 남기지 않음
        pass

    def _save_snapshot(self, *a, **k):
        return None

    def run(self):
        try:
            try:
                self._connect()
            except (LinkError, OSError, RuntimeError) as e:
                raise StopRun(f'연결·설정 실패: {e}')
            self._t_start = self.link.now()
            self._run_started = True
            for rep in range(self.repeat):
                self._rep = rep
                for i, (home, r1, r2) in enumerate(self.scenarios):
                    self._maybe_rest()
                    self._set('homing')
                    self._home(home)
                    self._set('episode')
                    self._rl_episode('play', home, r1, r2, 0.5, 'deterministic', scenario=str(i))
                    if self.pid:
                        self._maybe_rest()
                        self._set('homing')
                        self._home(home)
                        self._set('baseline')
                        self._pid_episode(home, r1, r2, 0.5, scenario=str(i), kind='pid')
            self._event('시험 끝')
        except (Pause, StopRun) as e:
            self.end_reason = str(e)
            self._event(f'중단: {e}')
        except LinkError as e:
            self.end_reason = f'연결 오류: {e}'
            self._event(self.end_reason)
        except Exception as e:
            self.end_reason = f'{type(e).__name__}: {e}'
            self._event('오류로 중단: ' + self.end_reason)
            self.rec.event(traceback.format_exc())
        finally:
            self._set('stopping', '정지')
            try:
                if self.link is not None and not self.link.error:
                    self.link.send('STOP')
                    if not self.link.is_virtual:
                        time.sleep(0.1)
            except Exception:
                pass
            try:
                if self.link is not None:
                    self.link.close()
            except Exception:
                pass
            self.rec.close()
            self._set('error' if self.end_reason else 'done', '끝')
            self._finished.set()


# ====================================================================== 출력
def _f(v, d=2):
    return '–' if v is None or (isinstance(v, float) and not math.isfinite(v)) else f'{v:.{d}f}'


def print_table(results):
    head = f'{"#":>2} {"제어":<4} {"시나리오":<20} {"상태":<8} {"RMS°":>6} {"정착|e|°":>8} {"σ(e)°":>6} ' \
           f'{"OS°":>5} {"OS%":>5} {"PWM":>6} {"PWMσ":>5} {"miss":>4} {"FB":>3}'
    print(head)
    print('-' * len(head) + '-' * 6)
    for k, r in enumerate(results):
        m = r['metrics']
        sc = f'{r["scenario"]}: {r["home"]:g}→{r["r1"]:g}→{r["r2"]:g}'
        print(f'{k + 1:>2} {r["kind"].upper():<4} {sc:<20} {r["status"]:<8} {_f(m.get("rms")):>6} '
              f'{_f(m.get("settled_mean_abs_e")):>8} {_f(m.get("settled_std_e")):>6} {_f(m.get("overshoot_deg"), 1):>5} '
              f'{_f(m.get("overshoot_pct"), 0):>5} {_f(m.get("mean_pwm"), 1):>6} {_f(m.get("pwm_std_settled"), 1):>5} '
              f'{r["miss"]:>4} {"예" if r["fallback"] else "-":>3}')


def aggregate(results):
    out = {}
    for kind in sorted({r['kind'] for r in results}):
        rs = [r for r in results if r['kind'] == kind]
        agg = dict(episodes=len(rs), fallbacks=sum(r['fallback'] for r in rs), miss=sum(r['miss'] for r in rs))
        for key in ('rms', 'settled_mean_abs_e', 'settled_std_e', 'overshoot_deg', 'overshoot_pct', 'mean_pwm',
                    'pwm_std_settled'):
            v = [r['metrics'].get(key) for r in rs if r['metrics'].get(key) is not None]
            agg[key] = round(float(np.mean(v)), 4) if v else None
        out[kind] = agg
    return out


def _korean_font():
    try:
        from matplotlib import font_manager
        names = {f.name for f in font_manager.fontManager.ttflist}
        for n in ('Malgun Gothic', 'AppleGothic', 'NanumGothic', 'Noto Sans CJK KR'):
            if n in names:
                return n
    except Exception:
        pass
    return None


def plot(results, path, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fn = _korean_font()
    if fn:
        plt.rcParams['font.family'] = [fn, 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    scen = sorted({r['scenario'] for r in results})
    fig, axes = plt.subplots(2 * len(scen), 1, figsize=(10, 4.2 * len(scen)), squeeze=False,
                             gridspec_kw=dict(height_ratios=[2, 1] * len(scen)))
    color = {'rl': '#1f6feb', 'pid': '#d97706'}
    name = {'rl': 'RL 정책', 'pid': '보드 PID'}
    for j, s in enumerate(scen):
        ax, axu = axes[2 * j, 0], axes[2 * j + 1, 0]
        labeled = set()
        ref_drawn = False
        for r in [x for x in results if x['scenario'] == s]:
            rows = r['rows']
            if not rows:
                continue
            t0 = rows[0][0]
            t = np.array([(x[0] - t0) / 1000 for x in rows])
            k = r['kind']
            lab = None if k in labeled else f'θ {name.get(k, k)}'
            alpha = 1.0 if r['run'] == 0 else 0.45
            ax.plot(t, [x[2] for x in rows], color=color.get(k, 'gray'), lw=1.4, alpha=alpha, label=lab)
            axu.plot(t, [x[1] for x in rows], color=color.get(k, 'gray'), lw=1.0, alpha=alpha,
                     label=None if k in labeled else f'PWM {name.get(k, k)}')
            labeled.add(k)
            if not ref_drawn:
                ax.plot(t, [x[6] for x in rows], color='k', ls='--', lw=1.1, label='기준 ref')
                ref_drawn = True
        r0 = [x for x in results if x['scenario'] == s][0]
        ms = ', '.join(f'{name.get(x["kind"], x["kind"])} RMS {_f(x["metrics"].get("rms"))}°'
                       for x in results if x['scenario'] == s and x['run'] == 0)
        ax.set_title(f'시나리오 {s}: home {r0["home"]:g}° → {r0["r1"]:g}° → {r0["r2"]:g}°   ({ms})', fontsize=10)
        ax.set_ylabel('θ [°]')
        ax.grid(alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        axu.set_ylabel('PWM [0–255]')
        axu.set_xlabel('시간 [s]')
        axu.grid(alpha=0.3)
        axu.legend(loc='best', fontsize=8)
    if any(r['run'] > 0 for r in results):
        title += '   (옅은 선 = 반복 2회째부터)'
    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ====================================================================== CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description='에어로펜듈럼 RL 정책 시험 (학습 없음)')
    ap.add_argument('--port', help='실제 보드 포트 (모터가 돕니다)')
    ap.add_argument('--fake', action='store_true', help='가짜 보드')
    ap.add_argument('--fast', action='store_true', help='가짜 보드 가상시간')
    ap.add_argument('--latency-ms', type=float, nargs=2, help='가짜 보드 명령 도착 지연 [ms]')
    ap.add_argument('--ckpt', required=True, help='실행 폴더, 스냅샷 폴더, checkpoint 폴더 또는 .pt 파일')
    ap.add_argument('--snapshot', help='실행 폴더의 스냅샷 이름 (best, ep00050, manual_…)')
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--hold', type=float, help='한 각도 유지 [deg]')
    g.add_argument('--refs', type=float, nargs='+', metavar='DEG', help='home r1 r2 (3개씩 여러 시나리오 가능)')
    ap.add_argument('--dur', type=float, default=20.0, help='에피소드 길이 [s] (기본 20)')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--pid', action='store_true', help='같은 시나리오를 보드 PID 로도 돌려 비교')
    ap.add_argument('--bench', action='store_true', help='BENCH 1 드라이런 (모터 핀 LOW)')
    ap.add_argument('--settle', type=float, default=3.0, help='기준 변경 뒤 정착 구간에서 뺄 시간 [s] (기본 3)')
    ap.add_argument('--no-plot', action='store_true')
    ap.add_argument('--out', help='출력 폴더 (기본 <run_dir>/play/<YYYYMMDD_HHMMSS>)')
    ap.add_argument('--quiet', action='store_true', help='이벤트 줄 출력 생략')
    args = ap.parse_args(argv)
    if args.fast and not args.fake:
        ap.error('--fast 는 --fake 와 함께')
    if not args.fake and not args.port:
        ap.error('--port COMx 또는 --fake 가 필요합니다')
    if args.repeat < 1 or args.dur <= 0:
        ap.error('--repeat ≥ 1, --dur > 0')

    pol, run_dir, meta = resolve_policy(args.ckpt, args.snapshot)
    cfg = load_run_config(os.path.join(run_dir, 'config.json'))
    cfg.episode.dur_s = float(args.dur)
    cfg.run.save_episode_csv = True
    if args.bench:
        cfg.run.bench = True
    if args.hold is not None:
        scen = [[args.hold] * 3]
    elif args.refs:
        if len(args.refs) % 3:
            ap.error('--refs 는 home r1 r2 의 3개씩')
        scen = [args.refs[i:i + 3] for i in range(0, len(args.refs), 3)]
    else:
        scen = [list(x) for x in cfg.run.eval_refs]
    lo, hi = cfg.safe.th_lo_cdeg / 100 + 5, cfg.safe.th_hi_cdeg / 100 - 5
    for s in scen:
        for x in s:
            if not lo <= x <= hi:
                ap.error(f'기준 {x}° 가 SAFE 범위 안쪽({lo:g}…{hi:g}°)이 아닙니다')
            if not cfg.episode.ref_lo - 1e-9 <= x <= cfg.episode.ref_hi + 1e-9:
                print(f'경고: 기준 {x}° 는 학습 범위 {cfg.episode.ref_lo:g}…{cfg.episode.ref_hi:g}° 밖입니다', flush=True)

    out = os.path.abspath(args.out) if args.out else os.path.join(run_dir, 'play', time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)

    from .train import make_link_factory
    factory, sim = make_link_factory(args, cfg)
    log = (lambda s: None) if args.quiet else (lambda s: print(s, flush=True))
    pl = Player(cfg, factory, out, pol, scen, repeat=args.repeat, pid=args.pid, log=log, sim=sim,
                settle_s=args.settle)
    snap = meta.get('name') if meta else None
    print(f'정책: {pol}' + (f'  (스냅샷 {snap}, 학습 {meta.get("episode")} ep, 평가 RMS {_f(meta.get("eval_rms"), 3)}°)'
                            if meta else f'  (갱신 {pl.policy_info.get("updates")})'), flush=True)
    print(f'설정: {os.path.join(run_dir, "config.json")}', flush=True)
    print(f'출력: {out}', flush=True)
    print(f'시나리오 {scen} × {args.repeat}회, 에피소드 {args.dur:g} s' + (', PID 비교' if args.pid else ''), flush=True)
    if not args.fake and not args.bench:
        print(f'주의: {args.port} 의 실제 보드에서 모터가 돕니다 (진자 주변을 비우세요, Ctrl+C = 즉시 STOP).', flush=True)

    pl.start()
    try:
        while not pl.wait(0.5):
            pass
    except KeyboardInterrupt:
        print('정지 요청 (Ctrl+C) → STOP', flush=True)
        pl.command('stop')
        try:
            if not pl.wait(10) and pl.link is not None:
                pl.link.send('STOP')
        except (KeyboardInterrupt, Exception):
            try:
                if pl.link is not None:
                    pl.link.send('STOP')
            except Exception:
                pass

    res = pl.results
    print()
    if res:
        print_table(res)
    else:
        print('끝난 에피소드가 없습니다')
    agg = aggregate(res)
    for k, a in agg.items():
        print(f'  {k.upper()} 평균: RMS {_f(a["rms"])}°, 정착 |e| {_f(a["settled_mean_abs_e"])}°, '
              f'σ(e) {_f(a["settled_std_e"])}°, OS {_f(a["overshoot_pct"], 0)}%, PWM {_f(a["mean_pwm"], 1)} '
              f'(σ {_f(a["pwm_std_settled"], 1)}), miss {a["miss"]}, FALLBACK {a["fallbacks"]}/{a["episodes"]}')
    if pl.end_reason:
        print(f'중단 이유: {pl.end_reason}', flush=True)

    summary = dict(policy=pol, run_dir=run_dir, snapshot=meta, policy_info=pl.policy_info,
                   link=getattr(pl.link, 'name', None), fw=getattr(pl.link, 'fw', None), sim=sim,
                   bench=bool(cfg.run.bench), scenarios=scen, repeat=args.repeat, dur_s=args.dur, pid=args.pid,
                   settle_s=args.settle, time=time.strftime('%Y-%m-%dT%H:%M:%S'), end_reason=pl.end_reason,
                   episodes=[{k: v for k, v in r.items() if k != 'rows'} for r in res], aggregate=agg)
    with open(os.path.join(out, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    if res and not args.no_plot:
        try:
            title = f'정책 시험: {os.path.basename(run_dir)} / {snap or os.path.basename(os.path.dirname(pol) if os.path.isfile(pol) else pol)}'
            plot(res, os.path.join(out, 'play.png'), title)
            print(f'그림: {os.path.join(out, "play.png")}', flush=True)
        except Exception as e:
            print(f'그림을 만들지 못했습니다: {e}', flush=True)
    print(f'요약: {os.path.join(out, "summary.json")}', flush=True)
    return 1 if pl.end_reason else 0


if __name__ == '__main__':
    sys.exit(main())
