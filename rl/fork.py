"""실행 분기(fork): 기존 실행의 한 시점(정책 스냅샷)에서 새 실행 폴더를 만들어 `rl.train --resume` 으로 이어 합니다.

    python -m rl.fork data/20261115_rl3 --snapshot ep00050
    python -m rl.train --port COM4 --resume data/<새 폴더>

언제 쓰나: 실행 중간 이후의 데이터가 오염됐을 때(11-15 rl3: 진동으로 자이로가 편향돼 θ 가 θacc 에서 떠내려감),
그 전의 정책으로 되돌려 다시 학습하려고. `checkpoint/` 는 덮어써서 마지막 상태만 있으므로 그대로는 되돌릴 수 없습니다.

만드는 것 (원본 실행 폴더는 읽기만 함):
- 액터: 스냅샷 `checkpoints/<이름>/actor.pt` (α 도 스냅샷 값). 액터 옵티마이저는 새로.
- 리플레이 버퍼: 원본 `checkpoint/buffer.npz` 에서 그 스냅샷까지 쌓인 전이만 (train_log 의 buffer 열로 자름).
  `--max-gap` 이 0 보다 크면 그중 모터 가동 중 θ − θacc 의 이동평균(monitor.att_gap_win_s, 기본 10 s)이 이 값을 넘는
  결정 샘플의 전이도 뺍니다 (학습기의 자세 추정 감시와 같은 기준). 에피소드 CSV 의 reward 가 있는 줄 = 전이 순서.
- 크리틱: 스냅샷에는 액터만 있으므로 원본의 마지막 체크포인트 크리틱·타깃·크리틱 옵티마이저에서 시작해,
  거른 버퍼로 크리틱만 `--critic-steps` 번 갱신해 스냅샷 액터에 다시 맞춥니다 (갱신 카운터는 스냅샷 값 유지).
- 진행 상태: 학습 에피소드·결정 수·평가 점수·학습 곡선·CSV 번호·경과 시간을 그 시점 값으로. 그 시점 이후의
  평가·기준선은 버립니다. PID 기준선은 비워서 이어 하기 시작 때 다시 잽니다(`--keep-baseline` 이면 유지).
- train_log.csv: 그 시점까지의 줄 (현재 열 형식, att_gap 은 에피소드 CSV 에서 다시 계산). 스냅샷 폴더 중 그 시점
  이전 것은 복사. 에피소드 CSV 는 복사하지 않습니다(원본에 있음). fork.json 에 출처·거른 수·크리틱 손실.
- 설정: 원본 config.json (옛 실행 호환 규칙 그대로) + `--config` (바꿀 항목만 적은 JSON).
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import re
import shutil
import sys
import time

import numpy as np

from .config import _merge, load_run_config, save_config
from .record import LOG_FIELDS, new_run_dir


def _rolling_gap(pwm, th, tha, win: int) -> np.ndarray:
    """모터 가동 샘플의 θ − θacc 뒤쪽 이동평균 (창 win 샘플). 모터 정지 샘플에서 창을 비움.
    구간 시작 뒤 win/5 샘플(10 s 창이면 2 s)까지는 θacc 잡음(σ ~4°) 때문에 평균이 튀므로 그 시점의 값으로 채움
    (학습기 감시는 에피소드·복귀를 이어서 보므로 에피소드 시작에서도 창이 차 있음)."""
    n = len(th)
    out = np.zeros(n)
    m = max(1, win // 5)
    buf, s, start = [], 0.0, None
    segs = []
    for i, (u, a, b) in enumerate(zip(pwm, th, tha)):
        if u == 0:
            if start is not None:
                segs.append((start, i))
            buf, s, start = [], 0.0, None
            continue
        if start is None:
            start = i
        g = a - b
        buf.append(g)
        s += g
        if len(buf) > win:
            s -= buf.pop(0)
        out[i] = s / len(buf)
    if start is not None:
        segs.append((start, n))
    for a, b in segs:
        k = min(a + m - 1, b - 1)
        out[a:k] = out[k]
    return out


def _read_episode(path):
    pwm, th, tha, rew = [], [], [], []
    with open(path, encoding='utf-8') as f:
        rd = csv.DictReader(line for line in f if not line.startswith('#'))
        for r in rd:
            pwm.append(int(float(r['pwm'])))
            th.append(float(r['theta_deg']))
            tha.append(float(r['theta_acc_deg']))
            rew.append(r.get('reward', '') != '')
    return np.array(pwm), np.array(th), np.array(tha), np.array(rew, bool)


def fork(src: str, snapshot: str, out: str | None = None, max_gap: float = 3.0, critic_steps: int = 10000,
         overrides: dict | None = None, keep_baseline: bool = False, log=print) -> dict:
    import torch
    from .env import obs_dim
    from .sac import SAC, load_actor_state

    cfg = load_run_config(os.path.join(src, 'config.json'))
    if overrides:
        _merge(cfg, overrides, '')
    snap_dir = os.path.join(src, 'checkpoints', snapshot)
    with open(os.path.join(snap_dir, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    actor_sd, info = load_actor_state(snap_dir)
    od = obs_dim(cfg)
    if meta.get('obs_dim', od) != od:
        raise SystemExit(f'스냅샷 관측 차원 {meta.get("obs_dim")} ≠ 설정 {od}')
    ep_snap, cut_n = int(meta['episode']), int(meta['decisions'])

    with open(os.path.join(src, 'train_log.csv'), encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))
    cut = None
    for i, r in enumerate(rows):
        if r['kind'] in ('train', 'eval') and r.get('buffer') and int(r['buffer']) == cut_n:
            cut = i
    if cut is None:
        raise SystemExit(f'train_log 에서 버퍼 {cut_n} 인 줄을 찾지 못했습니다 (스냅샷 {snapshot})')
    keep = rows[:cut + 1]
    files = sorted(glob.glob(os.path.join(src, 'episodes', 'ep*.csv')))
    names = {int(re.search(r'ep(\d+)_', os.path.basename(p)).group(1)): p for p in files}
    if any(k not in names for k in range(1, cut + 2)):
        raise SystemExit('에피소드 CSV 가 train_log 줄과 맞지 않습니다 (번호 빠짐)')

    z = np.load(os.path.join(src, 'checkpoint', 'buffer.npz'))
    if int(z['n']) < cut_n or int(z['n']) > int(z['size']):
        raise SystemExit(f'원본 버퍼 {int(z["n"])} 개 < 스냅샷 결정 수 {cut_n} (또는 버퍼가 한 바퀴 돎)')
    win = max(1, int(round(cfg.monitor.att_gap_win_s / cfg.ts_s)))
    mask = np.ones(cut_n, bool)
    prev, gaps = 0, []
    for i, r in enumerate(keep):
        pwm, th, tha, rew = _read_episode(names[i + 1])
        on = pwm > 0
        gaps.append(float(np.mean(th[on] - tha[on])) if on.any() else None)
        if r['kind'] not in ('train', 'eval'):
            continue
        b = int(r['buffer'])
        k = b - prev
        if k > 0 and max_gap > 0:
            g = _rolling_gap(pwm, th, tha, win)[rew]
            if len(g) == 0:
                g = np.zeros(1)
            if len(g) < k:                      # 종단 전이(FALLBACK)는 reward 줄이 없음 → 마지막 값
                g = np.concatenate([g, np.full(k - len(g), g[-1])])
            mask[prev:b] = np.abs(g[:k]) <= max_gap
        prev = b
    if prev != cut_n:
        raise SystemExit(f'에피소드 전이 합 {prev} ≠ 스냅샷 결정 수 {cut_n}')
    kept = int(mask.sum())

    sac = SAC(od, 1, cfg.sac)
    last = torch.load(os.path.join(src, 'checkpoint', 'sac.pt'), weights_only=False)
    sac.critic.load_state_dict(last['critic'])
    sac.target.load_state_dict(last['target'])
    sac.opt_critic.load_state_dict(last['opt_critic'])
    sac.actor.load_state_dict(actor_sd)
    alpha = float(info.get('alpha') or meta.get('alpha') or cfg.sac.init_alpha)
    with torch.no_grad():
        sac.log_alpha.copy_(torch.tensor(math.log(alpha)))
    sac.updates = int(meta.get('updates') or 0)
    B = sac.buffer
    for name in ('o', 'a', 'r', 'o2', 'd'):
        getattr(B, name)[:kept] = z[name][:cut_n][mask]
    B.n = kept
    B.ptr = kept % B.size
    losses = []
    t0 = time.time()
    step = 1000
    for done in range(0, critic_steps, step):
        losses.append(round(sac.update_critic(min(step, critic_steps - done)), 5))
    sac.sync_policy()
    log(f'크리틱 재적응 {critic_steps} 회 ({time.time() - t0:.0f} s), q 손실 1000 회마다: {losses}')

    ex = last.get('extra', {})
    evals = [e for e in ex.get('eval_scores', []) if e.get('episode', 0) < ep_snap]
    bh = [b for b in ex.get('baseline_history', []) if b.get('episode', 0) < ep_snap]
    state = dict(train_episodes=ep_snap, eval_rounds=len(evals), decisions=cut_n,
                 fallbacks=int(float(keep[-1].get('fallbacks_total') or 0)),
                 aborts=sum(r['status'] == 'abort' for r in keep), misses=sum(int(float(r.get('miss') or 0)) for r in keep
                                                                          if r['kind'] != 'baseline'),
                 pauses=0, eval_episodes=sum(r['kind'] == 'eval' for r in keep), elapsed=float(keep[-1]['board_s']),
                 baseline=(bh[-1] if (keep_baseline and bh) else None), baseline_history=bh, eval_scores=evals,
                 curve=list(ex.get('curve', []))[:cut + 1], ep_index=cut + 1,
                 manual_evals=sum(1 for e in evals if e.get('manual')))

    out = out or new_run_dir()
    os.makedirs(os.path.join(out, 'episodes'), exist_ok=True)
    save_config(cfg, os.path.join(out, 'config.json'))
    ck = os.path.join(out, 'checkpoint')
    sac.save(ck, extra=state)
    with open(os.path.join(ck, 'trainer.json'), 'w', encoding='utf-8') as f:
        json.dump({k: v for k, v in state.items() if k != 'curve'}, f, ensure_ascii=False, indent=1)
    with open(os.path.join(out, 'train_log.csv'), 'w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=LOG_FIELDS, extrasaction='ignore')
        w.writeheader()
        for r, g in zip(keep, gaps):
            r = dict(r)
            r['att_gap'] = '' if g is None else round(g, 3)
            w.writerow(r)
    snaps = []
    for d in sorted(glob.glob(os.path.join(src, 'checkpoints', '*'))):
        try:
            with open(os.path.join(d, 'meta.json'), encoding='utf-8') as f:
                m = json.load(f)
        except (OSError, ValueError):
            continue
        if int(m.get('episode', 1 << 30)) <= ep_snap and int(m.get('decisions', 1 << 62)) <= cut_n:
            shutil.copytree(d, os.path.join(out, 'checkpoints', os.path.basename(d)), dirs_exist_ok=True)
            snaps.append(os.path.basename(d))
    prov = dict(src=os.path.abspath(src), snapshot=snapshot, snapshot_meta=meta, cut_row=cut, cut_transitions=cut_n,
                max_gap_deg=max_gap, gap_window_s=cfg.monitor.att_gap_win_s, kept_transitions=kept,
                dropped_transitions=cut_n - kept, critic_steps=critic_steps, critic_q_loss=losses,
                alpha=alpha, keep_baseline=keep_baseline, overrides=overrides or {}, copied_snapshots=snaps,
                time=time.strftime('%Y-%m-%dT%H:%M:%S'))
    with open(os.path.join(out, 'fork.json'), 'w', encoding='utf-8') as f:
        json.dump(prov, f, ensure_ascii=False, indent=1)
    with open(os.path.join(out, 'events.log'), 'a', encoding='utf-8') as f:
        f.write(f'{time.strftime("%Y-%m-%dT%H:%M:%S")} 분기: {src} 의 스냅샷 {snapshot} (학습 {ep_snap} ep, 결정 {cut_n}) '
                f'→ 버퍼 {kept}/{cut_n} (θ−θacc > {max_gap:g}° {cut_n - kept} 개 제외), 크리틱 재적응 {critic_steps} 회, '
                f'PID 기준선 {"유지" if keep_baseline else "다시 잼"}\n')
    return dict(out=out, kept=kept, cut=cut_n, dropped=cut_n - kept, losses=losses, episode=ep_snap, snapshots=snaps)


def main(argv=None):
    ap = argparse.ArgumentParser(description='실행 분기: 기존 실행의 정책 스냅샷 시점에서 새 실행 폴더를 만듭니다')
    ap.add_argument('src', help='원본 실행 폴더 (data/<날짜>_rlN)')
    ap.add_argument('--snapshot', required=True, help='checkpoints/ 아래 스냅샷 이름 (예: ep00050, best)')
    ap.add_argument('--out', help='새 실행 폴더 (기본: data/<오늘>_rl<다음 번호>)')
    ap.add_argument('--max-gap', type=float, default=3.0,
                    help='θ − θacc 이동평균이 이 값[°]을 넘는 결정 샘플의 전이 제외 (0 = 거르지 않음, 기본 3)')
    ap.add_argument('--critic-steps', type=int, default=10000, help='크리틱 재적응 갱신 수 (기본 10000)')
    ap.add_argument('--config', help='바꿀 설정만 적은 JSON (예: {"rest": {"max_on_s": 150}})')
    ap.add_argument('--keep-baseline', action='store_true', help='PID 기준선을 다시 재지 않고 유지')
    args = ap.parse_args(argv)
    ov = None
    if args.config:
        with open(args.config, encoding='utf-8') as f:
            ov = json.load(f)
    r = fork(args.src, args.snapshot, args.out, args.max_gap, args.critic_steps, ov, args.keep_baseline)
    print(f'새 실행 폴더: {r["out"]}  (학습 {r["episode"]} ep 시점, 버퍼 {r["kept"]}/{r["cut"]}, '
          f'제외 {r["dropped"]}, 스냅샷 복사 {r["snapshots"]})')
    print(f'이어 하기: .venv\\Scripts\\python.exe -m rl.train --port COM4 --resume {r["out"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
