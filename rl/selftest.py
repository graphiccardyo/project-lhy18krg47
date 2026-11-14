"""하드웨어 없이 하는 자체 시험.

    python -m rl.selftest            전부 (몇 분)
    python -m rl.selftest --quick    짧은 학습 시험 생략

1. 프로토콜: bench_test 전 항목을 가짜 보드(가상시간)로
2. SAC: 버퍼 저장/복원, numpy 추론 = torch 액터, 체크포인트 왕복, 추론 시간, 작은 문제 학습
3. env: 합성 D 줄로 결정 위상·실제 증분·종단/절단
4. 트레이너: 가상시간 짧은 학습이 실제로 나아지는지(첫 평가 대비 25%↓, PID보다 낮음), 일시정지/재개(사용자·고장 주입), --resume
5. 스냅샷: 주기·best·수동(snapshot 명령, 일시정지 중 즉시), eval 명령(정기 일정과 별도), 옛 설정·체크포인트 호환
6. rl.play: 가짜 보드로 --refs --pid / --hold, 지표·CSV·그림, STOP 으로 끝남, ABORT 중단
7. 정상상태: steady_metrics, 정밀 보상·미세 오차 관측, 옛 실행 설정 호환(관측 7차원 유지), 주기 PID 기준선,
   train_log ss_* 열, 이어 하기 때 옛 머리줄 유지
시험 실행 폴더는 임시 폴더에 만들고 지웁니다.
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
import tempfile
import time

import numpy as np

RESULTS = []


def check(name, ok, detail=''):
    RESULTS.append((name, bool(ok), detail))
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f' — {detail}' if detail else ''), flush=True)
    return ok


def t_protocol():
    print('== 1. 프로토콜 (bench_test, 가짜 보드 가상시간)', flush=True)
    from .bench_test import run_bench
    from .config import PlantCfg
    from .fake_board import FakeBoard
    from .link import VirtualLink
    lines = []
    ok, b = run_bench(VirtualLink(FakeBoard(PlantCfg(), seed=5)), log=lines.append, quick=True)
    req = [r for r in b.results if r[3]]
    fails = [f'{r[0]} — {r[2]}' for r in req if not r[1]]
    check(f'bench_test 필수 {sum(r[1] for r in req)}/{len(req)} 통과', ok, '; '.join(fails[:3]))


def t_sac(tmp):
    print('== 2. SAC 단위 시험', flush=True)
    import torch
    from .config import SacCfg
    from .sac import SAC
    cfg = SacCfg(hidden=64, batch=64, buffer_size=5000, warmup_steps=0, seed=3)
    s = SAC(3, 1, cfg)
    rng = np.random.default_rng(0)
    for _ in range(300):
        s.buffer.add(rng.normal(size=3), rng.uniform(-1, 1, 1), rng.normal(), rng.normal(size=3), 0.0)
    s.update(5)
    s.sync_policy()
    o = rng.normal(size=3).astype(np.float32)
    with torch.no_grad():
        mu, ls = s.actor(torch.from_numpy(o)[None])
    a_np = s.policy.act(o, True)
    a_t = math.tanh(mu[0, 0].item())
    check('numpy 추론 = torch 액터 (결정적)', abs(a_t - float(a_np[0])) < 1e-5, f'{a_t:.6f} vs {float(a_np[0]):.6f}')
    d = os.path.join(tmp, 'ck')
    s.save(d, extra={'x': 1})
    s2 = SAC(3, 1, cfg)
    ex = s2.load(d)
    a2 = s2.policy.act(o, True)
    ok = (ex.get('x') == 1 and s2.buffer.n == 300 and s2.updates == s.updates and abs(a2[0] - a_np[0]) < 1e-9
          and np.allclose(s2.buffer.o[:300], s.buffer.o[:300]) and abs(s2.alpha - s.alpha) < 1e-9
          and len(s2.opt_critic.state_dict()['state']) > 0)
    check('체크포인트 왕복 (망·버퍼·옵티마이저·α·카운터)', ok)
    tt = []
    for _ in range(3000):
        t0 = time.perf_counter()
        s.policy.act(o)
        tt.append(time.perf_counter() - t0)
    check('numpy 추론 시간 < 0.2 ms (중앙값)', np.median(tt) < 2e-4,
          f'중앙 {np.median(tt) * 1e6:.1f} µs, p99 {np.percentile(tt, 99) * 1e6:.1f} µs')

    # 작은 문제: 적분기 s' = s + 0.2a, r = −s² → 결정적 정책이 0 으로 몰아야 함
    cfg = SacCfg(hidden=64, batch=128, buffer_size=20000, gamma=0.9, seed=1, init_alpha=0.1)
    ag = SAC(1, 1, cfg)
    rng = np.random.default_rng(1)
    x = 0.0
    for step in range(3000):
        if step % 30 == 0:
            x = rng.uniform(-1, 1)
        a = rng.uniform(-1, 1) if step < 500 else float(ag.policy.act(np.array([x], np.float32))[0])
        x2 = float(np.clip(x + 0.2 * a, -1.5, 1.5))
        ag.buffer.add([x], [a], -x2 * x2, [x2], 0.0)
        x = x2
        if step >= 500:
            ag.update(1)
            if step % 50 == 0:
                ag.sync_policy()
    ag.sync_policy()
    finals = []
    for x0 in (-0.9, -0.4, 0.5, 1.0):
        x = x0
        for _ in range(20):
            x = float(np.clip(x + 0.2 * ag.policy.act(np.array([x], np.float32), True)[0], -1.5, 1.5))
        finals.append(abs(x))
    check('작은 문제 학습 (적분기를 0 으로)', max(finals) < 0.1, f'20스텝 뒤 |x| 최대 {max(finals):.3f}')


def t_env():
    print('== 3. env (합성 D 줄)', flush=True)
    from .config import Config
    from .env import EpisodeEnv
    from .link import parse_line
    cfg = Config()
    cfg.env.act_pow = 1.0          # 이 시험은 선형 행동 곡선 기준 (a 0.5 → du 5)
    env = EpisodeEnv(cfg)
    env.arm(lambda o: 0.5, 2, 10)
    sent = []
    # 대기 중 줄(7필드)은 세지 않아야 함
    env.on_D(parse_line('D,0,140,2000,0,2000,0,2000'))
    u = 140
    for n in range(10):
        if n in (2, 4, 8):        # n = 6 은 누락(miss) 흉내 → 그 전이의 실제 증분은 0
            u = min(210, u + 5)
        cmd = env.on_D(parse_line(f'D,{10 * n},{u},{2000 + n},0,2000,1,2500,0'))
        sent.append(cmd)
    check('결정 샘플에서만 A (n = 0, 2, 4, …)', sent[0] == 'A 5' and sent[1] is None and sent[2] == 'A 5'
          and sum(c is not None for c in sent) == 5, str(sent))
    check('전이 수 = 결정 − 1, 실제 증분 a_eff = Δu/du_max (누락은 0)', len(env.trans) == 4 and
          [round(t[1], 3) for t in env.trans] == [0.5, 0.5, 0.0, 0.5], f'{[round(t[1], 2) for t in env.trans]}')
    o = env.trans[-1][3]
    check('관측: 오차·PWM·직전 행동', abs(o[0] - (25.0 - 20.08) / cfg.env.th_scale) < 1e-4 and
          abs(o[4] - (u - cfg.env.u_center) / cfg.env.u_scale) < 1e-5 and abs(o[5] - 0.5) < 1e-6, str(np.round(o, 3)))
    env.terminal('FALLBACK')
    check('FALLBACK → 종단 전이 (done=1, 벌점)', env.trans[-1][4] == 1.0 and
          abs(env.trans[-1][2] + cfg.env.reward_scale * cfg.env.fallback_penalty) < 1e-9,
          f'r={env.trans[-1][2]:.2f}')
    env.arm(lambda o: 0.0, 2, 10)
    for n in range(6):
        env.on_D(parse_line(f'D,{10 * n},140,2000,0,2000,1,2500,0'))
    env.finish()
    check('DONE → 절단 (마지막 결정은 전이로 만들지 않음, done=0)', len(env.trans) == 2 and
          all(t[4] == 0.0 for t in env.trans))


def _virtual_trainer(tmp, name, episodes, overrides=None, resume_dir=None, seed=11):
    from .config import Config
    from .fake_board import FakeBoard
    from .link import VirtualLink
    from .trainer import Trainer
    cfg = Config()
    cfg.stop.max_episodes = episodes
    cfg.plant.seed = seed
    for path, v in (overrides or {}).items():
        obj = cfg
        parts = path.split('.')
        for p in parts[:-1]:
            obj = getattr(obj, p)
        setattr(obj, parts[-1], v)
    board = FakeBoard(cfg.plant, seed=seed)
    vl = VirtualLink(board)
    run_dir = resume_dir or os.path.join(tmp, name)
    os.makedirs(run_dir, exist_ok=True)
    tr = Trainer(cfg, lambda: vl, run_dir, resume=bool(resume_dir), autostart=True, log=None, sim=True)
    return tr, board, run_dir


def t_trainer(tmp, quick):
    print('== 4. 트레이너 (가상시간)', flush=True)
    # 일시정지/재개: 사용자 명령, 고장 주입(overruns), ABORT, 그리고 --resume
    tr, board, rd = _virtual_trainer(tmp, 'pause', 6, {'run.baseline_at_start': False, 'sac.warmup_steps': 500,
                                                       'run.eval_every': 3, 'rest.max_on_s': 30.0, 'rest.rest_s': 30.0})
    tr.start()

    def wait_state(states, timeout=60):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = tr.status()
            if s['state'] in states:
                return s
            time.sleep(0.005)
        return tr.status()

    s = wait_state(('episode',))
    tr.command('pause')
    s = wait_state(('paused',))
    check('사용자 pause → 일시정지', s['state'] == 'paused' and s['paused_reason'] == '사용자 일시정지',
          s['paused_reason'])
    tr.command('resume')
    s = wait_state(('episode',))
    board.inject_overruns = 50
    s = wait_state(('paused',))
    check('S 줄 overruns 급증 → 자동 일시정지', s['state'] == 'paused' and 'overruns' in (s['paused_reason'] or ''),
          s['paused_reason'])
    tr.command('resume')
    s = wait_state(('episode',))
    board.alim = 500      # 각도 한계를 5° 로 → 에피소드 중 EVT ABORT ANGLE_LIMIT
    s = wait_state(('paused',))
    check('EVT ABORT → 자동 일시정지 (모터 STOP)', s['state'] == 'paused' and 'ABORT' in (s['paused_reason'] or '')
          and board.u_out == 0, s['paused_reason'])
    board.alim = 10000
    tr.command('resume')
    s = wait_state(('resting',), timeout=120)
    check('모터 가동 누적 → 휴식(STOP)', s['state'] == 'resting' and s['rest_left_s'] is not None,
          f'rest_left {s["rest_left_s"]}')
    tr.wait(300)
    s = tr.status()
    check('max_episodes 에서 자동 종료 + 체크포인트', s['state'] == 'done' and s['episode'] == 6 and
          os.path.exists(os.path.join(rd, 'checkpoint', 'sac.pt')), f'{s["state"]}, {s["episode"]} ep')
    import json
    try:
        js = json.dumps(s, ensure_ascii=False)
        keys = ('state', 'episode', 'recent_episodes', 'learning_curve', 'eval_scores', 'baseline', 'counts',
                'mean_pwm', 'remaining_s', 'motor_on_s', 'sac', 'log')
        check('status() 는 JSON 직렬화 가능, 문서의 키 포함', all(k in s for k in keys), f'{len(js)} 바이트')
    except TypeError as e:
        check('status() 는 JSON 직렬화 가능', False, str(e))
    n_csv = len(os.listdir(os.path.join(rd, 'episodes')))
    check('기록: 에피소드 CSV·train_log·config.json', n_csv >= 6 and os.path.exists(os.path.join(rd, 'train_log.csv'))
          and os.path.exists(os.path.join(rd, 'config.json')), f'CSV {n_csv}개')
    with open(os.path.join(rd, 'episodes', sorted(os.listdir(os.path.join(rd, 'episodes')))[0]), encoding='utf-8') as f:
        head = [f.readline() for _ in range(3)]
    check('CSV 헤더 형식 (type=RL, # pid, 열 끝 ref_deg,miss,du,reward)', 'type=RL' in head[0] and
          head[1].startswith('# pid;') and head[2].strip().endswith('ref_deg,miss,du,reward'), head[2].strip())
    dec0, upd0 = s['decisions'], s['updates']
    tr2, _, _ = _virtual_trainer(tmp, 'pause', 8, {'run.baseline_at_start': False, 'sac.warmup_steps': 500,
                                                   'run.eval_every': 3}, resume_dir=rd)
    st = tr2.status()
    check('--resume: 카운터·버퍼·갱신 수 복원', st['episode'] == 6 and st['decisions'] == dec0 and st['updates'] == upd0
          and st['buffer'] == dec0, f'ep {st["episode"]}, 결정 {st["decisions"]}, 갱신 {st["updates"]}')
    tr2.start()
    tr2.wait(300)
    check('--resume 뒤 이어서 학습', tr2.status()['episode'] == 8 and tr2.status()['state'] == 'done')

    t_snapshots(tmp)
    t_play(tmp)
    t_steady(tmp)

    if quick:
        return
    # 짧은 학습이 나아지는지: 무작위 워밍업 직후 평가 vs 40 에피소드 뒤
    print('  (짧은 학습: 가상시간 40 에피소드, 몇 분 걸림)', flush=True)
    t0 = time.time()
    tr, board, rd = _virtual_trainer(tmp, 'learn', 40, {'run.eval_every': 10, 'sac.warmup_steps': 2500}, seed=21)
    tr.start()
    tr.wait(3600)
    s = tr.status()
    ev = [e['rms_mean'] for e in s['eval_scores']]
    base = s['baseline']['rms_mean'] if s['baseline'] else float('nan')
    # 11-14: 기준 ≤ 30°·PWM 255·정밀 보상으로 첫 평가부터 낮아져(6.5 → 3.6°) 상대 감소 기준을 40 → 25% 로, PID 비교 추가
    check('짧은 학습으로 평가 RMS 개선 (첫 평가 대비 25% 이상 감소, PID 기준선보다 낮음)',
          len(ev) >= 3 and ev[-1] < 0.75 * ev[0] and ev[-1] < base,
          f'평가 RMS {ev}, PID {base:.3f}°, 가상 {s["elapsed_s"] / 60:.1f} 분 / 실제 {(time.time() - t0) / 60:.1f} 분')


def _wait(tr, cond, timeout=120):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = tr.status()
        if cond(s):
            return s
        time.sleep(0.005)
    return tr.status()


SNAP_RUN = {}


def t_snapshots(tmp):
    print('== 5. 스냅샷·snapshot/eval 명령 (가상시간)', flush=True)
    import json
    import torch  # noqa: F401  (sac 로드)
    from .config import from_dict, to_dict
    from .sac import SAC, load_actor_state
    tr, board, rd = _virtual_trainer(tmp, 'snap', 6, {'run.baseline_at_start': False, 'sac.warmup_steps': 500,
                                                      'run.eval_every': 3, 'run.snapshot_every': 2})
    tr.start()
    _wait(tr, lambda s: s['state'] == 'episode')
    tr.command('pause')
    s = _wait(tr, lambda s: s['state'] == 'paused')
    tr.command('snapshot')
    s = _wait(tr, lambda s: any(m['kind'] == 'manual' for m in s['snapshots']), timeout=10)
    check('일시정지 중 snapshot → 바로 저장', s['state'] == 'paused' and any(m['kind'] == 'manual' for m in s['snapshots']),
          str([m['name'] for m in s['snapshots']]))
    tr.command('eval')
    s = tr.status()
    tr.command('resume')
    s = _wait(tr, lambda s: s['episode'] >= 1)
    ok_cmd = tr.command('snapshot')[0] and tr.command('eval')[0] and not tr.command('bogus')[0]
    tr.wait(300)
    s = tr.status()
    names = [m['name'] for m in s['snapshots']]
    check('명령 snapshot·eval 받음 (모르는 명령은 거부)', ok_cmd)
    check('주기 스냅샷 ep00002/4/6 + best + manual 2개', all(f'ep0000{k}' in names for k in (2, 4, 6)) and 'best' in names
          and sum(n.startswith('manual_') for n in names) == 2, str(names))
    man = [e for e in s['eval_scores'] if e.get('manual')]
    reg = [e for e in s['eval_scores'] if not e.get('manual')]
    check('수동 eval 은 정기 일정과 별도 (정기 ep 3, 수동 2회)', len(man) == 2 and [e['episode'] for e in reg] == [3]
          and s['eval_rounds'] == 3, f'정기 {[e["episode"] for e in reg]}, 수동 {[e["episode"] for e in man]}')
    best = min(e['rms_mean'] for e in s['eval_scores'])
    check('best = 평가 최소 RMS, status.best_rms 일치', abs((s['best_rms'] or -1) - best) < 1e-9,
          f'best_rms {s["best_rms"]}, 평가 {[e["rms_mean"] for e in s["eval_scores"]]}')
    d6 = os.path.join(rd, 'checkpoints', 'ep00006')
    with open(os.path.join(d6, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    sz = os.path.getsize(os.path.join(d6, 'actor.pt'))
    check('스냅샷 폴더 = actor.pt + meta.json (버퍼 없음, 작음)', sorted(os.listdir(d6)) == ['actor.pt', 'meta.json']
          and sz < 200_000 and all(k in meta for k in ('episode', 'decisions', 'updates', 'eval_rms', 'time', 'kind')),
          f'actor.pt {sz} B, meta {sorted(meta)}')
    sd, info = load_actor_state(d6)
    s2 = SAC(tr.sac.obs_dim, 1, tr.cfg.sac)
    s2.load_policy(d6)
    o = np.random.default_rng(0).normal(size=tr.sac.obs_dim)
    check('ep00006 스냅샷 정책 = 학습 끝 정책 (결정적 행동)', abs(float(s2.policy.act(o, True)[0]) -
                                                   float(tr.sac.policy.act(o, True)[0])) < 1e-9
          and info['updates'] == s['updates'], f'갱신 {info["updates"]} / {s["updates"]}')
    # 호환: 옛 config(새 키 없음)·옛 체크포인트(manual_evals 없음)
    d = to_dict(tr.cfg)
    d['run'].pop('snapshot_every')
    cfg_old = from_dict(d)
    ck = os.path.join(rd, 'checkpoint', 'sac.pt')
    st = torch.load(ck, weights_only=False)
    st['extra'].pop('manual_evals', None)
    torch.save(st, ck)
    tr3, _, _ = _virtual_trainer(tmp, 'snap', 6, {'run.baseline_at_start': False}, resume_dir=rd)
    st3 = tr3.status()
    check('호환: 옛 config 키 없음 → snapshot_every 50, 옛 체크포인트 → manual_evals 0, 스냅샷 목록 다시 읽음',
          cfg_old.run.snapshot_every == 50 and tr3.manual_evals == 0 and st3['episode'] == 6
          and len(st3['snapshots']) == len(names) and st3['best_rms'] == s['best_rms'])
    tr3.rec.close()
    SNAP_RUN['dir'] = rd


def t_play(tmp):
    print('== 6. rl.play (가짜 보드 가상시간)', flush=True)
    import json
    from .config import load_config
    from .fake_board import FakeBoard
    from .link import VirtualLink
    from .play import Player, main as play_main, resolve_policy
    rd = SNAP_RUN.get('dir')
    if not rd:
        check('rl.play 시험 (스냅샷 실행 폴더 필요)', False)
        return
    out = os.path.join(tmp, 'play1')
    rc = play_main(['--fake', '--fast', '--ckpt', rd, '--snapshot', 'best', '--refs', '20', '30', '22', '--pid',
                    '--dur', '6', '--out', out, '--quiet'])
    with open(os.path.join(out, 'summary.json'), encoding='utf-8') as f:
        sm = json.load(f)
    eps = sm['episodes']
    keys = ('rms', 'settled_mean_abs_e', 'settled_std_e', 'overshoot_pct', 'mean_pwm', 'pwm_std_settled')
    check('play --refs --pid: RL·PID 에피소드, 지표, CSV, 그림', rc == 0 and [e['kind'] for e in eps] == ['rl', 'pid']
          and all(e['metrics'].get(k) is not None for e in eps for k in keys)
          and len(os.listdir(os.path.join(out, 'episodes'))) == 2 and os.path.exists(os.path.join(out, 'play.png'))
          and not os.path.exists(os.path.join(out, 'checkpoint')),
          ', '.join(f'{e["kind"]} RMS {e["metrics"]["rms"]:.2f}° OS {e["metrics"]["overshoot_pct"]:.0f}%' for e in eps))
    with open(os.path.join(out, 'episodes', sorted(os.listdir(os.path.join(out, 'episodes')))[0]), encoding='utf-8') as f:
        head = [f.readline() for _ in range(3)]
    check('play CSV = 학습기 형식', 'type=RL' in head[0] and 'mode=play' in head[0] and head[1].startswith('# pid;')
          and head[2].strip().endswith('ref_deg,miss,du,reward'))
    pol, rdir, meta = resolve_policy(os.path.join(rd, 'checkpoints', 'ep00002'))
    pol2, _, meta2 = resolve_policy(rd)
    check('--ckpt 해석: 스냅샷 폴더 → 위쪽 config.json, 실행 폴더 → checkpoint/',
          os.path.samefile(rdir, rd) and meta['name'] == 'ep00002' and pol2.endswith('checkpoint') and meta2 is None)
    # 직접 Player: STOP 으로 끝나는지, ZERO 를 안 보내는지, ABORT 에서 멈추는지
    cfg = load_config(os.path.join(rd, 'config.json'))
    cfg.episode.dur_s = 4.0
    board = FakeBoard(cfg.plant, seed=3)
    vl = VirtualLink(board)
    sent = []
    orig = vl.send

    def spy(line):
        sent.append(line)
        return orig(line)
    vl.send = spy
    pl = Player(cfg, lambda: vl, os.path.join(tmp, 'play2'), pol, [[25.0, 25.0, 25.0]], log=None, sim=True)
    pl.start()
    pl.wait(120)
    vl.hb_enabled = False
    for _ in range(20):          # 가상시간: 마지막 STOP 이 보드에 닿도록 몇 틱 진행
        vl._advance()
    m = pl.results[0]['metrics'] if pl.results else {}
    check('play --hold: 끝나면 STOP, 모터 0, ZERO 없음, 갱신 없음', pl.end_reason is None and sent[-1] == 'STOP'
          and board.u_out == 0 and not any(x.startswith('ZERO') for x in sent) and pl.sac.updates == 0
          and pl.sac.buffer.n == 0, f'RMS {m.get("rms", float("nan")):.2f}°, 보낸 줄 {len(sent)}개')
    board2 = FakeBoard(cfg.plant, seed=4)
    vl2 = VirtualLink(board2)
    pl2 = Player(cfg, lambda: vl2, os.path.join(tmp, 'play3'), pol, [[25.0, 30.0, 22.0]], log=None, sim=True)
    orig_home = pl2._home

    def home_then_lower_limit(h):
        orig_home(h)
        board2.alim = 500          # 에피소드 시작 직전 각도 한계 5° → EVT ABORT
    pl2._home = home_then_lower_limit
    pl2.start()
    pl2.wait(120)
    check('play 중 EVT ABORT → 중단·STOP·모터 0', pl2.end_reason is not None and 'ABORT' in pl2.end_reason
          and board2.u_out == 0, pl2.end_reason)


def t_steady(tmp):
    print('== 7. 정상상태 지표·보상·호환 (가상시간)', flush=True)
    import csv
    from .config import Config, from_dict, to_dict
    from .env import EpisodeEnv, obs_dim
    from .record import LOG_FIELDS, Recorder
    from .trainer import steady_metrics
    rows = [[i * 10, 100 + (i % 2), 20.0 + (0.5 if i < 500 else -0.3), 0, 0, 1, 20.0] for i in range(1000)]
    m = steady_metrics(rows, 500, 150)
    check('steady_metrics: 창 평균 e(부호), 평균 |e|, PWM σ', m == dict(ss_e1=-0.5, ss_e2=0.3, ss_abs=0.4, ss_pwm_sd=0.5),
          str(m))
    m2 = steady_metrics(rows[:300], 500, 150)
    check('steady_metrics: 스텝 전에 끝난 에피소드 → 창 1 만', m2['ss_e1'] == -0.5 and m2['ss_e2'] is None, str(m2))
    cfg = Config()
    env = EpisodeEnv(cfg)
    r = [env._reward(e, 0.0, 0.0) for e in (0.0, 0.2, 0.5, 1.0, 8.0)]
    old = Config()
    old.env.w_fine = 0.0
    r0 = [EpisodeEnv(old)._reward(e, 0.0, 0.0) for e in (0.5, 8.0)]
    check('정밀 보상: 단조 감소, 0.5° 벌점 비중 ↑ (8°/0.5° 비 < 20, 옛 보상은 > 50)',
          all(a > b for a, b in zip(r, r[1:])) and r[4] / r[2] < 20 and r0[1] / r0[0] > 50,
          f'새 {[round(x, 4) for x in r]}, 비 {r[4] / r[2]:.1f} (옛 {r0[1] / r0[0]:.1f})')
    d = to_dict(cfg)
    for k in ('obs_efine', 'w_fine', 'efine_scale', 'fine_scale'):
        d['env'].pop(k)
    d['run'].pop('baseline_every')
    c_leg, c_new = from_dict(d, legacy=True), from_dict(d)
    check('옛 실행 config(키 없음) → 관측 7차원·정밀 항 0·기준선 시작 때만, --config 는 새 기본값 8차원',
          obs_dim(c_leg) == 7 and c_leg.env.w_fine == 0.0 and c_leg.run.baseline_every == 0 and obs_dim(c_new) == 8,
          f'옛 {obs_dim(c_leg)}, 새 {obs_dim(c_new)}')
    check('기본값: PWM 상한 255 (LIM = u_hi), 기준 ≤ 30°, 평가 기준 ≤ 30°',
          cfg.lim.pwm_max == 255 and cfg.rl.u_hi == 255 and cfg.episode.ref_hi == 30.0
          and max(max(x) for x in cfg.run.eval_refs) <= 30.0)
    # 주기 기준선 + ss 기록
    tr, board, rd = _virtual_trainer(tmp, 'steady', 5, {'sac.warmup_steps': 500, 'run.eval_every': 2,
                                                        'run.baseline_every': 2}, seed=31)
    tr.start()
    tr.wait(600)
    s = tr.status()
    bh = s['baseline_history']
    check('주기 PID 기준선: 시작 + 정기 평가 2회마다 (ep 0, 4)', [b['episode'] for b in bh] == [0, 4]
          and s['baseline'] == bh[-1] and all(b['ss_mean'] is not None for b in bh), str([b['episode'] for b in bh]))
    check('평가 점수에 정상상태 |e| (ss_mean, ss_per_scenario)', s['eval_scores'] and all(
        e['ss_mean'] is not None and len(e['ss_per_scenario']) == 3 for e in s['eval_scores']),
          str([e.get('ss_mean') for e in s['eval_scores']]))
    with open(os.path.join(rd, 'train_log.csv'), encoding='utf-8') as f:
        lr = list(csv.DictReader(f))
    check('train_log 에 ss_e1·ss_e2·ss_abs·ss_pwm_sd (모든 줄)', lr and all(x['ss_abs'] != '' for x in lr)
          and list(lr[0].keys()) == LOG_FIELDS, f'{len(lr)} 줄')
    check('학습 곡선에 ss_abs', any(p.get('ss_abs') is not None for p in s['learning_curve']))
    tr.rec.close()
    # 이어 하기: 옛 머리줄(ss 열 없음)이면 그 열 그대로
    od = os.path.join(tmp, 'oldlog')
    os.makedirs(od, exist_ok=True)
    old_fields = LOG_FIELDS[:-4]
    with open(os.path.join(od, 'train_log.csv'), 'w', encoding='utf-8', newline='') as f:
        csv.writer(f).writerow(old_fields)
    rec = Recorder(od)
    rec.log_row({k: 1 for k in LOG_FIELDS})
    rec.close()
    with open(os.path.join(od, 'train_log.csv'), encoding='utf-8') as f:
        lines = list(csv.reader(f))
    check('이어 하기: 옛 train_log 머리줄 유지 (열 수 일치)', len(lines) == 2 and len(lines[1]) == len(old_fields),
          f'머리 {len(lines[0])}열, 줄 {len(lines[1])}열')


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--quick', action='store_true')
    args = ap.parse_args(argv)
    tmp = tempfile.mkdtemp(prefix='rl_selftest_')
    try:
        t_env()
        t_sac(tmp)
        t_protocol()
        t_trainer(tmp, args.quick)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    n = sum(ok for _, ok, _ in RESULTS)
    print(f'\n자체 시험: {n}/{len(RESULTS)} 통과', flush=True)
    for name, ok, det in RESULTS:
        if not ok:
            print(f'  실패: {name} — {det}')
    return 0 if n == len(RESULTS) else 1


if __name__ == '__main__':
    sys.exit(main())
