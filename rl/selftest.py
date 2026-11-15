"""하드웨어 없이 하는 자체 시험.

    python -m rl.selftest            전부 (몇 분)
    python -m rl.selftest --quick    짧은 학습 시험 생략

1. 프로토콜: bench_test 전 항목을 가짜 보드(가상시간)로
2. SAC: 버퍼 저장/복원, numpy 추론 = torch 액터, 체크포인트 왕복, 추론 시간, 작은 문제 학습
3. env: 합성 D 줄로 결정 위상·실제 증분·종단/절단, 고정 적분 보조(정책 0 이어도 오차 제거, 벌점은 정책 몫만)
4. 트레이너: 가상시간 짧은 학습이 실제로 나아지는지(학습 return 개선, 마지막 평가 < PID), 일시정지/재개(사용자·고장 주입), --resume
5. 스냅샷: 주기·best·수동(snapshot 명령, 일시정지 중 즉시), eval 명령(정기 일정과 별도), 옛 설정·체크포인트 호환
6. rl.play: 가짜 보드로 --refs --pid / --hold, 지표·CSV·그림, STOP 으로 끝남, ABORT 중단
7. 정상상태: steady_metrics(편향·리플·정착 시간·오버슈트), 정밀 보상·미세 오차 관측, 옛 실행 설정 호환(관측 7차원 유지), 주기 PID 기준선,
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
    cfg.env.ki_assist = 0.0        # 적분 보조 없이 (보조는 아래 t_env_ki)
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
    t_env_ki()


def _ki_loop(ki, secs=10.0, band=0.0):
    """정책 출력 0, 합성 1차 플랜트(θ_ss = 0.15·(u − 40)°, τ 0.3 s), 기준 25°, 시작 u 170 (e ≈ 5.5°)."""
    from .config import Config
    from .env import EpisodeEnv
    from .link import parse_line
    cfg = Config()
    cfg.env.ki_assist = ki
    cfg.env.ki_band_deg = band
    env = EpisodeEnv(cfg)
    env.arm(lambda o: 0.0, 2, 10)
    u, th, pend = 170, 0.15 * 130, 0
    for n in range(int(secs / 0.01)):
        u = max(40, min(255, u + pend))
        pend = 0
        th += (0.15 * (u - 40) - th) * (0.01 / 0.3)
        cmd = env.on_D(parse_line(f'D,{10 * n},{u},{round(th * 100)},0,{round(th * 100)},1,2500,0'))
        if cmd:
            pend = int(cmd.split()[1])
    env.finish()
    return env, 25.0 - th, u


def t_env_ki():
    print('== 3b. env 고정 적분 보조 (합성 플랜트, 정책 출력 0)', flush=True)
    env0, e0, u0 = _ki_loop(0.0)
    env3, e3, u3 = _ki_loop(3.0)
    check('k_I 0: 정책 0 이면 PWM 유지 → 오차 그대로', u0 == 170 and abs(e0 - 5.5) < 0.1, f'e {e0:.2f}°, u {u0}')
    check('k_I 3: 정책 0 이어도 적분이 오차를 없앰 (10 s 뒤 |e| < 0.3°)', abs(e3) < 0.3 and u3 > 190,
          f'e {e3:.2f}°, u {u3}')
    th_s = env3.cfg.env.th_scale
    a_ok = all(abs(t[1]) < 1e-9 for t in env3.trans)
    r_ok = all(abs(t[2] - env3._reward(float(t[3][0]) * th_s, 0.0, 0.0)) < 1e-6 for t in env3.trans)
    check('k_I 3: 리플레이 행동 = 정책 몫 0, 보상에 적분 증분 벌점 없음', a_ok and r_ok,
          f'적분 증분 평균 {env3.summary()["mean_abs_di"]:.2f} PWM/결정')
    _, eb, ub = _ki_loop(10.0, band=1.5)
    _, en, un = _ki_loop(10.0)
    check('적분 분리(폭 1.5°): 큰 오차(5.5°)에서는 적분 거의 0, 분리 없으면 k_I 10 이 오차 제거',
          ub <= 171 and abs(eb - 5.5) < 0.2 and abs(en) < 0.3, f'분리 e {eb:.2f}° u {ub}, 분리 없음 e {en:.2f}° u {un}')


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
    s = wait_state(('episode',))
    # 자세 추정 표류 흉내: θacc 를 5° 낮춤 (θ − θacc = +5°) → 10 s 평균이 3° 를 넘으면 자동 일시정지
    orig_measure = board.plant.measure
    board.plant.measure = lambda running: (lambda m: (m[0], m[1], m[2] - 5.0))(orig_measure(running))
    s = wait_state(('paused',), timeout=120)
    board.plant.measure = orig_measure
    # 가상 링크는 시간이 흘러야 명령을 보드에 전달하고, 일시정지 중에는 시간을 진행하지 않음 →
    # STOP 이 보내졌는지(대기열에 있거나 이미 반영돼 u_out 0)로 확인 (실보드는 즉시 처리)
    stop_sent = board.u_out == 0 or any(ln == 'STOP' for _, ln in list(tr.link.pending))
    check('θ − θacc 표류(+5°) → 자동 일시정지 (STOP 전송)', s['state'] == 'paused'
          and '자세 추정' in (s['paused_reason'] or '') and stop_sent and (s['att_gap_deg'] or 0) >= 3.0,
          f'{s["paused_reason"]}, 간극 {s["att_gap_deg"]}, STOP {stop_sent}')
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
    t_fork(tmp)
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
    rets = [p['ret'] for p in s['learning_curve'] if p['kind'] == 'train']
    r_first, r_last = float(np.mean(rets[:3])), float(np.mean(rets[-10:]))
    # 학습이 되는지만 보는 연기 시험. 예전에는 "평가 RMS 가 첫 평가보다 x% 감소" 였는데, 기본값이 바뀔 때마다
    # (기준 ≤ 30°, 적분 보조, 20 s 에피소드 — 긴 구간은 RMS 를 낮춤) 첫 평가 자체가 낮아져 비율이 흔들렸음.
    # 지금: 무작위 워밍업 3 에피소드의 return 대비 마지막 10 에피소드 return 이 절반 이하(벌점), 마지막 평가 < PID.
    # (11-15 이 시드: 새 기본값 −226 → −57, 평가 2.35 → 1.90° / PID 2.46°. k_I 3·10 s 는 −241 → −45, 2.78 / 3.59°)
    check('짧은 학습: 학습 return 개선 (워밍업 대비 벌점 절반 이하), 마지막 평가 RMS < PID 기준선',
          len(ev) >= 3 and len(rets) >= 13 and r_last > 0.5 * r_first and ev[-1] < base,
          f'return {r_first:.0f} → {r_last:.0f}, 평가 RMS {ev}, PID {base:.3f}°, 가상 {s["elapsed_s"] / 60:.1f} 분 / '
          f'실제 {(time.time() - t0) / 60:.1f} 분')


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


def t_fork(tmp):
    print('== 6b. rl.fork (스냅샷 시점에서 분기 → 이어 하기)', flush=True)
    import csv
    import json
    from .fork import fork
    src = SNAP_RUN.get('dir')
    if not src:
        check('fork: 원본 실행 (스냅샷 시험) 없음', False)
        return
    with open(os.path.join(src, 'checkpoints', 'ep00004', 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    out = os.path.join(tmp, 'fork1')
    r = fork(src, 'ep00004', out, max_gap=3.0, critic_steps=300, log=lambda s: None)
    with open(os.path.join(out, 'train_log.csv'), encoding='utf-8') as f:
        lr = list(csv.DictReader(f))
    check('fork: 스냅샷까지의 버퍼·train_log·스냅샷 복사 (가짜 보드는 간극 0 → 제외 0)',
          r['episode'] == 4 and r['cut'] == meta['decisions'] and r['kept'] == r['cut'] and int(lr[-1]['buffer']) == r['cut']
          and lr[-1]['kind'] == 'train' and 'ep00004' in r['snapshots'] and 'ep00006' not in r['snapshots']
          and all(x['att_gap'] != '' for x in lr), f'{r}, 줄 {len(lr)}')
    r2 = fork(src, 'ep00004', os.path.join(tmp, 'fork2'), max_gap=0.05, critic_steps=0, log=lambda s: None)
    check('fork: --max-gap 으로 간극 큰 전이 제외', 0 < r2['kept'] < r2['cut'], f'{r2["kept"]}/{r2["cut"]}')
    tr, _, _ = _virtual_trainer(tmp, 'fork1', 6, {'run.baseline_at_start': True, 'run.eval_every': 3,
                                                  'run.baseline_every': 1}, resume_dir=out)
    st0 = tr.status()
    from .sac import load_actor_state
    import torch
    sd, _ = load_actor_state(os.path.join(src, 'checkpoints', 'ep00004'))
    same = all(torch.equal(sd[k], v) for k, v in tr.sac.actor.state_dict().items())
    check('fork → --resume: 학습 4 ep·스냅샷 갱신 수·스냅샷 액터에서 시작', st0['episode'] == 4 and same
          and st0['updates'] == meta['updates'] and st0['buffer'] == r['kept'], f'ep {st0["episode"]}, 갱신 {st0["updates"]}')
    tr.start()
    tr.wait(300)
    s = tr.status()
    eps = [b['episode'] for b in s['baseline_history']]
    check('fork 이어 하기: 시작 때 PID 기준선 다시, 같은 에피소드에서 중복 없음, 학습 6 ep 까지', s['state'] == 'done'
          and s['episode'] == 6 and eps.count(4) == 1 and s['baseline']['episode'] >= 4, f'{s["state"]}, 기준선 {eps}')
    tr.rec.close()


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
    sub = {k: m[k] for k in ('ss_e1', 'ss_e2', 'ss_abs', 'ss_pwm_sd', 'ss_bias', 'ss_ripple')}
    check('steady_metrics: 창 평균 e(부호), 평균 |e|, PWM σ, 편향 = |창 평균|, 리플 0',
          sub == dict(ss_e1=-0.5, ss_e2=0.3, ss_abs=0.4, ss_pwm_sd=0.5, ss_bias=0.4, ss_ripple=0.0)
          and m['settle_s'] is None, str(m))
    m2 = steady_metrics(rows[:300], 500, 150)
    check('steady_metrics: 스텝 전에 끝난 에피소드 → 창 1 만', m2['ss_e1'] == -0.5 and m2['ss_e2'] is None, str(m2))
    # 1차 응답(τ 0.5 s)으로 20 → 25° → 22°, 끝 편향 0.3°, 리플 사인 0.1°: 정착·오버슈트·리플 분리
    rows3, th = [], 20.0
    for i in range(2000):
        r_ = 25.0 if i < 1000 else 22.0
        th += (r_ - 0.3 - th) * 0.02
        rows3.append([i * 10, 150, th + 0.1 * math.sin(i * 0.5), 0, 0, 1, r_])
    m3 = steady_metrics(rows3, 1000, 300)
    check('steady_metrics: 정착 시간(±0.5°, 구간 안 이동평균)·오버슈트·편향/리플 분리',
          abs(m3['settle1'] - 1.67) < 0.1 and abs(m3['settle2'] - 0.76) < 0.1 and m3['os1'] == 0.0
          and abs(m3['os2'] - 13.3) < 0.5 and abs(m3['ss_bias'] - 0.3) < 0.01 and abs(m3['ss_ripple'] - 0.0707) < 0.005,
          str(m3))
    rows4 = [[x[0], x[1], x[2] - 1.0, 0, 0, 1, x[6]] for x in rows3]   # 편향 1.3° → 띠 밖 → 구간 길이(검열)
    m4 = steady_metrics(rows4, 1000, 300)
    check('steady_metrics: 끝까지 띠에 못 들어오면 정착 = 구간 길이', m4['settle1'] == 10.0 and m4['settle2'] == 10.0,
          str({k: m4[k] for k in ('settle1', 'settle2', 'ss_bias')}))
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
    for k in ('obs_efine', 'w_fine', 'efine_scale', 'fine_scale', 'ki_assist'):
        d['env'].pop(k)
    d['run'].pop('baseline_every')
    c_leg, c_new = from_dict(d, legacy=True), from_dict(d)
    check('옛 실행 config(키 없음) → 관측 7차원·정밀 항 0·적분 보조 0·기준선 시작 때만, --config 는 새 기본값',
          obs_dim(c_leg) == 7 and c_leg.env.w_fine == 0.0 and c_leg.env.ki_assist == 0.0
          and c_leg.run.baseline_every == 0 and obs_dim(c_new) == 8 and c_new.env.ki_assist > 0,
          f'옛 {obs_dim(c_leg)}, 새 {obs_dim(c_new)}')
    check('기본값: PWM 상한 255 (LIM = u_hi), 기준 ≤ 30°, 평가 기준 ≤ 30°, 에피소드 20 s·창 3 s·k_I 6',
          cfg.lim.pwm_max == 255 and cfg.rl.u_hi == 255 and cfg.episode.ref_hi == 30.0
          and max(max(x) for x in cfg.run.eval_refs) <= 30.0 and cfg.episode.dur_s == 20.0
          and cfg.run.ss_win_s == 3.0 and cfg.env.ki_assist == 6.0 and cfg.env.ki_band_deg == 0.0)
    # 주기 기준선 + ss 기록
    tr, board, rd = _virtual_trainer(tmp, 'steady', 5, {'sac.warmup_steps': 500, 'run.eval_every': 2,
                                                        'run.baseline_every': 2}, seed=31)
    tr.start()
    tr.wait(600)
    s = tr.status()
    bh = s['baseline_history']
    check('주기 PID 기준선: 시작 + 정기 평가 2회마다 (ep 0, 4)', [b['episode'] for b in bh] == [0, 4]
          and s['baseline'] == bh[-1] and all(b['ss_mean'] is not None for b in bh), str([b['episode'] for b in bh]))
    check('평가 점수에 정상상태 |e|·편향·리플·정착·OS', s['eval_scores'] and all(
        e['ss_mean'] is not None and len(e['ss_per_scenario']) == 3 and e['bias_mean'] is not None
        and e['ripple_mean'] is not None and e['settle_mean'] is not None and e['os_mean'] is not None
        for e in s['eval_scores']) and all(b['bias_mean'] is not None and b['settle_mean'] is not None for b in bh),
          str([(e.get('bias_mean'), e.get('settle_mean')) for e in s['eval_scores']]))
    with open(os.path.join(rd, 'train_log.csv'), encoding='utf-8') as f:
        lr = list(csv.DictReader(f))
    check('train_log 에 ss_*·편향·리플·정착·OS (모든 줄)', lr and all(x['ss_abs'] != '' and x['ss_bias'] != ''
                                                          and x['settle_s'] != '' for x in lr)
          and list(lr[0].keys()) == LOG_FIELDS, f'{len(lr)} 줄')
    check('학습 곡선에 ss_abs·ss_bias·settle_s', any(p.get('ss_abs') is not None and p.get('ss_bias') is not None
                                                and p.get('settle_s') is not None for p in s['learning_curve']))
    s_json = __import__('json').dumps(s)   # numpy 값이 섞이면 실패
    check('status() JSON 직렬화 (새 지표 포함)', len(s_json) > 0)
    tr.rec.close()
    # 이어 하기: 옛 머리줄(ss 열 없음)이면 그 열 그대로
    od = os.path.join(tmp, 'oldlog')
    os.makedirs(od, exist_ok=True)
    old_fields = LOG_FIELDS[:-9]
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
