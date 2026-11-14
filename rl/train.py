"""학습 실행 CLI.

    python -m rl.train --port COM4                 실제 보드 (모터가 돕니다 — 사람이 자리에서 허락한 뒤에만)
    python -m rl.train --fake                      가짜 보드, 실시간
    python -m rl.train --fake --fast               가짜 보드, 가상시간 (빠름)
    옵션: --config cfg.json  --resume data/<날짜>_rlN  --hours 2  --episodes 500  --bench  --randomize
          --latency-ms 7 9 (가짜 보드 명령 도착 지연)  --quiet

실행 중 표준입력으로 p(일시정지) / r(재개) / q(정지) / s(상태) 를 입력할 수 있습니다. Ctrl+C = 정지.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

from .config import load_config


def make_link_factory(args, cfg):
    """--port / --fake / --fast 에 맞는 연결 생성 함수 (재연결 때 다시 불림)."""
    if args.fake:
        from .fake_board import FakeBoard, FakePort
        from .link import SerialLink, VirtualLink
        board = FakeBoard(cfg.plant, seed=cfg.plant.seed)
        lat = tuple(args.latency_ms) if args.latency_ms else (0.3, 0.8)
        if args.fast:
            vl = VirtualLink(board, latency_ms=lat)
            return (lambda: vl), True
        holder = {}

        def factory():
            # 같은 가짜 보드를 계속 씀 (재연결 = 포트만 새로)
            if 'port' in holder:
                holder['port'].close()
                time.sleep(0.05)
            holder['port'] = FakePort(board, latency_ms=lat)
            return SerialLink(holder['port'], watchdog_s=cfg.monitor.hb_watchdog_s, name='fake-realtime')
        return factory, True
    if not args.port:
        raise SystemExit('--port COMx 또는 --fake 가 필요합니다')
    from .link import SerialLink
    return (lambda: SerialLink(args.port, watchdog_s=cfg.monitor.hb_watchdog_s)), False


def main(argv=None):
    ap = argparse.ArgumentParser(description='에어로펜듈럼 SAC 실기 학습기')
    ap.add_argument('--port')
    ap.add_argument('--fake', action='store_true')
    ap.add_argument('--fast', action='store_true', help='가짜 보드 가상시간 모드')
    ap.add_argument('--config')
    ap.add_argument('--resume', help='이어 할 실행 폴더 (data/<날짜>_rlN)')
    ap.add_argument('--hours', type=float)
    ap.add_argument('--episodes', type=int)
    ap.add_argument('--bench', action='store_true', help='BENCH 1 (모터 핀 LOW) 로 루프만 시험')
    ap.add_argument('--randomize', action='store_true', help='가짜 보드 도메인 랜덤화')
    ap.add_argument('--shock', type=float, help='가짜 보드 케이블 충격 빈도 [1/s]')
    ap.add_argument('--latency-ms', type=float, nargs=2)
    ap.add_argument('--data-dir', help='실행 폴더를 만들 곳 (기본 data/)')
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--dashboard', nargs='?', type=int, const=8770, default=8770, metavar='PORT',
                    help='로컬 웹 대시보드 (127.0.0.1, 기본 포트 8770)')
    ap.add_argument('--no-dashboard', action='store_true', help='웹 대시보드 끄기')
    args = ap.parse_args(argv)
    if args.fast and not args.fake:
        ap.error('--fast 는 --fake 와 함께')

    from .record import new_run_dir
    if args.resume:
        run_dir = args.resume
        cfg = load_config(os.path.join(run_dir, 'config.json'))
    else:
        cfg = load_config(args.config)
        run_dir = new_run_dir(args.data_dir)
    if args.hours is not None:
        cfg.stop.max_hours = args.hours
    if args.episodes is not None:
        cfg.stop.max_episodes = args.episodes
    if args.bench:
        cfg.run.bench = True
    if args.randomize:
        cfg.plant.randomize = True
    if args.shock is not None:
        cfg.plant.shock_rate = args.shock

    factory, sim = make_link_factory(args, cfg)
    from .trainer import Trainer
    log = (lambda s: None) if args.quiet else (lambda s: print(s, flush=True))
    tr = Trainer(cfg, factory, run_dir, resume=bool(args.resume), autostart=True, log=log, sim=sim)
    print(f'실행 폴더: {run_dir}', flush=True)
    tr.start()
    httpd = None
    if not args.no_dashboard:
        try:
            from .dashboard import serve
            httpd = serve(tr, port=args.dashboard)
            print(f'대시보드: http://127.0.0.1:{httpd.server_address[1]}/', flush=True)
        except OSError as e:
            print(f'대시보드를 열지 못했습니다 (포트 {args.dashboard}): {e}', flush=True)

    def stdin_loop():
        for line in sys.stdin:
            c = line.strip().lower()
            if c in ('p', 'pause'):
                print(tr.command('pause')[1], flush=True)
            elif c in ('r', 'resume'):
                print(tr.command('resume')[1], flush=True)
            elif c in ('q', 'stop', 'quit'):
                print(tr.command('stop')[1], flush=True)
            elif c in ('s', 'status'):
                st = tr.status()
                for k in ('recent_episodes', 'learning_curve', 'log'):
                    st.pop(k)
                print(json.dumps(st, ensure_ascii=False, indent=1), flush=True)

    threading.Thread(target=stdin_loop, daemon=True).start()
    try:
        while not tr.wait(1.0):
            pass
    except KeyboardInterrupt:
        print('정지 요청 (Ctrl+C)', flush=True)
        tr.command('stop')
        tr.wait(30)
    st = tr.status()
    print(f'끝: 상태 {st["state"]}, 학습 {st["episode"]} ep, 평가 {st["eval_rounds"]}회, '
          f'FALLBACK {st["counts"]["fallbacks"]}, 오류 {st["last_error"]}', flush=True)
    if st['eval_scores']:
        print(f'마지막 평가 RMS {st["eval_scores"][-1]["rms_mean"]:.3f}°, PID 기준선 '
              f'{st["baseline"]["rms_mean"] if st["baseline"] else float("nan"):.3f}°', flush=True)
    if httpd is not None:
        httpd.shutdown()
    return 0 if st['state'] == 'done' else 1


if __name__ == '__main__':
    sys.exit(main())
