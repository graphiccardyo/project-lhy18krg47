"""실행 폴더 기록: data/<YYYYMMDD>_rl<N>/

    config.json          사용한 설정
    train_log.csv        에피소드마다 한 줄 (학습·평가·기준선)
    episodes/epNNNNN_<kind>.csv   에피소드 시계열 (PROTOCOL.md CSV 형식 + ref_deg,miss,du,reward)
    checkpoint/          sac.pt, buffer.npz, trainer.json (--resume 으로 이어 하기)
    events.log           상태 변화·경고·오류
"""
from __future__ import annotations

import csv
import datetime as _dt
import glob
import os
import re

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LOG_FIELDS = ['episode', 'kind', 'scenario', 'status', 'wall_s', 'board_s', 'home', 'r1', 'r2', 'step_s',
              'samples', 'decisions', 'return', 'rms', 'rms_after_step', 'mean_pwm', 'miss', 'miss_frac',
              'mean_abs_du', 'fallbacks_total', 'updates', 'buffer', 'alpha', 'q_loss', 'pi_loss', 'entropy',
              'motor_on_s']


def new_run_dir(base: str | None = None, date: str | None = None) -> str:
    base = base or os.path.join(REPO, 'data')
    date = date or _dt.datetime.now().strftime('%Y%m%d')
    os.makedirs(base, exist_ok=True)
    nums = []
    for p in glob.glob(os.path.join(base, f'{date}_rl*')):
        m = re.fullmatch(rf'{date}_rl(\d+)', os.path.basename(p))
        if m:
            nums.append(int(m.group(1)))
    d = os.path.join(base, f'{date}_rl{max(nums, default=0) + 1}')
    os.makedirs(os.path.join(d, 'episodes'), exist_ok=True)
    os.makedirs(os.path.join(d, 'checkpoint'), exist_ok=True)
    return d


class Recorder:
    def __init__(self, run_dir: str):
        self.dir = run_dir
        os.makedirs(os.path.join(run_dir, 'episodes'), exist_ok=True)
        os.makedirs(os.path.join(run_dir, 'checkpoint'), exist_ok=True)
        self.log_path = os.path.join(run_dir, 'train_log.csv')
        new = not os.path.exists(self.log_path)
        self._logf = open(self.log_path, 'a', newline='', encoding='utf-8')
        self._log = csv.DictWriter(self._logf, fieldnames=LOG_FIELDS, extrasaction='ignore')
        if new:
            self._log.writeheader()
        self._evf = open(os.path.join(run_dir, 'events.log'), 'a', encoding='utf-8')

    def event(self, text: str):
        self._evf.write(f'{_dt.datetime.now().isoformat(timespec="seconds")} {text}\n')
        self._evf.flush()

    def log_row(self, row: dict):
        self._log.writerow(row)
        self._logf.flush()

    def episode_csv(self, name: str, kind: str, meta: dict, pid: dict, rows: list, ts: float):
        """rows: (t_ms, pwm, th, om, tha, phase, ref, miss, du, reward). t_s 는 첫 줄 기준."""
        path = os.path.join(self.dir, 'episodes', f'{name}.csv')
        params = ' '.join(f'{k}={v}' for k, v in meta.get('params', {}).items())
        head = (f'# aeropendulum_rl; name={name}; type=RL; params={params}; Ts={ts:g}; input=pwm(0-255); '
                f'axis={meta.get("axis", "?")}; beta={meta.get("beta", "?")}; dlpf={meta.get("dlpf", "?")}; '
                f'cal={meta.get("cal", "?")}; zero={meta.get("zero", "?")}; sim={meta.get("sim", 0)}; '
                f'bench={meta.get("bench", 0)}; status={meta.get("status", "?")}; mode={kind}; '
                f'fw={meta.get("fw", "?")}; start={meta.get("start", "")}\n')
        pidl = (f'# pid; kp={pid["kp"]:g}; ki={pid["ki"]:g}; kd={pid["kd"]:g}; ff={pid["ff"]}; umin={pid["umin"]}; '
                f'notch_cHz={pid["notch_cHz"]}; zz={pid["notch_zz"]}; zp={pid["notch_zp"]}\n')
        with open(path, 'w', encoding='utf-8', newline='') as f:
            f.write(head)
            f.write(pidl)
            f.write('t_s,pwm,duty_pct,theta_deg,omega_dps,theta_acc_deg,phase,ref_deg,miss,du,reward\n')
            t0 = rows[0][0] if rows else 0
            for t_ms, u, th, om, tha, ph, ref, miss, du, rew in rows:
                f.write(f'{(t_ms - t0) / 1000:.3f},{u},{u / 2.55:.2f},{th:.2f},{om:.2f},{tha:.2f},{ph},{ref:.2f},'
                        f'{"" if miss is None else miss},{"" if du is None else du},'
                        f'{"" if rew is None else f"{rew:.5f}"}\n')
        return path

    def close(self):
        for f in (self._logf, self._evf):
            try:
                f.close()
            except Exception:
                pass
