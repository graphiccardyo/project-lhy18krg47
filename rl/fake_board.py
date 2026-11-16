"""가짜 보드: RL 펌웨어(AEROPEND_RL 1.0)를 줄 단위 프로토콜 그대로 흉내 냅니다.

기준: docs/RL_PROTOCOL.md (+ PROTOCOL.md), 대조: firmware/aeropendulum_rl/aeropendulum_rl.ino.
- `FakeBoard` 는 시간 진행을 스스로 하지 않습니다. `write(line)` 으로 명령을 넣고 `tick()` 으로 5 ms 진행하면
  보드가 내보낼 줄을 `emit` 콜백으로 돌려줍니다(줄 끝 문자 없음).
- 실시간: `FakePort` (pyserial 흉내, 별도 스레드가 실제 시간에 맞춰 tick) → `link.SerialLink` 에 그대로 꽂힘.
- 가상시간: `link.VirtualLink` 가 필요한 만큼 tick 을 직접 돌림(실시간보다 수백 배 빠름).

세부 규칙(명세 "세부 규칙" 절): 한 샘플 = 종료 검사 → A 적용 → SAFE(θ→ω) → 각도 한계 → 출력 → D.
결정 샘플 D 이후~적용 시점 전에 온 A 만 사용, 늦은 A 와 n=0 전 A 는 버림. u_lo 먼저, min(u_hi, PWMmax) 나중.
PID 전환 시 I = (직전 샘플 적용 PWM) − ff.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np

from .config import Config, PlantCfg
from .plant import Plant

FW_ID = 'AEROPEND_RL 1.2'
TICK_S = 0.005
LINK_TIMEOUT_S = 1.0
MAX_SAMPLES = 120000


def _ints(s: str, max_n: int):
    """펌웨어 parseInts 와 같은 규칙. 실패하면 None."""
    out = []
    for tok in s.split(' '):
        if tok == '':
            continue
        if len(out) >= max_n:
            return None
        t = tok[1:] if tok[0] in '+-' else tok
        if not t.isdigit() or not t.isascii():
            return None
        v = int(t)
        if v > 100000000:
            return None
        out.append(-v if tok[0] == '-' else v)
    return out


class FakeBoard:
    def __init__(self, plant_cfg: PlantCfg | None = None, emit=None, seed: int = 1):
        self.plant_cfg = plant_cfg or PlantCfg()
        self.rng = np.random.default_rng(seed)
        self.plant = Plant(self.plant_cfg, self.rng)
        self.emit = emit or (lambda line: None)
        self.t = 0.0                  # 보드 시계 [s]
        self.tick_count = 0
        # 설정 (재부팅 기본값)
        self.axis, self.sign, self.beta, self.dlpf = 'Z', '-', 20, 4
        self.cal, self.zero = 1, 1
        self.div = 2
        self.umax, self.alim, self.rev, self.bench = 255, 12000, 0, 0
        self.imu_ok = True
        self.backup = None            # CFG BACKUP 사본 (EEPROM 128)
        # PID
        self.kp = self.ki = self.kd = 0.0
        self.ff, self.umin = 150, 0
        self.nfc, self.nzz, self.nzp = 0, 150, 700
        self.nb = (1.0, 0.0, 0.0, 0.0, 0.0)
        self.pidI = self.omF = self.nz1 = self.nz2 = 0.0
        self.ref = 0.0
        self.cl_hold = False
        # RL
        self.dec, self.lag, self.dumax, self.ulo, self.uhi = 2, 1, 10, 0, 255
        self.res = 0                  # 잔차 모드 한계 [PWM] (0 = 증분 모드)
        self.pid_add = 0              # 잔차 모드 r (PID 출력에 더함)
        self.thlo, self.thhi, self.wmax = -2000, 6000, 40000
        self.rlU = 0
        self.rlDu = 0
        self.rlHave = False
        self.rlPh = 0
        self.rlMiss = 0
        self.rlHome = 0
        # 실행 상태
        self.prof = None              # None | dict(type='RL'|'CLSTEP', n, total, ...)
        self.u_manual = 0
        self.u_out = 0
        self.phase = 0
        self.theta = self.omega = self.theta_acc = 0.0
        self.last_hb = 0.0
        self.div_count = 0
        self.aux_count = 0
        self.last_status = 0.0
        self.overruns = 0
        self.sat_count = 0
        self.max_tick_us = 0
        # 시험용 고장 주입
        self.inject_overruns = 0      # 다음 S 줄까지 overruns 를 이만큼 더함
        self.inject_sat = 0           # 다음 S 줄들의 sat 값
        self.inject_silent = False    # True 면 아무 줄도 내보내지 않음 (연결 끊김 흉내)

    # ------------------------------------------------------------------ 출력
    def _out(self, line: str):
        if not self.inject_silent:
            self.emit(line)

    def boot(self):
        self._out('READY ' + FW_ID)
        self._out('IMU READY addr=0x68 i2c=400')
        self._print_cfg()
        self._print_cal()

    def _print_cfg(self):
        self._out(f'CFG axis={self.axis} sign={self.sign} beta={self.beta} dlpf={self.dlpf} div={self.div} '
                  f'ts_ms={5 * self.div} umax={self.umax} alim={self.alim} rev={self.rev} bench={self.bench} '
                  f'arng=16 grng=2000 cal={self.cal} zero={self.zero}')

    def _print_cal(self):
        self._out(f'CALVAL {self.cal} 0.9200 -1.0100 -0.1500 -0.0171 -0.0220 0.1475 g0 0.0000 0.0000 1.0000')

    def _print_pid(self):
        self._out(f'PIDCFG kp={self.kp:.3f} ki={self.ki:.3f} kd={self.kd:.3f} ff={self.ff} notch={self.nfc} '
                  f'zz={self.nzz} zp={self.nzp} umin={self.umin}')

    def _print_rl(self):
        self._out(f'RLCFG dec={self.dec} lag={self.lag} dumax={self.dumax} ulo={self.ulo} uhi={self.uhi} res={self.res} '
                  f'thlo={self.thlo} thhi={self.thhi} wmax={self.wmax}')

    def _err(self, code: str):
        self._out('ERR ' + code)

    # ------------------------------------------------------------------ 제어
    def _ts(self):
        return TICK_S * self.div

    def _notch_coeffs(self):
        if not self.nfc:
            self.nb = (1.0, 0.0, 0.0, 0.0, 0.0)
            return
        ts, w = self._ts(), 2 * math.pi * self.nfc * 0.01
        k = w / math.tan(w * ts * 0.5)
        k2, w2 = k * k, w * w
        zz, zp = self.nzz * 0.001, self.nzp * 0.001
        a0 = k2 + 2 * zp * w * k + w2
        b0 = (k2 + 2 * zz * w * k + w2) / a0
        b1 = 2 * (w2 - k2) / a0
        b2 = (k2 - 2 * zz * w * k + w2) / a0
        a2 = (k2 - 2 * zp * w * k + w2) / a0
        self.nb = (b0, b1, b2, b1, a2)

    def _pid_reset(self):
        self.pidI = 0.0
        self.omF = self.omega
        self.nz1 = self.nz2 = 0.0
        self._notch_coeffs()

    def _pid_step(self, r: float) -> int:
        ts = self._ts()
        e = r - self.theta
        self.omF += ts / (0.0159 + ts) * (self.omega - self.omF)
        v = self.kp * e + self.pidI - self.kd * self.omF + self.pid_add   # 잔차도 노치를 통과 (1.2)
        if self.nfc:
            b0, b1, b2, a1, a2 = self.nb
            y = b0 * v + self.nz1
            self.nz1 = b1 * v - a1 * y + self.nz2
            self.nz2 = b2 * v - a2 * y
            v = y
        u = self.ff + v
        hi = float(self.umax)
        lo = -hi if self.rev else (self.umin if self.umin < hi else hi)
        us = lo if u < lo else (hi if u > hi else u)
        if u == us or (u > us and e < 0) or (u < us and e > 0):
            self.pidI = max(-255.0, min(255.0, self.pidI + self.ki * e * ts))
        return int(math.floor(us + 0.5)) if us >= 0 else -int(math.floor(-us + 0.5))

    def _apply_motor(self, u: int):
        if u < 0 and not self.rev:
            u = 0
        u = max(-self.umax, min(self.umax, u))
        self.u_out = u

    def _stop_all(self):
        self.prof = None
        self.cl_hold = False
        self.pidI = 0.0
        self.pid_add = 0
        self.u_manual = 0
        self.phase = 0
        self._apply_motor(0)

    def _abort(self, why: str):
        was = self.prof is not None or self.u_out != 0
        self._stop_all()
        if was:
            self._out('EVT ABORT ' + why)

    def _busy(self):
        return self.prof is not None

    def _ms2n(self, ms: int) -> int:
        ts = 5 * self.div
        return 0 if ms <= 0 else (ms + ts // 2) // ts

    # ------------------------------------------------------------------ 명령
    def write(self, line: str):
        """명령 한 줄 (줄 끝 문자 없이)."""
        if len(line) > 71 or any(ord(c) < 32 or ord(c) > 126 for c in line):
            self._abort('INVALID_LINE')
            self._err('INVALID_LINE')
            return
        if not line:
            return
        self.last_hb = self.t
        self._command(line)

    def _command(self, line: str):
        ep = self.prof is not None and self.prof['type'] == 'RL'
        if line == 'PING':
            return
        if line.startswith('A '):
            v = _ints(line[2:], 1)
            if v is None or len(v) != 1:
                return self._err('BAD_ARGS')
            if not ep:
                return self._err('NOT_RL')
            m = self.res if self.res else self.dumax      # 잔차 모드면 잔차 절댓값 ±res
            self.rlDu = max(-m, min(m, v[0]))
            self.rlHave = True
            return
        if line.startswith('RL '):
            if ep:
                return self._err('BUSY')
            v = _ints(line[3:], 6)
            if v is not None and len(v) == 5:
                v.append(0)
            if v is None or len(v) != 6 or not (0 <= v[5] <= 100) or not (1 <= v[0] <= 20) or not (1 <= v[1] <= v[0]) or not (1 <= v[2] <= 255) \
                    or v[3] < 0 or v[3] >= v[4] or v[4] > 255:
                return self._err('BAD_ARGS')
            self.dec, self.lag, self.dumax, self.ulo, self.uhi, self.res = v
            return self._print_rl()
        if line.startswith('SAFE '):
            if ep:
                return self._err('BUSY')
            v = _ints(line[5:], 3)
            if v is None or len(v) != 3 or v[0] < -18000 or v[1] > 18000 or v[0] >= v[1] or not (1 <= v[2] <= 200000):
                return self._err('BAD_ARGS')
            self.thlo, self.thhi, self.wmax = v
            return self._print_rl()
        if line == 'HELLO':
            self._stop_all()
            self._out('READY ' + FW_ID)
            self._out('IMU READY' if self.imu_ok else 'IMU ERROR NOT_READY')
            self._print_cfg()
            self._print_cal()
            return
        if line == 'STOP':
            self._stop_all()
            self._out('EVT STOP')
            return
        if line == 'GET':
            self._print_cfg()
            self._print_cal()
            self._print_pid()
            self._print_rl()
            return
        if line.startswith('PID '):
            w = _ints(line[4:], 5)
            if w is not None and len(w) == 4:
                w.append(self.umin)
            if w is None or len(w) != 5 or min(w) < 0 or w[0] > 200000 or w[1] > 500000 or w[2] > 50000 \
                    or w[3] > 255 or w[4] > 255:
                return self._err('BAD_ARGS')
            self.kp, self.ki, self.kd = w[0] * 0.001, w[1] * 0.001, w[2] * 0.001
            self.ff, self.umin = w[3], w[4]
            return self._print_pid()
        if line.startswith('NOTCH '):
            w = _ints(line[6:], 3)
            if w is not None and len(w) == 1 and w[0] == 0:
                self.nfc = 0
            elif w is not None and len(w) == 3 and 10 <= w[0] <= 2000 and 0 <= w[1] <= 2000 and 0 < w[2] <= 2000:
                self.nfc, self.nzz, self.nzp = w
            else:
                return self._err('BAD_ARGS')
            self.nz1 = self.nz2 = 0.0
            self._notch_coeffs()
            return self._print_pid()
        if line.startswith('REF '):
            if not ep and self._busy():
                return self._err('BUSY')
            if not self.imu_ok and not self.bench:
                return self._err('IMU_NOT_READY')
            v = _ints(line[4:], 1)
            if v is None or len(v) != 1 or abs(v[0]) > 17000:
                return self._err('BAD_ARGS')
            if not ep and not self.cl_hold:
                self._pid_reset()
                self.u_manual = 0
                self.cl_hold = True
            self.ref = v[0] * 0.01
            self._out(f'EVT REF {v[0]}')
            return
        if line.startswith('U '):
            if not ep and self._busy():
                return self._err('BUSY')
            if not self.imu_ok and not self.bench:
                return self._err('IMU_NOT_READY')
            v = _ints(line[2:], 1)
            if v is None or len(v) != 1 or abs(v[0]) > 255:
                return self._err('BAD_ARGS')
            if ep:
                self._stop_all()
                self._out('EVT STOP')
            self.cl_hold = False
            self.u_manual = v[0]
            return
        if line.startswith('LIM '):
            v = _ints(line[4:], 3)
            if v is None or len(v) not in (2, 3) or not (0 <= v[0] <= 255) or not (500 <= v[1] <= 18000):
                return self._err('BAD_ARGS')
            self.umax, self.alim = v[0], v[1]
            if len(v) == 3:
                self.rev = 1 if v[2] else 0
            return self._print_cfg()
        if line.startswith('STREAM '):
            if self._busy():
                return self._err('BUSY')
            v = _ints(line[7:], 1)
            if v is None or len(v) != 1 or not (1 <= v[0] <= 20):
                return self._err('BAD_ARGS')
            self.div = v[0]
            self._notch_coeffs()
            return self._print_cfg()
        if line.startswith('AXIS '):
            a, s = line[5:6], line[7:8] if line[6:7] == ' ' else ''
            if a not in ('X', 'Y', 'Z') or s not in ('+', '-') or len(line) != 8:
                return self._err('BAD_ARGS')
            if self.u_out != 0 or self._busy():
                return self._err('BUSY')
            self.axis, self.sign = a, s
            return self._print_cfg()
        if line.startswith('BETA '):
            v = _ints(line[5:], 1)
            if v is None or len(v) != 1 or not (1 <= v[0] <= 1000):
                return self._err('BAD_ARGS')
            self.beta = v[0]
            return self._print_cfg()
        if line.startswith('DLPF '):
            v = _ints(line[5:], 1)
            if v is None or len(v) != 1 or not (0 <= v[0] <= 6):
                return self._err('BAD_ARGS')
            self.dlpf = v[0]
            return self._print_cfg()
        if line == 'ZERO':
            if self.u_out != 0 or self._busy():
                return self._err('BUSY')
            self.zero = 1
            self._out('ZERO OK')
            self._print_cfg()
            self._print_cal()
            return
        if line in ('CAL', 'CAL FLAT'):
            if self.u_out != 0 or self._busy():
                return self._err('BUSY')
            # 보정 자체는 흉내 내지 않음 (즉시 성공)
            self._out(f'CAL START mode={"FLAT" if line == "CAL FLAT" else "STILL"} n=1000')
            self._out('CAL OK mode=STILL gstd=0.041 astd=0.0028 anorm=1.002')
            self.cal = 1
            self._print_cal()
            return
        if line == 'CAL CLEAR' or line.startswith('ACCOFF '):
            if self._busy():
                return self._err('BUSY')
            self._print_cal()
            return
        if line.startswith('BENCH '):
            v = _ints(line[6:], 1)
            if v is None or len(v) != 1 or not (0 <= v[0] <= 1):
                return self._err('BAD_ARGS')
            if self.u_out != 0 or self._busy():
                return self._err('BUSY')
            self._stop_all()
            self.bench = v[0]
            self._apply_motor(0)
            return self._print_cfg()
        if line.startswith('RUN '):
            return self._run(line[4:])
        if line == 'CFG BACKUP':
            if self._busy():
                return self._err('BUSY')
            self.backup = dict(axis=self.axis, sign=self.sign, beta=self.beta, dlpf=self.dlpf, cal=self.cal, zero=self.zero)
            self._out('CFG BACKUP OK')
            return
        if line == 'CFG RESTORE':
            if self.u_out != 0 or self._busy():
                return self._err('BUSY')
            if self.backup is None:
                return self._err('NO_BACKUP')
            for k, val in self.backup.items():
                setattr(self, k, val)
            self._out('CFG RESTORE OK')
            self._print_cfg()
            self._print_cal()
            return
        if line == 'FACTORY':
            if self.u_out != 0 or self._busy():
                return self._err('BUSY')
            self.axis, self.sign, self.beta, self.dlpf, self.cal, self.zero = 'X', '+', 20, 4, 0, 0
            self._print_cfg()
            self._print_cal()
            return
        self._err('UNKNOWN_COMMAND')

    def _run(self, args: str):
        if self._busy():
            return self._err('BUSY')
        if not self.imu_ok and not self.bench:
            return self._err('IMU_NOT_READY')
        name, _, rest = args.partition(' ')
        v = _ints(rest, 5)
        n = len(v) if v is not None else 255
        if name == 'RL':
            if n != 3 or abs(v[0]) > 17000 or v[1] <= 0 or abs(v[2]) > 17000:
                return self._err('BAD_ARGS')
            if self.kp == 0 and self.ki == 0 and self.kd == 0:
                return self._err('NO_PID')
            total = self._ms2n(v[1])
            if total == 0 or total > MAX_SAMPLES:
                return self._err('BAD_DURATION')
            u = self.u_out
            hi = min(self.uhi, self.umax)
            if u < self.ulo:
                u = self.ulo
            if u > hi:
                u = hi
            self.prof = dict(type='RL', n=0, total=total)
            self.rlU = u
            self.rlHave = False
            self.rlPh = 0
            self.rlMiss = 0
            self.rlHome = v[2]
            self.ref = v[0] * 0.01
            self.cl_hold = False
            self._pid_reset()
            self.pid_add = 0
            if self.res:                     # 잔차 모드: PID 를 지금 출력에서 이어 감 (적분 = u − ff)
                self.pidI = max(-255.0, min(255.0, float(self.u_out - self.ff)))
            if self.plant_cfg.randomize:
                self.plant.randomize()
            self._start_profile('RL', total)
            return
        if name != 'CLSTEP':
            return self._err('UNKNOWN_PROFILE')
        if n != 5 or abs(v[0]) > 17000 or abs(v[1]) > 17000 or v[2] < 0 or v[3] <= 0 or v[4] < 0:
            return self._err('BAD_ARGS')
        a, b, c = self._ms2n(v[2]), self._ms2n(v[3]), self._ms2n(v[4])
        self.cl_hold = False
        self._pid_reset()
        if a + b + c == 0 or a + b + c > MAX_SAMPLES:
            return self._err('BAD_DURATION')
        self.prof = dict(type='CLSTEP', n=0, total=a + b + c, a=a, b=b, r0=v[0], r1=v[1])
        self._start_profile('CLSTEP', a + b + c)

    def _start_profile(self, name: str, total: int):
        self.u_manual = 0
        self.div_count = 0
        self._out(f'EVT RUN {name} total={total} ts_ms={5 * self.div}')

    # ------------------------------------------------------------------ 샘플
    def _rl_to_pid(self) -> int:
        u0 = self.u_out
        self.pid_add = 0
        self.prof = None
        self.phase = 0
        self._pid_reset()
        self.pidI = max(-255.0, min(255.0, float(u0 - self.ff)))
        self.ref = self.rlHome * 0.01
        self.cl_hold = True
        self._out(f'EVT REF {self.rlHome}')
        return self._pid_step(self.ref)

    def _rl_step(self) -> int:
        p = self.prof
        n = p['n']
        if n >= p['total']:
            self._out('EVT DONE')
            return self._rl_to_pid()
        if n != 0 and self.rlPh == self.lag % self.dec:
            if self.rlHave:
                if self.res:
                    self.pid_add = self.rlDu
                else:
                    u = self.rlU + self.rlDu
                    hi = min(self.uhi, self.umax)
                    if u < self.ulo:
                        u = self.ulo
                    if u > hi:
                        u = hi
                    self.rlU = u
                self.rlHave = False
            else:
                self.rlMiss += 1
        th = self.theta * 100
        why = None
        if th < self.thlo or th > self.thhi:
            why = 'THETA'
        elif abs(self.omega) * 100 > self.wmax:
            why = 'OMEGA'
        if why:
            self._out('EVT FALLBACK ' + why)
            return self._rl_to_pid()
        if self.rlPh == 0:
            self.rlHave = False
        self.rlPh += 1
        if self.rlPh >= self.dec:
            self.rlPh = 0
        p['n'] += 1
        self.phase = 1
        if self.res:                         # 잔차 모드: u = PID(ref) + r, [ulo, min(uhi, umax)]
            return max(self.ulo, min(min(self.uhi, self.umax), self._pid_step(self.ref)))
        return self.rlU

    def _clstep(self) -> int:
        p = self.prof
        n = p['n']
        p['n'] += 1
        if n < p['a']:
            r, self.phase = p['r0'], 1
        elif n < p['a'] + p['b']:
            r, self.phase = p['r1'], 2
        else:
            r, self.phase = p['r0'], 3
        self.ref = r * 0.01
        return self._pid_step(self.ref)

    def _sample(self):
        running = self.u_out != 0 and not self.bench
        self.theta, self.omega, self.theta_acc = self.plant.measure(running)
        if self.prof is not None and self.prof['type'] == 'RL':
            u = self._rl_step()
        elif self.prof is not None:
            if self.prof['n'] >= self.prof['total']:
                self._stop_all()
                self._out('EVT DONE')
                u = 0
            else:
                u = self._clstep()
        elif self.cl_hold:
            u = self._pid_step(self.ref)
            self.phase = 0
        else:
            u = self.u_manual
            self.phase = 0
        if u != 0 and abs(self.theta) * 100 > self.alim:
            self._abort('ANGLE_LIMIT')
            u = 0
        self._apply_motor(u)
        # D 줄
        s = (f'D,{int(self.t * 1000)},{self.u_out},{round(self.theta * 100)},{round(self.omega * 100)},'
             f'{round(self.theta_acc * 100)},{self.phase}')
        if self.cl_hold or self.prof is not None:
            s += f',{round(self.ref * 100)}'
        if self.prof is not None and self.prof['type'] == 'RL':
            s += f',{self.rlMiss}'
        self._out(s)
        self.aux_count += 1
        if self.aux_count >= 10:
            self.aux_count = 0
            self._out(f'A,-17,-22,{1147 + int(self.rng.integers(-3, 4))},92,-101,{-15 + round(self.plant.omega * 100)}')

    def tick(self):
        """5 ms 진행: (하트비트 검사) → 플랜트 → 샘플이면 제어·D 줄 → 1 s 마다 S 줄."""
        if (self.u_out != 0 or self.prof is not None or self.cl_hold) and self.t - self.last_hb > LINK_TIMEOUT_S:
            self._abort('LINK_TIMEOUT')
        self.plant.step(0 if self.bench else self.u_out, 5)
        self.t += TICK_S
        self.tick_count += 1
        took = 3600 + int(self.rng.integers(0, 500)) if self.div_count == self.div - 1 else 1500
        self.max_tick_us = max(self.max_tick_us, took)
        self.div_count += 1
        if self.div_count >= self.div:
            self.div_count = 0
            self._sample()
        if self.t - self.last_status >= 1.0 - 1e-9:
            self.last_status = self.t
            state = 2 if self.prof is not None else (4 if self.cl_hold else (1 if self.u_out != 0 else 0))
            self.overruns += self.inject_overruns
            self.inject_overruns = 0
            gate = int(5 + 10 * abs(self.u_out) / 255) if self.u_out else 0
            self._out(f'S,{state},1,{self.cal},{self.zero},{self.max_tick_us},{self.overruns},0,{gate},{self.inject_sat}')
            self.max_tick_us = 0


class FakePort:
    """pyserial `Serial` 흉내 (write/read/in_waiting/close). 안에서 FakeBoard 를 실제 시간으로 돌립니다."""

    def __init__(self, board: FakeBoard, latency_ms=(0.3, 0.8)):
        """latency_ms = (lo, hi): PC→보드 명령 도착 지연 균등분포. 실기 왕복 7–9 ms 흉내는 (7, 9)."""
        self.board = board
        self.latency_ms = latency_ms
        self._lrng = np.random.default_rng(12345)
        self._rx = bytearray()           # 보드 → PC
        self._tx = deque()               # (도착 시각, 줄)
        self._inbuf = b''
        self._cv = threading.Condition()
        self._lock = threading.Lock()    # 보드 상태 보호
        self.is_open = True
        self.broken = False              # True 면 read/write 가 예외 (USB 빠짐 흉내)
        board.emit = self._emit
        self._thread = threading.Thread(target=self._run, name='fake-board', daemon=True)
        with self._lock:
            board.boot()
        self._thread.start()

    def _emit(self, line: str):
        with self._cv:
            self._rx += (line + '\r\n').encode()
            self._cv.notify_all()

    def _run(self):
        t0 = time.perf_counter() - self.board.t
        while self.is_open:
            now = time.perf_counter()
            target = now - t0
            n = 0
            while self.board.t + TICK_S <= target and n < 40:
                with self._lock:
                    self._deliver(t0 + self.board.t)
                    self.board.tick()
                n += 1
            if n >= 40:          # 너무 밀리면(일시정지된 프로세스 등) 시계를 따라잡음
                t0 = time.perf_counter() - self.board.t
            with self._lock:
                self._deliver(time.perf_counter())
            nxt = t0 + self.board.t + TICK_S
            dt = nxt - time.perf_counter()
            if dt > 0:
                time.sleep(min(dt, 0.002))

    def _deliver(self, now: float):
        while self._tx and self._tx[0][0] <= now:
            _, line = self._tx.popleft()
            self.board.write(line)

    # ---- pyserial 인터페이스
    def write(self, data: bytes):
        if self.broken or not self.is_open:
            raise OSError('fake port closed')
        self._inbuf += data
        t = time.perf_counter() + 0.001 * self._lrng.uniform(*self.latency_ms)
        with self._lock:
            if self._tx:
                t = max(t, self._tx[-1][0])     # 시리얼은 순서를 지킴
            while b'\n' in self._inbuf:
                raw, self._inbuf = self._inbuf.split(b'\n', 1)
                self._tx.append((t, raw.decode(errors='replace').rstrip('\r')))
        return len(data)

    @property
    def in_waiting(self):
        if self.broken:
            raise OSError('fake port broken')
        return len(self._rx)

    def read(self, n: int = 1, timeout: float = 0.05) -> bytes:
        with self._cv:
            if not self._rx and not self.broken:
                self._cv.wait(timeout)
            if self.broken:
                raise OSError('fake port broken')
            out = bytes(self._rx[:max(n, len(self._rx))])
            del self._rx[:len(out)]
            return out

    def reset_input_buffer(self):
        with self._cv:
            self._rx.clear()

    def close(self):
        self.is_open = False
        with self._cv:
            self._cv.notify_all()

    def locked(self):
        """보드 상태를 직접 건드릴 때 (고장 주입 등)."""
        return self._lock
