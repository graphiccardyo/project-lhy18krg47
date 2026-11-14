"""RL 프로토콜 적합성 시험 (docs/RL_PROTOCOL.md, AEROPEND_RL 1.0).

    python -m rl.bench_test --port COM4        실제 보드 (BENCH 1 에서만 — 모터 핀 LOW)
    python -m rl.bench_test --fake             가짜 보드 실시간 (--latency-ms 7 9 로 실기 왕복 지연 흉내)
    python -m rl.bench_test --fake --fast      가짜 보드 가상시간

안전: HELLO 로 RL 펌웨어를 확인하고 `CFG BACKUP` → `BENCH 1` 확인 뒤에만 모터를 돌릴 수 있는 명령
(REF, RUN, U)을 보냅니다. 끝나면(실패·예외 포함) STOP → 설정 되돌림 → `CFG RESTORE`.
ZERO·FACTORY·보정 명령은 보내지 않습니다(에피소드 중 CAL 은 BUSY 확인용으로만, 거부돼야 정상).
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from .link import LinkError

END_EVTS = ('DONE', 'FALLBACK', 'ABORT', 'STOP')


class BenchFail(RuntimeError):
    pass


def check_pwm(lines, sent, dec, lag, dumax, ulo, hi, u0):
    """9필드 D 줄(n 순서)과 결정별로 보낸 du 로 PWM 궤적을 재현해 어긋난 곳 목록을 돌려줌.
    miss 증가는 그 적용 시점의 A 누락으로 해석합니다."""
    bad = []
    u = u0
    miss_prev = 0
    for n, f in enumerate(lines):
        miss = f[7]
        is_app = n >= lag and (n - lag) % dec == 0
        if is_app:
            k = (n - lag) // dec
            if miss == miss_prev + 1:
                pass
            elif miss == miss_prev and k in sent:
                d = max(-dumax, min(dumax, sent[k]))
                u = min(hi, max(ulo, u + d))
            else:
                bad.append(f'n={n}: 적용 시점 miss {miss_prev}->{miss}, A {"있음" if k in sent else "없음"}')
        elif miss != miss_prev:
            bad.append(f'n={n}: 적용 시점이 아닌데 miss {miss_prev}->{miss}')
        miss_prev = miss
        if f[1] != u:
            bad.append(f'n={n}: pwm {f[1]} ≠ 예상 {u}')
    return bad


class Bench:
    def __init__(self, link, log=print, quick=False):
        self.link = link
        if hasattr(link, 'watchdog_s'):
            link.watchdog_s = 1e9     # 시험기는 kick() 을 부르지 않으므로 하트비트 감시견을 끔 (하트비트 시험은 hb_enabled 로)
        self.log = log
        self.quick = quick
        self.results = []       # (이름, ok, 설명, 필수)
        self.bench_ok = False
        self.s_lines = []
        self.th0 = 0.0
        self.init = {}

    # ---------------------------------------------------------------- 도구
    def check(self, name, ok, detail='', required=True):
        self.results.append((name, bool(ok), detail, required))
        tag = 'PASS' if ok else ('FAIL' if required else 'WARN')
        self.log(f'  [{tag}] {name}' + (f' — {detail}' if detail else ''))
        return ok

    def _sink(self, m):
        if m.kind == 'S':
            self.s_lines.append(m.f)

    def req(self, cmd, kinds, timeout=2.0):
        return self.link.request(cmd, kinds, timeout=timeout, sink=self._sink)

    def responses(self, cmd, wait=0.25):
        """cmd 를 보내고 wait 초 동안 D/S/AUX 가 아닌 응답을 모음."""
        self.link.send(cmd)
        out = []
        t_end = self.link.now() + wait
        while self.link.now() < t_end:
            m = self.link.recv(0.05)
            if m is None:
                continue
            if m.kind == 'S':
                self.s_lines.append(m.f)
            elif m.kind not in ('D', 'AUX'):
                out.append(m)
        return out

    def expect_err(self, name, cmd, code):
        m = self.req(cmd, ('ERR', 'RLCFG', 'PIDCFG', 'CFG', 'EVT', 'OK'))
        return self.check(name, m.kind == 'ERR' and m.words == [code], f'{cmd!r} → {m.raw}')

    def lines(self, seconds, kind='D'):
        out = []
        t_end = self.link.now() + seconds
        while self.link.now() < t_end:
            m = self.link.recv(0.05)
            if m is None:
                continue
            if m.kind == 'S':
                self.s_lines.append(m.f)
            if m.kind == kind:
                out.append(m)
        return out

    def motor_cmd(self, cmd):
        if not self.bench_ok:
            raise BenchFail(f'BENCH 1 확인 전에는 보내지 않음: {cmd}')
        self.link.send(cmd)

    def set_rl(self, dec, lag, dumax, ulo, uhi):
        m = self.req(f'RL {dec} {lag} {dumax} {ulo} {uhi}', ('RLCFG', 'ERR'))
        if m.kind != 'RLCFG':
            raise BenchFail(f'RL 설정 실패: {m.raw}')

    def set_safe(self, lo, hi, w):
        m = self.req(f'SAFE {lo} {hi} {w}', ('RLCFG', 'ERR'))
        if m.kind != 'RLCFG':
            raise BenchFail(f'SAFE 설정 실패: {m.raw}')

    def stop(self):
        self.req('STOP', 'EVT')
        self.link.drain(self._sink)

    def hold(self, cdeg, seconds=0.6):
        if not self.bench_ok:
            raise BenchFail('BENCH 1 확인 전')
        m = self.req(f'REF {cdeg}', ('EVT', 'ERR'))
        if m.kind != 'EVT':
            raise BenchFail(f'REF 실패: {m.raw}')
        return self.lines(seconds)

    def episode(self, r, dur_ms, home, dec, responder=None, after=None, pre_send=None, max_s=None, post_s=0.3):
        """RUN RL 하나. responder(k, msg) -> du | None 는 결정 샘플마다 읽기 경로에서 불림.
        after(n, link) 는 9필드 줄마다 주 스레드에서 불림. 돌려줌: dict(lines, sent, evts, post, total)."""
        if not self.bench_ok:
            raise BenchFail('BENCH 1 확인 전')
        st = dict(n=0, sent={}, lat=[])

        def fast(m):
            f = m.f
            if len(f) != 8 or f[5] != 1:
                return None
            n = st['n']
            st['n'] += 1
            if responder is None or n % dec:
                return None
            t0 = time.perf_counter()
            k = n // dec
            du = responder(k, m)
            if du is None:
                return None
            st['sent'][k] = du
            st['lat'].append((time.perf_counter() - t0, self.link.now() - m.t))
            return f'A {du}'

        self.link.fast_path = fast
        try:
            self.link.send(f'RUN RL {r} {dur_ms} {home}')
            if pre_send:
                self.link.send(pre_send)
            lines, evts, post = [], [], []
            total = None
            t_end = self.link.now() + (max_s or dur_ms / 1000 + 3)
            ended = None
            while self.link.now() < t_end:
                m = self.link.recv(0.1)
                if m is None:
                    continue
                if m.kind == 'S':
                    self.s_lines.append(m.f)
                elif m.kind == 'D':
                    if ended is None:
                        if len(m.f) == 8 and m.f[5] == 1:
                            lines.append(m.f)
                            if after:
                                after(len(lines) - 1, m)
                    else:
                        post.append(m.f)
                elif m.kind in ('EVT', 'ERR'):
                    evts.append((len(lines), m))
                    if m.kind == 'EVT' and m.words[0] == 'RUN':
                        total = int(m.words[2].split('=')[1])
                    if m.kind == 'ERR' and total is None:
                        ended = self.link.now()
                        break
                    if m.kind == 'EVT' and m.words[0] in END_EVTS and ended is None:
                        ended = self.link.now()
                        t_end = ended + post_s
        finally:
            self.link.fast_path = None
        return dict(lines=lines, sent=st['sent'], evts=evts, post=post, total=total, lat=st['lat'])

    @staticmethod
    def evt_words(res):
        return [' '.join([m.kind] + m.words) for _, m in res['evts']]

    # ---------------------------------------------------------------- 시험
    def run(self):
        L = self.link
        self.log('== 연결')
        try:
            fw = L.hello()
        except LinkError as e:
            self.check('HELLO → READY AEROPEND_RL', False, str(e))
            return False
        self.check('HELLO → READY AEROPEND_RL', fw.startswith('AEROPEND_RL'), fw)
        self.stop()
        m = self.req('CFG BACKUP', ('OK', 'ERR'))
        if not self.check('CFG BACKUP', m.raw == 'CFG BACKUP OK', m.raw):
            return False
        try:
            self._run_all()
        except BenchFail as e:
            self.check('시험 진행', False, str(e))
        except LinkError as e:
            self.check('시험 진행 (연결)', False, str(e))
        finally:
            self._restore()
        return all(ok for _, ok, _, req in self.results if req)

    def _restore(self):
        self.log('== 되돌림')
        L = self.link
        L.fast_path = None
        L.hb_enabled = True
        try:
            self.stop()
            i = self.init
            if 'cfg' in i:
                c = i['cfg']
                self.req(f'LIM {c["umax"]} {c["alim"]} {c["rev"]}', ('CFG', 'ERR'))
            if 'rl' in i:
                r = i['rl']
                self.req(f'RL {r["dec"]} {r["lag"]} {r["dumax"]} {r["ulo"]} {r["uhi"]}', ('RLCFG', 'ERR'))
                self.req(f'SAFE {r["thlo"]} {r["thhi"]} {r["wmax"]}', ('RLCFG', 'ERR'))
            if 'pid' in i:
                p = i['pid']
                self.req(f'PID {round(float(p["kp"]) * 1000)} {round(float(p["ki"]) * 1000)} '
                         f'{round(float(p["kd"]) * 1000)} {p["ff"]} {p["umin"]}', ('PIDCFG', 'ERR'))
                self.req(f'NOTCH {p["notch"]} {p["zz"]} {p["zp"]}' if p['notch'] != '0' else 'NOTCH 0', ('PIDCFG', 'ERR'))
            if 'cfg' in i and i['cfg'].get('bench') == '0':
                self.req('BENCH 0', ('CFG', 'ERR'))
            m = self.req('CFG RESTORE', ('OK', 'ERR'))
            self.check('CFG RESTORE', m.raw == 'CFG RESTORE OK', m.raw)
        except Exception as e:
            self.check('되돌림', False, repr(e))

    def _run_all(self):
        L = self.link
        # ---- 초기 상태
        L.send('GET')
        got = {}
        t_end = L.now() + 2
        while L.now() < t_end and len(got) < 3:
            m = L.recv(0.1)
            if m is not None and m.kind in ('CFG', 'PIDCFG', 'RLCFG'):
                got[m.kind] = m.kv
        if len(got) < 3:
            raise BenchFail(f'GET 응답 부족: {list(got)}')
        self.init = dict(cfg=got['CFG'], pid=got['PIDCFG'], rl=got['RLCFG'])
        rl0 = got['RLCFG']
        self.check('GET 끝에 RLCFG', True, ' '.join(f'{k}={v}' for k, v in rl0.items()))
        self.check('RLCFG 재부팅 기본값 (dec=2 lag=1 dumax=10 ulo=0 uhi=255 thlo=-2000 thhi=6000 wmax=40000)',
                   [rl0.get(k) for k in ('dec', 'lag', 'dumax', 'ulo', 'uhi', 'thlo', 'thhi', 'wmax')]
                   == ['2', '1', '10', '0', '255', '-2000', '6000', '40000'],
                   '재부팅 직후가 아니면 다를 수 있음', required=False)
        m = self.req('BENCH 1', ('CFG', 'ERR'))
        self.bench_ok = m.kind == 'CFG' and m.kv.get('bench') == '1'
        if not self.check('BENCH 1', self.bench_ok, m.raw):
            raise BenchFail('벤치 모드가 아니면 진행하지 않음')
        self.req('LIM 210 18000 0', ('CFG', 'ERR'))
        d = self.lines(0.5)
        self.th0 = float(np.mean([x.f[2] for x in d])) / 100 if d else 0.0
        th0c = int(round(self.th0 * 100))
        self.log(f'  (현재 θ = {self.th0:.2f}°, 이 각도를 기준·home 으로 씀)')

        self.log('== RL·SAFE 범위 검사')
        for cmd in ('RL 0 1 10 0 255', 'RL 21 1 10 0 255', 'RL 2 0 10 0 255', 'RL 2 3 10 0 255', 'RL 2 1 0 0 255',
                    'RL 2 1 256 0 255', 'RL 2 1 10 100 100', 'RL 2 1 10 120 100', 'RL 2 1 10 0 256',
                    'RL 2 1 10 0', 'RL 2 1 10 0 255 9', 'RL 2 1 x 0 255'):
            self.expect_err(f'{cmd} → BAD_ARGS', cmd, 'BAD_ARGS')
        out = self.responses('RL 4 4 20 30 200')
        self.check('RL 성공 → RLCFG 한 줄만', [m.kind for m in out] == ['RLCFG'] and
                   out[0].kv.get('dec') == '4' and out[0].kv.get('lag') == '4' and out[0].kv.get('dumax') == '20'
                   and out[0].kv.get('ulo') == '30' and out[0].kv.get('uhi') == '200', ' | '.join(m.raw for m in out))
        for cmd in ('SAFE 100 100 1000', 'SAFE 200 100 1000', 'SAFE -18001 100 1000', 'SAFE 0 18001 1000',
                    'SAFE 0 100 0', 'SAFE 0 100 200001', 'SAFE 0 100'):
            self.expect_err(f'{cmd} → BAD_ARGS', cmd, 'BAD_ARGS')
        out = self.responses(f'SAFE {th0c - 3000} {th0c + 4000} 50000')
        self.check('SAFE 성공 → RLCFG 한 줄만', [m.kind for m in out] == ['RLCFG'] and
                   out[0].kv.get('thlo') == str(th0c - 3000) and out[0].kv.get('wmax') == '50000',
                   ' | '.join(m.raw for m in out))
        out = self.responses('HELLO', 0.4)
        self.check('HELLO 는 RLCFG 를 내지 않음', 'RLCFG' not in [m.kind for m in out] and
                   any(m.kind == 'READY' for m in out), ' | '.join(m.raw for m in out))

        self.log('== RUN RL / A 오류')
        self.req('PID 0 0 0 140 40', ('PIDCFG', 'ERR'))
        self.expect_err('RUN RL 인자 부족 → BAD_ARGS (NO_PID 보다 먼저)', 'RUN RL 2000 1000', 'BAD_ARGS')
        self.expect_err('PID 이득 0 → NO_PID', f'RUN RL {th0c} 1000 {th0c}', 'NO_PID')
        self.req('PID 900 7000 20 140 40', ('PIDCFG', 'ERR'))
        self.expect_err('dur 0 → BAD_ARGS', f'RUN RL {th0c} 0 {th0c}', 'BAD_ARGS')
        self.expect_err('|r| > 17000 → BAD_ARGS', f'RUN RL 17001 1000 {th0c}', 'BAD_ARGS')
        self.expect_err('|home| > 17000 → BAD_ARGS', f'RUN RL {th0c} 1000 -17001', 'BAD_ARGS')
        self.expect_err('dur 2 ms (0샘플) → BAD_DURATION', f'RUN RL {th0c} 2 {th0c}', 'BAD_DURATION')
        self.expect_err('dur 1300 s → BAD_DURATION', f'RUN RL {th0c} 1300000 {th0c}', 'BAD_DURATION')
        self.expect_err('에피소드 밖 A → NOT_RL', 'A 3', 'NOT_RL')
        self.expect_err('A 숫자 아님 → BAD_ARGS (NOT_RL 보다 먼저)', 'A x', 'BAD_ARGS')
        for p in ('STEP 100 120 100 100 100', 'STAIR 50 100 10 100', 'CHIRP 150 20 100 5000 0 1000 1',
                  'PRBS 150 20 50 0 1000', f'RELAY {th0c} 20 50 1000 1000'):
            self.expect_err(f'삭제된 RUN {p.split()[0]} → UNKNOWN_PROFILE', 'RUN ' + p, 'UNKNOWN_PROFILE')

        self.log('== 에피소드 기본 (A 없음, 대기 상태에서 시작, 에피소드 중 REF)')
        self.stop()
        self.set_rl(2, 1, 10, 40, 210)
        self.set_safe(-18000, 18000, 200000)
        ref_new = th0c + 500
        sent_ref = {}

        def after(n, m):
            if n == 50 and not sent_ref:
                L.send(f'REF {ref_new}')
                sent_ref['n'] = n
        res = self.episode(th0c + 1000, 1000, th0c, 2, after=after)
        ln = res['lines']
        self.check('EVT RUN RL total=100 ts_ms=10', 'EVT RUN RL total=100 ts_ms=10' in self.evt_words(res),
                   str(self.evt_words(res)[:2]))
        self.check('9필드 D 줄 100개 (n = 0…99)', len(ln) == 100, f'{len(ln)}개')
        self.check('시작 PWM = 0 을 u_lo 로 자른 40', ln and ln[0][1] == 40, f'첫 pwm {ln[0][1] if ln else None}')
        self.check('A 없으면 PWM 유지', ln and all(f[1] == 40 for f in ln))
        self.check('miss = 적용 시점 수 50 (lag 1: n=1,3,…,99)', ln and ln[-1][7] == 50, f'miss {ln[-1][7] if ln else None}')
        ev = self.evt_words(res)
        self.check('EVT DONE → EVT REF home', 'EVT DONE' in ev and f'EVT REF {th0c}' in ev
                   and ev.index(f'EVT REF {th0c}') > ev.index('EVT DONE'), str(ev))
        self.check('에피소드 중 REF → EVT REF, ref 필드만 바뀜', f'EVT REF {ref_new}' in ev and
                   ln and all(f[6] == ref_new for f in ln[60:]) and all(f[6] == th0c + 1000 for f in ln[:45]),
                   f'ref {ln[40][6] if ln else ""} → {ln[-1][6] if ln else ""}')
        self.check('종료 뒤 D 줄 phase 0·8필드', res['post'] and all(len(f) == 7 and f[5] == 0 for f in res['post']),
                   f'{len(res["post"])}줄, 필드 {sorted(set(len(f) + 1 for f in res["post"]))}')
        s = self.lines(1.2, 'S')
        self.check('종료 뒤 S state 4 (PID 유지)', s and s[-1].f[0] == 4, s[-1].raw if s else '없음')
        self.stop()
        self.set_rl(2, 2, 10, 40, 210)
        res = self.episode(th0c, 1000, th0c, 2)
        ln = res['lines']
        self.check('lag 2: miss 49 (첫 적용 n = lag = 2)', ln and ln[-1][7] == 49 and ln[1][7] == 0 and ln[2][7] == 1,
                   f'miss {ln[-1][7] if ln else None}')

        self.log('== A 반영·자르기')
        self.stop()
        self.req('LIM 180 18000 0', ('CFG', 'ERR'))
        self.set_rl(2, 2, 10, 40, 210)
        res = self.episode(th0c, 1000, th0c, 2, responder=lambda k, m: 30 if k < 22 else -300)
        ln = res['lines']
        bad = check_pwm(ln, res['sent'], 2, 2, 10, 40, 180, 40)
        self.check('A 반영 (±du_max 로 자름, [u_lo, min(u_hi, LIM)])', ln and not bad, '; '.join(bad[:4]))
        self.check('LIM 180 이 u_hi 210 보다 먼저 (최대 180)', ln and max(f[1] for f in ln) == 180,
                   f'최대 {max(f[1] for f in ln) if ln else None}')
        self.check('u_lo 40 아래로 안 내려감', ln and min(f[1] for f in ln) == 40 and ln[-1][1] == 40)
        self.req('LIM 210 18000 0', ('CFG', 'ERR'))

        self.log('== lag 타이밍 (dec 4, lag 3)')
        self.stop()
        self.set_rl(4, 3, 10, 40, 210)
        res = self.episode(th0c, 800, th0c, 4, responder=lambda k, m: 5)
        ln = res['lines']
        bad = check_pwm(ln, res['sent'], 4, 3, 10, 40, 210, 40)
        chg = [n for n in range(1, len(ln)) if ln[n][1] != ln[n - 1][1]]
        self.check('PWM 은 n ≡ 3 (mod 4) 에서만 바뀜', ln and chg and all(n % 4 == 3 for n in chg) and not bad,
                   f'바뀐 n: {chg[:6]} … ; {"; ".join(bad[:3])}')

        self.log('== miss 계수 · 늦은 A · n=0 전 A')
        self.stop()
        self.set_rl(2, 2, 10, 40, 210)
        res = self.episode(th0c, 1000, th0c, 2, responder=lambda k, m: 3 if k % 2 == 0 else None)
        ln = res['lines']
        apps = [n for n in range(len(ln)) if n >= 2 and (n - 2) % 2 == 0]
        exp_miss = sum(1 for n in apps if ((n - 2) // 2) % 2 == 1)
        bad = check_pwm(ln, res['sent'], 2, 2, 10, 40, 210, 40)
        self.check('짝수 결정만 응답 → miss = 응답 안 한 적용 시점 수', ln and ln[-1][7] == exp_miss and not bad,
                   f'miss {ln[-1][7] if ln else None} (예상 {exp_miss}) {"; ".join(bad[:3])}')
        self.stop()
        self.set_rl(4, 1, 10, 40, 210)

        def late(n, m):
            if n % 4 == 2:
                L.send('A 7')     # 적용 시점(n=4k+1) 지난 뒤 → 다음 결정 샘플에서 버려져야 함
        res = self.episode(th0c, 800, th0c, 4, after=late, pre_send='A 9')
        ln = res['lines']
        n_app = sum(1 for n in range(len(ln)) if n >= 1 and (n - 1) % 4 == 0)
        self.check('늦은 A 와 n=0 전 A 는 버림 (PWM 그대로, 모든 적용 시점 miss)',
                   ln and all(f[1] == 40 for f in ln) and ln[-1][7] == n_app,
                   f'pwm {sorted(set(f[1] for f in ln))}, miss {ln[-1][7] if ln else None}/{n_app}')

        self.log('== 에피소드 중 명령 규칙 (BUSY)')
        self.stop()
        self.set_rl(2, 2, 10, 40, 210)
        busy = {}

        def busy_probe(n, m):
            if n == 10:
                for cmd in ('RL 2 1 10 40 210', 'SAFE -1000 7000 60000', f'RUN RL {th0c} 1000 {th0c}',
                            f'RUN CLSTEP {th0c} {th0c} 100 100 100', 'STREAM 2', 'CAL', 'BENCH 0',
                            'PID 900 7000 20 140 40', 'LIM 210 18000 0', 'GET', 'PING'):
                    busy[cmd] = self.responses(cmd, 0.12 if not L.is_virtual else 0.05)
                busy['_done'] = True
        res = self.episode(th0c, 3000, th0c, 2, responder=lambda k, m: 0, after=busy_probe, max_s=8)
        for cmd in ('RL 2 1 10 40 210', 'SAFE -1000 7000 60000', f'RUN RL {th0c} 1000 {th0c}',
                    f'RUN CLSTEP {th0c} {th0c} 100 100 100', 'STREAM 2', 'CAL', 'BENCH 0'):
            out = busy.get(cmd, [])
            self.check(f'에피소드 중 {cmd.split()[0]}{" " + cmd.split()[1] if cmd.startswith("RUN") else ""} → ERR BUSY 만',
                       [m.raw for m in out] == ['ERR BUSY'], ' | '.join(m.raw for m in out))
        self.check('에피소드 중 PID → PIDCFG', [m.kind for m in busy.get('PID 900 7000 20 140 40', [])] == ['PIDCFG'])
        self.check('에피소드 중 LIM → CFG', [m.kind for m in busy.get('LIM 210 18000 0', [])] == ['CFG'])
        g = [m.kind for m in busy.get('GET', [])]
        self.check('에피소드 중 GET → CFG CALVAL PIDCFG RLCFG', g[:1] == ['CFG'] and g[-1:] == ['RLCFG'], str(g))
        self.check('에피소드 중 PING → 응답 없음', busy.get('PING', None) == [])
        self.check('BUSY 명령 뒤에도 에피소드가 끝까지 감 (EVT DONE)', 'EVT DONE' in self.evt_words(res),
                   str(self.evt_words(res)))
        # 위 에피소드는 3 s 로 끝까지 감. 이번엔 U 로 끝내기
        self.stop()
        stop_u = {}

        def u_probe(n, m):
            if n == 20 and not stop_u:
                stop_u['x'] = 1
                L.send('U 0')
        res = self.episode(th0c, 3000, th0c, 2, responder=lambda k, m: 0, after=u_probe)
        ev = self.evt_words(res)
        self.check('에피소드 중 U → EVT STOP 후 수동 출력', 'EVT STOP' in ev and res['post'] and
                   all(len(f) == 6 and f[5] == 0 for f in res['post']) and len(res['lines']) < 60,
                   f'{ev}, 뒤 D 줄 필드 {sorted(set(len(f) + 1 for f in res["post"]))}')
        self.stop()

        self.log('== 끊김 없는 전환 (REF 유지 → RL → DONE → PID)')
        self.set_rl(2, 2, 10, 40, 210)
        h = self.hold(th0c, 1.5)
        u_hold = h[-1].f[1] if h else None
        res = self.episode(th0c, 600, th0c, 2, responder=lambda k, m: 10 if k < 8 else 0)
        ln = res['lines']
        u_start_exp = min(210, max(40, u_hold)) if u_hold is not None else None
        self.check('시작 PWM = 직전 PID 출력 (u_lo/u_hi 로 자름)', ln and abs(ln[0][1] - u_start_exp) <= 1,
                   f'유지 {u_hold} → 시작 {ln[0][1] if ln else None}')
        u_last = ln[-1][1] if ln else None
        p0 = res['post'][0][1] if res['post'] else None
        self.check('DONE → PID 첫 출력 ≈ 마지막 RL PWM (I = u − ff)', p0 is not None and abs(p0 - u_last) <= 3,
                   f'RL 마지막 {u_last} → PID 첫 {p0}')
        self.stop()

        self.log('== SAFE → FALLBACK')
        h = self.hold(th0c, 1.0)
        u_hold = h[-1].f[1] if h else 0
        self.set_safe(th0c + 1000, th0c + 5000, 200000)     # θ 가 범위 밖 → 즉시 FALLBACK
        res = self.episode(th0c + 2000, 1000, th0c, 2, responder=lambda k, m: 0)
        ev = self.evt_words(res)
        self.check('θ < th_lo → EVT FALLBACK THETA → EVT REF home', 'EVT FALLBACK THETA' in ev and f'EVT REF {th0c}' in ev,
                   str(ev))
        post = res['post']
        self.check('FALLBACK 뒤 PID 유지 (phase 0·8필드) 와 출력 연속', post and all(len(f) == 7 and f[5] == 0 for f in post)
                   and abs(post[0][1] - max(40, min(210, u_hold))) <= 3,
                   f'유지 {u_hold} → FALLBACK 뒤 첫 {post[0][1] if post else None}')
        s = self.lines(1.2, 'S')
        self.check('FALLBACK 뒤 S state 4', s and s[-1].f[0] == 4, s[-1].raw if s else '없음')
        self.stop()
        self.set_safe(-18000, 18000, 1)
        res = self.episode(th0c, 1000, th0c, 2, responder=lambda k, m: 0)
        ev = self.evt_words(res)
        self.check('|ω| > w_max → EVT FALLBACK OMEGA', 'EVT FALLBACK OMEGA' in ev, str(ev),
                   required=True)
        self.stop()
        self.set_safe(th0c - 3000, th0c + 4000, 50000)

        self.log('== 하트비트 정지')
        self.hold(th0c, 0.3)
        L.hb_enabled = False
        t0 = L.now()
        got = None
        while L.now() - t0 < 3:
            m = L.recv(0.1)
            if m is not None and m.kind == 'EVT' and m.words[:2] == ['ABORT', 'LINK_TIMEOUT']:
                got = m.t - L.last_tx_time()      # 마지막 송신(PING 포함)부터
                break
        L.hb_enabled = True
        self.check('REF 유지 중 1 s 무명령 → EVT ABORT LINK_TIMEOUT', got is not None and 0.95 <= got <= 1.3,
                   f'{got:.2f} s' if got else '없음')
        self.link.drain(self._sink)
        L.hb_enabled = False
        res = self.episode(th0c, 5000, th0c, 2, max_s=4)
        L.hb_enabled = True
        ev = self.evt_words(res)
        self.check('에피소드 중 PC 멈춤(A·PING 없음) → LINK_TIMEOUT', 'EVT ABORT LINK_TIMEOUT' in ev and 80 <= len(res['lines']) <= 160,
                   f'{ev}, {len(res["lines"])} 샘플 뒤')
        self.stop()

        self.log('== 결정 위상 (REF 유지 중 D 줄이 섞여도 9필드만 셈)')
        self.set_rl(2, 2, 10, 40, 210)
        self.hold(th0c, 0.5)
        res = self.episode(th0c, 2000, th0c, 2, responder=lambda k, m: (1 if k % 2 else -1))
        ln = res['lines']
        bad = check_pwm(ln, res['sent'], 2, 2, 10, 40, 210, ln[0][1] if ln else 0)
        # 위상이 어긋나면 거의 전부 누락되므로, 지터로 인한 드문 누락(≤ 2%)은 허용
        self.check('위상 일치: miss ≤ 2%, PWM 재현', ln and ln[-1][7] <= max(1, len(ln) // 100) and not bad,
                   f'miss {ln[-1][7] if ln else None}; {"; ".join(bad[:3])}')
        self.stop()

        mt = [f[4] for f in self.s_lines if len(f) >= 9]
        self.check('S 줄 maxTick_us < 5000', mt and max(mt) < 5000, f'최대 {max(mt) if mt else None} µs ({len(mt)}줄)')

        self.latency()

    def latency(self):
        """30 s 에피소드: numpy 정책 추론 포함 D 수신 → A 송신 시간 분포, miss 비율."""
        from .config import Config
        from .env import EpisodeEnv, obs_dim
        from .sac import SAC
        L = self.link
        cfg = Config()
        dur = 10 if self.quick else 30
        self.log(f'== 왕복 지연 측정 ({dur} s 에피소드, numpy SAC 정책 추론 포함, RL {cfg.rl.dec}/{cfg.rl.lag})')
        th0c = int(round(self.th0 * 100))
        self.set_rl(cfg.rl.dec, cfg.rl.lag, cfg.rl.du_max, cfg.rl.u_lo, cfg.rl.u_hi)
        self.set_safe(-18000, 18000, 200000)
        sac = SAC(obs_dim(cfg), 1, cfg.sac)       # (무거운 생성은 REF 유지 전에)
        env = EpisodeEnv(cfg)
        self.hold(th0c, 0.5)
        env.arm(lambda o: sac.policy.act(o)[0], cfg.rl.dec, cfg.rl.du_max)
        lat = []

        def fast(m):
            t0 = time.perf_counter()
            cmd = env.on_D(m)
            if cmd is not None:
                lat.append((time.perf_counter() - t0, L.now() - m.t))
            return cmd
        L.fast_path = fast
        try:
            L.send(f'RUN RL {th0c} {dur * 1000} {th0c}')
            done = False
            t_end = L.now() + dur + 4
            evs = []
            while L.now() < t_end and not done:
                m = L.recv(0.1)
                if m is not None and m.kind in ('EVT', 'ERR'):
                    evs.append(m.raw)
                    if m.kind == 'EVT' and m.words[0] in END_EVTS:
                        done = m.words[0] == 'DONE'
                        break
                    if m.kind == 'ERR':
                        break
        finally:
            L.fast_path = None
            env.active = False
        s = env.summary()
        comp = np.array([a for a, _ in lat]) * 1000
        tot = np.array([b for _, b in lat]) * 1000
        if len(comp):
            self.log(f'  결정 {len(comp)}회, 정책+관측 계산 [ms] p50 {np.percentile(comp, 50):.3f} '
                     f'p95 {np.percentile(comp, 95):.3f} p99 {np.percentile(comp, 99):.3f} 최대 {comp.max():.3f}')
            if not L.is_virtual:
                self.log(f'  D 수신 → A 송신 [ms] p50 {np.percentile(tot, 50):.3f} p95 {np.percentile(tot, 95):.3f} '
                         f'p99 {np.percentile(tot, 99):.3f} 최대 {tot.max():.3f}')
        self.log(f'  miss {s["miss"]} / {s["decisions"]} ({s["miss_frac"]:.2%})')
        self.check('왕복 지연 에피소드 완료 (EVT DONE)', done and s['decisions'] >= dur * 45,
                   f'{s["decisions"]} 결정, {evs}')
        # 순수 추론 시간 (스레드 경합 없이)
        o = np.zeros(obs_dim(cfg), np.float32)
        tt = []
        for _ in range(2000):
            t0 = time.perf_counter()
            sac.policy.act(o)
            tt.append(time.perf_counter() - t0)
        p99 = float(np.percentile(tt, 99)) * 1000
        med = float(np.median(tt)) * 1000
        self.check('numpy 정책 추론 1회 < 0.2 ms (중앙값)', med < 0.2, f'중앙 {med:.3f} ms, p99 {p99:.3f} ms')
        self.check('miss 비율 ≤ 1%', s['miss_frac'] <= 0.01, f'{s["miss_frac"]:.2%}')
        self.stop()

    def summary(self):
        req = [r for r in self.results if r[3]]
        npass = sum(1 for r in req if r[1])
        warn = [r for r in self.results if not r[3] and not r[1]]
        self.log(f'\n결과: 필수 {npass}/{len(req)} 통과' + (f', 경고 {len(warn)}' if warn else ''))
        for name, ok, det, r in self.results:
            if r and not ok:
                self.log(f'  실패: {name} — {det}')
        return npass == len(req)


def make_link(args):
    if args.fake:
        from .config import PlantCfg
        from .fake_board import FakeBoard, FakePort
        from .link import SerialLink, VirtualLink
        board = FakeBoard(PlantCfg(), seed=3)
        lat = tuple(args.latency_ms) if args.latency_ms else (0.3, 0.8)
        if args.fast:
            return VirtualLink(board, latency_ms=lat)
        return SerialLink(FakePort(board, latency_ms=lat), name='fake-realtime')
    if not args.port:
        raise SystemExit('--port COMx 또는 --fake 가 필요합니다')
    from .link import SerialLink
    return SerialLink(args.port)


def run_bench(link, log=print, quick=False):
    link.open()
    try:
        b = Bench(link, log, quick=quick)
        b.run()
        return b.summary(), b
    finally:
        link.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description='RL 프로토콜 적합성 시험 (BENCH 1)')
    ap.add_argument('--port')
    ap.add_argument('--fake', action='store_true')
    ap.add_argument('--fast', action='store_true')
    ap.add_argument('--latency-ms', type=float, nargs=2)
    ap.add_argument('--quick', action='store_true', help='지연 측정을 10 s 로')
    args = ap.parse_args(argv)
    ok, _ = run_bench(make_link(args), quick=args.quick)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
