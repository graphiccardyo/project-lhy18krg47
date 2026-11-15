"""보드 연결 추상화. 실제 보드(`SerialLink` + pyserial)와 가짜 보드가 같은 인터페이스를 가집니다.

    link.open()                 연결 (가짜 보드는 부팅 메시지 출력)
    link.hello() -> str         HELLO → 'AEROPEND_RL 1.0' (RL 펌웨어가 아니면 LinkError)
    link.send(line)             명령 한 줄 (스레드 안전)
    link.recv(timeout) -> Msg   받은 줄 하나 (없으면 None). 가상시간이면 그동안 보드 시간이 흐름
    link.request(cmd, kinds)    명령을 보내고 kinds 중 하나 또는 ERR 가 올 때까지 기다림
    link.now() -> float         보드와 맞춘 시계 [s] (실시간 = 벽시계, 가상 = 보드 시계)
    link.kick()                 학습기 생존 신호. hb_watchdog_s 동안 없으면 하트비트를 끊어 보드가 1 s 뒤 정지
    link.fast_path              D 줄을 받은 즉시(읽기 스레드 안에서) 호출되는 함수 fn(msg) -> 보낼 명령 | None.
                                결정 샘플의 `A` 를 큐 대기 없이 보내려고 씁니다(실측 왕복 7–9 ms).
    link.hb_enabled             False 면 PING 하트비트를 멈춤 (bench_test 의 하트비트 정지 시험용)
    link.error                  포트 오류 문자열 (없으면 None). 오류 시 recv 는 kind='LINKERR' 를 한 번 돌려줌

종류:
    SerialLink(port)            port = 'COM4' 같은 이름(pyserial 로 엶) 또는 pyserial 흉내 객체(fake_board.FakePort)
    VirtualLink(board)          fake_board.FakeBoard 를 가상시간으로 직접 돌림 (실시간보다 수백 배 빠름)
"""
from __future__ import annotations

import os
import queue
import sys
import threading
import time
from collections import deque

import numpy as np

BAUD = 250000
HB_PERIOD = 0.25


class LinkError(RuntimeError):
    pass


class Msg:
    __slots__ = ('kind', 'raw', 't', 'f', 'kv', 'words', 'sent', 'info')

    def __init__(self, kind, raw, t=0.0):
        self.kind = kind     # D S AUX EVT ERR CFG PIDCFG RLCFG READY OK INFO OTHER LINKERR
        self.raw = raw
        self.t = t           # 받은 시각 (link.now 기준)
        self.f = None        # D/S/AUX: 정수 필드 (머리 글자 제외)
        self.kv = None       # CFG/PIDCFG/RLCFG: key=value 사전 (문자열)
        self.words = None    # EVT/ERR/READY: 공백으로 나눈 낱말 (머리 낱말 제외)
        self.sent = None     # fast_path 가 이 줄에 대해 보낸 명령
        self.info = None     # fast_path 가 남긴 정보 (env 결정 등)

    def __repr__(self):
        return f'Msg({self.kind}, {self.raw!r})'


def parse_line(line: str, t: float = 0.0) -> Msg:
    c = line[:1]
    if c == 'D' and line.startswith('D,'):
        m = Msg('D', line, t)
        try:
            m.f = [int(x) for x in line[2:].split(',')]
        except ValueError:
            m.kind = 'OTHER'
        return m
    if line.startswith('S,'):
        m = Msg('S', line, t)
        try:
            m.f = [int(x) for x in line[2:].split(',')]
        except ValueError:
            m.kind = 'OTHER'
        return m
    if line.startswith('A,'):
        return Msg('AUX', line, t)
    for head, kind in (('EVT ', 'EVT'), ('ERR ', 'ERR'), ('READY ', 'READY')):
        if line.startswith(head):
            m = Msg(kind, line, t)
            m.words = line[len(head):].split()
            return m
    if line in ('CFG BACKUP OK', 'CFG RESTORE OK'):
        return Msg('OK', line, t)
    for head, kind in (('CFG ', 'CFG'), ('PIDCFG ', 'PIDCFG'), ('RLCFG ', 'RLCFG')):
        if line.startswith(head):
            m = Msg(kind, line, t)
            m.kv = dict(p.split('=', 1) for p in line[len(head):].split() if '=' in p)
            return m
    if line.startswith(('IMU', 'CALVAL', 'CAL ', 'ZERO', 'NOTE')):
        return Msg('INFO', line, t)
    return Msg('OTHER', line, t)


class BaseLink:
    is_virtual = False
    name = 'link'

    def __init__(self):
        self.fast_path = None
        self.hb_enabled = True
        self.error = None
        self.fw = None
        self.rx_lines = 0
        self.last_rx = 0.0

    # 하위 클래스가 구현
    def open(self): ...
    def close(self): ...
    def send(self, line: str): ...
    def recv(self, timeout: float): ...
    def now(self) -> float: ...
    def kick(self): ...

    def drain(self, sink=None):
        """큐에 쌓인 줄을 모두 꺼냄 (기다리지 않음)."""
        n = 0
        while True:
            m = self.recv(0.0)
            if m is None:
                return n
            n += 1
            if sink:
                sink(m)

    def request(self, cmd: str, kinds, timeout: float = 2.0, sink=None) -> Msg:
        """cmd 를 보내고 kinds 중 하나 또는 ERR 를 기다림. 다른 줄은 sink 로."""
        if isinstance(kinds, str):
            kinds = (kinds,)
        self.send(cmd)
        t_end = self.now() + timeout
        while True:
            left = t_end - self.now()
            if left <= 0:
                raise LinkError(f'응답 없음: {cmd}')
            m = self.recv(min(left, 0.2))
            if m is None:
                continue
            if m.kind == 'LINKERR':
                raise LinkError(f'포트 오류: {self.error}')
            if m.kind in kinds or m.kind == 'ERR':
                return m
            if sink:
                sink(m)

    def hello(self, timeout: float = 4.0, sink=None, require_rl: bool = True) -> str:
        """HELLO → READY. require_rl 이면 RL 펌웨어(AEROPEND_RL)가 아닐 때 LinkError
        (rl.imu_check 는 False: 실험용 펌웨어 AEROPEND_LAB 도 STOP·GET·STREAM·D 줄 형식이 같음)."""
        t_end = self.now() + timeout
        while self.now() < t_end:
            try:
                m = self.request('HELLO', 'READY', timeout=1.0, sink=sink)
            except LinkError:
                if self.error:
                    raise
                continue
            if m.kind == 'READY':
                fw = ' '.join(m.words)
                if require_rl and not fw.startswith('AEROPEND_RL'):
                    raise LinkError(f'RL 펌웨어가 아닙니다: "{fw}" (AEROPEND_RL 필요)')
                self.fw = fw
                # 부팅 메시지의 READY 였을 수 있으므로, HELLO 응답 끝(CALVAL)까지 읽어 큐를 비움
                self.send('GET')
                t2 = self.now() + 1.5
                seen_rl = False
                while self.now() < t2 and not seen_rl:
                    m2 = self.recv(0.2)
                    if m2 is None:
                        continue
                    if m2.kind == 'RLCFG':
                        seen_rl = True
                    elif sink:
                        sink(m2)
                return fw
        raise LinkError('HELLO 응답 없음')


class SerialLink(BaseLink):
    """pyserial (또는 FakePort) + 읽기 스레드 + 0.25 s PING 하트비트 스레드."""

    def __init__(self, port, watchdog_s: float = 20.0, name: str | None = None):
        super().__init__()
        self.port_arg = port
        self.port = None
        self.watchdog_s = watchdog_s
        self.name = name or (port if isinstance(port, str) else 'fake-realtime')
        self.q = queue.Queue()
        self._wlock = threading.Lock()
        self._run = False
        self._t0 = time.perf_counter()
        self._last_tx = 0.0
        self._last_kick = time.perf_counter()
        self._threads = []
        self._err_reported = False

    def open(self):
        if isinstance(self.port_arg, str):
            import serial   # pyserial
            self.port = serial.Serial(self.port_arg, BAUD, timeout=0.05, write_timeout=0.5)
            self._winio = _WinPortIO(self.port) if os.name == 'nt' else None
        else:
            self.port = self.port_arg
            self._winio = None
        # GIL 전환 간격을 줄여 읽기 스레드가 D 줄에 빨리 반응하게 (기본 5 ms 는 결정 창 20 ms 에 비해 큼)
        sys.setswitchinterval(0.0005)
        self._run = True
        self._last_kick = time.perf_counter()
        for fn, nm in ((self._reader, 'link-reader'), (self._heartbeat, 'link-heartbeat')):
            th = threading.Thread(target=fn, name=nm, daemon=True)
            th.start()
            self._threads.append(th)

    def close(self):
        self._run = False
        for th in self._threads:
            th.join(timeout=1.0)
        try:
            if self.port is not None:
                self.port.close()
        except Exception:
            pass

    def now(self):
        return time.perf_counter() - self._t0

    def kick(self):
        self._last_kick = time.perf_counter()

    def last_tx_time(self):
        return self._last_tx - self._t0

    def _fail(self, e):
        if self.error is None:
            self.error = f'{type(e).__name__}: {e}'
        if not self._err_reported:
            self._err_reported = True
            self.q.put(Msg('LINKERR', self.error, self.now()))

    def send(self, line: str):
        if self.error:
            return
        try:
            with self._wlock:
                data = (line + '\n').encode()
                if self._winio is not None:
                    self._winio.write(data)
                else:
                    self.port.write(data)
                self._last_tx = time.perf_counter()
        except Exception as e:   # 포트 오류
            self._fail(e)

    def _reader(self):
        buf = b''
        fast = None
        while self._run:
            try:
                if self._winio is not None:
                    data = self._winio.read()
                else:
                    n = self.port.in_waiting
                    data = self.port.read(n or 1)
            except Exception as e:
                self._fail(e)
                return
            if not data:
                continue
            buf += data
            if b'\n' not in buf:
                continue
            *lines, buf = buf.split(b'\n')
            now = self.now()
            for raw in lines:
                line = raw.decode('ascii', errors='replace').strip()
                if not line:
                    continue
                m = parse_line(line, now)
                self.rx_lines += 1
                self.last_rx = now
                fast = self.fast_path
                if fast is not None and m.kind == 'D':
                    try:
                        cmd = fast(m)
                    except Exception as e:   # 학습기 쪽 버그가 읽기 스레드를 죽이지 않게
                        m.info = {'fast_error': repr(e)}
                        cmd = None
                    if cmd:
                        self.send(cmd)
                        m.sent = cmd
                self.q.put(m)

    def _heartbeat(self):
        while self._run:
            time.sleep(0.05)
            now = time.perf_counter()
            if (self.hb_enabled and not self.error and now - self._last_tx >= HB_PERIOD
                    and now - self._last_kick < self.watchdog_s):
                self.send('PING')

    def recv(self, timeout: float):
        try:
            if timeout <= 0:
                return self.q.get_nowait()
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None


class _WinPortIO:
    """Windows 에서 pyserial 의 read/write 를 거치지 않는 직렬 입출력.

    pyserial 은 겹친(overlapped) ReadFile/WriteFile 직후 ctypes 로 GetLastError 를 읽어 성공 여부를 판단합니다.
    스레드가 여럿이고 GIL 경합이 심하면(학습 갱신 중) 그 사이 인터프리터의 GIL 대기가 스레드의 마지막 오류
    값을 덮어써, 정상 진행 중인 입출력을 "WriteFile failed" 로 잘못 보고합니다(실기에서 수 초 만에 재현).
    여기서는 GetLastError 를 쓰지 않고 GetOverlappedResult 의 반환값과 바이트 수로만 판단합니다.
    읽기 시간 제한은 pyserial 이 포트에 설정한 값(timeout)을 그대로 따릅니다.
    """

    def __init__(self, ser):
        import ctypes
        from serial import win32
        self.ct, self.w, self.h = ctypes, win32, ser._port_handle
        self.ovr = win32.OVERLAPPED()
        self.ovr.hEvent = win32.CreateEvent(None, 1, 0, None)
        self.ovw = win32.OVERLAPPED()
        self.ovw.hEvent = win32.CreateEvent(None, 0, 0, None)

    def write(self, data: bytes):
        ct, w = self.ct, self.w
        n = w.DWORD()
        w.WriteFile(self.h, data, len(data), ct.byref(n), ct.byref(self.ovw))
        ok = w.GetOverlappedResult(self.h, ct.byref(self.ovw), ct.byref(n), True)
        if not ok or n.value != len(data):
            raise OSError(f'직렬 쓰기 실패 ({n.value}/{len(data)} 바이트)')

    def read(self) -> bytes:
        ct, w = self.ct, self.w
        flags, cs = w.DWORD(), w.COMSTAT()
        if not w.ClearCommError(self.h, ct.byref(flags), ct.byref(cs)):
            raise OSError('직렬 포트 상태 읽기 실패 (연결 끊김?)')
        size = cs.cbInQue or 1
        buf = ct.create_string_buffer(size)
        rc = w.DWORD()
        w.ResetEvent(self.ovr.hEvent)
        w.ReadFile(self.h, buf, size, ct.byref(rc), ct.byref(self.ovr))
        if not w.GetOverlappedResult(self.h, ct.byref(self.ovr), ct.byref(rc), True):
            raise OSError('직렬 읽기 실패 (연결 끊김?)')
        return buf.raw[:rc.value]


class VirtualLink(BaseLink):
    """가짜 보드를 가상시간으로 돌리는 연결. recv/request 가 기다리는 동안만 보드 시간이 흐릅니다.

    명령 도착 지연은 latency_ms=(lo, hi) 균등분포(가상), 하트비트는 보드 시간 0.25 s 마다 자동.
    """
    is_virtual = True

    def __init__(self, board, latency_ms=(0.3, 0.8), seed: int = 7):
        super().__init__()
        from .fake_board import TICK_S
        self.TICK = TICK_S
        self.board = board
        self.name = 'fake-virtual'
        self.latency_ms = latency_ms
        self.rng = np.random.default_rng(seed)
        self.q = deque()
        self.pending = deque()    # (도착 시각, 줄)
        self.last_tx = 0.0
        board.emit = self._on_line

    def open(self):
        self.board.boot()

    def close(self):
        pass

    def now(self):
        return self.board.t

    def kick(self):
        pass

    def last_tx_time(self):
        return self.last_tx

    def _on_line(self, line: str):
        m = parse_line(line, self.board.t)
        self.rx_lines += 1
        self.last_rx = self.board.t
        fast = self.fast_path
        if fast is not None and m.kind == 'D':
            try:
                cmd = fast(m)
            except Exception as e:
                m.info = {'fast_error': repr(e)}
                cmd = None
            if cmd:
                self.send(cmd)
                m.sent = cmd
        self.q.append(m)

    def send(self, line: str):
        t = self.board.t + 0.001 * self.rng.uniform(*self.latency_ms)
        if self.pending:
            t = max(t, self.pending[-1][0])
        self.pending.append((t, line))
        self.last_tx = self.board.t

    def _advance(self):
        b = self.board
        if self.hb_enabled and b.t - self.last_tx >= HB_PERIOD:
            self.pending.append((b.t, 'PING'))
            self.last_tx = b.t
        lim = b.t + self.TICK + 1e-9
        while self.pending and self.pending[0][0] <= lim:
            b.write(self.pending.popleft()[1])
        b.tick()

    def recv(self, timeout: float):
        if self.q:
            return self.q.popleft()
        if timeout <= 0:
            return None
        deadline = self.board.t + timeout
        while not self.q and self.board.t < deadline:
            self._advance()
        return self.q.popleft() if self.q else None
