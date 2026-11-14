"""학습기 로컬 웹 대시보드 (표준 라이브러리만: http.server, threading, json).

    from rl.dashboard import serve
    httpd = serve(trainer, port=8770)      # 데몬 스레드로 서버를 띄우고 바로 돌아옵니다

- 127.0.0.1 에만 바인드합니다(외부 노출 없음).
- GET  /               → rl/dashboard.html
- GET  /api/status     → trainer.status() (NaN/inf 는 null)
- POST /api/command    본문 {"cmd": "start|pause|resume|stop"} → trainer.command(cmd) 결과 {ok, msg}
                       그 외 cmd 는 400. 모터를 움직일 수 있는 명령이므로 다른 호스트 이름(DNS 재바인딩)과
                       JSON 이 아닌 본문(교차 출처 폼 전송)은 거부합니다.
"""
from __future__ import annotations

import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dashboard.html')
COMMANDS = ('start', 'pause', 'resume', 'stop')
ALLOWED_HOSTS = ('127.0.0.1', 'localhost', '[::1]')
MAX_BODY = 4096


def _clean(o):
    """JSON 으로 보낼 수 있게 바꿉니다: NaN/inf → None, numpy 값 → 파이썬 값, 나머지 → str."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if o is None or isinstance(o, (bool, int, str)):
        return o
    if hasattr(o, 'item'):          # numpy 스칼라
        try:
            return _clean(o.item())
        except Exception:
            pass
    return str(o)


class _Handler(BaseHTTPRequestHandler):
    server_version = 'AeroRLDashboard/1.0'

    def log_message(self, fmt, *args):   # 콘솔을 어지럽히지 않도록 접속 로그 끔
        pass

    # ------------------------------------------------------------ 공통
    def _host_ok(self) -> bool:
        host = (self.headers.get('Host') or '').lower()
        if host.startswith('['):
            name = host[:host.find(']') + 1] if ']' in host else host
        else:
            name = host.split(':')[0]
        return name in ALLOWED_HOSTS

    def _send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def _json(self, code: int, obj):
        body = json.dumps(_clean(obj), ensure_ascii=False, allow_nan=False).encode('utf-8')
        self._send(code, body, 'application/json; charset=utf-8')

    # ------------------------------------------------------------ GET
    def do_GET(self):
        if not self._host_ok():
            return self._json(403, {'ok': False, 'msg': '허용되지 않은 호스트'})
        path = urlsplit(self.path).path
        trainer = self.server.trainer
        if path in ('/', '/index.html'):
            try:
                with open(HTML_PATH, 'rb') as f:
                    body = f.read()
            except OSError:
                return self._send(500, 'dashboard.html 을 읽을 수 없습니다'.encode('utf-8'), 'text/plain; charset=utf-8')
            return self._send(200, body, 'text/html; charset=utf-8')
        if path == '/api/status':
            try:
                return self._json(200, trainer.status())
            except Exception as e:   # 학습기 쪽 오류도 화면에는 알림
                return self._json(500, {'error': f'{type(e).__name__}: {e}'})
        return self._json(404, {'ok': False, 'msg': '없는 경로'})

    # ------------------------------------------------------------ POST
    def do_POST(self):
        if not self._host_ok():
            return self._json(403, {'ok': False, 'msg': '허용되지 않은 호스트'})
        if urlsplit(self.path).path != '/api/command':
            return self._json(404, {'ok': False, 'msg': '없는 경로'})
        ctype = (self.headers.get('Content-Type') or '').split(';')[0].strip().lower()
        if ctype != 'application/json':
            return self._json(400, {'ok': False, 'msg': 'Content-Type 은 application/json 이어야 합니다'})
        try:
            n = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            n = -1
        if n <= 0 or n > MAX_BODY:
            return self._json(400, {'ok': False, 'msg': '본문 길이가 잘못됐습니다'})
        try:
            data = json.loads(self.rfile.read(n).decode('utf-8'))
        except (ValueError, UnicodeDecodeError):
            return self._json(400, {'ok': False, 'msg': 'JSON 을 읽을 수 없습니다'})
        cmd = data.get('cmd') if isinstance(data, dict) else None
        if cmd not in COMMANDS:
            return self._json(400, {'ok': False, 'msg': f'알 수 없는 명령: {cmd}'})
        ok, msg = self.server.trainer.command(cmd)
        return self._json(200, {'ok': bool(ok), 'msg': str(msg)})


def serve(trainer, port: int = 8770, host: str = '127.0.0.1'):
    """대시보드 서버를 데몬 스레드로 띄우고 ThreadingHTTPServer 를 돌려줍니다 (종료: httpd.shutdown())."""
    if host != '127.0.0.1':
        raise ValueError('대시보드는 127.0.0.1 에만 바인드합니다')
    httpd = ThreadingHTTPServer((host, port), _Handler)
    httpd.daemon_threads = True
    httpd.trainer = trainer
    threading.Thread(target=httpd.serve_forever, name='rl-dashboard', daemon=True).start()
    return httpd
