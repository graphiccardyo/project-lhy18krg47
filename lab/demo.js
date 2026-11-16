// 시연 UI — 시연용 펌웨어(AEROPEND_DEMO 1.0, docs/DEMO.md)와 Web Serial 로 대화합니다.
// 보드 혼자 PID 또는 고정 RL 정책으로 기준 각도를 유지하고, 이 페이지는 제어기·기준·PID 이득을 보내고 그래프를 그립니다.
// 연결된 동안 250 ms 마다 PING (보드는 1 s 동안 명령이 없으면 정지).
'use strict';

const BAUD = 250000;
const enc = new TextEncoder();
const $ = (id) => document.getElementById(id);
const S = { link: null, connected: false, isSim: false, mode: 0, wantRl: false, pidSynced: false, hold: false,
  d: { t: [], th: [], ref: [], u: [], mode: [], a: [], tha: [] }, lastD: null, pidcfg: '', fw: '' };
const KEEP_S = 1800;                      // CSV 용 기록 (30 분, 50 Hz)

function log(msg, cls) {
  const el = $('log');
  const div = document.createElement('div');
  if (cls) div.className = cls;
  div.textContent = msg;
  el.appendChild(div);
  while (el.childNodes.length > 400) el.removeChild(el.firstChild);
  el.scrollTop = el.scrollHeight;
}
function banner(msg) {
  const b = $('banner');
  if (!msg) { b.classList.remove('show'); return; }
  b.textContent = msg; b.classList.add('show');
  clearTimeout(banner.t); banner.t = setTimeout(() => b.classList.remove('show'), 10000);
}
function dot(id, cls) { $(id).className = 'dot' + (cls ? ' ' + cls : ''); }

// ------------------------------------------------------------------ 전송
class SerialLink {
  async open() {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: BAUD, bufferSize: 1 << 16 });
    this.writer = this.port.writable.getWriter();
    this.queue = Promise.resolve();
    this.alive = true;
    this.readLoop();
  }
  async readLoop() {
    const dec = new TextDecoder();
    let buf = '';
    try {
      this.reader = this.port.readable.getReader();
      for (;;) {
        const { value, done } = await this.reader.read();
        if (done) break;
        buf += dec.decode(value, { stream: true });
        let i;
        while ((i = buf.indexOf('\n')) >= 0) {
          const line = buf.slice(0, i).replace(/\r$/, '');
          buf = buf.slice(i + 1);
          if (line) this.onLine(line);
        }
        if (buf.length > 4096) buf = '';
      }
    } catch (e) {
      log('수신 오류: ' + e.message, 'e');
    } finally {
      try { this.reader.releaseLock(); } catch (_) {}
      if (this.alive) { this.alive = false; this.onClose && this.onClose(); }
    }
  }
  send(line) {
    this.queue = this.queue.then(() => this.writer.write(enc.encode(line + '\n'))).catch((e) => log('송신 오류: ' + e.message, 'e'));
    return this.queue;
  }
  async close() {
    this.alive = false;
    try { await this.reader.cancel(); } catch (_) {}
    try { await this.queue; this.writer.releaseLock(); } catch (_) {}
    try { await this.port.close(); } catch (_) {}
  }
}

class SimLink {
  async open() { this.dev = new DemoSimDevice(); this.dev.onLine = (l) => this.onLine(l); }
  send(line) { setTimeout(() => this.dev && this.dev.write(line), 2); return Promise.resolve(); }
  async close() { if (this.dev) this.dev.close(); this.dev = null; }
}

function send(line, quiet = false) {
  if (!S.connected) return false;
  if (!quiet) log('> ' + line, 'tx');
  S.link.send(line);
  return true;
}

async function connect(sim) {
  if (S.connected) { await disconnect(); return; }
  if (!sim && !('serial' in navigator)) { banner('이 브라우저는 Web Serial을 지원하지 않습니다. Edge 또는 Chrome으로 여세요. (시뮬레이터는 사용 가능)'); return; }
  const link = sim ? new SimLink() : new SerialLink();
  link.onLine = onLine;
  link.onClose = () => { log('포트가 닫혔습니다', 'e'); cleanup(); };
  try {
    await link.open();
  } catch (e) {
    if (e.name !== 'NotFoundError') banner('연결 실패: ' + e.message + ' — 다른 프로그램(실험 UI, 시리얼 모니터, 학습기)이 포트를 쓰고 있지 않은지 확인하세요.');
    return;
  }
  Object.assign(S, { link, connected: true, isSim: sim, pidSynced: false, fw: '', hold: false, mode: 0 });
  for (const k in S.d) S.d[k] = [];
  $('connectBtn').textContent = sim ? '시뮬레이터 종료' : '연결 해제';
  $('simBtn').disabled = true;
  if (sim) { $('connectBtn').disabled = false; }
  dot('dotLink', 'warn'); $('txtLink').textContent = sim ? '시뮬레이터' : '보드 응답 대기…';
  log(sim ? '시뮬레이터 시작' : '포트 열림 — 보드 재부팅 대기', 'ok');
  clearTimeout(S.helloTimer);
  S.helloTimer = setTimeout(() => send('HELLO'), sim ? 300 : 2600);   // 포트를 열면 우노가 재부팅됨
  clearInterval(S.pingTimer);
  S.pingTimer = setInterval(() => { if (S.connected) S.link.send('PING'); }, 250);
  updateUi();
}

async function disconnect() {
  if (S.connected) { send('STOP'); await new Promise((r) => setTimeout(r, 80)); }
  const l = S.link;
  cleanup();
  if (l) await l.close();
}

function cleanup() {
  clearInterval(S.pingTimer); clearTimeout(S.helloTimer);
  S.connected = false; S.link = null; S.hold = false; S.mode = 0;
  $('connectBtn').textContent = '보드 연결'; $('connectBtn').disabled = false; $('simBtn').disabled = false;
  dot('dotLink'); $('txtLink').textContent = '연결 안 됨'; dot('dotImu');
  updateUi();
}

// ------------------------------------------------------------------ 수신
function kv(line) {
  const o = {};
  for (const m of line.matchAll(/(\w+)=(\S+)/g)) o[m[1]] = m[2];
  return o;
}

function onLine(line) {
  if (line.startsWith('D,')) { onData(line); return; }
  if (line.startsWith('S,')) { onStatus(line); return; }
  log(line, line.startsWith('ERR') || line.includes('ABORT') || line.includes('FALLBACK') ? 'e' : '');
  if (line.startsWith('READY')) {
    S.fw = line.slice(6);
    if (!line.includes('AEROPEND_DEMO')) {
      banner('시연용 펌웨어가 아닙니다: ' + S.fw + ' — scripts/Build-Demo.ps1 -Upload 로 올리세요. (실험 UI는 index.html, 학습기는 rl/)');
      dot('dotLink', 'crit'); $('txtLink').textContent = S.fw;
    } else {
      dot('dotLink', 'good'); $('txtLink').textContent = S.isSim ? '시뮬레이터' : '보드 연결됨';
    }
  } else if (line.startsWith('IMU READY')) {
    dot('dotImu', 'good'); $('txtImu').textContent = 'IMU';
  } else if (line.startsWith('IMU ERROR')) {
    dot('dotImu', 'crit'); $('txtImu').textContent = 'IMU 오류';
  } else if (line.startsWith('CFG ')) {
    const c = kv(line);
    const warn = [];
    if (c.eeprom === '0') warn.push('EEPROM 보정이 없습니다(실험용 펌웨어로 CAL·ZERO 먼저)');
    if (c.beta && c.beta !== '10') warn.push(`β ${c.beta / 1000} — RL은 β 0.01로 학습했습니다`);
    if (c.bench === '1') warn.push('벤치 모드(모터 핀 LOW)');
    if (warn.length) banner('주의: ' + warn.join(' · '));
  } else if (line.startsWith('PIDCFG')) {
    S.pidcfg = line.slice(7);
    $('pidcfg').textContent = S.pidcfg;
    if (!S.pidSynced) { fillPid(kv(line)); S.pidSynced = true; }
  } else if (line.startsWith('MODE ')) {
    S.wantRl = line === 'MODE RL';
  } else if (line.startsWith('EVT FALLBACK')) {
    S.wantRl = false;
    banner('RL 안전 범위(−10…70°, 600 °/s)를 벗어나 PID로 돌아왔습니다.');
  } else if (line.startsWith('EVT ABORT')) {
    banner('정지: ' + line.slice(10) + (line.includes('LINK_TIMEOUT') ? ' (1 s 동안 PC 신호 없음)' : ''));
    S.hold = false;
  } else if (line === 'EVT STOP') {
    S.hold = false;
  } else if (line.startsWith('EVT REF')) {
    S.hold = true;
  }
  updateUi();
}

function onData(line) {
  const f = line.slice(2).split(',').map(Number);
  if (f.length < 8) return;
  const [t, u, th, , tha, mode, ref, a] = f;
  const d = S.d;
  d.t.push(t / 1000); d.u.push(u); d.th.push(th / 100); d.tha.push(tha / 100); d.mode.push(mode); d.ref.push(ref / 100); d.a.push(a / 1000);
  if (d.t.length > KEEP_S * 50) for (const k in d) d[k].splice(0, d[k].length - KEEP_S * 50);
  const prevMode = S.mode;
  S.mode = mode; S.hold = mode !== 0; S.lastD = f;
  if (mode !== prevMode) updateUi();
}

function onStatus(line) {
  const f = line.slice(2).split(',').map(Number);
  dot('dotImu', f[1] ? 'good' : 'crit');
  const parts = [`틱 부하 ${f[11] ?? '—'}%`, `maxTick ${(f[4] / 1000).toFixed(1)} ms`];
  if (f[9]) parts.push(`추론 최대 ${(f[9] / 1000).toFixed(1)} ms`);
  parts.push(`RL 누락 ${f[10]}`);
  if (f[6]) parts.push(`I2C 오류 ${f[6]}`);
  $('status').textContent = parts.join(' · ');
}

// ------------------------------------------------------------------ 표시
const MODE_TXT = ['대기 (모터 꺼짐)', '<b>PID</b>로 기준 유지 중', '<b>RL 대기</b> — PID로 기준까지 가는 중 (|오차| &lt; 2°를 1초 유지하면 RL로)', '<b>RL</b>(강화학습 정책)이 제어 중'];

function updateUi() {
  const c = S.connected && S.fw.includes('AEROPEND_DEMO');
  $('startBtn').disabled = !c;
  $('startBtn').textContent = S.hold ? '기준 적용' : '시작';
  $('applyPid').disabled = !c;
  $('csvBtn').disabled = !S.d.t.length;
  $('modePid').setAttribute('aria-pressed', String(!S.wantRl));
  $('modeRl').setAttribute('aria-pressed', String(S.wantRl));
  $('state').innerHTML = c ? MODE_TXT[S.mode] || '—' : '연결되지 않음';
}

function fillPid(c) {
  if (c.kp) $('kp').value = (+c.kp).toFixed(2);
  if (c.ki) $('ki').value = (+c.ki).toFixed(2);
  if (c.kd) $('kd').value = (+c.kd).toFixed(3);
  if (c.ff) $('ff').value = c.ff;
  if (c.umin) $('umin').value = c.umin;
  if (c.notch) { $('notchOn').checked = +c.notch > 0; if (+c.notch > 0) $('notchHz').value = (+c.notch / 100).toFixed(2); }
}

function fmt(v, d = 1) { return Number.isFinite(v) ? v.toFixed(d) : '—'; }

function draw() {
  requestAnimationFrame(draw);
  const d = S.d, n = d.t.length;
  if (S.lastD) {
    const [, u, th, , , mode, ref] = S.lastD;
    $('rTheta').textContent = fmt(th / 100) + '°';
    $('rRef').textContent = mode ? fmt(ref / 100) + '°' : '—';
    $('rErr').textContent = mode ? fmt((ref - th) / 100, 2) + '°' : '—';
    $('rPwm').textContent = u;
  }
  if (!n) return;
  const win = +$('win').value;
  const t1 = d.t[n - 1], t0 = t1 - win;
  let i0 = n - 1;
  while (i0 > 0 && d.t[i0 - 1] >= t0) i0--;
  const x = d.t.slice(i0).map((v) => v - t1);
  const th = d.th.slice(i0), mode = d.mode.slice(i0);
  const thPid = th.map((v, i) => (mode[i] === 3 ? NaN : v));
  const thRl = th.map((v, i) => (mode[i] === 3 || mode[i + 1] === 3 ? v : NaN));
  const ref = d.ref.slice(i0).map((v, i) => (mode[i] ? v : NaN));
  // 최근 5 s σ(오차) (유지 중 샘플만)
  const e = [];
  for (let i = n - 1; i >= 0 && d.t[i] >= t1 - 5; i--) if (d.mode[i]) e.push(d.ref[i] - d.th[i]);
  if (e.length > 20) {
    const m = e.reduce((s, v) => s + v, 0) / e.length;
    $('rStd').textContent = Math.sqrt(e.reduce((s, v) => s + (v - m) ** 2, 0) / e.length).toFixed(2) + '°';
  } else $('rStd').textContent = '—';
  drawChart($('chart'), {
    x, xmin: -win, xmax: 0, xlabel: '시간 [s]', yLabelL: '각도 [°]', yLabelR: 'PWM',
    yR: [0, 260],
    series: [
      { y: ref, color: '--s5', dash: [5, 4], step: true, label: '기준' },
      { y: thPid, color: '--s1', width: 1.6, label: 'θ (PID)' },
      { y: thRl, color: '--s3', width: 2.6, label: 'θ (RL)' },
      { y: d.u.slice(i0), color: '--s2', axis: 'R', width: 1, label: 'PWM' },
    ],
  });
}

function saveCsv() {
  const d = S.d;
  const rows = ['# aeropendulum demo; fw=' + S.fw + '; pid; ' + S.pidcfg, 't_s,pwm,theta_deg,thetaAcc_deg,mode,ref_deg,rl_action'];
  for (let i = 0; i < d.t.length; i++) rows.push([d.t[i].toFixed(3), d.u[i], d.th[i].toFixed(2), d.tha[i].toFixed(2), d.mode[i], d.ref[i].toFixed(2), d.a[i].toFixed(3)].join(','));
  const blob = new Blob([rows.join('\n') + '\n'], { type: 'text/csv' });
  const a = document.createElement('a');
  const ts = new Date();
  const p = (v) => String(v).padStart(2, '0');
  a.download = `aero_DEMO_${ts.getFullYear()}${p(ts.getMonth() + 1)}${p(ts.getDate())}_${p(ts.getHours())}${p(ts.getMinutes())}${p(ts.getSeconds())}.csv`;
  a.href = URL.createObjectURL(blob);
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ------------------------------------------------------------------ 동작
function refCdeg() { return Math.round(+$('refRange').value * 100); }

function applyPid() {
  const kp = +$('kp').value, ki = +$('ki').value, kd = +$('kd').value, ff = Math.round(+$('ff').value), umin = Math.round(+$('umin').value);
  if (![kp, ki, kd].every((v) => Number.isFinite(v) && v >= 0) || !(ff >= 0 && ff <= 255) || !(umin >= 0 && umin <= 255)) { banner('이득 값을 확인하세요 (0 이상, ff·최소 PWM 0–255)'); return; }
  send(`PID ${Math.round(kp * 1000)} ${Math.round(ki * 1000)} ${Math.round(kd * 1000)} ${ff} ${umin}`);
  const hz = +$('notchHz').value;
  if ($('notchOn').checked && hz >= 0.1 && hz <= 20) send(`NOTCH ${Math.round(hz * 100)} 150 700`);
  else send('NOTCH 0');
}

$('connectBtn').onclick = () => connect(false);
$('simBtn').onclick = () => connect(true);
$('stopBtn').onclick = () => { send('STOP'); };
$('startBtn').onclick = () => send(`REF ${refCdeg()}`);
$('modePid').onclick = () => { S.wantRl = false; updateUi(); send('MODE PID'); };
$('modeRl').onclick = () => { S.wantRl = true; updateUi(); send('MODE RL'); };
$('applyPid').onclick = applyPid;
$('csvBtn').onclick = saveCsv;
$('refRange').oninput = () => { $('refTxt').textContent = (+$('refRange').value).toFixed(1); };
$('refRange').onchange = () => { if (S.hold) send(`REF ${refCdeg()}`); };
for (const b of document.querySelectorAll('.refPreset')) {
  b.onclick = () => { $('refRange').value = b.dataset.v; $('refRange').oninput(); if (S.hold) send(`REF ${refCdeg()}`); };
}
for (const b of document.querySelectorAll('.pidPreset')) {
  b.onclick = () => {
    const [kp, ki, kd, ff, umin, hz] = b.dataset.p.split(',').map(Number);
    fillPid({ kp, ki, kd, ff, umin, notch: Math.round(hz * 100) });
    $('notchOn').checked = hz > 0;
    if (hz > 0) $('notchHz').value = hz.toFixed(2);
    banner('');
  };
}
document.addEventListener('keydown', (e) => {
  if ((e.key === 'Escape' || e.key === ' ') && !['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) {
    e.preventDefault(); send('STOP');
  }
});
window.addEventListener('beforeunload', () => { if (S.connected) S.link.send('STOP'); });
updateUi();
requestAnimationFrame(draw);
