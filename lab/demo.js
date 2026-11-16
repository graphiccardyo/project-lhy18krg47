// 시연 UI — 시연용 펌웨어(AEROPEND_DEMO 1.0, docs/DEMO.md)와 Web Serial 로 대화합니다.
// 보드 혼자 PID 또는 고정 RL 정책으로 기준 각도를 유지하고, 이 페이지는 제어기·기준·PID 이득을 보내고 그래프를 그립니다.
// 연결된 동안 250 ms 마다 PING (보드는 1 s 동안 명령이 없으면 정지).
'use strict';

const BAUD = 250000;
const enc = new TextEncoder();
const $ = (id) => document.getElementById(id);
const S = { link: null, connected: false, isSim: false, mode: 0, wantRl: false, pidSynced: false, hold: false,
  d: { t: [], th: [], ref: [], u: [], mode: [], a: [], tha: [] }, lastD: null, pidcfg: '', fw: '', cfg: {},
  test: null, results: [] };
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
  if (S.test) testEnd('aborted', '연결 끊김');
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
    S.cfg = c;
    const warn = [];
    if (c.eeprom === '0') warn.push('EEPROM 보정이 없습니다(실험용 펌웨어로 CAL·ZERO 먼저)');
    if (c.beta && c.beta !== '10') warn.push(`β ${c.beta / 1000} — RL은 β 0.01로 학습했습니다`);
    if (c.bench === '1') warn.push('벤치 모드(모터 핀 LOW)');
    if (warn.length) banner('주의: ' + warn.join(' · '));
  } else if (line.startsWith('PIDCFG')) {
    S.pidcfg = line.slice(7);
    $('pidcfg').textContent = S.pidcfg;
    if (!S.pidSynced) { fillPid(kv(line)); S.pidSynced = true; }
  } else if (line.startsWith('RLPOL ')) {
    S.rlpol = line.slice(6);
    const c = kv(line), src = line.slice(6).split(' res=')[0];
    $('rlpol').textContent = `RL 정책: ${src} · 잔차 ±${c.res} PWM · 출력 ${c.ulo}–${c.uhi}
` +
      `기반 PID (고정): Kp ${+c.kp} · Ki ${+c.ki} · Kd ${+c.kd} · ff ${c.ff} · 노치 ${+c.notch ? (c.notch / 100).toFixed(2) + ' Hz' : '끔'}`;
    $('rlpol').style.whiteSpace = 'pre-wrap';
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
  const [t, u, th, om, tha, mode, ref, a] = f;
  // 9번째 필드 = RL 잔차 r [PWM] (RL 일 때만, 펌웨어 2.0)
  if (S.test) testOnData(t, u, th / 100, om / 100, tha / 100, mode, ref / 100, a);
  // 모터 가동 시간 (새 모터 보호: 120 s 가동 → 90 s 휴식, rl/configs/motor2.json)
  if (u > 0) {
    if (S.onSince == null) S.onSince = t;
    const on = (t - S.onSince) / 1000;
    if (on >= MOTOR_ON_WARN_S && !S.onWarned) { S.onWarned = true; banner(`모터가 ${MOTOR_ON_WARN_S} s 넘게 돌고 있습니다 — 새 모터 보호를 위해 정지하고 ${MOTOR_REST_S} s 쉬세요.`); }
  } else { S.onSince = null; S.onWarned = false; }
  const d = S.d;
  d.t.push(t / 1000); d.u.push(u); d.th.push(th / 100); d.tha.push(tha / 100); d.mode.push(mode); d.ref.push(ref / 100); d.a.push(a);
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
const MOTOR_ON_WARN_S = 120, MOTOR_REST_S = 90;
const MODE_TXT = ['대기 (모터 꺼짐)', '<b>PID</b>로 기준 유지 중', '<b>RL 대기</b> — PID로 기준까지 가는 중 (|오차| &lt; 2°를 1초 유지하면 RL로)', '<b>RL</b>(강화학습 정책)이 제어 중'];

function updateUi() {
  const c = S.connected && S.fw.includes('AEROPEND_DEMO');
  $('startBtn').disabled = !c;
  $('startBtn').textContent = S.hold ? '기준 적용' : '시작';
  $('applyPid').disabled = !c;
  $('csvBtn').disabled = !S.d.t.length;
  const busy = !!S.test;                    // 시험 중에는 제어기·기준·이득을 못 바꿈 (기록이 섞이지 않게)
  for (const id of ['modePid', 'modeRl', 'refRange']) $(id).disabled = busy;
  for (const b of document.querySelectorAll('.refPreset')) b.disabled = busy;
  $('applyPid').disabled = !c || busy;
  if (busy) $('startBtn').disabled = true;
  $('testBtn').disabled = !c || busy;
  $('testAbortBtn').disabled = !busy;
  $('tCtrl').textContent = S.wantRl ? 'RL' : 'PID';
  $('resCsvBtn').disabled = !S.results.length;
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
  if (c.notch) { $('notchOn').checked = +c.notch > 0; if (+c.notch > 0) $('nf').value = (+c.notch / 100).toFixed(2); }
  if (c.zz) $('nzz').value = +c.zz / 1000;
  if (c.zp) $('nzp').value = +c.zp / 1000;
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
    const r = S.lastD[7];
    $('rRes').textContent = mode === 3 ? (r > 0 ? '+' : '') + r : '—';
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
  const rows = ['# aeropendulum demo; fw=' + S.fw + '; pid; ' + S.pidcfg + '; rlpol; ' + (S.rlpol || ''), 't_s,pwm,theta_deg,thetaAcc_deg,mode,ref_deg,rl_res_pwm'];
  for (let i = 0; i < d.t.length; i++) rows.push([d.t[i].toFixed(3), d.u[i], d.th[i].toFixed(2), d.tha[i].toFixed(2), d.mode[i], d.ref[i].toFixed(2), d.a[i]].join(','));
  const ts = new Date();
  const p = (v) => String(v).padStart(2, '0');
  download(`aero_DEMO_${ts.getFullYear()}${p(ts.getMonth() + 1)}${p(ts.getDate())}_${p(ts.getHours())}${p(ts.getMinutes())}${p(ts.getSeconds())}.csv`,
    rows.join('\n') + '\n', 'text/csv');
}

function download(name, text, type) {
  const a = document.createElement('a');
  a.download = name;
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ------------------------------------------------------------------ 시험: 일정 시간 유지 / 스텝 입력
// 보드는 그대로이고 이 페이지가 REF 를 보내며 D 줄(보드 시각 t_ms)을 기록합니다.
//  준비: 시작 기준으로 REF → |오차| < 2° 를 1 s 유지 (RL 은 mode 3 이 된 뒤) → 기록 시작 (t = 0)
//  유지: T 초 기록.  스텝: pre 초 r0 → REF r1 → hold 초 → (post > 0 이면) REF r0 → post 초
//  보드에서 기준이 바뀐 줄(D 줄 ref)로 구간(phase 1/2/3)을 나누므로 PC 지연은 결과에 들어가지 않음.
const PREP_TIMEOUT_S = 40;

function numIn(id, lo, hi) {
  const v = +$(id).value;
  return Number.isFinite(v) && v >= lo && v <= hi ? v : NaN;
}

function testStart() {
  if (S.test || !S.connected) return;
  const type = $('tStep').getAttribute('aria-pressed') === 'true' ? 'STEP' : 'HOLD';
  let p;
  if (type === 'HOLD') p = { r0: numIn('hRef', 5, 40), dur_s: numIn('hDur', 5, 600) };
  else p = { r0: numIn('sR0', 5, 40), r1: numIn('sR1', 5, 40), pre_s: numIn('sPre', 1, 60), hold_s: numIn('sHold', 3, 120), post_s: numIn('sPost', 0, 60) };
  if (Object.values(p).some((v) => !Number.isFinite(v))) { banner('시험 값을 확인하세요 (기준 5–40°, 시간 범위)'); return; }
  if (type === 'STEP' && Math.abs(p.r1 - p.r0) < 1) { banner('스텝 크기는 1° 이상으로 하세요'); return; }
  const total = type === 'HOLD' ? p.dur_s : p.pre_s + p.hold_s + p.post_s;
  // '# pid' 줄 = 실제로 쓴 이득: PID 시험은 사용자 이득, RL 시험은 RL 기반 PID(RLPOL)
  S.test = { type, p, total, ctrl: S.wantRl ? 'RL' : 'PID', pid: S.wantRl ? (S.rlpol || '').replace(/^.* uhi=\d+ /, '') : S.pidcfg, rlpol: S.rlpol || '', sim: S.isSim, cfg: { ...S.cfg }, start: new Date(), phase: 'prep',
    prepT0: null, settleT0: null, seenHold: false, t0: null, sent1: false, sent0: false, rows: [] };
  send(`REF ${Math.round(p.r0 * 100)}`);
  $('testMsg').textContent = `${type === 'HOLD' ? '유지' : '스텝'} 시험 준비 — ${p.r0}°로 가서 정착 대기…`;
  $('testBar').style.width = '0';
  log(`시험 시작: ${type} ${JSON.stringify(p)} (${S.test.ctrl})`, 'ok');
  updateUi();
}

function testOnData(t, u, th, om, tha, mode, ref, a) {
  const T = S.test, p = T.p;
  if (T.prepT0 === null) T.prepT0 = t;
  if (mode !== 0) T.seenHold = true;
  if (T.seenHold && mode === 0) return testEnd('aborted', '정지');
  if (T.phase === 'prep') {
    if (T.ctrl === 'RL' && T.seenHold && !S.wantRl) return testEnd('aborted', 'RL 복귀(FALLBACK)');
    const ok = mode !== 0 && Math.abs(ref - p.r0) < 0.005 && Math.abs(ref - th) < 2 && (T.ctrl === 'PID' ? mode === 1 : mode === 3);
    if (!ok) T.settleT0 = null;
    else if (T.settleT0 === null) T.settleT0 = t;
    if (T.settleT0 !== null && t - T.settleT0 >= 1000) {
      T.phase = 'run'; T.t0 = t;
      $('testMsg').textContent = '기록 중…';
    } else {
      if (t - T.prepT0 > PREP_TIMEOUT_S * 1000) return testEnd('aborted', `${PREP_TIMEOUT_S} s 안에 정착하지 못함`);
      return;
    }
  }
  if (T.ctrl === 'RL' && mode !== 3) return testEnd('aborted', 'RL 복귀(FALLBACK)');
  const ts = (t - T.t0) / 1000;
  let ph = 1;
  if (T.type === 'STEP') {
    if (!T.sent1 && ts >= p.pre_s) { send(`REF ${Math.round(p.r1 * 100)}`); T.sent1 = true; }
    if (p.post_s > 0 && !T.sent0 && ts >= p.pre_s + p.hold_s) { send(`REF ${Math.round(p.r0 * 100)}`); T.sent0 = true; }
    const last = T.rows.length ? T.rows[T.rows.length - 1].ph : 1;
    ph = last === 1 ? (Math.abs(ref - p.r1) < 0.005 ? 2 : 1) : last === 2 ? (T.sent0 && Math.abs(ref - p.r0) < 0.005 ? 3 : 2) : 3;
  }
  T.rows.push({ t: ts, u, th, om, tha, mode, ref, a, ph });
  $('testBar').style.width = Math.min(100, 100 * ts / T.total).toFixed(1) + '%';
  if (ts >= T.total) testEnd('done');
}

function mean(a) { return a.length ? a.reduce((s, v) => s + v, 0) / a.length : NaN; }
function std(a) { const m = mean(a); return a.length ? Math.sqrt(mean(a.map((v) => (v - m) ** 2))) : NaN; }
function smooth(y, k = 2) {                 // 앞뒤 k 샘플 이동평균 (20 ms × 5 = 0.1 s, 리플·잡음 제거용)
  return y.map((_, i) => mean(y.slice(Math.max(0, i - k), i + k + 1)));
}

function testMetrics(T) {
  const R = T.rows, p = T.p, m = {};
  if (T.type === 'HOLD') {
    const e = R.map((r) => r.ref - r.th), u = R.map((r) => r.u);
    Object.assign(m, { bias: mean(e), std: std(e), rms: Math.sqrt(mean(e.map((v) => v * v))), maxabs: Math.max(...e.map(Math.abs)),
      pwm_mean: mean(u), pwm_std: std(u), dpwm: mean(u.slice(1).map((v, i) => Math.abs(v - u[i]))) });
    return m;
  }
  const seg = R.filter((r) => r.ph === 2);
  if (seg.length < 10) return m;
  const t1 = seg[0].t, tt = seg.map((r) => r.t - t1), y = seg.map((r) => r.th), ys = smooth(y), d = p.r1 - p.r0;
  const s = ys.map((v) => (v - p.r0) / d);
  const i10 = s.findIndex((v) => v >= 0.1), i90 = s.findIndex((v) => v >= 0.9);
  m.tr = i10 >= 0 && i90 >= 0 ? tt[i90] - tt[i10] : NaN;
  m.os = Math.max(0, (Math.max(...s) - 1) * 100);
  let iOut = -1;
  for (let i = 0; i < ys.length; i++) if (Math.abs(ys[i] - p.r1) > 0.05 * Math.abs(d)) iOut = i;
  m.ts5 = iOut < 0 ? 0 : iOut < ys.length - 1 ? tt[iOut + 1] : NaN;
  const tEnd = tt[tt.length - 1];
  const last2 = y.filter((_, i) => tt[i] > tEnd - 2), last3 = seg.filter((_, i) => tt[i] > tEnd - 3);
  m.bias = p.r1 - mean(last2);
  m.std = std(last3.map((r) => r.th));
  m.rms = Math.sqrt(mean(seg.map((r) => (r.ref - r.th) ** 2)));
  m.pwm_std = std(last3.map((r) => r.u));
  m.pwm_mean = mean(last3.map((r) => r.u));
  return m;
}

function f2(v, d = 2) { return Number.isFinite(v) ? v.toFixed(d) : ''; }
function paramTxt(T) { return Object.entries(T.p).map(([k, v]) => `${k}=${v}`).join(' '); }

function testCsv(T, status) {
  const c = T.cfg, q = kv(T.pid || '');
  const m = T.metrics || {};
  const rows = [
    `# aeropendulum_demo; name=${T.type}_${T.ctrl}; type=${T.type}; ctrl=${T.ctrl}; params=${paramTxt(T)}; Ts=0.02; input=pwm(0-255); ` +
      `axis=${c.axis ?? '?'}${c.sign ?? ''}; beta=${c.beta ? c.beta / 1000 : '?'}; dlpf=${c.dlpf ?? '?'}; cal=${c.cal ?? '?'}; zero=${c.zero ?? '?'}; ` +
      `sim=${T.sim ? 1 : 0}; bench=${c.bench ?? '?'}; status=${status}; fw=${S.fw}; start=${T.start.toISOString()}`,
    `# pid; kp=${+q.kp}; ki=${+q.ki}; kd=${+q.kd}; ff=${q.ff}; umin=${q.umin}; notch_cHz=${q.notch}; zz=${q.zz}; zp=${q.zp}` +
      (T.ctrl === 'RL' ? '; source=RL 기반 PID (policy.h)' : ''),
    `# rl; ${T.ctrl === 'RL' ? 'policy=' + T.rlpol : 'none'}`,
    '# result; ' + Object.entries(m).map(([k, v]) => `${k}=${f2(v, 3)}`).join('; '),
    't_s,pwm,duty_pct,theta_deg,omega_dps,theta_acc_deg,phase,ref_deg,mode,rl_res_pwm',
  ];
  for (const r of T.rows) rows.push([r.t.toFixed(3), r.u, (r.u / 2.55).toFixed(2), r.th.toFixed(2), r.om.toFixed(2), r.tha.toFixed(2), r.ph, r.ref.toFixed(2), r.mode, r.a].join(','));
  return rows.join('\n') + '\n';
}

function testEnd(status, why) {
  const T = S.test;
  if (!T) return;
  S.test = null;
  const ran = T.phase === 'run' && T.rows.length > 5;
  T.metrics = status === 'done' ? testMetrics(T) : {};
  const z = (n) => String(n).padStart(2, '0'), d = T.start;
  const name = `aero_DEMO_${T.type}_${T.ctrl}_${d.getFullYear()}${z(d.getMonth() + 1)}${z(d.getDate())}_${z(d.getHours())}${z(d.getMinutes())}${z(d.getSeconds())}${status === 'done' ? '' : '_partial'}.csv`;
  const res = { n: S.results.length + 1, T, status, why: why || '', name, csv: ran ? testCsv(T, status) : null };
  S.results.push(res);
  addResultRow(res);
  if (res.csv && $('autoCsv').checked) { download(name, res.csv, 'text/csv'); log('CSV 저장: ' + name, 'ok'); }
  $('testMsg').textContent = status === 'done' ? `끝 — ${T.type === 'HOLD' || T.p.post_s > 0 ? T.p.r0 : T.p.r1}° 유지 중 (정지로 끄세요)` : `중단: ${why}`;
  if (status !== 'done') banner('시험 중단: ' + why);
  log(`시험 ${status === 'done' ? '끝' : '중단(' + why + ')'}: ${T.type} ${T.ctrl} ${JSON.stringify(T.metrics, (k, v) => (typeof v === 'number' ? +v.toFixed(3) : v))}`, status === 'done' ? 'ok' : 'e');
  updateUi();
}

function addResultRow(res) {
  const T = res.T, m = T.metrics, tr = document.createElement('tr');
  const refTxt = T.type === 'HOLD' ? `${T.p.r0}° · ${T.p.dur_s} s` : `${T.p.r0}→${T.p.r1}°`;
  const cells = [res.n, T.start.toLocaleTimeString('ko-KR', { hour12: false }), T.type === 'HOLD' ? '유지' : '스텝', T.ctrl, refTxt,
    f2(m.tr) && f2(m.tr) + ' s', f2(m.os, 1) && f2(m.os, 1) + '%',
    f2(m.ts5) ? f2(m.ts5) + ' s' : (T.type === 'STEP' && res.status === 'done' ? `> ${T.p.hold_s} s` : ''), f2(m.bias) && f2(m.bias) + '°',
    f2(m.std) && f2(m.std) + '°', f2(m.rms) && f2(m.rms) + '°', f2(m.pwm_std, 1), res.status === 'done' ? '완료' : '중단'];
  for (const c of cells) { const td = document.createElement('td'); td.textContent = c === '' ? '—' : c; tr.appendChild(td); }
  if (res.status !== 'done') { tr.lastChild.className = 'ab'; tr.lastChild.title = res.why; }
  const td = document.createElement('td');
  if (res.csv) {
    const b = document.createElement('button');
    b.className = 'small'; b.textContent = 'CSV'; b.title = res.name;
    b.onclick = () => download(res.name, res.csv, 'text/csv');
    td.appendChild(b);
  }
  tr.appendChild(td);
  $('resTable').tBodies[0].appendChild(tr);
}

function resultsCsv() {
  const keys = ['tr', 'os', 'ts5', 'bias', 'std', 'rms', 'maxabs', 'pwm_mean', 'pwm_std', 'dpwm'];
  const rows = ['n,start,type,ctrl,params,pid,' + ['tr_s', 'os_pct', 'ts5_s', 'bias_deg', 'std_deg', 'rms_deg', 'maxabs_deg', 'pwm_mean', 'pwm_std', 'dpwm_per_sample'].join(',') + ',status,note,file,sim'];
  for (const r of S.results) {
    const T = r.T;
    rows.push([r.n, T.start.toISOString(), T.type, T.ctrl, `"${paramTxt(T)}"`, `"${T.pid}"`, ...keys.map((k) => f2(T.metrics[k], 3)), r.status, `"${r.why}"`, r.csv ? r.name : '', T.sim ? 1 : 0].join(','));
  }
  const d = new Date(), z = (n) => String(n).padStart(2, '0');
  download(`aero_DEMO_results_${d.getFullYear()}${z(d.getMonth() + 1)}${z(d.getDate())}_${z(d.getHours())}${z(d.getMinutes())}.csv`, rows.join('\n') + '\n', 'text/csv');
}

// ------------------------------------------------------------------ 동작
function refCdeg() { return Math.round(+$('refRange').value * 100); }

function applyPid() {
  const kp = +$('kp').value, ki = +$('ki').value, kd = +$('kd').value, ff = Math.round(+$('ff').value), umin = Math.round(+$('umin').value);
  if (![kp, ki, kd].every((v) => Number.isFinite(v) && v >= 0) || !(ff >= 0 && ff <= 255) || !(umin >= 0 && umin <= 255)) { banner('이득 값을 확인하세요 (0 이상, ff·최소 PWM 0–255)'); return; }
  const hz = +$('nf').value, zz = +$('nzz').value, zp = +$('nzp').value;
  if ($('notchOn').checked && !(hz >= 0.1 && hz <= 20 && zz >= 0 && zz <= 2 && zp > 0 && zp <= 2)) { banner('노치 값을 확인하세요 (f 0.1–20 Hz, ζz 0–2, ζp 0–2)'); return; }
  send(`PID ${Math.round(kp * 1000)} ${Math.round(ki * 1000)} ${Math.round(kd * 1000)} ${ff} ${umin}`);
  if ($('notchOn').checked) send(`NOTCH ${Math.round(hz * 100)} ${Math.round(zz * 1000)} ${Math.round(zp * 1000)}`);
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
for (const [id, other, show, hide] of [['tHold', 'tStep', 'tHoldArgs', 'tStepArgs'], ['tStep', 'tHold', 'tStepArgs', 'tHoldArgs']]) {
  $(id).onclick = () => {
    if (S.test) return;
    $(id).setAttribute('aria-pressed', 'true'); $(other).setAttribute('aria-pressed', 'false');
    $(show).hidden = false; $(hide).hidden = true;
  };
}
$('testBtn').onclick = testStart;
$('testAbortBtn').onclick = () => testEnd('aborted', '사용자 중단 (기준 유지는 계속)');
$('resCsvBtn').onclick = resultsCsv;
$('refRange').oninput = () => { $('refTxt').textContent = (+$('refRange').value).toFixed(1); };
$('refRange').onchange = () => { if (S.hold) send(`REF ${refCdeg()}`); };
for (const b of document.querySelectorAll('.refPreset')) {
  b.onclick = () => { $('refRange').value = b.dataset.v; $('refRange').oninput(); if (S.hold) send(`REF ${refCdeg()}`); };
}

// ------------------------------------------------------------------ 저장된 설정 — 실험 UI(app.js)와 같은 저장소·파일 형식
// 실험 UI 는 모든 탭의 입력을 한 묶음(values)으로 저장합니다. 시연 UI 는 그중 이득 항목(PID_FIELDS)만 읽고 씁니다.
// 같은 이름으로 저장하면 values 의 다른 항목(시험 설정·안전 한계)은 그대로 두고 이득만 바꿉니다.
const PRESET_KEY = 'aeroLab.presets.v1', PRESET_FORMAT = 'aeropend-lab-settings';
const PID_FIELDS = ['kp', 'ki', 'kd', 'ff', 'umin', 'notchOn', 'nf', 'nzz', 'nzp'];
function readPresets() {
  try { const o = JSON.parse(localStorage.getItem(PRESET_KEY) || '{}'); return o && typeof o === 'object' ? o : {}; } catch (_) { return {}; }
}
function writePresets(o) {
  try { localStorage.setItem(PRESET_KEY, JSON.stringify(o)); return true; }
  catch (e) { banner('설정을 저장하지 못했습니다 (브라우저 저장소 사용 불가): ' + e.message); return false; }
}
function pidValues() {
  const o = {};
  for (const id of PID_FIELDS) { const el = $(id); o[id] = el.type === 'checkbox' ? el.checked : el.value; }
  return o;
}
function presetLabel(name, v) {
  const n = (x) => (x === undefined || x === '' ? '?' : +x);
  return `${name} — Kp ${n(v.kp)} · Ki ${n(v.ki)} · Kd ${n(v.kd)} · ff ${n(v.ff)}${+v.umin ? ' · 최소 ' + v.umin : ''}${v.notchOn ? ' · 노치' : ''}`;
}
function refreshPresetSel(select) {
  const all = readPresets(), sel = $('presetSel'), keep = select ?? sel.value;
  const names = Object.keys(all).sort((a, b) => a.localeCompare(b, 'ko'));
  sel.replaceChildren(new Option(names.length ? '— 선택 —' : '— 없음 —', ''));
  for (const k of names) sel.add(new Option(presetLabel(k, all[k].values || {}), k));
  sel.value = names.includes(keep) ? keep : '';
}
function savePreset() {
  const name = $('presetName').value.trim() || $('presetSel').value;
  if (!name) return banner('저장할 이름을 입력하세요.');
  const all = readPresets();
  if (all[name] && !confirm(`"${name}" 설정이 이미 있습니다. 이득을 현재 값으로 덮어쓸까요?`)) return;
  all[name] = { saved: new Date().toISOString(), values: { ...(all[name] && all[name].values), ...pidValues() } };
  if (!writePresets(all)) return;
  refreshPresetSel(name); $('presetName').value = '';
  $('presetMsg').textContent = `"${name}" 저장됨`;
  log(`설정 저장: ${name}`, 'ok');
}
function loadPresetByName(name) {
  const p = readPresets()[name];
  if (!p) return banner('불러올 설정을 고르세요.');
  const v = p.values || {};
  let n = 0;
  for (const id of PID_FIELDS) {
    if (!(id in v)) continue;
    const el = $(id);
    if (el.type === 'checkbox') el.checked = !!v[id]; else el.value = v[id];
    n++;
  }
  if (!n) return banner(`"${name}"에는 이득 값이 없습니다.`);
  const extra = S.connected ? ' 이득은 [이득 적용]을 눌러야 보드에 들어갑니다.' : '';
  $('presetMsg').textContent = `"${name}" 불러옴 (${p.saved ? new Date(p.saved).toLocaleString('ko-KR', { hour12: false }) + ' 저장' : ''}).${extra}`;
  log(`설정 불러옴: ${name}`, 'ok');
}
function deletePreset() {
  const name = $('presetSel').value;
  if (!name) return banner('삭제할 설정을 고르세요.');
  if (!confirm(`"${name}" 설정을 삭제할까요? (실험 UI에서도 사라집니다)`)) return;
  const all = readPresets(); delete all[name];
  if (writePresets(all)) { refreshPresetSel(''); $('presetMsg').textContent = `"${name}" 삭제됨`; }
}
function exportPresets() {
  const all = readPresets();
  if (!Object.keys(all).length) return banner('내보낼 저장 설정이 없습니다. 먼저 [현재 값 저장]을 누르세요.');
  const d = new Date(), p2 = (x) => String(x).padStart(2, '0');
  download(`aeropend_settings_${d.getFullYear()}${p2(d.getMonth() + 1)}${p2(d.getDate())}.json`,
    JSON.stringify({ format: PRESET_FORMAT, version: 1, presets: all }, null, 2), 'application/json');
}
async function importPresets(file) {
  let o;
  try { o = JSON.parse(await file.text()); } catch (e) { return banner(`${file.name}: JSON 형식이 아닙니다.`); }
  if (!o || o.format !== PRESET_FORMAT || typeof o.presets !== 'object') return banner(`${file.name}: 실험실 설정 파일이 아닙니다.`);
  const all = readPresets(), names = Object.keys(o.presets).filter((k) => o.presets[k] && typeof o.presets[k].values === 'object');
  const dup = names.filter((k) => all[k]);
  if (dup.length && !confirm(`같은 이름이 이미 있습니다: ${dup.join(', ')}\n파일의 값으로 덮어쓸까요? (취소하면 겹치는 것은 건너뜀)`))
    names.splice(0, names.length, ...names.filter((k) => !all[k]));
  for (const k of names) all[k] = { saved: o.presets[k].saved || null, values: o.presets[k].values };
  if (!writePresets(all)) return;
  refreshPresetSel(names[0]);
  $('presetMsg').textContent = `${file.name}: ${names.length}개 가져옴`;
  log(`설정 가져오기: ${file.name} (${names.join(', ') || '없음'})`, 'ok');
}
refreshPresetSel('');
$('presetSaveBtn').onclick = savePreset;
$('presetLoadBtn').onclick = () => loadPresetByName($('presetSel').value);
$('presetSel').ondblclick = () => loadPresetByName($('presetSel').value);
$('presetDelBtn').onclick = deletePreset;
$('presetExportBtn').onclick = exportPresets;
$('presetImportBtn').onclick = () => $('presetFile').click();
$('presetFile').onchange = async (e) => { for (const f of e.target.files) await importPresets(f); e.target.value = ''; };
$('presetName').addEventListener('keydown', (e) => { if (e.key === 'Enter') savePreset(); });
window.addEventListener('storage', (e) => { if (e.key === PRESET_KEY) refreshPresetSel(); });   // 실험 UI 를 같이 열어 둔 경우
document.addEventListener('keydown', (e) => {
  if ((e.key === 'Escape' || e.key === ' ') && !['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) {
    e.preventDefault(); send('STOP');
  }
});
window.addEventListener('beforeunload', () => { if (S.connected) S.link.send('STOP'); });
updateUi();
requestAnimationFrame(draw);
