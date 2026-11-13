'use strict';
// Aeropendulum 실험실 UI — Web Serial(Edge/Chrome) 또는 내장 시뮬레이터
const $ = (id) => document.getElementById(id);
const BAUD = 250000;
const enc = new TextEncoder();
const duty = (pwm) => pwm / 2.55;          // PWM 카운트 → 듀티 % (참고 표시용)

// ------------------------------------------------------------------ 상태
const S = {
  link: null, connected: false, isSim: false, fw: '', helloSent: false,
  cfg: {}, calv: null, imu: false, devState: 0, lastStatus: null,
  live: { t: [], u: [], th: [], om: [], tha: [], ph: [], ref: [] },
  last: null, lastTms: null, drops: 0, rxCount: 0, rxHz: 0, lastRateAt: performance.now(),
  aux: [], rec: null, recs: [], pendingRun: null, recSeq: 0,
  win: 30, paused: false, calCountdown: null, axisDetect: null,
  staticResult: null, dynType: 'STEP',
};
const LIVE_MAX = 14000;

// ------------------------------------------------------------------ 로그/배너
function log(msg, cls = '') {
  const el = $('log');
  const d = document.createElement('div');
  if (cls) d.className = cls;
  d.textContent = `${new Date().toLocaleTimeString('ko-KR', { hour12: false })}  ${msg}`;
  el.appendChild(d);
  while (el.childElementCount > 400) el.firstChild.remove();
  el.scrollTop = el.scrollHeight;
}
function banner(msg) {
  const b = $('banner');
  if (!msg) { b.classList.remove('show'); return; }
  b.textContent = msg; b.classList.add('show');
  clearTimeout(banner.t); banner.t = setTimeout(() => b.classList.remove('show'), 9000);
}
function dot(id, cls) { $(id).className = 'dot' + (cls ? ' ' + cls : ''); }

// ------------------------------------------------------------------ 전송 계층
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
  async open() { this.dev = new SimDevice(); this.dev.onLine = (l) => this.onLine(l); }
  send(line) { setTimeout(() => this.dev && this.dev.write(line), 2); return Promise.resolve(); }
  async close() { if (this.dev) this.dev.close(); this.dev = null; }
}

function send(line, quiet = false) {
  if (!S.connected) { log('연결되지 않음: ' + line, 'e'); return false; }
  if (!quiet) log('> ' + line, 'tx');
  S.link.send(line);
  return true;
}

async function connect(sim) {
  if (S.connected) { await disconnect(); return; }
  if (!sim && !('serial' in navigator)) {
    banner('이 브라우저는 Web Serial을 지원하지 않습니다. Edge 또는 Chrome으로 여세요. (시뮬레이터는 사용 가능)');
    return;
  }
  const link = sim ? new SimLink() : new SerialLink();
  link.onLine = onLine;
  link.onClose = () => { log('포트가 닫혔습니다', 'e'); cleanupDisconnected(); };
  try {
    await link.open();
  } catch (e) {
    if (e.name === 'NotFoundError') log('포트를 선택하지 않았습니다 (또는 이 브라우저에서 포트 선택창을 열 수 없음)', 'e');
    else banner('연결 실패: ' + e.message + ' — 다른 프로그램(시리얼 모니터 등)이 포트를 쓰고 있지 않은지 확인하세요.');
    return;
  }
  S.link = link; S.connected = true; S.isSim = sim; S.helloSent = false; S.fw = ''; S.synced = false;
  S.live = { t: [], u: [], th: [], om: [], tha: [], ph: [], ref: [] }; S.lastTms = null; S.drops = 0;
  $('connectBtn').textContent = '연결 해제';
  $('simBtn').disabled = true; $('connectBtn').disabled = false;
  if (sim) $('connectBtn').textContent = '시뮬레이터 종료';
  dot('dotLink', 'warn'); $('txtLink').textContent = sim ? '시뮬레이터' : '보드 응답 대기…';
  log(sim ? '시뮬레이터 시작' : `포트 열림 (${BAUD} baud) — 보드 재부팅 대기`, 'ok');
  clearTimeout(S.helloTimer);
  S.helloTimer = setTimeout(() => { if (!S.helloSent) hello(); }, 2600);
  clearInterval(S.pingTimer);
  S.pingTimer = setInterval(() => { if (S.connected) S.link.send('PING'); }, 250);
  updateButtons();
}

function hello() {
  S.helloSent = true;
  send('HELLO');
  applyLimits(true);
  send('GET', true);          // PIDCFG 포함 현재 설정
}

async function disconnect() {
  if (S.connected) { S.link.send('STOP'); await new Promise((r) => setTimeout(r, 60)); }
  const l = S.link;
  cleanupDisconnected();
  if (l) await l.close();
}

function cleanupDisconnected() {
  if (!S.connected) return;
  S.connected = false;
  clearInterval(S.pingTimer); clearTimeout(S.helloTimer);
  if (S.rec) finishRecording('disconnected');
  $('connectBtn').textContent = '보드 연결'; $('simBtn').disabled = false;
  dot('dotLink', ''); $('txtLink').textContent = '연결 안 됨';
  ['dotImu', 'dotCal', 'dotZero', 'dotState'].forEach((d) => dot(d, ''));
  $('benchBadge').hidden = true;
  $('fwId').textContent = '';
  S.link = null;
  updateButtons();
}

// ------------------------------------------------------------------ 수신 처리
function onLine(line) {
  // 리셋 직후 잡음 바이트가 첫 줄 앞에 붙을 수 있음
  const k = line.indexOf('READY AEROPEND');
  if (k > 0) line = line.slice(k);
  // 포트를 연 직후에는 리셋 전 잔여 조각이 올 수 있으므로 첫 READY 전까지 무시 (READY가 없으면 2.6초 뒤 HELLO로 받음)
  if (!S.synced) { if (k < 0) return; S.synced = true; }
  const c = line.charCodeAt(0);
  if (line.startsWith('D,')) return onData(line);
  if (line.startsWith('A,')) return onAux(line);
  if (line.startsWith('S,')) return onStatus(line);
  if (c < 32) return;
  if (line.startsWith('READY ')) {
    S.fw = line.slice(6); $('fwId').textContent = S.fw;
    const ver = parseFloat((S.fw.match(/AEROPEND_LAB (\d+\.\d+)/) || [])[1]);
    S.fwHasPid = ver >= 2.1;
    if (!S.fwHasPid) banner(`보드 펌웨어(${S.fw})가 폐루프 기능 이전 버전입니다. ⑥ 폐루프·PID(이득 적용·기준 유지·폐루프 스텝·릴레이)를 쓰려면 firmware/aeropendulum_lab 을 다시 업로드하세요. 개루프 시험(①–⑤)은 그대로 쓸 수 있습니다.`);
    dot('dotLink', 'good'); $('txtLink').textContent = S.isSim ? '시뮬레이터' : '연결됨';
    log(line, 'ok');
    if (!S.helloSent) hello();
    return;
  }
  if (line.startsWith('CFG ')) return onCfg(line);
  if (line.startsWith('CALVAL ')) return onCalVal(line);
  if (line.startsWith('IMU ')) {
    const was = S.imu;
    S.imu = line.startsWith('IMU READY');
    dot('dotImu', S.imu ? 'good' : 'crit');
    if (S.imu || line !== S.lastImuErr) log(line, S.imu ? 'ok' : 'e');
    if (!S.imu && line.includes('NOT_FOUND') && line !== S.lastImuErr) banner('IMU 없음: ' + line + ' — 배선(VCC 3.3V, GND, SDA→A4, SCL→A5)을 확인하세요. 1초마다 자동 재시도합니다. 센서 없이 로직만 보려면 ② 탭의 벤치 모드를 쓰세요.');
    S.lastImuErr = S.imu ? null : (line.includes('NOT_READY') ? S.lastImuErr : line);
    if (S.imu && !was) banner('');
    return;
  }
  if (line.startsWith('CAL ')) return onCal(line);
  if (line.startsWith('ZERO OK')) { log('영점 설정 완료', 'ok'); return; }
  if (line.startsWith('PIDCFG ')) return onPidCfg(line);
  if (line.startsWith('EVT ')) return onEvent(line.slice(4));
  if (line.startsWith('ERR ')) {
    const code = line.slice(4);
    log(line, 'e');
    let msg = ERRORS[code] || code;
    if ((code === 'UNKNOWN_COMMAND' || code === 'UNKNOWN_PROFILE') && !S.fwHasPid)
      msg += ` — 보드 펌웨어(${S.fw || '?'})에 이 기능이 없습니다. 최신 펌웨어(2.1 이상)를 업로드하세요.`;
    banner('보드 오류: ' + msg);
    if (S.pendingRun) S.pendingRun = null;
    return;
  }
  log(line);
}

const ERRORS = {
  BUSY: '다른 작업(시험/보정/출력)이 진행 중입니다. 정지 후 다시 시도하세요.',
  IMU_NOT_READY: 'IMU가 준비되지 않았습니다.',
  BAD_ARGS: '명령 인자가 올바르지 않습니다.',
  AXIS_PARALLEL_TO_GRAVITY: '선택한 회전축이 중력과 거의 평행합니다. 회전축(X/Y/Z) 선택을 확인하세요.',
  TOO_MANY_STEPS: '계단 수가 너무 많습니다(최대 250).',
  BAD_DURATION: '시험 시간이 너무 깁니다(최대 120000샘플).',
  UNKNOWN_COMMAND: '알 수 없는 명령', COMMAND_TIMEOUT: '명령이 잘려 폐기되었습니다.', INVALID_LINE: '잘못된 문자가 포함된 명령',
};

function onData(line) {
  const p = line.split(',');
  if (p.length < 7) return;
  const tms = +p[1], u = +p[2], th = +p[3] / 100, om = +p[4] / 100, tha = +p[5] / 100, ph = +p[6];
  const ref = p.length > 7 && p[7] !== '' ? +p[7] / 100 : NaN;   // 폐루프일 때만 기준
  if (!Number.isFinite(tms)) return;
  const ts = S.cfg.ts_ms || 10;
  if (S.lastTms != null && tms - S.lastTms > ts * 1.6) S.drops++;
  S.lastTms = tms;
  S.rxCount++;
  const L = S.live, t = tms / 1000;
  L.t.push(t); L.u.push(u); L.th.push(th); L.om.push(om); L.tha.push(tha); L.ph.push(ph); L.ref.push(ref);
  if (L.t.length > LIVE_MAX) for (const k in L) L[k].splice(0, L[k].length - LIVE_MAX + 2000);
  S.last = { tms, u, th, om, tha, ph, ref };
  if (S.rec) {
    const r = S.rec;
    r.tms.push(tms); r.u.push(u); r.th.push(th); r.om.push(om); r.tha.push(tha); r.ph.push(ph); r.ref.push(ref);
  }
}

function onAux(line) {
  const p = line.split(',').slice(1).map(Number);
  if (p.length < 6) return;
  const a = { ax: p[0] / 1000, ay: p[1] / 1000, az: p[2] / 1000, gx: p[3] / 100, gy: p[4] / 100, gz: p[5] / 100, t: performance.now() };
  S.aux.push(a);
  if (S.aux.length > 60) S.aux.shift();
  if (S.axisDetect) S.axisDetect.samples.push(a);
  updateSensorPanel();
}

function onStatus(line) {
  const p = line.split(',').slice(1).map(Number);
  const [state, imu, cal, zero, maxTick, overruns, i2c] = p;
  const gate = p.length > 7 ? p[7] : null, sat = p.length > 8 ? p[8] : 0;
  S.devState = state; S.imu = !!imu;
  S.lastStatus = { state, imu, cal, zero, maxTick, overruns, i2c, gate, sat };
  dot('dotImu', imu ? 'good' : 'crit');
  dot('dotCal', cal ? 'good' : 'warn'); $('txtCal').textContent = cal ? '보정됨' : '미보정';
  dot('dotZero', zero ? 'good' : 'warn'); $('txtZero').textContent = zero ? '영점 설정' : '영점 없음';
  const st = ['대기', '수동 출력', '시험 중', '보정 중', '폐루프 유지'][state] || '?';
  dot('dotState', state === 0 ? 'good' : state === 3 ? 'warn' : 'crit');
  $('txtState').textContent = st;
  $('stepCal').classList.toggle('done', !!cal);
  $('stepZero').classList.toggle('done', !!zero);
  const now = performance.now();
  S.rxHz = S.rxCount * 1000 / (now - S.lastRateAt);
  S.rxCount = 0; S.lastRateAt = now;
  $('rRate').textContent = `${S.rxHz.toFixed(0)} Hz · ${(maxTick / 1000).toFixed(1)} ms` + (overruns || i2c || S.drops ? ` · 누락 ${S.drops}/${overruns}/${i2c}` : '') +
    (gate != null ? ` · 게이트 ${gate}%` : '') + (sat ? ` · 포화 ${sat}` : '');
  $('rRate').title = `수신 ${S.rxHz.toFixed(1)} Hz, 루프 최대 ${maxTick} µs, 수신 누락 ${S.drops}, 주기 초과 ${overruns}, I2C 오류 ${i2c}` +
    (gate != null ? `\n게이트 ${gate}%: 가속도 크기가 1 g±0.15에서 벗어나 Madgwick이 자이로만 적분한 비율 (진동 지표)` : '') +
    `\n포화 ${sat}: 1초 동안 센서 범위(±16 g/±2000 °/s)를 넘은 샘플 수`;
  $('rRate').style.color = sat ? 'var(--crit)' : gate > 50 ? 'var(--warn)' : '';
  if (sat && !S.satWarned) { S.satWarned = true; banner(`센서 포화 감지 (${sat}샘플/초) — 진동이 측정 범위를 넘습니다. 센서를 진동 절연하거나 프로펠러 균형을 확인하세요. 이 구간의 각도는 신뢰하기 어렵습니다.`); log(`센서 포화 ${sat}샘플/초`, 'e'); }
  if (!sat) S.satWarned = false;
  updateButtons();
}

function onCfg(line) {
  const cfg = {};
  for (const kv of line.slice(4).split(' ')) {
    const [k, v] = kv.split('=');
    cfg[k] = /^-?\d+(\.\d+)?$/.test(v) ? +v : v;
  }
  S.cfg = cfg;
  setSeg('axisSeg', cfg.axis); setSeg('signSeg', cfg.sign);
  if (document.activeElement !== $('beta')) $('beta').value = (cfg.beta / 1000).toString();
  if (document.activeElement !== $('dlpf')) $('dlpf').value = cfg.dlpf;
  $('stepAxis').classList.add('done');
  updateLimMsg();
  dot('dotCal', cfg.cal ? 'good' : 'warn'); $('txtCal').textContent = cfg.cal ? '보정됨' : '미보정';
  dot('dotZero', cfg.zero ? 'good' : 'warn'); $('txtZero').textContent = cfg.zero ? '영점 설정' : '영점 없음';
  $('benchMode').checked = !!cfg.bench;
  $('benchBadge').hidden = !cfg.bench;
  updateManualRange();
}

function onCalVal(line) {
  const p = line.split(' ');
  // CALVAL valid gbx gby gbz aox aoy aoz g0 x y z
  const n = p.slice(1).filter((x) => x !== 'g0').map(Number);
  S.calv = { valid: n[0], gb: n.slice(1, 4), ao: n.slice(4, 7), g0: n.slice(7, 10) };
  const f = (a, d) => a.map((v) => (v >= 0 ? ' ' : '') + v.toFixed(d)).join('  ');
  $('calVals').textContent =
    `자이로 바이어스 [°/s] : ${f(S.calv.gb, 3)}   ${S.calv.valid ? '(정지 보정됨, EEPROM)' : '(미보정)'}\n` +
    `가속도 오프셋  [g]   : ${f(S.calv.ao, 4)}   (저장값)\n` +
    `영점 중력방향 g0     : ${f(S.calv.g0, 3)}`;
  const ids = ['aoX', 'aoY', 'aoZ'];
  if (!ids.includes(document.activeElement && document.activeElement.id)) ids.forEach((id, i) => ($(id).value = (S.calv.ao[i] * 1000).toFixed(1)));
}

const CAL_FAIL = {
  MOVING: '움직임/진동이 감지되었습니다. 책상 진동을 줄이고 손을 뗀 채 다시 시도하세요.',
  NOT_Z_UP: '+Z가 위를 향하지 않습니다. 부품면이 위로 오게 뒤집어 놓으세요.',
  NOT_FLAT: '수평이 아닙니다(X/Y 축에 0.2 g 이상). 수평면에 놓으세요.',
  ACC_NORM: '가속도 크기가 1 g에서 15% 넘게 벗어났습니다. 진동 중이거나 가속도 오프셋 값이 잘못되었을 수 있습니다.',
  GYRO_BIAS_RANGE: '자이로 바이어스가 비정상적으로 큽니다(센서 이상 또는 회전 중).',
  CANCELLED: '사용자가 취소했습니다.',
};

function onCal(line) {
  if (line.startsWith('CAL START')) { $('calMsg').textContent = '측정 중… 움직이지 마세요'; $('calBar').style.width = '0%'; log(line); }
  else if (line.startsWith('CAL PROG')) { $('calBar').style.width = line.split(' ')[2] + '%'; }
  else if (line.startsWith('CAL OK')) {
    $('calBar').style.width = '100%'; $('calMsg').textContent = '보정 완료 ✓ ' + line.slice(7); log(line, 'ok');
    if (S.calThenZero) { S.calThenZero = false; send('ZERO'); $('calMsg').textContent += ' → 영점 설정'; }
  }
  else if (line.startsWith('CAL FAIL')) {
    const code = line.split(' ')[2];
    S.calThenZero = false;
    $('calMsg').textContent = '보정 실패: ' + (CAL_FAIL[code] || code) + '  [' + line.split(' ').slice(3).join(' ') + ']';
    $('calBar').style.width = '0%';
    log(line, 'e');
    if (code !== 'CANCELLED') banner('보정 실패 — ' + (CAL_FAIL[code] || code) + ' (기존 보정값 유지)');
  } else log(line);
  updateButtons();
}

function onEvent(ev) {
  const [kind, ...rest] = ev.split(' ');
  if (kind === 'RUN') {
    const kv = Object.fromEntries(rest.slice(1).map((s) => s.split('=')));
    const type = rest[0];
    if (S.rec) finishRecording(S.rec.type === 'MANUAL' ? 'done' : 'interrupted');
    const pr = S.pendingRun && S.pendingRun.type === type ? S.pendingRun : { type, params: {} };
    S.pendingRun = null;
    startRecording(type, pr.params, { total: +kv.total, ts: (+kv.ts_ms || 10) / 1000, extra: pr.extra });
    log(`시험 시작: ${type} (${(+kv.total * (+kv.ts_ms) / 1000).toFixed(1)} s)`, 'ok');
  } else if (kind === 'DONE') {
    S.devState = 0;
    log('시험 완료', 'ok');
    if (S.rec && S.rec.type !== 'MANUAL') finishRecording('done');
  } else if (kind === 'ABORT') {
    const why = rest.join(' ');
    const msg = { ANGLE_LIMIT: '각도 한계 초과로 정지', LINK_TIMEOUT: '통신 두절로 정지', IMU_FAIL: 'IMU 오류로 정지', INVALID_LINE: '잘못된 명령으로 정지' }[why] || why;
    log('중단: ' + msg, 'e');
    banner('출력 중단 — ' + msg);
    if (S.rec && S.rec.type !== 'MANUAL') finishRecording('aborted: ' + why);
    resetManualUi();
  } else if (kind === 'STOP') {
    S.devState = 0;
    log('정지', 'ok');
    if (S.rec && S.rec.type !== 'MANUAL') finishRecording('stopped');
    updateButtons();
  } else log('EVT ' + ev);
}

// ------------------------------------------------------------------ 기록
function startRecording(type, params, o = {}) {
  const now = new Date();
  S.recSeq++;
  S.rec = {
    id: S.recSeq, type, params, extra: o.extra || {}, total: o.total || 0, ts: o.ts || (S.cfg.ts_ms || 10) / 1000,
    name: `${type}_${now.toTimeString().slice(0, 8).replace(/:/g, '')}`, start: now,
    status: 'recording', meta: snapshotMeta(),
    tms: [], u: [], th: [], om: [], tha: [], ph: [], ref: [], pid: S.pidCfg ? Object.assign({}, S.pidCfg) : null,
  };
  updateButtons();
}

function snapshotMeta() {
  const c = S.cfg;
  return { fw: S.fw, axis: `${c.axis}${c.sign}`, beta: c.beta / 1000, dlpf: c.dlpf, cal: c.cal, zero: c.zero, sim: S.isSim ? 1 : 0, bench: c.bench ? 1 : 0 };
}

function finishRecording(status) {
  const r = S.rec;
  if (!r) return;
  S.rec = null;
  r.status = status;
  if (r.tms.length < 5) { log('기록이 너무 짧아 버립니다', 'e'); updateButtons(); return; }
  S.recs.push(r);
  refreshRecTable();
  if (r.type === 'STAIR') showStatic(r);
  if (r.type === 'STEP') showStepResult(r);
  if (r.type === 'CLSTEP') showClResult(r);
  if (r.type === 'RELAY') showRelayResult(r);
  if ($('autoSave').checked && (status === 'done' || r.type === 'MANUAL')) downloadCsv(r);
  updateButtons();
}

function recToCsv(r) {
  const m = r.meta;
  const params = Object.entries(r.params).map(([k, v]) => `${k}=${v}`).join(' ');
  const head = `# aeropendulum_lab; name=${r.name}; type=${r.type}; params=${params}; Ts=${r.ts}; input=pwm(0-255); ` +
    `axis=${m.axis}; beta=${m.beta}; dlpf=${m.dlpf}; cal=${m.cal}; zero=${m.zero}; sim=${m.sim}; bench=${m.bench || 0}; status=${r.status}; fw=${m.fw}; start=${r.start.toISOString()}`;
  const hasRef = r.ref && r.ref.some(Number.isFinite);
  const pidTxt = r.pid ? `# pid; kp=${r.pid.kp}; ki=${r.pid.ki}; kd=${r.pid.kd}; ff=${r.pid.ff}; notch_cHz=${r.pid.notch}; zz=${r.pid.zz}; zp=${r.pid.zp}` : null;
  const rows = [head, 't_s,pwm,duty_pct,theta_deg,omega_dps,theta_acc_deg,phase' + (hasRef ? ',ref_deg' : '')];
  if (pidTxt && hasRef) rows.splice(1, 0, pidTxt);
  const t0 = r.tms[0];
  for (let k = 0; k < r.tms.length; k++) {
    rows.push(`${((r.tms[k] - t0) / 1000).toFixed(3)},${r.u[k]},${duty(r.u[k]).toFixed(2)},${r.th[k].toFixed(2)},${r.om[k].toFixed(2)},${r.tha[k].toFixed(2)},${r.ph[k]}` + (hasRef ? ',' + (Number.isFinite(r.ref[k]) ? r.ref[k].toFixed(2) : '') : ''));
  }
  return rows.join('\n') + '\n';
}

function download(name, text) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'text/csv' }));
  a.download = name;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}

function csvName(r) {
  const d = r.start, z = (n) => String(n).padStart(2, '0');
  return `aero_${r.type}_${d.getFullYear()}${z(d.getMonth() + 1)}${z(d.getDate())}_${z(d.getHours())}${z(d.getMinutes())}${z(d.getSeconds())}${r.status === 'done' || r.type === 'MANUAL' ? '' : '_partial'}.csv`;
}
function downloadCsv(r) { download(csvName(r), recToCsv(r)); log('CSV 저장: ' + csvName(r), 'ok'); }

function parseCsv(text, fileName) {
  const lines = text.split(/\r?\n/).filter((l) => l.trim());
  const meta = {};
  while (lines.length > 1 && lines[1].startsWith('#')) lines.splice(1, 1);   // 추가 주석 줄(# pid ...) 무시
  if (lines[0].startsWith('#')) {
    for (const part of lines.shift().slice(1).split(';')) {
      const i = part.indexOf('=');
      if (i > 0) meta[part.slice(0, i).trim()] = part.slice(i + 1).trim();
    }
  }
  const header = lines.shift().split(',');
  const col = (n) => header.indexOf(n);
  const ci = { t: col('t_s'), u: col('pwm'), th: col('theta_deg'), om: col('omega_dps'), tha: col('theta_acc_deg'), ph: col('phase'), ref: col('ref_deg') };
  if (ci.t < 0 || ci.u < 0 || ci.th < 0) throw new Error('열 이름(t_s,pwm,theta_deg)이 없습니다');
  const r = { tms: [], u: [], th: [], om: [], tha: [], ph: [], ref: [] };
  for (const l of lines) {
    const v = l.split(',').map(Number);
    r.tms.push(v[ci.t] * 1000); r.u.push(v[ci.u]); r.th.push(v[ci.th]);
    r.om.push(ci.om >= 0 ? v[ci.om] : NaN); r.tha.push(ci.tha >= 0 ? v[ci.tha] : NaN); r.ph.push(ci.ph >= 0 ? v[ci.ph] : 1);
    r.ref.push(ci.ref >= 0 && Number.isFinite(v[ci.ref]) ? v[ci.ref] : NaN);
  }
  S.recSeq++;
  const params = {};
  (meta.params || '').split(' ').filter(Boolean).forEach((kv) => { const [k, v] = kv.split('='); params[k] = +v; });
  return Object.assign(r, {
    id: S.recSeq, type: meta.type || 'CSV', params, extra: {}, name: (meta.name || fileName.replace(/\.csv$/i, '')) + ' (파일)',
    ts: +meta.Ts || (r.tms.length > 1 ? (r.tms[r.tms.length - 1] - r.tms[0]) / 1000 / (r.tms.length - 1) : 0.01),
    start: meta.start ? new Date(meta.start) : new Date(), status: meta.status || 'file',
    meta: { fw: meta.fw || '', axis: meta.axis || '', beta: +meta.beta, dlpf: +meta.dlpf, cal: +meta.cal, zero: +meta.zero, sim: +meta.sim || 0 },
  });
}

function refreshRecTable() {
  const tb = $('recTable').querySelector('tbody');
  tb.innerHTML = '';
  for (const r of S.recs) {
    const tr = document.createElement('tr');
    const dur = (r.tms[r.tms.length - 1] - r.tms[0]) / 1000;
    tr.innerHTML = `<td>${r.name}</td><td>${r.type}</td><td>${dur.toFixed(1)}</td><td>${r.tms.length}</td><td>${r.status}</td>` +
      `<td><button class="small" data-act="csv">CSV</button> <button class="small" data-act="view">보기</button> <button class="small" data-act="del">삭제</button></td>`;
    tr.querySelector('[data-act=csv]').onclick = () => downloadCsv(r);
    tr.querySelector('[data-act=view]').onclick = () => viewRecord(r);
    tr.querySelector('[data-act=del]').onclick = () => { S.recs = S.recs.filter((x) => x !== r); refreshRecTable(); };
    tb.appendChild(tr);
  }
  const opts = S.recs.map((r) => `<option value="${r.id}">${r.name} · ${r.type}</option>`).join('');
  const es = $('estSel').value, vs = $('valSel').value;
  $('estSel').innerHTML = opts; $('valSel').innerHTML = opts;
  const dyn = S.recs.filter((r) => r.type !== 'STAIR');
  if (es && S.recs.some((r) => r.id == es)) $('estSel').value = es; else if (dyn.length) $('estSel').value = dyn[dyn.length - 1].id;
  if (vs && S.recs.some((r) => r.id == vs)) $('valSel').value = vs;
  else if (dyn.length > 1) $('valSel').value = dyn[dyn.length - 2].id;
  else if (dyn.length) $('valSel').value = dyn[0].id;
}

function viewRecord(r) {
  const t0 = r.tms[0], t = r.tms.map((v) => (v - t0) / 1000);
  drawChart($('cEst'), {
    x: t, xlabel: `${r.name} — 시간 [s]`, yLabelL: 'θ [°]', yLabelR: 'PWM',
    series: [{ y: r.th, color: '--s1', label: 'θ' }, { y: r.u, color: '--s2', axis: 'R', step: true, width: 1.2, label: 'u' }],
  });
  $('estOut').textContent = `${r.name}: ${r.type} ${Object.entries(r.params).map(([k, v]) => k + '=' + v).join(' ')}\nTs=${r.ts}s, 샘플 ${r.tms.length}, 상태 ${r.status}`;
  selectTab('tData');
}

// ------------------------------------------------------------------ 실시간 그래프
let lastDraw = 0;
function frame(ts) {
  requestAnimationFrame(frame);
  if (ts - lastDraw < 33 || document.hidden) return;
  lastDraw = ts;
  const L = S.live, n = L.t.length;
  if (S.last) {
    const l = S.last;
    $('rTheta').textContent = l.th.toFixed(1) + '°';
    $('rOmega').textContent = l.om.toFixed(1);
    $('rU').textContent = l.u.toFixed(0);
    $('rDuty').textContent = duty(l.u).toFixed(1);
    $('rThAcc').textContent = l.tha.toFixed(1);
    $('zeroTheta').textContent = l.th.toFixed(1);
  }
  if (S.rec) {
    const r = S.rec, pct = r.total ? Math.min(100, r.tms.length / r.total * 100) : 0;
    const bar = { STAIR: 'stBar', CLSTEP: 'clBar', RELAY: 'rlBar' }[r.type] || 'dynBar';
    if (r.total) $(bar).style.width = pct.toFixed(1) + '%';
    const info = { STAIR: 'stInfo', MANUAL: 'recMsg', CLSTEP: 'clInfo', RELAY: 'rlInfo' }[r.type] || 'dynInfo';
    const el = (r.tms.length * r.ts).toFixed(1);
    $(info).textContent = r.total ? `진행 ${el} / ${(r.total * r.ts).toFixed(1)} s (${pct.toFixed(0)}%)` : `기록 중 ${el} s`;
  }
  if (S.paused || !n) return;
  const tEnd = L.t[n - 1];
  let i0 = n - 1;
  while (i0 > 0 && L.t[i0 - 1] >= tEnd - S.win) i0--;
  const t = L.t.slice(i0).map((v) => v - tEnd);
  drawChart($('cMain'), {
    x: t, xmin: -S.win, xmax: 0, yLabelL: 'θ [°]', yLabelR: 'PWM', zeroLine: true,
    series: [
      { y: L.tha.slice(i0), color: '--s5', width: 1, label: 'θacc' },
      { y: L.th.slice(i0), color: '--s1', width: 1.8, label: 'θ (Madgwick)' },
      { y: L.ref.slice(i0), color: '--s4', width: 1.4, dash: [5, 3], step: true, label: '기준' },
      { y: L.u.slice(i0), color: '--s2', axis: 'R', step: true, width: 1.3, label: 'PWM' },
    ],
  });
  drawChart($('cSub'), {
    x: t, xmin: -S.win, xmax: 0, yLabelL: 'ω [°/s]', xlabel: '시간 [s]', zeroLine: true,
    series: [{ y: L.om.slice(i0), color: '--s3', width: 1.4, label: 'ω' }],
  });
}

// ------------------------------------------------------------------ ① 센서·보정
function updateSensorPanel() {
  const n = S.aux.length;
  if (!n) return;
  const a = S.aux[n - 1];
  const ao = S.calv ? S.calv.ao : [0, 0, 0];
  const cx = a.ax - ao[0], cy = a.ay - ao[1], cz = a.az - ao[2];
  const an = Math.hypot(cx, cy, cz);
  $('corrAcc').textContent = `x ${cx.toFixed(3)}  y ${cy.toFixed(3)}  z ${cz.toFixed(3)} g   |a| = ${an.toFixed(3)} g`;
  const recent = S.aux.slice(-10);
  const sd = (k) => { const m = recent.reduce((s, x) => s + x[k], 0) / recent.length; return Math.sqrt(recent.reduce((s, x) => s + (x[k] - m) ** 2, 0) / Math.max(1, recent.length - 1)); };
  const gsd = Math.max(sd('gx'), sd('gy'), sd('gz'));
  $('rawGyro').textContent = `x ${a.gx.toFixed(2)}  y ${a.gy.toFixed(2)}  z ${a.gz.toFixed(2)} °/s  (1초 표준편차 ${gsd.toFixed(2)})`;
  const normOk = an > 0.85 && an < 1.15, still = recent.length >= 5 && gsd < 0.4;
  $('badgeNorm').className = 'badge ' + (normOk ? 'good' : 'crit');
  $('badgeStill').className = 'badge ' + (still ? 'good' : 'warn');
}

function resetCalBtn() { $('calBtn').textContent = $('calZero').checked ? '정지 보정 + 영점' : '정지 보정'; }

function startCalibration(flat = false) {
  if (S.calCountdown) {               // 카운트다운 중 취소
    clearInterval(S.calCountdown); S.calCountdown = null;
    $('calMsg').textContent = '취소됨'; resetCalBtn(); return;
  }
  if (S.devState === 3) { send('STOP'); return; }
  S.calThenZero = !flat && $('calZero').checked;
  let n = 3;
  $('calBtn').textContent = '취소';
  $('calMsg').textContent = `손을 떼세요… ${n}`;
  S.calCountdown = setInterval(() => {
    n--;
    if (n > 0) { $('calMsg').textContent = `손을 떼세요… ${n}`; return; }
    clearInterval(S.calCountdown); S.calCountdown = null;
    resetCalBtn();
    send(flat ? 'CAL FLAT' : 'CAL');
  }, 1000);
}

function startAxisDetect() {
  if (S.axisDetect) return;
  S.axisDetect = { samples: [] };
  $('axisMsg').textContent = '진자를 손으로 흔드세요… (3초)';
  setTimeout(() => {
    const s = S.axisDetect.samples;
    S.axisDetect = null;
    if (s.length < 10) { $('axisMsg').textContent = '데이터 부족 — 연결 상태 확인'; return; }
    const rms = ['gx', 'gy', 'gz'].map((k) => {
      const m = s.reduce((a, x) => a + x[k], 0) / s.length;
      return Math.sqrt(s.reduce((a, x) => a + (x[k] - m) ** 2, 0) / s.length);
    });
    const best = rms.indexOf(Math.max(...rms));
    const sorted = [...rms].sort((a, b) => b - a);
    if (sorted[0] < 8) { $('axisMsg').textContent = `흔들림이 부족합니다 (최대 ${sorted[0].toFixed(1)} °/s RMS)`; return; }
    const axis = 'XYZ'[best];
    $('axisMsg').textContent = `감지: ${axis}축 (RMS ${rms.map((v) => v.toFixed(0)).join(' / ')} °/s)` + (sorted[1] > sorted[0] * 0.5 ? ' — 축이 비스듬히 장착된 듯합니다' : '');
    setSeg('axisSeg', axis);
    send(`AXIS ${axis} ${segValue('signSeg') || '+'}`);
  }, 3000);
}

// ------------------------------------------------------------------ ② 수동 구동
function limitsFromForm() {
  return {
    umax: Math.round(Math.max(0, Math.min(255, +$('uMax').value))),
    alim: Math.round(Math.max(5, Math.min(180, +$('aLim').value)) * 100),
    rev: $('allowRev').checked ? 1 : 0,
    div: +$('rate').value,
  };
}

// what: 'lim' = 한계만, 'stream' = 샘플링만, 생략 = 둘 다
function applyLimits(quiet = false, what) {
  const f = limitsFromForm();
  if (what !== 'stream') send(`LIM ${f.umax} ${f.alim} ${f.rev}`, quiet);
  if (what !== 'lim' && S.devState !== 2 && S.devState !== 3) send(`STREAM ${f.div}`, quiet);
}

// 입력란과 보드 설정(CFG)이 다르면 경고 — 슬라이더·시험 범위는 보드 값을 따름
function updateLimMsg() {
  const c = S.cfg, el = $('limMsg');
  if (c.umax == null) { el.textContent = ''; return; }
  const f = limitsFromForm();
  const diff = f.umax !== c.umax || f.alim !== c.alim || f.rev !== c.rev || (c.div != null && f.div !== c.div);
  el.textContent = `보드: 최대 PWM ${c.umax} · 한계 ${c.alim / 100}° · 역추력 ${c.rev ? '허용' : '금지'} · Ts ${c.ts_ms} ms` +
    (diff ? '  ⚠ 입력값이 아직 보드에 적용되지 않음' : '');
  el.style.color = diff ? 'var(--warn)' : '';
}

function updateManualRange() {
  const max = (S.cfg.umax != null ? S.cfg.umax : +$('uMax').value);
  const rev = S.cfg.rev != null ? !!S.cfg.rev : $('allowRev').checked;
  $('manSlider').max = max; $('manSlider').min = rev ? -max : 0;
  $('manNum').max = max; $('manNum').min = rev ? -max : 0;
}

let manTimer = null, manPending = null;
function showManual(pwm) {
  $('manVal').textContent = pwm.toFixed(0);
  $('manDuty').textContent = `듀티 ${duty(pwm).toFixed(1)} %`;
}
function sendManual(pwm, quiet) {
  const v = Math.round(pwm);
  if (send(`U ${v}`, quiet)) showManual(v);
}
function onSlider() {
  const pwm = +$('manSlider').value;
  $('manNum').value = pwm;
  showManual(pwm);
  if (!$('manLive').checked) return;
  manPending = pwm;
  if (!manTimer) {
    manTimer = setTimeout(function flush() {
      manTimer = null;
      if (manPending != null) { sendManual(manPending, true); manPending = null; }
    }, 50);
  }
}
function resetManualUi() {
  $('manSlider').value = 0; $('manNum').value = 0; $('manLive').checked = false;
  showManual(0);
}

function toggleManualRecord() {
  if (S.rec && S.rec.type === 'MANUAL') { finishRecording('done'); $('recBtn').textContent = '기록 시작'; $('recMsg').textContent = '저장됨'; return; }
  if (S.rec) { banner('시험 기록이 진행 중입니다.'); return; }
  startRecording('MANUAL', {}, { ts: (S.cfg.ts_ms || 10) / 1000 });
  $('recBtn').textContent = '기록 정지';
}

// ------------------------------------------------------------------ ③ 시험1 정적 특성
function runStatic() {
  const a = +$('stStart').value, b = +$('stEnd').value, inc = +$('stInc').value, dwell = +$('stDwell').value;
  if (!(inc > 0) || !(dwell > 0) || a === b) return banner('입력 범위/증분/유지 시간을 확인하세요.');
  const umax = S.cfg.umax;
  if (![a, b, inc].every(Number.isInteger)) return banner('PWM 값은 정수로 입력하세요.');
  if (Math.min(a, b) < 0 || Math.max(a, b) > umax) return banner(`PWM 범위(0–${umax})를 벗어납니다. ② 수동 구동 탭에서 한계를 조정하세요.`);
  if (!S.cfg.zero) log('주의: 영점이 설정되지 않았습니다', 'e');
  const params = { pwm_start: a, pwm_end: b, pwm_inc: inc, dwell_s: dwell };
  S.pendingRun = { type: 'STAIR', params, extra: { settle: +$('stSettle').value / 100 } };
  send(`RUN STAIR ${a} ${b} ${inc} ${Math.round(dwell * 1000)}`);
}

function updateStaticInfo() {
  const a = +$('stStart').value, b = +$('stEnd').value, inc = +$('stInc').value, dwell = +$('stDwell').value;
  if (inc > 0 && dwell > 0) {
    const steps = Math.floor(Math.abs(b - a) / inc + 1e-9) + 1;
    if (!S.rec) $('stInfo').textContent = `${steps}단계 · 약 ${(steps * dwell).toFixed(0)} s`;
  }
}

function showStatic(r) {
  const settle = r.extra && r.extra.settle != null ? r.extra.settle : +$('stSettle').value / 100;
  const res = SysId.staticMap(r, settle);
  S.staticResult = { res, name: r.name };
  const tb = $('stTable').querySelector('tbody');
  tb.innerHTML = '';
  for (const p of res.pts) {
    const tr = document.createElement('tr');
    tr.className = 'clickable';
    tr.innerHTML = `<td>${p.step}</td><td>${p.u.toFixed(0)}</td><td>${duty(p.u).toFixed(1)}</td><td>${p.theta.toFixed(2)}</td><td>${p.std.toFixed(2)}</td><td>${Number.isFinite(p.slope) ? p.slope.toFixed(2) : '—'}</td>`;
    tr.onclick = () => { $('u0').value = Math.round(p.u); updateU0V(); updateDynInfo(); saveForm(); log(`운전점 u0 = PWM ${Math.round(p.u)} (θ≈${p.theta.toFixed(1)}°)`, 'ok'); };
    tb.appendChild(tr);
  }
  const l = res.lin;
  $('stFit').textContent = l ? `sinθ ≈ ${l.k.toFixed(6)}·(PWM − ${l.udz.toFixed(1)})  (R² ${l.r2.toFixed(3)}, ${l.n}점) → 사각지대 ≈ PWM ${l.udz.toFixed(0)}` : '회귀에 쓸 점이 부족합니다 (θ > 2° 인 점 2개 이상 필요)';
  drawStatic();
  selectTab('tStatic');
}

function drawStatic() {
  const sr = S.staticResult;
  if (!sr) return;
  const { res } = sr, sx = 1;
  const series = [{ x: res.pts.map((p) => p.u * sx), y: res.pts.map((p) => p.theta), color: '--s1', points: true, line: true, width: 1, label: 'θss 측정' }];
  if (res.lin) {
    const xs = [], ys = [];
    const u0 = Math.min(...res.pts.map((p) => p.u)), u1 = Math.max(...res.pts.map((p) => p.u));
    for (let i = 0; i <= 80; i++) {
      const u = u0 + (u1 - u0) * i / 80, s = res.lin.k * (u - res.lin.udz);
      xs.push(u * sx); ys.push(s <= 0 ? 0 : s >= 1 ? 90 : Math.asin(s) * 180 / Math.PI);
    }
    series.push({ x: xs, y: ys, color: '--s2', dash: [5, 4], width: 1.4, label: 'asin 모델' });
  }
  drawChart($('cStatic'), { series, xlabel: '입력 PWM (0–255)', yLabelL: 'θss [°]' });
}

function staticCsv() {
  const sr = S.staticResult;
  if (!sr) return banner('시험 1 결과가 없습니다.');
  const rows = ['step,pwm,duty_pct,theta_ss_deg,std_deg,n,dtheta_dpwm'];
  for (const p of sr.res.pts) rows.push(`${p.step},${p.u.toFixed(1)},${duty(p.u).toFixed(2)},${p.theta.toFixed(3)},${p.std.toFixed(3)},${p.n},${Number.isFinite(p.slope) ? p.slope.toFixed(4) : ''}`);
  download(`aero_static_map_${sr.name}.csv`, rows.join('\n') + '\n');
}

// ------------------------------------------------------------------ ④ 시험2 동적
function updateU0V() { $('u0V').textContent = `듀티 ${duty(+$('u0').value).toFixed(1)} %`; }

function dynSpec() {
  const u0 = +$('u0').value, pre = +$('pre').value;
  const t = S.dynType;
  if (t === 'STEP') {
    const du = +$('stepDu').value, hold = +$('stepHold').value, post = +$('stepPost').value;
    return { lo: Math.min(u0, u0 + du), hi: Math.max(u0, u0 + du), dur: pre + hold + post,
      params: { u0, du, pre_s: pre, hold_s: hold, post_s: post },
      cmd: `RUN STEP ${Math.round(u0)} ${Math.round(u0 + du)} ${Math.round(pre * 1000)} ${Math.round(hold * 1000)} ${Math.round(post * 1000)}` };
  }
  if (t === 'CHIRP') {
    const amp = +$('chAmp').value, f0 = +$('chF0').value, f1 = +$('chF1').value, dur = +$('chDur').value, lg = +$('chLog').value;
    return { lo: u0 - amp, hi: u0 + amp, dur: pre + dur, bad: !(f0 > 0 && f1 > 0 && f1 <= 50) && '주파수를 확인하세요 (0 < f ≤ 50 Hz)',
      params: { u0, amp, f0, f1, pre_s: pre, dur_s: dur, log: lg },
      cmd: `RUN CHIRP ${Math.round(u0)} ${Math.round(amp)} ${Math.round(f0 * 1000)} ${Math.round(f1 * 1000)} ${Math.round(pre * 1000)} ${Math.round(dur * 1000)} ${lg}` };
  }
  const amp = +$('prAmp').value, bit = +$('prBit').value, dur = +$('prDur').value;
  return { lo: u0 - amp, hi: u0 + amp, dur: pre + dur,
    params: { u0, amp, bit_ms: bit, pre_s: pre, dur_s: dur },
    cmd: `RUN PRBS ${Math.round(u0)} ${Math.round(amp)} ${Math.round(bit)} ${Math.round(pre * 1000)} ${Math.round(dur * 1000)}` };
}

function updateDynInfo() {
  const s = dynSpec();
  if (!S.rec) $('dynInfo').textContent = `입력 PWM ${Math.round(s.lo)}–${Math.round(s.hi)} · 약 ${s.dur.toFixed(0)} s`;
}

function runDyn() {
  const s = dynSpec();
  if (s.bad) return banner(s.bad);
  const umax = S.cfg.umax;
  if (s.hi > umax) return banner(`최대 입력(PWM ${umax})을 넘습니다 (${Math.round(s.hi)}).`);
  if (s.lo < 0 && !S.cfg.rev) return banner('입력이 PWM 0 아래로 내려갑니다. 운전점을 높이거나 진폭을 줄이세요.');
  if (!S.cfg.zero) log('주의: 영점이 설정되지 않았습니다', 'e');
  S.pendingRun = { type: S.dynType, params: s.params };
  $('dynResult').hidden = true;
  send(s.cmd);
}

function showStepResult(r) {
  const m = SysId.stepMetrics(r, r.ts);
  const el = $('dynResult');
  if (!m) { el.hidden = true; return; }
  if (Math.abs(m.dy) < 0.05) {
    el.textContent = `스텝 PWM ${m.u0.toFixed(0)} → ${(m.u0 + m.du).toFixed(0)}: 각도 변화가 없습니다 (Δθ ${m.dy.toFixed(3)}°).\n` +
      (r.meta.bench ? '벤치 모드 기록이므로 정상입니다 (모터·센서 미사용).' : '센서 연결, 영점, 모터 전원, 사각지대(PWM이 너무 낮음)를 확인하세요.');
    el.hidden = false;
    return;
  }
  let txt = `스텝 PWM ${m.u0.toFixed(0)} → ${(m.u0 + m.du).toFixed(0)}  (θ0 ${m.y0.toFixed(2)}°)\n` +
    `Δθss = ${m.dy.toFixed(2)}°   정상상태 이득 K = ${m.K.toFixed(4)} °/PWM\n` +
    `오버슈트 ${(m.Mp * 100).toFixed(1)}%   첨두시간 ${m.tp.toFixed(2)} s   2% 정착 ${Number.isFinite(m.ts) ? m.ts.toFixed(2) + ' s' : '—'}\n`;
  if (Number.isFinite(m.zeta) && Number.isFinite(m.wn)) {
    txt += `→ ζ ≈ ${m.zeta.toFixed(3)},  ωn ≈ ${m.wn.toFixed(3)} rad/s (${(m.wn / 2 / Math.PI).toFixed(3)} Hz)\n` +
      `   G(s) ≈ ${(m.K * m.wn * m.wn).toFixed(3)} / (s² + ${(2 * m.zeta * m.wn).toFixed(3)} s + ${(m.wn * m.wn).toFixed(3)})   [°/PWM]`;
  } else txt += '오버슈트가 없어 2차 근사 지표를 계산하지 않았습니다 (과감쇠). ⑤ 탭에서 추정하세요.';
  el.textContent = txt;
  el.hidden = false;
}

// ------------------------------------------------------------------ ⑤ 추정
function runEstimate() {
  const er = S.recs.find((r) => r.id == $('estSel').value);
  const vr = S.recs.find((r) => r.id == $('valSel').value);
  if (!er) return banner('추정 데이터를 선택하세요.');
  const nk = Math.max(1, Math.round(+$('nk').value));
  const pe = SysId.prepare(er);
  if (pe.u.length < 50) return banner('추정 데이터가 너무 짧습니다.');
  const m = SysId.fitSecondOrder(pe.u, pe.y, er.ts, { nk });
  if (!m) { $('estOut').textContent = '추정 실패 (불안정하거나 입력 변화가 부족). 다른 기록/nk를 시도하세요.'; return; }
  let out = `추정: ${er.name}  (Ts=${er.ts}s, ${pe.u.length}샘플, 운전점 PWM0=${pe.u0.toFixed(1)}, θ0=${pe.y0.toFixed(2)}°)\n` +
    `이산 모델  A = [1 ${m.a1.toFixed(5)} ${m.a2.toFixed(5)}],  B = [${m.b1.toFixed(5)} ${m.b2.toFixed(5)}],  nk=${nk}\n` +
    `정상상태 이득 K = ${m.K.toFixed(5)} °/PWM\n`;
  if (m.wn) {
    out += `ωn = ${m.wn.toFixed(4)} rad/s (${m.fn.toFixed(3)} Hz),  ζ = ${m.zeta.toFixed(4)}\n` +
      `연속 근사  G(s) = ${(m.K * m.wn * m.wn).toFixed(4)} / (s² + ${(2 * m.zeta * m.wn).toFixed(4)} s + ${(m.wn * m.wn).toFixed(4)})   [°/PWM]\n` +
      `MATLAB:  G = tf(${(m.K * m.wn * m.wn).toFixed(6)}, [1 ${(2 * m.zeta * m.wn).toFixed(5)} ${(m.wn * m.wn).toFixed(5)}]);\n`;
  } else out += '연속시간 변환 불가 (음의 실수 극점) — Ts 또는 nk를 바꿔 보세요.\n';
  out += `적합도(추정 데이터, 시뮬레이션) = ${m.fit.toFixed(1)} %\n`;
  let chart = null;
  if (vr) {
    const pv = SysId.prepare(vr);
    const yh = SysId.simulate(m, pv.u, nk);
    const fit = SysId.fitPercent(pv.y, yh);
    out += `검증: ${vr.name}${vr === er ? ' (추정과 같은 데이터 — 다른 기록으로 검증하세요)' : ''} → 적합도 ${fit.toFixed(1)} %`;
    const t = pv.y.map((_, k) => k * vr.ts);
    chart = { t, y: pv.y.map((v) => v + pv.y0), yh: Array.from(yh, (v) => v + pv.y0), u: pv.u.map((v) => v + pv.u0) };
  }
  $('estOut').textContent = out;
  if (chart) {
    drawChart($('cEst'), {
      x: chart.t, xlabel: '시간 [s] (검증 데이터)', yLabelL: 'θ [°]', yLabelR: 'PWM',
      series: [
        { y: chart.y, color: '--s1', width: 1.4, label: '측정' },
        { y: chart.yh, color: '--s2', width: 1.6, dash: [6, 3], label: '모델' },
        { y: chart.u, color: '--s5', axis: 'R', step: true, width: 1, label: 'PWM' },
      ],
    });
  }
  log(`추정 완료: K=${m.K.toFixed(3)} ωn=${m.wn ? m.wn.toFixed(3) : '—'} ζ=${m.zeta ? m.zeta.toFixed(3) : '—'}`, 'ok');
}

// ------------------------------------------------------------------ ⑥ 폐루프·PID
// analysis/pid_design.py 결과 (식별 모델, K 0.12–0.21 × 공진 ±10% 에서 Ms ≤ 1.6 / P 는 Ms ≤ 2)
const PRESETS = {
  P: { kp: 3.70, ki: 0, kd: 0, notch: false },
  PI: { kp: 2.76, ki: 8.40, kd: 0, notch: false },
  PIN: { kp: 7.85, ki: 23.9, kd: 0, notch: true },
  PIDN: { kp: 7.89, ki: 24.0, kd: 0.129, notch: true },
  ZNPID: { kp: 6.89, ki: 46.1, kd: 0.258, notch: false },
  ZNPIDN: { kp: 6.89, ki: 46.1, kd: 0.258, notch: true },
};

function pidFromForm() {
  const notch = $('notchOn').checked ? { f: +$('nf').value, zz: +$('nzz').value, zp: +$('nzp').value } : null;
  return { kp: +$('kp').value, ki: +$('ki').value, kd: +$('kd').value, ff: Math.round(+$('ff').value), notch };
}

function loadPreset(k) {
  const p = PRESETS[k];
  if (!p) return;
  $('kp').value = p.kp; $('ki').value = p.ki; $('kd').value = p.kd; $('notchOn').checked = p.notch;
  if (p.notch) { $('nf').value = 3.47; $('nzz').value = 0.15; $('nzp').value = 0.7; }
  if (k === 'ZNPID') banner('Z-N PID(노치 없음)는 모델상 이득이 큰 쪽(K≈0.21)이나 공진 −10%에서 불안정합니다. 각도 한계를 낮게 두고 시험하세요.');
  updatePidPred(); saveForm();
}

function applyPid() {
  const g = pidFromForm();
  if (![g.kp, g.ki, g.kd].every((v) => Number.isFinite(v) && v >= 0) || !(g.ff >= 0 && g.ff <= 255)) return banner('이득은 0 이상의 숫자, ff 는 0–255 입니다.');
  if (g.kp > 200 || g.ki > 500 || g.kd > 50) return banner('이득 범위: Kp ≤ 200, Ki ≤ 500, Kd ≤ 50 (펌웨어 제한).');
  if (g.notch && g.notch.f > 0 && !(g.notch.f >= 0.1 && g.notch.f <= 20 && g.notch.zz >= 0 && g.notch.zz <= 2 && g.notch.zp > 0 && g.notch.zp <= 2))
    return banner('노치 범위: f 0.1–20 Hz, ζz 0–2, ζp 0–2 (ζp > 0).');
  if (S.connected && !S.isSim && S.fwHasPid === false) banner(`보드 펌웨어(${S.fw})에는 PID 명령이 없습니다. 펌웨어를 다시 업로드하세요.`);
  send(`PID ${Math.round(g.kp * 1000)} ${Math.round(g.ki * 1000)} ${Math.round(g.kd * 1000)} ${g.ff}`);
  if (g.notch && g.notch.f > 0) send(`NOTCH ${Math.round(g.notch.f * 100)} ${Math.round(g.notch.zz * 1000)} ${Math.round(g.notch.zp * 1000)}`);
  else send('NOTCH 0');
}

function onPidCfg(line) {
  const c = {};
  for (const kv of line.slice(7).split(' ')) { const [k, v] = kv.split('='); c[k] = +v; }
  S.pidCfg = c;
  $('pidMsg').textContent = `보드: Kp ${c.kp} · Ki ${c.ki} · Kd ${c.kd} · ff ${c.ff} · 노치 ${c.notch ? (c.notch / 100).toFixed(2) + ' Hz (ζz ' + c.zz / 1000 + ', ζp ' + c.zp / 1000 + ')' : '끔'}`;
}

function updatePidPred() {
  const g = pidFromForm();
  const ti = g.ki > 0 ? g.kp / g.ki : Infinity, td = g.kp > 0 ? g.kd / g.kp : 0;
  $('tiTxt').textContent = Number.isFinite(ti) ? `Ti = ${ti.toFixed(3)} s` : '';
  $('tdTxt').textContent = td ? `Td = ${(td * 1000).toFixed(0)} ms` : '';
  const ref = []; for (let k = 0; k < 800; k++) ref.push(k < 100 ? 20 : 25);
  const lines = [];
  for (const K of [0.12, 0.15, 0.21]) {
    const r = SysId.simulateClosedLoop({ K }, g, ref);
    const rec = { th: Array.from(r.th), u: Array.from(r.u), ref, ph: ref.map((v, k) => (k < 100 ? 1 : 2)), ts: 0.01 };
    const m = SysId.clStepMetrics(rec);
    const blow = !m || !Number.isFinite(m.os) || Math.max(...rec.th.map(Math.abs)) > 200;
    lines.push(blow ? `K=${K.toFixed(2)}: 발산 또는 진동 지속 — 이득을 낮추세요`
      : `K=${K.toFixed(2)}: 상승 ${m.tr.toFixed(2)} s · 오버슈트 ${m.os.toFixed(1)}% · 정착(5%) ${Number.isFinite(m.ts5) ? m.ts5.toFixed(2) + ' s' : '—'} · 정상오차 ${m.ess.toFixed(2)}° · PWM ${m.uMin.toFixed(0)}–${m.uPeak.toFixed(0)}`);
  }
  $('pidPred').textContent = '모델 예측 (기준 20°→25°, 공진 3.47 Hz, 자이로 잡음 없음):\n' + lines.join('\n');
}

function runClStep() {
  const r0 = +$('clR0').value, r1 = +$('clR1').value, pre = +$('clPre').value, hold = +$('clHold').value, post = +$('clPost').value;
  if (!S.pidCfg || !(S.pidCfg.kp > 0 || S.pidCfg.ki > 0)) return banner('먼저 이득을 적용하세요 (이득 적용).');
  const lim = (S.cfg.alim || 12000) / 100;
  if (Math.max(Math.abs(r0), Math.abs(r1)) > lim - 5) return banner(`기준이 각도 한계(${lim}°)에 너무 가깝습니다.`);
  S.pendingRun = { type: 'CLSTEP', params: { r0, r1, pre_s: pre, hold_s: hold, post_s: post } };
  $('clResult').hidden = true;
  send(`RUN CLSTEP ${Math.round(r0 * 100)} ${Math.round(r1 * 100)} ${Math.round(pre * 1000)} ${Math.round(hold * 1000)} ${Math.round(post * 1000)}`);
}

function showClResult(r) {
  const m = SysId.clStepMetrics(r);
  const el = $('clResult');
  if (!m) { el.textContent = '지표를 계산할 수 없습니다 (스텝 구간이 짧거나 기준 정보 없음).'; el.hidden = false; return; }
  const g = r.pid ? { kp: r.pid.kp, ki: r.pid.ki, kd: r.pid.kd, ff: r.pid.ff, notch: r.pid.notch ? { f: r.pid.notch / 100, zz: r.pid.zz / 1000, zp: r.pid.zp / 1000 } : null } : pidFromForm();
  el.textContent =
    `기준 ${m.r0}° → ${m.r1}°  (Kp ${g.kp}, Ki ${g.ki}, Kd ${g.kd}, ff ${g.ff}${g.notch ? ', 노치 ' + g.notch.f + ' Hz' : ''})\n` +
    `상승시간(10–90%) ${Number.isFinite(m.tr) ? m.tr.toFixed(2) + ' s' : '—'}   오버슈트 ${m.os.toFixed(1)}%   첨두 ${m.tp.toFixed(2)} s   정착(±5%) ${Number.isFinite(m.ts5) ? m.ts5.toFixed(2) + ' s' : '구간 내 미정착'}\n` +
    `정상상태 오차 ${m.ess.toFixed(2)}°   스텝 직전 오차 ${m.preErr.toFixed(2)}°   PWM 범위 ${m.uMin.toFixed(0)}–${m.uPeak.toFixed(0)} (포화 ${(m.sat * 100).toFixed(0)}%)`;
  el.hidden = false;
  // 측정 vs 모델 예측 (스텝 1 s 전부터)
  const i2 = r.ph.indexOf(2); const s0 = Math.max(0, i2 - Math.round(1 / r.ts));
  const ref = r.ref.slice(s0).map((v) => (Number.isFinite(v) ? v : m.r0));
  const t = ref.map((_, k) => k * r.ts);
  const series = [{ y: r.th.slice(s0), color: '--s1', width: 1.2, label: '측정 θ' }, { y: ref, color: '--s4', dash: [5, 3], step: true, label: '기준' }];
  for (const [K, col] of [[0.12, '--s3'], [0.21, '--s2']]) {
    const sim = SysId.simulateClosedLoop({ K }, g, ref, r.ts);
    series.push({ y: Array.from(sim.th), color: col, width: 1, dash: [2, 2], label: `모델 K=${K}` });
  }
  $('cCl').hidden = false;
  S.clChart = { x: t, xlabel: '시간 [s] (스텝 1 s 전부터)', yLabelL: 'θ [°]', series };
  selectTab('tPid');   // 탭 전환 후 다음 프레임에 그림 (레이아웃 크기 확정 후)
}

function runRelay() {
  const rr = +$('rlR').value, d = Math.round(+$('rlD').value), h = +$('rlH').value, pre = +$('rlPre').value, dur = +$('rlDur').value;
  if (!S.pidCfg || !(S.pidCfg.ki > 0)) return banner('유지 구간에서 평형 PWM을 찾기 위해 적분이 있는 이득(PI 등)을 먼저 적용하세요.');
  if (!(d >= 1 && d <= 60)) return banner('d 는 1–60 PWM 범위로 하세요.');
  S.pendingRun = { type: 'RELAY', params: { r: rr, d, hyst: h, pre_s: pre, dur_s: dur } };
  $('rlResult').hidden = true; $('znBtns').hidden = true;
  send(`RUN RELAY ${Math.round(rr * 100)} ${d} ${Math.round(h * 100)} ${Math.round(pre * 1000)} ${Math.round(dur * 1000)}`);
}

function showRelayResult(r) {
  const d = r.params.d ?? +$('rlD').value, h = r.params.hyst ?? +$('rlH').value;
  const res = SysId.relayAnalyze(r, d, h);
  const el = $('rlResult');
  if (!res) { el.textContent = '지속 진동을 찾지 못했습니다. d 를 키우거나 히스테리시스를 줄이세요.'; el.hidden = false; return; }
  S.zn = SysId.zieglerNichols(res.Ku, res.Tu);
  const z = S.zn, f = (g) => `Kp ${g.kp.toFixed(2)}  Ki ${g.ki.toFixed(2)}  Kd ${g.kd.toFixed(3)}`;
  el.textContent =
    `릴레이 진동: 진폭 a = ${res.a.toFixed(2)}° (±${res.aSd.toFixed(2)}), 주기 Tu = ${res.Tu.toFixed(3)} s (${(1 / res.Tu).toFixed(2)} Hz, ±${(res.TuSd * 1000).toFixed(0)} ms), ${res.cycles}주기\n` +
    `한계이득 Ku = 4d/(π√(a²−ε²)) = ${res.Ku.toFixed(2)} PWM/°   (d=${d}, ε=${h}°)\n` +
    `Z-N  P  : ${f(z.P)}\nZ-N  PI : ${f(z.PI)}\nZ-N  PID: ${f(z.PID)}\n` +
    (res.Tu < 0.5 ? '※ Tu 가 0.5 s 미만 = 진동이 3–4 Hz 구조 공진에서 일어났습니다. Z-N 이득은 진자 동특성이 아니라 공진으로 정해지므로 공격적일 수 있습니다. 노치를 켜거나 이득을 줄여 시작하세요.' : '');
  el.hidden = false; $('znBtns').hidden = false;
  selectTab('tPid');
}

function loadZn(kind) {
  const g = S.zn && S.zn[kind];
  if (!g) return;
  $('kp').value = +g.kp.toFixed(3); $('ki').value = +g.ki.toFixed(3); $('kd').value = +g.kd.toFixed(4); $('notchOn').checked = false;
  updatePidPred(); saveForm();
  log(`Z-N ${kind} 이득을 입력란에 불러옴 (적용은 [이득 적용])`, 'ok');
}

// ------------------------------------------------------------------ UI 공통
function selectTab(id) {
  document.querySelectorAll('.tabs button').forEach((b) => b.setAttribute('aria-selected', b.dataset.tab === id));
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.id === id));
  if (id === 'tStatic') requestAnimationFrame(drawStatic);
  if (id === 'tPid' && S.clChart) requestAnimationFrame(() => drawChart($('cCl'), S.clChart));
}
function setSeg(id, v) { document.querySelectorAll(`#${id} button`).forEach((b) => b.setAttribute('aria-pressed', b.dataset.v === String(v))); }
function segValue(id) { const b = document.querySelector(`#${id} button[aria-pressed=true]`); return b && b.dataset.v; }
function bindSeg(id, fn) {
  document.querySelectorAll(`#${id} button`).forEach((b) => b.addEventListener('click', () => { setSeg(id, b.dataset.v); fn && fn(b.dataset.v); }));
}

function updateButtons() {
  const c = S.connected, running = S.rec && S.rec.type !== 'MANUAL', calibrating = S.devState === 3 || !!S.calCountdown;
  const busy = running || calibrating || S.devState === 2;
  const set = (id, en) => { $(id).disabled = !en; };
  set('stopBtn', c);
  set('calBtn', c && !running && S.devState !== 2 && S.devState !== 1);
  set('calClearBtn', c && !busy);
  set('calFlatBtn', c && !busy && S.devState !== 1); set('accOffBtn', c && !busy); set('accDefBtn', c && !busy);
  set('axisApplyBtn', c && !busy); set('axisAutoBtn', c && !busy);
  set('zeroBtn', c && !busy && S.devState !== 1); set('signFlipBtn', c && !busy);
  set('betaBtn', c); set('dlpfBtn', c); set('factoryBtn', c && !busy);
  set('limApplyBtn', c);
  set('manSetBtn', c && !busy); set('manZeroBtn', c);
  set('recBtn', c && !running);
  set('stRunBtn', c && !busy); set('dynRunBtn', c && !busy);
  set('rawSendBtn', c);
  set('pidApplyBtn', c); set('refBtn', c && !running && S.devState !== 2 && S.devState !== 3);
  set('clRunBtn', c && !busy); set('rlRunBtn', c && !busy);
}

const PERSIST = ['kp', 'ki', 'kd', 'ff', 'notchOn', 'nf', 'nzz', 'nzp', 'refVal', 'clR0', 'clR1', 'clPre', 'clHold', 'clPost',
  'rlR', 'rlD', 'rlH', 'rlPre', 'rlDur', 'calZero', 'uMax', 'aLim', 'allowRev', 'rate', 'stStart', 'stEnd', 'stInc', 'stDwell', 'stSettle', 'u0', 'pre',
  'stepDu', 'stepHold', 'stepPost', 'chAmp', 'chF0', 'chF1', 'chDur', 'chLog', 'prAmp', 'prBit', 'prDur', 'autoSave', 'nk'];
function saveForm() {
  const o = {};
  for (const id of PERSIST) { const el = $(id); o[id] = el.type === 'checkbox' ? el.checked : el.value; }
  o.dynType = S.dynType;
  localStorage.setItem('aeroLab.form.v2', JSON.stringify(o));
}
function loadForm() {
  try {
    const o = JSON.parse(localStorage.getItem('aeroLab.form.v2') || '{}');
    for (const id of PERSIST) if (id in o) { const el = $(id); if (el.type === 'checkbox') el.checked = o[id]; else el.value = o[id]; }
    if (o.dynType) S.dynType = o.dynType;
  } catch (_) {}
}

function stopNow() {
  if (S.calCountdown) { clearInterval(S.calCountdown); S.calCountdown = null; resetCalBtn(); $('calMsg').textContent = '취소됨'; }
  S.calThenZero = false;
  resetManualUi();
  if (S.connected) { S.link.send('STOP'); log('> STOP', 'tx'); }
}

function init() {
  loadForm();
  $('connectBtn').onclick = () => connect(S.isSim && S.connected ? true : false);
  $('simBtn').onclick = () => connect(true);
  $('stopBtn').onclick = stopNow;
  document.addEventListener('keydown', (e) => {
    const typing = e.target.matches('input[type=text], input[type=number], textarea');
    if (e.key === 'Escape' || (e.code === 'Space' && !typing)) { e.preventDefault(); stopNow(); }
  }, true);
  document.querySelectorAll('.tabs button').forEach((b) => (b.onclick = () => selectTab(b.dataset.tab)));

  // ①
  $('calBtn').onclick = () => startCalibration(false);
  $('calZero').onchange = () => { if (!S.calCountdown) resetCalBtn(); };
  $('calFlatBtn').onclick = () => { if (confirm('수평면 보정: 센서 +Z를 위로 해서 수평면에 놓았나요? 가속도 오프셋이 새 측정값으로 바뀝니다.')) startCalibration(true); };
  $('calClearBtn').onclick = () => { if (confirm('자이로 바이어스 보정을 해제할까요? (가속도 오프셋은 유지)')) send('CAL CLEAR'); };
  $('accOffBtn').onclick = () => {
    const v = ['aoX', 'aoY', 'aoZ'].map((id) => Math.round(+$(id).value));
    if (v.some((x) => !Number.isFinite(x) || Math.abs(x) > 500)) return banner('가속도 오프셋은 ±500 mg 범위입니다.');
    send(`ACCOFF ${v.join(' ')}`);
  };
  $('accDefBtn').onclick = () => send('ACCOFF DEFAULT');
  bindSeg('axisSeg'); bindSeg('signSeg');
  $('axisApplyBtn').onclick = () => send(`AXIS ${segValue('axisSeg') || 'X'} ${segValue('signSeg') || '+'}`);
  $('axisAutoBtn').onclick = startAxisDetect;
  $('zeroBtn').onclick = () => send('ZERO');
  $('signFlipBtn').onclick = () => send(`AXIS ${S.cfg.axis || 'X'} ${S.cfg.sign === '-' ? '+' : '-'}`);
  $('betaBtn').onclick = () => { const b = +$('beta').value; if (b > 0 && b <= 1) send(`BETA ${Math.max(1, Math.round(b * 1000))}`); else banner('β는 0.001–1 범위입니다.'); };
  $('dlpfBtn').onclick = () => send(`DLPF ${$('dlpf').value}`);
  $('factoryBtn').onclick = () => { if (confirm('EEPROM의 보정·영점·축·필터 설정을 모두 초기화할까요?')) send('FACTORY'); };

  // ②
  $('limApplyBtn').onclick = () => applyLimits(false);
  // 한계·샘플링 입력은 바꾸는 즉시 보드에 적용 (연결 중일 때)
  ['uMax', 'aLim', 'allowRev'].forEach((id) => $(id).addEventListener('change', () => { if (S.connected) applyLimits(false, 'lim'); else updateLimMsg(); }));
  $('rate').addEventListener('change', () => { if (S.connected) applyLimits(false, 'stream'); else updateLimMsg(); });
  ['uMax', 'aLim'].forEach((id) => $(id).addEventListener('input', updateLimMsg));
  $('benchMode').onchange = () => {
    const on = $('benchMode').checked;
    if (!send(`BENCH ${on ? 1 : 0}`)) $('benchMode').checked = !on;
    else if (on) log('벤치 모드: 시험 로직만 실행되고 모터 핀은 LOW로 유지됩니다', 'ok');
  };
  $('manSlider').oninput = onSlider;
  $('manSetBtn').onclick = () => { const v = +$('manNum').value; $('manSlider').value = v; sendManual(v, false); };
  $('manZeroBtn').onclick = () => { resetManualUi(); sendManual(0, false); };
  $('manLive').onchange = () => { if ($('manLive').checked) onSlider(); };
  $('recBtn').onclick = toggleManualRecord;
  $('u0').addEventListener('input', updateU0V);

  // ③
  $('stRunBtn').onclick = runStatic;
  ['stStart', 'stEnd', 'stInc', 'stDwell'].forEach((id) => $(id).addEventListener('input', updateStaticInfo));
  $('stCsvBtn').onclick = staticCsv;

  // ④
  bindSeg('dynSeg', (v) => {
    S.dynType = v;
    document.querySelectorAll('[data-dyn]').forEach((el) => (el.hidden = el.dataset.dyn !== v));
    updateDynInfo(); saveForm();
  });
  setSeg('dynSeg', S.dynType);
  document.querySelectorAll('[data-dyn]').forEach((el) => (el.hidden = el.dataset.dyn !== S.dynType));
  document.querySelectorAll('#tDyn input, #tDyn select').forEach((el) => el.addEventListener('input', updateDynInfo));
  $('dynRunBtn').onclick = runDyn;

  // ⑥
  $('pidPresetBtn').onclick = () => loadPreset($('pidPreset').value);
  $('pidApplyBtn').onclick = applyPid;
  ['kp', 'ki', 'kd', 'ff', 'notchOn', 'nf', 'nzz', 'nzp'].forEach((id) => $(id).addEventListener('input', updatePidPred));
  $('notchOn').addEventListener('change', updatePidPred);
  $('refBtn').onclick = () => { const r = +$('refVal').value; if (Number.isFinite(r)) send(`REF ${Math.round(r * 100)}`); };
  $('clRunBtn').onclick = runClStep;
  $('rlRunBtn').onclick = runRelay;
  document.querySelectorAll('#znBtns button').forEach((b) => (b.onclick = () => loadZn(b.dataset.zn)));
  updatePidPred();

  // ⑤
  $('estBtn').onclick = runEstimate;
  $('saveAllBtn').onclick = () => S.recs.forEach((r, i) => setTimeout(() => downloadCsv(r), i * 300));
  $('loadCsvBtn').onclick = () => $('csvFile').click();
  $('csvFile').onchange = async (e) => {
    for (const f of e.target.files) {
      try { const r = parseCsv(await f.text(), f.name); S.recs.push(r); log(`불러옴: ${f.name} (${r.tms.length}샘플)`, 'ok'); if (r.type === 'STAIR') showStatic(r); }
      catch (err) { banner(`${f.name}: ${err.message}`); }
    }
    e.target.value = '';
    refreshRecTable();
  };

  // 공통
  bindSeg('winSeg', (v) => (S.win = +v));
  $('pauseBtn').onclick = () => { S.paused = !S.paused; $('pauseBtn').textContent = S.paused ? '재개' : '일시정지'; };
  $('rawSendBtn').onclick = () => { const v = $('rawCmd').value.trim(); if (v) { send(v); $('rawCmd').value = ''; } };
  $('rawCmd').addEventListener('keydown', (e) => { if (e.key === 'Enter') $('rawSendBtn').click(); });
  document.querySelectorAll('input, select').forEach((el) => el.addEventListener('change', saveForm));
  window.addEventListener('beforeunload', (e) => {
    if (S.connected) S.link.send('STOP');
    if (S.recs.length || S.rec) { e.preventDefault(); e.returnValue = ''; }
  });

  updateU0V(); updateStaticInfo(); updateDynInfo(); updateManualRange(); resetCalBtn(); updateButtons();
  if (!('serial' in navigator)) log('Web Serial 미지원 브라우저 — Edge/Chrome에서 여세요. 시뮬레이터는 사용 가능합니다.', 'e');
  log('준비됨. [보드 연결] 또는 [시뮬레이터]를 누르세요.');
  requestAnimationFrame(frame);
}

init();
