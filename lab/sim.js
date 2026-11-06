// 펌웨어 프로토콜을 흉내 내는 에어로펜듈럼 시뮬레이터 (하드웨어 없이 UI·PID 연습/검증용)
// 플랜트 = 2026-11-06 식별 결과 (data/20261106_run3/analysis/REPORT.md):
//   θss(PWM) = 정적 특성(run2 계단 평균, 운전점 PWM 150 기울기 ≈ 0.2 °/PWM)
//   동특성   = 지연 16 ms → 1차 지연 T = 0.328 s → 구조 공진 3.47 Hz, ζ = 0.123
// 입력 u 는 펌웨어와 같이 PWM 카운트(0..255). 폐루프(PID/노치/CLSTEP/RELAY/REF)도 펌웨어와 같은 식으로 계산.
(function (root) {
  'use strict';

  const MAP = [[0, 0], [20, 0], [50, 5.4], [60, 6.75], [70, 7.65], [80, 9.4], [90, 10.55], [100, 11.95], [110, 13.4], [120, 15.1],
    [130, 17.25], [140, 19.6], [150, 21.75], [160, 23.5], [170, 25.3], [180, 26.9], [190, 28.45], [200, 30.25], [210, 31.55],
    [220, 33.15], [230, 35.3], [240, 37.5], [250, 40.0], [255, 41.0]];
  function thetaSS(pwm) {
    const u = Math.max(0, Math.min(255, Math.abs(pwm)));
    for (let i = 1; i < MAP.length; i++) {
      if (u <= MAP[i][0]) { const [a, ya] = MAP[i - 1], [b, yb] = MAP[i]; return ya + (yb - ya) * (u - a) / (b - a); }
    }
    return MAP[MAP.length - 1][1];
  }

  class SimDevice {
    constructor() {
      this.onLine = null;
      this.p = { T: 0.328, L: 0.016, wr: 21.8, zr: 0.123, noise: 0.04, mount: 3.0, gain: 1.0 };
      this.x1 = 0; this.y = 0; this.yd = 0; this.ubuf = new Array(16).fill(0);
      this.cfg = { axis: 'X', sign: '+', beta: 20, dlpf: 4, div: 2, umax: 255, alim: 12000, rev: 0, bench: 0, cal: 0, zero: 0 };
      this.defaultAo = [-0.017059, -0.022036, 0.147525];
      this.calv = { gb: [0, 0, 0], ao: this.defaultAo.slice(), g0: [0, 0, 1] };
      this.trueBias = [0.92, -1.01, -0.15];
      this.trueAccOff = [-0.017, -0.022, 0.147];
      this.pid = { kp: 0, ki: 0, kd: 0, ff: 150, nfc: 0, nzz: 150, nzp: 700, b: [1, 0, 0], a: [0, 0] };
      this.pidI = 0; this.omF = 0; this.nz1 = 0; this.nz2 = 0; this.ref = 0; this.clHold = false;
      this.theta = 0; this.omega = 0;
      this.uMan = 0; this.uOut = 0; this.phase = 0;
      this.prof = null; this.cal = null;
      this.lastHb = 0; this.t0 = performance.now(); this.simT = 0;
      this.auxCnt = 0; this.statusT = 0; this.seed = 12345;
      this.closed = false;
      setTimeout(() => {
        this.emit('READY AEROPEND_LAB 2.0 (SIM)');
        this.emit('IMU READY addr=0x68');
        this.printCfg(); this.printCal(); this.printPid();
      }, 150);
      this.timer = setInterval(() => this.run(), 20);
    }

    emit(line) { if (this.onLine && !this.closed) this.onLine(line); }
    rnd() { this.seed = (this.seed * 16807) % 2147483647; return this.seed / 2147483647 - 0.5; }
    gauss() { let s = 0; for (let i = 0; i < 6; i++) s += this.rnd(); return s * 1.41; }
    ts() { return 0.005 * this.cfg.div; }
    nowMs() { return Math.round(this.simT * 1000); }

    printCfg() {
      const c = this.cfg;
      this.emit(`CFG axis=${c.axis} sign=${c.sign} beta=${c.beta} dlpf=${c.dlpf} div=${c.div} ts_ms=${5 * c.div} umax=${c.umax} alim=${c.alim} rev=${c.rev} bench=${c.bench} arng=16 grng=2000 cal=${c.cal} zero=${c.zero}`);
    }
    printCal() {
      const v = this.calv, f = (x) => x.toFixed(4);
      this.emit(`CALVAL ${this.cfg.cal} ${v.gb.map(f).join(' ')} ${v.ao.map(f).join(' ')} g0 ${v.g0.map(f).join(' ')}`);
    }
    printPid() {
      const q = this.pid;
      this.emit(`PIDCFG kp=${q.kp.toFixed(3)} ki=${q.ki.toFixed(3)} kd=${q.kd.toFixed(3)} ff=${q.ff} notch=${q.nfc} zz=${q.nzz} zp=${q.nzp}`);
    }

    // ---- 폐루프 (펌웨어 pidStep 과 같은 식)
    notchCoeffs() {
      const q = this.pid;
      q.b = [1, 0, 0]; q.a = [0, 0];
      if (!q.nfc) return;
      const ts = this.ts(), w = 2 * Math.PI * q.nfc * 0.01, k = w / Math.tan(w * ts / 2), k2 = k * k, w2 = w * w;
      const zz = q.nzz / 1000, zp = q.nzp / 1000, a0 = k2 + 2 * zp * w * k + w2;
      q.b = [(k2 + 2 * zz * w * k + w2) / a0, 2 * (w2 - k2) / a0, (k2 - 2 * zz * w * k + w2) / a0];
      q.a = [2 * (w2 - k2) / a0, (k2 - 2 * zp * w * k + w2) / a0];
    }
    pidReset() { this.pidI = 0; this.omF = this.omega; this.nz1 = this.nz2 = 0; this.notchCoeffs(); }
    pidStep(r) {
      const q = this.pid, ts = this.ts(), e = r - this.theta;
      this.omF += ts / (0.0159 + ts) * (this.omega - this.omF);
      let v = q.kp * e + this.pidI - q.kd * this.omF;
      if (q.nfc) {
        const y = q.b[0] * v + this.nz1;
        this.nz1 = q.b[1] * v - q.a[0] * y + this.nz2;
        this.nz2 = q.b[2] * v - q.a[1] * y;
        v = y;
      }
      const u = q.ff + v, lo = this.cfg.rev ? -this.cfg.umax : 0, hi = this.cfg.umax;
      const us = Math.min(hi, Math.max(lo, u));
      if (u === us || (u > us && e < 0) || (u < us && e > 0)) this.pidI = Math.max(-255, Math.min(255, this.pidI + q.ki * e * ts));
      return Math.round(us);
    }

    write(line) {
      if (this.closed) return;
      this.lastHb = this.simT;
      const tok = line.trim().split(/\s+/), cmd = tok[0];
      const ints = tok.slice(1).map(Number);
      const busy = this.prof || this.cal;
      const bad = () => this.emit('ERR BAD_ARGS');
      switch (cmd) {
        case 'PING': return;
        case 'HELLO': this.stopAll(); this.emit('READY AEROPEND_LAB 2.0 (SIM)'); this.emit('IMU READY'); this.printCfg(); this.printCal(); return;
        case 'GET': this.printCfg(); this.printCal(); this.printPid(); return;
        case 'STOP':
          if (this.cal) { this.cal = null; this.emit('CAL FAIL CANCELLED'); }
          this.stopAll(); this.emit('EVT STOP'); return;
        case 'U':
          if (busy) return this.emit('ERR BUSY');
          if (tok.length !== 2 || !(Math.abs(ints[0]) <= 255)) return bad();
          this.clHold = false; this.uMan = ints[0]; return;
        case 'PID':
          if (tok.length !== 5 || ints.some((x) => !(x >= 0)) || ints[3] > 255) return bad();
          Object.assign(this.pid, { kp: ints[0] / 1000, ki: ints[1] / 1000, kd: ints[2] / 1000, ff: ints[3] });
          return this.printPid();
        case 'NOTCH':
          if (tok.length === 2 && ints[0] === 0) this.pid.nfc = 0;
          else if (tok.length === 4 && ints[0] >= 10 && ints[0] <= 2000 && ints[2] > 0) Object.assign(this.pid, { nfc: ints[0], nzz: ints[1], nzp: ints[2] });
          else return bad();
          this.nz1 = this.nz2 = 0; this.notchCoeffs(); return this.printPid();
        case 'REF':
          if (busy) return this.emit('ERR BUSY');
          if (tok.length !== 2 || !(Math.abs(ints[0]) <= 17000)) return bad();
          if (!this.clHold) { this.pidReset(); this.uMan = 0; this.clHold = true; }
          this.ref = ints[0] / 100; this.emit(`EVT REF ${ints[0]}`); return;
        case 'LIM':
          if (tok.length < 3 || !(ints[0] >= 0 && ints[0] <= 255 && ints[1] >= 500 && ints[1] <= 18000)) return bad();
          this.cfg.umax = ints[0]; this.cfg.alim = ints[1]; if (tok.length > 3) this.cfg.rev = ints[2] ? 1 : 0;
          return this.printCfg();
        case 'STREAM':
          if (busy) return this.emit('ERR BUSY');
          if (!(ints[0] >= 1 && ints[0] <= 20)) return bad();
          this.cfg.div = ints[0]; this.notchCoeffs(); return this.printCfg();
        case 'AXIS':
          if (!/^[XYZ]$/.test(tok[1] || '') || !/^[+-]$/.test(tok[2] || '')) return bad();
          this.cfg.axis = tok[1]; this.cfg.sign = tok[2]; return this.printCfg();
        case 'BENCH':
          if (busy || this.uOut) return this.emit('ERR BUSY');
          if (!(ints[0] === 0 || ints[0] === 1)) return bad();
          this.cfg.bench = ints[0]; return this.printCfg();
        case 'BETA': if (!(ints[0] >= 1 && ints[0] <= 1000)) return bad(); this.cfg.beta = ints[0]; return this.printCfg();
        case 'DLPF': if (!(ints[0] >= 0 && ints[0] <= 6)) return bad(); this.cfg.dlpf = ints[0]; return this.printCfg();
        case 'ZERO':
          if (busy || this.uOut) return this.emit('ERR BUSY');
          this.cfg.zero = 1; this.calv.g0 = [0, Math.sin(this.p.mount * Math.PI / 180), Math.cos(this.p.mount * Math.PI / 180)];
          this.emit('ZERO OK'); this.printCfg(); this.printCal(); return;
        case 'CAL':
          if (tok[1] === 'CLEAR') {
            this.cfg.cal = 0; this.calv.gb = [0, 0, 0]; this.printCfg(); this.printCal(); return;
          }
          if (tok.length > 1 && tok[1] !== 'FLAT') return this.emit('ERR UNKNOWN_COMMAND');
          if (busy || this.uOut) return this.emit('ERR BUSY');
          this.cal = { n: 0, target: 1000, last: 0, flat: tok[1] === 'FLAT' };
          this.emit(`CAL START mode=${this.cal.flat ? 'FLAT' : 'STILL'} n=1000`); return;
        case 'ACCOFF':
          if (busy) return this.emit('ERR BUSY');
          if (tok[1] === 'DEFAULT') { this.calv.ao = this.defaultAo.slice(); return this.printCal(); }
          if (tok.length !== 4 || ints.some((x) => !(Math.abs(x) <= 500))) return bad();
          this.calv.ao = ints.map((x) => x / 1000); return this.printCal();
        case 'FACTORY':
          this.cfg = Object.assign(this.cfg, { axis: 'X', sign: '+', beta: 20, dlpf: 4, cal: 0, zero: 0 });
          this.calv = { gb: [0, 0, 0], ao: this.defaultAo.slice(), g0: [0, 0, 1] }; this.printCfg(); this.printCal(); return;
        case 'RUN': return this.runCmd(tok[1], ints.slice(1), busy);
        default: this.emit('ERR UNKNOWN_COMMAND');
      }
    }

    ms2n(ms) { return Math.max(0, Math.round(ms / (5 * this.cfg.div))); }

    runCmd(type, v, busy) {
      if (busy) return this.emit('ERR BUSY');
      const bad = () => this.emit('ERR BAD_ARGS');
      let p;
      if (type === 'STEP' && v.length === 5) {
        p = { type, u0: v[0], u1: v[1], a: this.ms2n(v[2]), b: this.ms2n(v[3]), c: this.ms2n(v[4]) };
        p.total = p.a + p.b + p.c;
      } else if (type === 'STAIR' && v.length === 4 && v[2] > 0) {
        const steps = Math.floor(Math.abs(v[1] - v[0]) / v[2]) + 1;
        if (steps > 250) return this.emit('ERR TOO_MANY_STEPS');
        p = { type, u0: v[0], inc: v[1] >= v[0] ? v[2] : -v[2], a: Math.max(1, this.ms2n(v[3])) };
        p.total = p.a * steps;
      } else if (type === 'CHIRP' && v.length === 7 && v[2] > 0 && v[3] > 0) {
        p = { type, u0: v[0], amp: v[1], a: this.ms2n(v[4]), b: this.ms2n(v[5]), log: !!v[6], ph: 0 };
        p.total = p.a + p.b;
        const f0 = v[2] / 1000, f1 = v[3] / 1000;
        p.f = f0; p.df = (f1 - f0) / p.b; p.kf = Math.pow(f1 / f0, 1 / p.b);
      } else if (type === 'PRBS' && v.length === 5 && v[2] > 0) {
        p = { type, u0: v[0], amp: v[1], a: this.ms2n(v[3]), b: Math.max(1, this.ms2n(v[2])), lfsr: 0x1ff, level: 1 };
        p.total = p.a + this.ms2n(v[4]);
      } else if (type === 'CLSTEP' && v.length === 5 && v[3] > 0) {
        p = { type, r0: v[0] / 100, r1: v[1] / 100, a: this.ms2n(v[2]), b: this.ms2n(v[3]), c: this.ms2n(v[4]) };
        p.total = p.a + p.b + p.c;
      } else if (type === 'RELAY' && v.length === 5 && v[1] > 0 && v[4] > 0) {
        p = { type, r: v[0] / 100, d: v[1], h: v[2] / 100, a: this.ms2n(v[3]), sum: 0, cnt: 0, center: 0, level: 1 };
        p.total = p.a + this.ms2n(v[4]);
      } else return type && /^(STEP|STAIR|CHIRP|PRBS|CLSTEP|RELAY)$/.test(type) ? bad() : this.emit('ERR UNKNOWN_PROFILE');
      if (!p.total || p.total > 120000) return this.emit('ERR BAD_DURATION');
      p.n = 0; this.prof = p; this.uMan = 0; this.clHold = false; this.pidReset();
      this.emit(`EVT RUN ${type} total=${p.total} ts_ms=${5 * this.cfg.div}`);
    }

    profStep() {
      const p = this.prof, n = p.n;
      let u = 0;
      if (p.type === 'STEP') { if (n < p.a) { u = p.u0; this.phase = 1; } else if (n < p.a + p.b) { u = p.u1; this.phase = 2; } else { u = p.u0; this.phase = 3; } }
      else if (p.type === 'STAIR') { const i = Math.floor(n / p.a); u = p.u0 + i * p.inc; this.phase = Math.min(255, i + 1); }
      else if (p.type === 'CHIRP') {
        if (n < p.a) { u = p.u0; this.phase = 1; }
        else { u = p.u0 + Math.round(p.amp * Math.sin(p.ph)); p.ph = (p.ph + 2 * Math.PI * p.f * this.ts()) % (2 * Math.PI); if (p.log) p.f *= p.kf; else p.f += p.df; this.phase = 2; }
      } else if (p.type === 'PRBS') {
        if (n < p.a) { u = p.u0; this.phase = 1; }
        else {
          if ((n - p.a) % p.b === 0) { const bit = ((p.lfsr >> 8) ^ (p.lfsr >> 4)) & 1; p.lfsr = ((p.lfsr << 1) | bit) & 0x1ff; p.level = bit ? 1 : -1; }
          u = p.u0 + p.level * p.amp; this.phase = 2;
        }
      } else if (p.type === 'CLSTEP') {
        if (n < p.a) { this.ref = p.r0; this.phase = 1; } else if (n < p.a + p.b) { this.ref = p.r1; this.phase = 2; } else { this.ref = p.r0; this.phase = 3; }
        u = this.pidStep(this.ref);
      } else if (p.type === 'RELAY') {
        this.ref = p.r;
        if (n < p.a) { u = this.pidStep(this.ref); this.phase = 1; if (n >= Math.floor(p.a / 2)) { p.sum += u; p.cnt++; } }
        else {
          if (n === p.a) { p.center = p.cnt ? p.sum / p.cnt : this.pid.ff; p.level = 1; }
          const e = this.ref - this.theta;
          if (e > p.h) p.level = 1; else if (e < -p.h) p.level = -1;
          u = Math.round(p.center + p.level * p.d); this.phase = 2;
        }
      }
      p.n++;
      return u;
    }

    stopAll() { this.prof = null; this.clHold = false; this.pidI = 0; this.uMan = 0; this.phase = 0; this.uOut = 0; }

    clampU(u) {
      u = Math.max(-this.cfg.umax, Math.min(this.cfg.umax, u));
      if (u < 0 && !this.cfg.rev) u = 0;
      return u;
    }

    physics(dt) {   // 1 ms 단위: 지연 → 1차 지연(정적 특성 목표) → 공진
      const { T, wr, zr, gain } = this.p;
      this.ubuf.push(this.cfg.bench ? 0 : this.uOut);
      const ud = this.ubuf.shift();
      this.x1 += (gain * thetaSS(ud) - this.x1) * dt / T;
      this.yd += (wr * wr * (this.x1 - this.y) - 2 * zr * wr * this.yd) * dt;
      this.y += this.yd * dt;
    }

    run() {
      if (this.closed) return;
      const target = (performance.now() - this.t0) / 1000;
      let guard = 0;
      while (this.simT + 0.005 <= target && guard++ < 400) this.tick();
      if (guard >= 400) this.simT = target;
    }

    tick() {
      for (let i = 0; i < 5; i++) this.physics(0.001);
      this.simT += 0.005;
      this.tickCount = (this.tickCount || 0) + 1;
      if (this.cal) {
        this.cal.n++;
        const pct = Math.floor(this.cal.n * 100 / this.cal.target);
        if (Math.floor(pct / 10) !== Math.floor(this.cal.last / 10)) { this.cal.last = pct; this.emit(`CAL PROG ${pct}`); }
        if (this.cal.n >= this.cal.target) {
          const flat = this.cal.flat;
          this.cal = null; this.cfg.cal = 1;
          this.calv.gb = this.trueBias.map((b) => b + this.gauss() * 0.01);
          if (flat) this.calv.ao = this.trueAccOff.map((b) => b + this.gauss() * 0.001);
          this.emit(`CAL OK mode=${flat ? 'FLAT' : 'STILL'} gstd=0.041 astd=0.0028 anorm=1.002`); this.printCal();
        }
      }
      if (this.tickCount % this.cfg.div !== 0) return;
      // 샘플
      if ((this.uOut || this.prof || this.clHold) && this.simT - this.lastHb > 1.0) { this.stopAll(); this.emit('EVT ABORT LINK_TIMEOUT'); }
      const sgn = this.cfg.sign === '+' ? 1 : -1;
      const mount = this.cfg.zero ? 0 : this.p.mount;
      const thDeg = sgn * (this.y + mount);
      const running = Math.abs(this.uOut) > 0 && !this.cfg.bench;
      this.theta = thDeg + this.gauss() * this.p.noise;
      this.omega = sgn * this.yd + this.gauss() * 0.5 + (this.cfg.cal ? 0 : sgn * this.trueBias[0]) +
        (running ? 25 * Math.SQRT2 * Math.sin(2 * Math.PI * 46.3 * this.simT) : 0);   // 프로펠러 진동 (실측 σ≈25 °/s)
      const thetaAcc = thDeg + this.gauss() * (running ? 20 : 0.3);
      let u;
      if (this.prof) {
        if (this.prof.n >= this.prof.total) { this.stopAll(); this.emit('EVT DONE'); u = 0; }
        else u = this.profStep();
      } else if (this.clHold) { u = this.pidStep(this.ref); this.phase = 0; }
      else { u = this.uMan; this.phase = 0; }
      u = this.clampU(u);
      if (u !== 0 && Math.abs(this.theta) * 100 > this.cfg.alim) { this.stopAll(); this.emit('EVT ABORT ANGLE_LIMIT'); u = 0; }
      this.uOut = u;
      const cl = this.clHold || (this.prof && (this.prof.type === 'CLSTEP' || this.prof.type === 'RELAY'));
      this.emit(`D,${this.nowMs()},${u},${Math.round(this.theta * 100)},${Math.round(this.omega * 100)},${Math.round(thetaAcc * 100)},${this.phase}` + (cl ? `,${Math.round(this.ref * 100)}` : ''));
      if (++this.auxCnt >= 10) {
        this.auxCnt = 0;
        const ao = this.trueAccOff, tb = this.trueBias;
        const a = [ao[0] + this.gauss() * 0.003, ao[1] + this.gauss() * 0.003, 1 + ao[2] + this.gauss() * 0.003];
        const g = [tb[0] + this.yd + this.gauss() * 0.05, tb[1] + this.gauss() * 0.05, tb[2] + this.gauss() * 0.05];
        this.emit(`A,${a.map((x) => Math.round(x * 1000)).join(',')},${g.map((x) => Math.round(x * 100)).join(',')}`);
      }
      if (this.simT - this.statusT >= 1) {
        this.statusT = this.simT;
        const state = this.cal ? 3 : this.prof ? 2 : this.clHold ? 4 : this.uOut ? 1 : 0;
        const gate = running ? Math.round(10 + 30 * Math.abs(this.uOut) / 255) : 0;
        this.emit(`S,${state},1,${this.cfg.cal},${this.cfg.zero},2100,0,0,${gate},0`);
      }
    }

    close() { this.closed = true; clearInterval(this.timer); }
  }

  root.SimDevice = SimDevice;
})(window);
