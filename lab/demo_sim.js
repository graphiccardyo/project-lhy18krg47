// 시연용 펌웨어(AEROPEND_DEMO 1.0) 프로토콜을 흉내 내는 시뮬레이터 — 보드 없이 시연 UI(demo.html)를 확인·연습할 때.
// 플랜트 = lab/sim.js 와 같은 2026-11-15 재식별 모델 (정적 특성 + 지연 24 ms + 1차 0.324 s + 느린 경로 3 s + 공진 2.35 Hz ζ 0.084).
// PID·노치 식과 RL(관측·정수 망·적분 보조·시그마-델타·정착 후 전환·SAFE 복귀)은 펌웨어와 같은 식.
// 정책 가중치는 demo_policy.js (python -m rl.demo_policy 가 policy.h 와 함께 생성).
// 실제 장치와 다른 점: 추론 누락 없음, 케이블 충격·자이로 표류 없음. RL 정책은 실기에서 학습했으므로 모델 위 성능은 참고용.
(function (root) {
  'use strict';

  const MAP = [[0, 0], [30, 0], [50, 1.6], [60, 1.6], [70, 3.9], [80, 5.4], [90, 7.3], [100, 8.7], [110, 10.8], [120, 13.3],
    [130, 15.4], [140, 17.7], [150, 19.8], [160, 22.6], [170, 25.2], [180, 26.6], [190, 27.9], [200, 29.9], [210, 31.6],
    [220, 32.5], [230, 35.5], [240, 37.5], [250, 38.6], [255, 39.2]];
  function thetaSS(pwm) {
    const u = Math.max(0, Math.min(255, pwm));
    for (let i = 1; i < MAP.length; i++) {
      if (u <= MAP[i][0]) { const [a, ya] = MAP[i - 1], [b, yb] = MAP[i]; return ya + (yb - ya) * (u - a) / (b - a); }
    }
    return MAP[MAP.length - 1][1];
  }
  const TS = 0.01;

  // ---------------------------------------------------------------- 정수 정책 (rl/demo_policy.py 와 같은 계산)
  function policyMu(xq) {
    const P = root.DEMO_POLICY;
    const h1 = new Array(64);
    for (let j = 0; j < 64; j++) {
      let a = P.B1[j];
      for (let i = 0; i < 8; i++) a += P.W1[j * 8 + i] * xq[i];
      h1[j] = Math.max(0, Math.min(32767, Math.floor(a / 2 ** P.S1[j])));
    }
    let acc3 = 0;
    for (let j = 0; j < 64; j++) {
      let a = P.B2[j];
      for (let k = 0; k < 64; k++) if (h1[k]) a += P.W2[j * 64 + k] * h1[k];
      const h = Math.max(0, Math.min(32767, Math.floor(a / 2 ** P.S2[j])));
      acc3 += P.W3[j] * h;
    }
    return Math.fround(Math.fround(acc3) * P.M3 + P.C3);
  }
  const qx = (v) => Math.max(-32767, Math.min(32767, Math.sign(v) * Math.floor(Math.abs(v) * 2048 + 0.5)));

  class DemoSimDevice {
    constructor() {
      this.onLine = null;
      this.closed = false;
      this.p = { T: 0.324, L: 0.024, wr: 2 * Math.PI * 2.35, zr: 0.084, slow: 0.42, Tc: 3.0, noise: 0.04, stallPwm: 30, startPwm: 45, startDelay: 0.15 };
      this.x1 = 0; this.xs = 0; this.y = 0; this.yd = 0; this.ubuf = new Array(24).fill(0);
      this.motorOn = false; this.startT = 0;
      this.seed = 4321;
      this.uMax = 255; this.alim = 10000; this.bench = 0; this.outDiv = 2; this.beta = 10;
      this.pid = { kp: 4.3, ki: 13.3, kd: 0, ff: 150, nfc: 235, nzz: 150, nzp: 700, umin: 40, b: [1, 0, 0], a: [0, 0] };
      this.notch();
      this.pidI = 0; this.omF = 0; this.nz1 = 0; this.nz2 = 0;
      this.hold = false; this.wantRl = false; this.rlOn = false; this.settle = 0; this.refC = 0;
      this.uOut = 0; this.theta = 0; this.omega = 0;
      this.simT = 0; this.lastHb = 0; this.tickN = 0; this.outN = 0; this.statusT = 0;
      this.t0 = performance.now();
      setTimeout(() => { this.emit('READY AEROPEND_DEMO 1.0 (SIM)'); this.emit('IMU READY'); this.printCfg(); this.printPid(); }, 150);
      this.timer = setInterval(() => this.run(), 20);
    }

    emit(l) { if (this.onLine && !this.closed) this.onLine(l); }
    close() { this.closed = true; clearInterval(this.timer); }
    rnd() { this.seed = (this.seed * 16807) % 2147483647; return this.seed / 2147483647 - 0.5; }
    gauss() { let s = 0; for (let i = 0; i < 6; i++) s += this.rnd(); return s * 1.41; }
    f3(x) { return x.toFixed(3); }

    printCfg() {
      this.emit(`CFG axis=Z sign=+ beta=${this.beta} dlpf=5 ts_ms=10 out_ms=${10 * this.outDiv} umax=${this.uMax} alim=${this.alim} bench=${this.bench} cal=1 zero=1 eeprom=1`);
    }
    printPid() {
      const p = this.pid;
      this.emit(`PIDCFG kp=${this.f3(p.kp)} ki=${this.f3(p.ki)} kd=${this.f3(p.kd)} ff=${p.ff} notch=${p.nfc} zz=${p.nzz} zp=${p.nzp} umin=${p.umin}`);
    }
    printMode() { this.emit('MODE ' + (this.wantRl ? 'RL' : 'PID')); }

    notch() {
      const p = this.pid;
      p.b = [1, 0, 0]; p.a = [0, 0];
      if (!p.nfc) return;
      const w = 2 * Math.PI * p.nfc * 0.01, k = w / Math.tan(w * TS / 2), k2 = k * k, w2 = w * w;
      const zz = p.nzz / 1000, zp = p.nzp / 1000, a0 = k2 + 2 * zp * w * k + w2;
      p.b = [(k2 + 2 * zz * w * k + w2) / a0, 2 * (w2 - k2) / a0, (k2 - 2 * zz * w * k + w2) / a0];
      p.a = [p.b[1], (k2 - 2 * zp * w * k + w2) / a0];
    }
    pidReset() { this.pidI = 0; this.omF = this.omega; this.nz1 = this.nz2 = 0; this.notch(); }
    pidStep(r) {
      const p = this.pid, e = r - this.theta;
      this.omF += TS / (0.0159 + TS) * (this.omega - this.omF);
      let v = p.kp * e + this.pidI - p.kd * this.omF;
      if (p.nfc) { const y = p.b[0] * v + this.nz1; this.nz1 = p.b[1] * v - p.a[0] * y + this.nz2; this.nz2 = p.b[2] * v - p.a[1] * y; v = y; }
      const u = p.ff + v, hi = this.uMax, lo = Math.min(p.umin, hi), us = Math.max(lo, Math.min(hi, u));
      if (u === us || (u > us && e < 0) || (u < us && e > 0)) this.pidI = Math.max(-255, Math.min(255, this.pidI + p.ki * e * TS));
      return Math.round(us);
    }

    // ------------------------------------------------ RL (펌웨어 rlStep / nnFinish 와 같은 식, 추론은 즉시)
    clampRl(u) { u = Math.max(40, u); return Math.min(Math.min(255, this.uMax), u); }
    rlStart(u0) {
      Object.assign(this, { rlOn: true, rlFirst: true, rlN: 0, rlU: this.clampRl(u0), rlPend: null, rlResid: 0, rlEi: 0, rlA: 0 });
      this.emit('EVT RL ON');
    }
    rlToPid() { this.rlOn = false; this.settle = 0; this.pidReset(); this.pidI = Math.max(-255, Math.min(255, this.uOut - this.pid.ff)); }
    rlStep(thC, omC) {
      const th = thC * 0.01, om = omC * 0.01;
      if (this.rlN === 0 && !this.rlFirst) {
        if (this.rlPend !== null) this.rlU = this.clampRl(this.rlU + this.rlPend);
        this.rlPend = null;
      }
      const badTh = thC < -1000 || thC > 7000;
      if (badTh || Math.abs(om) > 600) {
        this.emit('EVT FALLBACK ' + (badTh ? 'THETA' : 'OMEGA'));
        this.wantRl = false; this.rlToPid();
        return this.pidStep(this.refC * 0.01);
      }
      const e = this.refC * 0.01 - th;
      this.rlWf = this.rlFirst ? om : this.rlWf + 0.466512 * (om - this.rlWf);
      this.rlEi += 0.00332779 * (e - this.rlEi);
      if (this.rlN === 0) {
        let aS = 0, dth = 0;
        if (!this.rlFirst) {
          const dEff = this.rlU - this.rlPrevU;
          if (dEff === this.rlPrevDu) aS = this.rlPrevA;
          else { const xp = Math.max(-1, Math.min(1, dEff / 10 - this.rlPrevXi)); aS = Math.sign(xp) * Math.sqrt(Math.abs(xp)); }
          dth = Math.max(-5, Math.min(5, th - this.rlPrevTh));
        }
        const o = [e / 10, (th - 25) / 10, this.rlWf / 100, dth, (this.rlU - 125) / 85, aS, this.rlEi / 2, Math.tanh(e / 0.5)];
        const xi = 0.012 * e;
        this.rlPrevXi = xi; this.rlPrevU = this.rlU; this.rlPrevTh = th;
        const a = Math.max(-1, Math.min(1, Math.tanh(policyMu(o.map(qx)))));
        this.rlResid += (a * Math.abs(a) + xi) * 10;
        const du = Math.max(-10, Math.min(10, Math.sign(this.rlResid) * Math.floor(Math.abs(this.rlResid) + 0.5)));
        this.rlResid = Math.max(-1, Math.min(1, this.rlResid - du));
        this.rlA = a; this.rlPrevA = a; this.rlPrevDu = du; this.rlPend = du;
      }
      this.rlFirst = false;
      this.rlN = (this.rlN + 1) % 2;
      return this.rlU;
    }

    stopAll() { this.hold = false; this.rlOn = false; this.settle = 0; this.pidI = 0; this.uOut = 0; }

    write(line) {
      if (this.closed) return;
      this.lastHb = this.simT;
      const m = line.trim();
      const ints = (s) => s.trim().split(/\s+/).map(Number);
      const err = (c) => this.emit('ERR ' + c);
      if (m === 'PING') return;
      if (m === 'HELLO') { this.stopAll(); this.emit('READY AEROPEND_DEMO 1.0 (SIM)'); this.emit('IMU READY'); this.printCfg(); this.printPid(); this.printMode(); return; }
      if (m === 'STOP') { this.stopAll(); this.emit('EVT STOP'); return; }
      if (m === 'GET') { this.printCfg(); this.printPid(); this.printMode(); return; }
      if (m.startsWith('REF ')) {
        const v = ints(m.slice(4));
        if (v.length !== 1 || !Number.isInteger(v[0]) || Math.abs(v[0]) > 9000) return err('BAD_ARGS');
        if (!this.hold) { this.pidReset(); this.hold = true; this.settle = 0; }
        this.refC = v[0]; this.emit('EVT REF ' + v[0]); return;
      }
      if (m === 'MODE RL') { this.wantRl = true; this.settle = 0; this.printMode(); return; }
      if (m === 'MODE PID') { this.wantRl = false; if (this.rlOn) { this.rlToPid(); this.emit('EVT RL OFF'); } this.printMode(); return; }
      if (m.startsWith('PID ')) {
        const v = ints(m.slice(4));
        if (v.length === 4) v.push(this.pid.umin);
        if (v.length !== 5 || v.some((x) => !Number.isInteger(x) || x < 0) || v[0] > 200000 || v[1] > 500000 || v[2] > 50000 || v[3] > 255 || v[4] > 255) return err('BAD_ARGS');
        Object.assign(this.pid, { kp: v[0] / 1000, ki: v[1] / 1000, kd: v[2] / 1000, ff: v[3], umin: v[4] });
        this.printPid(); return;
      }
      if (m.startsWith('NOTCH ')) {
        const v = ints(m.slice(6));
        if (v.length === 1 && v[0] === 0) this.pid.nfc = 0;
        else if (v.length === 3 && v[0] >= 10 && v[0] <= 2000 && v[1] >= 0 && v[1] <= 2000 && v[2] > 0 && v[2] <= 2000) Object.assign(this.pid, { nfc: v[0], nzz: v[1], nzp: v[2] });
        else return err('BAD_ARGS');
        this.nz1 = this.nz2 = 0; this.notch(); this.printPid(); return;
      }
      if (m.startsWith('LIM ')) {
        const v = ints(m.slice(4));
        if (v.length !== 2 || v[0] < 0 || v[0] > 255 || v[1] < 500 || v[1] > 18000) return err('BAD_ARGS');
        this.uMax = v[0]; this.alim = v[1]; this.printCfg(); return;
      }
      if (m.startsWith('OUT ')) {
        const v = ints(m.slice(4));
        if (v.length !== 1 || (v[0] !== 1 && v[0] !== 2)) return err('BAD_ARGS');
        this.outDiv = v[0]; this.outN = 0; this.printCfg(); return;
      }
      if (m.startsWith('BENCH ')) {
        if (this.hold || this.uOut) return err('BUSY');
        this.bench = m.slice(6).trim() === '1' ? 1 : 0; this.printCfg(); return;
      }
      if (m === 'ZERO') { if (this.hold || this.uOut) return err('BUSY'); this.emit('ZERO OK'); return; }
      if (m === 'CAL') { if (this.hold || this.uOut) return err('BUSY'); this.emit('CAL START'); setTimeout(() => this.emit('CAL OK range=0.62'), 5000); return; }
      if (m.startsWith('BETA ')) { this.beta = Number(m.slice(5)) || this.beta; this.printCfg(); return; }
      err('UNKNOWN_COMMAND');
    }

    physics(dt) {
      const P = this.p;
      this.ubuf.push(this.bench ? 0 : this.uOut);
      const ud = this.ubuf.shift();
      if (this.motorOn) { if (ud < P.stallPwm) this.motorOn = false; }
      else if (ud >= P.startPwm) { this.startT += dt; if (this.startT >= P.startDelay) { this.motorOn = true; this.startT = 0; } }
      else this.startT = 0;
      const target = this.motorOn ? thetaSS(ud) : 0;
      this.x1 += ((1 - P.slow) * target - this.x1) * dt / P.T;
      this.xs += (P.slow * target - this.xs) * dt / P.Tc;
      this.yd += (P.wr * P.wr * (this.x1 + this.xs - this.y) - 2 * P.zr * P.wr * this.yd) * dt;
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
      if (++this.tickN % 2) return;
      if ((this.uOut || this.hold) && this.simT - this.lastHb > 1.0) { this.stopAll(); this.emit('EVT ABORT LINK_TIMEOUT'); }
      this.theta = this.y + this.gauss() * this.p.noise;
      this.omega = this.yd + this.gauss() * 0.5;
      const thC = Math.round(this.theta * 100), omC = Math.round(this.omega * 100);
      let u = 0, mode = 0;
      if (this.hold) {
        if (this.rlOn) { u = this.rlStep(thC, omC); mode = this.rlOn ? 3 : 1; }
        else {
          u = this.pidStep(this.refC * 0.01);
          mode = this.wantRl ? 2 : 1;
          if (this.wantRl) {
            if (Math.abs(this.refC * 0.01 - this.theta) < 2) { if (++this.settle >= 100) this.rlStart(u); }
            else this.settle = 0;
          }
        }
      }
      if (u !== 0 && Math.abs(this.theta) * 100 > this.alim) { this.stopAll(); this.emit('EVT ABORT ANGLE_LIMIT'); u = 0; mode = 0; }
      this.uOut = Math.max(0, Math.min(this.uMax, u));
      if (++this.outN >= this.outDiv) {
        this.outN = 0;
        const tha = Math.round((this.y + this.gauss() * (this.uOut && !this.bench ? 0.6 : 0.3)) * 100);
        this.emit(`D,${Math.round(this.simT * 1000)},${this.uOut},${thC},${omC},${tha},${mode},${this.refC},${mode === 3 ? Math.round(this.rlA * 1000) : 0}`);
      }
      if (this.simT - this.statusT >= 1) {
        this.statusT = this.simT;
        this.emit(`S,${this.hold ? 4 : 0},1,1,1,4200,0,0,0,0,${this.rlOn ? 9000 : 0},0,61`);
      }
    }
  }

  root.DemoSimDevice = DemoSimDevice;
  root.demoPolicyMu = policyMu;
})(window);
