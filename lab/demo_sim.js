// 시연용 펌웨어(AEROPEND_DEMO 2.0) 프로토콜을 흉내 내는 시뮬레이터 — 보드 없이 시연 UI(demo.html)를 확인·연습할 때.
// 플랜트 = 11-16 모터 교체 뒤 재식별 (data/20261116_run2, rl/configs/motor2.json 의 가짜 보드와 같은 값):
//   정적 특성(계단) + 지연 23 ms → [빠른 경로 1차 T 0.357 s (K 0.192)] + [느린 경로 Tc 5 s (Ks 0.06)] → 공진 2.57 Hz ζ 0.15
// PID·노치 식, RL(잔차 r = RES·a|a| 를 학습 때 PID 의 노치 앞에 더함, 관측·정수 망·정착 후 전환·SAFE 복귀)은 펌웨어와 같은 식.
// 정책·설정은 demo_policy.js (python -m rl.demo_policy 가 policy.h 와 함께 생성).
// 실제 장치와 다른 점: 추론 누락 없음, 케이블 충격·자이로 표류·운전점 이득 변화 없음. RL 은 실기에서 학습했으므로 모델 위 성능은 참고용.
(function (root) {
  'use strict';

  const MAP = [[0, 0], [30, 0], [40, 2.7], [50, 3.1], [60, 3.8], [70, 4.8], [80, 7.0], [90, 9.0], [100, 10.8], [110, 13.0],
    [120, 15.5], [130, 17.7], [140, 19.7], [150, 21.7], [160, 23.7], [170, 25.8], [180, 27.7], [190, 31.3], [200, 34.5],
    [210, 36.1], [220, 38.5], [230, 40.6], [240, 42.7], [250, 44.9], [255, 46.0]];   // 11-16 run2 계단 (motor2.json)
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
    const P = root.DEMO_POLICY, H = P.H || 64;
    const h1 = new Array(H);
    for (let j = 0; j < H; j++) {
      let a = P.B1[j];
      for (let i = 0; i < 8; i++) a += P.W1[j * 8 + i] * xq[i];
      h1[j] = Math.max(0, Math.min(32767, Math.floor(a / 2 ** P.S1[j])));
    }
    let acc3 = 0;
    for (let j = 0; j < H; j++) {
      let a = P.B2[j];
      for (let k = 0; k < H; k++) if (h1[k]) a += P.W2[j * H + k] * h1[k];
      const h = Math.max(0, Math.min(32767, Math.floor(a / 2 ** P.S2[j])));
      acc3 += P.W3[j] * h;
    }
    return Math.fround(Math.fround(acc3) * P.M3 + P.C3);
  }
  const qx = (v) => Math.max(-32767, Math.min(32767, Math.sign(v) * Math.floor(Math.abs(v) * 2048 + 0.5)));
  const lround = (v) => Math.sign(v) * Math.floor(Math.abs(v) + 0.5);
  const f3 = (x) => x.toFixed(3);

  function notchCoeffs(p) {
    p.b = [1, 0, 0]; p.a = [0, 0];
    if (!p.nfc) return;
    const w = 2 * Math.PI * p.nfc * 0.01, k = w / Math.tan(w * TS / 2), k2 = k * k, w2 = w * w;
    const zz = p.nzz / 1000, zp = p.nzp / 1000, a0 = k2 + 2 * zp * w * k + w2;
    p.b = [(k2 + 2 * zz * w * k + w2) / a0, 2 * (w2 - k2) / a0, (k2 - 2 * zz * w * k + w2) / a0];
    p.a = [p.b[1], (k2 - 2 * zp * w * k + w2) / a0];
  }
  const gainsTxt = (p) => `kp=${f3(p.kp)} ki=${f3(p.ki)} kd=${f3(p.kd)} ff=${p.ff} notch=${p.nfc} zz=${p.nzz} zp=${p.nzp} umin=${p.umin}`;

  class DemoSimDevice {
    constructor() {
      const K = root.DEMO_POLICY.cfg;
      this.K = K;
      this.onLine = null;
      this.closed = false;
      this.p = { T: 0.357, L: 0.023, wr: 2 * Math.PI * 2.57, zr: 0.15, slow: 0.06 / (0.192 + 0.06), Tc: 5.0, noise: 0.04,
        stallPwm: 30, startPwm: 40, startDelay: 0.15 };
      this.x1 = 0; this.xs = 0; this.y = 0; this.yd = 0; this.ubuf = new Array(23).fill(0);
      this.motorOn = false; this.startT = 0;
      this.seed = 4321;
      this.uMax = K.umax; this.alim = 10000; this.bench = 0; this.outDiv = 2; this.beta = 10;
      this.pidU = { kp: 4.3, ki: 13.3, kd: 0, ff: K.pid.ff, nfc: 235, nzz: 150, nzp: 700, umin: 40 };   // 사용자 이득
      this.pidRl = { ...K.pid };                                                                         // RL 기반 (고정)
      notchCoeffs(this.pidU); notchCoeffs(this.pidRl);
      this.P = this.pidU;
      this.pidI = 0; this.omF = 0; this.nz1 = 0; this.nz2 = 0; this.pidAdd = 0;
      this.hold = false; this.wantRl = false; this.rlOn = false; this.settle = 0; this.refC = 0;
      this.uOut = 0; this.theta = 0; this.omega = 0;
      this.simT = 0; this.lastHb = 0; this.tickN = 0; this.outN = 0; this.statusT = 0;
      this.t0 = performance.now();
      setTimeout(() => { this.emit('READY AEROPEND_DEMO 2.0 (SIM)'); this.emit('IMU READY'); this.printCfg(); this.printPid(); this.printRl(); }, 150);
      this.timer = setInterval(() => this.run(), 20);
    }

    emit(l) { if (this.onLine && !this.closed) this.onLine(l); }
    close() { this.closed = true; clearInterval(this.timer); }
    rnd() { this.seed = (this.seed * 16807) % 2147483647; return this.seed / 2147483647 - 0.5; }
    gauss() { let s = 0; for (let i = 0; i < 6; i++) s += this.rnd(); return s * 1.41; }

    printCfg() {
      this.emit(`CFG axis=Z sign=+ beta=${this.beta} dlpf=5 ts_ms=10 out_ms=${10 * this.outDiv} umax=${this.uMax} alim=${this.alim} bench=${this.bench} cal=1 zero=1 eeprom=1`);
    }
    printPid() { this.emit('PIDCFG ' + gainsTxt(this.pidU)); }
    printRl() { const K = this.K; this.emit(`RLPOL ${root.DEMO_POLICY.src} res=${K.res} ulo=${K.u_lo} uhi=${K.u_hi} ${gainsTxt(this.pidRl)}`); }
    printMode() { this.emit('MODE ' + (this.wantRl ? 'RL' : 'PID')); }

    pidReset() { this.pidI = 0; this.omF = this.omega; this.nz1 = this.nz2 = 0; }
    pidStep(r) {
      const p = this.P, e = r - this.theta;
      this.omF += TS / (0.0159 + TS) * (this.omega - this.omF);
      let v = p.kp * e + this.pidI - p.kd * this.omF + this.pidAdd;
      if (p.nfc) { const y = p.b[0] * v + this.nz1; this.nz1 = p.b[1] * v - p.a[0] * y + this.nz2; this.nz2 = p.b[2] * v - p.a[1] * y; v = y; }
      const u = p.ff + v, hi = this.uMax, lo = Math.min(p.umin, hi), us = Math.max(lo, Math.min(hi, u));
      if (u === us || (u > us && e < 0) || (u < us && e > 0)) this.pidI = Math.max(-255, Math.min(255, this.pidI + p.ki * e * TS));
      return lround(us);
    }
    pidUse(to, u0) {                       // 펌웨어 pidUse 와 같음
      const P = this.P;
      if (to === P) return;
      const same = to.kp === P.kp && to.ki === P.ki && to.kd === P.kd && to.nfc === P.nfc && to.nzz === P.nzz && to.nzp === P.nzp;
      const i = same ? this.pidI + P.ff - to.ff : u0 - to.ff;
      if (!same) this.nz1 = this.nz2 = 0;
      this.P = to;
      this.pidI = Math.max(-255, Math.min(255, i));
    }

    // ------------------------------------------------ RL (펌웨어 rlStep / nnFinish 와 같은 식, 추론은 즉시)
    clampRl(u) { u = Math.max(this.K.u_lo, u); return Math.min(Math.min(this.K.u_hi, this.uMax), u); }
    rlStart(u0) {
      Object.assign(this, { rlOn: true, rlFirst: true, rlN: 0, rlPend: null, pidAdd: 0, rlAppA: 0, rlEi: 0 });
      this.pidUse(this.pidRl, u0);
      this.emit('EVT RL ON');
    }
    rlToPid() { this.rlOn = false; this.settle = 0; this.pidAdd = 0; this.pidUse(this.pidU, this.uOut); }
    rlStep(thC, omC) {
      const K = this.K, th = thC * 0.01, om = omC * 0.01;
      if (this.rlN === 0 && !this.rlFirst) {
        if (this.rlPend) { this.pidAdd = this.rlPend.r; this.rlAppA = this.rlPend.a; }
        this.rlPend = null;
      }
      const badTh = thC < K.safe_lo || thC > K.safe_hi;
      if (badTh || Math.abs(om) > K.safe_w) {
        this.emit('EVT FALLBACK ' + (badTh ? 'THETA' : 'OMEGA'));
        this.wantRl = false; this.rlToPid();
        return this.pidStep(this.refC * 0.01);
      }
      const u = this.clampRl(this.pidStep(this.refC * 0.01));
      const e = this.refC * 0.01 - th;
      this.rlWf = this.rlFirst ? om : this.rlWf + K.aw * (om - this.rlWf);
      this.rlEi += K.ai * (e - this.rlEi);
      if (this.rlN === 0) {
        const dth = this.rlFirst ? 0 : Math.max(-5, Math.min(5, th - this.rlPrevTh));
        const o = [e / K.th_s, (th - K.th_c) / K.th_s, this.rlWf / K.w_s, dth, (u - K.u_c) / K.u_s, this.rlAppA, this.rlEi / K.ei_s, Math.tanh(e / K.ef_s)];
        this.rlPrevTh = th;
        const a = Math.max(-1, Math.min(1, Math.tanh(policyMu(o.map(qx)))));
        this.rlPend = { a, r: lround(K.res * a * Math.abs(a)) };
      }
      this.rlFirst = false;
      this.rlN = (this.rlN + 1) % 2;
      return u;
    }

    stopAll() { this.hold = false; this.rlOn = false; this.settle = 0; this.pidI = 0; this.pidAdd = 0; this.P = this.pidU; this.uOut = 0; }

    write(line) {
      if (this.closed) return;
      this.lastHb = this.simT;
      const m = line.trim();
      const ints = (s) => s.trim().split(/\s+/).map(Number);
      const err = (c) => this.emit('ERR ' + c);
      if (m === 'PING') return;
      if (m === 'HELLO') { this.stopAll(); this.emit('READY AEROPEND_DEMO 2.0 (SIM)'); this.emit('IMU READY'); this.printCfg(); this.printPid(); this.printRl(); this.printMode(); return; }
      if (m === 'STOP') { this.stopAll(); this.emit('EVT STOP'); return; }
      if (m === 'GET') { this.printCfg(); this.printPid(); this.printRl(); this.printMode(); return; }
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
        if (v.length === 4) v.push(this.pidU.umin);
        if (v.length !== 5 || v.some((x) => !Number.isInteger(x) || x < 0) || v[0] > 200000 || v[1] > 500000 || v[2] > 50000 || v[3] > 255 || v[4] > 255) return err('BAD_ARGS');
        Object.assign(this.pidU, { kp: v[0] / 1000, ki: v[1] / 1000, kd: v[2] / 1000, ff: v[3], umin: v[4] });
        this.printPid(); return;
      }
      if (m.startsWith('NOTCH ')) {
        const v = ints(m.slice(6));
        if (v.length === 1 && v[0] === 0) this.pidU.nfc = 0;
        else if (v.length === 3 && v[0] >= 10 && v[0] <= 2000 && v[1] >= 0 && v[1] <= 2000 && v[2] > 0 && v[2] <= 2000) Object.assign(this.pidU, { nfc: v[0], nzz: v[1], nzp: v[2] });
        else return err('BAD_ARGS');
        notchCoeffs(this.pidU);
        if (this.P === this.pidU) this.nz1 = this.nz2 = 0;
        this.printPid(); return;
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
        this.emit(`D,${Math.round(this.simT * 1000)},${this.uOut},${thC},${omC},${tha},${mode},${this.refC},${mode === 3 ? this.pidAdd : 0}`);
      }
      if (this.simT - this.statusT >= 1) {
        this.statusT = this.simT;
        this.emit(`S,${this.hold ? 4 : 0},1,1,1,4300,0,0,0,0,${this.rlOn ? 1800 : 0},0,62`);
      }
    }
  }

  root.DemoSimDevice = DemoSimDevice;
  root.demoPolicyMu = policyMu;
})(window);
