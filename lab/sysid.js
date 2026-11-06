// 시스템 식별 보조 계산 (브라우저 + Node 테스트 공용)
// - 2차 이산 모델 y(k) = -a1 y(k-1) - a2 y(k-2) + b1 u(k-nk) + b2 u(k-nk-1)
//   ARX 최소제곱으로 시작해 Steiglitz–McBride 반복으로 출력오차(OE) 형태에 가깝게 보정.
// - 정식 추정은 MATLAB tfest를 사용하고, 이 모듈은 실험 현장 빠른 확인용.
(function (root) {
  'use strict';

  function solve(A, b) {           // 가우스 소거 (부분 피벗)
    const n = b.length;
    const M = A.map((row, i) => row.concat([b[i]]));
    for (let c = 0; c < n; c++) {
      let p = c;
      for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-12) return null;
      [M[c], M[p]] = [M[p], M[c]];
      for (let r = 0; r < n; r++) {
        if (r === c) continue;
        const f = M[r][c] / M[c][c];
        for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
      }
    }
    return M.map((row, i) => row[n] / row[i]);
  }

  function filterA(x, a1, a2) {   // x / A(q)
    const y = new Float64Array(x.length);
    for (let k = 0; k < x.length; k++) {
      y[k] = x[k] - a1 * (k >= 1 ? y[k - 1] : 0) - a2 * (k >= 2 ? y[k - 2] : 0);
    }
    return y;
  }

  function lsq(y, u, nk) {
    const N = y.length, start = Math.max(2, nk + 1);
    const A = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    const b = [0, 0, 0, 0];
    for (let k = start; k < N; k++) {
      const phi = [-y[k - 1], -y[k - 2], u[k - nk], u[k - nk - 1]];
      for (let i = 0; i < 4; i++) {
        b[i] += phi[i] * y[k];
        for (let j = 0; j < 4; j++) A[i][j] += phi[i] * phi[j];
      }
    }
    const th = solve(A, b);
    return th && th.every(Number.isFinite) ? { a1: th[0], a2: th[1], b1: th[2], b2: th[3] } : null;
  }

  function stable(m) {            // z^2 + a1 z + a2 의 근이 단위원 내부
    return Math.abs(m.a2) < 1 && Math.abs(m.a1) < 1 + m.a2;
  }

  function simulate(m, u, nk = 1, y0 = 0) {
    const N = u.length, y = new Float64Array(N);
    for (let k = 0; k < N; k++) {
      const y1 = k >= 1 ? y[k - 1] : y0, y2 = k >= 2 ? y[k - 2] : y0;
      const u1 = k - nk >= 0 ? u[k - nk] : 0, u2 = k - nk - 1 >= 0 ? u[k - nk - 1] : 0;
      y[k] = -m.a1 * y1 - m.a2 * y2 + m.b1 * u1 + m.b2 * u2;
    }
    return y;
  }

  function fitPercent(y, yh) {    // MATLAB compare 와 같은 NRMSE 적합도
    let mean = 0;
    for (const v of y) mean += v;
    mean /= y.length || 1;
    let e = 0, d = 0;
    for (let k = 0; k < y.length; k++) { e += (y[k] - yh[k]) ** 2; d += (y[k] - mean) ** 2; }
    return d > 0 ? 100 * (1 - Math.sqrt(e / d)) : NaN;
  }

  function continuous(m, Ts) {
    const disc = m.a1 * m.a1 - 4 * m.a2;
    const K = (m.b1 + m.b2) / (1 + m.a1 + m.a2);
    const out = { K, disc };
    if (disc < 0) {
      const re = -m.a1 / 2, im = Math.sqrt(-disc) / 2;
      const r = Math.hypot(re, im), ang = Math.atan2(im, re);
      const sr = Math.log(r) / Ts, si = ang / Ts;
      out.wn = Math.hypot(sr, si);
      out.zeta = -sr / out.wn;
      out.poles = [[sr, si], [sr, -si]];
    } else {
      const z1 = (-m.a1 + Math.sqrt(disc)) / 2, z2 = (-m.a1 - Math.sqrt(disc)) / 2;
      if (z1 > 0 && z2 > 0) {
        const s1 = Math.log(z1) / Ts, s2 = Math.log(z2) / Ts;
        out.wn = Math.sqrt(s1 * s2);
        out.zeta = -(s1 + s2) / (2 * out.wn);
        out.poles = [[s1, 0], [s2, 0]];
      }
    }
    if (out.wn) out.fn = out.wn / (2 * Math.PI);
    return out;
  }

  // u, y: 운전점을 뺀 편차 신호
  function fitSecondOrder(u, y, Ts, opts = {}) {
    const nk = opts.nk ?? 1, iters = opts.iters ?? 15;
    let m = lsq(y, u, nk);
    if (!m) return null;
    let best = m, bestFit = stable(m) ? fitPercent(y, simulate(m, u, nk)) : -Infinity;
    for (let i = 0; i < iters && stable(m); i++) {
      const next = lsq(filterA(y, m.a1, m.a2), filterA(u, m.a1, m.a2), nk);
      if (!next || !stable(next)) break;
      m = next;
      const f = fitPercent(y, simulate(m, u, nk));
      if (f > bestFit) { best = m; bestFit = f; }
    }
    if (!stable(best)) return null;
    return Object.assign({ nk, Ts, fit: bestFit }, best, continuous(best, Ts));
  }

  // 기록에서 식별 구간 추출: pre(phase 1) 후반을 운전점으로 보고 그 지점부터 사용
  function prepare(rec) {
    const ph = rec.ph, N = ph.length;
    let preS = -1, preE = -1, endE = -1;
    for (let k = 0; k < N; k++) {
      if (ph[k] === 1) { if (preS < 0) preS = k; preE = k + 1; }
      if (ph[k] >= 1) endE = k + 1;
    }
    let s0, s1, fitFrom;
    if (preS >= 0 && preE - preS >= 4) {
      s0 = preS + Math.floor((preE - preS) / 2);   // 운전점 = pre 후반 평균
      s1 = preE;
      // 식별 구간은 가진 직전 약 1초부터 (pre 초반의 상승 과도응답 제외)
      const oneSec = rec.ts ? Math.round(1 / rec.ts) : 100;
      fitFrom = Math.max(s0, preE - oneSec);
    } else {                    // pre 구간이 없으면 앞 10%를 운전점으로
      s0 = 0; s1 = Math.max(2, Math.floor(N * 0.1));
      fitFrom = 0;
    }
    if (endE < 0) endE = N;
    let u0 = 0, y0 = 0;
    for (let k = s0; k < s1; k++) { u0 += rec.u[k]; y0 += rec.th[k]; }
    u0 /= (s1 - s0); y0 /= (s1 - s0);
    const u = [], y = [];
    for (let k = fitFrom; k < endE; k++) { u.push(rec.u[k] - u0); y.push(rec.th[k] - y0); }
    return { u, y, u0, y0, start: fitFrom, end: endE };
  }

  // 정적 특성: 계단 구간마다 앞부분(settle)을 버리고 평균
  function staticMap(rec, settleFrac = 0.5) {
    const groups = new Map();
    for (let k = 0; k < rec.ph.length; k++) {
      const p = rec.ph[k];
      if (p < 1) continue;
      if (!groups.has(p)) groups.set(p, []);
      groups.get(p).push(k);
    }
    const pts = [];
    for (const [p, idx] of [...groups.entries()].sort((a, b) => a[0] - b[0])) {
      const from = Math.floor(idx.length * settleFrac);
      const use = idx.slice(from);
      if (use.length < 3) continue;
      let su = 0, st = 0;
      for (const k of use) { su += rec.u[k]; st += rec.th[k]; }
      const u = su / use.length, th = st / use.length;
      let v = 0;
      for (const k of use) v += (rec.th[k] - th) ** 2;
      pts.push({ step: p, u, theta: th, std: Math.sqrt(v / (use.length - 1)), n: use.length });
    }
    // sinθ ≈ k (u − u_dz), 각도가 2° 이상인 점만 사용
    const fitPts = pts.filter((q) => q.theta > 2 && q.theta < 89);
    let lin = null;
    if (fitPts.length >= 2) {
      let sx = 0, sy = 0, sxx = 0, sxy = 0;
      for (const q of fitPts) {
        const x = q.u, yv = Math.sin(q.theta * Math.PI / 180);
        sx += x; sy += yv; sxx += x * x; sxy += x * yv;
      }
      const n = fitPts.length, den = n * sxx - sx * sx;
      if (Math.abs(den) > 1e-12) {
        const k = (n * sxy - sx * sy) / den, b = (sy - k * sx) / n;
        let ssr = 0, sst = 0;
        const my = sy / n;
        for (const q of fitPts) {
          const yv = Math.sin(q.theta * Math.PI / 180);
          ssr += (yv - (k * q.u + b)) ** 2; sst += (yv - my) ** 2;
        }
        lin = { k, b, udz: -b / k, r2: sst > 0 ? 1 - ssr / sst : NaN, n };
      }
    }
    for (let i = 0; i < pts.length; i++) {     // 국소 기울기 dθ/du (중앙차분)
      const a = pts[Math.max(0, i - 1)], c = pts[Math.min(pts.length - 1, i + 1)];
      pts[i].slope = c.u !== a.u ? (c.theta - a.theta) / (c.u - a.u) : NaN;
    }
    return { pts, lin };
  }

  // 스텝 응답 지표 (STEP 기록, phase 2 구간)
  function stepMetrics(rec, Ts) {
    const p = prepare(rec);
    const idx = [];
    for (let k = p.start; k < p.end; k++) if (rec.ph[k] === 2) idx.push(k);
    if (idx.length < 10) return null;
    const du = rec.u[idx[0]] - p.u0;
    const tail = idx.slice(Math.floor(idx.length * 0.75));
    let yss = 0;
    for (const k of tail) yss += rec.th[k];
    yss = yss / tail.length - p.y0;
    let peak = 0, kPeak = idx[0];
    for (const k of idx) {
      const d = (rec.th[k] - p.y0) * Math.sign(yss || 1);
      if (d > peak) { peak = d; kPeak = k; }
    }
    const Mp = Math.abs(yss) > 1e-6 ? Math.max(0, peak / Math.abs(yss) - 1) : NaN;
    const tp = (kPeak - idx[0]) * Ts;
    let ts = NaN;                       // 2% 정착시간
    for (let i = idx.length - 1; i >= 0; i--) {
      if (Math.abs(rec.th[idx[i]] - p.y0 - yss) > 0.02 * Math.abs(yss)) { ts = (i + 1) * Ts; break; }
    }
    let zeta = NaN, wn = NaN;
    if (Mp > 0.001) {
      const l = Math.log(Mp);
      zeta = -l / Math.sqrt(Math.PI ** 2 + l * l);
      if (tp > 0) wn = Math.PI / (tp * Math.sqrt(1 - zeta * zeta));
    }
    return { du, dy: yss, K: du ? yss / du : NaN, Mp, tp, ts, zeta, wn, u0: p.u0, y0: p.y0 };
  }

  // ---------------------------------------------------------------- 폐루프
  // 식별 모델 (data/20261106_run3): ΔΘ/ΔPWM = K e^(-Ls)/(Ts+1) · ωr²/(s²+2ζr ωr s+ωr²)
  const PLANT = { K: 0.15, T: 0.328, L: 0.016, wr: 21.8, zr: 0.123 };

  function notchCoeffs(n, ts) {
    if (!n || !n.f) return null;
    const w = 2 * Math.PI * n.f, k = w / Math.tan(w * ts / 2), k2 = k * k, w2 = w * w, a0 = k2 + 2 * n.zp * w * k + w2;
    return { b: [(k2 + 2 * n.zz * w * k + w2) / a0, 2 * (w2 - k2) / a0, (k2 - 2 * n.zz * w * k + w2) / a0], a: [2 * (w2 - k2) / a0, (k2 - 2 * n.zp * w * k + w2) / a0] };
  }

  // 펌웨어 pidStep 과 같은 제어기 + 선형 플랜트(운전점 r0 에서 ff 로 평형) 시뮬레이션. 기준 배열 ref[k] 를 따름.
  function simulateClosedLoop(plant, g, ref, ts = 0.01, opts = {}) {
    const P = Object.assign({}, PLANT, plant);
    const dt = 0.001, nsub = Math.round(ts / dt), nd = Math.round(P.L / dt);
    const ubuf = new Array(nd + 1).fill(0);
    const nc = notchCoeffs(g.notch, ts);
    const umin = opts.umin ?? 0, umax = opts.umax ?? 255, r0 = ref[0];
    let x1 = 0, y = 0, yd = 0, I = 0, omF = 0, z1 = 0, z2 = 0;
    const th = new Float64Array(ref.length), u = new Float64Array(ref.length);
    for (let k = 0; k < ref.length; k++) {
      const theta = r0 + y, e = ref[k] - theta;
      omF += ts / (0.0159 + ts) * (yd - omF);
      let v = g.kp * e + I - g.kd * omF;
      if (nc) { const o = nc.b[0] * v + z1; z1 = nc.b[1] * v - nc.a[0] * o + z2; z2 = nc.b[2] * v - nc.a[1] * o; v = o; }
      const uu = g.ff + v, us = Math.min(umax, Math.max(umin, uu));
      if (uu === us || (uu > us && e < 0) || (uu < us && e > 0)) I = Math.max(-255, Math.min(255, I + g.ki * e * ts));
      th[k] = theta; u[k] = us;
      for (let i = 0; i < nsub; i++) {
        ubuf.push(us - g.ff); const ud = ubuf.shift();
        x1 += (P.K * ud - x1) * dt / P.T;
        yd += (P.wr * P.wr * (x1 - y) - 2 * P.zr * P.wr * yd) * dt;
        y += yd * dt;
      }
    }
    return { th, u };
  }

  // 폐루프 스텝 지표 (CLSTEP 기록: phase 1 = r0, phase 2 = r1). 기준 변화량 대비.
  function clStepMetrics(rec) {
    const i2 = []; for (let k = 0; k < rec.ph.length; k++) if (rec.ph[k] === 2) i2.push(k);
    if (i2.length < 20 || !rec.ref) return null;
    const k0 = i2[0], r1 = rec.ref[k0], pre = [];
    for (let k = Math.max(0, k0 - 100); k < k0; k++) pre.push(k);
    const r0 = rec.ref[k0 - 1] ?? r1, d = r1 - r0, ts = rec.ts;
    if (Math.abs(d) < 1e-6) return null;
    // 3 Hz 이동평균 성격의 간단한 평활 (진동 잡음 제거, 5샘플)
    const y = i2.map((k) => { let s = 0, c = 0; for (let j = k - 2; j <= k + 2; j++) if (j >= 0 && j < rec.th.length) { s += rec.th[j]; c++; } return (s / c - r0) / d; });
    const t = i2.map((k, i) => i * ts);
    const i10 = y.findIndex((v) => v >= 0.1), i90 = y.findIndex((v) => v >= 0.9);
    const tail = y.slice(Math.floor(y.length * 0.8));
    const yf = tail.reduce((a, b) => a + b, 0) / tail.length;
    let ymax = -Infinity, iMax = 0; y.forEach((v, i) => { if (v > ymax) { ymax = v; iMax = i; } });
    let tsIdx = 0; for (let i = y.length - 1; i >= 0; i--) if (Math.abs(y[i] - 1) > 0.05) { tsIdx = i + 1; break; }
    const pre0 = pre.length ? pre.reduce((a, k) => a + rec.th[k], 0) / pre.length : r0;
    const uPeak = Math.max(...i2.map((k) => rec.u[k])), uMin = Math.min(...i2.map((k) => rec.u[k]));
    const sat = i2.filter((k) => rec.u[k] >= 254.5 || rec.u[k] <= 0.5).length / i2.length;
    return { r0, r1, d, tr: i10 >= 0 && i90 >= 0 ? (i90 - i10) * ts : NaN, os: Math.max(0, ymax - 1) * 100, tp: t[iMax],
      ts5: tsIdx >= y.length ? NaN : tsIdx * ts, ess: (1 - yf) * d, preErr: r0 - pre0, uPeak, uMin, sat };
  }

  // 릴레이 시험 → Ku, Tu (Åström–Hägglund). 히스테리시스 ε 보정: Ku = 4d / (π √(a² − ε²))
  function relayAnalyze(rec, d, eps = 0) {
    const idx = []; for (let k = 0; k < rec.ph.length; k++) if (rec.ph[k] === 2) idx.push(k);
    if (idx.length < 100) return null;
    const use = idx.slice(Math.floor(idx.length * 0.3));
    const y0 = use.reduce((a, k) => a + rec.th[k], 0) / use.length;
    const y = use.map((k) => { let s = 0, c = 0; for (let j = k - 1; j <= k + 1; j++) { s += rec.th[j]; c++; } return s / c - y0; });
    const up = []; for (let i = 1; i < y.length; i++) if (y[i - 1] < 0 && y[i] >= 0) up.push(i - 1 + (-y[i - 1]) / (y[i] - y[i - 1]));
    if (up.length < 3) return null;
    const periods = []; for (let i = 1; i < up.length; i++) periods.push((up[i] - up[i - 1]) * rec.ts);
    const amps = []; for (let i = 1; i < up.length; i++) { const seg = y.slice(Math.floor(up[i - 1]), Math.ceil(up[i]) + 1); amps.push((Math.max(...seg) - Math.min(...seg)) / 2); }
    const mean = (v) => v.reduce((a, b) => a + b, 0) / v.length, sd = (v) => Math.sqrt(mean(v.map((x) => (x - mean(v)) ** 2)));
    const Tu = mean(periods), a = mean(amps);
    const Ku = 4 * d / (Math.PI * Math.sqrt(Math.max(1e-9, a * a - eps * eps)));
    return { Ku, Tu, a, cycles: periods.length, TuSd: sd(periods), aSd: sd(amps), center: y0 };
  }

  function zieglerNichols(Ku, Tu) {
    return {
      P: { kp: 0.5 * Ku, ki: 0, kd: 0 },
      PI: { kp: 0.45 * Ku, ki: 0.45 * Ku / (Tu / 1.2), kd: 0 },
      PID: { kp: 0.6 * Ku, ki: 0.6 * Ku / (Tu / 2), kd: 0.6 * Ku * Tu / 8 },
    };
  }

  const api = { solve, lsq, simulate, fitPercent, continuous, fitSecondOrder, prepare, staticMap, stepMetrics, stable,
    PLANT, simulateClosedLoop, clStepMetrics, relayAnalyze, zieglerNichols };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.SysId = api;
})(typeof window !== 'undefined' ? window : globalThis);
