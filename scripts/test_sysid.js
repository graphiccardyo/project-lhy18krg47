// node scripts/test_sysid.js — 알려진 2차계로 SysId 추정 정확도 확인
const S = require('../lab/sysid.js');
const Ts = 0.01, wn = 5.5, zeta = 0.08, K = 1.8;
// 연속계 K wn^2/(s^2+2 zeta wn s+wn^2) 를 RK4로 적분해 측정 데이터 생성
function plant(u, noise) {
  let x1 = 0, x2 = 0; const y = [];
  const f = (a, b, uk) => [b, K * wn * wn * uk - 2 * zeta * wn * b - wn * wn * a];
  let seed = 1; const rnd = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647 - 0.5; };
  for (let k = 0; k < u.length; k++) {
    y.push(x1 + noise * rnd() * 3.46);
    for (let s = 0; s < 10; s++) {
      const h = Ts / 10, uk = u[k];
      const k1 = f(x1, x2, uk), k2 = f(x1 + h/2*k1[0], x2 + h/2*k1[1], uk),
            k3 = f(x1 + h/2*k2[0], x2 + h/2*k2[1], uk), k4 = f(x1 + h*k3[0], x2 + h*k3[1], uk);
      x1 += h/6*(k1[0]+2*k2[0]+2*k3[0]+k4[0]); x2 += h/6*(k1[1]+2*k2[1]+2*k3[1]+k4[1]);
    }
  }
  return y;
}
let ok = true;
const check = (name, v, ref, tol) => { const pass = Math.abs(v - ref) <= tol; ok = ok && pass;
  console.log(`${pass ? 'PASS' : 'FAIL'} ${name}=${v.toFixed(4)} (ref ${ref}, tol ${tol})`); };
// chirp 0.1→3 Hz, 60 s
const N = 6000, u = [];
let ph = 0; for (let k = 0; k < N; k++) { const f = 0.1 * Math.pow(30, k / N); ph += 2*Math.PI*f*Ts; u.push(5 * Math.sin(ph)); }
for (const noise of [0, 0.2]) {
  const y = plant(u, noise);
  const m = S.fitSecondOrder(u, y, Ts, { nk: 1 });
  console.log(`noise=${noise}: fit=${m.fit.toFixed(1)}%`);
  check('K', m.K, K, 0.05 * K); check('wn', m.wn, wn, 0.05 * wn); check('zeta', m.zeta, zeta, 0.02);
}
// 스텝 기록 형식 + stepMetrics / prepare
const rec = { u: [], th: [], ph: [] };
const uu = []; for (let k = 0; k < 2000; k++) uu.push(k < 500 ? 40 : k < 1300 ? 45 : 40);
const yy = plant(uu.map(v => v - 40), 0).map(v => v + 30);
for (let k = 0; k < 2000; k++) { rec.u.push(uu[k]); rec.th.push(yy[k]); rec.ph.push(k < 500 ? 1 : k < 1300 ? 2 : 3); }
const sm = S.stepMetrics(rec, Ts);
check('step K', sm.K, K, 0.05); check('step zeta', sm.zeta, zeta, 0.02); check('step wn', sm.wn, wn, 0.3);
// 정적 특성: theta = asin(k (u-udz))
const st = { u: [], th: [], ph: [] };
for (let i = 0; i < 8; i++) { const uv = 20 + 10 * i; const th = Math.asin(Math.min(1, 0.012 * Math.max(0, uv - 15))) * 180 / Math.PI;
  for (let j = 0; j < 100; j++) { st.u.push(uv); st.th.push(th); st.ph.push(i + 1); } }
const map = S.staticMap(st, 0.5);
check('map k', map.lin.k, 0.012, 1e-4); check('map udz', map.lin.udz, 15, 0.1);
process.exit(ok ? 0 : 1);
