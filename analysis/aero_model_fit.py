"""주파수영역 모델 맞춤 + 다른 실험으로 시간영역 검증 (별도 검증 데이터 없이 실험 간 교차검증)

모델:  G(s) = K e^(-Ls) / (T s + 1) · ωr² / (s² + 2 ζr ωr s + ωr²)     [deg/PWM]
       (저주파: 강하게 감쇠된 진자 응답 ≈ 1차, 3–4 Hz: 약하게 감쇠된 기계 공진)

사용:
  python analysis/aero_model_fit.py --est data/20261106_run3/aero_CHIRP_20261106_143817.csv data/20261106_run3/aero_CHIRP_20261106_143940.csv
                                    --val data/20261106_run3/aero_CHIRP_20261106_1434*.csv data/20261106_run2/aero_STEP_*.csv ...
  추정용 CHIRP 들의 주파수응답(평균)에 맞추고, --val 기록 각각에 시뮬레이션해 적합도를 보고.
  CHIRP 검증은 가진 구간의 평균을 뺀 뒤 비교(운전점 오프셋·드리프트 영향 제거), STEP 은 pre 운전점 기준.
결과: 첫 추정 파일 폴더의 analysis/ 아래 model_fit.png, model_fit.txt
"""
import glob, os, sys
import numpy as np
from scipy import optimize, signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['Malgun Gothic', 'DejaVu Sans']; plt.rcParams['axes.unicode_minus'] = False
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aero_analyze as A
Ts = A.Ts


def etfe(recs, edges):
    Hs = []
    for r in recs:
        m = r['ph'] == 2
        u = r['u'][m] - r['u'][m].mean(); y = r['th'][m] - r['th'][m].mean()
        U, Y = np.fft.rfft(u), np.fft.rfft(y); f = np.fft.rfftfreq(len(u), Ts)
        h = []
        for a, b in zip(edges[:-1], edges[1:]):
            s = (f >= a) & (f < b)
            pu = (np.abs(U[s]) ** 2).sum()
            h.append((Y[s] * np.conj(U[s])).sum() / pu if pu > 1e-6 * (np.abs(U) ** 2).sum() / len(edges) else np.nan)
        Hs.append(h)
    return np.array(Hs)


def tf(p):
    K, T, L, wr, zr = p
    num = [K * wr ** 2]
    den = np.polymul([T, 1], [1, 2 * zr * wr, wr ** 2])
    return num, den, L


def freq(p, w):
    num, den, L = tf(p); s = 1j * w
    return np.polyval(num, s) / np.polyval(den, s) * np.exp(-s * L)


def sim(p, u):
    num, den, L = tf(p)
    bd, ad, _ = signal.cont2discrete((num, den), Ts, 'zoh'); bd = np.ravel(bd)
    nd = int(round(L / Ts))
    return signal.lfilter(bd, ad, np.concatenate([np.zeros(nd), u[:len(u) - nd]]) if nd else u)


def val_segment(r):
    """시간영역 검증 구간: 운전점 = pre 후반 평균, 가진 직전 3 s 부터. CHIRP 은 비교 시 가진 구간 평균 제거."""
    ph = r['ph']; pre = np.where(ph == 1)[0]
    s0, preE = pre[len(pre) // 2], pre[-1] + 1
    end = np.where(ph >= 1)[0][-1] + 1; start = max(s0, preE - 300)
    u0, y0 = r['u'][s0:preE].mean(), r['th'][s0:preE].mean()
    lp = signal.butter(2, 8.0, fs=1 / Ts)
    return dict(t=r['t'][start:end], u=r['u'][start:end] - u0, y=signal.filtfilt(*lp, r['th'][start:end] - y0),
                exc=r['ph'][start:end] >= 2, chirp=r['type'] in ('CHIRP', 'PRBS'))


def score(p, s):
    yh = sim(p, s['u']); y = s['y'].copy()
    if s['chirp']:
        m = s['exc']; y, yh = y[m] - y[m].mean(), yh[m] - yh[m].mean()
    return A.fitpct(y, yh), yh


def main(est_files, val_files, fmax):
    est = [A.load(f) for f in est_files]
    edges = np.geomspace(0.09, fmax, 30); fc = np.sqrt(edges[:-1] * edges[1:]); w = 2 * np.pi * fc
    H = etfe(est, edges); Hm = np.nanmean(H, 0); ok = np.isfinite(Hm)
    def res(p):
        e = (freq(p, w[ok]) - Hm[ok]) / np.abs(Hm[ok])
        return np.concatenate([e.real, e.imag])
    lo, hi = [0, 0.01, 0, 2 * np.pi * 1.5, 0.02], [1, 5, 0.2, 2 * np.pi * 8, 1.0]
    best = None
    for T0 in (0.2, 0.35, 0.6):
        for f0 in (3.0, 3.5, 4.0):
            for z0 in (0.1, 0.25):
                r = optimize.least_squares(res, [0.15, T0, 0.02, 2 * np.pi * f0, z0], bounds=(lo, hi))
                if best is None or r.cost < best.cost: best = r
    p = best.x; K, T, L, wr, zr = p
    ferr = np.sqrt(np.mean(np.abs((freq(p, w[ok]) - Hm[ok]) / Hm[ok]) ** 2))
    lines = ['추정 데이터: ' + ', '.join(os.path.basename(f) for f in est_files),
             'G(s) = %.4f e^(-%.3fs) / (%.3f s + 1) · %.1f² / (s² + %.2f s + %.1f²)' % (K, L, T, wr, 2 * zr * wr, wr),
             '  K=%.4f °/PWM, T=%.3f s, L=%.0f ms, 공진 fr=%.2f Hz (ωr=%.1f rad/s), ζr=%.3f, 주파수응답 상대오차 %.0f%%'
             % (K, T, L * 1000, wr / 2 / np.pi, wr, zr, 100 * ferr)]
    # 위상 -180° 교차 주파수와 그때 이득 (P 제어 이득 여유 참고)
    ff = np.geomspace(0.05, 10, 4000); Gf = freq(p, 2 * np.pi * ff); ph = np.unwrap(np.angle(Gf))
    i180 = np.where(ph <= -np.pi)[0]
    if len(i180):
        i = i180[0]; lines.append('  위상 −180° 교차: %.2f Hz, 그때 |G| = %.4f °/PWM → 순수 P 제어 이득 한계 ≈ %.1f PWM/°' % (ff[i], abs(Gf[i]), 1 / abs(Gf[i])))
    lines.append('\n시간영역 검증 (CHIRP 은 가진 구간 평균 제거 후 비교):')
    vals = [A.load(f) for f in val_files]
    fig = plt.figure(figsize=(13, 3.0 * (len(vals) + 3) / 2 + 4)); gs = fig.add_gridspec((len(vals) + 1) // 2 + 2, 2)
    a1 = fig.add_subplot(gs[0, :]); a2 = fig.add_subplot(gs[1, :], sharex=a1)
    for h, f in zip(H, est_files):
        a1.loglog(fc, np.abs(h), 'o', ms=4, alpha=.6, label=os.path.basename(f)[5:-4])
        a2.semilogx(fc, np.unwrap(np.nan_to_num(np.angle(h))) * 180 / np.pi, 'o', ms=4, alpha=.6)
    a1.loglog(ff, np.abs(Gf), 'k-', label='모델'); a2.semilogx(ff, ph * 180 / np.pi, 'k-')
    a1.set_ylabel('|G| °/PWM'); a2.set_ylabel('위상 °'); a2.set_xlabel('Hz'); a1.legend(fontsize=7); a1.grid(alpha=.3, which='both'); a2.grid(alpha=.3, which='both')
    a1.set_title(lines[1], fontsize=9)
    for i, r in enumerate(vals):
        s = val_segment(r); fitp, yh = score(p, s)
        tag = '(추정)' if any(os.path.samefile(r_, f_) for r_ in [os.path.join(os.path.dirname(est_files[0]), r['name'])] if os.path.exists(r_) for f_ in est_files) else ''
        lines.append('  %-16s %-30s 적합도 %5.1f%% %s' % (r['short'], os.path.dirname(val_files[i]), fitp, tag))
        ax = fig.add_subplot(gs[2 + i // 2, i % 2])
        y = s['y']; yp = sim(p, s['u'])
        if s['chirp']:
            m = s['exc']; y = np.where(m, y - y[m].mean(), np.nan); yp = np.where(m, yp - yp[m].mean(), np.nan)
        ax.plot(s['t'], y, lw=.8, label='측정'); ax.plot(s['t'], yp, lw=1, label='모델')
        ax.set_title('%s  적합 %.0f%% %s' % (r['short'], fitp, tag), fontsize=9); ax.grid(alpha=.3); ax.legend(fontsize=7)
    od = os.path.join(os.path.dirname(est_files[0]), 'analysis'); os.makedirs(od, exist_ok=True)
    fig.tight_layout(); fig.savefig(os.path.join(od, 'model_fit.png'), dpi=80)
    with open(os.path.join(od, 'model_fit.txt'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    a = sys.argv[1:]
    def take(flag):
        if flag not in a: return []
        i = a.index(flag) + 1; out = []
        while i < len(a) and not a[i].startswith('--'):
            out += sorted(glob.glob(a[i])) or [a[i]]; i += 1
        return out
    fmax = float(take('--fmax')[0]) if '--fmax' in a else 7.0
    e = take('--est'); v = take('--val') or e
    main(e, v, fmax)
