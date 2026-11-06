"""스윕 사인(CHIRP) 기록들로 경험적 주파수응답(ETFE)을 구하고 후보 모델과 비교.
같은 조건의 기록이 여러 개면 응답을 평균해 잡음을 줄이고, 기록 간 차이로 대역별 불확실성을 표시.
사용: python analysis/aero_freqresp.py data/20261106_run2 [K wn zeta ...]  (후보 모델은 3개씩)
"""
import glob, os, sys
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['Malgun Gothic', 'DejaVu Sans']; plt.rcParams['axes.unicode_minus'] = False
sys.path.insert(0, os.path.dirname(__file__))
import aero_analyze as A

d = sys.argv[1]
cands = [tuple(map(float, sys.argv[i:i + 3])) for i in range(2, len(sys.argv) - 2, 3)]
recs = [A.load(f) for f in sorted(glob.glob(os.path.join(d, 'aero_CHIRP_*.csv')))]
edges = np.geomspace(0.09, 3.2, 17)
H = []
for r in recs:
    m = r['ph'] == 2
    u = r['u'][m] - r['u'][m].mean(); y = r['th'][m] - r['th'][m].mean()
    U, Y = np.fft.rfft(u), np.fft.rfft(y); f = np.fft.rfftfreq(len(u), A.Ts)
    h = []
    for a, b in zip(edges[:-1], edges[1:]):
        s = (f >= a) & (f < b)
        h.append((Y[s] * np.conj(U[s])).sum() / (np.abs(U[s]) ** 2).sum())
    H.append(np.array(h))
H = np.array(H); fc = np.sqrt(edges[:-1] * edges[1:])
Hm = H.mean(0); spread = np.abs(H - Hm).max(0) if len(H) > 1 else np.zeros(len(fc))
print(' f[Hz]   |G| °/PWM   위상[°]   기록 간 차이(|ΔG|/|G|)')
for fi, h, sp in zip(fc, Hm, spread):
    print('%6.3f   %8.4f   %7.1f   %5.0f%%' % (fi, abs(h), np.angle(h, deg=True), 100 * sp / abs(h)))
w = 2 * np.pi * fc
fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
ax[0].errorbar(fc, abs(Hm), spread, fmt='o', capsize=3, label='측정 (CHIRP %d회 평균)' % len(H))
ax[1].plot(fc, np.unwrap(np.angle(Hm)) * 180 / np.pi, 'o', label='측정')
ff = np.geomspace(0.05, 5, 300); ww = 2 * np.pi * ff
for c in cands:
    if len(c) == 3:
        K, wn, z = c
        G = K * wn ** 2 / ((1j * ww) ** 2 + 2 * z * wn * 1j * ww + wn ** 2)
        Gm = K * wn ** 2 / ((1j * w) ** 2 + 2 * z * wn * 1j * w + wn ** 2)
        lab = 'K=%.3f ωn=%.2f ζ=%.2f' % c
    ax[0].loglog(ff, abs(G), label=lab); ax[1].semilogx(ff, np.angle(G, deg=True), label=lab)
    err = np.sqrt(np.mean(np.abs(Gm - Hm) ** 2) / np.mean(np.abs(Hm) ** 2))
    print('후보 %s: 주파수응답 상대오차 %.0f%%' % (lab, 100 * err))
ax[0].set_ylabel('|Θ/PWM| [°/PWM]'); ax[1].set_ylabel('위상 [°]'); ax[1].set_xlabel('주파수 [Hz]')
for a in ax: a.grid(alpha=.3, which='both'); a.legend(fontsize=8)
od = os.path.join(d, 'analysis'); os.makedirs(od, exist_ok=True)
fig.tight_layout(); fig.savefig(os.path.join(od, 'freqresp.png'), dpi=90)
