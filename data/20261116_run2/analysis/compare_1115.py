import numpy as np, pandas as pd, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from scipy import signal
plt.rcParams['font.family'] = ['Malgun Gothic', 'DejaVu Sans']; plt.rcParams['axes.unicode_minus'] = False
OLD, NEW = '#2a78d6', '#eb6834'; INK2 = '#52514e'
def stairs(f):
    d = pd.read_csv(f, comment='#'); u = d.pwm.values; th = d.theta_deg.values; w = d.omega_dps.values
    segs = np.split(np.arange(len(u)), np.where(np.diff(u) != 0)[0] + 1); R = []
    for s in segs:
        if len(s) < 300: continue
        h = s[len(s)//2:]; x = w[h] - w[h].mean()
        fq, P = signal.welch(x, fs=100, nperseg=256); b = (fq > 1.5) & (fq < 4)
        R.append((u[h][0], th[h].mean(), np.sqrt(np.trapezoid(P[b], fq[b]))))
    return np.array(R)
def relay(f):
    d = pd.read_csv(f, comment='#'); i2 = np.where(d.phase.values == 2)[0][0]
    t = d.t_s.values[i2:] - d.t_s.values[i2]; e = d.theta_deg.values[i2:] - d.ref_deg.values[i2:]
    return t, e, d.pwm.values[i2:]
o = stairs('data/20261115_run1/aero_STAIR_20261115_124525.csv'); n = stairs('data/20261116_run2/aero_STAIR_20261116_213908.csv')
fig, ax = plt.subplots(2, 2, figsize=(12, 7.5))
a = ax[0, 0]
a.plot(o[:, 0], o[:, 1], '-o', color=OLD, lw=2, ms=4, label='11-15 (옛 모터)')
a.plot(n[:, 0], n[:, 1], '-o', color=NEW, lw=2, ms=4, label='11-16 (새 모터)')
for deg in (20, 25, 30): a.axhline(deg, color=INK2, lw=.6, ls=':')
a.set_xlabel('PWM'); a.set_ylabel('정상 각도 θ [°]'); a.set_title('정적 특성 (STAIR, 계단 후반 50% 평균)', fontsize=10)
a.grid(alpha=.25); a.legend(fontsize=8, loc='upper left')
a.annotate('PWM 240–250: θ−θacc 1.9–3.2°\n(이 구간 θ 신뢰 낮음)', (247, 43.5), (195, 14), fontsize=7.5, color=INK2, va='center', arrowprops=dict(arrowstyle='->', color=INK2, lw=.7))
a = ax[0, 1]; wdt = 4
a.bar(o[:, 0] - wdt/2 - .5, o[:, 2], wdt, color=OLD, label='11-15 (옛 모터)')
a.bar(n[:, 0] + wdt/2 + .5, n[:, 2], wdt, color=NEW, label='11-16 (새 모터)')
a.set_xlabel('PWM'); a.set_ylabel('ω 1.5–4 Hz 성분 RMS [°/s]'); a.set_title('개루프 공진 대역 흔들림 (계단 후반)', fontsize=10)
a.grid(alpha=.25, axis='y'); a.legend(fontsize=8, loc='upper left')
for k, (f, lab) in enumerate([('data/20261115_run1/aero_RELAY_20261115_130315.csv', '11-15 20°: Tu 0.439 s (2.28 Hz), Ku 4.87'),
                              ('data/20261116_run2/aero_RELAY_20261116_214233.csv', '11-16 20°: Tu 0.390 s (2.56 Hz), Ku 7.56')]):
    t, e, u = relay(f); m = (t >= 2) & (t <= 6)
    ax[1, 0].plot(t[m], e[m], color=(OLD, NEW)[k], lw=1.5, label=lab)
ax[1, 0].axhline(0, color=INK2, lw=.6)
ax[1, 0].set_xlabel('릴레이 시작 후 t [s]'); ax[1, 0].set_ylabel('θ − r [°]'); ax[1, 0].set_title('릴레이 20° (d = 15 PWM, 같은 설정)', fontsize=10)
ax[1, 0].grid(alpha=.25); ax[1, 0].legend(fontsize=8, loc='upper right')
t, e, u = relay('data/20261116_run2/aero_RELAY_20261116_214303.csv'); m = (t >= 2) & (t <= 6)
ax[1, 1].plot(t[m], e[m], color=NEW, lw=1.5, label='11-16 30°: Tu 0.420 s (2.38 Hz), Ku 4.85')
ax[1, 1].axhline(0, color=INK2, lw=.6); [x.set_ylim(-5.2, 5.2) for x in ax[1]]
ax[1, 1].set_xlabel('릴레이 시작 후 t [s]'); ax[1, 1].set_ylabel('θ − r [°]'); ax[1, 1].set_title('릴레이 30° (새 모터만; 11-15에는 30° 릴레이 없음)', fontsize=10)
ax[1, 1].grid(alpha=.25); ax[1, 1].legend(fontsize=8, loc='upper right')
fig.suptitle('모터 교체 전후 비교: 11-15 run1 vs 11-16 run2', fontsize=12)
fig.tight_layout(); fig.savefig('data/20261116_run2/analysis/compare_1115.png', dpi=90)
for r1 in o: print('old %3d %5.1f %5.1f' % tuple(r1))
for r1 in n: print('new %3d %5.1f %5.1f' % tuple(r1))
