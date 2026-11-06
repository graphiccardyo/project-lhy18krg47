"""Aeropendulum PID 설계·검토 (식별 모델 기반)

플랜트 (data/20261106_run3/analysis/REPORT.md, PWM≈150 소신호):
    P(s) = K e^(-Ls) / (T s + 1) · ωr² / (s² + 2ζr ωr s + ωr²)   [deg/PWM]
    K = 0.12–0.21 (공칭 0.15, 세션 중 변동), T = 0.328 s, L = 0.016 s, ωr = 21.8 rad/s (3.47 Hz), ζr = 0.123

제어기 (펌웨어와 동일한 구조, Ts = 10 ms):
    u = ff + Kp·e + Ki·∫e dt − Kd·ω_f      (D 는 자이로 각속도 ω 를 10 Hz 저역통과해 측정값에 적용 → 기준 변화 시 킥 없음)
    선택: 출력 노치 N(s) = (s² + 2ζz ωn s + ωn²)/(s² + 2ζp ωn s + ωn²)  (구조 공진 억제)
    출력 포화 [0, 255] + 적분 조건부 정지(anti-windup)

평가:
    강인성 = 최대 감도 Ms = max|1/(1+L)| (작을수록 강인, 1.4–2.0 권장) 를 K∈{0.12,0.15,0.21}, ωr ±10% 에서 계산
    시간응답 = 기준 20°→25° 스텝을 1 ms 연속 플랜트 + 10 ms 이산 제어기로 시뮬레이션 (자이로 진동 잡음 포함)

사용: python analysis/pid_design.py   → analysis/pid_design/ 아래 summary.txt, *.png
"""
import os
import numpy as np
from scipy import linalg, signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = ['Malgun Gothic', 'DejaVu Sans']; plt.rcParams['axes.unicode_minus'] = False

T, L, WR, ZR, TS = 0.328, 0.016, 21.8, 0.123, 0.01
K_SET = (0.12, 0.15, 0.21)
WR_SET = (0.9 * WR, WR, 1.1 * WR)
K_NOM = 0.15
TAU_D = 1 / (2 * np.pi * 10)          # D 항 저역통과 (10 Hz)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'pid_design')
W = np.geomspace(0.05, 300, 6000)


def plant(w, K=K_NOM, wr=WR):
    s = 1j * w
    return K * np.exp(-s * L) / (T * s + 1) * wr ** 2 / (s ** 2 + 2 * ZR * wr * s + wr ** 2)


def notch(w, n):
    if not n: return 1.0
    s = 1j * w; wn, zz, zp = n
    return (s ** 2 + 2 * zz * wn * s + wn ** 2) / (s ** 2 + 2 * zp * wn * s + wn ** 2)


def ctrl(w, kp, ki, kd, n=None):
    s = 1j * w
    zoh = np.exp(-s * TS / 2)          # 이산 구현(ZOH) 반 샘플 지연
    return (kp + ki / s + kd * s / (TAU_D * s + 1)) * notch(w, n) * zoh


def margins(kp, ki, kd, n=None, K=K_NOM, wr=WR):
    Lw = ctrl(W, kp, ki, kd, n) * plant(W, K, wr)
    Ms = np.max(1 / np.abs(1 + Lw))
    mag = np.abs(Lw); ph = np.unwrap(np.angle(Lw))
    ic = np.where((mag[:-1] >= 1) & (mag[1:] < 1))[0]
    wc = W[ic[-1]] if len(ic) else np.nan
    pm = 180 + np.degrees(ph[ic[-1]]) if len(ic) else np.nan
    gms = [1 / mag[i] for i in range(len(W) - 1) if np.floor((ph[i] + np.pi) / (2 * np.pi)) != np.floor((ph[i + 1] + np.pi) / (2 * np.pi))]
    gm = 20 * np.log10(min(gms)) if gms else np.inf
    # 폐루프 안정성: 나이퀴스트 대신 극점으로 확인 (지연은 Pade 근사)
    stable = closed_loop_stable(kp, ki, kd, n, K, wr)
    return dict(Ms=Ms, wc=wc, pm=pm, gm=gm, stable=stable)


def tf_poly(kp, ki, kd, n, K, wr):
    """폐루프 특성다항식용 개루프 분자/분모 (지연·ZOH 는 2차 Pade)"""
    tau = L + TS / 2
    pn, pd = [tau ** 2 / 12, -tau / 2, 1], [tau ** 2 / 12, tau / 2, 1]   # 2차 Pade 근사
    Pn = np.polymul([K * wr ** 2], pn)
    Pd = np.polymul(np.polymul([T, 1], [1, 2 * ZR * wr, wr ** 2]), pd)
    if ki:
        Cn = np.polyadd(np.polymul([kp, ki], [TAU_D, 1]), [kd, 0, 0])   # (kp s + ki)(τ s + 1) + kd s²
        Cd = np.polymul([1, 0], [TAU_D, 1])
    else:                                                           # 적분 없음: s 공통인수 제거
        Cn = np.polyadd(np.polymul([kp], [TAU_D, 1]), [kd, 0])
        Cd = [TAU_D, 1]
    if n:
        wn, zz, zp = n
        Cn = np.polymul(Cn, [1, 2 * zz * wn, wn ** 2]); Cd = np.polymul(Cd, [1, 2 * zp * wn, wn ** 2])
    return np.polymul(Cn, Pn), np.polymul(Cd, Pd)


def closed_loop_stable(kp, ki, kd, n, K, wr):
    num, den = tf_poly(kp, ki, kd, n, K, wr)
    return bool(np.all(np.real(np.roots(np.polyadd(den, num))) < 0))


def robust(kp, ki, kd, n=None):
    res = [margins(kp, ki, kd, n, K, wr) for K in K_SET for wr in WR_SET]
    return dict(Ms=max(r['Ms'] for r in res), gm=min(r['gm'] for r in res), pm=min(r['pm'] for r in res),
                stable=all(r['stable'] for r in res), nom=margins(kp, ki, kd, n))


# ---------------------------------------------------------------- 시간영역 시뮬레이션
def plant_ss(K, wr):
    num = [K * wr ** 2]; den = np.polymul([T, 1], [1, 2 * ZR * wr, wr ** 2])
    A, B, C, D = signal.tf2ss(num, den)
    return A, B, C


def simulate(kp, ki, kd, n=None, K=K_NOM, wr=WR, r0=20.0, r1=25.0, tstep=1.0, tend=6.0, ff=150.0, noise=True, seed=1):
    """기준 r0→r1 스텝. 플랜트는 운전점 편차 모델(θ0 = r0 에서 ff 로 평형이라고 가정)."""
    A, B, C = plant_ss(K, wr)
    dt = 0.001; Ad = linalg.expm(A * dt); Bd = np.linalg.solve(A, (Ad - np.eye(len(A))) @ B)
    nsub = int(TS / dt); nd = int(round(L / dt))
    x = np.zeros((len(A), 1)); ubuf = [0.0] * (nd + 1)
    N = int(tend / TS); rng = np.random.default_rng(seed)
    if n:
        wn, zz, zp = n
        nb, na = signal.bilinear([1, 2 * zz * wn, wn ** 2], [1, 2 * zp * wn, wn ** 2], fs=1 / TS)
        zi = np.zeros(2)
    I = 0.0; omf = 0.0; ad = TS / (TAU_D + TS)
    tt, th, uu, rr = [], [], [], []
    du = 0.0
    for k in range(N):
        t = k * TS
        theta = r0 + (C @ x).item()
        omega = (C @ A @ x).item()
        if noise:   # 자이로 진동(약 46 Hz, DLPF 후 σ≈25 °/s) + 각도 잡음
            theta += 0.04 * rng.standard_normal()
            omega += 25 * np.sqrt(2) * np.sin(2 * np.pi * 46.3 * t + 0.3) + 3 * rng.standard_normal()
        r = r1 if t >= tstep else r0
        e = r - theta
        omf += ad * (omega - omf)
        v = kp * e + I - kd * omf
        if n:
            vv, zi = signal.lfilter(nb, na, [v], zi=zi); v = float(vv[0])
        u = ff + v; us = min(255.0, max(0.0, u))
        if u == us or (u > us and e < 0) or (u < us and e > 0):
            I += ki * e * TS
        du = us - ff
        for _ in range(nsub):
            ubuf.append(du); ui = ubuf.pop(0)
            x = Ad @ x + Bd * ui
        tt.append(t); th.append(theta); uu.append(us); rr.append(r)
    return np.array(tt), np.array(th), np.array(uu), np.array(rr)


def metrics(t, th, r, tstep=1.0):
    m = t >= tstep; t, th = t[m] - tstep, th[m]
    r0, r1 = r[0], r[-1]; d = r1 - r0
    ys = signal.filtfilt(*signal.butter(2, 3, fs=1 / TS), (th - r0) / d)
    yf = ys[t > t[-1] - 1].mean()                     # 최종값 (기준 대비 비율)
    if not np.isfinite(yf) or abs(yf) < 0.05 or np.abs(ys).max() > 20:
        return dict(tr=np.nan, os=np.nan, ts=np.nan, ess=np.nan)
    y = ys / yf                                        # 상승시간·오버슈트·정착은 최종값 기준
    tr = t[np.argmax(y >= 0.9)] - t[np.argmax(y >= 0.1)] if y.max() >= 0.9 else np.nan
    os_ = max(0, y.max() - 1) * 100
    out = np.where(np.abs(y - 1) > 0.05)[0]
    ts = t[out[-1]] if len(out) else 0.0
    return dict(tr=tr, os=os_, ts=ts, ess=(1 - yf) * 100)


# ---------------------------------------------------------------- 설계
def best_gain(make, lo, hi, ms_max, n=None):
    """make(g) → (kp,ki,kd). Ms ≤ ms_max 이고 모든 K·ωr 에서 안정인 최대 g (이분법)"""
    ok = lambda g: (lambda r: r['stable'] and r['Ms'] <= ms_max)(robust(*make(g), n))
    if not ok(lo): return None
    for _ in range(40):
        mid = np.sqrt(lo * hi)
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo


def ziegler_nichols(K=K_NOM):
    Pz = plant(W, K) * np.exp(-1j * W * TS / 2)
    ph = np.unwrap(np.angle(Pz)); i = np.where(ph <= -np.pi)[0][0]
    Ku = 1 / np.abs(Pz[i]); Tu = 2 * np.pi / W[i]
    return Ku, Tu


def main():
    os.makedirs(OUT, exist_ok=True)
    lines = []
    say = lambda s='': (print(s), lines.append(s))  # noqa: E731
    NOTCH = (WR, 0.15, 0.7)
    say('플랜트: K∈%s, T=%.3f s, L=%.0f ms, 공진 %.2f Hz ζ=%.3f;  강인성 평가: K×ωr(±10%%) 9개 조합' % (K_SET, T, L * 1e3, WR / 2 / np.pi, ZR))
    designs = {}
    Ti = T
    # (4) 폐루프 스텝 확인용 P 제어
    g = best_gain(lambda g: (g, 0, 0), 0.05, 100, 2.0); designs['P (Ms≤2.0)'] = ((g, 0, 0), None)
    # (5) PI: Ti = T (극점 상쇄), Ms ≤ 1.6
    g = best_gain(lambda g: (g, g / Ti, 0), 0.1, 100, 1.6); designs['PI (Ti=T, Ms≤1.6)'] = ((g, g / Ti, 0), None)
    # PI + 노치: 공진을 깎아 이득을 더 올림
    g = best_gain(lambda g: (g, g / Ti, 0), 0.1, 100, 1.6, NOTCH); designs['PI + 노치 (Ms≤1.6)'] = ((g, g / Ti, 0), NOTCH)
    # PID + 노치: Td = Ti/4 정도, Ms ≤ 1.6
    for td_ratio in (0.05, 0.1):
        g = best_gain(lambda g: (g, g / Ti, g * Ti * td_ratio), 0.1, 100, 1.6, NOTCH)
        if g: designs['PID + 노치 (Td=%.2fTi)' % td_ratio] = ((g, g / Ti, g * Ti * td_ratio), NOTCH)
    # (6) Z-N (모델로 계산한 Ku, Tu)
    Ku, Tu = ziegler_nichols()
    say('\nZ-N 한계 이득 (공칭 K=%.2f 모델): Ku = %.2f PWM/°, Tu = %.3f s (%.2f Hz) — 위상 −180° 가 3.5 Hz 구조 공진에서 일어남' % (K_NOM, Ku, Tu, 1 / Tu))
    zn = {'Z-N P': (0.5 * Ku, 0, 0), 'Z-N PI': (0.45 * Ku, 0.45 * Ku / (Tu / 1.2), 0), 'Z-N PID': (0.6 * Ku, 0.6 * Ku / (Tu / 2), 0.6 * Ku * Tu / 8)}
    for k, v in zn.items(): designs[k] = (v, None)
    designs['Z-N PID + 노치'] = (zn['Z-N PID'], NOTCH)
    Ku2, _ = ziegler_nichols(0.21)
    say('  (K=0.21 이면 Ku = %.2f — 이득 변동만으로 한계가 %.0f%% 줄어듦)' % (Ku2, 100 * (1 - Ku2 / Ku)))

    say('\n%-24s %6s %6s %6s | %6s %6s %5s %6s %6s | %5s %5s %5s %5s %5s' % ('설계', 'Kp', 'Ki', 'Kd', '공칭Ms', '최악Ms', 'GM', 'PM', '안정', 'tr', 'OS%', 'ts', 'Δu', 'ess%'))
    sims = {}
    for name, ((kp, ki, kd), n) in designs.items():
        if kp is None: say('%-24s 조건을 만족하는 이득 없음' % name); continue
        rb = robust(kp, ki, kd, n)
        t, th, uu, rr = simulate(kp, ki, kd, n)
        m = metrics(t, th, rr)
        sims[name] = (t, th, uu, rr)
        say('%-24s %6.2f %6.2f %6.3f | %6.2f %6.2f %5.1f %6.1f %6s | %5.2f %5.1f %5.2f %5.0f %5.1f' % (
            name, kp, ki, kd, rb['nom']['Ms'], rb['Ms'], rb['gm'], rb['pm'], '예' if rb['stable'] else '아니오',
            m['tr'], m['os'], m['ts'], np.abs(uu - 150).max(), m['ess']))
    say('\n최악Ms·GM[dB]·PM[°]·안정 = 9개 플랜트 조합(K 0.12–0.21 × 공진 ±10%) 중 최악값. tr = 10–90% 상승시간[s], OS·ts(5%)는 최종값 기준, ess = 기준 대비 정상상태 오차[%].')
    say('최악Ms > 2 이거나 안정 = 아니오 이면 하드웨어 실험 전에 이득을 낮추세요. Δu = PWM 150 기준 최대 변화.')
    say('노치: fn=%.2f Hz, ζz=%.2f, ζp=%.2f (출력단 2차 필터). D 항: 자이로 ω 10 Hz LPF, 측정값에 적용.' % (NOTCH[0] / 2 / np.pi, NOTCH[1], NOTCH[2]))

    # K 변동에 대한 응답 (권장 설계)
    rec = 'PI + 노치 (Ms≤1.6)'
    fig, axs = plt.subplots(2, 2, figsize=(13, 8))
    for name in [k for k in designs if not k.startswith('Z-N')]:
        t, th, uu, rr = sims[name]; axs[0, 0].plot(t, th, label=name)
    axs[0, 0].plot(t, rr, 'k:', lw=1); axs[0, 0].set_title('설계별 스텝 응답 (공칭 K=0.15)'); axs[0, 0].set_ylabel('θ [°]'); axs[0, 0].legend(fontsize=7); axs[0, 0].grid(alpha=.3)
    for name in [k for k in designs if k.startswith('Z-N')]:
        t, th, uu, rr = sims[name]; axs[0, 1].plot(t, np.clip(th, 0, 45), label=name)
    t, th, uu, rr = simulate(*designs['Z-N PID'][0], None, K=0.21, wr=0.9 * WR, tend=4.0)
    axs[0, 1].plot(t, np.clip(th, 0, 45), 'm--', lw=1, label='Z-N PID, K=0.21·공진 −10% (불안정)')
    axs[0, 1].plot(t, rr, 'k:', lw=1); axs[0, 1].set_title('Z-N 설계 (공칭 K, 점선 = 최악 조합)'); axs[0, 1].legend(fontsize=7); axs[0, 1].grid(alpha=.3); axs[0, 1].set_ylim(0, 45)
    (kp, ki, kd), n = designs[rec]
    for K in K_SET:
        t, th, uu, rr = simulate(kp, ki, kd, n, K=K)
        axs[1, 0].plot(t, th, label='K=%.2f' % K); axs[1, 1].plot(t, uu, lw=.6, label='K=%.2f' % K)
    axs[1, 0].plot(t, rr, 'k:', lw=1); axs[1, 0].set_title('권장 설계 [%s] — K 변동' % rec); axs[1, 0].legend(fontsize=8); axs[1, 0].grid(alpha=.3); axs[1, 0].set_xlabel('t [s]')
    axs[1, 1].set_title('제어 입력 PWM (자이로 진동 잡음 포함)'); axs[1, 1].legend(fontsize=8); axs[1, 1].grid(alpha=.3); axs[1, 1].set_xlabel('t [s]')
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'step_responses.png'), dpi=85); plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
    for name in ['PI (Ti=T, Ms≤1.6)', rec, 'Z-N PID']:
        (kp, ki, kd), n = designs[name]
        Lw = ctrl(W, kp, ki, kd, n) * plant(W, 0.21)
        ax[0].loglog(W / 2 / np.pi, np.abs(Lw), label=name + ' (K=0.21)')
        ax[1].semilogx(W / 2 / np.pi, 20 * np.log10(np.abs(1 / (1 + Lw))), label=name)
    ax[0].axhline(1, color='k', lw=.6); ax[0].set_xlabel('Hz'); ax[0].set_title('개루프 |L|'); ax[0].grid(alpha=.3, which='both'); ax[0].legend(fontsize=7); ax[0].set_xlim(0.01, 30)
    ax[1].axhline(20 * np.log10(1.6), color='r', ls='--', lw=.8, label='Ms=1.6'); ax[1].set_xlabel('Hz'); ax[1].set_title('감도 |S| [dB] (K=0.21)'); ax[1].grid(alpha=.3, which='both'); ax[1].legend(fontsize=7); ax[1].set_xlim(0.01, 30); ax[1].set_ylim(-40, 20)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, 'loop_shapes.png'), dpi=85); plt.close(fig)
    with open(os.path.join(OUT, 'summary.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
