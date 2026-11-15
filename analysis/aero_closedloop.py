"""폐루프 기록(CLSTEP·RELAY)을 식별 모델 + 펌웨어와 같은 PID 로 재현해 실측과 비교

사용:
  python analysis/aero_closedloop.py data/20261113_run2 data/20261113_run3 [--model K T L fr zr [Ks Tc]] [--out …]
  --model 기본값 = 2026-11-13 기구 수정 후 모델 (data/20261113_run1/analysis/REPORT.md)
  모델: G(s) = K e^(−Ls)/(T s+1) · ωr²/(s²+2ζr ωr s+ωr²)  +  Ks e^(−Ls)/(Tc s+1)   [deg/PWM]
        (뒤의 항 = 느린 병렬 경로(케이블·피벗 크리프로 추정). 정적 이득 = K + Ks)

하는 일
 1. CSV 두 번째 줄 `# pid; …` 에서 이득·ff·umin·노치를 읽음. 최대 PWM 은 CSV 에 없으므로 기록 최대값이 210 을 넘으면 255, 아니면 210 으로 가정
 2. CLSTEP: 기준 스텝 2 s 전(6 s)부터 끝까지 시뮬레이션. 초기 상태 = 6–8 s 평균 (θ0, u0) 에서 평형, 적분기는 u0 에 맞춤
    플랜트 = θ0 + G(s)·(u − u0) (운전점 편차 선형 모델), 1 ms 연속 + 10 ms 펌웨어 pidStep(D 10 Hz 필터, 노치 사전왜곡 쌍선형, umin/umax 포화, 조건부 적분)
    → 상승시간·오버슈트·정상오차·적합도(θ, PWM), 그리고 기록별로 가장 잘 맞는 K (Ks·Tc 고정)
 3. RELAY: 실측 진동 주기 Tu·진폭 a → Ku = 4d/(πa). 같은 릴레이를 모델에서 돌려 예측 Tu·a·Ku 와 비교
 4. 기록마다 제어기 + 모델의 주파수 여유(Ms, 위상여유, 이득여유, 안정 여부)를 K 범위에서 계산
"""
import glob
import os
import sys

import numpy as np
from scipy import linalg, signal
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

plt.rcParams['font.family'] = ['Malgun Gothic', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aero_analyze as A  # noqa: E402

TS = 0.01
TAU_D = 0.0159                      # 펌웨어 D 항 저역통과 시정수 (10 Hz)
MODEL = dict(K=0.14, T=0.319, L=0.021, fr=2.84, zr=0.136, Ks=0.05, Tc=6.0)   # 11-13 기구 수정 후 (data/20261113_run1)
MARGIN_K = (0.12, 0.14, 0.24)        # 여유 계산 K (빠른 경로): 하한 / 공칭(20°) / 30° 릴레이
K_GRID = np.round(np.arange(0.08, 0.301, 0.005), 3)
W = np.geomspace(0.05, 300, 6000)


def read_pid(path):
    with open(path, encoding='utf-8') as f:
        f.readline(); ln = f.readline()
    q = dict(kp=0, ki=0, kd=0, ff=150, umin=0, notch_cHz=0, zz=150, zp=700)
    if ln.startswith('# pid'):
        for part in ln.lstrip('#').split(';'):
            if '=' in part:
                k, v = part.split('=', 1); q[k.strip()] = float(v)
    return q


def notch_coef(q):
    """펌웨어 notchCoeffs 와 같은 식 (사전왜곡 쌍선형)"""
    if not q['notch_cHz']:
        return None
    w = 2 * np.pi * q['notch_cHz'] * 0.01; k = w / np.tan(w * TS / 2); k2, w2 = k * k, w * w
    zz, zp = q['zz'] / 1000, q['zp'] / 1000; a0 = k2 + 2 * zp * w * k + w2
    return np.array([(k2 + 2 * zz * w * k + w2) / a0, 2 * (w2 - k2) / a0, (k2 - 2 * zz * w * k + w2) / a0]), \
        np.array([2 * (w2 - k2) / a0, (k2 - 2 * zp * w * k + w2) / a0])


def plant_ss(m):
    wr = 2 * np.pi * m['fr']
    num = [m['K'] * wr ** 2]; den = np.polymul([m['T'], 1], [1, 2 * m['zr'] * wr, wr ** 2])
    Am, Bm, Cm, _ = signal.tf2ss(num, den)
    dt = 0.001; Ad = linalg.expm(Am * dt); Bd = np.linalg.solve(Am, (Ad - np.eye(len(Am))) @ Bm)
    return Am, Ad, Bd, Cm


def simulate(m, q, umax, ref, th0, u0, relay=None, x0=None):
    """ref: 매 10 ms 기준 배열. 운전점 (th0, u0) 평형에서 시작. relay=(d, h, center, k0) 이면 k0 샘플부터 릴레이 출력(그 전은 center 유지).
    반환: θ, u (10 ms 샘플, 측정 시점 = 틱 시작)"""
    Am, Ad, Bd, Cm = plant_ss(m)
    nd = int(round(m['L'] / 0.001)); buf = [0.0] * nd
    x = np.zeros((len(Am), 1)) if x0 is None else x0.copy()
    nc = notch_coef(q); z1 = z2 = 0.0
    lo = min(q['umin'], umax)
    I = u0 - q['ff'] - q['kp'] * (ref[0] - th0); omf = 0.0; level = 1
    Ks, Tc = m.get('Ks', 0.0), m.get('Tc', 5.0); xs = 0.0; ac = np.exp(-0.001 / Tc)   # 느린 병렬 경로 Ks/(Tc s+1)
    th_out, u_out = np.empty(len(ref)), np.empty(len(ref))
    for k, r in enumerate(ref):
        th = th0 + (Cm @ x).item() + xs; om = (Cm @ Am @ x).item() + (Ks * (buf[0] if buf else u - u0) - xs) / Tc
        e = r - th
        if relay is None:
            omf += TS / (TAU_D + TS) * (om - omf)
            v = q['kp'] * e + I - q['kd'] * omf
            if nc is not None:
                b, a = nc; y = b[0] * v + z1; z1 = b[1] * v - a[0] * y + z2; z2 = b[2] * v - a[1] * y; v = y
            uu = q['ff'] + v; us = min(umax, max(lo, uu))
            if uu == us or (uu > us and e < 0) or (uu < us and e > 0):
                I = max(-255.0, min(255.0, I + q['ki'] * e * TS))
            u = float(np.round(us))
        else:
            d, h, c, k0 = relay
            if k < k0: e = 0.0
            if e > h: level = 1
            elif e < -h: level = -1
            u = c if k < k0 else max(lo, min(umax, float(np.round(c + level * d))))
        th_out[k], u_out[k] = th, u
        for _ in range(10):
            buf.append(u - u0); ud = buf.pop(0); x = Ad @ x + Bd * ud
            xs = ac * xs + (1 - ac) * Ks * ud
    return th_out, u_out


def step_metrics(t, y, r0, r1, t1, t2):
    """기준 r0→r1 이 t1 에 바뀌고 t2 에 돌아감. 10–90% 상승시간, 오버슈트, 정상오차(t2 직전 2 s 평균)"""
    m = (t >= t1) & (t < t2); tt, yy = t[m] - t1, signal.filtfilt(*signal.butter(2, 4, fs=1 / TS), y)[m]
    s = (yy - r0) / (r1 - r0)
    tr = tt[np.argmax(s >= 0.9)] - tt[np.argmax(s >= 0.1)] if s.max() >= 0.9 else np.nan
    yss = yy[tt > tt[-1] - 2].mean()
    return dict(tr=tr, os=max(0.0, (s.max() - 1) * 100), ess=r1 - yss, osc=np.std(y[m][tt > tt[-1] - 3] - yss))


def margins(m, q, Ks):
    """제어기(펌웨어 구조 + ZOH 반 샘플) × 모델. 안정성은 나이퀴스트 대신 극점(지연 2차 Pade)으로"""
    s = 1j * W; wr = 2 * np.pi * m['fr']
    C = (q['kp'] + q['ki'] / s + q['kd'] * s / (TAU_D * s + 1)) * np.exp(-s * TS / 2)
    if q['notch_cHz']:
        wn = 2 * np.pi * q['notch_cHz'] * 0.01; zz, zp = q['zz'] / 1000, q['zp'] / 1000
        C = C * (s ** 2 + 2 * zz * wn * s + wn ** 2) / (s ** 2 + 2 * zp * wn * s + wn ** 2)
    out = []
    for K in Ks:
        P = (K / (m['T'] * s + 1) * wr ** 2 / (s ** 2 + 2 * m['zr'] * wr * s + wr ** 2)
             + m.get('Ks', 0) / (m.get('Tc', 1) * s + 1)) * np.exp(-s * m['L'])
        Lw = C * P; mag = np.abs(Lw); ph = np.unwrap(np.angle(Lw))
        ic = np.where((mag[:-1] >= 1) & (mag[1:] < 1))[0]
        pm = 180 + np.degrees(ph[ic[0]]) if len(ic) else np.nan
        gms = [1 / mag[i] for i in range(len(W) - 1)
               if np.floor((ph[i] + np.pi) / (2 * np.pi)) != np.floor((ph[i + 1] + np.pi) / (2 * np.pi))]
        out.append(dict(K=K, Ms=np.max(1 / np.abs(1 + Lw)), pm=pm, gm=20 * np.log10(min(gms)) if gms else np.inf,
                        stable=stable(m, q, K)))
    return out


def stable(m, q, K):
    wr = 2 * np.pi * m['fr']; tau = m['L'] + TS / 2
    pn, pd = [tau ** 2 / 12, -tau / 2, 1], [tau ** 2 / 12, tau / 2, 1]
    Df = np.polymul([m['T'], 1], [1, 2 * m['zr'] * wr, wr ** 2]); Nf = [K * wr ** 2]
    Ks, Tc = m.get('Ks', 0), m.get('Tc', 1)
    if Ks:                                            # 빠른 경로 + 느린 경로 Ks/(Tc s+1)
        Nf = np.polyadd(np.polymul(Nf, [Tc, 1]), np.polymul([Ks], Df)); Df = np.polymul(Df, [Tc, 1])
    Pn = np.polymul(Nf, pn); Pd = np.polymul(Df, pd)
    if q['ki']:
        Cn = np.polyadd(np.polymul([q['kp'], q['ki']], [TAU_D, 1]), [q['kd'], 0, 0]); Cd = np.polymul([1, 0], [TAU_D, 1])
    else:                                             # 적분 없음: s 공통인수 제거
        Cn = np.polyadd(np.polymul([q['kp']], [TAU_D, 1]), [q['kd'], 0]); Cd = [TAU_D, 1]
    if q['notch_cHz']:
        wn = 2 * np.pi * q['notch_cHz'] * 0.01; zz, zp = q['zz'] / 1000, q['zp'] / 1000
        Cn = np.polymul(Cn, [1, 2 * zz * wn, wn ** 2]); Cd = np.polymul(Cd, [1, 2 * zp * wn, wn ** 2])
    return bool(np.all(np.real(np.roots(np.polyadd(np.polymul(Cd, Pd), np.polymul(Cn, Pn)))) < 0))


def relay_measure(t, th, r, t_start):
    """t_start 이후 θ−r 의 상향 영점 교차로 주기, 사이클별 최대−최소로 진폭"""
    m = t >= t_start; tt, e = t[m], signal.filtfilt(*signal.butter(2, 8, fs=1 / TS), th[m]) - r[m]
    e = e - e.mean()
    up = np.where((e[:-1] < 0) & (e[1:] >= 0))[0]
    if len(up) < 3:
        return np.nan, np.nan
    Tu = np.median(np.diff(tt[up]))
    amps = [(e[a:b].max() - e[a:b].min()) / 2 for a, b in zip(up[:-1], up[1:])]
    return Tu, float(np.median(amps))


def fitpct(y, yh):
    return 100 * (1 - np.linalg.norm(y - yh) / np.linalg.norm(y - np.mean(y)))


def main(dirs, m, out):
    files = sorted(f for d in dirs for f in glob.glob(os.path.join(d, 'aero_*.csv')))
    recs = [(A.load(f), read_pid(f)) for f in files]
    recs = [(r, q) for r, q in recs if r['type'] in ('CLSTEP', 'RELAY')]
    os.makedirs(out, exist_ok=True)
    L = []
    say = lambda s='': (print(s), L.append(s))  # noqa: E731
    say('모델: G(s) = K e^(-%.3fs) / (%.3f s + 1) · ωr² / (s² + 2·%.3f·ωr s + ωr²) + %.3f e^(-%.3fs) / (%.1f s + 1), ωr = 2π·%.2f Hz;  공칭 K = %.3f °/PWM (정적 %.3f)'
        % (m['L'], m['T'], m['zr'], m.get('Ks', 0), m['L'], m.get('Tc', 0), m['fr'], m['K'], m['K'] + m.get('Ks', 0)))
    say('시뮬레이션: 기준 스텝 2 s 전 평형에서 시작, 운전점 편차 선형 모델 + 펌웨어 pidStep (umin·umax 포화 포함). 잡음·마찰·케이블 없음.')
    say('\n=== 1. CLSTEP: 실측 vs 모델 (공칭 K / 기록별 최적 K)')
    say('%-14s %-34s %4s | %-28s | %-28s | %-28s | %5s %6s %6s' % (
        '기록', '제어기', 'umax', '실측 tr / OS / ess / 진동σ', '모델(공칭K) tr / OS / ess', '모델(최적K) tr / OS / ess',
        '최적K', '적합θ', '적합u'))
    fig, axs = plt.subplots(sum(r['type'] == 'CLSTEP' for r, _ in recs), 2, figsize=(14, 2.6 * len(recs)), squeeze=False)
    row = 0; rows = []
    for r, q in recs:
        if r['type'] != 'CLSTEP':
            continue
        umax = 255 if r['u'].max() > 210 else 210
        t = r['t']; i0 = np.searchsorted(t, 6.0); i1 = np.searchsorted(t, 8.0)
        th0, u0 = r['th'][i0:i1].mean(), r['u'][i0:i1].mean()
        ref = r['ref'][i0:]; tt = t[i0:]; y, uu = r['th'][i0:], r['u'][i0:]
        r0, r1 = ref[0], ref[np.searchsorted(tt, 9.0)]
        t2 = tt[np.where(r['ph'][i0:] == 3)[0][0]]
        meas = step_metrics(tt, y, r0, r1, 8.0, t2)
        best = None
        for K in K_GRID:
            mk = dict(m, K=K); ys, us = simulate(mk, q, umax, ref, th0, u0)
            if not np.all(np.isfinite(ys)) or np.abs(ys).max() > 200:
                continue
            f = fitpct(y, ys)
            if best is None or f > best[0]:
                best = (f, K, ys, us)
        ysn, usn = simulate(m, q, umax, ref, th0, u0)
        mn = step_metrics(tt, np.clip(ysn, -90, 90), r0, r1, 8.0, t2)
        fb, Kb, ysb, usb = best
        mb = step_metrics(tt, ysb, r0, r1, 8.0, t2)
        ctl = 'Kp %.3g Ki %.3g Kd %.3g ff %d umin %d %s' % (q['kp'], q['ki'], q['kd'], q['ff'], q['umin'],
                                                           ('노치 %.2f Hz' % (q['notch_cHz'] / 100)) if q['notch_cHz'] else '노치 없음')
        fm = lambda d: '%4.2f s / %4.1f%% / %+5.2f° / %4.2f°' % (d['tr'], d['os'], d['ess'], d['osc'])  # noqa: E731
        say('%-14s %-34s %4d | %-28s | %-28s | %-28s | %5.3f %5.1f%% %5.1f%%' % (
            r['short'], ctl, umax, fm(meas), fm(mn)[:-8], fm(mb)[:-8], Kb, fb, fitpct(uu, usb)))
        rows.append(dict(name=r['short'], q=q, meas=meas, nom=mn, best=mb, Kb=Kb, fit=fb, fitn=fitpct(y, ysn),
                         fitu=fitpct(uu, usb)))
        a, b = axs[row]
        a.plot(tt, y, lw=.8, label='실측 θ'); a.plot(tt, np.clip(ysn, -5, 50), lw=1, label='모델 K=%.3f (적합 %.0f%%)' % (m['K'], fitpct(y, ysn)))
        a.plot(tt, ysb, '--', lw=1, label='모델 K=%.3f (적합 %.0f%%)' % (Kb, fb)); a.plot(tt, ref, 'k:', lw=.8)
        a.set_ylim(min(r0, r1) - 8, max(r0, r1) + 10); a.set_ylabel('θ [°]'); a.grid(alpha=.3); a.legend(fontsize=7, loc='upper right')
        a.set_title('%s  %s' % (r['short'], ctl), fontsize=9)
        b.plot(tt, uu, lw=.6, label='실측 PWM'); b.plot(tt, usn, lw=.8, label='모델 K=%.3f' % m['K']); b.plot(tt, usb, '--', lw=.8, label='모델 K=%.3f' % Kb)
        b.axhline(umax, color='r', lw=.5); b.set_ylabel('PWM'); b.grid(alpha=.3); b.legend(fontsize=7, loc='upper right')
        row += 1
    axs[-1, 0].set_xlabel('t [s]'); axs[-1, 1].set_xlabel('t [s]')
    fig.tight_layout(); fig.savefig(os.path.join(out, 'closedloop_fit.png'), dpi=75); plt.close(fig)
    say('tr = 10–90% 상승시간, OS = 오버슈트, ess = 기준 − (복귀 직전 2 s 평균), 진동σ = 그 3 s 의 표준편차(실측만 의미). 적합 = 6 s~끝 θ·PWM 의 NRMSE 적합도.')

    say('\n=== 2. RELAY: Ku = 4d/(πa), Tu')
    rel = [(r, q) for r, q in recs if r['type'] == 'RELAY']
    fig, axs = plt.subplots(max(1, len(rel)), 1, figsize=(12, 3 * max(1, len(rel))), squeeze=False)
    for (r, q), a in zip(rel, axs[:, 0]):
        prm = dict(kv.split('=') for kv in r['meta']['params'].split())
        d, h = float(prm['d']), float(prm['hyst']); umax = 255 if r['u'].max() > 210 else 210
        t = r['t']; i2 = np.where(r['ph'] == 2)[0][0]
        c = r['u'][i2:].astype(float); center = (c.max() + c.min()) / 2
        Tu, amp = relay_measure(t, r['th'], r['ref'], t[i2] + 2.0)
        i0 = i2 - 100; th0 = r['th'][i0:i2].mean()
        ref = r['ref'][i0:]
        say('%-14s d=%g h=%g°  중심 PWM %.0f (상한 %d, 하한 %d)  실측: Tu = %.3f s (%.2f Hz), a = %.2f° → Ku = %.2f PWM/°'
            % (r['short'], d, h, center, c.max(), c.min(), Tu, 1 / Tu, amp, 4 * d / np.pi / amp))
        for K in sorted({m['K'], 0.12, 0.17, 0.20, 0.24}):
            mk = dict(m, K=K)
            ys, us = simulate(mk, q, umax, ref, th0, center, relay=(d, h, center, i2 - i0))
            Tm, am = relay_measure(t[i0:], ys, ref, t[i2] + 2.0)
            say('    모델 K=%.3f: Tu = %.3f s (%.2f Hz), a = %.2f° → Ku = %.2f' % (K, Tm, 1 / Tm, am, 4 * d / np.pi / am))
            if K == m['K']:
                a.plot(t[i0:], ys, lw=1, label='모델 K=%.3f: Tu %.3f s, a %.2f°' % (K, Tm, am))
        a.plot(t[i0:], r['th'][i0:], lw=.8, label='실측: Tu %.3f s, a %.2f°' % (Tu, amp)); a.plot(t[i0:], ref, 'k:', lw=.8)
        a.set_title('%s (d=%g, h=%g°)' % (r['short'], d, h), fontsize=9); a.grid(alpha=.3); a.legend(fontsize=7, loc='upper right'); a.set_ylabel('θ [°]')
    # 모델의 −180° 교차 (Z-N 이론값)
    wr = 2 * np.pi * m['fr']; s = 1j * W
    P1 = np.exp(-s * (m['L'] + TS / 2)) / (m['T'] * s + 1) * wr ** 2 / (s ** 2 + 2 * m['zr'] * wr * s + wr ** 2)
    ph = np.unwrap(np.angle(P1)); i = np.where(ph <= -np.pi)[0][0]
    say('모델 위상 −180° 교차 (빠른 경로, ZOH 반 샘플 포함): %.2f Hz → Tu = %.3f s, Ku = %.2f / K  (%s)'
        % (W[i] / 2 / np.pi, 2 * np.pi / W[i], 1 / abs(P1[i]), ', '.join('K=%.2f: %.1f' % (K, 1 / abs(P1[i]) / K) for K in MARGIN_K)))
    axs[-1, 0].set_xlabel('t [s]'); fig.tight_layout(); fig.savefig(os.path.join(out, 'relay_fit.png'), dpi=75); plt.close(fig)

    say('\n=== 3. 제어기별 여유 (모델, K 범위)')
    seen = {}
    for r, q in recs:
        key = (q['kp'], q['ki'], q['kd'], q['notch_cHz'])
        if r['type'] == 'CLSTEP' and key not in seen:
            seen[key] = (r['short'], q)
    seen['old'] = ('(11-06 권장 PI+노치, 실험 안 함)', dict(kp=7.85, ki=23.9, kd=0, ff=150, umin=0, notch_cHz=347, zz=150, zp=700))
    say('%-34s %-30s | %s' % ('제어기', '기록', '   '.join('K=%.2f: Ms / PM / GM / 안정' % K for K in MARGIN_K)))
    for key, (name, q) in seen.items():
        ms = margins(m, q, MARGIN_K)
        ctl = 'Kp %.3g Ki %.3g Kd %.3g %s' % (q['kp'], q['ki'], q['kd'], ('노치 %.2f' % (q['notch_cHz'] / 100)) if q['notch_cHz'] else '')
        say('%-34s %-30s | %s' % (ctl, name, '   '.join('%5.2f / %6s / %4.1f dB / %s' % (x['Ms'], ('%.1f°' % x['pm']) if np.isfinite(x['pm']) else '—', x['gm'], '예' if x['stable'] else '아니오') for x in ms)))
    with open(os.path.join(out, 'closedloop.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')
    print('\n결과: ' + out)
    return rows


if __name__ == '__main__':
    a = sys.argv[1:]
    m = dict(MODEL)
    if '--model' in a:
        i = a.index('--model'); j = i + 1
        while j < len(a) and not a[j].startswith('--') and not os.path.isdir(a[j]):
            j += 1
        vals = list(map(float, a[i + 1:j])); m.update(zip(('K', 'T', 'L', 'fr', 'zr', 'Ks', 'Tc'), vals))
        if len(vals) == 5:
            m['Ks'] = 0.0
        del a[i:j]
    out = None
    if '--out' in a:
        i = a.index('--out'); out = a[i + 1]; del a[i:i + 2]
    dirs = a or ['data/20261113_run2', 'data/20261113_run3']
    main(dirs, m, out or os.path.join(dirs[-1], 'analysis'))
