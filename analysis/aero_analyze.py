"""Aeropendulum 실험 데이터 분석 (별도 검증 데이터 없이 전달함수 추정)

사용:  python analysis/aero_analyze.py data/20261106 [--angle auto|theta|gyro]
출력:  data/20261106/analysis/ 아래 그림(png)과 summary.txt

하는 일
 1. 데이터 품질 진단: θacc 흔들림(진동), Madgwick θ 와 자이로 적분각(∫ω) 차이, 진동 주파수
 2. 각도 소스 선택: 기록된 Madgwick θ 가 자이로 적분각과 잘 맞으면 θ, 진동으로 떠내려갔으면 ∫ω (auto)
 2b. 반복 실험 재현성: 같은 입력을 두 번 준 기록 쌍에서 잡음(차이)과 신호(평균)를 분리해 SNR 계산
 3. 정적 특성(STAIR): 계단별 ∫ω 평균 → 운전점 부근 기울기 dθ/dPWM
 4. 자유 진동(STAIR 계단 직후 링잉) → ωn, ζ (감쇠 정현 맞춤)
 5. 동적 시험(STEP/CHIRP): 0.02–3 Hz 대역통과한 ∫ω 에 2차 모델 출력오차 맞춤
    - 별도 검증 데이터가 없으므로 실험 간 교차검증(STEP→CHIRP, CHIRP→STEP)과 전체 합동 추정을 함께 보고
 6. 후보 모델 G(s) = K ωn² / (s² + 2ζωn s + ωn²) [deg/PWM] 를 각 데이터에 대해 점수화
"""
import glob
import os
import sys

import numpy as np
from scipy import optimize, signal
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

plt.rcParams['font.family'] = ['Malgun Gothic', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
Ts = 0.01


def load(path):
    with open(path, encoding='utf-8') as f:
        head = f.readline()
    meta = {}
    for part in head.lstrip('#').split(';'):
        if '=' in part:
            k, v = part.split('=', 1)
            meta[k.strip()] = v.strip()
    with open(path, encoding='utf-8') as f:
        body = [ln for ln in f if not ln.startswith('#')][1:]      # 주석 줄(# 메타, # pid) 과 열 이름 제외
    M = np.genfromtxt(body, delimiter=',')
    r = dict(name=os.path.basename(path), type=meta.get('type', '?'), meta=meta,
             t=M[:, 0], u=M[:, 1], th=M[:, 3], om=M[:, 4], tha=M[:, 5], ph=M[:, 6].astype(int),
             ref=M[:, 7] if M.shape[1] > 7 else np.full(len(M), np.nan))
    r['thg'] = r['th'][0] + np.cumsum(r['om']) * Ts          # 자이로 적분각
    r['y'] = r['th']                                          # 분석에 쓸 각도 (main 에서 결정)
    r['short'] = r['type'] + '_' + r['name'].split('_')[-1].replace('.csv', '')
    return r


def fitpct(y, yh):
    return 100 * (1 - np.linalg.norm(y - yh) / np.linalg.norm(y - np.mean(y)))


def sim2(p, u, nd):
    K, wn, z = p
    bd, ad, _ = signal.cont2discrete(([K * wn * wn], [1.0, 2 * z * wn, wn * wn]), Ts, 'zoh')
    bd = np.ravel(bd)
    ud = np.concatenate([np.zeros(nd), u[:len(u) - nd]]) if nd else u
    return signal.lfilter(bd, ad, ud)


BAND = signal.butter(2, [0.02, 3.0], btype='band', fs=1 / Ts)


LP5 = signal.butter(2, 5.0, fs=1 / Ts)


def dyn_segment(r, src):
    """운전점(pre 후반 평균)을 빼고 가진 직전 3 s 부터 사용.
    src='theta': 출력만 5 Hz 영위상 저역통과 (입력 그대로) / src='gyro': 입출력 모두 0.02–3 Hz 대역통과(드리프트 제거)"""
    ph = r['ph']
    pre = np.where(ph == 1)[0]
    s0, preE = pre[len(pre) // 2], pre[-1] + 1
    end = np.where(ph >= 1)[0][-1] + 1
    start = max(s0, preE - 300)
    y = r['y']
    u0, y0 = r['u'][s0:preE].mean(), y[s0:preE].mean()
    uu, yy = r['u'][start:end] - u0, y[start:end] - y0
    if src == 'gyro':
        uf, yf = signal.filtfilt(*BAND, uu), signal.filtfilt(*BAND, yy)
    else:
        uf, yf = uu, signal.filtfilt(*LP5, yy)
    return dict(t=r['t'][start:end], u=uf, y=yf, uraw=uu, u0=u0, y0=y0)


def fit_dyn(segs):
    best = None
    lo, hi = [-5, np.log(0.5), np.log(0.02)], [5, np.log(30), np.log(3)]
    for nd in range(0, 21, 2):
        for w0 in (2, 3.5, 6):
            for z0 in (0.1, 0.3, 0.8):
                def res(q):
                    p = (q[0], np.exp(q[1]), np.exp(q[2]))
                    return np.concatenate([(sim2(p, s['u'], nd) - s['y']) / np.std(s['y']) for s in segs])
                try:
                    r = optimize.least_squares(res, [0.2, np.log(w0), np.log(z0)], bounds=(lo, hi), max_nfev=300)
                except Exception:
                    continue
                if best is None or r.cost < best[0]:
                    best = (r.cost, nd, (r.x[0], np.exp(r.x[1]), np.exp(r.x[2])))
    return best[2], best[1]


def ringing(r, min_len=120):
    """계단 직후 감쇠 정현 맞춤. 계단 유지가 짧으면(1.5 s) 저주파는 못 잡으므로 min_len 이상만"""
    bl, al = signal.butter(4, 5, fs=1 / Ts)
    th = signal.filtfilt(bl, al, r['y'])
    edges = np.where(np.diff(r['u']) != 0)[0] + 1
    out = []
    for i, e in enumerate(edges):
        stop = edges[i + 1] if i + 1 < len(edges) else len(th)
        idx = np.arange(e + 5, stop)
        if len(idx) < min_len:
            continue
        tt, y = r['t'][idx] - r['t'][idx[0]], th[idx]

        def f(p):
            A, s, w, phi, c, dr = p
            return A * np.exp(-s * tt) * np.cos(w * tt + phi) + c + dr * tt - y
        best = None
        for w0 in (2.5, 4, 6, 9):
            try:
                q = optimize.least_squares(f, [y.std() * 2, 0.5, w0, 0, y.mean(), 0],
                                           bounds=([0, 0, 1.2, -7, -400, -50], [200, 10, 20, 7, 400, 50]))
            except Exception:
                continue
            if best is None or q.cost < best.cost:
                best = q
        A, s, w, phi, c, dr = best.x
        fp = 100 * (1 - np.linalg.norm(best.fun) / np.linalg.norm(y - y.mean()))
        wn = np.hypot(s, w)
        at_bound = w < 1.25 or w > 19.5
        if A > 1.0 and fp > 75 and s / wn < 0.6 and not at_bound:
            out.append(dict(pwm=(r['u'][e - 1], r['u'][e]), amp=A, wn=wn, zeta=s / wn, fit=fp, level=c,
                            tt=tt, y=y, yh=y + best.fun))
    return out


def main(d, angle='auto'):
    files = sorted(glob.glob(os.path.join(d, 'aero_*.csv')))
    recs = [load(f) for f in files]
    od = os.path.join(d, 'analysis')
    os.makedirs(od, exist_ok=True)
    L = []
    say = lambda s='': (print(s), L.append(s))  # noqa: E731

    say('=== 1. 데이터 품질')
    for r in recs:
        run = r['u'] > 0
        drift = np.abs(r['thg'] - r['th'])[run].max() if run.any() else 0
        f, P = signal.welch(r['om'][run], fs=1 / Ts, nperseg=1024)
        say('%-36s %-6s β=%s dlpf=%s  θacc σ=%5.1f°  max|∫ω−θ|=%5.1f°  ω 진동 σ=%5.1f°/s @ %.1f Hz'
            % (r['name'], r['type'], r['meta'].get('beta'), r['meta'].get('dlpf'),
               r['tha'][run].std(), drift, r['om'][run].std(), f[np.argmax(P)]))

    gaps = [np.median(np.abs(r['thg'] - r['th'])[r['u'] > 0]) for r in recs if (r['u'] > 0).any()]
    if angle == 'auto':
        angle = 'theta' if np.median(gaps) < 3.0 else 'gyro'
    for r in recs:
        r['y'] = r['th'] if angle == 'theta' else r['thg']
    say('\n각도 소스: %s  (모터 구동 중 |∫ω−θ| 중앙값의 중앙값 = %.2f°; 3° 미만이면 Madgwick θ 사용)'
        % ('Madgwick θ (기록값)' if angle == 'theta' else '자이로 적분각 ∫ω', np.median(gaps)))

    say('\n=== 1b. 반복 실험 재현성 (같은 입력 두 번 → 차이 = 잡음 + 비재현성)')
    groups = {}
    for r in recs:
        groups.setdefault((r['type'], r['meta'].get('params')), []).append(r)
    rep_stats = {}
    for (typ, prm), rs in groups.items():
        if len(rs) < 2 or typ == 'MANUAL':
            continue
        a, b = rs[0], rs[1]
        n = min(len(a['y']), len(b['y']))
        act = (a['ph'][:n] >= 1) & (b['ph'][:n] >= 1)
        ya, yb = signal.filtfilt(*LP5, a['y'][:n]), signal.filtfilt(*LP5, b['y'][:n])
        off = np.mean(ya[act] - yb[act])
        if typ in ('STEP', 'CHIRP', 'PRBS'):
            exc = (a['ph'][:n] >= 2)
            ma = ya[exc] - ya[exc].mean(); mb = yb[exc] - yb[exc].mean()
            sig = np.var((ma + mb) / 2); noise = np.var(ma - mb) / 4   # 평균 신호의 잡음 분산 = σ²/2, 차이 = 2σ² → σ² = var(diff)/2
            snr1 = np.var((ma + mb) / 2) / (np.var(ma - mb) / 2)        # 단일 기록 기준 신호/잡음 전력비 (대략)
            corr = np.corrcoef(ma, mb)[0, 1]
            best_fit = 100 * (1 - np.sqrt(1 / (1 + snr1)))               # 잡음만으로 제한되는 최대 적합도(단일 기록)
            rep_stats[typ] = dict(snr=snr1, corr=corr, best=best_fit)
            say('%-6s %s vs %s: 가진 구간 상관 %.3f, 단일 기록 SNR ≈ %.1f (전력비), 잡음 한계 적합도 ≈ %.0f%%, 평균 레벨 차 %.2f°'
                % (typ, a['short'], b['short'], corr, snr1, best_fit, off))
        elif typ == 'STAIR':
            pa, pb = [], []
            for p in np.unique(a['ph'][a['ph'] >= 1]):
                ia = np.where(a['ph'] == p)[0]; ib = np.where(b['ph'] == p)[0]
                ia, ib = ia[len(ia) // 2:], ib[len(ib) // 2:]
                pa.append(a['y'][ia].mean()); pb.append(b['y'][ib].mean())
            dd = np.array(pa) - np.array(pb)
            rep_stats['STAIR'] = dict(maxdiff=np.abs(dd).max(), rms=np.sqrt(np.mean(dd ** 2)))
            say('STAIR  %s vs %s: 계단별 θss 차이 RMS %.2f°, 최대 %.2f°' % (a['short'], b['short'], rep_stats['STAIR']['rms'], rep_stats['STAIR']['maxdiff']))

    say('\n=== 2. 정적 특성 (계단 후반 50% 평균)')
    stairs = [r for r in recs if r['type'] == 'STAIR']
    fig, ax = plt.subplots(figsize=(8, 5))
    slopes = []
    for r in stairs:
        pts = []
        for p in np.unique(r['ph'][r['ph'] >= 1]):
            idx = np.where(r['ph'] == p)[0]
            idx = idx[len(idx) // 2:]
            pts.append((r['u'][idx].mean(), r['y'][idx].mean(), r['th'][idx].mean(), r['y'][idx].std()))
        pts = np.array(pts)
        ax.errorbar(pts[:, 0], pts[:, 1], pts[:, 3], marker='o', capsize=2, label=r['short'])
        if angle == 'gyro':
            ax.plot(pts[:, 0], pts[:, 2], 'x--', alpha=.6, label=r['short'] + ' (Madgwick)')
        say('%-18s ' % r['short'] + ' '.join('%d:%.1f' % (q[0], q[1]) for q in pts))
        for lo_, hi_ in ((130, 170), (160, 200)):
            sel = (pts[:, 0] >= lo_) & (pts[:, 0] <= hi_)
            if sel.sum() >= 3:
                k = np.polyfit(pts[sel, 0], pts[sel, 1], 1)[0]
                if lo_ == 130:
                    slopes.append(k)
                say('%-18s PWM %d–%d 기울기 dθ/dPWM = %.3f °/PWM' % (r['short'], lo_, hi_, k))
    ax.set_xlabel('PWM'); ax.set_ylabel('θss [°]'); ax.grid(alpha=.3); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(od, 'static_map.png'), dpi=90); plt.close(fig)

    say('\n=== 3. 자유 진동 (STAIR 계단 직후 링잉)')
    rings = []
    for r in stairs:
        for g in ringing(r):
            rings.append(g)
            say('%-36s PWM %3d→%3d  진폭 %.1f°  ωn=%.2f rad/s (%.2f Hz)  ζ=%.3f  적합 %.0f%%'
                % (r['name'], g['pwm'][0], g['pwm'][1], g['amp'], g['wn'], g['wn'] / 2 / np.pi, g['zeta'], g['fit']))
    if rings:
        n = min(4, len(rings))
        fig, axs = plt.subplots(1, n, figsize=(4 * n, 3.2), squeeze=False)
        for a, g in zip(axs[0], sorted(rings, key=lambda g: -g['fit'])[:n]):
            a.plot(g['tt'], g['y'], label='∫ω'); a.plot(g['tt'], g['yh'], '--', label='감쇠 정현')
            a.set_title('PWM %d→%d ωn=%.2f ζ=%.2f' % (g['pwm'][0], g['pwm'][1], g['wn'], g['zeta']), fontsize=9)
            a.grid(alpha=.3); a.legend(fontsize=7)
        fig.tight_layout(); fig.savefig(os.path.join(od, 'ringing.png'), dpi=90); plt.close(fig)

    say('\n=== 4. 동적 시험 출력오차 맞춤 (교차검증: 각 기록으로 추정 → 나머지 기록에 비교)')
    dyn = {r['short']: dyn_segment(r, angle) for r in recs if r['type'] in ('STEP', 'CHIRP', 'PRBS')}
    keys = list(dyn)
    combos = [[k] for k in keys] + ([keys] if len(keys) > 1 else [])
    fitted = {}
    for est in combos:
        p, nd = fit_dyn([dyn[k] for k in est])
        fitted['+'.join(est)] = (p, nd)
        say('추정=%-14s K=%.3f ωn=%.2f ζ=%.2f 지연=%3d ms | %s' % (
            '전체' if len(est) > 1 else est[0], p[0], p[1], p[2], nd * 10,
            '  '.join('%s=%5.1f%%' % (k, fitpct(dyn[k]['y'], sim2(p, dyn[k]['u'], nd))) for k in keys)))

    say('\n=== 5. 후보 모델 (정적 기울기 + 링잉) 점수')
    K = float(np.mean(slopes)) if slopes else 0.2
    near = [g for g in rings if g['level'] > 25] or rings
    wn = float(np.median([g['wn'] for g in near])) if near else 3.3
    zeta = float(np.median([g['zeta'] for g in near])) if near else 0.3
    cands = [(K, wn, zeta)]
    if rings:
        best_ring = max(rings, key=lambda g: g['fit'])
        cands.append((K, best_ring['wn'], best_ring['zeta']))
    for c in cands:
        say('G(s) = %.3f·%.2f² / (s² + %.3f s + %.2f)  [K=%.3f °/PWM, ωn=%.2f, ζ=%.2f] | %s' % (
            c[0], c[1], 2 * c[2] * c[1], c[1] ** 2, c[0], c[1], c[2],
            '  '.join('%s=%5.1f%%' % (k, fitpct(dyn[k]['y'], sim2(c, dyn[k]['u'], 0))) for k in keys)))

    fig, axs = plt.subplots(len(keys), 1, figsize=(12, 3.4 * len(keys)), squeeze=False)
    pj, ndj = fitted['+'.join(keys)] if len(keys) > 1 else fitted[keys[0]]
    for a, k in zip(axs[:, 0], keys):
        s = dyn[k]
        a.plot(s['t'], s['y'], label='측정 (%s)' % ('θ, 5 Hz LPF' if angle == 'theta' else '∫ω, 0.02–3 Hz'))
        a.plot(s['t'], sim2(cands[0], s['u'], 0), label='후보 모델 (정적 기울기+링잉)')
        a.plot(s['t'], sim2(pj, s['u'], ndj), ':', label='합동 OE 맞춤')
        a2 = a.twinx(); a2.plot(s['t'], s['uraw'] + s['u0'], color='0.6', lw=.7); a2.set_ylabel('PWM')
        a.set_title(k); a.grid(alpha=.3); a.legend(loc='upper left', fontsize=8); a.set_ylabel('Δθ [°]')
    fig.tight_layout(); fig.savefig(os.path.join(od, 'dynamic_fit.png'), dpi=90); plt.close(fig)

    with open(os.path.join(od, 'summary.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')
    print('\n결과: ' + od)


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    ang = 'auto'
    if '--angle' in sys.argv:
        ang = sys.argv[sys.argv.index('--angle') + 1]
        args = [a for a in args if a != ang]
    main(args[0] if args else 'data/20261106', ang)
