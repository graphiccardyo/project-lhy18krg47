function R = aero_pid_design(Kset)
%AERO_PID_DESIGN  식별 모델로 P / PI / PID / Z-N 설계를 비교한다 (Control System Toolbox).
%   R = aero_pid_design              % K = 0.12, 0.164, 0.23 (2026-11-15 재식별 범위)
%   R = aero_pid_design([0.164 0.2])
%
%   플랜트 (2026-11-15 재식별, data/20261115_run1·run2/analysis/REPORT.md, θ≈20°·PWM≈150 소신호, deg/PWM):
%     P(s) = [K/(0.324 s + 1) · ωr²/(s² + 2ζr ωr s + ωr²) + Ks/(Tc s + 1)] e^(-0.024 s)
%     ωr = 2π·2.35 rad/s (2.35 Hz), ζr = 0.084, Ks = 0.12, Tc = 3 s (느린 경로: 케이블·크리프 추정)
%   (11-13 모델: T 0.319, L 0.021, 2.84 Hz ζ 0.136, Ks 0.05 Tc 6, K 0.12–0.24;
%    기구 수정 전 11-06 모델: K e^(-0.016 s)/(0.328 s + 1) · 21.8²/(s² + 5.37 s + 21.8²), K 0.12–0.21)
%   제어기 (펌웨어와 같은 구조):
%     u = ff + Kp e + Ki ∫e − Kd ω_f,  ω_f = 자이로 각속도 10 Hz 저역통과 (D 는 측정값에)
%     선택: 출력 노치 N(s) = (s² + 2·0.15·ωr s + ωr²)/(s² + 2·0.7·ωr s + ωr²)
%   이득 값은 analysis/pid_design.py 에서 "K 0.12–0.23 × 공진 ±10% 에서 최대 감도 Ms ≤ 1.6" 으로 정한 것.
%   구현 지연: ZOH 반 샘플(5 ms) 포함.

if nargin < 1, Kset = [0.12 0.164 0.23]; end
T = 0.324; L = 0.024; wr = 2 * pi * 2.35; zr = 0.084; Ks = 0.12; Tc = 3; Ts = 0.01; tauD = 1 / (2 * pi * 10);
s = tf('s');
P = @(k) (k / (T * s + 1) * wr ^ 2 / (s ^ 2 + 2 * zr * wr * s + wr ^ 2) + Ks / (Tc * s + 1)) * exp(-L * s);
zoh = exp(-s * Ts / 2);
N = (s ^ 2 + 2 * 0.15 * wr * s + wr ^ 2) / (s ^ 2 + 2 * 0.7 * wr * s + wr ^ 2);
one = tf(1);

%            이름            Kp     Ki      Kd     노치
D = { 'P (목표 4)',          1.59,  0,      0,     one
      'PI',                  1.17,  3.61,   0,     one
      'PI + 노치 (권장)',    4.30,  13.26,  0,     N
      'PID + 노치',          4.11,  12.70,  0.067, N
      'Z-N PID',             2.95,  13.63,  0.159, one
      '손튜닝 PID (11-13)',  0.90,  7.00,   0.020, one };

% Z-N 한계이득 (공칭 K = 0.164): 위상 -180° 주파수의 이득
[~, ~, Wcg] = margin(P(0.164) * zoh);
Ku = 1 / abs(freqresp(P(0.164) * zoh, Wcg)); Tu = 2 * pi / Wcg;
fprintf('Z-N 한계이득 (모델, K=0.164): Ku = %.2f PWM/deg, Tu = %.3f s (%.2f Hz) — 2.35 Hz 구조 공진에서 결정됨\n\n', Ku, Tu, 1 / Tu);

w = logspace(-2, 2.5, 3000);
fprintf('%-18s %6s %6s %6s | %6s %7s %7s %7s | %6s %6s %6s\n', '설계', 'Kp', 'Ki', 'Kd', 'K', 'GM[dB]', 'PM[deg]', 'Ms', 'tr[s]', 'OS[%]', 'ts[s]');
R = struct('name', {}, 'K', {}, 'GM', {}, 'PM', {}, 'Ms', {}, 'info', {});
figure('Name', 'Aeropendulum PID 설계 비교');
for i = 1:size(D, 1)
    [name, kp, ki, kd, Nf] = D{i, :};
    Cpi = kp + ki / s; Cd = kd * s / (tauD * s + 1);
    subplot(2, 3, i); hold on; grid on; title(name);
    for k = Kset
        G = P(k) * Nf * zoh;
        Lw = G * (Cpi + Cd);                  % 안정성·여유는 전체 루프
        Tr = feedback(G, Cpi + Cd) * Cpi;     % 기준 → θ (D 는 측정값에만 → 기준 킥 없음)
        Sens = feedback(one, Lw);
        Ms = max(abs(squeeze(freqresp(Sens, w))));
        [Gm, Pm] = margin(Lw);
        stable = isstable(feedback(Lw, 1));
        if stable
            info = stepinfo(Tr, 'SettlingTimeThreshold', 0.05);
            step(Tr, 4);
        else
            info = struct('RiseTime', NaN, 'Overshoot', NaN, 'SettlingTime', NaN);
        end
        fprintf('%-18s %6.2f %6.2f %6.3f | %6.2f %7.1f %7.1f %7.2f | %6.2f %6.1f %6.2f %s\n', name, kp, ki, kd, k, ...
            20 * log10(Gm), Pm, Ms, info.RiseTime, info.Overshoot, info.SettlingTime, ternary(stable, '', ' ← 불안정'));
        R(end + 1) = struct('name', name, 'K', k, 'GM', 20 * log10(Gm), 'PM', Pm, 'Ms', Ms, 'info', info); %#ok<AGROW>
    end
    legend(arrayfun(@(k) sprintf('K=%.2f', k), Kset, 'UniformOutput', false), 'Location', 'southeast');
end
fprintf('\nMs (최대 감도) 1.4–2.0 이 일반적 권장 범위. Ms > 2 이거나 불안정이면 하드웨어 실험 전에 이득을 낮출 것.\n');
fprintf('실험: lab/index.html ⑥ 탭에서 같은 이득으로 폐루프 스텝 → CSV → 측정 vs 모델 비교.\n');
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
