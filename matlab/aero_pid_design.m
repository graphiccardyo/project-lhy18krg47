function R = aero_pid_design(Kset)
%AERO_PID_DESIGN  식별 모델로 P / PI / PID / Z-N 설계를 비교한다 (Control System Toolbox).
%   R = aero_pid_design              % K = 0.12, 0.15, 0.21 (2026-11-06 측정 범위)
%   R = aero_pid_design([0.15 0.2])
%
%   플랜트 (data/20261106_run3/analysis/REPORT.md, PWM≈150 소신호, deg/PWM):
%     P(s) = K e^(-0.016 s)/(0.328 s + 1) · ωr²/(s² + 2ζr ωr s + ωr²),  ωr = 21.8 rad/s (3.47 Hz), ζr = 0.123
%   제어기 (펌웨어와 같은 구조):
%     u = ff + Kp e + Ki ∫e − Kd ω_f,  ω_f = 자이로 각속도 10 Hz 저역통과 (D 는 측정값에)
%     선택: 출력 노치 N(s) = (s² + 2·0.15·ωr s + ωr²)/(s² + 2·0.7·ωr s + ωr²)
%   이득 값은 analysis/pid_design.py 에서 "K 0.12–0.21 × 공진 ±10% 에서 최대 감도 Ms ≤ 1.6" 으로 정한 것.
%   구현 지연: ZOH 반 샘플(5 ms) 포함.

if nargin < 1, Kset = [0.12 0.15 0.21]; end
T = 0.328; L = 0.016; wr = 21.8; zr = 0.123; Ts = 0.01; tauD = 1 / (2 * pi * 10);
s = tf('s');
P = @(k) k * exp(-L * s) / (T * s + 1) * wr ^ 2 / (s ^ 2 + 2 * zr * wr * s + wr ^ 2);
zoh = exp(-s * Ts / 2);
N = (s ^ 2 + 2 * 0.15 * wr * s + wr ^ 2) / (s ^ 2 + 2 * 0.7 * wr * s + wr ^ 2);
one = tf(1);

%            이름            Kp     Ki      Kd     노치
D = { 'P (목표 4)',          3.70,  0,      0,     one
      'PI',                  2.76,  8.40,   0,     one
      'PI + 노치 (권장)',    7.85,  23.93,  0,     N
      'PID + 노치',          7.89,  24.05,  0.129, N
      'Z-N PID',             6.89,  46.13,  0.258, one
      'Z-N PID + 노치',      6.89,  46.13,  0.258, N };

% Z-N 한계이득 (공칭 K = 0.15): 위상 -180° 주파수의 이득
[~, ~, Wcg] = margin(P(0.15) * zoh);
Ku = 1 / abs(freqresp(P(0.15) * zoh, Wcg)); Tu = 2 * pi / Wcg;
fprintf('Z-N 한계이득 (모델, K=0.15): Ku = %.2f PWM/deg, Tu = %.3f s (%.2f Hz) — 3.5 Hz 구조 공진에서 결정됨\n\n', Ku, Tu, 1 / Tu);

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
