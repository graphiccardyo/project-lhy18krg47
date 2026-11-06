function T = aero_static_map(file, settleFrac)
%AERO_STATIC_MAP  시험 1(STAIR) CSV에서 PWM–정상상태 각도 특성을 구한다.
%   T = aero_static_map('aero_STAIR_....csv')        % 각 계단 후반 50% 평균
%   T = aero_static_map('aero_STAIR_....csv', 0.6)   % 앞 60% 버림
%
%   정적 모델: m g l sin(theta) = L * Thrust(pwm)  →  sin(theta) ≈ k (pwm − pwm_dz)
%   (각도 2° 이상인 점으로 선형회귀). 운전점 부근 국소 이득 dθ/dpwm도 계산한다.

if nargin < 2, settleFrac = 0.5; end
d = aero_load(file);
steps = unique(d.phase(d.phase >= 1))';
pwm = []; th = []; sd = []; n = [];
for s = steps
    idx = find(d.phase == s);
    idx = idx(floor(numel(idx) * settleFrac) + 1:end);
    if numel(idx) < 3, continue; end
    pwm(end+1, 1) = mean(d.pwm(idx));   %#ok<AGROW>
    th(end+1, 1) = mean(d.theta(idx));  %#ok<AGROW>
    sd(end+1, 1) = std(d.theta(idx));   %#ok<AGROW>
    n(end+1, 1) = numel(idx);           %#ok<AGROW>
end
slope = gradient(th) ./ gradient(pwm);
T = table(pwm, th, sd, n, slope, 'VariableNames', {'pwm', 'theta_ss_deg', 'std_deg', 'n', 'dtheta_dpwm'});
disp(T);

use = th > 2 & th < 89;
figure('Name', 'Aeropendulum 정적 특성');
errorbar(pwm, th, sd, 'o-', 'LineWidth', 1.2); hold on; grid on;
xlabel('입력 PWM (0–255)'); ylabel('\theta_{ss} [deg]');
title('시험 1: PWM – 정상상태 각도');
if nnz(use) >= 2
    p = polyfit(pwm(use), sind(th(use)), 1);
    k = p(1); dz = -p(2) / p(1);
    pp = linspace(min(pwm), max(pwm), 200);
    s = min(max(k * (pp - dz), 0), 1);
    plot(pp, asind(s), '--', 'LineWidth', 1.2);
    legend('측정 (±1σ)', sprintf('sin\\theta = %.5f (pwm − %.1f)', k, dz), 'Location', 'northwest');
    fprintf('sin(theta) ≈ %.6f * (pwm - %.2f)   → 사각지대 ≈ PWM %.1f\n', k, dz, dz);
    fprintf('운전점 PWM0에서 선형화 이득: dθ/dpwm = (180/pi) * k / cos(θ0)  [deg/PWM]\n');
end
end
