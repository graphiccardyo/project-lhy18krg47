function [G, info] = aero_sysid(estFile, valFile, np, nz, ioDelay)
%AERO_SYSID  시험 2 데이터로 PWM → 각도 전달함수를 추정하고 검증한다 (System Identification Toolbox).
%   G = aero_sysid('aero_CHIRP_....csv', 'aero_STEP_....csv')
%   G = aero_sysid(est, val, np, nz)            % 극점/영점 수 (기본 2, 0)
%   G = aero_sysid(est, val, 2, 0, NaN)         % 입력 지연도 추정
%
%   단위: G(s) = Θ(s)/PWM(s)  [deg / PWM count], 운전점 주변 소신호 모델.
%   추정 데이터와 검증 데이터는 서로 다른 실험(예: CHIRP/PRBS로 추정, STEP으로 검증)을 권장.
%
%   System Identification App에서 직접 하려면:
%     de = aero_load(est); [ze, ope] = aero_iddata(de);
%     dv = aero_load(val); [zv, opv] = aero_iddata(dv);
%     systemIdentification      % Import data → Object → ze, zv  /  Estimate → Transfer Function Models

if nargin < 2 || isempty(valFile), valFile = estFile; end
if nargin < 3 || isempty(np), np = 2; end
if nargin < 4 || isempty(nz), nz = 0; end
if nargin < 5, ioDelay = 0; end

de = aero_load(estFile);  [ze, ope] = aero_iddata(de);
dv = aero_load(valFile);  [zv, opv] = aero_iddata(dv);
fprintf('추정: %s  (%d샘플, Ts=%.3fs, PWM0=%.1f, θ0=%.2f°)\n', estFile, size(ze, 1), ze.Ts, ope.pwm0, ope.theta0);
fprintf('검증: %s  (%d샘플, PWM0=%.1f, θ0=%.2f°)\n', valFile, size(zv, 1), opv.pwm0, opv.theta0);
if abs(ope.pwm0 - opv.pwm0) > 5
    warning('추정/검증 데이터의 운전점이 다릅니다 (PWM %.1f vs %.1f). 비선형성 때문에 적합도가 낮을 수 있습니다.', ope.pwm0, opv.pwm0);
end

opt = tfestOptions('InitialCondition', 'estimate', 'Display', 'off');
opt.EnforceStability = true;
G = tfest(ze, np, nz, ioDelay, opt);
G.InputName = 'pwm'; G.OutputName = 'theta';

[~, fitE] = compare(ze, G);
[~, fitV] = compare(zv, G);
info.fitEst = fitE; info.fitVal = fitV;
info.opEst = ope; info.opVal = opv;
[wn, zeta] = damp(G);
info.wn = wn; info.zeta = zeta;
info.K = dcgain(G);

fprintf('\nG(s) [deg/PWM] =\n'); disp(G);
fprintf('DC 이득 K = %.5f deg/PWM\n', info.K);
for i = 1:numel(wn)
    fprintf('극점 %d: ωn = %.4f rad/s (%.3f Hz), ζ = %.4f\n', i, wn(i), wn(i) / 2 / pi, zeta(i));
end
fprintf('적합도: 추정 %.1f %%, 검증 %.1f %%\n', fitE, fitV);

figure('Name', 'Aeropendulum 시스템 식별');
subplot(2, 2, [1 2]); compare(zv, G); title(sprintf('검증 데이터 비교 (fit %.1f%%)', fitV)); grid on;
subplot(2, 2, 3); resid(ze, G); title('잔차 분석 (추정 데이터)');
subplot(2, 2, 4); step(G); grid on; title('개루프 스텝 응답 (ΔPWM = 1)');
ylabel('\Delta\theta [deg]');

save('aero_model.mat', 'G', 'info', 'ze', 'zv');
fprintf('\n모델을 aero_model.mat 에 저장했습니다.\n');
end
