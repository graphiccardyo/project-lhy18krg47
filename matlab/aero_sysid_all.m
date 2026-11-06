function [G, info] = aero_sysid_all(files, varargin)
%AERO_SYSID_ALL  별도 검증 데이터 없이, 가진 실험 여러 개로 PWM → 각도 전달함수를 추정한다.
%   G = aero_sysid_all({'aero_STEP_....csv', 'aero_CHIRP_....csv'})
%   G = aero_sysid_all(files, 'np', 2, 'nz', 0, 'ioDelay', NaN, 'angle', 'gyro', 'holdout', 0.3)
%
%   - 최종 모델: 모든 실험을 합친 다중 실험 iddata(merge)로 tfest
%   - 검증(별도 데이터 없음):
%       실험이 2개 이상 → 실험 하나씩 빼고 추정해 뺀 실험에 비교 (leave-one-experiment-out)
%       실험이 1개      → 앞 (1-holdout) 비율로 추정, 뒤 holdout 비율로 비교
%     교차검증 적합도가 낮거나 실험마다 크게 다르면, 모델보다 데이터(잡음·비선형)를 먼저 의심할 것.
%
%   옵션: 'np' 극점 수(2), 'nz' 영점 수(0), 'ioDelay' 입력 지연 [s] (0, NaN이면 추정),
%         'angle' 'theta'(기록된 Madgwick θ) | 'gyro'(∫ω, 진동이 심할 때), 'holdout' 0.3

p = inputParser;
p.addParameter('np', 2); p.addParameter('nz', 0); p.addParameter('ioDelay', 0);
p.addParameter('angle', 'theta'); p.addParameter('holdout', 0.3);
p.parse(varargin{:});
o = p.Results;
if ischar(files) || isstring(files), files = cellstr(files); end

zs = cell(1, numel(files)); ops = cell(1, numel(files));
for i = 1:numel(files)
    d = aero_load(files{i});
    if strcmpi(d.type, 'STAIR'), error('%s: STAIR(정적 특성) 데이터는 aero_static_map 으로 분석하세요.', files{i}); end
    [zs{i}, ops{i}] = aero_iddata(d, o.angle);
    fprintf('%-40s %-6s %5d샘플  PWM0=%.1f  θ0=%.2f°\n', files{i}, d.type, size(zs{i}, 1), ops{i}.pwm0, ops{i}.theta0);
end
pw = cellfun(@(s) s.pwm0, ops);
if max(pw) - min(pw) > 5
    warning('실험마다 운전점이 다릅니다 (PWM %s). 비선형성 때문에 합동 추정이 어긋날 수 있습니다.', mat2str(round(pw)));
end

opt = tfestOptions('InitialCondition', 'estimate', 'Display', 'off');
opt.EnforceStability = true;
est = @(z) tfest(z, o.np, o.nz, o.ioDelay, opt);

% ---- 검증 (별도 검증 데이터 없이)
info.cv = struct('estimate', {}, 'validate', {}, 'fit', {});
if numel(zs) >= 2
    fprintf('\n[실험 간 교차검증]\n');
    for i = 1:numel(zs)
        others = zs(setdiff(1:numel(zs), i));
        Gi = est(merge(others{:}));
        [~, f] = compare(zs{i}, Gi);
        info.cv(end+1) = struct('estimate', sprintf('제외 %d', i), 'validate', zs{i}.ExperimentName, 'fit', f);
        fprintf('  %s 제외하고 추정 → %s 에 비교: 적합도 %.1f %%\n', zs{i}.ExperimentName, zs{i}.ExperimentName, f);
    end
else
    n = size(zs{1}, 1); nEst = round(n * (1 - o.holdout));
    ze = zs{1}(1:nEst); zv = zs{1}(nEst+1:end);
    Gi = est(ze);
    [~, f] = compare(zv, Gi);
    info.cv(end+1) = struct('estimate', sprintf('앞 %.0f%%', 100 * (1 - o.holdout)), 'validate', sprintf('뒤 %.0f%%', 100 * o.holdout), 'fit', f);
    fprintf('\n[단일 실험 분할 검증] 앞 %.0f%% 추정 → 뒤 %.0f%% 비교: 적합도 %.1f %%\n', 100 * (1 - o.holdout), 100 * o.holdout, f);
end

% ---- 최종 모델: 전체 합동 추정
if numel(zs) >= 2, zall = merge(zs{:}); else, zall = zs{1}; end
G = est(zall);
G.InputName = 'pwm'; G.OutputName = 'theta';
[~, fitAll] = compare(zall, G);
[wn, zeta] = damp(G);
info.K = dcgain(G); info.wn = wn; info.zeta = zeta; info.fitAll = fitAll; info.ops = ops;

fprintf('\n[최종 모델: 전체 %d개 실험 합동 추정]  G(s) [deg/PWM] =\n', numel(zs)); disp(G);
fprintf('DC 이득 K = %.4f deg/PWM\n', info.K);
for i = 1:numel(wn), fprintf('극점 %d: ωn = %.3f rad/s (%.3f Hz), ζ = %.3f\n', i, wn(i), wn(i) / 2 / pi, zeta(i)); end
fprintf('추정 데이터 적합도: %s %%\n', mat2str(round(fitAll(:)', 1)));
if all([info.cv.fit] < 50)
    fprintf('\n※ 교차검증 적합도가 모두 50%% 미만입니다. 측정 잡음(진동)·비선형이 커서 모델을 신뢰하기 어렵습니다.\n');
end

figure('Name', 'Aeropendulum 시스템 식별 (검증 데이터 없음)');
subplot(2, 1, 1); compare(zall, G); grid on; title('추정 데이터와 모델 비교');
subplot(2, 2, 3); resid(zall, G); title('잔차 분석');
subplot(2, 2, 4); step(G); grid on; title('개루프 스텝 응답 (ΔPWM = 1)'); ylabel('\Delta\theta [deg]');
save('aero_model.mat', 'G', 'info', 'zs');
fprintf('모델을 aero_model.mat 에 저장했습니다.\n');
end
