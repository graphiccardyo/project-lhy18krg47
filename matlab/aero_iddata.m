function [z, op] = aero_iddata(d, angleSource)
%AERO_IDDATA  aero_load 결과를 운전점 편차 iddata로 변환한다 (입력 = PWM 0..255).
%   [z, op] = aero_iddata(d)            % 출력 = 기록된 θ (Madgwick)
%   [z, op] = aero_iddata(d, 'gyro')    % 출력 = 자이로 적분각 ∫ω dt (선형 드리프트 제거)
%
%   프로펠러 진동으로 가속도가 오염되면 Madgwick θ가 떠내려가므로 'gyro'가 더 나을 수 있다.
%   (2026-11-06 데이터 참고: data/20261106/analysis)
%
%   - 운전점(op.pwm0, op.theta0) = 정착 구간(phase==1) 후반의 평균
%   - 식별 구간 = 가진 직전 1초 ~ 가진/복귀 구간 끝 (상승 과도응답 제외)
%   z.InputData = pwm - pwm0,  z.OutputData = theta - theta0  (소신호 선형 모델용)

if nargin < 2 || isempty(angleSource), angleSource = 'theta'; end
u = d.pwm;
if strcmpi(angleSource, 'gyro')
    y = d.theta(1) + cumsum(d.omega) * d.Ts;
else
    y = d.theta;
end
ph = d.phase;

pre = find(ph == 1);
if numel(pre) >= 4
    preS = pre(1); preE = pre(end);
    s0 = preS + floor((preE - preS + 1) / 2);
    opIdx = s0:preE;
    fitFrom = max(s0, preE + 1 - round(1 / d.Ts));
else
    n0 = max(2, floor(numel(y) * 0.1));
    opIdx = 1:n0;
    fitFrom = 1;
end
act = find(ph >= 1);
if isempty(act), fitTo = numel(y); else, fitTo = act(end); end

op.pwm0 = mean(u(opIdx));
op.theta0 = mean(y(opIdx));
op.range = [fitFrom fitTo];
op.angleSource = angleSource;
idx = fitFrom:fitTo;
yy = y(idx) - op.theta0;
if strcmpi(angleSource, 'gyro')
    tt = (0:numel(idx) - 1)' * d.Ts;
    yy = yy - polyval(polyfit(tt, yy, 1), tt);   % 자이로 적분 드리프트(선형) 제거
end

z = iddata(yy, u(idx) - op.pwm0, d.Ts);
z.InputName = 'pwm';    z.InputUnit = 'count';
z.OutputName = 'theta'; z.OutputUnit = 'deg';
z.TimeUnit = 'seconds';
if isfield(d.meta, 'name'), z.ExperimentName = d.meta.name; end
end
