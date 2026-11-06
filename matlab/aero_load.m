function d = aero_load(file)
%AERO_LOAD  Aeropendulum 실험실(lab/index.html)이 저장한 CSV를 읽는다.
%   d = aero_load('aero_STEP_20261106_130000.csv')
%
%   d.t (s), d.pwm (0..255, 입력), d.duty (%, 참고), d.theta (deg), d.omega (deg/s),
%   d.theta_acc (deg), d.phase, d.ref (폐루프 기준, 없으면 NaN), d.Ts (s), d.meta (헤더 정보), d.type
%
%   phase: 0 대기, 1 정착(pre), 2 가진, 3 복귀(post) / STAIR는 계단 번호(1..)

fid = fopen(file, 'r', 'n', 'UTF-8');
if fid < 0, error('파일을 열 수 없습니다: %s', file); end
first = fgetl(fid);
extra = 0;                       % 추가 주석 줄 (# pid; ...)
ln = fgetl(fid);
while ischar(ln) && startsWith(ln, '#'), extra = extra + 1; ln = fgetl(fid); end
fclose(fid);

meta = struct();
nHeader = 1;
if startsWith(first, '#')
    nHeader = 2;
    parts = strsplit(strtrim(first(2:end)), ';');
    for i = 1:numel(parts)
        kv = strtrim(parts{i});
        k = strfind(kv, '=');
        if isempty(k), continue; end
        key = matlab.lang.makeValidName(strtrim(kv(1:k(1)-1)));
        val = strtrim(kv(k(1)+1:end));
        num = str2double(val);
        if ~isnan(num) && ~any(strcmp(key, {'params', 'name', 'fw', 'start', 'status', 'axis', 'type', 'input'}))
            meta.(key) = num;
        else
            meta.(key) = val;
        end
    end
end

M = readmatrix(file, 'NumHeaderLines', nHeader + extra, 'Delimiter', ',');
d.t = M(:, 1);
d.pwm = M(:, 2);
d.duty = M(:, 3);
d.theta = M(:, 4);
d.omega = M(:, 5);
d.theta_acc = M(:, 6);
d.phase = M(:, 7);
if size(M, 2) >= 8, d.ref = M(:, 8); else, d.ref = nan(size(d.t)); end   % 폐루프 기준 [deg] (CLSTEP/RELAY)
d.meta = meta;
d.type = '';
if isfield(meta, 'type'), d.type = meta.type; end
if isfield(meta, 'Ts')
    d.Ts = meta.Ts;
else
    d.Ts = median(diff(d.t));
end

gaps = find(diff(d.t) > 1.5 * d.Ts);
if ~isempty(gaps)
    warning('aero_load:gaps', '%s: 샘플 누락 의심 %d곳 (최대 간격 %.3f s)', file, numel(gaps), max(diff(d.t)));
end
end
