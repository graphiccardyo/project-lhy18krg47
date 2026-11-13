param(
    [string]$Port = 'COM4'
)
# aeropendulum_lab 펌웨어 자동 시험.
# 모터는 돌리지 않습니다: 출력이 필요한 시험은 모두 BENCH 1(드라이런, 모터 핀 LOW)에서 수행합니다.
# IMU가 없거나 연결이 잘못돼도 실행할 수 있습니다 (IMU 관련 항목은 상태만 보고).
# 실험실 페이지·시리얼 모니터가 포트를 쓰고 있으면 닫으세요. 시작할 때 EEPROM 설정을 백업(CFG BACKUP)하고 끝나면 복원합니다.
$ErrorActionPreference = 'Stop'
$script:p = $null
$script:buf = ''
$script:lines = New-Object System.Collections.Generic.List[string]
$script:results = New-Object System.Collections.Generic.List[object]

function Open-Board {
    $script:p = New-Object System.IO.Ports.SerialPort($Port, 250000, 'None', 8, 'One')
    $script:p.NewLine = "`n"
    $script:p.DtrEnable = $true          # 포트를 열면 우노가 리셋됨
    $script:p.Open()
    $script:p.DtrEnable = $false; Start-Sleep -Milliseconds 100; $script:p.DtrEnable = $true   # 확실히 리셋
    $script:buf = ''
    $script:lines.Clear()
    Pump 3200 -NoPing
}
function Close-Board { if ($script:p -and $script:p.IsOpen) { $script:p.WriteLine('STOP'); Start-Sleep -Milliseconds 50; $script:p.Close() } }

function Pump([int]$ms, [switch]$NoPing) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    $lastPing = 0
    while ($sw.ElapsedMilliseconds -lt $ms) {
        $script:buf += $script:p.ReadExisting()
        $i = $script:buf.IndexOf("`n")
        while ($i -ge 0) {
            $script:lines.Add($script:buf.Substring(0, $i).TrimEnd("`r"))
            $script:buf = $script:buf.Substring($i + 1)
            $i = $script:buf.IndexOf("`n")
        }
        if (-not $NoPing -and $sw.ElapsedMilliseconds - $lastPing -ge 250) { $script:p.WriteLine('PING'); $lastPing = $sw.ElapsedMilliseconds }
        Start-Sleep -Milliseconds 5
    }
}
function Send([string]$c, [int]$ms = 300, [switch]$NoPing) {
    $script:lines.Clear()
    $script:p.WriteLine($c)
    Pump $ms -NoPing:$NoPing
    return ,@($script:lines.ToArray())
}
function Text($arr) { ($arr | Where-Object { $_ -notmatch '^[DAS],' }) -join ' | ' }
function DLines($arr) { @($arr | Where-Object { $_ -match '^D,\d' } | ForEach-Object { $f = $_.Split(','); [pscustomobject]@{ t = [int64]$f[1]; u = [int]$f[2]; th = [int]$f[3]; ph = [int]$f[6] } }) }
function Check([string]$name, [bool]$ok, [string]$detail = '') {
    $script:results.Add([pscustomobject]@{ Result = $(if ($ok) { 'PASS' } else { 'FAIL' }); Test = $name; Detail = $detail })
    $c = if ($ok) { 'Green' } else { 'Red' }
    Write-Host ("{0}  {1}  {2}" -f $(if ($ok) { 'PASS' } else { 'FAIL' }), $name, $detail) -ForegroundColor $c
}
function Between($arr, [string]$startPat, [string]$endPat) {
    $out = New-Object System.Collections.Generic.List[string]; $on = $false
    foreach ($l in $arr) {
        if ($l -match $startPat) { $on = $true; continue }
        if ($on -and $l -match $endPat) { break }
        if ($on) { $out.Add($l) }
    }
    return ,@($out.ToArray())
}

try {
    Open-Board
    $boot = @($script:lines.ToArray())
    Check '부팅 배너 2.1 (리셋 후)' ([bool]($boot -match 'READY AEROPEND_LAB 2\.1')) ''
    $imuLine = ($boot | Where-Object { $_ -match '^IMU ' } | Select-Object -First 1)
    $imuOk = [bool]($imuLine -match 'IMU READY')
    Write-Host "IMU 상태: $imuLine" -ForegroundColor Yellow

    $r = Send 'HELLO' 400
    Check 'HELLO 응답 (READY/CFG/CALVAL)' ((Text $r) -match 'READY AEROPEND_LAB.*CFG axis=.*CALVAL') (Text $r)
    $script:userCfg = (Text $r)
    Write-Host "시험 전 설정 (끝나면 복원): $(($r | Where-Object { $_ -match '^(CFG|CALVAL) ' }) -join ' / ')" -ForegroundColor Yellow
    $r = Send 'CFG BACKUP' 300
    Check '설정 백업 (CFG BACKUP)' ((Text $r) -match 'CFG BACKUP OK') (Text $r)
    $script:backedUp = (Text $r) -match 'CFG BACKUP OK'
    Check '센서 범위 ±16 g / ±2000 dps' ($script:userCfg -match 'arng=16 grng=2000') ''
    Send 'FACTORY' 400 | Out-Null

    # ---- 명령 오류 처리
    $cases = @(
        @('NOPE', 'ERR UNKNOWN_COMMAND'), @('LIM 300 9000', 'ERR BAD_ARGS'), @('AXIS Q +', 'ERR BAD_ARGS'),
        @('STREAM 0', 'ERR BAD_ARGS'), @('BETA 0', 'ERR BAD_ARGS'), @('DLPF 7', 'ERR BAD_ARGS'),
        @('RUN WAVE 1 2 3', 'ERR (UNKNOWN_PROFILE|IMU_NOT_READY)'), @('BENCH 2', 'ERR BAD_ARGS'), @(('X' * 100), 'ERR INVALID_LINE')
    )
    foreach ($c in $cases) { $r = Send $c[0] 250; $label = if ($c[0].Length -gt 20) { 'X*100 (너무 긴 줄)' } else { $c[0] }; Check "오류: $label" ((Text $r) -match $c[1]) (Text $r) }
    $script:lines.Clear(); $script:p.Write('GE'); Pump 500 -NoPing      # PING 이 'GE' 뒤에 붙으면 'GEPING' 이 되므로 보내지 않음
    Check '잘린 명령 250ms 후 폐기' ((Text $script:lines) -match 'ERR COMMAND_TIMEOUT') (Text $script:lines)

    if (-not $imuOk) {
        foreach ($c in 'U 50', 'RUN STEP 0 50 100 100 100', 'CAL', 'CAL FLAT', 'ZERO') {
            $r = Send $c 250
            Check "IMU 없음 → 거부: $c" ((Text $r) -match 'ERR IMU_NOT_READY') (Text $r)
        }
    }

    # ---- 가속도 오프셋 (기본 = 저장값 -17.1/-22.0/+147.5 mg)
    $def = 'CALVAL 0 0\.0000 0\.0000 0\.0000 -0\.0171 -0\.0220 0\.1475'
    $r = Send 'GET' 300
    Check '가속도 오프셋 기본값 = 저장값' ((Text $r) -match $def) (Text $r)
    $r = Send 'ACCOFF 600 0 0' 250;  Check 'ACCOFF 범위 초과 거부' ((Text $r) -match 'ERR BAD_ARGS') (Text $r)
    $r = Send 'ACCOFF 10 -20 150' 300; Check 'ACCOFF 직접 지정 (mg)' ((Text $r) -match 'CALVAL 0 \S+ \S+ \S+ 0\.0100 -0\.0200 0\.1500') (Text $r)
    $r = Send 'CAL CLEAR' 300;       Check 'CAL CLEAR는 가속도 오프셋 유지' ((Text $r) -match 'CALVAL 0 0\.0000 0\.0000 0\.0000 0\.0100 -0\.0200 0\.1500') (Text $r)

    # ---- EEPROM 저장 (재부팅 후 유지)
    Send 'AXIS Y -' 300 | Out-Null; Send 'BETA 80' 300 | Out-Null; Send 'DLPF 5' 300 | Out-Null
    Close-Board; Open-Board
    $cfg = ($script:lines | Where-Object { $_ -match '^CFG ' } | Select-Object -First 1)
    $cv = ($script:lines | Where-Object { $_ -match '^CALVAL ' } | Select-Object -First 1)
    Check 'EEPROM: 재부팅 후 축/β/DLPF 유지' ($cfg -match 'axis=Y sign=- beta=80 dlpf=5') $cfg
    Check 'EEPROM: 재부팅 후 가속도 오프셋 유지' ($cv -match ' 0\.0100 -0\.0200 0\.1500') $cv
    Check '재부팅 시 벤치 모드 해제' ($cfg -match 'bench=0') ''
    $r = Send 'FACTORY' 400
    Check 'FACTORY 초기화 (설정, β=0.02 DLPF=4)' ((Text $r) -match 'axis=X sign=\+ beta=20 dlpf=4') (Text $r)
    Check 'FACTORY 초기화 (가속도 오프셋 = 저장값)' ((Text $r) -match $def) ''
    $r = Send 'ACCOFF 1 2 3' 300; $r = Send 'ACCOFF DEFAULT' 300
    Check 'ACCOFF DEFAULT' ((Text $r) -match ' -0\.0171 -0\.0220 0\.1475') (Text $r)

    # ---- 벤치 모드 & 스트리밍 주기
    $r = Send 'BENCH 1' 300
    Check 'BENCH 1' ((Text $r) -match 'bench=1') (Text $r)
    foreach ($div in 1, 2, 4) {
        Send "STREAM $div" 200 | Out-Null
        $script:lines.Clear(); Pump 2000
        $d = DLines $script:lines
        $ts = 5 * $div
        $dts = for ($i = 1; $i -lt $d.Count; $i++) { $d[$i].t - $d[$i - 1].t }
        $bad = @($dts | Where-Object { [math]::Abs($_ - $ts) -gt 1 }).Count
        $exp = 2000 / $ts
        Check "스트림 Ts=$ts ms" (($d.Count -ge $exp * 0.95) -and ($bad -le 2)) ("{0}줄/2s (기대 {1}), 간격 벗어남 {2}" -f $d.Count, $exp, $bad)
    }
    Send 'STREAM 2' 200 | Out-Null
    $st = ($script:lines | Where-Object { $_ -match '^S,' } | Select-Object -Last 1)
    if (-not $st) { $script:lines.Clear(); Pump 1100; $st = ($script:lines | Where-Object { $_ -match '^S,' } | Select-Object -Last 1) }
    Write-Host "상태줄: $st (state,imu,cal,zero,maxTick_us,overruns,i2cErr,gate%,sat)" -ForegroundColor Yellow
    Check '상태줄 10항목 (게이트·포화 포함)' (($st -split ',').Count -eq 10) $st

    # ---- 수동 출력 / 한계 / 역추력
    function LastU($arr) { $d = DLines $arr; if ($d.Count) { $d[-1].u } else { $null } }
    $r = Send 'U 120' 200;                         Check '수동 U 120' ((LastU $r) -eq 120) "u=$(LastU $r)"
    Send 'LIM 100 12000 0' 200 | Out-Null; $r = Send 'U 200' 200; Check '최대 PWM 100으로 제한' ((LastU $r) -eq 100) "u=$(LastU $r)"
    $r = Send 'U -50' 200;                         Check '역추력 금지 시 음수 → 0' ((LastU $r) -eq 0) "u=$(LastU $r)"
    Send 'LIM 255 12000 1' 200 | Out-Null; $r = Send 'U -50' 200; Check '역추력 허용 시 -50' ((LastU $r) -eq -50) "u=$(LastU $r)"
    Send 'U 0' 200 | Out-Null; Send 'LIM 255 12000 0' 200 | Out-Null

    # ---- 통신 두절 감시
    $script:p.WriteLine('U 80'); $script:lines.Clear(); Pump 1700 -NoPing
    $tx = Text $script:lines; $after = DLines (Between $script:lines 'EVT ABORT LINK_TIMEOUT' 'NEVER')
    Check '통신 1초 두절 → 정지' (($tx -match 'EVT ABORT LINK_TIMEOUT') -and $after.Count -gt 0 -and @($after | Where-Object { $_.u -ne 0 }).Count -eq 0) $tx

    # ---- STEP 프로파일 (pre 300, step 500, post 200 ms → 30/50/20 샘플)
    $r = Send 'RUN STEP 100 140 300 500 200' 1500
    $seg = DLines (Between $r '^EVT RUN STEP' '^EVT DONE')
    $p1 = @($seg | Where-Object { $_.ph -eq 1 -and $_.u -eq 100 }).Count
    $p2 = @($seg | Where-Object { $_.ph -eq 2 -and $_.u -eq 140 }).Count
    $p3 = @($seg | Where-Object { $_.ph -eq 3 -and $_.u -eq 100 }).Count
    Check 'STEP: total=100 알림' ((Text $r) -match 'EVT RUN STEP total=100 ts_ms=10') ''
    Check 'STEP: 구간별 샘플 30/50/20' ($p1 -eq 30 -and $p2 -eq 50 -and $p3 -eq 20 -and $seg.Count -eq 100) "pre $p1, step $p2, post $p3, 합계 $($seg.Count)"
    $post = DLines (Between $r '^EVT DONE' 'NEVER')
    Check 'STEP: 종료 후 PWM 0' ((Text $r) -match 'EVT DONE' -and $post.Count -gt 0 -and @($post | Where-Object { $_.u -ne 0 }).Count -eq 0) ''

    # ---- 진행 중 BUSY / STOP
    Send 'RUN STEP 100 140 2000 2000 2000' 200 | Out-Null
    $r = Send 'RUN STEP 1 2 100 100 100' 200; Check '시험 중 RUN → BUSY' ((Text $r) -match 'ERR BUSY') (Text $r)
    $r = Send 'U 50' 200;                     Check '시험 중 U → BUSY' ((Text $r) -match 'ERR BUSY') (Text $r)
    $r = Send 'STREAM 1' 200;                 Check '시험 중 STREAM → BUSY' ((Text $r) -match 'ERR BUSY') (Text $r)
    $r = Send 'STOP' 300
    $after = DLines (Between $r '^EVT STOP' 'NEVER')
    Check 'STOP → 즉시 PWM 0' ((Text $r) -match 'EVT STOP' -and $after.Count -gt 0 -and @($after | Where-Object { $_.u -ne 0 }).Count -eq 0) (Text $r)

    # ---- STAIR (50→80, 10씩, 200 ms)
    $r = Send 'RUN STAIR 50 80 10 200' 1200
    $seg = DLines (Between $r '^EVT RUN STAIR' '^EVT DONE')
    $okStair = $seg.Count -eq 80
    foreach ($k in 0..3) { $n = @($seg | Where-Object { $_.ph -eq ($k + 1) -and $_.u -eq (50 + 10 * $k) }).Count; if ($n -ne 20) { $okStair = $false } }
    Check 'STAIR: 50/60/70/80 각 20샘플' $okStair "합계 $($seg.Count)"

    # ---- CHIRP (f0=f1=1 Hz, ±30, 2 s) → 사인 2주기
    $r = Send 'RUN CHIRP 120 30 1000 1000 0 2000 0' 2600
    $seg = DLines (Between $r '^EVT RUN CHIRP' '^EVT DONE')
    $u = $seg | ForEach-Object { $_.u }
    $mx = ($u | Measure-Object -Maximum).Maximum; $mn = ($u | Measure-Object -Minimum).Minimum
    $cross = 0; for ($i = 1; $i -lt $u.Count; $i++) { if (($u[$i - 1] - 120) -lt 0 -and ($u[$i] - 120) -ge 0) { $cross++ } }
    Check 'CHIRP: 200샘플, 진폭 90–150, 1 Hz' ($seg.Count -eq 200 -and $mx -eq 150 -and $mn -eq 90 -and $cross -ge 1 -and $cross -le 2) "n=$($seg.Count) min=$mn max=$mx 상승교차=$cross"
    $r = Send 'RUN CHIRP 120 30 100 3000 0 3000 1' 3600
    $seg = DLines (Between $r '^EVT RUN CHIRP' '^EVT DONE')
    $u = $seg | ForEach-Object { $_.u }
    $c1 = 0; $c2 = 0; for ($i = 1; $i -lt $u.Count; $i++) { if (($u[$i - 1] - 120) -lt 0 -and ($u[$i] - 120) -ge 0) { if ($i -lt 150) { $c1++ } else { $c2++ } } }
    Check 'CHIRP 로그 스윕: 후반 주파수가 더 높음' ($seg.Count -eq 300 -and $c2 -gt $c1 * 3) "전반 교차 $c1, 후반 교차 $c2"

    # ---- PRBS (±20, 비트 50 ms = 5샘플)
    $r = Send 'RUN PRBS 120 20 50 0 2000' 2600
    $seg = DLines (Between $r '^EVT RUN PRBS' '^EVT DONE')
    $vals = @($seg | ForEach-Object { $_.u } | Sort-Object -Unique)
    $runsOk = $true; $runLen = 1; $chg = 0
    for ($i = 1; $i -lt $seg.Count; $i++) { if ($seg[$i].u -ne $seg[$i - 1].u) { $chg++; if ($runLen % 5 -ne 0) { $runsOk = $false }; $runLen = 1 } else { $runLen++ } }
    Check 'PRBS: 두 레벨(100/140), 5샘플 배수로 전환' ($seg.Count -eq 200 -and ($vals -join ',') -eq '100,140' -and $runsOk -and $chg -ge 10) "레벨 $($vals -join ','), 전환 ${chg}회"

    # ---- 폐루프 (벤치: 모터 핀 LOW. IMU 가 있으면 실제 θ, 없으면 θ=0 → 기대 출력은 각 샘플의 θ 로 계산)
    function DRef($arr) { @($arr | Where-Object { $_ -match '^D,\d' } | ForEach-Object { $f = $_.Split(','); [pscustomobject]@{ u = [int]$f[2]; th = [int]$f[3] / 100.0; ph = [int]$f[6]; ref = $(if ($f.Count -ge 8) { [int]$f[7] } else { $null }) } }) }
    # P 제어 기대값: u = clamp(ff + Kp(ref − θ), 0, 최대 PWM). θ 는 같은 줄의 값(제어에 쓰인 값과 같은 샘플)
    function PErr($d, $kp, $ff, $umax = 255) { ($d | ForEach-Object { [math]::Abs($_.u - [math]::Min($umax, [math]::Max(0, [math]::Round($ff + $kp * ($_.ref / 100.0 - $_.th))))) } | Measure-Object -Maximum).Maximum }
    # 앞의 FACTORY 시험으로 축·영점이 초기화되어 θ 가 크게(예: −90°) 나오므로, 사용자 설정(축·영점)을 먼저 되돌림 → 매달린 진자 θ ≈ 0
    if ($script:backedUp) { Send 'CFG RESTORE' 500 | Out-Null }
    $r = Send 'PID 2000 0 0 150' 250
    Check 'PID 설정 → PIDCFG' ((Text $r) -match 'PIDCFG kp=2\.000 ki=0\.000 kd=0\.000 ff=150 notch=0') (Text $r)
    $r = Send 'PID -1 0 0 150' 250;   Check 'PID 음수 이득 거부' ((Text $r) -match 'ERR BAD_ARGS') (Text $r)
    $r = Send 'NOTCH 5 150 700' 250;  Check 'NOTCH 범위 밖 거부' ((Text $r) -match 'ERR BAD_ARGS') (Text $r)
    $r = Send 'REF 500' 600
    $d = DRef $r
    $withRef = @($d | Where-Object { $_.ref -eq 500 }).Count
    Check 'REF 5° → EVT REF, D 줄에 기준(8번째 칸)' ((Text $r) -match 'EVT REF 500' -and $withRef -ge 40 -and ($d | Select-Object -Last 1).ref -eq 500) "D $($d.Count)줄 중 기준 포함 $withRef"
    $e1 = PErr ($d | Select-Object -Last 20) 2 150
    Check 'P 제어 출력 = clamp(ff + Kp·(ref−θ)) (±1)' ($e1 -le 1) "최대 오차 $e1, 마지막 u=$(($d | Select-Object -Last 1).u), θ=$(($d | Select-Object -Last 1).th)"
    $script:lines.Clear(); Pump 1200
    $st = ($script:lines | Where-Object { $_ -match '^S,' } | Select-Object -Last 1)
    Check '폐루프 유지 중 상태 4' ($st -match '^S,4,') $st
    $r = Send 'NOTCH 347 150 700' 1200
    $d = DRef $r
    $e2 = PErr ($d | Select-Object -Last 20) 2 150
    Check '노치 켬: 정상상태 출력은 노치 없을 때와 같음 (DC 이득 1, ±1)' ((Text $r) -match 'notch=347 zz=150 zp=700' -and $e2 -le 1) "최대 오차 $e2"
    Send 'NOTCH 0' 200 | Out-Null
    $r = Send 'PID 0 10000 0 150' 1000
    $d = DRef $r
    $lin = @($d | Where-Object { $_.u -lt 254 -and $_.u -gt 1 })      # 포화되지 않은 구간만
    $first = $lin | Select-Object -First 1; $last = $lin | Select-Object -Last 1
    $rate = if ($lin.Count -gt 10) { ($last.u - $first.u) / (($lin.Count - 1) * 0.01) } else { [double]::NaN }; $expRate = 10 * (5 - $last.th)
    Check '적분 동작: Ki=10 → 출력 증가율 ≈ 10·(ref−θ) PWM/s' ([math]::Abs($rate - $expRate) -le 0.15 * [math]::Abs($expRate) + 3) ("증가율 {0:N1} PWM/s (기대 {1:N1})" -f $rate, $expRate)
    $r = Send 'LIM 200 12000 0' 2500
    $d = DRef (Between $r '^CFG ' 'NEVER')                              # LIM 이 적용된 뒤(CFG 응답 이후)만
    Check '출력 포화 = 최대 PWM(200), 적분 정지' (@($d | Where-Object { $_.u -gt 200 }).Count -eq 0 -and ($d | Select-Object -Last 1).u -eq 200) "최대 u=$(($d | Measure-Object -Property u -Maximum).Maximum)"
    $r = Send 'REF 0' 300
    $d = DRef $r
    Check '기준 낮춤 → 와인드업 없이 즉시 내려옴 (조건부 적분)' (($d | Select-Object -Last 1).u -le 200 -and ($d | Select-Object -Last 1).u -ge 150) "u=$(($d | Select-Object -Last 1).u)"
    Send 'LIM 255 12000 0' 200 | Out-Null
    $r = Send 'U 0' 300
    $d = DRef $r
    Check 'U 명령 → 폐루프 해제 (기준 칸 없음)' (($d | Select-Object -Last 5 | Where-Object { $_.ref -ne $null }).Count -eq 0) ''
    Send 'PID 2000 0 0 150' 200 | Out-Null
    $r = Send 'RUN CLSTEP 1000 2000 200 300 100' 1200
    $seg = DRef (Between $r '^EVT RUN CLSTEP' '^EVT DONE')
    $c1 = @($seg | Where-Object { $_.ph -eq 1 -and $_.ref -eq 1000 }).Count
    $c2 = @($seg | Where-Object { $_.ph -eq 2 -and $_.ref -eq 2000 }).Count
    $c3 = @($seg | Where-Object { $_.ph -eq 3 -and $_.ref -eq 1000 }).Count
    $e3 = PErr $seg 2 150
    Check 'CLSTEP: 기준 10°/20°/10° (20/30/10 샘플), u = 150+2(ref−θ) ±1' ($c1 -eq 20 -and $c2 -eq 30 -and $c3 -eq 10 -and $e3 -le 1 -and (Text $r) -match 'EVT DONE') "pre $c1, step $c2, post $c3, 최대 오차 $e3"
    $r = Send 'RUN RELAY 1000 15 10 200 500' 1200
    $seg = DRef (Between $r '^EVT RUN RELAY' '^EVT DONE')
    $pre = @($seg | Where-Object { $_.ph -eq 1 }); $rel = @($seg | Where-Object { $_.ph -eq 2 })
    $center = ($pre | Select-Object -Last 10 | Measure-Object -Property u -Average).Average
    $expRel = [math]::Min(255, $center + 15)
    $okRel = @($rel | Where-Object { [math]::Abs($_.u - $expRel) -le 1.5 }).Count
    Check 'RELAY: 유지 구간 후반 평균 + d(15) (θ < 기준이므로 +쪽 유지)' ($pre.Count -eq 20 -and $rel.Count -eq 50 -and $okRel -eq 50) ("중심 {0:N1}, 릴레이 u={1}" -f $center, (($rel | Select-Object -First 3 | ForEach-Object { $_.u }) -join ','))
    $r = Send 'RUN CLSTEP 1000 1000 0 3000 0' 300                      # 3 s 시험 진행 중에
    $r2 = Send 'REF 500' 200
    Check '시험 중 REF → BUSY' ((Text $r2) -match 'ERR BUSY') (Text $r2)
    Send 'STOP' 300 | Out-Null
    $script:p.WriteLine('REF 500'); $script:lines.Clear(); Pump 1700 -NoPing
    Check '폐루프 유지 중 통신 두절 → 정지' ((Text $script:lines) -match 'EVT ABORT LINK_TIMEOUT') (Text $script:lines)

    # ---- 폐루프 최소 PWM (모터가 멈추지 않도록 하한)
    $r = Send 'PID 5000 0 0 150 80' 250
    Check 'PID 최소 PWM 설정 → PIDCFG umin=80' ((Text $r) -match 'PIDCFG kp=5\.000 .* umin=80') (Text $r)
    $r = Send 'PID 5000 0 0 150 300' 250; Check '최소 PWM 범위 밖 거부' ((Text $r) -match 'ERR BAD_ARGS') (Text $r)
    $r = Send 'REF -3000' 600
    $d = @(DRef $r | Where-Object { $_.ref -eq -3000 })
    Check '기준이 낮아 출력이 음수여야 할 때 → 최소 PWM(80)에서 멈춤' ($d.Count -gt 20 -and @($d | Where-Object { $_.u -lt 80 }).Count -eq 0 -and ($d | Select-Object -Last 1).u -eq 80) "u 최소 $(($d | Measure-Object -Property u -Minimum).Minimum), 마지막 $(($d | Select-Object -Last 1).u)"
    Send 'STOP' 200 | Out-Null
    Send 'PID 0 20000 0 150 80' 200 | Out-Null
    Send 'REF -3000' 1500 | Out-Null                                     # 적분만: 하한에 붙은 뒤 1.2 s 이상 유지
    $r = Send 'REF 500' 300
    $d = @(DRef $r | Where-Object { $_.ref -eq 500 })
    Check '하한에서 적분 정지 → 기준이 다시 오르면 0.3 s 안에 바로 회복' ($d.Count -gt 5 -and ($d | Select-Object -Last 1).u -ge 84) "0.3 s 뒤 u=$(($d | Select-Object -Last 1).u) (와인드업이면 80 에 계속 붙어 있음)"
    $r = Send 'PID 2000 0 0 150' 250
    Check '인자 4개 PID 는 최소 PWM 유지 (umin=80)' ((Text $r) -match 'umin=80') (Text $r)
    Send 'STOP' 200 | Out-Null
    Send 'PID 0 0 0 150 0' 200 | Out-Null

    # ---- 타이밍
    $script:lines.Clear(); Pump 1200
    $st = ($script:lines | Where-Object { $_ -match '^S,' } | Select-Object -Last 1)
    Write-Host "상태줄: $st" -ForegroundColor Yellow

    $r = Send 'BENCH 0' 300
    Check 'BENCH 0' ((Text $r) -match 'bench=0') (Text $r)
}
finally {
    try {
        if ($script:p -and $script:p.IsOpen) {
            $script:p.WriteLine('STOP'); Start-Sleep -Milliseconds 100
            if ($script:backedUp) {
                $r = Send 'CFG RESTORE' 500
                Check '시험 전 설정 복원 (CFG RESTORE)' ((Text $r) -match 'CFG RESTORE OK') (($r | Where-Object { $_ -match '^(CFG|CALVAL) ' }) -join ' / ')
            } else { $script:p.WriteLine('FACTORY'); Start-Sleep -Milliseconds 200 }
        }
    } catch {}
    Close-Board
}
$fail = @($script:results | Where-Object { $_.Result -eq 'FAIL' }).Count
Write-Host ("`n총 {0}개 중 통과 {1}, 실패 {2}" -f $script:results.Count, ($script:results.Count - $fail), $fail) -ForegroundColor $(if ($fail) { 'Red' } else { 'Green' })
if ($fail) { exit 1 }
