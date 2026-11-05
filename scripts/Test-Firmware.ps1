param([string]$Port = 'COM4')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$log = New-Object System.Collections.Generic.List[string]
$serial = New-Object System.IO.Ports.SerialPort($Port, 115200, 'None', 8, 'One')
$serial.NewLine = "`n"
$serial.ReadTimeout = 150
$serial.WriteTimeout = 300
$serial.DtrEnable = $true
function Read-For([int]$milliseconds) {
    $clock = [System.Diagnostics.Stopwatch]::StartNew()
    $response = ''
    while ($clock.ElapsedMilliseconds -lt $milliseconds) {
        $response += $serial.ReadExisting()
        Start-Sleep -Milliseconds 10
    }
    if ($response) { $log.Add($response.Trim()) }
    return $response
}
function Send-Check([string]$command, [string]$expected) {
    $serial.DiscardInBuffer()
    $log.Add('> ' + $command)
    $serial.WriteLine($command)
    $response = Read-For 160
    if ($response -notmatch $expected) { throw "Unexpected response to '$command': $response" }
    $log.Add('PASS ' + $command)
}
try {
    $serial.Open()
    $boot = Read-For 2200
    Send-Check 'HELLO' 'READY UNO_L298N_PWM 1\.0'
    foreach ($pct in @(0, 1, 25, 50, 75, 100)) {
        $pwm = [math]::Floor(($pct * 255 + 50) / 100)
        Send-Check "SET $pct F" "STATE pct=$pct pwm=$pwm dir=F reason=SET"
    }
    Send-Check 'SET 40 R' 'ERR STOP_BEFORE_REVERSE'
    Send-Check 'STATUS' 'STATE pct=0 pwm=0'
    Send-Check 'SET 25 R' 'STATE pct=25 pwm=64 dir=R'
    Send-Check 'STOP' 'STATE pct=0 pwm=0'
    Send-Check 'SET 101 F' 'ERR BAD_SET'
    Send-Check 'SET -1 F' 'ERR BAD_PERCENT'
    Send-Check 'SET 50 F extra' 'ERR BAD_SET'
    Send-Check ('X' * 70) 'ERR INVALID_LINE'
    Send-Check 'SET 10 F' 'STATE pct=10 pwm=26'
    for ($i = 0; $i -lt 7; $i++) { Send-Check 'PING' 'PONG'; Start-Sleep -Milliseconds 160 }
    Send-Check 'STATUS' 'STATE pct=10 pwm=26'
    $log.Add('PASS heartbeat sustains output beyond 1.5 seconds')
    $serial.DiscardInBuffer()
    $serial.WriteLine('SET 40 F')
    $elapsed = [System.Diagnostics.Stopwatch]::StartNew()
    $response = ''
    while ($elapsed.ElapsedMilliseconds -lt 2400 -and $response -notmatch 'reason=TIMEOUT') {
        $response += $serial.ReadExisting()
        Start-Sleep -Milliseconds 10
    }
    if ($response -notmatch 'STATE pct=0 pwm=0 dir=F reason=TIMEOUT' -or $elapsed.ElapsedMilliseconds -lt 1400) {
        throw "Heartbeat timeout failed: $response"
    }
    $log.Add($response.Trim())
    $log.Add("PASS link timeout -> 0% at $($elapsed.ElapsedMilliseconds) ms")
    Send-Check 'SET 30 F' 'STATE pct=30 pwm=77'
    $serial.Write('SET 99')
    $response = Read-For 1750
    if ($response -notmatch 'STATE pct=0 pwm=0 dir=F reason=TIMEOUT') { throw 'Partial-line timeout failed.' }
    $serial.WriteLine('')
    $null = Read-For 160
    Send-Check 'STOP' 'STATE pct=0 pwm=0'
    $log.Add('PASS partial command does not bypass link timeout')
    $log.Add('ALL FIRMWARE CHECKS PASSED (no motor connected)')
} finally {
    try { if ($serial.IsOpen) { $serial.WriteLine('STOP') } } catch { }
    $serial.Close()
    $serial.Dispose()
    $log | Set-Content -LiteralPath (Join-Path $projectRoot 'logs\firmware-test.txt') -Encoding UTF8
}
$log
