param(
    [string]$Port = 'COM4',
    # Off by default: any PWM above 0 spins the motor when its 12V supply is on.
    [switch]$EnergizeMotor
)
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
    return $response
}
function Send-Check([string]$command, [string]$expected) {
    $serial.DiscardInBuffer()
    $log.Add('> ' + $command)
    $serial.WriteLine($command)
    $response = Read-For 220
    if ($response -notmatch $expected) { throw "Unexpected response to '$command': $response" }
    $log.Add('PASS ' + $command)
}
try {
    $serial.Open()
    $boot = Read-For 3200
    if ($boot -notmatch 'READY UNO_DEMO 1\.0') { throw "No boot banner: $boot" }
    $log.Add('PASS boot banner')
    if ($boot -notmatch 'IMU READY') { throw "IMU not ready: $boot" }
    $log.Add('PASS IMU ready')

    Send-Check 'HELLO' 'READY UNO_DEMO 1\.0[\s\S]*STATE pct=0 pwm=0 dir=F d7=0 d8=0 reason=CONNECTED'
    Send-Check 'PING' 'PONG'
    Send-Check 'SET 0 F' 'STATE pct=0 pwm=0 dir=F d7=0 d8=0 reason=SET'
    Send-Check 'SET 101 F' 'ERR BAD_SET'
    Send-Check 'SET -1 F' 'ERR BAD_PERCENT'
    Send-Check 'SET 50 F extra' 'ERR BAD_SET'
    Send-Check 'NOPE' 'ERR UNKNOWN_COMMAND'
    Send-Check ('X' * 120) 'ERR INVALID_LINE'
    $null = Read-For 300

    $restore = 'RESTORE 3 -0.017059 -0.022036 0.147525 0.92519 -1.014962 -0.145191'
    $serial.DiscardInBuffer()
    for ($i = 0; $i -lt $restore.Length; $i += 32) {
        $serial.Write($restore.Substring($i, [math]::Min(32, $restore.Length - $i)))
        Start-Sleep -Milliseconds 15
    }
    $serial.Write("`n")
    $response = Read-For 300
    if ($response -notmatch 'CAL RESTORED id=3') { throw "Restore failed: $response" }
    $log.Add('PASS chunked RESTORE')

    $serial.DiscardInBuffer()
    $null = $serial.ReadExisting()
    $clock = [System.Diagnostics.Stopwatch]::StartNew()
    $stream = Read-For 3000
    $seconds = $clock.Elapsed.TotalSeconds
    $samples = @($stream -split "`n" | Where-Object { $_ -match '^S,' -and ($_.Trim() -split ',').Count -eq 14 })
    $rate = $samples.Count / $seconds
    if ($rate -lt 18) { throw "Sample rate too low: $rate Hz" }
    $log.Add(('PASS stream {0} samples, {1:N1} Hz' -f $samples.Count, $rate))
    $fields = $samples[-1].Trim() -split ','
    $invariant = [System.Globalization.CultureInfo]::InvariantCulture
    $values = 2..7 | ForEach-Object { [double]::Parse($fields[$_], $invariant) }
    $accel = [math]::Sqrt($values[0] * $values[0] + $values[1] * $values[1] + $values[2] * $values[2])
    if ($accel -lt 0.8 -or $accel -gt 1.2) { throw "Accel magnitude out of range: $accel g" }
    $log.Add(('PASS accel magnitude {0:N3} g, last line {1}' -f $accel, $samples[-1].Trim()))

    if ($EnergizeMotor) {
        foreach ($pct in @(1, 25, 50, 75, 100)) {
            $pwm = [math]::Floor(($pct * 255 + 50) / 100)
            Send-Check "SET $pct F" "STATE pct=$pct pwm=$pwm dir=F d7=1 d8=0 reason=SET"
        }
        Send-Check 'SET 40 R' 'ERR STOP_BEFORE_REVERSE'
        Send-Check 'SET 25 R' 'STATE pct=25 pwm=64 dir=R d7=0 d8=1 reason=SET'
        Send-Check 'STOP' 'STATE pct=0 pwm=0 dir=R d7=0 d8=0 reason=STOP'
        $serial.DiscardInBuffer()
        $serial.WriteLine('SET 30 F')
        $elapsed = [System.Diagnostics.Stopwatch]::StartNew()
        $response = ''
        while ($elapsed.ElapsedMilliseconds -lt 2600 -and $response -notmatch 'reason=TIMEOUT') {
            $response += $serial.ReadExisting()
            Start-Sleep -Milliseconds 10
        }
        if ($response -notmatch 'STATE pct=0 pwm=0 dir=F d7=0 d8=0 reason=TIMEOUT' -or $elapsed.ElapsedMilliseconds -lt 1400) {
            throw 'Heartbeat timeout failed.'
        }
        $log.Add("PASS link timeout -> 0% at $($elapsed.ElapsedMilliseconds) ms")
        if ($response -notmatch '(?m)^S,') { throw 'IMU stream stopped while the motor was commanded.' }
        $log.Add('PASS IMU stream continues during motor command')
    } else {
        $log.Add('SKIP motor output checks (run with -EnergizeMotor)')
    }
    $log.Add('ALL DEMO FIRMWARE CHECKS PASSED')
} finally {
    try { if ($serial.IsOpen) { $serial.WriteLine('STOP') } } catch { }
    $serial.Close()
    $serial.Dispose()
    $log | Set-Content -LiteralPath (Join-Path $projectRoot 'logs\demo-firmware-test.txt') -Encoding UTF8
}
$log
