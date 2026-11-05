param(
    [string]$Port = 'COM4',
    # Off by default: RUN switches the bridge fully on and spins the motor when its 12V supply is on.
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
    $response = Read-For 200
    if ($response -notmatch $expected) { throw "Unexpected response to '$command': $response" }
    $log.Add('PASS ' + $command)
}
try {
    $serial.Open()
    $boot = Read-For 2200
    if ($boot -notmatch 'READY UNO_L298N_ONOFF 1\.0') { throw "No boot banner: $boot" }
    $log.Add('PASS boot banner')
    Send-Check 'HELLO' 'READY UNO_L298N_ONOFF 1\.0[\s\S]*STATE run=0 ena=0 d7=0 d8=0 reason=CONNECTED'
    Send-Check 'PING' 'PONG'
    Send-Check 'STATUS' 'STATE run=0 ena=0 d7=0 d8=0 reason=STATUS'
    Send-Check 'RUN X' 'ERR UNKNOWN_COMMAND'
    Send-Check 'RUN F extra' 'ERR UNKNOWN_COMMAND'
    Send-Check 'SET 50 F' 'ERR UNKNOWN_COMMAND'
    Send-Check ('X' * 50) 'ERR INVALID_LINE'
    $null = Read-For 300
    Send-Check 'STOP' 'STATE run=0 ena=0 d7=0 d8=0 reason=STOP'

    if ($EnergizeMotor) {
        Send-Check 'RUN F' 'STATE run=F ena=1 d7=1 d8=0 reason=RUN'
        Send-Check 'RUN R' 'ERR STOP_BEFORE_REVERSE[\s\S]*STATE run=0 ena=0 d7=0 d8=0 reason=ERROR'
        Send-Check 'RUN R' 'STATE run=R ena=1 d7=0 d8=1 reason=RUN'
        Send-Check 'STOP' 'STATE run=0 ena=0 d7=0 d8=0 reason=STOP'
        $serial.DiscardInBuffer()
        $serial.WriteLine('RUN F')
        $elapsed = [System.Diagnostics.Stopwatch]::StartNew()
        $response = ''
        while ($elapsed.ElapsedMilliseconds -lt 2400 -and $response -notmatch 'reason=TIMEOUT') {
            $response += $serial.ReadExisting()
            Start-Sleep -Milliseconds 10
        }
        if ($response -notmatch 'STATE run=0 ena=0 d7=0 d8=0 reason=TIMEOUT' -or $elapsed.ElapsedMilliseconds -lt 1400) {
            throw "Heartbeat timeout failed: $response"
        }
        $log.Add("PASS link timeout -> off at $($elapsed.ElapsedMilliseconds) ms")
    } else {
        $log.Add('SKIP motor output checks (run with -EnergizeMotor)')
    }
    $log.Add('ALL ON/OFF FIRMWARE CHECKS PASSED')
} finally {
    try { if ($serial.IsOpen) { $serial.WriteLine('STOP') } } catch { }
    $serial.Close()
    $serial.Dispose()
    $log | Set-Content -LiteralPath (Join-Path $projectRoot 'logs\onoff-firmware-test.txt') -Encoding UTF8
}
$log
