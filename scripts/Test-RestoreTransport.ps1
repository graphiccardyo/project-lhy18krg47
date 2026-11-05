param([switch]$Paced, [switch]$CheckRecovery, [int]$Attempts = 16, [string]$LogName = 'restore-transport-test.txt')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$culture = [Globalization.CultureInfo]::InvariantCulture
$saved = Get-Content (Join-Path $projectRoot 'mpu6050-calibration.json') -Raw | ConvertFrom-Json
$parts = @('RESTORE', [string][int]$saved.CalibrationId)
foreach ($offset in $saved.Offsets) { $parts += ([double]$offset).ToString('R', $culture) }
$restore = ($parts -join ' ') + "`n"
$serial = New-Object System.IO.Ports.SerialPort('COM4',115200,'None',8,'One')
$serial.DtrEnable = $true
$serial.NewLine = "`n"
$serial.WriteTimeout = 500
$log = New-Object System.Collections.Generic.List[string]
$script:ready = $false
function Read-For([int]$milliseconds) {
    $watch = [Diagnostics.Stopwatch]::StartNew()
    $received = ''
    while ($watch.ElapsedMilliseconds -lt $milliseconds) { $received += $serial.ReadExisting(); Start-Sleep -Milliseconds 1 }
    foreach ($line in $received.Split("`n")) { if ($line.StartsWith('#')) { $log.Add($line.TrimEnd("`r")) } }
    return $received
}
$acks = 0; $failures = 0
try {
    $serial.Open()
    $boot = Read-For 2600
    if ($boot -notmatch '# READY UNO_MPU6050 3\.0') { throw 'Sensor firmware not ready.' }
    $log.Add("RESTORE length including newline=$($restore.Length), paced=$Paced")
    for ($i = 0; $i -lt $Attempts; $i++) {
        $serial.DiscardInBuffer()
        $serial.WriteLine('STATUS')
        $null = Read-For (30 + ($i % 5) * 7)
        $serial.DiscardInBuffer()
        $log.Add("Attempt $($i + 1)")
        if ($Paced) {
            for ($start = 0; $start -lt $restore.Length; $start += 24) {
                $serial.Write($restore.Substring($start, [math]::Min(24,$restore.Length-$start)))
                if ($start + 24 -lt $restore.Length) { Start-Sleep -Milliseconds 20 }
            }
        } else { $serial.Write($restore) }
        $response = Read-For 160
        if ($response -match '# CAL RESTORED') { $acks++ }
        else {
            $failures++
            # A read-only short command exposes a truncated/unterminated RESTORE.
            $serial.WriteLine('STATUS')
            $null = Read-For 350
        }
    }
    $log.Add("RESULT attempts=$Attempts acknowledgements=$acks missing_ack=$failures")
    if ($CheckRecovery) {
        $serial.DiscardInBuffer()
        $serial.Write('RESTORE 3 0')
        $response = Read-For 550
        if ($response -notmatch '# ERROR COMMAND_TIMEOUT') { throw 'Partial command was not discarded.' }
        $serial.WriteLine('STATUS')
        $response = Read-For 200
        if ($response -notmatch ('# OFFSETS cal_id=' + $saved.CalibrationId + ',')) { throw 'Next command was consumed by partial command.' }
        $log.Add('PASS partial command timeout and next-command recovery; no CAL sent')
    }
} finally {
    $serial.Close(); $serial.Dispose()
    $log | Set-Content -LiteralPath (Join-Path $projectRoot ('logs\' + $LogName)) -Encoding UTF8
}
$log
if ($Paced -and $failures -gt 0) { exit 1 }
