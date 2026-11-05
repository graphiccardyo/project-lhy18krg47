param([string]$Port = 'COM4', [switch]$SmokeTest, [switch]$ConnectOnStart, [string]$SnapshotPath = '')
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()

$script:serial = $null
$script:ready = $false
$script:running = $false
$script:receiveBuffer = ''
$script:pendingChange = $false
$script:syncing = $false
$script:connectedAt = [datetime]::MinValue
$script:lastReceive = [datetime]::MinValue
$script:lastPing = [datetime]::MinValue
$script:lastHello = [datetime]::MinValue
$script:lastPins = [datetime]::MinValue
$script:lastEdit = [datetime]::MinValue
$script:confirmedPercent = 0
$script:confirmedPwm = 0
$script:confirmedDirection = 'F'
$script:pin7 = -1
$script:pin8 = -1
$script:stateReason = 'DISCONNECTED'
$script:snapshotSaved = $false
$script:tracePath = $null
try {
    $traceDirectory = Join-Path $PSScriptRoot 'logs'
    [void][System.IO.Directory]::CreateDirectory($traceDirectory)
    $script:tracePath = Join-Path $traceDirectory ('controller-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.log')
} catch { }
function Write-Trace([string]$message) {
    if ($script:tracePath) {
        try { Add-Content -LiteralPath $script:tracePath -Value ((Get-Date -Format 'HH:mm:ss.fff') + ' ' + $message) -Encoding UTF8 } catch { }
    }
}

$form = New-Object System.Windows.Forms.Form
$form.Text = 'Arduino Uno · L298N 모터 제어'
$form.ClientSize = New-Object System.Drawing.Size(650, 630)
$form.MinimumSize = New-Object System.Drawing.Size(666, 669)
$form.StartPosition = 'CenterScreen'
$form.Font = New-Object System.Drawing.Font('맑은 고딕', 10)
$form.BackColor = [System.Drawing.Color]::FromArgb(246, 248, 251)
$form.AutoScaleMode = 'Dpi'
$form.KeyPreview = $true

function Add-Label($text, $x, $y, $w, $h, $size = 10) {
    $control = New-Object System.Windows.Forms.Label
    $control.Text = $text
    $control.SetBounds($x, $y, $w, $h)
    $control.Font = New-Object System.Drawing.Font('맑은 고딕', $size)
    $form.Controls.Add($control)
    return $control
}
function Add-Button($text, $x, $y, $w, $h) {
    $control = New-Object System.Windows.Forms.Button
    $control.Text = $text
    $control.SetBounds($x, $y, $w, $h)
    $control.FlatStyle = 'Flat'
    $control.BackColor = [System.Drawing.Color]::White
    $form.Controls.Add($control)
    return $control
}
$null = Add-Label 'DC MOTOR CONTROL' 28 22 590 34 18
$null = Add-Label 'Arduino Uno  /  L298N 채널 A  /  USB' 30 62 590 25
$null = Add-Label '포트' 30 110 46 26
$ports = New-Object System.Windows.Forms.ComboBox
$ports.SetBounds(80, 106, 145, 30)
$ports.DropDownStyle = 'DropDownList'
$form.Controls.Add($ports)
$refresh = Add-Button '새로고침' 235 105 100 34
$connect = Add-Button '연결' 345 105 120 34
$status = Add-Label '연결 안 됨' 30 151 590 28
$status.ForeColor = [System.Drawing.Color]::DimGray

$null = Add-Label '우노가 보고한 PWM' 30 197 255 25
$actual = Add-Label '—' 25 224 220 76 40
$actual.ForeColor = [System.Drawing.Color]::FromArgb(25, 91, 185)
$raw = Add-Label '우노 연결을 기다리는 중' 285 246 320 27
$reason = Add-Label '출력 미확인' 285 275 320 25
$null = Add-Label '설정 듀티비' 30 318 180 26
$numeric = New-Object System.Windows.Forms.NumericUpDown
$numeric.SetBounds(463, 313, 100, 34)
$numeric.Minimum = 0
$numeric.Maximum = 100
$numeric.DecimalPlaces = 0
$numeric.Font = New-Object System.Drawing.Font('맑은 고딕', 13)
$form.Controls.Add($numeric)
$null = Add-Label '%' 575 318 40 27 13
$slider = New-Object System.Windows.Forms.TrackBar
$slider.SetBounds(25, 355, 590, 50)
$slider.Minimum = 0
$slider.Maximum = 100
$slider.TickFrequency = 10
$slider.LargeChange = 10
$slider.SmallChange = 1
$form.Controls.Add($slider)

$null = Add-Label '회전 방향' 30 416 110 27
$direction = New-Object System.Windows.Forms.ComboBox
$direction.SetBounds(145, 411, 220, 34)
$direction.DropDownStyle = 'DropDownList'
$null = $direction.Items.Add('정회전 (F)')
$null = $direction.Items.Add('역회전 (R)')
$direction.SelectedIndex = 0
$form.Controls.Add($direction)
$start = Add-Button '운전 시작' 30 465 275 55
$start.BackColor = [System.Drawing.Color]::FromArgb(25, 91, 185)
$start.ForeColor = [System.Drawing.Color]::White
$stop = Add-Button '정지 · 0%  [Space]' 325 465 290 55
$stop.BackColor = [System.Drawing.Color]::FromArgb(180, 42, 48)
$stop.ForeColor = [System.Drawing.Color]::White
$null = Add-Label "운전 중 슬라이더 변경은 바로 적용됩니다. 방향 변경 시 정지합니다.`r`n창 종료·연결 해제 시 정지 / 통신 두절 시 우노가 약 1.5초 후 출력 차단" 30 537 590 51 9
$null = Add-Label 'D5 → ENA (점퍼 제거)    D7 → IN1    D8 → IN2    GND 공통' 30 594 595 24 9

function Update-Controls {
    $start.Enabled = $script:ready -and -not $script:running
    $stop.Enabled = $script:ready
    $ports.Enabled = $null -eq $script:serial
    $refresh.Enabled = $null -eq $script:serial
    $connect.Text = if ($null -eq $script:serial) { '연결' } else { '연결 해제' }
}
function Refresh-Ports {
    $selected = if ($ports.SelectedItem) { [string]$ports.SelectedItem } else { $Port }
    $ports.Items.Clear()
    foreach ($name in ([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)) {
        $null = $ports.Items.Add($name)
    }
    if ($ports.Items.Contains($selected)) { $ports.SelectedItem = $selected }
    elseif ($ports.Items.Count -gt 0) { $ports.SelectedIndex = 0 }
}
function Send-Line([string]$line) {
    if ($null -eq $script:serial -or -not $script:serial.IsOpen) { throw 'USB 연결이 없습니다.' }
    $script:serial.WriteLine($line)
    if ($line -ne 'PING') { Write-Trace ('TX ' + $line) }
}
function Disconnect-Port([string]$message = '연결 해제됨 · 출력 정지') {
    if ($null -ne $script:serial) {
        try { if ($script:serial.IsOpen) { $script:serial.WriteLine('STOP') } } catch { }
        try { $script:serial.Close(); $script:serial.Dispose() } catch { }
    }
    $script:serial = $null
    $script:ready = $false
    $script:running = $false
    $script:pendingChange = $false
    $script:receiveBuffer = ''
    $script:pin7 = -1
    $script:pin8 = -1
    $status.Text = $message
    $status.ForeColor = [System.Drawing.Color]::DimGray
    $actual.Text = '—'
    $raw.Text = '우노 출력 확인 불가'
    $reason.Text = '통신 두절 시 우노 자동 정지'
    Update-Controls
}
function Stop-Output {
    $script:running = $false
    $script:pendingChange = $false
    if ($script:ready) { Send-Line 'STOP' }
    $script:syncing = $true
    $slider.Value = 0
    $numeric.Value = 0
    $script:syncing = $false
    Update-Controls
}
function Send-Setpoint {
    $dir = if ($direction.SelectedIndex -eq 0) { 'F' } else { 'R' }
    Send-Line ('SET {0} {1}' -f [int]$numeric.Value, $dir)
    $script:pendingChange = $false
}
function Handle-Line([string]$line) {
    if ($line -ne 'PONG') { Write-Trace ('RX ' + $line) }
    if ($line -eq 'READY UNO_L298N_PWM 1.0') {
        if (-not $script:ready) {
            $script:ready = $true
            $script:running = $false
            $script:pendingChange = $false
            $status.Text = "$($script:serial.PortName) 연결됨 · 운전 시작을 누르면 출력됩니다."
            $status.ForeColor = [System.Drawing.Color]::FromArgb(25, 115, 65)
            Send-Line 'STOP'
            Update-Controls
        }
    } elseif ($line -match '^STATE pct=(\d+) pwm=(\d+) dir=([FR]) reason=(\w+)$') {
        $script:confirmedPercent = [int]$Matches[1]
        $script:confirmedPwm = [int]$Matches[2]
        $script:confirmedDirection = $Matches[3]
        $script:stateReason = $Matches[4]
        $actual.Text = "$($Matches[1])%"
        $raw.Text = "8비트 출력: $($Matches[2]) / 255"
        $reason.Text = if ([int]$Matches[1] -eq 0) { '정지 · 출력 0%' } else { "운전 중 · $($Matches[3])" }
        if ($Matches[4] -in @('TIMEOUT', 'ERROR', 'BOOT', 'CONNECTED', 'STOP')) {
            $script:running = $false
            $script:pendingChange = $false
            Update-Controls
        }
        if ($script:stateReason -eq 'TIMEOUT') {
            $status.Text = '통신 지연으로 우노가 자동 정지했습니다. 운전 시작을 다시 누르세요.'
        }
    } elseif ($line -match '^PINS d7=([01]) d8=([01]) out7=([01]) out8=([01]) mode7=([01]) mode8=([01])$') {
        $script:pin7 = [int]$Matches[1]
        $script:pin8 = [int]$Matches[2]
        $raw.Text = "PWM $($script:confirmedPwm)/255 · D7=$($script:pin7) D8=$($script:pin8)"
    } elseif ($line.StartsWith('ERR ')) {
        $script:running = $false
        $script:pendingChange = $false
        $status.Text = "우노 오류 · 출력 정지: $line"
        Update-Controls
    }
}
function Connect-Port {
    if (-not $ports.SelectedItem) { throw '연결할 COM 포트를 선택하세요.' }
    $script:serial = New-Object System.IO.Ports.SerialPort([string]$ports.SelectedItem, 115200, 'None', 8, 'One')
    $script:serial.NewLine = "`n"
    $script:serial.ReadTimeout = 150
    $script:serial.WriteTimeout = 150
    $script:serial.DtrEnable = $true
    $script:serial.RtsEnable = $false
    $script:ready = $false
    $script:running = $false
    $script:receiveBuffer = ''
    $script:serial.Open()
    $script:connectedAt = [datetime]::Now
    $script:lastReceive = [datetime]::Now
    $script:lastHello = [datetime]::MinValue
    $script:lastPing = [datetime]::MinValue
    $script:lastPins = [datetime]::MinValue
    $script:pin7 = -1
    $script:pin8 = -1
    $status.Text = '우노 재시작 및 프로그램 확인 중…'
    Update-Controls
}
function Show-Error($errorRecord) {
    $message = $errorRecord.Exception.Message
    Disconnect-Port "연결 오류: $message"
}
$refresh.Add_Click({ Refresh-Ports })
$connect.Add_Click({
    try {
        if ($null -ne $script:serial) { Disconnect-Port } else { Connect-Port }
    } catch { Show-Error $_ }
})
$slider.Add_ValueChanged({
    if ($script:syncing) { return }
    $script:syncing = $true
    $numeric.Value = $slider.Value
    $script:syncing = $false
    $script:pendingChange = $script:running
    $script:lastEdit = [datetime]::Now
})
$numeric.Add_ValueChanged({
    if ($script:syncing) { return }
    $script:syncing = $true
    $slider.Value = [int]$numeric.Value
    $script:syncing = $false
    $script:pendingChange = $script:running
    $script:lastEdit = [datetime]::Now
})
$direction.Add_SelectedIndexChanged({
    try { Stop-Output } catch { Show-Error $_ }
})
$start.Add_Click({
    try {
        if (-not $script:ready) { return }
        $script:running = $true
        Send-Setpoint
        Update-Controls
    } catch { Show-Error $_ }
})
$stop.Add_Click({ try { Stop-Output } catch { Show-Error $_ } })
$form.Add_KeyDown({
    if ($_.KeyCode -eq 'Space' -or $_.KeyCode -eq 'Escape') {
        $_.SuppressKeyPress = $true
        try { Stop-Output } catch { Show-Error $_ }
    }
})
$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 50
$timer.Add_Tick({
    if ($null -eq $script:serial) { return }
    try {
        $now = [datetime]::Now
        $chunk = $script:serial.ReadExisting()
        if ($chunk.Length -gt 0) { $script:receiveBuffer += $chunk }
        while (($index = $script:receiveBuffer.IndexOf("`n")) -ge 0) {
            $line = $script:receiveBuffer.Substring(0, $index).TrimEnd("`r")
            $script:receiveBuffer = $script:receiveBuffer.Substring($index + 1)
            # Acknowledgements must be from this firmware, not arbitrary serial noise.
            if ($line -eq 'PONG' -or $line -eq 'READY UNO_L298N_PWM 1.0' -or $line.StartsWith('STATE ') -or $line.StartsWith('PINS ') -or $line.StartsWith('ERR ')) {
                $script:lastReceive = $now
                Handle-Line $line
            }
        }
        if ($script:receiveBuffer.Length -gt 512) { throw '잘못된 시리얼 응답입니다.' }
        if (-not $script:ready) {
            if (($now - $script:connectedAt).TotalSeconds -gt 8) { throw '모터 제어 프로그램 응답이 없습니다. 우노에 펌웨어를 업로드하세요.' }
            if (($now - $script:connectedAt).TotalSeconds -gt 2 -and ($now - $script:lastHello).TotalMilliseconds -gt 500) {
                Send-Line 'HELLO'
                $script:lastHello = $now
            }
        } else {
            if (($now - $script:lastReceive).TotalMilliseconds -gt 1800) { throw '우노 응답이 끊겼습니다. 자동 정지를 기다린 뒤 다시 연결하세요.' }
            if (($now - $script:lastPing).TotalMilliseconds -gt 350) {
                Send-Line 'PING'
                $script:lastPing = $now
            }
            if (($now - $script:lastPins).TotalMilliseconds -gt 1000) {
                Send-Line 'PINS'
                $script:lastPins = $now
            }
            if ($script:running -and $script:pendingChange -and ($now - $script:lastEdit).TotalMilliseconds -ge 80) { Send-Setpoint }
        }
        if ($SnapshotPath -and -not $script:snapshotSaved -and $script:ready -and $script:stateReason -eq 'STOP') {
            $form.Refresh()
            $bitmap = New-Object System.Drawing.Bitmap($form.Width, $form.Height)
            try {
                $form.DrawToBitmap($bitmap, (New-Object System.Drawing.Rectangle(0, 0, $form.Width, $form.Height)))
                $bitmap.Save($SnapshotPath, [System.Drawing.Imaging.ImageFormat]::Png)
            } finally { $bitmap.Dispose() }
            @{ Visible = $form.Visible; Ready = $script:ready; Percent = $script:confirmedPercent; Status = $status.Text; ProcessId = $PID } |
                ConvertTo-Json | Set-Content -LiteralPath ($SnapshotPath + '.json') -Encoding UTF8
            $script:snapshotSaved = $true
        }
    } catch { Show-Error $_ }
})
$form.Add_FormClosing({
    $timer.Stop()
    Disconnect-Port
})
if ($ConnectOnStart -and -not $SmokeTest) {
    $form.Add_Shown({ try { Connect-Port } catch { Show-Error $_ } })
}
Refresh-Ports
Update-Controls
$timer.Start()
if ($SmokeTest) {
    # Exercise actual WinForms event handlers against the connected, unwired Uno.
    $script:smokeStep = 0
    $script:smokeTicks = 0
    $script:smokeResults = New-Object System.Collections.Generic.List[string]
    $smokeTimer = New-Object System.Windows.Forms.Timer
    $smokeTimer.Interval = 600
    $smokeTimer.Add_Tick({
        try {
            $script:smokeTicks++
            if ($script:smokeTicks -gt 30) { throw 'GUI smoke test timed out.' }
            switch ($script:smokeStep) {
                0 { $connect.PerformClick(); $script:smokeStep = 1 }
                1 { if ($script:ready) { $numeric.Value = 37; $start.PerformClick(); $script:smokeStep = 2 } }
                2 {
                    if ($script:confirmedPercent -ne 37 -or $script:confirmedPwm -ne 94) { throw '37% start acknowledgement missing.' }
                    if ($script:confirmedDirection -ne 'F' -or $script:pin7 -ne 1 -or $script:pin8 -ne 0) { throw 'Forward pin readback mismatch.' }
                    $script:smokeResults.Add('PASS connect and start 37% -> PWM 94, F, D7=1 D8=0')
                    $slider.Value = 63; $script:smokeStep = 3
                }
                3 {
                    if ($script:confirmedPercent -ne 63 -or $script:confirmedPwm -ne 161) { throw 'Live slider acknowledgement missing.' }
                    $script:smokeResults.Add('PASS live slider 63% -> PWM 161')
                    $direction.SelectedIndex = 1; $script:smokeStep = 4
                }
                4 {
                    if ($script:confirmedPercent -ne 0 -or $script:running -or $numeric.Value -ne 0) { throw 'Direction change did not stop.' }
                    $script:smokeResults.Add('PASS direction change stops and resets target')
                    $numeric.Value = 22; $start.PerformClick(); $script:smokeStep = 5
                }
                5 {
                    if ($script:confirmedPercent -ne 22) { throw 'Reverse start acknowledgement missing.' }
                    if ($script:confirmedDirection -ne 'R' -or $script:pin7 -ne 0 -or $script:pin8 -ne 1) { throw 'Reverse pin readback mismatch.' }
                    $script:smokeResults.Add('PASS reverse 22% -> R, D7=0 D8=1')
                    $stop.PerformClick(); $script:smokeStep = 6
                }
                6 {
                    if ($script:confirmedPercent -ne 0 -or $script:running) { throw 'Stop acknowledgement missing.' }
                    $script:smokeResults.Add('PASS reverse start and STOP -> 0%')
                    $connect.PerformClick()
                    if ($null -ne $script:serial -or $script:ready) { throw 'Disconnect failed.' }
                    $script:smokeResults.Add('PASS disconnect')
                    $script:smokeResults | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'logs\gui-smoke.txt') -Encoding UTF8
                    $smokeTimer.Stop(); $form.Close()
                }
            }
        } catch {
            @('FAIL', $_.Exception.Message) | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'logs\gui-smoke.txt') -Encoding UTF8
            $smokeTimer.Stop(); $form.Close()
        }
    })
    $smokeTimer.Start()
}
[void]$form.ShowDialog()
$timer.Dispose()
if ($SmokeTest) { $smokeTimer.Dispose() }
$form.Dispose()
