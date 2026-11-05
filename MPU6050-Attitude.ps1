param([string]$Port = 'COM4', [switch]$SelfTest, [string]$SnapshotPath = '', [switch]$ExpandDetails)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName System.Windows.Forms.DataVisualization
Add-Type -Path (Join-Path $PSScriptRoot 'MpuAttitudeView.cs') -ReferencedAssemblies @('System.dll', 'System.Windows.Forms.dll', 'System.Drawing.dll')
[System.Windows.Forms.Application]::EnableVisualStyles()
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$script:serial = $null
$script:writer = $null
$script:events = $null
$script:ready = $false
$script:buffer = ''
$script:samples = 0
$script:latest = $null
$script:calibrationId = 0
$script:countdown = $null
$script:calibrating = $false
$script:calAcknowledged = $false
$script:calStarted = [datetime]::MinValue
$script:openedAt = [datetime]::MinValue
$script:lastSample = [datetime]::MinValue
$script:address = ''
$script:identity = ''
$script:csvPath = ''
$script:history = New-Object System.Collections.ArrayList
$script:calCommands = 0
$script:testDone = $false
$script:restoreSent = $false
$script:restorePending = $false
$script:restoreStarted = [datetime]::MinValue
$script:expectedRestoreId = 0
$script:commandErrors = 0
$script:yawZero = 0.0
$script:calibrationPath = Join-Path $PSScriptRoot 'mpu6050-calibration.json'
$script:header = 'ms,cal_id,ax_before_g,ay_before_g,az_before_g,gx_before_dps,gy_before_dps,gz_before_dps,ax_after_g,ay_after_g,az_after_g,gx_after_dps,gy_after_dps,gz_after_dps,temp_c,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,fusion_dt_ms'
[void][System.IO.Directory]::CreateDirectory((Join-Path $PSScriptRoot 'logs'))

$form = New-Object System.Windows.Forms.Form
$form.Text = 'MPU6050 · 3D 자세'
$form.ClientSize = New-Object System.Drawing.Size(900, 630)
$form.FormBorderStyle = 'FixedSingle'
$form.MaximizeBox = $false
$form.StartPosition = 'CenterScreen'
$form.AutoScaleMode = 'Dpi'
$form.AutoScroll = $true
$form.Font = New-Object System.Drawing.Font('맑은 고딕', 10)
$form.BackColor = [System.Drawing.Color]::FromArgb(246, 248, 251)
function Label($text, $x, $y, $w, $h, $size = 10) {
    $item = New-Object System.Windows.Forms.Label
    $item.Text = $text
    $item.SetBounds($x, $y, $w, $h)
    $item.Font = New-Object System.Drawing.Font('맑은 고딕', $size)
    $form.Controls.Add($item)
    return $item
}
function Button($text, $x, $y, $w, $h) {
    $item = New-Object System.Windows.Forms.Button
    $item.Text = $text
    $item.SetBounds($x, $y, $w, $h)
    $item.FlatStyle = 'Flat'
    $item.BackColor = [System.Drawing.Color]::White
    $form.Controls.Add($item)
    return $item
}
$null = Label 'MPU6050 · 3D ATTITUDE' 26 16 840 39 20
$null = Label 'Madgwick 6축 자세 추정 · Roll/Pitch 중력 기준 · Yaw 초기 방향 기준 (드리프트 가능)' 29 62 840 26
$ports = New-Object System.Windows.Forms.ComboBox
$ports.SetBounds(29, 99, 135, 30)
$ports.DropDownStyle = 'DropDownList'
$form.Controls.Add($ports)
$refresh = Button '새로고침' 178 97 100 34
$connect = Button '연결' 292 97 115 34
$status = Label '연결 대기' 29 143 840 29
$status.ForeColor = [System.Drawing.Color]::FromArgb(25, 91, 185)
$levelCheck = New-Object System.Windows.Forms.CheckBox
$levelCheck.Text = '센서를 수평으로 놓고 정지시켰습니다 (+Z 위쪽)'
$levelCheck.SetBounds(29, 178, 600, 28)
$form.Controls.Add($levelCheck)
$calibrate = Button '보정 시작 (3초 준비)' 29 216 250 37
$calibrate.BackColor = [System.Drawing.Color]::FromArgb(25, 91, 185)
$calibrate.ForeColor = [System.Drawing.Color]::White
$cancel = Button '준비 취소' 294 216 145 37
$clear = Button '보정 해제' 454 216 145 37
$calStatus = Label '미보정 · 두 열의 값이 같습니다. 보정은 자동 실행되지 않습니다.' 29 263 840 33

$grid = New-Object System.Windows.Forms.DataGridView
$grid.SetBounds(29, 307, 842, 214)
$grid.ReadOnly = $true
$grid.AllowUserToAddRows = $false
$grid.AllowUserToDeleteRows = $false
$grid.AllowUserToResizeRows = $false
$grid.AllowUserToResizeColumns = $false
$grid.RowHeadersVisible = $false
$grid.ScrollBars = 'None'
$grid.AutoSizeColumnsMode = 'Fill'
$grid.ColumnHeadersHeightSizeMode = 'DisableResizing'
$grid.ColumnHeadersHeight = 32
$grid.RowTemplate.Height = 30
$grid.BackgroundColor = [System.Drawing.Color]::White
$grid.BorderStyle = 'None'
$grid.DefaultCellStyle.Alignment = 'MiddleRight'
$grid.ColumnHeadersDefaultCellStyle.Alignment = 'MiddleCenter'
$grid.DefaultCellStyle.Font = New-Object System.Drawing.Font('맑은 고딕', 12)
foreach ($name in @('측정축 / 단위', '보정 전', '보정 후', '차이 (전 − 후)')) { [void]$grid.Columns.Add($name, $name) }
$grid.Columns[0].FillWeight = 140
$grid.Columns[0].DefaultCellStyle.Alignment = 'MiddleLeft'
$script:channelNames = @('가속도 X (g)', '가속도 Y (g)', '가속도 Z (g)', '각속도 X (°/s)', '각속도 Y (°/s)', '각속도 Z (°/s)')
foreach ($name in $script:channelNames) { [void]$grid.Rows.Add($name, '—', '—', '—') }
for ($i = 0; $i -lt 6; $i++) {
    $grid.Rows[$i].DefaultCellStyle.BackColor = if ($i -lt 3) { [System.Drawing.Color]::White } else { [System.Drawing.Color]::FromArgb(238, 243, 250) }
}
$grid.ClearSelection()
$form.Controls.Add($grid)
$channelLabel = Label '그래프 채널' 29 537 125 26
$channel = New-Object System.Windows.Forms.ComboBox
$channel.SetBounds(155, 532, 210, 30)
$channel.DropDownStyle = 'DropDownList'
foreach ($name in $script:channelNames) { [void]$channel.Items.Add($name) }
$channel.SelectedIndex = 2
$form.Controls.Add($channel)
$temperature = Label '내부 온도: — °C' 402 537 215 26
$norm = Label '|가속도| 전/후: — / — g' 618 537 270 26

$chart = New-Object System.Windows.Forms.DataVisualization.Charting.Chart
$chart.SetBounds(29, 574, 842, 137)
$chart.BackColor = [System.Drawing.Color]::White
$area = New-Object System.Windows.Forms.DataVisualization.Charting.ChartArea('Signals')
$area.AxisX.Title = '우노 시간 (s) · 최근 15초'
$area.AxisX.LabelStyle.Format = '0.0'
$area.AxisX.MajorGrid.LineColor = [System.Drawing.Color]::FromArgb(230, 234, 240)
$area.AxisY.MajorGrid.LineColor = [System.Drawing.Color]::FromArgb(230, 234, 240)
$area.AxisY.IsStartedFromZero = $false
$area.AxisY.LabelStyle.Format = '0.###'
$chart.ChartAreas.Add($area)
$legend = New-Object System.Windows.Forms.DataVisualization.Charting.Legend('Legend')
$legend.Docking = 'Top'
$legend.Alignment = 'Far'
$chart.Legends.Add($legend)
foreach ($name in @('보정 전', '보정 후')) {
    $series = New-Object System.Windows.Forms.DataVisualization.Charting.Series($name)
    $series.ChartType = 'Line'
    $series.BorderWidth = 2
    $series.ChartArea = 'Signals'
    $series.Legend = 'Legend'
    $series.Color = if ($name -eq '보정 전') { [System.Drawing.Color]::FromArgb(190, 98, 41) } else { [System.Drawing.Color]::FromArgb(25, 91, 185) }
    $chart.Series.Add($series)
}
$form.Controls.Add($chart)
$counter = Label '수신 대기 · 가속도 보정 목표 (0, 0, +1) g / 자이로 목표 (0, 0, 0) °/s' 29 722 842 25 9
$fileLabel = Label '오프셋만 보정합니다. 재연결·리셋 시 보정값이 초기화됩니다.' 29 754 842 28 9

# Keep the comparison controls in a collapsed, scrollable lower section.
$details = New-Object System.Windows.Forms.Panel
$details.SetBounds(29, 625, 842, 610)
$details.Visible = $false
$form.Controls.Add($details)
foreach ($item in @($levelCheck, $calibrate, $cancel, $clear, $calStatus, $grid, $channelLabel, $channel, $temperature, $norm, $chart, $counter, $fileLabel)) {
    $newX = $item.Left - 29
    $newY = $item.Top - 178
    $details.Controls.Add($item)
    $item.Location = New-Object System.Drawing.Point($newX, $newY)
}
$view = New-Object MpuAttitudeView
$view.SetBounds(29, 183, 842, 325)
$form.Controls.Add($view)
$orientation = Label 'Roll —°     Pitch —°     Yaw —°' 29 526 842 34 15
$zeroYaw = Button '현재 방향을 Yaw 0°로' 29 573 215 35
$toggleDetails = Button '보정·전후 비교 펼치기 ▾' 259 573 270 35
$mainCalStatus = Label '미보정' 550 579 320 27 9

function Save-FormSnapshot([string]$path) {
    $form.Refresh()
    $bitmap = New-Object System.Drawing.Bitmap($form.Width, $form.Height)
    try {
        $form.DrawToBitmap($bitmap, (New-Object System.Drawing.Rectangle(0, 0, $form.Width, $form.Height)))
        $bitmap.Save($path, [System.Drawing.Imaging.ImageFormat]::Png)
    } finally { $bitmap.Dispose() }
}
function Set-DetailsExpanded([bool]$expanded) {
    $form.AutoScrollPosition = New-Object System.Drawing.Point(0, 0)
    $details.Visible = $expanded
    if ($expanded) {
        $form.ClientSize = New-Object System.Drawing.Size(900, ([math]::Min(900, [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea.Height - 100)))
        $form.AutoScrollMinSize = New-Object System.Drawing.Size(0, 1245)
        $toggleDetails.Text = '보정·전후 비교 접기 ▴'
    } else {
        $form.AutoScrollMinSize = New-Object System.Drawing.Size(0, 0)
        $form.ClientSize = New-Object System.Drawing.Size(900, 630)
        $toggleDetails.Text = '보정·전후 비교 펼치기 ▾'
    }
}
function Restore-SavedCalibration {
    if ($script:restoreSent -or -not (Test-Path -LiteralPath $script:calibrationPath)) { return }
    try {
        $saved = Get-Content -LiteralPath $script:calibrationPath -Raw | ConvertFrom-Json
        if ($saved.Enabled -eq $false -or $saved.Address -ne $script:address -or $saved.Library -ne 'MPU6050_light-1.2.1') { return }
        if ($saved.CalibrationId -lt 1 -or $saved.CalibrationId -gt 65535 -or $saved.Offsets.Count -ne 6) { return }
        $parts = @('RESTORE', [string][int]$saved.CalibrationId)
        for ($i = 0; $i -lt 6; $i++) {
            $value = [double]$saved.Offsets[$i]
            if ([double]::IsNaN($value) -or [double]::IsInfinity($value) -or [math]::Abs($value) -gt $(if ($i -lt 3) { 0.5 } else { 20 })) { return }
            $parts += $value.ToString('R', $culture)
        }
        $script:restoreSent = $true
        $script:restorePending = $true
        $script:restoreStarted = [datetime]::Now
        $script:expectedRestoreId = [int]$saved.CalibrationId
        $calStatus.Text = '저장된 보정값 복원 응답을 기다리는 중…'
        Update-Controls
        Send-Command ($parts -join ' ')
    } catch {
        $script:restorePending = $false
        $calStatus.Text = '저장된 보정값을 불러오지 못했습니다. 필요하면 수동 보정하세요.'
        Update-Controls
    }
}

function Update-Controls {
    $connected = $null -ne $script:serial
    $busy = $script:calibrating -or $script:restorePending -or $null -ne $script:countdown
    $connect.Text = if ($connected) { '연결 해제' } else { '연결' }
    $ports.Enabled = -not $connected
    $refresh.Enabled = -not $connected
    $levelCheck.Enabled = $script:ready -and -not $busy
    $calibrate.Enabled = $script:ready -and $levelCheck.Checked -and -not $busy
    $cancel.Enabled = $null -ne $script:countdown
    $clear.Enabled = $script:ready -and -not $busy -and $script:calibrationId -gt 0
    $zeroYaw.Enabled = $script:ready -and $null -ne $script:latest
}
function Refresh-Ports {
    $selected = if ($ports.SelectedItem) { [string]$ports.SelectedItem } else { $Port }
    $ports.Items.Clear()
    foreach ($name in ([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)) { [void]$ports.Items.Add($name) }
    if ($ports.Items.Contains($selected)) { $ports.SelectedItem = $selected }
    elseif ($ports.Items.Count) { $ports.SelectedIndex = 0 }
}
function Write-Event([string]$line) {
    if ($script:events) { $script:events.WriteLine((Get-Date -Format 'HH:mm:ss.fff') + ' ' + $line) }
}
function Send-Command([string]$line) {
    if (-not $script:serial -or -not $script:serial.IsOpen) { throw 'USB 연결이 없습니다.' }
    if ($line -eq 'CAL') { $script:calCommands++ }
    Write-Event ('TX ' + $line)
    # Uno's default RX ring has 63 usable bytes. A RESTORE line is longer,
    # and float/CSV printing can keep the MCU busy while the bytes arrive.
    # Pace 24-byte chunks so one command cannot overflow that ring.
    $payload = $line + "`n"
    if ($payload.Length -gt 159) { throw '명령이 너무 깁니다.' }
    for ($start = 0; $start -lt $payload.Length; $start += 24) {
        $script:serial.Write($payload.Substring($start, [math]::Min(24, $payload.Length - $start)))
        if ($start + 24 -lt $payload.Length) { [System.Threading.Thread]::Sleep(20) }
    }
}
function Clear-Chart {
    $script:history.Clear()
    foreach ($series in $chart.Series) { $series.Points.Clear() }
}
function Plot-Frame($values) {
    $selected = $channel.SelectedIndex
    $seconds = $values[0] / 1000.0
    [void]$chart.Series[0].Points.AddXY($seconds, $values[2 + $selected])
    [void]$chart.Series[1].Points.AddXY($seconds, $values[8 + $selected])
    foreach ($series in $chart.Series) {
        while ($series.Points.Count -gt 0 -and $series.Points[0].XValue -lt $seconds - 15) { $series.Points.RemoveAt(0) }
    }
    $area.AxisX.Minimum = [math]::Max(0, $seconds - 15)
    $area.AxisX.Maximum = [math]::Max(1, $seconds)
}
function Disconnect-Sensor([string]$message = '연결 해제됨') {
    foreach ($writer in @($script:writer, $script:events)) { if ($writer) { try { $writer.Dispose() } catch { } } }
    $script:writer = $null; $script:events = $null
    if ($script:serial) { try { $script:serial.Close(); $script:serial.Dispose() } catch { }; $script:serial = $null }
    $script:ready = $false
    $script:countdown = $null
    $script:calibrating = $false
    $script:calAcknowledged = $false
    $script:restorePending = $false
    $script:buffer = ''
    $levelCheck.Checked = $false
    $view.SetDisconnected()
    $orientation.Text = 'Roll —°     Pitch —°     Yaw —°'
    $status.Text = $message
    foreach ($row in $grid.Rows) { for ($i = 1; $i -le 3; $i++) { $row.Cells[$i].Value = '—' } }
    $temperature.Text = '내부 온도: — °C'
    $norm.Text = '|가속도| 전/후: — / — g'
    Update-Controls
}
function Connect-Sensor {
    if (-not $ports.SelectedItem) { throw '연결할 COM 포트가 없습니다.' }
    $script:ready = $false; $script:buffer = ''; $script:samples = 0; $script:latest = $null
    $script:calibrationId = 0; $script:calCommands = 0; $script:address = ''; $script:identity = ''
    $script:restoreSent = $false; $script:restorePending = $false; $script:calAcknowledged = $false; $script:yawZero = 0
    $script:commandErrors = 0
    $view.SetYawReference(0)
    $script:lastSample = [datetime]::MinValue
    $levelCheck.Checked = $false
    Clear-Chart
    $script:serial = New-Object System.IO.Ports.SerialPort([string]$ports.SelectedItem, 115200, 'None', 8, 'One')
    $script:serial.DtrEnable = $true
    $script:serial.ReadTimeout = 150
    $script:serial.WriteTimeout = 200
    $script:serial.NewLine = "`n"
    $script:serial.Open()
    $script:serial.DiscardInBuffer()
    $script:openedAt = [datetime]::Now
    $script:csvPath = Join-Path $PSScriptRoot ('logs\mpu6050-attitude-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.csv')
    $encoding = New-Object System.Text.UTF8Encoding($false)
    $script:writer = New-Object System.IO.StreamWriter($script:csvPath, $false, $encoding)
    $script:writer.AutoFlush = $true
    $script:writer.WriteLine($script:header)
    $script:events = New-Object System.IO.StreamWriter(($script:csvPath + '.events.log'), $false, $encoding)
    $script:events.AutoFlush = $true
    $fileLabel.Text = 'CSV: ' + [System.IO.Path]::GetFileName($script:csvPath)
    $status.Text = '우노 재시작 및 라이브러리 연결 확인 중…'
    $calStatus.Text = '미보정 · 보정 시작 버튼을 누르기 전에는 전·후가 같습니다.'
    Update-Controls
}
function Handle-Line([string]$line) {
    if ($line.StartsWith('#')) {
        Write-Event ('RX ' + $line)
        if ($line.StartsWith('# UNO_MPU6050 BOOT')) {
            $script:ready = $false; $script:calibrationId = 0; $script:countdown = $null; $script:calibrating = $false
            $script:restoreSent = $false; $script:yawZero = 0
            $script:restorePending = $false; $script:calAcknowledged = $false
            $view.SetYawReference(0)
            $levelCheck.Checked = $false; Clear-Chart
            $calStatus.Text = '우노 리셋 · 보정값 초기화됨'
        }
        if ($line -match '^# WHO_AM_I address=(0x[0-9A-F]+) id=(0x[0-9A-F]+)') {
            $script:address = $Matches[1]; $script:identity = $Matches[2]
        }
        if ($line.StartsWith('# READY UNO_MPU6050 3.0')) {
            $script:ready = $true
            $script:calibrationId = 0
            $calStatus.Text = '미보정 · 보정 시작 버튼을 누르기 전에는 전·후가 같습니다.'
            Send-Command 'STATUS'
            Restore-SavedCalibration
        }
        if ($line.StartsWith('# CAL BEGIN')) {
            $script:calibrating = $true; $script:calStarted = [datetime]::Now
            $script:calAcknowledged = $true
            $calStatus.Text = '보정 중 · 수평·정지를 유지하세요. 약 3~5초 소요됩니다.'
        } elseif ($line.StartsWith('# CAL DONE')) {
            $script:calibrating = $false; $levelCheck.Checked = $false
            $script:calAcknowledged = $false
            $script:yawZero = 0; $view.SetYawReference(0)
            $calStatus.Text = '보정 완료 · 같은 샘플의 전·후 값과 그래프를 비교하세요.'
        } elseif ($line.StartsWith('# CAL REJECTED')) {
            $script:calibrating = $false; $levelCheck.Checked = $false
            $script:calAcknowledged = $false
            $calStatus.Text = '보정 실패 · 수평·정지 또는 통신 확인 필요. 기존 보정값은 유지됩니다.'
        } elseif ($line.StartsWith('# CAL CLEARED')) {
            $script:calibrationId = 0; $levelCheck.Checked = $false
            $calStatus.Text = '보정 해제 · 전·후 값이 같습니다.'
            $script:yawZero = 0; $view.SetYawReference(0)
            @{ Enabled = $false; Library = 'MPU6050_light-1.2.1'; Address = $script:address } | ConvertTo-Json | Set-Content -LiteralPath $script:calibrationPath -Encoding UTF8
        } elseif ($line.StartsWith('# CAL RESTORED')) {
            $script:restorePending = $false
            $calStatus.Text = '기존에 직접 수행한 보정값을 복원했습니다. 새 보정은 실행하지 않았습니다.'
            $script:yawZero = 0; $view.SetYawReference(0)
        }
        if ($line -match '^# OFFSETS cal_id=([1-9][0-9]*),(.+)$') {
            $offsetId = [int]$Matches[1]
            if ($script:restorePending -and $offsetId -eq $script:expectedRestoreId) {
                $script:restorePending = $false
                $calStatus.Text = '기존 수동 보정값 복원이 확인됐습니다.'
            }
            $offsetValues = @($Matches[2].Split(',') | ForEach-Object { [double]::Parse($_, $culture) })
            if ($offsetValues.Count -eq 6) {
                @{ Enabled = $true; CalibrationId = $offsetId; Offsets = $offsetValues; Address = $script:address; Library = 'MPU6050_light-1.2.1'; SavedAt = (Get-Date -Format o) } |
                    ConvertTo-Json | Set-Content -LiteralPath $script:calibrationPath -Encoding UTF8
            }
        }
        if ($line.StartsWith('# ERROR')) {
            $script:commandErrors++
            $status.Text = $line.Substring(2)
            if ($script:calibrating -or $script:restorePending) {
                $script:calibrating = $false; $script:calAcknowledged = $false; $script:restorePending = $false; $script:countdown = $null
                $levelCheck.Checked = $false
                $calStatus.Text = '명령 처리 오류: ' + $line.Substring(8) + ' · 기존 저장값은 보존됩니다.'
            }
        }
        Update-Controls
        return
    }
    if (-not $script:ready -or $line -notmatch '^\d+,') { return }
    $fields = $line.Split(',')
    if ($fields.Count -ne 23) { return }
    $values = New-Object double[] 23
    for ($i = 0; $i -lt 23; $i++) {
        $parsedValue = 0.0
        if (-not [double]::TryParse($fields[$i], [System.Globalization.NumberStyles]::Float, $culture, [ref]$parsedValue)) { return }
        if ([double]::IsNaN($parsedValue) -or [double]::IsInfinity($parsedValue)) { return }
        $values[$i] = $parsedValue
    }
    $script:calibrationId = [int]$values[1]
    $script:latest = $values
    $script:samples++
    $script:lastSample = [datetime]::Now
    $script:writer.WriteLine($line)
    for ($i = 0; $i -lt 6; $i++) {
        $grid.Rows[$i].Cells[1].Value = $values[2 + $i].ToString('F3', $culture)
        $grid.Rows[$i].Cells[2].Value = $values[8 + $i].ToString('F3', $culture)
        $grid.Rows[$i].Cells[3].Value = ($values[2 + $i] - $values[8 + $i]).ToString('F3', $culture)
    }
    $temperature.Text = '내부 온도: ' + $values[14].ToString('F2', $culture) + ' °C'
    $beforeNorm = [math]::Sqrt($values[2] * $values[2] + $values[3] * $values[3] + $values[4] * $values[4])
    $afterNorm = [math]::Sqrt($values[8] * $values[8] + $values[9] * $values[9] + $values[10] * $values[10])
    $norm.Text = '|가속도| 전/후: ' + $beforeNorm.ToString('F3', $culture) + ' / ' + $afterNorm.ToString('F3', $culture) + ' g'
    $status.Text = "$($script:serial.PortName) · 주소 $($script:address) · Madgwick 쿼터니언 · 자세 수신 중"
    $view.SetOrientation($values[15], $values[16], $values[17], $values[18])
    $relativeYaw = $values[21] - $script:yawZero
    while ($relativeYaw -gt 180) { $relativeYaw -= 360 }
    while ($relativeYaw -lt -180) { $relativeYaw += 360 }
    $orientation.Text = 'Roll ' + $values[19].ToString('F1', $culture) + '°     Pitch ' + $values[20].ToString('F1', $culture) + '°     Yaw ' + $relativeYaw.ToString('F1', $culture) + '°'
    $mainCalStatus.Text = if ($script:calibrationId -gt 0) { '보정 적용됨 · ID ' + $script:calibrationId } else { '미보정 · 펼쳐서 수동 보정 가능' }
    $counter.Text = "샘플: $($script:samples) · 보정 ID: $($script:calibrationId) · 목표: 가속도 (0,0,+1) g / 각속도 (0,0,0) °/s"
    [void]$script:history.Add($values)
    while ($script:history.Count -gt 320) { $script:history.RemoveAt(0) }
    Plot-Frame $values
    Update-Controls
}
$refresh.Add_Click({ Refresh-Ports })
$connect.Add_Click({
    try { if ($script:serial) { Disconnect-Sensor } else { Connect-Sensor } }
    catch { Disconnect-Sensor ('연결 오류: ' + $_.Exception.Message) }
})
$levelCheck.Add_CheckedChanged({ Update-Controls })
$calibrate.Add_Click({
    if (-not $script:ready -or -not $levelCheck.Checked -or $script:calibrating -or $script:restorePending -or $null -ne $script:countdown) { return }
    $script:countdown = [datetime]::Now.AddSeconds(3)
    $calStatus.Text = '3초 후 보정을 시작합니다. 센서에서 손을 떼고 정지시키세요.'
    Update-Controls
})
$cancel.Add_Click({
    $script:countdown = $null
    $calStatus.Text = '보정 준비 취소 · 기존 보정값을 유지합니다.'
    Update-Controls
})
$clear.Add_Click({ try { Send-Command 'CLEAR' } catch { Disconnect-Sensor $_.Exception.Message } })
$toggleDetails.Add_Click({ Set-DetailsExpanded (-not $details.Visible) })
$zeroYaw.Add_Click({
    if ($script:latest) {
        $script:yawZero = $script:latest[21]
        $view.SetYawReference($script:yawZero)
    }
})
$channel.Add_SelectedIndexChanged({
    foreach ($series in $chart.Series) { $series.Points.Clear() }
    foreach ($values in $script:history) { Plot-Frame $values }
})
$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 100
$timer.Add_Tick({
    if (-not $script:serial) { return }
    try {
        $script:buffer += $script:serial.ReadExisting()
        while (($index = $script:buffer.IndexOf("`n")) -ge 0) {
            $line = $script:buffer.Substring(0, $index).TrimEnd("`r")
            $script:buffer = $script:buffer.Substring($index + 1)
            Handle-Line $line
        }
        if ($script:buffer.Length -gt 2048) { throw '시리얼 데이터 형식이 올바르지 않습니다.' }
        if ($null -ne $script:countdown) {
            $remaining = ($script:countdown - [datetime]::Now).TotalSeconds
            if ($remaining -le 0) {
                $script:countdown = $null; $script:calibrating = $true; $script:calStarted = [datetime]::Now
                $script:calAcknowledged = $false
                Send-Command 'CAL'
                $calStatus.Text = '보정 중 · 수평·정지를 유지하세요.'
                Update-Controls
            } else { $calStatus.Text = ([math]::Ceiling($remaining)).ToString() + '초 후 보정 시작 · 센서에서 손을 떼세요.' }
        }
        if ($script:restorePending -and ([datetime]::Now - $script:restoreStarted).TotalSeconds -gt 4) {
            $script:restorePending = $false
            $calStatus.Text = '보정값 복원 응답이 없습니다. 다시 연결하세요. 저장값은 보존됩니다.'
            Write-Event 'ERROR RESTORE_ACK_TIMEOUT'
            Update-Controls
        }
        if ($script:calibrating -and -not $script:calAcknowledged -and ([datetime]::Now - $script:calStarted).TotalSeconds -gt 4) { throw '보정 시작 응답이 없습니다. 명령 전달 상태를 확인하고 다시 연결하세요.' }
        if ($script:calibrating -and $script:calAcknowledged -and ([datetime]::Now - $script:calStarted).TotalSeconds -gt 25) { throw '보정 계산 응답 시간이 초과되었습니다. 다시 연결하세요.' }
        if (-not $script:calibrating -and $script:samples -gt 0 -and ([datetime]::Now - $script:lastSample).TotalSeconds -gt 2) { $status.Text = '센서 데이터 수신이 멈췄습니다. 전원·배선을 확인하세요.' }
        if ($SelfTest -and -not $script:testDone) {
            if ($script:samples -ge 100) {
                if ($script:calCommands -ne 0 -or $calibrate.Enabled) { throw 'Unexpected calibration during attitude test.' }
                if ($script:restorePending -or $script:commandErrors -gt 0) { throw 'Restore acknowledgement missing or command error received.' }
                if ($script:restoreSent -and $script:calibrationId -ne $script:expectedRestoreId) { throw 'Saved calibration was not restored.' }
                $qNorm = [math]::Sqrt($script:latest[15]*$script:latest[15] + $script:latest[16]*$script:latest[16] + $script:latest[17]*$script:latest[17] + $script:latest[18]*$script:latest[18])
                if ([math]::Abs($qNorm - 1) -gt 0.002 -or $view.RenderCount -lt 5) { throw 'Invalid quaternion or 3D rendering missing.' }
                if ($details.Visible) { throw 'Details must be collapsed at startup.' }
                if ($chart.Series[0].Points.Count -lt 90 -or $chart.Series[1].Points.Count -lt 90) { throw 'Plot data missing.' }
                $script:testDone = $true
                if ($SnapshotPath) {
                    Save-FormSnapshot $SnapshotPath
                    Set-DetailsExpanded $true
                    if (-not $details.Visible) { throw 'Details did not expand.' }
                    $form.AutoScrollPosition = New-Object System.Drawing.Point(0, 600)
                    Save-FormSnapshot ([System.IO.Path]::ChangeExtension($SnapshotPath, 'expanded.png'))
                    Set-DetailsExpanded $false
                }
                @{ Result = 'PASS'; Samples = $script:samples; CalibrationCommands = $script:calCommands; CalibrationId = $script:calibrationId; RestorePending = $script:restorePending; CommandErrors = $script:commandErrors; RenderCount = $view.RenderCount; QuaternionNorm = $qNorm; DetailsCollapsed = (-not $details.Visible); Latest = $script:latest; CsvPath = $script:csvPath; Visible = $form.Visible } |
                    ConvertTo-Json | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'logs\mpu6050-attitude-test.json') -Encoding UTF8
                $form.Close()
            } elseif (([datetime]::Now - $script:openedAt).TotalSeconds -gt 15) { throw 'Comparison monitor self-test timed out.' }
        }
    } catch {
        $message = $_.Exception.Message
        Disconnect-Sensor ('오류: ' + $message)
        if ($SelfTest) {
            @{ Result = 'FAIL'; Error = $message } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'logs\mpu6050-attitude-test.json') -Encoding UTF8
            $form.Close()
        }
    }
})
$form.Add_Shown({
    try { Connect-Sensor }
    catch {
        $message = $_.Exception.Message
        Disconnect-Sensor ('연결 오류: ' + $message)
        if ($SelfTest) {
            @{ Result = 'FAIL'; Error = $message } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'logs\mpu6050-attitude-test.json') -Encoding UTF8
            $form.Close()
        }
    }
})
$form.Add_FormClosing({ $timer.Stop(); Disconnect-Sensor })
Refresh-Ports
Update-Controls
if ($ExpandDetails) { Set-DetailsExpanded $true }
$timer.Start()
[void]$form.ShowDialog()
$timer.Dispose()
$form.Dispose()
