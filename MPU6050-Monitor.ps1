param([string]$Port = 'COM4', [switch]$SelfTest, [string]$SnapshotPath = '')
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
[System.Windows.Forms.Application]::EnableVisualStyles()
$culture = [System.Globalization.CultureInfo]::InvariantCulture
$script:serial = $null
$script:writer = $null
$script:ready = $false
$script:buffer = ''
$script:samples = 0
$script:lastSample = [datetime]::MinValue
$script:openedAt = [datetime]::MinValue
$script:firstSample = [datetime]::MinValue
$script:address = ''
$script:identity = ''
$script:latest = $null
$script:testFinished = $false
$script:csvPath = ''
$script:header = 'ms,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c'
[void][System.IO.Directory]::CreateDirectory((Join-Path $PSScriptRoot 'logs'))

$form = New-Object System.Windows.Forms.Form
$form.Text = 'MPU6050 · I²C 센서 모니터'
$form.ClientSize = New-Object System.Drawing.Size(690, 480)
$form.FormBorderStyle = 'FixedSingle'
$form.MaximizeBox = $false
$form.StartPosition = 'CenterScreen'
$form.AutoScaleMode = 'Dpi'
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
$null = Label 'MPU6050 SENSOR' 25 20 630 38 20
$null = Label 'I²C / 가속도 ±2 g / 자이로 ±250 °/s / 보정 전 측정값' 28 63 630 28
$ports = New-Object System.Windows.Forms.ComboBox
$ports.SetBounds(28, 108, 140, 30)
$ports.DropDownStyle = 'DropDownList'
$form.Controls.Add($ports)
$refresh = Button '새로고침' 180 106 100 34
$connect = Button '연결' 293 106 120 34
$status = Label '연결 대기' 28 153 630 43
$status.ForeColor = [System.Drawing.Color]::FromArgb(25, 91, 185)

$grid = New-Object System.Windows.Forms.DataGridView
$grid.SetBounds(28, 207, 632, 115)
$grid.ReadOnly = $true
$grid.AllowUserToAddRows = $false
$grid.AllowUserToDeleteRows = $false
$grid.AllowUserToResizeRows = $false
$grid.AllowUserToResizeColumns = $false
$grid.RowHeadersVisible = $false
$grid.ScrollBars = 'None'
$grid.AutoSizeColumnsMode = 'Fill'
$grid.ColumnHeadersHeightSizeMode = 'DisableResizing'
$grid.ColumnHeadersHeight = 34
$grid.RowTemplate.Height = 38
$grid.BackgroundColor = [System.Drawing.Color]::White
$grid.BorderStyle = 'None'
$grid.DefaultCellStyle.Alignment = 'MiddleRight'
$grid.ColumnHeadersDefaultCellStyle.Alignment = 'MiddleCenter'
$grid.DefaultCellStyle.Font = New-Object System.Drawing.Font('맑은 고딕', 13)
foreach ($name in @('측정값', 'X', 'Y', 'Z')) { [void]$grid.Columns.Add($name, $name) }
$grid.Columns[0].FillWeight = 145
$grid.Columns[0].DefaultCellStyle.Alignment = 'MiddleLeft'
[void]$grid.Rows.Add('가속도 (g)', '—', '—', '—')
[void]$grid.Rows.Add('각속도 (°/s)', '—', '—', '—')
$grid.ClearSelection()
$form.Controls.Add($grid)
$temperature = Label '센서 내부 온도: — °C' 28 345 320 32 14
$norm = Label '|가속도|: — g' 370 345 290 32 14
$counter = Label '샘플: 0' 28 392 632 25
$fileLabel = Label '연결 시 logs 폴더에 원시값과 변환값을 CSV로 저장합니다.' 28 429 632 36 9

function Update-Controls {
    $connected = $null -ne $script:serial
    $connect.Text = if ($connected) { '연결 해제' } else { '연결' }
    $ports.Enabled = -not $connected
    $refresh.Enabled = -not $connected
}
function Refresh-Ports {
    $selected = if ($ports.SelectedItem) { [string]$ports.SelectedItem } else { $Port }
    $ports.Items.Clear()
    foreach ($name in ([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object)) { [void]$ports.Items.Add($name) }
    if ($ports.Items.Contains($selected)) { $ports.SelectedItem = $selected }
    elseif ($ports.Items.Count) { $ports.SelectedIndex = 0 }
}
function Disconnect-Sensor([string]$message = '연결 해제됨') {
    if ($script:writer) { try { $script:writer.Dispose() } catch { }; $script:writer = $null }
    if ($script:serial) { try { $script:serial.Close(); $script:serial.Dispose() } catch { }; $script:serial = $null }
    $script:ready = $false
    $script:buffer = ''
    $status.Text = $message
    foreach ($row in $grid.Rows) { for ($i = 1; $i -le 3; $i++) { $row.Cells[$i].Value = '—' } }
    $temperature.Text = '센서 내부 온도: — °C'
    $norm.Text = '|가속도|: — g'
    Update-Controls
}
function Connect-Sensor {
    if (-not $ports.SelectedItem) { throw '연결할 COM 포트가 없습니다.' }
    $script:ready = $false
    $script:buffer = ''
    $script:samples = 0
    $script:latest = $null
    $script:address = ''
    $script:identity = ''
    $script:firstSample = [datetime]::MinValue
    $script:lastSample = [datetime]::MinValue
    $script:serial = New-Object System.IO.Ports.SerialPort([string]$ports.SelectedItem, 115200, 'None', 8, 'One')
    $script:serial.DtrEnable = $true
    $script:serial.ReadTimeout = 150
    $script:serial.Open()
    $script:serial.DiscardInBuffer()
    $script:openedAt = [datetime]::Now
    $script:csvPath = Join-Path $PSScriptRoot ('logs\mpu6050-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.csv')
    $script:writer = New-Object System.IO.StreamWriter($script:csvPath, $false, (New-Object System.Text.UTF8Encoding($false)))
    $script:writer.AutoFlush = $true
    $script:writer.WriteLine($script:header)
    $fileLabel.Text = 'CSV 저장 중: ' + [System.IO.Path]::GetFileName($script:csvPath)
    $status.Text = '우노 재시작 및 I²C 센서 확인 중…'
    Update-Controls
}
function Handle-Line([string]$line) {
    if ($line.StartsWith('#')) {
        if ($line.StartsWith('# UNO_MPU6050 BOOT')) {
            $script:ready = $false
            $script:samples = 0
            $script:firstSample = [datetime]::MinValue
        }
        if ($line -match '^# WHO_AM_I address=(0x[0-9A-F]+) id=(0x[0-9A-F]+)') {
            $script:address = $Matches[1]; $script:identity = $Matches[2]
        }
        if ($line.StartsWith('# READY UNO_MPU6050 1.0')) { $script:ready = $true }
        if ($line.StartsWith('# ERROR')) { $status.Text = $line.Substring(2) + ' · 전원과 배선을 확인하세요.' }
        return
    }
    if (-not $script:ready -or $line -notmatch '^\d+,') { return }
    $fields = $line.Split(',')
    if ($fields.Count -ne 15) { return }
    $values = New-Object double[] 15
    for ($i = 0; $i -lt 15; $i++) {
        $parsedValue = 0.0
        if (-not [double]::TryParse($fields[$i], [System.Globalization.NumberStyles]::Float, $culture, [ref]$parsedValue)) { return }
        if ([double]::IsNaN($parsedValue) -or [double]::IsInfinity($parsedValue)) { return }
        $values[$i] = $parsedValue
    }
    # Skip startup transients; the firmware also waits after configuring the sensor.
    if ($values[0] -lt 600) { return }
    $script:latest = $values
    $script:writer.WriteLine($line)
    $script:samples++
    $script:lastSample = [datetime]::Now
    if ($script:firstSample -eq [datetime]::MinValue) { $script:firstSample = $script:lastSample }
    for ($i = 0; $i -lt 3; $i++) {
        $grid.Rows[0].Cells[$i + 1].Value = $values[$i + 8].ToString('F3', $culture)
        $grid.Rows[1].Cells[$i + 1].Value = $values[$i + 11].ToString('F3', $culture)
    }
    $temperature.Text = '센서 내부 온도: ' + $values[14].ToString('F2', $culture) + ' °C'
    $accelerationNorm = [math]::Sqrt($values[8] * $values[8] + $values[9] * $values[9] + $values[10] * $values[10])
    $norm.Text = '|가속도|: ' + $accelerationNorm.ToString('F3', $culture) + ' g'
    $status.Text = "$($script:serial.PortName) · 주소 $($script:address) · WHO_AM_I $($script:identity) · 수신 중"
    $counter.Text = "샘플: $($script:samples)    우노 시간: $($fields[0]) ms    출력 주기: 약 20 Hz"
}
$refresh.Add_Click({ Refresh-Ports })
$connect.Add_Click({
    try { if ($script:serial) { Disconnect-Sensor } else { Connect-Sensor } }
    catch { Disconnect-Sensor ('연결 오류: ' + $_.Exception.Message) }
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
        if ($script:samples -gt 0 -and ([datetime]::Now - $script:lastSample).TotalSeconds -gt 2) {
            $status.Text = '센서 데이터 수신이 멈췄습니다. 전원·배선을 확인하세요.'
        } elseif ($script:samples -eq 0 -and ([datetime]::Now - $script:openedAt).TotalSeconds -gt 10 -and -not $status.Text.Contains('ERROR')) {
            $status.Text = '센서 데이터가 없습니다. MPU6050 펌웨어 및 I²C 연결을 확인하세요.'
        }
        if ($SelfTest -and -not $script:testFinished) {
            if ($script:samples -ge 60) {
                $script:testFinished = $true
                if ($SnapshotPath) {
                    $form.Refresh()
                    $bitmap = New-Object System.Drawing.Bitmap($form.Width, $form.Height)
                    try {
                        $form.DrawToBitmap($bitmap, (New-Object System.Drawing.Rectangle(0, 0, $form.Width, $form.Height)))
                        $bitmap.Save($SnapshotPath, [System.Drawing.Imaging.ImageFormat]::Png)
                    } finally { $bitmap.Dispose() }
                }
                @{ Result = 'PASS'; Samples = $script:samples; Address = $script:address; Identity = $script:identity; Latest = $script:latest; CsvPath = $script:csvPath; Visible = $form.Visible } |
                    ConvertTo-Json | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'logs\mpu6050-monitor-test.json') -Encoding UTF8
                $form.Close()
            } elseif (([datetime]::Now - $script:openedAt).TotalSeconds -gt 15) { throw 'MPU6050 monitor self-test timed out.' }
        }
    } catch {
        $errorMessage = $_.Exception.Message
        Disconnect-Sensor ('수신 오류: ' + $errorMessage)
        if ($SelfTest) {
            @{ Result = 'FAIL'; Error = $errorMessage } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'logs\mpu6050-monitor-test.json') -Encoding UTF8
            $form.Close()
        }
    }
})
$form.Add_Shown({
    try { Connect-Sensor }
    catch {
        Disconnect-Sensor ('연결 오류: ' + $_.Exception.Message)
        if ($SelfTest) {
            @{ Result = 'FAIL'; Error = $_.Exception.Message } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'logs\mpu6050-monitor-test.json') -Encoding UTF8
            $form.Close()
        }
    }
})
$form.Add_FormClosing({ $timer.Stop(); Disconnect-Sensor })
Refresh-Ports
Update-Controls
$timer.Start()
[void]$form.ShowDialog()
$timer.Dispose()
$form.Dispose()
