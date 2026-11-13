param(
    [string]$Port = 'COM4',
    # 지정하면 컴파일 후 우노에 업로드합니다. 업로드 전 모터 12V 전원을 끄세요.
    [switch]$Upload,
    # arduino-cli.exe 경로 (PATH에 없을 때)
    [string]$Cli = ''
)
# Aeropendulum 실험 펌웨어(firmware/aeropendulum_lab) 컴파일/업로드
# 외부 라이브러리 불필요 (Arduino AVR 코어의 Wire, EEPROM만 사용)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
if (-not $Cli) {
    $cmd = Get-Command arduino-cli -ErrorAction SilentlyContinue
    if ($cmd) { $Cli = $cmd.Source }
    elseif (Test-Path "$root\.tools\arduino-cli.exe") { $Cli = "$root\.tools\arduino-cli.exe" }
    else {
        # Arduino IDE 2 에 들어 있는 arduino-cli (사용자 설치 / 전체 설치)
        $bundled = @("$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
                     "$env:ProgramFiles\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe") |
                   Where-Object { Test-Path $_ } | Select-Object -First 1
        if ($bundled) { $Cli = $bundled }
    }
    if (-not $Cli) { throw 'arduino-cli를 찾을 수 없습니다. -Cli 로 경로를 주거나 Arduino IDE에서 firmware/aeropendulum_lab/aeropendulum_lab.ino 를 여세요.' }
    Write-Host "arduino-cli: $Cli"
}
$sketch = Join-Path $root 'firmware\aeropendulum_lab'
$out = Join-Path $root 'build\aeropendulum_lab'
& $Cli core install arduino:avr | Out-Host
& $Cli compile -b arduino:avr:uno --warnings all --output-dir $out $sketch
if ($LASTEXITCODE -ne 0) { throw '컴파일 실패' }
if ($Upload) {
    Write-Host "업로드: $Port (시리얼 모니터/실험실 페이지가 포트를 쓰고 있으면 닫으세요)"
    & $Cli upload -b arduino:avr:uno -p $Port --input-dir $out $sketch
    if ($LASTEXITCODE -ne 0) { throw '업로드 실패' }
    Write-Host '완료. Start-Lab.cmd 로 실험실 페이지를 여세요.'
}
