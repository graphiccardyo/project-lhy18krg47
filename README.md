# Uno + L298N DC 모터 제어

## 1주차 웹 시연

`Start-Demo.cmd`를 더블클릭해 Edge로 `AeropendulumDemo.html`을 열고(Chrome도 가능) **보드 연결** → COM4 선택. 모터 PWM 조절과 MPU6050 3D 자세를 한 화면에서 함께 시연합니다. 서버나 설치는 필요 없고, 인터넷 없이 동작합니다. 기존 GUI나 시리얼 모니터를 먼저 닫으세요.

현재 우노에는 두 기능을 합친 `firmware/uno_demo/uno_demo.ino`가 올라가 있습니다. 아래 기존 모터 조절기나 [MPU6050.md](MPU6050.md)의 센서 프로그램을 다시 쓰려면 해당 스케치를 재업로드해야 합니다.

**확인한 가동 범위**는 슬라이더를 움직이며 직접 찍어 두는 값이며, 브라우저에 저장됩니다. 파형은 설정값으로 그린 도식입니다. 회전수 센서가 없으므로 회전 여부는 눈으로 판단합니다. 저장된 보정값은 페이지의 `CAL` 상수에 복사해 두었으므로, 다시 보정하면 함께 수정하세요.

펌웨어 검사: `scripts/Test-DemoFirmware.ps1`. 기본값은 모터가 돌지 않는 범위만 검사하며, `-EnergizeMotor`는 PWM을 100%까지 출력하므로 12V를 끈 상태에서 실행하세요.

## PWM 없는 On/Off 제어

`Start-OnOff.cmd` → **보드 연결** → 정회전 ON / 역회전 ON / 정지. ENA를 `digitalWrite`로 HIGH/LOW만 하므로 PWM 스위칭 없이 전원 전압이 그대로 걸립니다. L298N 전압강하를 PWM과 분리해 측정할 때 씁니다. 배선은 아래 모터 조절기와 같고, 통신이 1.5초 끊기면 정지합니다.

펌웨어: `firmware/uno_onoff/uno_onoff.ino` · 검사: `scripts/Test-OnOffFirmware.ps1` (`-EnergizeMotor`는 모터를 실제로 켜므로 12V를 끈 상태에서 실행)

## 기존 모터 조절기

`Start-MotorControl.cmd`를 더블클릭 → COM4 연결 → PWM(0~100%) 설정 → **운전 시작**.
운전 중에는 슬라이더/숫자 변경이 적용됩니다. Space/Esc는 정지, 방향 변경은 정지 후 다시 시작합니다.

| 우노 / 전원 | L298N |
|---|---|
| D5 | ENA — ENA 점퍼 제거 |
| D7 | IN1 |
| D8 | IN2 |
| GND, 12V 전원 − | GND (공통) |
| 별도 12V 전원 + | 모터 전원 입력 (+12V / VS) |
| 모터 두 선 | OUT1, OUT2 |
| 우노 5V | **5V-EN 전원 점퍼를 제거한 뒤** 모듈의 5V 로직 전원 입력 |

전원을 끈 상태에서 배선하세요. 위 전원 구성은 5V-EN 점퍼를 제거해 외부 5V 입력을 지원하는 일반 L298N 모듈 기준입니다. ENA와 5V-EN은 서로 다른 점퍼입니다. 리셋 중에도 ENA를 LOW로 유지하려면 ENA–GND에 10kΩ을 연결하세요. 모터 전류는 USB/우노 5V에서 공급하지 않습니다.

시작/재연결은 0%, 통신 두절은 약 1.5초 후 출력 0%. 정지는 ENA를 끄는 관성 정지입니다. %는 PWM 듀티비이며 회전수 %가 아닙니다. 우노 R3의 D5 기본 PWM은 약 980Hz, 8비트(0~255)입니다.

화면의 `D7=… D8=…`은 MCU에서 읽은 디지털 상태입니다. 정회전 운전은 `1,0`, 역회전 운전은 `0,1`, 정지는 `0,0`이 정상입니다. 실제 단자 전압 측정을 대신하지는 않습니다. 명령과 핀 판독은 `logs/controller-*.log`에 기록됩니다.

펌웨어: `firmware/uno_l298n/uno_l298n.ino` · 업로드 전 백업: `backups/uno-before-motor-control.hex`

참고: [Arduino PWM](https://support.arduino.cc/hc/en-us/articles/9350537961500-Use-PWM-output-with-Arduino), [Arduino analogWrite](https://github.com/arduino/reference-en/blob/master/Language/Functions/Analog%20IO/analogWrite.adoc), [ST L298 데이터시트](https://www.st.com/resource/en/datasheet/l298.pdf).
