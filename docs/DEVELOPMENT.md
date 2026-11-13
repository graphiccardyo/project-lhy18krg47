# 개발 가이드 (코드를 고치는 팀원용)

## 필요한 도구

| 도구 | 용도 | 비고 |
|---|---|---|
| Arduino IDE 2 또는 `arduino-cli` | 펌웨어 빌드·업로드 | 코어 `arduino:avr` 1.8.x. 외부 라이브러리 없음 (Wire, EEPROM만). `Build-Lab.ps1`은 IDE 안의 arduino-cli를 자동으로 찾음 |
| Edge / Chrome | 실험 UI (Web Serial) | `file://`로 열어도 동작. Firefox·Safari는 Web Serial 미지원 |
| Python 3.10+ | `analysis/` | `pip install -r analysis/requirements.txt` (numpy, scipy, matplotlib) |
| Node.js | `scripts/test_sysid.js` (UI 계산 코드 시험) | |
| MATLAB + System Identification / Control System Toolbox | `matlab/` | 보고서용 추정·설계 |
| Windows PowerShell 5.1 | `scripts/*.ps1` | 한글이 있는 `.ps1`은 **UTF-8 BOM**으로 저장해야 깨지지 않음 |

## 빌드·시험

```bash
# 펌웨어 컴파일만 (크기 확인)
powershell -ExecutionPolicy Bypass -File scripts/Build-Lab.ps1
# 컴파일 + 업로드
powershell -ExecutionPolicy Bypass -File scripts/Build-Lab.ps1 -Upload -Port COM4
# 보드 자동 시험 (약 1분). 모터는 돌리지 않음(벤치 모드). 설정은 CFG BACKUP/RESTORE 로 보존
powershell -ExecutionPolicy Bypass -File scripts/Test-LabFirmware.ps1 -Port COM4
# UI 계산 코드 (식별·정적 특성) 시험
node scripts/test_sysid.js
# 분석 재현
python analysis/aero_analyze.py data/20261106_run3
python analysis/pid_design.py
```

- 보드 시험이나 업로드 전에 UI 페이지와 시리얼 모니터를 닫으세요. COM 포트는 한 프로그램만 열 수 있습니다.
- `Test-LabFirmware.ps1`은 IMU가 없어도 돕니다(IMU 관련 항목만 결과가 달라짐). 폐루프 항목은 각 샘플의 θ로 기대값을 계산하므로 진자가 매달려 있어도 됩니다.
- UI 변경은 **시뮬레이터**로 먼저 확인합니다. 자동 브라우저 시험이 필요하면 `python -m http.server 8765 --directory lab`으로 띄웁니다.

## 코드 지도

### 펌웨어 `firmware/aeropendulum_lab/aeropendulum_lab.ino` (단일 파일)

| 구역 | 내용 |
|---|---|
| 상수 | 핀(D5/D7/D8), 250000 baud, 200 Hz 틱, 센서 범위 ±16 g/±2000 °/s, 가속도 LPF·게이트 |
| `Config` + EEPROM | 축·부호·DLPF·β·자이로 바이어스·가속도 오프셋·영점, CRC-8, v3→v4 이전, 예비 영역(128) |
| `Madgwick` | 6축 IMU 필터 (`update`, `gravity`, `seed`) |
| `CalState` / `calAccumulate` | 정지(`CAL`)·수평(`CAL FLAT`) 보정, Welford 평균·분산, 실패 판정 |
| `Profile` / `profileStep` | STEP / STAIR / CHIRP / PRBS / CLSTEP / RELAY 신호 생성 (샘플 단위) |
| `Pid` / `notchCoeffs` / `pidStep` | 폐루프 제어기 (D on ω, 노치, 포화, 조건부 적분) |
| `applyMotor` / `stopAll` / `abortRun` | 모터 출력·정지 (벤치 모드면 핀 LOW 유지) |
| `probeImu` / `imuInit` / `imuRead` | MPU6050 직접 구동 (400 → 100 kHz 재시도, 포화 집계) |
| `pivotAngle` | 중력 벡터 → 피벗 축 기준 각도 |
| `sendData` / `sendAux` / `sendStatus` | `D` / `A` / `S` 줄 (빠른 출력을 위해 버퍼에 만든 뒤 한 번에 write) |
| `sampleStep` / `tick` / `loop` | 틱 스케줄, 센서·필터, 샘플마다 제어·출력, 하트비트·워치독 |
| `command` / `runCommand` / `parseInts` | 명령 해석 ([PROTOCOL.md](PROTOCOL.md)) |

### UI `lab/`

| 파일 | 내용 |
|---|---|
| `index.html` | 레이아웃·스타일, 탭 ①–⑥ 마크업 |
| `app.js` | 상태 `S`, 전송(`SerialLink` Web Serial / `SimLink`), 수신 분기 `onLine`, 기록·CSV, 실시간 그래프, 탭별 기능, `PERSIST` (localStorage 폼 저장) |
| `sim.js` | `SimDevice`: 펌웨어 명령을 똑같이 흉내 내는 시뮬레이터. 플랜트 = 정적 특성 + 1차 지연 + 3.47 Hz 공진 + 지연 |
| `sysid.js` | 계산 (ARX/Steiglitz–McBride, 정적 특성, 스텝 지표, 폐루프 시뮬레이션, 폐루프 지표, 릴레이 → Ku/Tu, Z-N). 브라우저와 Node 공용 |
| `chart.js` | 의존성 없는 캔버스 차트 |

### 분석

| 파일 | 내용 |
|---|---|
| `analysis/aero_analyze.py` | 데이터 품질(진동·드리프트), 반복 재현성·SNR, 정적 특성, 링잉, 교차검증 맞춤. 각도 소스 자동 선택(θ / ∫ω) |
| `analysis/aero_freqresp.py` | CHIRP 주파수응답 평균 |
| `analysis/aero_model_fit.py` | 공진 포함 모델을 주파수영역에서 맞추고 다른 기록으로 시간영역 검증 |
| `analysis/pid_design.py` | 강인성(Ms) 기준 PID 설계, Z-N, 시간응답 시뮬레이션 |
| `matlab/aero_*.m` | 같은 작업의 MATLAB 버전 (보고서용) |

## 지켜야 할 것

1. **입력 단위는 PWM(0–255)**입니다. 전압으로 바꾸지 마세요(팀 결정). 모델·이득 단위는 deg/PWM, PWM/deg입니다.
2. **프로토콜은 다섯 곳이 함께 움직입니다**: 펌웨어, `sim.js`, `app.js`, `scripts/Test-LabFirmware.ps1`, `docs/PROTOCOL.md`. 하나만 바꾸면 시뮬레이터와 실제 보드가 달라집니다.
3. **플래시가 97%입니다**(31.4 KB / 32.3 KB). 기능을 추가하기 전에 컴파일해서 크기를 확인하세요. 이미 쓴 절약 방법:
   - `Serial.print(float)` 대신 `printFix`
   - `strtol` 대신 `parseInts`
   - 로그 스윕 계수는 `pow` 대신 `sqrt` 반복
   - 피해야 할 것: `String`, `sprintf("%f")`, `pow`/`log`/`exp`, 긴 `F()` 문자열
4. **틱 시간 예산**: `S` 줄의 `maxTick_us`가 5000 µs보다 작아야 합니다(현재 약 4 ms). 무거운 계산은 샘플 틱(`sampleStep`)이 아닌 곳으로 옮기세요.
5. **안전 동작을 약화시키지 마세요**: 하트비트 1 s, 각도 한계, 최대 PWM, IMU 실패 시 정지, 워치독, 벤치 모드의 핀 LOW.
6. 보정·영점은 EEPROM에 있습니다. 시험 스크립트처럼 설정을 바꾸는 코드는 `CFG BACKUP` → … → `CFG RESTORE`로 원래대로 돌려놓으세요.

## 기능 추가 절차 (예: 새 시험 신호, 새 명령)

1. 펌웨어: `command`/`runCommand`, `profileStep`(신호), 필요하면 `ProfileType` 추가 → 컴파일해서 크기 확인
2. `sim.js`에 같은 동작 추가 (명령 파싱, 신호 생성, 출력 형식 동일하게)
3. `app.js` / `index.html` UI 추가. 새 입력란은 `PERSIST`에 등록
4. `scripts/Test-LabFirmware.ps1`에 벤치 모드 시험 추가 (모터를 돌리지 않는 범위)
5. `docs/PROTOCOL.md`, `docs/EXPERIMENT_GUIDE.md` 갱신
6. 시뮬레이터로 UI 확인 → 보드 업로드 → 시험 스크립트 통과 확인

## 데이터 관리

- 실험 CSV는 `data/<YYYYMMDD>_run<N>/`에 원본 그대로 넣습니다(수정 금지). 조건(전원 전압, 장착 변경 등)은 `data/README.md`에 한 줄로 남기세요.
- 분석 결과는 같은 폴더의 `analysis/`에 두고, 결론은 `analysis/REPORT.md`에 씁니다. 스크립트 출력은 다시 만들 수 있게 명령을 적어 두세요.
- 큰 결론이 바뀌면 `docs/RESULTS.md`를 갱신합니다.

## Git

- 기능마다 브랜치를 만들고 PR로 `main`에 합칩니다. `build/`, `logs/`, `__pycache__/`, `.claude/`는 커밋하지 않습니다(`.gitignore`).
- 이전 1주차 기구 확인 프로그램은 첫 커밋 `73a69be`에 있습니다.
