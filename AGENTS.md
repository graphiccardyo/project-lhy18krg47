# AGENTS.md — AI 코딩 에이전트용 작업 지침

이 저장소에서 작업하는 에이전트(Claude Code, Codex 등)를 위한 문서입니다. 사람용 설명은 [README.md](README.md)와 [docs/](docs/)에 있습니다. 팀원과는 **한국어**로 소통하고, 문서·UI 문구도 한국어로 씁니다.

## 프로젝트 한 줄 요약

에어로펜듈럼(프로펠러로 진자 각도 제어). 대학 자동제어 과제의 목표는 다음과 같습니다.
1. 정적 특성 시험
2. 스텝·스윕 시험
3. 전달함수 식별·검증
4. 폐루프 스텝 응답
5. PID 설계·시험
6. Z-N 설계·시험
7. 강화학습 제어기

**1–3은 완료**, **4–6은 구현과 시뮬레이터 검증만 하고 하드웨어 시험 전**, **7은 시작 전**입니다(2026-11-06). 현재 결론은 [docs/RESULTS.md](docs/RESULTS.md)에 있습니다.

## 저장소 지도

| 경로 | 내용 |
|---|---|
| `firmware/aeropendulum_lab/aeropendulum_lab.ino` | Uno 펌웨어 (단일 파일, 외부 라이브러리 없음) |
| `lab/` | Web Serial 실험 UI (`index.html`, `app.js`), 시뮬레이터 `sim.js`, 계산 `sysid.js`, 차트 `chart.js` |
| `analysis/` | Python 분석·설계 (`aero_analyze.py`, `aero_freqresp.py`, `aero_model_fit.py`, `pid_design.py`) |
| `matlab/` | MATLAB 분석 (`aero_load`, `aero_iddata`, `aero_sysid(_all)`, `aero_static_map`, `aero_pid_design`) |
| `data/<날짜>_runN/` | 원본 CSV(수정 금지) + `analysis/REPORT.md` |
| `scripts/` | `Build-Lab.ps1`(컴파일·업로드), `Test-LabFirmware.ps1`(보드 시험 60여 항목), `test_sysid.js` |
| `docs/` | `EXPERIMENT_GUIDE`, `PROTOCOL`, `HARDWARE`, `RESULTS`, `DEVELOPMENT` |

## 반드시 지킬 것

1. **입력 단위는 PWM 카운트(0–255)**입니다. 팀이 명시적으로 정했으므로 전압으로 바꾸지 마세요. 모델은 deg/PWM입니다.
2. **프로토콜 동기화**: 시리얼 명령이나 출력 형식을 바꾸면 펌웨어, `lab/sim.js`, `lab/app.js`, `scripts/Test-LabFirmware.ps1`, `docs/PROTOCOL.md`를 **모두** 고칩니다. 시뮬레이터는 펌웨어의 계산식(예: `pidStep`, 노치 계수, 릴레이)을 그대로 복제하고 있습니다.
3. **플래시 97%** (31.4 / 32.3 KB). 펌웨어 기능을 추가하면 반드시 컴파일해서 크기를 확인합니다. `Serial.print(float)`, `String`, `pow/log/exp`, `strtol`, 긴 `F()` 문자열은 피하고, 이미 있는 `printFix`, `parseInts`를 재사용하세요. 넘치면 무엇을 줄일지 사용자에게 먼저 묻습니다.
4. **틱 예산**: 200 Hz 틱 안에 끝나야 합니다(`S` 줄 `maxTick_us` < 5000, 현재 약 4 ms).
5. **안전**: 하트비트 정지(1 s), 각도 한계, 최대 PWM, IMU 실패 정지, 워치독, 벤치 모드 핀 LOW를 약화시키지 마세요.
6. **하드웨어에서 모터를 돌리는 명령**(`U` > 0, 벤치 모드가 아닌 `RUN …`, `REF`, `RUN CLSTEP`, `RUN RELAY`)은 사람이 그 자리에서 명시적으로 허락하기 전에는 실행하지 않습니다. 자동 시험은 `BENCH 1`에서만 돌립니다.
7. **EEPROM의 보정·영점은 사용자 것입니다.** 설정을 바꾸는 시험은 `CFG BACKUP`으로 시작해 `CFG RESTORE`로 끝냅니다. `FACTORY`를 함부로 보내지 마세요.
8. `data/` 아래 원본 CSV는 고치지 않습니다. 새 데이터는 새 run 폴더에 둡니다.

## 검증 방법

| 변경 | 확인 |
|---|---|
| 펌웨어 | `arduino-cli compile -b arduino:avr:uno --output-dir build/aeropendulum_lab firmware/aeropendulum_lab` (경고 0, 플래시 확인) → 보드가 있으면 업로드 후 `scripts/Test-LabFirmware.ps1 -Port COM4` |
| `sysid.js` | `node scripts/test_sysid.js` |
| UI / 시뮬레이터 | `python -m http.server 8765 --directory lab` 로 띄워 브라우저에서 **시뮬레이터** 버튼으로 해당 탭 동작 확인 |
| 분석 스크립트 | `python analysis/aero_analyze.py data/20261106_run3` 등 기존 run에서 숫자가 REPORT.md와 같은지 |
| MATLAB | 실행 환경이 없을 수 있음 → 문법만 확인했다면 "실행 안 함"이라고 보고 |

하드웨어나 MATLAB으로 확인하지 못한 것은 결과 보고에 분명히 적습니다.

## 환경 함정 (Windows)

- `arduino-cli`가 PATH에 없을 수 있습니다. `scripts/Build-Lab.ps1 -Cli <경로>`나 Arduino IDE를 쓰세요. 보드는 보통 **COM4**이고, Uno R3 코어는 `arduino:avr`입니다.
- COM 포트는 한 프로그램만 엽니다. UI가 연결되어 있으면 업로드와 시험이 "access denied"로 실패하니 사용자에게 닫아 달라고 하세요. 포트가 아예 안 보이면 USB가 빠진 것입니다.
- Windows PowerShell 5.1은 BOM 없는 UTF-8을 ANSI로 읽습니다. 한글이 있는 `.ps1`은 **UTF-8 BOM**으로 저장하세요. PowerShell 문자열 안의 `$var한글`은 변수 이름으로 붙어 버리므로 `${var}`로 씁니다.
- 일부 에이전트 브라우저 미리보기는 `file://`을 정적 스냅샷으로 열어 스크립트가 안 돌고, Web Serial 포트 선택창도 못 엽니다. UI 확인은 로컬 HTTP 서버 + 시뮬레이터로 하고, 실제 보드 연결은 사람이 Edge/Chrome에서 합니다. 파일을 고친 뒤에는 브라우저 캐시 때문에 `fetch(f, {cache:'reload'})` 후 새로 고쳐야 반영될 수 있습니다.
- Arduino AVR 코어에서 `min`/`max`는 매크로라 `min<T>()` 템플릿 문법이 안 됩니다.

## 데이터 해석 시 주의 (지금까지 배운 것)

- 프로펠러 진동(약 40–48 Hz)이 센서에 실립니다. 첫 실험 데이터는 이것 때문에 쓸 수 없었습니다. 새 데이터를 받으면 먼저 `aero_analyze.py`의 품질 진단(θ<sub>acc</sub> σ, |∫ω − θ|, 게이트, 포화)을 보세요.
- 같은 시험을 반복한 기록 쌍으로 SNR과 적합도 상한을 계산할 수 있습니다. 별도 검증 데이터가 없을 때의 표준 방법입니다.
- 플랜트 이득 K는 세션 중 0.21 → 0.12로 변했습니다(전원 의심). 모델의 형태(T = 0.328 s, 지연 16 ms, 공진 3.47 Hz / ζ 0.12)는 일관됩니다.
- 3.47 Hz 구조 공진 때문에 위상 −180°가 생기므로, Z-N의 Ku와 Tu는 이 공진이 정합니다.
- 피벗 마찰로 정지 위치가 ±1–5° 바뀝니다. 동특성 분석은 운전점 또는 구간 평균을 빼서 합니다.

## 남은 작업 (우선순위)

1. 하드웨어 폐루프 시험 (목표 4–6). 절차는 `analysis/pid_design/REPORT.md` → UI ⑥이고, 결과는 `data/<날짜>_runN/`에 둡니다.
2. 모터 전원 전압 기록 방법. 펌웨어 A0 분압 측정을 넣으려면 플래시 확보가 먼저입니다.
3. 목표 7 강화학습. 식별 모델 시뮬레이터(`sim.js` / `pid_design.simulate`)를 환경으로 쓰고, 하드웨어 실행 경로를 설계해야 합니다.
