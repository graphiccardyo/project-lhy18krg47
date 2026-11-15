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

**1–3은 완료**, **4–6은 구현 완료(펌웨어 2.1)이고 하드웨어 폐루프 시험을 일부 했습니다**(2026-11-13: 손튜닝 PID, PI + 2.85 Hz 노치, P, Z-N 릴레이·PID → `data/20261113_run2`, `run3`). **7은 실기 SAC 학습 중**입니다(RL 펌웨어 + PC 학습기, 2026-11-14–15 실행 `data/2026111[45]_rl*`; 최선 `20261115_rl8` 평가 RMS 1.94° vs PID 2.41°, 편향 0.29° vs 0.11°). 2026-11-13에 **기구를 수정**(케이블 간섭 최소화)해서 모델을 다시 식별했습니다(2026-11-14). 현재 결론은 [docs/RESULTS.md](docs/RESULTS.md)에 있습니다.

## 저장소 지도

| 경로 | 내용 |
|---|---|
| `firmware/aeropendulum_lab/aeropendulum_lab.ino` | Uno 펌웨어 (단일 파일, 외부 라이브러리 없음) |
| `firmware/aeropendulum_rl/aeropendulum_rl.ino` | 강화학습용 펌웨어 `AEROPEND_RL 1.0` (실험용에서 갈라짐: 개루프 프로파일·RELAY 삭제, `RL`/`SAFE`/`RUN RL`/`A` 추가, EEPROM 형식 공유). 프로토콜은 `docs/RL_PROTOCOL.md` |
| `rl/` | Python SAC 실기 학습기 (`python -m rl.train`), 가짜 보드, 벤치 시험 `rl.bench_test`(83항목), 자체 시험 `rl.selftest`, 웹 대시보드. `.venv`의 Python을 씁니다(PyTorch). 설명은 `rl/README.md` |
| `lab/` | Web Serial 실험 UI (`index.html`, `app.js`), 시뮬레이터 `sim.js`, 계산 `sysid.js`, 차트 `chart.js` |
| `analysis/` | Python 분석·설계 (`aero_analyze.py`, `aero_freqresp.py`, `aero_model_fit.py`, `aero_closedloop.py`(폐루프 기록 재현), `pid_design.py`) |
| `matlab/` | MATLAB 분석 (`aero_load`, `aero_iddata`, `aero_sysid(_all)`, `aero_static_map`, `aero_pid_design`) |
| `data/<날짜>_runN/` | 원본 CSV(수정 금지) + `analysis/REPORT.md` |
| `scripts/` | `Build-Lab.ps1`(컴파일·업로드), `Test-LabFirmware.ps1`(보드 시험 67항목), `test_sysid.js` |
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
| RL 펌웨어 | `scripts/Build-Rl.ps1` (경고 0, 현재 29942 B = 92%) → 업로드 후 `.venv\Scripts\python.exe -m rl.bench_test --port COM4` (벤치 모드에서만 돌고 `CFG BACKUP`/`RESTORE`) |
| `rl/` | `python -m rl.selftest --quick`, `python -m rl.bench_test --fake --fast`. RL 프로토콜을 바꾸면 펌웨어, `rl/fake_board.py`, `rl/bench_test.py`, `docs/RL_PROTOCOL.md`를 함께 고칩니다 |
| `sysid.js` | `node scripts/test_sysid.js` |
| UI / 시뮬레이터 | `python -m http.server 8765 --directory lab` 로 띄워 브라우저에서 **시뮬레이터** 버튼으로 해당 탭 동작 확인 |
| 분석 스크립트 | `python analysis/aero_analyze.py data/20261113_run1`, `python analysis/aero_closedloop.py data/20261113_run3` 등 기존 run에서 숫자가 REPORT.md와 같은지 |
| MATLAB | 실행 환경이 없을 수 있음 → 문법만 확인했다면 "실행 안 함"이라고 보고 |

하드웨어나 MATLAB으로 확인하지 못한 것은 결과 보고에 분명히 적습니다.

## 환경 함정 (Windows)

- `arduino-cli`가 PATH에 없을 수 있습니다. Arduino IDE 2에 들어 있는 것(`%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`)을 `scripts/Build-Lab.ps1`이 자동으로 찾습니다. 다른 위치면 `-Cli <경로>`를 주세요. 보드는 보통 **COM4**이고, Uno R3 코어는 `arduino:avr`입니다.
- COM 포트는 한 프로그램만 엽니다. UI가 연결되어 있으면 업로드와 시험이 "access denied"로 실패하니 사용자에게 닫아 달라고 하세요. 포트가 아예 안 보이면 USB가 빠진 것입니다.
- Windows PowerShell 5.1은 BOM 없는 UTF-8을 ANSI로 읽습니다. 한글이 있는 `.ps1`은 **UTF-8 BOM**으로 저장하세요. PowerShell 문자열 안의 `$var한글`은 변수 이름으로 붙어 버리므로 `${var}`로 씁니다.
- 일부 에이전트 브라우저 미리보기는 `file://`을 정적 스냅샷으로 열어 스크립트가 안 돌고, Web Serial 포트 선택창도 못 엽니다. UI 확인은 로컬 HTTP 서버 + 시뮬레이터로 하고, 실제 보드 연결은 사람이 Edge/Chrome에서 합니다. 파일을 고친 뒤에는 브라우저 캐시 때문에 `fetch(f, {cache:'reload'})` 후 새로 고쳐야 반영될 수 있습니다.
- 보드에는 실험용(`AEROPEND_LAB`)과 RL용(`AEROPEND_RL`) 펌웨어 중 하나만 올라가 있습니다. `HELLO` 응답으로 확인하고, 실험 UI를 쓰려면 `Build-Lab.ps1 -Upload`, 학습기를 쓰려면 `Build-Rl.ps1 -Upload`로 바꿉니다(보정·영점은 유지).
- PC ↔ 보드 왕복 지연은 약 7–9 ms입니다(2026-11-14 측정). RL의 적용 지연 `lag`는 2샘플(20 ms)로 쓰세요. 1샘플은 누락이 납니다.
- Windows에서 pyserial의 `read`/`write`를 여러 스레드로 쓰면 GIL 경합 때 "WriteFile failed"가 잘못 납니다. `rl/link.py`의 `_WinPortIO`가 이를 우회하니 되돌리지 마세요.
- Arduino AVR 코어에서 `min`/`max`는 매크로라 `min<T>()` 템플릿 문법이 안 됩니다.

## 데이터 해석 시 주의 (지금까지 배운 것)

- 프로펠러 진동(약 40–48 Hz)이 센서에 실립니다. 첫 실험 데이터는 이것 때문에 쓸 수 없었습니다. 새 데이터를 받으면 먼저 `aero_analyze.py`의 품질 진단(θ<sub>acc</sub> σ, |∫ω − θ|, 게이트, 포화)을 보세요.
- 같은 시험을 반복한 기록 쌍으로 SNR과 적합도 상한을 계산할 수 있습니다. 별도 검증 데이터가 없을 때의 표준 방법입니다.
- **2026-11-13 기구 수정 후 모델**(현재 기준, `data/20261113_run1`·`run3`): 빠른 경로 K ≈ 0.14 (20° 0.15, 30° 0.24), T = 0.319 s, 지연 21 ms(DLPF 5), **공진 2.84 Hz / ζ 0.136**, **느린 경로 Ks 0.05 / Tc 6 s**. 폐루프 기록과 θ·PWM 92–94%, 20° 릴레이 Tu 0.370 s로 맞습니다. 11-06 값(K 0.12–0.21, T 0.328 s, 지연 16 ms, 공진 3.47 Hz / ζ 0.12)은 **기구 수정 전**이니 설계·학습에 쓰지 마세요.
- 구조 공진 때문에 위상 −180°가 생기므로, Z-N의 Ku와 Tu는 이 공진이 정합니다(11-13: Ku 10.2, Tu 0.37 s). PM보다 **Ms**로 판단하세요. Z-N PID는 PM 70°인데 30°에서 불안정했습니다.
- 스윕 이득(0.13–0.145)이 스텝·계단 이득(0.17–0.22)보다 작은 것은 수 초 단위 느린 경로 때문입니다. 폐루프에서 기준을 유지하는 동안 PWM이 6 s에 10–15씩 흘러가는 것도 같은 원인입니다.
- 이득은 운전점에 따라 달라지고(30°에서 20°의 약 1.6배), 11-06에는 세션 중 0.21 → 0.12로 변했습니다(원인 미확정).
- 피벗 마찰·케이블로 정지 위치가 ±1–5° 바뀌고(같은 20°를 유지하는 PWM이 129–145), 케이블이 걸렸다 풀리면 θ가 0.2 s에 15–25° 튀고 2.7 Hz로 울립니다(PWM이 먼저 변하지 않으면 제어기 불안정이 아닙니다). 동특성 분석은 운전점 또는 구간 평균을 빼서 합니다.
- **IMU 자이로 편향이 온도로 약 1 °/s 흔들립니다**(2026-11-15, 모터 끈 정지 점검: 따뜻할 때 +0.57, 식은 뒤 −0.41 °/s). β 0.004(보정 예산 ≈ 2β = 0.46 °/s)로는 θ가 θ<sub>acc</sub>에서 수십 도 떠내려가서, 사용자 승인으로 `CAL` 다시 + **β 0.01**로 바꿨습니다(11-15 rl8부터). 11-13 이후 그 전 기록은 β 0.004입니다(CSV 머리줄 `beta=`). 세션 전에 `rl.imu_check`로 확인하고, 기록에서 θ − θ<sub>acc</sub>가 정상(±0.3–0.9°)을 넘으면 그 구간은 의심하세요.
- RL 기록에서 "같은 PWM에 각도가 낮아짐"(11-14 rl3 후반, 11-15 rl1)은 과열 이득 변화와 자이로 표류가 섞였을 수 있습니다(원인 미확정).
- CSV에는 최대 PWM 설정이 없습니다. 폐루프 기록을 재현할 때는 기록의 최대값(210 또는 255)과 설정 파일로 추정합니다.

## 남은 작업 (우선순위)

1. 남은 하드웨어 폐루프 시험 (목표 4–6): 새 모델로 다시 설계한 PI + 2.84 Hz 노치(Kp 5.40, Ki 16.9) 등. 절차는 `analysis/pid_design/REPORT.md` → UI ⑥이고, 결과는 `data/<날짜>_runN/`에 둡니다.
2. 목표 7 강화학습. 실행 경로는 **PC 스트리밍**(RL 펌웨어 `A <du>`, `rl/`)으로 정했고 실기에서 학습합니다. 남은 것: `20261115_rl8` 정책(best ep 120)으로 긴 유지 시험(`rl.play --dur 30 --pid`)해서 정상상태 오차를 PID와 비교, 편향(0.3°) 줄이기. 결과 요약은 [docs/RESULTS.md](docs/RESULTS.md) §3a. 아래 보드 위 MLP 측정은 대안 경로 기록입니다. 도메인 랜덤화 범위(가짜 보드)는 `data/20261113_run3/analysis/REPORT.md` §6에 있습니다.
   - 플래시 측정(2026-11-13, 임시 변형 컴파일): 현재 31502 B(97%). CHIRP·PRBS 제거 → 29832 B(92%), STAIR·STEP까지 → 28728 B(89%), RELAY까지 → 27906 B(86%). MLP 4-16-1(가중치 RAM, `NN` 명령으로 업로드) 추가 비용은 플래시 약 0.7 KB, RAM 393 B입니다(현재 RAM 1040 / 2048 B).
   - 보드 위 추론 시간은 측정하지 않았습니다(추정 1–2 ms, 틱 예산 5 ms). 넣기 전에 `S` 줄 `maxTick`으로 확인하세요.
