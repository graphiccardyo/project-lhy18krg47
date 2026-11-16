# AGENTS.md — AI 코딩 에이전트용 작업 지침

이 저장소에서 작업하는 에이전트(Claude Code, Codex 등)를 위한 문서입니다. 사람용 설명은 [README.md](README.md)와 [docs/](docs/)에 있습니다. 팀원과는 **한국어**로 소통하고, 문서·UI 문구도 한국어로 씁니다.

## 프로젝트 한 줄 요약

에어로펜듈럼(프로펠러로 진자 각도 제어). 대학 자동제어 과제의 목표와 상태는 다음과 같습니다. 결과·근거는 [docs/RESULTS.md](docs/RESULTS.md)에 있습니다.

| 목표 | 상태 |
|---|---|
| 1. 정적 특성 시험 · 2. 스텝·스윕 시험 · 3. 전달함수 식별·검증 | 완료. **2026-11-15 재식별 모델이 기준**(아래 "기준 모델") |
| 4. 폐루프 스텝 응답 · 5. PID 설계·시험 · 6. Z-N 설계·시험 | 구현 완료(펌웨어 2.1), 하드웨어 폐루프 시험 대부분 완료(PI + 노치, PID + 노치, Z-N PI·PID·PID + 노치). P 등 남음 |
| 7. 강화학습 제어기 | 옛 모터: 순수 SAC `data/20261115_rl8` best(손튜닝 PID 대비). **11-16 모터 교체 뒤: PI + 노치 위의 잔차 RL `data/20261117_rl5`(펌웨어 1.2)** — 확인 시험 RMS 1.68° vs 1.85°, 정상상태 PID와 같음 |
| 시연 | 시연용 펌웨어 `AEROPEND_DEMO 2.0` = **잔차 RL**(학습 때 PI + 노치에 16-16 정책 잔차 ±15 PWM, 정책 `data/20261117_rl5` ep 81(`manual_00081_041012`, 확인 시험한 정책)을 정수 양자화) + 조정 가능한 PID + `lab/demo.html`(시험·결과 저장). 2.0 보드 벤치 시험 통과(11-17, 37항목, miss 0), **모터 돌린 실기 시연은 아직** ([docs/DEMO.md](docs/DEMO.md)). 1.0(rl8 증분 정책)은 커밋 a35dfe4 |

## 저장소 지도

| 경로 | 내용 |
|---|---|
| `firmware/aeropendulum_lab/aeropendulum_lab.ino` | Uno 펌웨어 (단일 파일, 외부 라이브러리 없음) |
| `firmware/aeropendulum_rl/aeropendulum_rl.ino` | 강화학습용 펌웨어 `AEROPEND_RL 1.2` (실험용에서 갈라짐: 개루프 프로파일·RELAY 삭제, `RL`/`SAFE`/`RUN RL`/`A` 추가, 1.1 = PID + 잔차 모드, 1.2 = 잔차가 노치 통과, EEPROM 형식 공유). 프로토콜은 `docs/RL_PROTOCOL.md` |
| `firmware/aeropendulum_demo/` | 시연용 펌웨어 `AEROPEND_DEMO 2.0` (RL 펌웨어에서 갈라짐: 보드 위 고정 잔차 RL 정책 + 사용자 PID / RL 기반 PID 두 묶음, `MODE`/`OUT`/`NN`/`RLPOL` 추가, 가진·CLSTEP·RL 스트리밍 삭제, EEPROM 형식 공유) + 자동 생성 `policy.h`(가중치 + 실행 config 상수). 설명·프로토콜은 `docs/DEMO.md` |
| `rl/` | Python SAC 실기 학습기 (`python -m rl.train`), 가짜 보드, 벤치 시험 `rl.bench_test`(83항목), 자체 시험 `rl.selftest`, 웹 대시보드. `.venv`의 Python을 씁니다(PyTorch). 설명은 `rl/README.md` |
| `lab/` | Web Serial 실험 UI (`index.html`, `app.js`), 시뮬레이터 `sim.js`, 계산 `sysid.js`, 차트 `chart.js`. 시연 UI `demo.html`·`demo.js` + 시뮬레이터 `demo_sim.js`·`demo_policy.js`(자동 생성) |
| `analysis/` | Python 분석·설계 (`aero_analyze.py`, `aero_freqresp.py`, `aero_model_fit.py`, `aero_closedloop.py`(폐루프 기록 재현), `pid_design.py`) |
| `matlab/` | MATLAB 분석 (`aero_load`, `aero_iddata`, `aero_sysid(_all)`, `aero_static_map`, `aero_pid_design`) |
| `data/<날짜>_runN/` | 원본 CSV(수정 금지) + `analysis/REPORT.md` |
| `scripts/` | `Build-Lab.ps1`(컴파일·업로드), `Build-Rl.ps1`, `Build-Demo.ps1`(`-NnTest`), `Test-LabFirmware.ps1`(보드 시험 67항목), `test_sysid.js` |
| `docs/` | `EXPERIMENT_GUIDE`, `PROTOCOL`, `HARDWARE`, `RESULTS`, `DEVELOPMENT`, `RL_PROTOCOL`, `RL_HISTORY`(강화학습 실행별 결정·근거 기록 — RL 설정을 바꾸면 여기에 한 항목 추가) |

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
| RL 펌웨어 | `scripts/Build-Rl.ps1` (경고 0, 현재 30362 B = 94%) → 업로드 후 `.venv\Scripts\python.exe -m rl.bench_test --port COM4` (벤치 모드에서만 돌고 `CFG BACKUP`/`RESTORE`) |
| `rl/` | `python -m rl.selftest --quick`, `python -m rl.bench_test --fake --fast`. RL 프로토콜을 바꾸면 펌웨어, `rl/fake_board.py`, `rl/bench_test.py`, `docs/RL_PROTOCOL.md`를 함께 고칩니다 |
| 시연 펌웨어 | `scripts/Build-Demo.ps1` (경고 0, 현재 26852 B = 83%, `NN` 시험 명령 포함) → 업로드 후 `.venv\Scripts\python.exe -m rl.demo_test --port COM4 --nn 300 --run <policy.h 를 만든 실행>` (벤치 모드만, EEPROM 안 바꿈). 핵심 지표: 잔차 r·PWM = 에뮬레이터, 관측 = `env.py`, 시연 설정 `miss 0`. 정책을 바꾸면 `python -m rl.demo_policy --run <잔차 RL 실행>`(policy.h + lab/demo_policy.js, 설정 상수도 함께). 프로토콜을 바꾸면 펌웨어, `lab/demo_sim.js`, `lab/demo.js`, `rl/demo_test.py`, `docs/DEMO.md`를 함께 고칩니다 |
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
- 시연 펌웨어의 RL 추론은 틱 사이 빈 시간(약 8 ms / 20 ms)에 돕니다(2.0의 16-16 망은 1 ms 안쪽, 1.0의 64-64는 약 5 ms). 틱·직렬 출력에 일을 더하면 추론이 20 ms를 넘어 `miss`가 생길 수 있으니 고친 뒤에는 `rl.demo_test`의 시연 설정 `miss 0`을 확인하세요. 추론 중에 `Serial`로 길게 보내지 마세요(전송 버퍼가 차면 막힘). 잔차 모드는 샘플마다 PID를 계산하므로 θacc는 샘플이 아닌 틱에서 계산합니다(RL 펌웨어 1.2와 같음).

## 기준 모델 (2026-11-15 재식별)

- 빠른 경로 K ≈ 0.164, T = 0.324 s, 지연 24 ms(DLPF 5), **공진 2.35 Hz / ζ 0.084**(30°에서 2.26 Hz / 0.066), **느린 경로 Ks 0.12 / Tc 3 s**. 20° 릴레이 실측 Ku 4.87 / Tu 0.439 s(모델 5.01 / 0.430), 폐루프 스텝 θ 87–91%. PWM 200 위에서 추력이 거의 늘지 않습니다(250 → 39°). 근거: `data/20261115_run1`·`run2`, [RESULTS.md §2](docs/RESULTS.md#2-플랜트-재식별과-pid-재설계-11-15).
- 권장 제어기: **PI + 2.35 Hz 노치 Kp 4.30, Ki 13.3**(하드웨어 상승 0.57–0.71 s).
- 모델 상수가 있는 곳: `analysis/pid_design.py`·`aero_closedloop.py` 기본값, `lab/sim.js`, `lab/sysid.js`, `rl/config.py`(`PlantCfg`, 도메인 랜덤화 `PlantCfg.dr`), `matlab/aero_pid_design.m`. 모델을 바꾸면 모두 고칩니다.
- **이전 모델은 설계·학습에 쓰지 마세요.** 11-13(기구 수정 직후) K 0.14, T 0.319 s, 21 ms, 2.84 Hz / ζ 0.136, Ks 0.05 / Tc 6 s — 그 권장값(PI 5.40 / 16.9 + 2.84 Hz 노치)은 새 플랜트에서 최악 Ms 3.3. 11-06(기구 수정 전) K 0.12–0.21, T 0.328 s, 16 ms, 3.47 Hz / ζ 0.12. 11-13 기록을 그 보고서 숫자대로 재현할 때는 `aero_closedloop.py … --model 0.14 0.319 0.021 2.84 0.136 0.05 6 --margin-k 0.12 0.14 0.24`.

## 데이터 해석 시 주의

근거와 수치는 [docs/RESULTS.md](docs/RESULTS.md)에 있습니다. 여기에는 규칙만 둡니다.

- **새 데이터는 먼저 품질 진단**: 프로펠러 진동(약 40–48 Hz)이 센서에 실립니다(첫 실험 데이터는 이것 때문에 못 씀). `aero_analyze.py`의 θ<sub>acc</sub> σ, |∫ω − θ|, 게이트, 포화를 보세요.
- **개루프 분석은 `aero_analyze.py --angle theta`**. 자동 선택이 ∫ω를 고르면 자이로 바이어스(온도로 ±0.4 °/s)가 60 s 시험에 쌓여 결과가 망가집니다(11-15 run1).
- 별도 검증 데이터가 없을 때는 같은 시험을 반복한 기록 쌍으로 SNR과 적합도 상한을 계산합니다.
- **플랜트가 세션 사이에 변합니다**(공진 11-13 2.84 → 11-15 2.35 Hz). 노치·이득이 공진에 맞는지 의심되면 릴레이 한 번(Tu)으로 확인하세요. RL 폐루프 기록에서도 공진을 추정할 수 있습니다(직접법, 주파수 편향 +0.05 Hz 정도).
- **Z-N의 Ku·Tu는 구조 공진이 정합니다**(위상 −180°가 공진에서 생김). PM보다 **Ms**로 판단하세요. 11-13 Z-N PID는 PM 70°인데 30°에서 불안정했습니다.
- **느린 경로**(수 초): 스윕 이득이 스텝·계단 이득보다 작고, 폐루프에서 기준을 유지하는 동안 PWM이 수 초에 10씩 흘러내립니다.
- **이득은 운전점에 따라 다릅니다**(30°에서 20°의 약 1.6배). 11-06에는 세션 중 0.21 → 0.12로 변했습니다(원인 미확정).
- **케이블·피벗 마찰**: 정지 위치가 ±1–5° 바뀌고(같은 20°를 유지하는 PWM이 129–145), 케이블이 걸렸다 풀리면 θ가 0.2 s에 15–25° 튀고 2.7 Hz로 울립니다. PWM이 먼저 변하지 않았으면 제어기 불안정이 아닙니다. 동특성 분석은 운전점 또는 구간 평균을 빼서 합니다.
- **IMU 자이로 편향이 온도로 약 1 °/s 흔들립니다.** 그래서 사용자 승인으로 `CAL` 다시 + **β 0.01**로 바꿨습니다(11-15 rl8부터; 11-13 이후 그 전 기록은 β 0.004, CSV 머리줄 `beta=`). 세션 전에 `rl.imu_check`로 확인하고, 기록에서 θ − θ<sub>acc</sub>가 정상(±0.3–0.9°)을 넘으면 그 구간은 의심하세요. 상세: [RESULTS.md §3](docs/RESULTS.md#자세-추정-imu-문제와-대책).
- RL 기록의 "같은 PWM에 각도가 낮아짐"(11-14 rl3 후반, 11-15 rl1)은 과열 이득 변화와 자이로 표류가 섞였을 수 있습니다(원인 미확정).
- CSV에는 최대 PWM 설정이 없습니다. 폐루프 기록을 재현할 때는 기록의 최대값(210 또는 255)과 설정 파일로 추정합니다.

## 남은 작업 (우선순위)

전체 목록은 [RESULTS.md §5](docs/RESULTS.md#5-남은-일)에 있습니다.

1. **남은 하드웨어 폐루프 시험** (목표 4–6, 11-15 모델): P(1.59)(Z-N PID와 Z-N PID + 노치는 `data/20261115_run3`에서 20–25° 시험: 노치 없으면 2.34 Hz 리플 지속, 노치 넣으면 PI + 노치 수준; 노치 없는 30°는 모델상 불안정이라 안 함), 같은 스텝의 손튜닝 PID, PI + 노치 30 s 유지 리플, 30° 스텝. 절차는 `analysis/pid_design/REPORT.md` → UI ⑥, 결과는 `data/<날짜>_runN/`.
   - UI에서 이득을 바꾸면 반드시 **이득 적용** 후 `PIDCFG` 줄로 확인하게 안내하세요(CSV `# pid` 줄 = 보드에 실제로 적용된 값). 11-15 run2의 Z-N PID 의도 기록 다섯 개는 이걸 놓쳐 Kp 2.19로 적용돼 있었습니다.
2. **목표 7 강화학습**: 옛 모터 결과는 `data/20261115_rl8` best(순수 SAC, 손튜닝 PID 대비), 모터 교체 뒤 결과는 `data/20261117_rl5`(잔차 RL on PI + 노치, `rl/configs/motor2.json`, 펌웨어 1.2, [RL_HISTORY D13](docs/RL_HISTORY.md#d13-모터-교체-뒤-잔차-rl-11-16)). 새 모터 보호로 PWM 230·가동 120 s/휴식 90 s. 남은 것은 보고서 정리이고, 사용자가 다시 요청하기 전에는 새 RL 실행을 제안하지 마세요.
   - RL 설정을 바꾸면 [docs/RL_HISTORY.md](docs/RL_HISTORY.md)에 한 항목을 추가합니다. 결과 요약은 [RESULTS.md §3](docs/RESULTS.md#3-강화학습-목표-7).
   - 보드 위 MLP 경로는 대안으로만 남아 있습니다(플래시·RAM 측정은 [RL_HISTORY.md D1](docs/RL_HISTORY.md#d1-실행-경로-pc-스트리밍)). 넣는다면 추론 시간을 `S` 줄 `maxTick`으로 먼저 확인하세요(틱 예산 5 ms).
