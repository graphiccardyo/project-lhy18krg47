# 시연용 펌웨어와 UI (AEROPEND_DEMO 1.0)

발표·시연에서 **PID와 강화학습(RL) 제어기를 같은 장치에서 번갈아 보여 주기** 위한 펌웨어와 UI입니다. 보드 혼자서 두 제어기 중 하나로 기준 각도를 유지하므로 PC에 Python·PyTorch가 필요 없습니다. 브라우저 한 장으로 제어기, 기준 각도, PID 이득만 바꿉니다.

| 파일 | 내용 |
|---|---|
| [firmware/aeropendulum_demo/aeropendulum_demo.ino](../firmware/aeropendulum_demo/aeropendulum_demo.ino) | 펌웨어: 센서·Madgwick·PID(+노치)·고정 RL 정책·안전장치 |
| [firmware/aeropendulum_demo/policy.h](../firmware/aeropendulum_demo/policy.h) | 정수로 양자화한 정책 가중치(자동 생성) |
| [lab/demo.html](../lab/demo.html) · [lab/demo.js](../lab/demo.js) | 시연 UI (Web Serial) |
| [lab/demo_sim.js](../lab/demo_sim.js) · [lab/demo_policy.js](../lab/demo_policy.js) | 보드 없이 쓰는 시뮬레이터(같은 정책) |
| [rl/demo_policy.py](../rl/demo_policy.py) | 정책 양자화, `policy.h`·`demo_policy.js` 생성 |
| [rl/demo_test.py](../rl/demo_test.py) | 보드 벤치 시험(모터 안 돎) |
| [scripts/Build-Demo.ps1](../scripts/Build-Demo.ps1) | 컴파일·업로드 |

## 쓰는 법

1. **업로드** (모터 12V 끄고, 실험 UI·학습기·시리얼 모니터는 닫기):
   ```bash
   powershell -ExecutionPolicy Bypass -File scripts/Build-Demo.ps1 -Upload -Port COM4
   ```
   EEPROM 형식이 실험용·RL 펌웨어와 같아서 보정·영점·β가 그대로 유지됩니다. 실험용으로 돌아갈 때는 `Build-Lab.ps1 -Upload`, 학습기는 `Build-Rl.ps1 -Upload`를 씁니다.
2. **UI**: Edge나 Chrome으로 `lab/demo.html`을 엽니다(파일을 바로 열어도 되고 `python -m http.server 8765 --directory lab`도 됨). **보드 연결**을 누르고 COM 포트를 고릅니다. 보드 없이 연습할 때는 **시뮬레이터**를 누릅니다.
3. **시연 순서** (예):
   1. 제어기 **PID**, 기준 20° → **시작**. 보드 기본 이득은 권장 PI + 2.35 Hz 노치(Kp 4.30, Ki 13.3)입니다.
   2. **강화학습(RL)** 을 누르면 PID로 기준 근처에 간 뒤, |오차| < 2°를 1초 유지하는 순간 RL로 넘어갑니다(학습 때 에피소드를 시작한 방식과 같음). 그래프에서 RL 구간은 초록 굵은 선입니다.
   3. 기준 15/20/25/30° 버튼으로 스텝 응답을 비교합니다. 학습 범위는 15–30°이고, 30°를 넘으면 PWM이 포화에 가까워집니다.
   4. **PID** 를 다시 누르면 같은 출력에서 끊김 없이 PID로 넘어갑니다. 이득 프리셋(권장 PI + 노치, 손튜닝 PID, P만)을 고르거나 숫자를 바꾼 뒤 **이득 적용**을 누릅니다. "보드에 적용된 값" 줄이 보드의 `PIDCFG` 응답입니다.
   5. **정지**(Esc / Space)로 모터를 끕니다. 창을 닫거나 연결이 끊겨도 보드가 1초 안에 멈춥니다.
4. **CSV 저장**은 연결 후 기록(최대 30분, 50 Hz)을 `aero_DEMO_*.csv`로 내려받습니다.

### 시연 전 점검

- 자이로 편향이 온도에 따라 흔들립니다([RESULTS.md §3](RESULTS.md#자세-추정-imu-문제와-대책)). 시연 직전에 진자가 멈춘 상태에서 정지 보정을 하세요. 이 펌웨어에도 `CAL` 명령(5초, 자이로 바이어스만)이 있습니다. 정밀 점검(`python -m rl.imu_check`)은 RL 펌웨어로 바꿔서 합니다.
- RL 정책은 **β 0.01**로 학습했습니다. 연결했을 때 β가 0.01이 아니면 UI가 경고합니다.
- 진자가 정지 위치에서 ±1–5° 바뀌면(케이블·피벗 마찰) `ZERO` 명령으로 영점을 다시 잡을 수 있습니다. 이 명령은 EEPROM을 바꾸므로 필요할 때만 씁니다.

## RL 정책을 보드에 넣은 방법

- **정책**: [data/20261115_rl8](../data/20261115_rl8/) best(ep 120), SAC 액터 8 → 64 → 64 → μ. 결정적 행동 a = tanh(μ)만 씁니다.
- **관측·행동**은 학습기([rl/env.py](../rl/env.py))와 같은 식을 C로 옮겼습니다. 관측은 오차, 각도, 10 Hz로 거른 각속도, Δθ, PWM, 직전 행동, 3 s 오차 평균, tanh(e/0.5°)입니다. 행동은 du = 시그마-델타(10·a·|a| + 고정 적분 보조 k<sub>I</sub> 6)입니다. 정책 PWM 범위는 40–255이고, 관측에 쓰는 θ·ω는 학습 때 D 줄처럼 0.01 단위로 반올림합니다.
- **시간 구조**도 학습 때 PC 경로(dec 2, lag 2)와 같습니다. 결정 샘플 k에서 관측을 만들어 추론을 시작하고, 결과를 k+2 샘플(20 ms 뒤)에 적용합니다. 그때까지 못 끝내면 PWM을 유지하고 `miss`를 셉니다. 학습기가 명령 누락을 처리한 방식과 같습니다.
- **양자화**: Uno에서는 float 곱셈 4 700번이 20 ms 안에 끝나지 않아서 정수로 바꿨습니다. 입력은 int16(×2¹¹)입니다. 1층은 14비트 가중치, 2층은 int8 가중치(편향 보정), 3층은 10비트 가중치이고, 활성값은 int16입니다. ReLU 동차성으로 뉴런 배율을 맞춰 재양자화는 시프트 한 번입니다. 자세한 것은 `rl/demo_policy.py` 머리말에 있습니다. rl8 리플레이 관측 168 831개에서 행동 오차는 RMS 0.011, 최대 0.16입니다. |e| < 1°에서 평균 증분 차이는 −0.008 PWM/결정이고, 평형 편향으로 약 0.07°에 해당합니다.
- **계산 시간**: 곱셈-누산을 AVR 어셈블리로 짰습니다. 1층은 16×16 비트, 2층은 ReLU로 0이 된 입력을 건너뛰고(리플레이에서 0이 아닌 것 최대 39/64) 두 행씩 계산합니다. 틱 없이 한 번에 4.2–5.8 ms입니다(C로 쓰면 21 ms). 200 Hz 틱(IMU 읽기 0.8 ms, Madgwick 1.3 ms)이 CPU의 약 61%를 쓰므로, 추론은 틱 사이 빈 시간에 조각(0.2–0.35 ms)으로 나눠 합니다.
- **가짜 보드 폐루프 비교** (rl8 설정, 평가 시나리오 3개 × 3회): float 정책 RMS 2.09°, 정착 |e| 0.54°, OS 16%이고 양자화 정책 RMS 2.09°, 0.55°, 17%로 차이가 없습니다.
- 플래시 31 742 B (98%), RAM 1 107 B입니다. 플래시를 맞추려고 실험용 펌웨어의 가진 신호·CLSTEP·RL 스트리밍·`A` 줄·`CAL FLAT`·`ACCOFF`·`AXIS`·`DLPF`·`FACTORY`·`CFG BACKUP/RESTORE`를 뺐고, `CAL`은 평균 + 움직임 범위 판정으로 줄였습니다.

## 검증 (2026-11-16, 보드 벤치)

`.venv\Scripts\python.exe -m rl.demo_test --port COM4`로 확인합니다. `--nn N`을 붙이면 `Build-Demo.ps1 -NnTest` 빌드가 필요합니다. 벤치 모드(모터 핀 LOW)에서만 돌고 EEPROM은 바꾸지 않습니다.

| 항목 | 결과 |
|---|---|
| 명령·오류·하트비트 정지(LINK_TIMEOUT)·D 줄 형식 | 통과 |
| `NN` 정책 계산 (리플레이 관측 300개) | 보드 정수 누산값 = `QuantPolicy` **정확히 일치**, μ 차이 < 6e-7 |
| RL 끝-끝 (10 s, D 줄 10 ms) | 497결정에서 보드 PWM·행동 = float32 에뮬레이터 **모두 일치**. 누락 추론 2–4회 = 보드 보고값. 관측 8개 = `rl/env.py` (최대 차이 1.1e-6) |
| 시연 설정(D 줄 20 ms) RL 10 s | **누락 0**, 추론 벽시계 최대 14.3 ms(< 20 ms), maxTick 4.7 ms, 오버런 0, 틱 부하 61% |
| UI | 시뮬레이터로 연결 → PID 시작 → RL 전환 → 기준 변경 → 이득 적용 → 정지, 휴대폰 폭 확인. JS 정책 = Python (300개, 차이 < 1e-6) |

**확인하지 못한 것**

- **모터를 돌린 실기 시연은 아직 하지 않았습니다.** PID·RL 모두 벤치 모드에서만 확인했습니다. 보드에서 계산이 학습 코드와 같다는 것은 확인했지만, 실제 진자에서의 응답은 시연 전에 사람이 지켜보며 한 번 돌려 봐야 합니다.
- D 줄을 10 ms로 받으면(`OUT 1`, 시험용) 직렬 출력이 늘어 1초에 한 번쯤 추론이 늦습니다. 시연 기본값은 20 ms(`OUT 2`)입니다.
- `CAL`·`ZERO`·`BETA`는 EEPROM을 바꾸므로 자동 시험에서 보내지 않았습니다(하드웨어 미확인).
- 시뮬레이터의 RL 응답은 11-15 모델 위에서 본 참고용입니다. 정책은 실기에서 학습했습니다.

## 프로토콜

250000 baud, 줄 단위 텍스트입니다. 각도 계산·보정·EEPROM은 [PROTOCOL.md](PROTOCOL.md)와 같습니다. **명령이나 출력 형식을 바꾸면 펌웨어, `lab/demo_sim.js`, `lab/demo.js`, `rl/demo_test.py`, 이 문서를 함께 고칩니다.**

| 명령 | 설명 |
|---|---|
| `HELLO` | 정지 후 `READY AEROPEND_DEMO 1.0`, `IMU READY`, `CFG …`, `PIDCFG …`, `MODE …` |
| `PING` | 하트비트 (응답 없음). 모터가 돌거나 유지 중일 때 1 s 동안 아무 명령이 없으면 `EVT ABORT LINK_TIMEOUT` |
| `STOP` | 모터 끔, 유지 해제 → `EVT STOP` |
| `GET` | `CFG`, `PIDCFG`, `MODE` |
| `REF <cdeg>` | 고른 제어기로 기준 유지 시작 / 기준 변경 (\|r\| ≤ 9000) → `EVT REF <cdeg>` |
| `MODE PID` / `MODE RL` | 제어기 선택 → `MODE …`. RL은 유지 중 \|e\| < 2°가 1 s 이어지면 `EVT RL ON`. RL 중 `MODE PID`면 적분항 I = u − ff로 넘김 → `EVT RL OFF` |
| `PID kp ki kd ff [umin]` | 이득(×1000)·피드포워드·최소 PWM, 돌고 있을 때도 바뀜 → `PIDCFG …` (실험용과 같은 범위) |
| `NOTCH f_cHz zz zp` / `NOTCH 0` | 출력단 노치 (실험용과 같음) |
| `LIM umax alim_cdeg` | 최대 PWM, 각도 한계 (기본 255, 10000) |
| `OUT 1` / `OUT 2` | D 줄 주기 10 / 20 ms (기본 20 ms) |
| `BENCH 0\|1` | 벤치(드라이런): 모터 핀 항상 LOW |
| `CAL` | 정지 보정(자이로 바이어스, 1000틱). 세 축 중 하나라도 최대−최소 > 2.5 °/s면 `CAL FAIL MOVING` |
| `ZERO` · `BETA <milli>` | 영점 저장, Madgwick β (EEPROM) |
| `NN x0 … x7` | (`-NnTest` 빌드만) 정책 계산 시험 → `NN <acc3> <μ×10⁶> <µs>` |

재부팅 기본값은 PID 4.30 / 13.3 / 0, ff 150, umin 40, 노치 2.35 Hz(ζz 0.15, ζp 0.7), `MODE PID`, `OUT 2`입니다. 이득은 저장하지 않습니다.

**출력**

```
D,t_ms,pwm,theta_cdeg,omega_cdps,thetaAcc_cdeg,mode,ref_cdeg,a_milli
S,state,imu,cal,zero,maxTick_us,overruns,i2cErr,gate_pct,sat,nn_us,miss,load_pct
```

- `mode`: 0 대기, 1 PID 유지, 2 RL 대기(PID로 정착 중), 3 RL. 그 줄의 PWM을 실제로 만든 제어기입니다.
- `a_milli`: 마지막으로 끝난 정책 행동 a × 1000 (RL일 때만, 아니면 0). `thetaAcc`는 D 줄을 보내는 샘플에서만 갱신합니다.
- `nn_us`: 1 s 동안 추론 시작 → 끝 벽시계 최댓값. `miss`: 이번 RL 구간의 누락 수(다음 RL 시작 때 0). `load_pct`: 1 s 중 틱이 쓴 비율. S 줄은 추론 중에는 미룹니다.
- 이벤트: `EVT REF`, `EVT RL ON`, `EVT RL OFF`, `EVT FALLBACK THETA|OMEGA`(RL 중 θ가 −10…70° 밖이거나 \|ω\| > 600 °/s → PID로 복귀, `MODE PID`), `EVT ABORT ANGLE_LIMIT|LINK_TIMEOUT|IMU_FAIL|INVALID_LINE`, `EVT STOP`.

## 안전

실험용 펌웨어의 안전장치(하트비트 1 s, 각도 한계, 최대 PWM, IMU 실패 정지, 워치독, 벤치 모드 핀 LOW)를 그대로 두었습니다. 여기에 RL 정책 PWM 범위(40–255), 결정당 변화 ±10, RL 안전 범위(벗어나면 PID 복귀)를 더했습니다. 역추력은 지원하지 않습니다.

## 정책 바꾸기

다른 스냅샷을 넣으려면 학습한 PC에서 실행합니다(리플레이 버퍼가 git에 없음).

```bash
.venv\Scripts\python.exe -m rl.demo_policy --snapshot data/<run>/checkpoints/<name> --buffer data/<run>/checkpoint/buffer.npz
```

`policy.h`와 `lab/demo_policy.js`가 다시 만들어집니다. 관측 정의나 config(k<sub>I</sub>, du_max, 정규화)가 rl8과 다른 실행이라면 펌웨어 `rlStep`의 상수도 고쳐야 합니다. 그 뒤 `Build-Demo.ps1 -NnTest -Upload` → `rl.demo_test --nn 300` → 기본 빌드 업로드 순서로 확인합니다.
