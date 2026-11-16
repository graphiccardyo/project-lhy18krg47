# 시연용 펌웨어와 UI (AEROPEND_DEMO 2.0)

발표·시연에서 **PID와 강화학습(RL) 제어기를 같은 장치에서 번갈아 보여 주기** 위한 펌웨어와 UI입니다. 보드 혼자서 두 제어기 중 하나로 기준 각도를 유지하므로 PC에 Python·PyTorch가 필요 없습니다. 브라우저 한 장으로 제어기, 기준 각도, PID 이득을 바꾸고 시험 결과를 저장합니다.

**2.0 (11-17)은 11-16 모터 교체 뒤의 RL 방식인 잔차 RL을 따릅니다** ([RL_HISTORY.md D13](RL_HISTORY.md#d13-모터-교체-뒤-잔차-rl-11-16)).
- 보드가 학습 때의 PI + 노치(4.30 / 13.3, ff 140, 2.35 Hz)를 계산하고, 정책이 잔차 r(±15 PWM)을 노치 앞에서 더합니다. RL 펌웨어 `AEROPEND_RL 1.2`의 잔차 모드와 같은 식입니다.
- 1.0(11-16)은 11-15 rl8 증분 정책(64-64)을 넣은 것이었고, 커밋 `a35dfe4`에 있습니다.

| 파일 | 내용 |
|---|---|
| [firmware/aeropendulum_demo/aeropendulum_demo.ino](../firmware/aeropendulum_demo/aeropendulum_demo.ino) | 펌웨어: 센서·Madgwick·PID(+노치)·고정 RL 정책(잔차)·안전장치 |
| [firmware/aeropendulum_demo/policy.h](../firmware/aeropendulum_demo/policy.h) | 정수로 양자화한 정책 + 실행 설정(잔차 크기, 출력 범위, 정규화, 학습 때 PID) — 자동 생성 |
| [lab/demo.html](../lab/demo.html) · [lab/demo.js](../lab/demo.js) | 시연 UI (Web Serial) |
| [lab/demo_sim.js](../lab/demo_sim.js) · [lab/demo_policy.js](../lab/demo_policy.js) | 보드 없이 쓰는 시뮬레이터(같은 정책, 11-16 모터 모델) |
| [rl/demo_policy.py](../rl/demo_policy.py) | 실행 폴더 → 정책 양자화, `policy.h`·`demo_policy.js` 생성 |
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
   1. 제어기 **PID**, 기준 20° → **시작**. 보드 기본 이득은 권장 PI + 2.35 Hz 노치(Kp 4.30, Ki 13.3, ff 140)이고, 최대 PWM은 230(새 모터 보호)입니다.
   2. **강화학습(RL)**을 누르면 PID로 기준 근처에 간 뒤, |오차| < 2°를 1초 유지하는 순간 RL로 넘어갑니다(학습 때 에피소드를 시작한 방식과 같음).
      - RL 중에는 **학습 때의 PID(화면의 "RL 정책" 칸, 고정)**에 정책 잔차를 더합니다.
      - 그래프에서 RL 구간은 초록 굵은 선이고, "RL 잔차 r" 칸에 지금 더하고 있는 값이 보입니다.
   3. 기준 15/20/25/30° 버튼으로 스텝 응답을 비교합니다. 학습 범위는 15–30°입니다.
   4. **PID**를 다시 누르면 잔차만 빠지고 같은 적분에서 이어 갑니다. 사용자 이득이 RL 기반 PID와 다르면 그 이득으로 바뀝니다.
      - 이득·ff·최소 PWM·노치(f, ζz, ζp)를 바꾸거나 설정을 불러온 뒤 **이득 적용**을 누릅니다. "보드에 적용된 값" 줄이 보드의 `PIDCFG` 응답입니다.
      - 사용자 이득은 PID 모드에만 쓰입니다.
   5. **정지**(Esc / Space)로 모터를 끕니다. 창을 닫거나 연결이 끊겨도 보드가 1초 안에 멈춥니다.
   6. **새 모터 보호**: 모터가 120 s 넘게 연속으로 돌면 UI가 정지하고 90 s 쉬라고 알립니다(`rl/configs/motor2.json`의 가동·휴식). 보드가 강제하지는 않습니다.
4. **저장된 설정**은 실험 UI(⑥ 탭)의 "저장된 설정"과 같은 것입니다. 같은 브라우저 저장소(`aeroLab.presets.v1`)와 같은 JSON 파일 형식(`aeropend-lab-settings`)을 씁니다.
   - 실험 UI에서 저장한 설정을 고르고 **불러오기**를 누르면 그중 이득·ff·최소 PWM·노치만 칸에 들어갑니다. 보드에 보내려면 **이득 적용**을 누릅니다.
   - **현재 값 저장**: 같은 이름이 있으면 그 설정의 다른 시험 값·안전 한계는 그대로 두고 이득만 바꿉니다. 새 이름이면 이득만 담은 설정이 생기고, 실험 UI에서도 보입니다.
   - **파일로 내보내기 / 파일에서 가져오기**는 실험 UI와 같은 파일을 주고받습니다. 다른 PC나 팀원과 나눌 때 씁니다.
   - 브라우저 저장소는 여는 방식마다 따로입니다. `Start-Lab.cmd`(파일로 열기)로 저장한 설정은 `demo.html`도 파일로 열어야 보이고, `http://localhost:8765`로 열면 따로 저장됩니다. 섞이면 파일로 내보내서 가져오세요.
5. **CSV 저장**(그래프 아래)은 연결 후 기록 전체(최대 30분, 50 Hz)를 `aero_DEMO_*.csv`로 내려받습니다(열 `rl_res_pwm` = RL 잔차).

### 시험: 일정 시간 유지 / 스텝 입력

오른쪽 **시험** 칸에서 지금 고른 제어기(PID 또는 RL)로 정해진 시험을 돌리고 결과를 저장합니다. 펌웨어는 그대로이고, 페이지가 `REF`를 보내며 보드 D 줄(20 ms, 보드 시각)을 기록합니다.

| 시험 | 설정 | 순서 |
|---|---|---|
| 일정 시간 유지 | 기준, 유지 시간(기본 20° · 30 s) | 기준으로 감 → 정착 → T초 기록 |
| 스텝 입력 | r0 → r1, 앞 시간, 스텝 뒤 시간, r0 복귀 시간(기본 20 → 25°, 5 s, 10 s, 0 s) | r0으로 감 → 정착 → 앞 시간 기록 → `REF r1` → 스텝 뒤 시간 → (복귀 > 0이면) `REF r0` → 복귀 시간 |

- **정착**: |오차| < 2°를 1초 유지해야 합니다. RL은 RL로 넘어간 뒤(mode 3)부터 셉니다. 40초 안에 안 되면 중단합니다.
- 시험 중에는 제어기·기준·이득 칸을 잠급니다. 끝나도 마지막 기준을 계속 유지하니 **정지**로 끄세요. 30 s 유지 시험은 준비 시간을 더하면 새 모터의 120 s 가동 한도에 가까워지니, 시험 사이에 쉬세요.
- **중단으로 남는 경우**: 정지, `EVT ABORT`, RL 시험 중 FALLBACK, 연결 끊김, **시험 중단** 버튼. 기록이 있으면 `_partial` CSV로 남깁니다.
- 구간(phase 1 앞, 2 스텝 뒤, 3 복귀)은 보드 D 줄의 기준 값이 바뀐 줄로 나눕니다. 그래서 PC 쪽 지연은 결과에 들어가지 않습니다.

**결과** (표에 쌓이고, **결과 표 CSV 저장**으로 한 파일로 내보냄):

- 스텝: 상승시간(10–90%), 오버슈트, 정착 시간(±5% 띠), 편향(= r1 − 스텝 구간 마지막 2 s 평균 θ), σ(θ)·PWM σ(마지막 3 s), 스텝 구간 RMS 오차.
  - 상승·OS·정착은 θ를 0.1 s 이동평균한 값으로 셉니다.
  - 정착이 `> N s`이면 끝까지 ±5% 띠에 들어오지 못한 것입니다(5° 스텝이면 ±0.25°).
- 유지: 편향(평균 오차), σ(e), RMS, 최대 |e|, PWM 평균·σ, 샘플당 평균 |ΔPWM|(20 ms 간격).

**CSV** `aero_DEMO_<STEP|HOLD>_<PID|RL>_<날짜_시각>.csv`는 실험 UI와 같은 형식입니다(끝나면 자동 저장, 끌 수 있음).

```
# aeropendulum_demo; name=STEP_RL; type=STEP; ctrl=RL; params=r0=20 r1=25 pre_s=5 hold_s=10 post_s=0; Ts=0.02; …; beta=0.01; …; status=done; fw=AEROPEND_DEMO 2.0; start=…
# pid; kp=4.3; ki=13.3; kd=0; ff=140; umin=40; notch_cHz=235; zz=150; zp=700; source=RL 기반 PID (policy.h)
# rl; policy=20261117_rl5/manual_00081_041012 ep81 res=15 ulo=40 uhi=230 kp=… (RL 시험만, PID 시험은 none)
# result; tr=…; os=…; ts5=…; bias=…; std=…; rms=…; pwm_std=…; pwm_mean=…
t_s,pwm,duty_pct,theta_deg,omega_dps,theta_acc_deg,phase,ref_deg,mode,rl_res_pwm
```

- `# pid` 줄은 **실제로 쓴 이득**입니다. PID 시험은 사용자 이득, RL 시험은 RL 기반 PID입니다.
- `analysis/aero_analyze.py`의 `load`와 `aero_closedloop.read_pid`로 읽힙니다(확인함).
- 다만 샘플 간격이 20 ms라, 10 ms를 가정한 처리(자이로 적분각, `aero_closedloop`의 모델 재현)에는 그대로 쓰면 안 됩니다.
- 실험 데이터로 남길 것은 `data/<날짜>_runN/`에 옮기세요.

### 시연 전 점검

- 자이로 편향이 온도에 따라 흔들립니다([RESULTS.md §3](RESULTS.md#자세-추정-imu-문제와-대책)).
  - 시연 직전에 진자가 멈춘 상태에서 정지 보정을 하세요. 이 펌웨어에도 `CAL` 명령(5초, 자이로 바이어스만)이 있습니다.
  - 정밀 점검(`python -m rl.imu_check`)은 RL 펌웨어로 바꿔서 합니다.
- RL 정책은 **β 0.01**로 학습했습니다. 연결했을 때 β가 0.01이 아니면 UI가 경고합니다.
- 진자가 정지 위치에서 ±1–5° 바뀌면(케이블·피벗 마찰) `ZERO` 명령으로 영점을 다시 잡을 수 있습니다. 이 명령은 EEPROM을 바꾸므로 필요할 때만 씁니다.

## RL 정책을 보드에 넣은 방법 (2.0, 잔차 RL)

- **정책**: [data/20261117_rl5](../data/20261117_rl5/) **ep 81**(스냅샷 `manual_00081_041012`, 학습 마지막). 확인 시험(`rl.play --pid`, 평가 시나리오 ×2)에서 6쌍 모두 RL이 낮았고 평균 RMS 1.68° vs PID 1.85°입니다([RL_HISTORY D13](RL_HISTORY.md#d13-모터-교체-뒤-잔차-rl-11-16)). 펌웨어 1.2(잔차가 노치 통과)로 학습한 실행입니다.
  - 양자화 오차: 리플레이 관측 107 892개에서 잔차 r 오차 RMS 0.08 PWM, 최대 0.55 PWM.
  - SAC 액터 8 → 16 → 16 → μ이고, 결정적 행동 a = tanh(μ)만 씁니다.
  - 어떤 정책이 들어 있는지는 `policy.h`의 `RL_TAG`와 보드 `RLPOL` 줄로 확인합니다.
- **설정은 실행 폴더에서 자동으로**: `rl.demo_policy`가 그 실행의 `config.json`에서 다음 값을 `policy.h` 상수로 넣습니다.
  - 잔차 크기 `res_max`(15), RL 출력 범위 `u_lo`/`u_hi`(40/230), 기본 최대 PWM(230)
  - 관측 정규화, ω 저역통과·오차 평균 계수, 안전 범위(SAFE)
  - **학습 때 보드 PID**(4.30 / 13.3 / 0, ff 140, umin 40, 노치 2.35 Hz / 0.15 / 0.7)

  펌웨어 구조와 맞지 않는 실행이면 멈춥니다. 증분 모드, dec/lag ≠ 2, act_pow ≠ 2, 관측 8개가 아닌 경우입니다.
- **계산** (`rl/env.py` `_decide_residual`, RL 펌웨어 1.2 잔차 모드와 같음):
  - 매 샘플 u = clamp(PID<sub>RL</sub>(ref) + r, [40, min(230, LIM)])이고, r은 노치 앞에서 PID 항과 함께 더합니다.
  - 결정 샘플(20 ms마다)에 관측 8개를 만듭니다: 오차, 각도, 10 Hz로 거른 각속도, Δθ, **이번 샘플 PWM**, **지금 적용된 잔차의 행동 a**, 3 s 오차 평균, tanh(e/0.5°).
  - 추론 결과 r = lround(15·a|a|)를 k+2 샘플(20 ms 뒤)에 적용합니다. 학습 때 PC 경로의 dec 2 / lag 2와 같습니다. 그때까지 못 끝내면 그 전 r을 유지하고 `miss`를 셉니다(학습기와 같음).
  - 관측에 쓰는 θ·ω는 학습 때 D 줄처럼 0.01 단위로 반올림합니다.
- **PID 묶음 둘**: 사용자 이득(`PID`/`NOTCH`)과 RL 기반 이득(고정)을 따로 둡니다.
  - 정책은 학습 때 PID 위의 잔차라서, 사용자가 이득을 바꿔도 RL 조건은 그대로입니다.
  - 전환할 때 두 묶음의 이득·노치가 같으면 ff 차이만 적분으로 옮겨 출력이 그대로 이어집니다.
  - 다르면 RL 펌웨어가 에피소드를 시작할 때처럼 노치 상태 0, 적분 = 직전 출력 − ff로 시작합니다.
  - 1.2와 다른 점: 같은 이득이면 노치 상태를 0으로 지우지 않고 이어 갑니다(전환 순간 튐 없음).
- **양자화**는 1.0과 같은 방식입니다(입력 int16 ×2¹¹, 1층 14비트, 2층 int8 + 편향 보정, 3층 10비트, 활성값 int16, 뉴런마다 시프트). rl5 리플레이 관측 48 951개에서 오차는 다음과 같습니다.
  - 행동 오차 RMS 0.006, 최대 0.069
  - **잔차 오차 RMS 0.08 PWM**, 최대 0.8 PWM. |e| < 1°에서 평균 −0.0003 PWM이고, 잔차를 정수로 반올림하는 크기(±0.5)보다 작습니다.
  - 스냅샷마다 다릅니다(ep 10 스냅샷은 RMS 0.18 PWM, rl4 best는 0.07 PWM).
- **계산 시간**: 망이 작아 추론이 1 ms 안쪽입니다(1.0의 64-64는 4.2–5.8 ms). 대신 잔차 모드는 샘플마다 PID를 계산하므로, θ<sub>acc</sub>(진단용)는 RL 펌웨어 1.2처럼 샘플이 아닌 틱에서 계산합니다.
- 플래시 26 852 B (83%, `NN` 시험 명령 포함), RAM 993 B입니다. 1.0에서 뺀 기능(가진 신호·CLSTEP·RL 스트리밍 등)은 그대로 빠져 있습니다.

## 검증

`.venv\Scripts\python.exe -m rl.demo_test --port COM4 --nn 300`으로 확인합니다. 벤치 모드(모터 핀 LOW)에서만 돌고 EEPROM은 바꾸지 않습니다. `--run`은 보드에 올린 `policy.h`를 만든 실행과 같게 줍니다(기본 rl5).

시험 내용(2.0):
- **명령·설정 줄**: `RLPOL` = `policy.h`, `PIDCFG` 기본값, umax 230.
- **`NN` 정책 계산**: 보드의 정수 결과가 `QuantPolicy`와 정확히 같은지 봅니다.
- **RL 끝-끝**: 사용자 이득을 Kp 0.9, 노치 끔으로 일부러 다르게 둔 채 RL을 돌립니다. 그리고 다음을 비교합니다.
  - 결정마다 잔차 r이 float32 에뮬레이터와 같은지
  - PWM이 RL 기반 PID + 잔차(노치 앞) 재계산과 같은지(±1)
  - 관측 8개가 `rl/env.py`(잔차 모드)와 같은지
  - 누락 수가 보드 보고와 같은지
- **시연 설정(D 줄 20 ms)**: 누락 0, 추론 시간, 틱 시간.

| 항목 (2.0, 2026-11-17) | 결과 |
|---|---|
| 컴파일 | 경고 0(코어 제외), 26 852 B / RAM 993 B |
| 에뮬레이터 ↔ `rl/env.py` 잔차 모드 (합성 D 줄 1200줄, 오프라인) | 599결정에서 관측 8개 최대 차이 2.9e-6, 잔차·PWM 경로 일치 |
| JS 시뮬레이터 정책 = Python (200개) | 차이 < 1e-6 |
| UI (시뮬레이터) | RLPOL 표시, 사용자 이득을 손튜닝 PID로 바꾼 뒤 RL·PID 스텝 시험 → RL 시험 CSV `# pid` = RL 기반 PID, `rl_res_pwm` 기록 |
| **보드 벤치 시험** (ep 81, `rl.demo_test --nn 300`, 벤치 모드) | **37개 모두 통과.** `NN` 300개 정수 누산 일치(μ 차 5.4e-7, 추론 0.7–0.9 ms). RL 끝-끝 497결정 잔차 r = 에뮬레이터, PWM 재계산 99.3% 일치(최대 차 1), 관측 = `env.py` 2.4e-7, 누락 0. 시연 설정(D 줄 20 ms): miss 0, 추론 벽시계 최대 5.0 ms, maxTick 4392 µs, 오버런 0 |

1.0(rl8, 11-16) 보드 벤치 결과는 다음과 같았습니다(같은 방식의 시험).
- `NN` 정수 일치
- 497결정 보드 = 에뮬레이터
- 관측 = `env.py` 1.1e-6
- 시연 설정 누락 0, 추론 최대 14.3 ms

**확인하지 못한 것**

- **모터를 돌린 실기 시연은 1.0·2.0 모두 아직 하지 않았습니다.** 실제 진자에서의 응답은 시연 전에 사람이 지켜보며 한 번 돌려 봐야 합니다.
- `CAL`·`ZERO`·`BETA`는 EEPROM을 바꾸므로 자동 시험에서 보내지 않았습니다(하드웨어 미확인).
- 시뮬레이터의 RL 응답은 11-16 모터 모델(가짜 보드와 같은 선형 모델 + 정적 지도) 위의 참고용입니다.

## 프로토콜

250000 baud, 줄 단위 텍스트입니다. 각도 계산·보정·EEPROM은 [PROTOCOL.md](PROTOCOL.md)와 같습니다. **명령이나 출력 형식을 바꾸면 펌웨어, `lab/demo_sim.js`, `lab/demo.js`, `rl/demo_test.py`, 이 문서를 함께 고칩니다.**

| 명령 | 설명 |
|---|---|
| `HELLO` | 정지 후 `READY AEROPEND_DEMO 2.0`, `IMU READY`, `CFG …`, `PIDCFG …`, `RLPOL …`, `MODE …` |
| `PING` | 하트비트 (응답 없음). 모터가 돌거나 유지 중일 때 1 s 동안 아무 명령이 없으면 `EVT ABORT LINK_TIMEOUT` |
| `STOP` | 모터 끔, 유지 해제 → `EVT STOP` |
| `GET` | `CFG`, `PIDCFG`, `RLPOL`, `MODE` |
| `REF <cdeg>` | 고른 제어기로 기준 유지 시작 / 기준 변경 (\|r\| ≤ 9000) → `EVT REF <cdeg>` |
| `MODE PID` / `MODE RL` | 제어기 선택 → `MODE …`. RL은 유지 중 \|e\| < 2°가 1 s 이어지면 `EVT RL ON`(RL 기반 PID로 바꾸고 잔차 시작). RL 중 `MODE PID`면 잔차를 빼고 사용자 이득으로 → `EVT RL OFF` |
| `PID kp ki kd ff [umin]` | 사용자 이득(×1000)·피드포워드·최소 PWM, 돌고 있을 때도 바뀜 → `PIDCFG …` (실험용과 같은 범위). RL 중에는 쓰이지 않음 |
| `NOTCH f_cHz zz zp` / `NOTCH 0` | 사용자 이득의 출력단 노치 (실험용과 같음) |
| `LIM umax alim_cdeg` | 최대 PWM, 각도 한계 (기본 230, 10000) |
| `OUT 1` / `OUT 2` | D 줄 주기 10 / 20 ms (기본 20 ms) |
| `BENCH 0\|1` | 벤치(드라이런): 모터 핀 항상 LOW |
| `CAL` | 정지 보정(자이로 바이어스, 1000틱). 세 축 중 하나라도 최대−최소 > 2.5 °/s면 `CAL FAIL MOVING` |
| `ZERO` · `BETA <milli>` | 영점 저장, Madgwick β (EEPROM) |
| `NN x0 … x7` | 정책 계산 시험 → `NN <acc3> <μ×10⁶> <µs>` (RL 중에는 `ERR BUSY`, `NN_TEST 0`으로 빌드하면 없음) |

재부팅 기본값은 다음과 같습니다. 이득은 저장하지 않습니다.
- 사용자 PID 4.30 / 13.3 / 0, ff 140(= RL 기반 PID), umin 40, 노치 2.35 Hz(ζz 0.15, ζp 0.7)
- 최대 PWM 230, `MODE PID`, `OUT 2`

**출력**

```
D,t_ms,pwm,theta_cdeg,omega_cdps,thetaAcc_cdeg,mode,ref_cdeg,res_pwm
S,state,imu,cal,zero,maxTick_us,overruns,i2cErr,gate_pct,sat,nn_us,miss,load_pct
RLPOL <실행/스냅샷 epN> res=15 ulo=40 uhi=230 kp=4.300 ki=13.300 kd=0.000 ff=140 notch=235 zz=150 zp=700 umin=40
```

- `mode`: 0 대기, 1 PID 유지, 2 RL 대기(PID로 정착 중), 3 RL. 그 줄의 PWM을 실제로 만든 제어기입니다.
- `res_pwm`: 지금 적용된 RL 잔차 r [PWM] (RL일 때만, 아니면 0). 1.0의 `a_milli`(행동 ×1000) 자리입니다.
- `thetaAcc`는 샘플이 아닌 틱(5 ms 전 가속도)에서 갱신합니다.
- `nn_us`: 1 s 동안 추론 시작 → 끝 벽시계 최댓값. `miss`: 이번 RL 구간의 누락 수(다음 RL 시작 때 0). `load_pct`: 1 s 중 틱이 쓴 비율. S 줄은 추론 중에는 미룹니다.
- 이벤트:
  - `EVT REF`, `EVT RL ON`, `EVT RL OFF`, `EVT STOP`
  - `EVT FALLBACK THETA|OMEGA`: RL 중 θ가 SAFE 범위(−10…70°) 밖이거나 \|ω\| > 600 °/s이면 PID로 복귀하고 `MODE PID`가 됩니다.
  - `EVT ABORT ANGLE_LIMIT|LINK_TIMEOUT|IMU_FAIL|INVALID_LINE`

## 안전

실험용 펌웨어의 안전장치(하트비트 1 s, 각도 한계, 최대 PWM, IMU 실패 정지, 워치독, 벤치 모드 핀 LOW)를 그대로 두었습니다. 여기에 다음을 더했습니다.
- 기본 최대 PWM 230(새 모터 보호)
- RL 출력 범위 40–230, 잔차 ±15 PWM
- RL 안전 범위(벗어나면 PID 복귀)
- UI의 120 s 가동 경고

역추력은 지원하지 않습니다.

## 정책 바꾸기

학습한 PC에서 실행합니다(체크포인트·리플레이 버퍼가 git에 없음). 잔차 RL 실행만 됩니다.

```bash
.venv\Scripts\python.exe -m rl.demo_policy --run data/20261117_rl5 --snapshot manual_00081_041012
```

`policy.h`(가중치 + 실행 설정)와 `lab/demo_policy.js`가 다시 만들어집니다. 펌웨어 상수는 헤더에서 오므로 소스는 고치지 않아도 됩니다. 확인 순서는 다음과 같습니다.

```bash
powershell -ExecutionPolicy Bypass -File scripts/Build-Demo.ps1 -Upload -Port COM4
```

```bash
.venv\Scripts\python.exe -m rl.demo_test --port COM4 --nn 300 --run data/20261117_rl5
```
