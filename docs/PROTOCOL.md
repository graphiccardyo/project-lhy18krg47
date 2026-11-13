# 시리얼 프로토콜 · 데이터 형식

펌웨어 [firmware/aeropendulum_lab/aeropendulum_lab.ino](../firmware/aeropendulum_lab/aeropendulum_lab.ino) ↔ UI [lab/app.js](../lab/app.js) ↔ 시뮬레이터 [lab/sim.js](../lab/sim.js)가 같은 프로토콜을 씁니다. **명령이나 출력 형식을 바꾸면 세 곳과 이 문서, 시험 스크립트를 같이 바꿔야 합니다**([DEVELOPMENT.md](DEVELOPMENT.md#기능-추가-절차)).

- **250000 baud**, 8N1, 줄 단위 ASCII. PC → 보드는 `\n`(또는 `\r\n`), 보드 → PC는 `\r\n`입니다.
- 명령 한 줄은 최대 71자입니다. 그보다 길거나 제어문자가 섞이면 `ERR INVALID_LINE`이 나고, 실행 중인 출력은 정지됩니다.
- 줄 중간에 250 ms 넘게 멈추면 받은 부분을 버립니다(`ERR COMMAND_TIMEOUT`).
- 모든 정수 인자는 공백으로 구분합니다. 실수는 고정소수점으로 보냅니다(예: 각도 cdeg = 0.01°, 이득 ×1000).
- **하트비트**: 모터 출력·시험·폐루프 중에 1 s 동안 아무 명령이 없으면 정지합니다(`EVT ABORT LINK_TIMEOUT`). UI는 0.25 s마다 `PING`을 보냅니다.

## 타이밍

| 항목 | 값 |
|---|---|
| 센서 읽기 + Madgwick | 200 Hz (5 ms 틱) |
| 샘플 주기 Ts | 5 ms × `STREAM` div (기본 div 2 → **10 ms**) |
| 한 샘플의 처리 순서 | θ·ω 계산 → 가진 신호 또는 PID → PWM 출력 → `D` 줄 전송 |
| `A` 줄 | 10샘플마다 (다음 틱에 전송) |
| `S` 줄 | 1 s마다 |

`D` 줄의 PWM은 같은 줄의 θ를 측정한 **직후** 적용된 값입니다(영차 유지). 따라서 θ(k+1)부터 영향을 줍니다.

## 명령

### 연결·상태

| 명령 | 응답 | 설명 |
|---|---|---|
| `HELLO` | `READY AEROPEND_LAB 2.1`, `IMU …`, `CFG …`, `CALVAL …` | 출력 정지 후 상태 보고. 2.1 = 폐루프 명령 포함 (2.0 은 개루프만) |
| `PING` | (없음) | 하트비트 |
| `GET` | `CFG …`, `CALVAL …`, `PIDCFG …` | 현재 설정 |
| `STOP` | `EVT STOP` | 출력·시험·보정·폐루프 모두 정지 |

### 출력·한계

| 명령 | 범위 | 설명 |
|---|---|---|
| `U <pwm>` | −255…255 | 수동 출력. 음수는 역추력 허용 시에만 적용. 폐루프 유지 해제 |
| `LIM <pwm_max> <angle_cdeg> [rev]` | 0–255, 500–18000, 0/1 | 최대 PWM, 각도 한계, 역추력 허용 (재부팅하면 255 / 120° / 금지) |
| `STREAM <div>` | 1–20 | 샘플 주기 = 5 ms × div. 시험 중에는 바꿀 수 없음 |
| `BENCH <0\|1>` | | 드라이런: 시험 로직은 그대로 돌지만 모터 핀은 항상 LOW. IMU가 없으면 θ = 0. 재부팅하면 꺼짐 |

### 센서·보정 (EEPROM 저장)

| 명령 | 설명 |
|---|---|
| `AXIS <X\|Y\|Z> <+\|->` | 진자 피벗 축과 부호 |
| `ZERO` | 현재 자세를 θ = 0으로 (축이 중력과 평행하면 `ERR AXIS_PARALLEL_TO_GRAVITY`) |
| `CAL` | 정지 자세 보정: 1000샘플 평균으로 자이로 바이어스 (자세 무관) |
| `CAL FLAT` | 수평면 보정: +Z 위로 평평하게 놓고 자이로 바이어스 + 가속도 오프셋 |
| `CAL CLEAR` | 자이로 바이어스 0 (가속도 오프셋 유지) |
| `ACCOFF <x> <y> <z>` / `ACCOFF DEFAULT` | 가속도 오프셋 [mg, ±500] / 기본 저장값 (−17.1, −22.0, +147.5 mg) |
| `BETA <β×1000>` | Madgwick β, 1–1000 (기본 20 = 0.02) |
| `DLPF <0..6>` | MPU6050 디지털 저역통과 (기본 4 = 21 Hz) |
| `CFG BACKUP` / `CFG RESTORE` | 현재 설정을 EEPROM 예비 영역에 복사 / 되돌리기 (`ERR NO_BACKUP`) |
| `FACTORY` | 설정 초기화 (축 X+, β 0.02, DLPF 4, 보정·영점 없음, 가속도 오프셋 = 기본값) |

### 개루프 시험 (`RUN …`, 각도 θ 외에는 PWM·ms 단위)

| 명령 | 구간(phase) |
|---|---|
| `RUN STEP pwm0 pwm1 pre_ms step_ms post_ms` | 1 = pwm0, 2 = pwm1, 3 = pwm0 |
| `RUN STAIR start end inc dwell_ms` | 계단 번호 1, 2, … (inc 1–255, 최대 250계단) |
| `RUN CHIRP pwm0 amp f0_mHz f1_mHz pre_ms dur_ms log` | 1 = pre(pwm0), 2 = pwm0 + amp·sin. log = 1이면 지수 스윕 (f1 ≤ 50 Hz) |
| `RUN PRBS pwm0 amp bit_ms pre_ms dur_ms` | 1 = pre, 2 = pwm0 ± amp (9비트 LFSR, x⁹ + x⁵ + 1) |

모든 시험은 최대 120000샘플입니다. 시작할 때 `EVT RUN <TYPE> total=<샘플수> ts_ms=<Ts>`, 끝나면 `EVT DONE`을 보냅니다. 시험 중 다른 `RUN`, `U`, `STREAM`, `REF` 명령은 `ERR BUSY`입니다.

### 폐루프

| 명령 | 설명 |
|---|---|
| `PID <kp×1000> <ki×1000> <kd×1000> <ff>` | 이득 (Kp ≤ 200, Ki ≤ 500, Kd ≤ 50, ff 0–255). 실행 중에도 변경 가능. **저장 안 함**(재부팅하면 0) → `PIDCFG …` |
| `NOTCH <f×100> <ζz×1000> <ζp×1000>` / `NOTCH 0` | 출력 노치 (0.1–20 Hz) / 끔. 예: `NOTCH 347 150 700` |
| `REF <cdeg>` | 폐루프로 기준 유지 (\|r\| ≤ 170°). 다시 보내면 기준만 바뀜 → `EVT REF <cdeg>`. `STOP`·`U`로 해제 |
| `RUN CLSTEP r0 r1 pre_ms step_ms post_ms` | 폐루프 스텝 (r: cdeg). phase 1 = r0, 2 = r1, 3 = r0 |
| `RUN RELAY r_cdeg d_pwm hyst_cdeg pre_ms dur_ms` | phase 1: PID로 r 유지하며 후반 출력 평균 → 중심. phase 2: 중심 ± d 릴레이 (히스테리시스 hyst) |

제어 법칙 (매 샘플, Ts = 5 ms × div):
```
e = r − θ
ω_f ← ω_f + Ts/(τ + Ts)·(ω − ω_f),  τ = 1/(2π·10 Hz)
v = Kp·e + I − Kd·ω_f
v = Notch(v)                    (켜져 있으면; 쌍선형 변환 + 중심주파수 프리워핑)
u = clamp(ff + v, [0 또는 −PWMmax, PWMmax])
I ← I + Ki·e·Ts  (u 가 포화되지 않았거나, 포화 반대 방향으로 오차가 있을 때만), |I| ≤ 255
```

## 보드 → PC 출력

| 줄 | 형식 | 주기 |
|---|---|---|
| `D` | `D,t_ms,pwm,theta_cdeg,omega_cdps,thetaAcc_cdeg,phase[,ref_cdeg]` | 매 샘플. `ref`는 폐루프일 때만 |
| `A` | `A,ax,ay,az,gx,gy,gz` (보정 전, mg / cdps) | 10샘플마다 |
| `S` | `S,state,imu,cal,zero,maxTick_us,overruns,i2cErr,gate_pct,sat` | 1 s |
| `CFG` | `CFG axis=Z sign=- beta=20 dlpf=4 div=2 ts_ms=10 umax=255 alim=12000 rev=0 bench=0 arng=16 grng=2000 cal=1 zero=1` | 설정 변경 시 |
| `CALVAL` | `CALVAL <cal> <gbx gby gbz [°/s]> <aox aoy aoz [g]> g0 <x y z>` | 보정 변경 시 |
| `PIDCFG` | `PIDCFG kp=7.850 ki=23.900 kd=0.000 ff=150 notch=347 zz=150 zp=700` | 이득 변경 시 |
| `CAL` | `CAL START mode=STILL n=1000`, `CAL PROG <pct>`, `CAL OK mode=… gstd=… astd=… anorm=…`, `CAL FAIL <code> …` | 보정 중 |
| `EVT` | `RUN <type> total= ts_ms=`, `DONE`, `STOP`, `REF <cdeg>`, `ABORT <ANGLE_LIMIT\|LINK_TIMEOUT\|IMU_FAIL\|INVALID_LINE>` | |
| `IMU` | `IMU READY addr=0x68 i2c=400` / `IMU ERROR NOT_FOUND no_ack(…)` / `IMU ERROR READ` | 상태 변화 시 (같은 오류는 한 번만) |
| `ERR` | `BUSY`, `BAD_ARGS`, `IMU_NOT_READY`, `UNKNOWN_COMMAND`, `UNKNOWN_PROFILE`, `TOO_MANY_STEPS`, `BAD_DURATION`, `AXIS_PARALLEL_TO_GRAVITY`, `COMMAND_TIMEOUT`, `INVALID_LINE`, `NO_BACKUP` | |
| `NOTE` | `NOTE CFG_MIGRATED v3->v4 …` | EEPROM 형식 갱신 시 1회 |

- `S.state`: 0 대기, 1 수동 출력, 2 시험, 3 보정, 4 폐루프 유지.
- `gate_pct`: 1 s 동안 가속도 크기가 1 g ± 0.15를 벗어나 Madgwick이 자이로만 적분한 비율로, 진동 지표입니다.
- `sat`: 1 s 동안 센서 범위(±16 g / ±2000 °/s)에 닿은 샘플 수입니다. 0이 아니면 그 구간 데이터는 신뢰하기 어렵습니다.
- `maxTick_us`: 한 틱의 최대 처리 시간입니다. 5000 µs보다 작아야 하며, 현재 약 3.7–4.3 ms입니다.

## 각도 계산

- 센서: ±16 g / ±2000 °/s, DLPF 설정값, I²C 400 kHz(응답이 없으면 100 kHz).
- Madgwick IMU 필터: 200 Hz, 실제 dt 사용. 가속도는 약 5 Hz 1차 저역통과 후 입력하고, |a|가 1 g ± 0.15를 벗어나면 그 틱은 가속도 보정을 생략합니다.
- θ: 쿼터니언에서 구한 중력 방향 g를 피벗 축 k에 수직인 평면에 투영해, 영점 중력 g₀와의 사이각을 구합니다(±180°, 짐벌락 없음). 부호는 `AXIS`의 부호입니다.
- ω: 바이어스를 보정한 자이로의 k축 성분 × 부호입니다. θ의 미분과 부호가 같습니다.
- θ<sub>acc</sub>: 가속도만으로 같은 방식으로 구한 각도입니다(진단용).

## CSV (UI가 저장)

```
# aeropendulum_lab; name=CHIRP_143817; type=CHIRP; params=u0=150 amp=25 f0=0.1 f1=7 pre_s=12 dur_s=60 log=1; Ts=0.01; input=pwm(0-255); axis=Z-; beta=0.02; dlpf=4; cal=1; zero=1; sim=0; bench=0; status=done; fw=AEROPEND_LAB 2.0; start=…
[# pid; kp=7.85; ki=23.9; kd=0; ff=150; notch_cHz=347; zz=150; zp=700]      ← 폐루프 기록만
t_s,pwm,duty_pct,theta_deg,omega_dps,theta_acc_deg,phase[,ref_deg]
0.000,150,58.82,0.83,1.20,3.10,1
```
- 파일 이름: `aero_<TYPE>_<YYYYMMDD>_<HHMMSS>.csv`. 중단된 기록은 `_partial`이 붙습니다.
- `phase`는 위 명령 표와 같습니다(0 = 대기/수동).
- 읽기: MATLAB `aero_load`, Python `analysis/aero_analyze.load`, UI ⑤ **CSV 불러오기**.

## EEPROM

| 주소 | 내용 |
|---|---|
| 0 | `Config` v4 (magic 0xAE01, 축·부호·DLPF·β·자이로 바이어스·가속도 오프셋·영점 g₀·플래그, CRC-8) |
| 128 | `CFG BACKUP` 사본 |

v3 형식(β·DLPF 기본값이 다르던 시절)은 부팅할 때 v4로 옮기면서 축·보정·영점은 유지하고, β = 0.02, DLPF = 4로 바꿉니다. 이득(`PID`, `NOTCH`)은 저장하지 않습니다.
