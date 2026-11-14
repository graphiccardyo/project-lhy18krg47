# RL 펌웨어 프로토콜 (AEROPEND_RL 1.0)

강화학습(목표 7)용 펌웨어 [firmware/aeropendulum_rl/aeropendulum_rl.ino](../firmware/aeropendulum_rl/aeropendulum_rl.ino) ↔ Python 학습기 [rl/](../rl/) ↔ 가짜 보드 `rl/fake_board.py`가 쓰는 프로토콜입니다. **명령이나 출력 형식을 바꾸면 세 곳과 이 문서, `rl/bench_test.py`를 같이 고칩니다.**

실험용 펌웨어(`AEROPEND_LAB 2.1`, [PROTOCOL.md](PROTOCOL.md))에서 갈라져 나왔고, 아래에 적지 않은 것은 전부 PROTOCOL.md와 같습니다(250000 baud, 줄 형식, 하트비트 1 s, 보정·EEPROM 형식, `D`/`A`/`S` 줄, 각도 계산). EEPROM 형식이 같으므로 두 펌웨어를 번갈아 올려도 보정과 영점이 유지됩니다.

## 실험용 펌웨어와의 차이

| | 내용 |
|---|---|
| 식별 | `HELLO` → `READY AEROPEND_RL 1.0` |
| 삭제 | `RUN STEP`, `RUN STAIR`, `RUN CHIRP`, `RUN PRBS`, `RUN RELAY` (→ `ERR UNKNOWN_PROFILE`) |
| 유지 | 연결·상태, `U`, `LIM`, `STREAM`, `BENCH`, 센서·보정 전부, `PID`, `NOTCH`, `REF`, `RUN CLSTEP`, 모든 안전장치 |
| 추가 | `RL`, `SAFE`, `RUN RL`, `A` (아래) |

## 구조

PC가 정책을 계산하고 보드는 실행과 안전을 맡습니다. 행동은 **PWM 증분**입니다.

```
결정 샘플 k (에피소드 샘플 번호 n 이 dec 의 배수):
  보드: θ_k 측정 → D 줄 전송
  PC  : D 줄 수신 → 관측 구성 → 정책 → "A <du>" 전송
샘플 k + lag:
  보드: 그 사이 A 가 왔으면  u ← clamp(u + du, [u_lo, min(u_hi, PWMmax)])
        안 왔으면            u 유지, miss += 1
  u 는 다음 적용 시점까지 유지 (영차 유지)
```

지연이 `lag` 샘플로 고정되므로 PC 쪽 지터는 누락(`miss`)으로만 나타납니다. 한 결정 구간에 `A`가 두 번 오면 마지막 것만 씁니다.

## 추가 명령

| 명령 | 범위 | 설명 |
|---|---|---|
| `RL <dec> <lag> <du_max> <u_lo> <u_hi>` | dec 1–20, lag 1–dec, du_max 1–255, 0 ≤ u_lo < u_hi ≤ 255 | 결정 주기(샘플), 적용 지연(샘플), 한 번에 바꿀 수 있는 최대 PWM, 정책 PWM 범위. 에피소드 중에는 `ERR BUSY` → `RLCFG …` |
| `SAFE <th_lo_cdeg> <th_hi_cdeg> <w_max_cdps>` | −18000…18000, lo < hi, 1–200000 | 에피소드 안전 범위. 에피소드 중에는 `ERR BUSY` → `RLCFG …` |
| `RUN RL <r_cdeg> <dur_ms> <home_cdeg>` | \|r\|, \|home\| ≤ 17000 | 에피소드 시작. `PID` 이득이 모두 0이면 `ERR NO_PID`, IMU 미준비면 `ERR IMU_NOT_READY` |
| `A <du>` | 정수 (±du_max로 자름) | 다음 적용 시점의 PWM 증분. 에피소드 밖이면 `ERR NOT_RL` |
| `REF <cdeg>` | | **에피소드 중에는** 기록용 기준 `ref`만 바꿈 (`EVT REF <cdeg>`, 제어에는 영향 없음). 에피소드 밖에서는 PROTOCOL.md와 같음 |

재부팅 기본값: `dec=2 lag=1 dumax=10 ulo=0 uhi=255`, `SAFE −2000 6000 40000` (−20°…60°, 400 °/s). 저장하지 않습니다.

`GET`은 `CFG`, `CALVAL`, `PIDCFG` 뒤에 `RLCFG`를 붙입니다.

```
RLCFG dec=2 lag=1 dumax=10 ulo=40 uhi=210 thlo=-2000 thhi=6000 wmax=40000
```

## 에피소드

1. `RUN RL r dur home` → `EVT RUN RL total=<샘플수> ts_ms=<Ts>`. `phase = 1`, 에피소드 샘플 번호 n = 0, `miss = 0`.
2. **시작 PWM은 그 순간 적용 중이던 출력**(`REF`로 유지 중이었다면 PID 출력, 대기 중이었다면 0)을 [u_lo, u_hi]로 자른 값입니다. 끊김 없이 넘겨받습니다.
3. 매 샘플: θ·ω 계산 → (적용 시점이면) `A` 반영 → 안전 검사 → PWM 출력 → `D` 줄.
4. 끝나는 경우:

| 경우 | 출력 | 그 뒤 |
|---|---|---|
| `dur_ms` 경과 | `EVT DONE` | PID로 `home` 유지 (상태 4) |
| θ가 [th_lo, th_hi] 밖 또는 \|ω\| > w_max | `EVT FALLBACK <THETA\|OMEGA>` | PID로 `home` 유지 (상태 4) |
| \|θ\| > 각도 한계(`LIM`) | `EVT ABORT ANGLE_LIMIT` | 정지 (PROTOCOL.md와 같음) |
| 1 s 동안 명령 없음 | `EVT ABORT LINK_TIMEOUT` | 정지 |
| `STOP`, `U` | `EVT STOP` | 정지 |

5. PID로 넘어갈 때는 **적분항을 `I = u − ff`로 맞춰**(±255로 자름) 출력이 튀지 않게 하고, `EVT REF <home>`을 보냅니다. 그 뒤는 보통의 `REF` 유지와 같아서 `STOP`·`U`로 풀고 하트비트가 필요합니다.

에피소드 중 `RUN`, `U`를 제외한 다른 시험 명령과 `STREAM`, `RL`, `SAFE`는 `ERR BUSY`입니다. `PID`·`NOTCH`·`LIM`·`PING`·`GET`·`STOP`·`REF`·`A`는 받습니다. `U`는 에피소드를 끝내고 수동 출력으로 넘어갑니다(`EVT STOP`).

### 세부 규칙

- **한 샘플의 순서**: 종료 검사(n ≥ total → `EVT DONE`) → `A` 적용 → `SAFE`(θ 먼저, 그다음 ω) → 각도 한계 → PWM 출력 → `D` 줄.
- **쓰이는 `A`**: 결정 샘플의 `D` 줄 이후부터 그 적용 시점 전까지 도착한 것만 씁니다. 적용 시점을 지나 도착한 `A`(lag < dec일 때)는 이미 miss로 세었으므로 다음 결정 샘플에서 버립니다. `RUN RL` 직후 n = 0 전에 온 `A`도 버립니다. 첫 적용 시점은 n = lag입니다.
- **범위 제한**: `u_lo`를 먼저, `min(u_hi, PWMmax)`를 나중에 적용합니다. `LIM`이 `u_lo`보다 낮으면 `LIM`이 이깁니다. 시작 PWM도 같은 방식으로 자릅니다.
- **PID로 넘어갈 때의 u**: 직전 샘플에 실제로 적용된 PWM(마지막 `D` 줄의 `pwm`)입니다. 넘어가는 샘플에서 바로 PID 출력이 나가고, 그 `D` 줄부터 `phase = 0`, 8개 필드입니다. 노치가 켜져 있으면 노치 상태가 0에서 시작해 첫 출력이 조금 튑니다.
- **응답**: `RL`·`SAFE`는 성공하면 `RLCFG` 한 줄, 에피소드 중이면 `ERR BUSY`만 보냅니다. `HELLO`는 `RLCFG`를 내지 않습니다(`GET`으로 확인).
- **오류 검사 순서**: `RUN RL`은 BUSY → IMU_NOT_READY → BAD_ARGS → NO_PID → BAD_DURATION이고, 통과하기 전에는 상태(예: `REF` 유지)를 바꾸지 않습니다. `A`는 BAD_ARGS → NOT_RL입니다.
- **에피소드 중 명령**: `RUN …`·`STREAM`·`RL`·`SAFE`·보정 명령은 `ERR BUSY`. `U`는 에피소드를 끝내고(`EVT STOP`) 수동 출력으로 넘어갑니다. `NO_PID` 검사는 시작할 때만 합니다.

## `D` 줄 (에피소드 중)

```
D,t_ms,pwm,theta_cdeg,omega_cdps,thetaAcc_cdeg,phase,ref_cdeg,miss
```

- 앞 8개 필드는 PROTOCOL.md의 폐루프 `D` 줄과 같습니다. `phase`는 1, `ref_cdeg`는 `RUN RL`의 r(또는 에피소드 중 `REF`로 바꾼 값)입니다.
- `miss`: 이 에피소드에서 `A`가 제때 오지 않은 적용 시점의 누적 횟수.
- `pwm`은 같은 줄의 θ를 측정한 직후 적용된 값입니다. n이 `dec`의 배수인 줄이 결정 샘플이고, 에피소드 첫 `D` 줄이 n = 0입니다.
- 에피소드가 끝나 PID 유지로 넘어간 뒤의 `D` 줄은 `phase = 0`, 8개 필드입니다.

`S` 줄의 `state`: 에피소드 중에는 2(시험)입니다.

## 오류 코드 (추가)

`NO_PID`, `NOT_RL`

## 안전

실험용 펌웨어의 안전장치(하트비트 1 s, 각도 한계, `LIM` 최대 PWM, IMU 실패 정지, 워치독, 벤치 모드 핀 LOW)는 그대로입니다. RL 펌웨어는 여기에 정책 PWM 범위(`u_lo`…`u_hi`), 변화율 제한(`du_max`), 에피소드 안전 범위(`SAFE` → PID 복귀)를 더합니다. PC가 멈추면 `A`가 오지 않아 PWM이 그대로 유지되다가 1 s 뒤 하트비트로 정지합니다.
