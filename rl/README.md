# rl/ — 실기 SAC 학습기 (목표 7)

PC(Python)가 정책을 계산해 결정 샘플마다 `A <du>`(PWM 증분)를 보내고, 보드(`AEROPEND_RL 1.0`)는 실행·안전·PID 복귀를 맡습니다. 프로토콜은 [docs/RL_PROTOCOL.md](../docs/RL_PROTOCOL.md)입니다.

## 구조

| 파일 | 내용 |
|---|---|
| `config.py` | 모든 숫자(플랜트·PID·LIM·RL·SAFE·에피소드·보상·휴식·SAC·정지 조건). JSON 읽기/쓰기. `[재식별]` 표시는 플랜트를 다시 식별하면 바꿀 값 |
| `link.py` | 연결: `SerialLink`(pyserial 또는 가짜 포트, 읽기 스레드, 0.25 s PING, 감시견), `VirtualLink`(가상시간). D 줄을 받은 읽기 스레드에서 바로 `A`를 보내는 `fast_path` |
| `plant.py`, `fake_board.py` | 11-13 모델 플랜트(정적 지도 + 1차 지연 + 느린 경로 + 공진 + 지연 + 마찰 + 정지 오프셋 + 선택: 케이블 충격, 도메인 랜덤화)와 RL 펌웨어를 줄 단위로 흉내 내는 가짜 보드(`FakePort` = 실시간) |
| `env.py` | D 줄 → 관측·행동·보상·전이 |
| `sac.py` | PyTorch SAC(쌍둥이 Q, 자동 엔트로피, 버퍼, 체크포인트) + numpy 추론 |
| `trainer.py` | 상태 기계 스레드: 연결·설정 → PID 기준선 → [휴식] → [평가] → 복귀 → 에피소드 → 갱신. `status()`/`command()` (모듈 docstring에 정확한 형식) |
| `record.py` | 실행 폴더 기록 |
| `train.py` | CLI |
| `play.py` | 학습 없이 저장된 정책 시험 CLI (`Trainer`의 연결·복귀·에피소드 코드를 그대로 씀) |
| `bench_test.py` | 프로토콜 적합성 시험 (BENCH 1 전용) + 왕복 지연 측정 |
| `selftest.py` | 하드웨어 없는 자체 시험 |

## 실행

Python은 `.venv\Scripts\python.exe`입니다(numpy, scipy, pyserial, torch CPU).

```powershell
.venv\Scripts\python.exe -m rl.selftest                    # 하드웨어 없이 전부 (몇 분, --quick 은 학습 시험 생략)
.venv\Scripts\python.exe -m rl.bench_test --fake --fast    # 가짜 보드로 프로토콜 시험
.venv\Scripts\python.exe -m rl.bench_test --port COM4      # 실제 보드, BENCH 1 에서만 (모터 안 돎)
.venv\Scripts\python.exe -m rl.train --fake --fast --episodes 200   # 가짜 보드 가상시간 학습
.venv\Scripts\python.exe -m rl.train --port COM4 --hours 2          # 실기 학습 (모터가 돕니다)
.venv\Scripts\python.exe -m rl.train --port COM4 --resume data\20261114_rl1   # 이어 하기
```

## 스냅샷 (정책 시점별 저장)

`checkpoint/`(이어 하기용, 덮어씀)와 별도로 `checkpoints/<이름>/`에 정책만 남깁니다. 폴더마다 `actor.pt`(액터 가중치, 약 22 KB — 리플레이 버퍼·크리틱·옵티마이저 없음)와 `meta.json`(종류, 학습 에피소드, 결정 수, 갱신 수, 그 정책의 평가 RMS, 저장 시점의 최근 평가 RMS, 시각)입니다.

| 이름 | 언제 |
|---|---|
| `ep00050`, `ep00100`, … | 학습 `run.snapshot_every`(기본 50, 0 = 끔) 에피소드마다 |
| `best` | 평가 라운드 평균 RMS가 지금까지의 `best`보다 작을 때 (덮어씀). 이 기능 전에 시작한 실행을 이어 하면 이어 한 뒤 첫 평가가 처음 `best`가 됩니다 |
| `manual_<에피소드>_<HHMMSS>` | 대시보드 **지금 저장**, 표준입력 `n`, `command('snapshot')` |

- 수동 저장은 제어 루프를 방해하지 않게 다음 에피소드 경계(또는 휴식 중)에 처리하고, 대기·일시정지 중이면 바로 저장합니다.
- **지금 평가**(표준입력 `e`, `command('eval')`)는 다음 에피소드 경계에서 평가 라운드를 한 번 더 돌립니다. 정기 평가와 똑같이 기록하고(`eval_scores`에 `manual: true`), 정기 평가 일정은 바뀌지 않습니다.
- 옛 `config.json`(이 키 없음)과 옛 `checkpoint/`도 그대로 `--resume` 됩니다. 단, 새 코드로 이어 하면 `config.json`에 `snapshot_every`가 추가되므로 그 실행을 다시 옛 코드로 열 수는 없습니다.

## 정책 시험 (`rl.play`, 학습 없음)

저장된 정책이 기준 각도를 얼마나 잘 유지·추종하는지 봅니다. 학습기가 포트를 쓰고 있지 않을 때 실행하세요. 정책은 결정적(평균 행동), 갱신·버퍼 없음. 설정은 그 실행 폴더의 `config.json`(PID·LIM·RL·SAFE 포함)을 그대로 보냅니다.

```powershell
# 최신 체크포인트(checkpoint/)로 25° 유지 20 s  — 모터가 돕니다
.venv\Scripts\python.exe -m rl.play --port COM4 --ckpt data\20261114_rl1 --hold 25
# best 스냅샷으로 20° → 30° → (10 s 뒤) 22°, 같은 시나리오를 보드 PID로도 돌려 비교, 2회 반복
.venv\Scripts\python.exe -m rl.play --port COM4 --ckpt data\20261114_rl1 --snapshot best --refs 20 30 22 --pid --repeat 2
# 스냅샷 폴더를 직접 / BENCH 1 드라이런(모터 안 돎) / 가짜 보드
.venv\Scripts\python.exe -m rl.play --port COM4 --ckpt data\20261114_rl1\checkpoints\ep00100 --bench
.venv\Scripts\python.exe -m rl.play --fake --fast --ckpt data\20261114_rl1 --refs 20 30 22 --pid --out $env:TEMP\play
```

- `--ckpt`: 실행 폴더(→ `checkpoint/`), 실행 폴더 + `--snapshot 이름`, 스냅샷 폴더, `checkpoint` 폴더, `.pt` 파일.
- 시나리오: `--hold θ`(= home·r1·r2 모두 θ) 또는 `--refs home r1 r2 [home r1 r2 …]`(학습기 평가와 같은 의미, r2는 에피소드 절반에서). 둘 다 없으면 설정의 평가 시나리오 3개. `--dur`(기본 20 s), `--repeat`, `--settle`(정착 구간에서 뺄 시간, 기본 3 s), `--no-plot`, `--out`.
- 절차·안전은 학습기와 같습니다: 연결·설정 → `REF home` 정착 → `RUN RL` → … → `STOP`. `EVT ABORT`·연결 오류·miss 초과·Ctrl+C 어떤 경우에도 `STOP`을 보내고 끝냅니다. `ZERO`는 보내지 않습니다. `--port`(벤치 아님)이면 시작 전에 모터가 돈다는 안내를 출력합니다.
- 출력: 표(전체 RMS, 정착 구간 평균 |e|·σ(e), 오버슈트 °/%, 평균 PWM, 정착 구간 PWM σ, miss, FALLBACK)와 `<run_dir>/play/<YYYYMMDD_HHMMSS>/`에 `episodes/*.csv`(학습기와 같은 형식, `mode=play`/`pid`), `play_log.csv`, `events.log`, `config.json`, `summary.json`, `play.png`(시나리오마다 θ·기준, PWM; `--pid`면 PID 겹침).

## 웹 대시보드

`train`을 실행하면 기본으로 `http://127.0.0.1:8770/` 대시보드가 같이 뜹니다(시작 시 주소를 출력). 브라우저로 열면 상태 배지, 시작·일시정지·재개·정지 버튼, 숫자 카드, 최근 에피소드 θ·기준·PWM 그래프, 학습 곡선(RMS·평가·PID 기준선), 정상상태 |e| 곡선(학습 점·평가 평균·PID 기준선 이력), 에피소드별 평균 PWM, 로그를 1초마다 갱신해 보여 줍니다.

- `--dashboard 8780`으로 포트를 바꾸고, `--no-dashboard`로 끕니다. 포트를 못 열면 경고만 찍고 학습은 계속합니다.
- 서버는 `127.0.0.1`에만 바인드합니다. 명령 API(`POST /api/command`)는 JSON 본문만, 로컬 호스트 이름(`127.0.0.1`·`localhost`·`[::1]`)만 받습니다.
- 정지 버튼은 확인창을 한 번 띄웁니다. 학습기가 죽으면 화면에 "학습기와 연결 끊김"이 뜹니다.

- **지금 저장**·**지금 평가** 버튼과 스냅샷 표(이름, 에피소드, 평가 RMS, 시각; `best` 굵게)가 있습니다.

`train` 옵션: `--config cfg.json`(바꿀 항목만 적은 JSON), `--hours`, `--episodes`, `--bench`(BENCH 1 드라이런), 가짜 보드 전용 `--randomize`(도메인 랜덤화), `--shock 0.1`(케이블 충격/s), `--latency-ms 7 9`. 실행 중 표준입력 `p`/`r`/`q`/`s`/`n`/`e` = 일시정지/재개/정지/상태/스냅샷/지금 평가.

## 설정 요점 (기본값)

- 보드: `RL 2 2 10 40 255`(결정 20 ms, 적용 지연 2샘플 — 실측 왕복 7–9 ms라 lag 1은 누락), `SAFE −10° 70° 600 °/s`, `LIM 255 100°`(PID 시험과 같은 상한), PID 0.9/7/0.02 ff 140 umin 40(노치 끔). 학습기가 시작할 때 전부 명시적으로 보냅니다.
- 에피소드 10 s, 기준 15–30°에서 무작위(home도 무작위), 중간(35–65%)에 `REF`로 한 번 스텝. 평가 시나리오 3개(20→30→22, 30→18→28, 25→30→16)는 PID 기준선과 같습니다.
- PID 기준선: 시작 때 + 정기 평가 `run.baseline_every`(5)회마다 다시 잽니다. 세션 중 플랜트가 흘러가기 때문입니다(rl1에서 30° 유지 PWM이 188 → 192–197).
- 관측 8차원: `[e/10, (θ−25)/10, ω_f/100, Δθ, (u−125)/85, a_prev, ē/2, tanh(e/0.5)]`. 마지막 미세 오차 채널(`env.obs_efine`)은 0.5° 오차가 첫 항으로는 0.05밖에 안 돼서 넣었습니다.
- 보상: `−min((e/2)², 16) − |e|/2 − 2·(1 − exp(−(e/0.5)²)) − 0.05·a² − 0.1·(a − a_prev)²`, 전체 ×0.03. 셋째 항(정밀 항, `env.w_fine`/`fine_scale`)은 0.2–1°에서 기울기가 큽니다. 이 항 덕분에 8° 대 0.5° 벌점 비가 64에서 14로 줄어, 작은 정상상태 오차가 과도 응답 벌점에 묻히지 않습니다. `w_ddu`를 0.02에서 0.1로 올린 것은 정착 구간의 du 떨림 때문입니다.
- 휴식: 모터 가동 누적 300 s마다 `STOP` 후 90 s. 휴식 끝에 정지 각도 |θ| ≤ 5° 확인(정지 위치가 0°로 안 돌아올 수 있어 너그럽게, `ZERO`는 절대 자동으로 보내지 않음).
- SAC: 64-64 MLP, 배치 256, γ 0.99, lr 3e-4, τ 0.005, 워밍업 2500 결정(약 5 에피소드) 무작위, 결정당 갱신 1회(에피소드 사이에 몰아서).

### 11-14 이후 기본값 변경과 옛 실행

`data/20261114_rl1`은 PWM 상한 210, 기준 15–35°(평가 25→33→16), 관측 7차원, 정밀 항 없음, `w_ddu` 0.02로 돌렸습니다. 33°에서 PWM 210에 붙어 정상상태 오차가 1–1.6° 남았고, 학습 샘플의 12%가 포화였습니다.

- 실행 폴더의 `config.json`은 설정 전체를 저장하므로 옛 실행은 옛 값 그대로 `--resume`·`rl.play` 됩니다. 새 키(`env.obs_efine`, `env.w_fine`, `run.baseline_every`)가 없으면 옛 동작(관측 7차원, 정밀 항 0, 기준선 시작 때만)으로 읽습니다(`config.load_run_config`).
- 새 기본값으로 비교하려면 새 실행으로 시작하세요(관측 차원이 달라 옛 체크포인트를 이어 받을 수 없음).
- 이어 하기 때 `train_log.csv`는 기존 머리줄의 열을 그대로 씁니다(옛 실행에는 `ss_*` 열이 안 생김).

## 안전

- 모터를 돌리는 명령은 `train --port`만 보냅니다. `bench_test`는 `BENCH 1`이 확인돼야 `REF`/`RUN`/`U`를 보내고, `CFG BACKUP`으로 시작해 `CFG RESTORE`로 끝냅니다. `ZERO`·`FACTORY`·보정은 보내지 않습니다.
- 자동 일시정지(모터 `STOP`, 사람이 `resume`): `EVT ABORT`, 포트 오류·2 s 무수신, `S` 줄 overruns 급증·sat 이상·IMU 오류, 에피소드 miss > 20%, 예상치 못한 `EVT STOP`, 휴식 끝 정지 각도 이상.
- 자동 종료: 총 시간, 학습 에피소드 수, FALLBACK 누적. 어떤 예외로 끝나도 `STOP`과 체크포인트를 남깁니다.
- 학습기 주 스레드가 20 s 넘게 멈추면 하트비트를 끊어 보드가 1 s 뒤 스스로 정지합니다. 프로세스가 죽어도 마찬가지입니다.
- 보드 쪽 안전장치(각도 한계, `SAFE` → PID 복귀, u_lo/u_hi, du_max, 하트비트)는 PC와 무관하게 동작합니다.

## 기록 (`data/<YYYYMMDD>_rl<N>/`, 번호 자동)

| 파일 | 내용 |
|---|---|
| `config.json` | 사용한 설정 전체 |
| `train_log.csv` | 에피소드마다 한 줄: 종류(train/eval/baseline), 상태(done/fallback/abort), 기준, return, RMS, 스텝 뒤 RMS, 평균 PWM, miss, α, 손실 등 + 정상상태 `ss_e1`/`ss_e2`(스텝 직전·끝 1.5 s 평균 e, 부호 있음), `ss_abs`(두 창 평균 \|e\|), `ss_pwm_sd`(창 안 PWM σ, 떨림) |
| `episodes/epNNNNN_<종류>.csv` | [PROTOCOL.md](../docs/PROTOCOL.md) CSV 형식(`type=RL`, `# pid` 줄) + 열 끝 `ref_deg,miss,du,reward` (du·reward는 결정 샘플에만) |
| `checkpoint/` | `sac.pt`(망·타깃·옵티마이저·α), `buffer.npz`, `trainer.json` — `--resume`으로 이어 하기 |
| `checkpoints/<이름>/` | 정책 스냅샷 `actor.pt` + `meta.json` (위 "스냅샷") |
| `play/<시각>/` | `rl.play` 결과 |
| `events.log` | 상태 변화·경고·오류 |
