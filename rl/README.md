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

## 웹 대시보드

`train`을 실행하면 기본으로 `http://127.0.0.1:8770/` 대시보드가 같이 뜹니다(시작 시 주소를 출력). 브라우저로 열면 상태 배지, 시작·일시정지·재개·정지 버튼, 숫자 카드, 최근 에피소드 θ·기준·PWM 그래프, 학습 곡선(RMS·평가·PID 기준선), 에피소드별 평균 PWM, 로그를 1초마다 갱신해 보여 줍니다.

- `--dashboard 8780`으로 포트를 바꾸고, `--no-dashboard`로 끕니다. 포트를 못 열면 경고만 찍고 학습은 계속합니다.
- 서버는 `127.0.0.1`에만 바인드합니다. 명령 API(`POST /api/command`)는 JSON 본문만, 로컬 호스트 이름(`127.0.0.1`·`localhost`·`[::1]`)만 받습니다.
- 정지 버튼은 확인창을 한 번 띄웁니다. 학습기가 죽으면 화면에 "학습기와 연결 끊김"이 뜹니다.

`train` 옵션: `--config cfg.json`(바꿀 항목만 적은 JSON), `--hours`, `--episodes`, `--bench`(BENCH 1 드라이런), 가짜 보드 전용 `--randomize`(도메인 랜덤화), `--shock 0.1`(케이블 충격/s), `--latency-ms 7 9`. 실행 중 표준입력 `p`/`r`/`q`/`s` = 일시정지/재개/정지/상태.

## 설정 요점 (기본값)

- 보드: `RL 2 2 10 40 210`(결정 20 ms, 적용 지연 2샘플 — 실측 왕복 7–9 ms라 lag 1은 누락), `SAFE −10° 70° 600 °/s`, `LIM 210 100°`, PID 0.9/7/0.02 ff 140 umin 40(노치 끔). 학습기가 시작할 때 전부 명시적으로 보냅니다.
- 에피소드 10 s, 기준 15–35°에서 무작위(home도 무작위), 중간(35–65%)에 `REF`로 한 번 스텝. 평가 시나리오 3개는 PID 기준선과 같습니다.
- 휴식: 모터 가동 누적 300 s마다 `STOP` 후 90 s. 휴식 끝에 정지 각도 |θ| ≤ 5° 확인(정지 위치가 0°로 안 돌아올 수 있어 너그럽게, `ZERO`는 절대 자동으로 보내지 않음).
- SAC: 64-64 MLP, 배치 256, γ 0.97, lr 3e-4, τ 0.005, 워밍업 2500 결정(약 5 에피소드) 무작위, 결정당 갱신 1회(에피소드 사이에 몰아서).

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
| `train_log.csv` | 에피소드마다 한 줄: 종류(train/eval/baseline), 상태(done/fallback/abort), 기준, return, RMS, 스텝 뒤 RMS, 평균 PWM, miss, α, 손실 등 |
| `episodes/epNNNNN_<종류>.csv` | [PROTOCOL.md](../docs/PROTOCOL.md) CSV 형식(`type=RL`, `# pid` 줄) + 열 끝 `ref_deg,miss,du,reward` (du·reward는 결정 샘플에만) |
| `checkpoint/` | `sac.pt`(망·타깃·옵티마이저·α), `buffer.npz`, `trainer.json` — `--resume`으로 이어 하기 |
| `events.log` | 상태 변화·경고·오류 |
