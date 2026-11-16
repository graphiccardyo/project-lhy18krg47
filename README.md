# Aeropendulum — 시스템 식별과 PID 제어 (자동제어 3조)

프로펠러 추력으로 진자 각도를 제어하는 **에어로펜듈럼**의 펌웨어, 실험 UI, 분석 도구입니다.
개루프 시험으로 전달함수를 식별하고(목표 1–3), 그 모델로 PID·Z-N 제어기를 설계해 시험합니다(목표 4–6).

- **하드웨어**: Arduino Uno + L298N(DC 모터 + 프로펠러) + MPU6050(GY-521) → [docs/HARDWARE.md](docs/HARDWARE.md)
- **입력 단위는 PWM 카운트(analogWrite 0–255)** 입니다. 전압으로 환산하지 않으며, 모든 모델의 단위는 deg/PWM입니다.
- **각도 θ**는 Madgwick 필터 자세를 진자 회전축 기준으로 투영한 값입니다. θ = 0은 모터를 끈 채 매달린 정지 자세이고, 추력 방향이 +입니다.

## 과제 진행 상황 (2026-11-15)

| 목표 | 내용 | 상태 | 어디에 |
|---|---|---|---|
| 1 | 개루프 시험: 입력 PWM ↔ 출력 각도 (정적 특성) | ✅ 완료 | UI ③, [data/20261115_run1](data/20261115_run1/analysis/REPORT.md) |
| 2 | 개루프 시험: 스텝, 스윕 사인 (+PRBS) | ✅ 완료 | UI ④, [data/20261115_run1](data/20261115_run1/analysis/REPORT.md) |
| 3 | 전달함수 추정 → 검증 | ✅ 모델 확정 (아래, 11-15 재식별). 릴레이·폐루프 스텝으로 검증 | UI ⑤, `matlab/`, `analysis/` |
| 4 | 폐루프 스텝 응답 | 🟡 하드웨어 시험 대부분 (11-15: PI + 노치, PID + 노치, Z-N PI·PID·PID + 노치, 30 s 유지; 11-13 모델로 P, PI, 손튜닝 PID). P(11-15 이득)는 아직 | UI ⑥, [data/20261115_run2](data/20261115_run2/analysis/REPORT.md) |
| 5 | PID 설계·시험 | 🟢 **PI + 2.35 Hz 노치**를 하드웨어에서 확인 (상승 0.57–0.71 s) | UI ⑥, [analysis/pid_design](analysis/pid_design/REPORT.md) |
| 6 | Z-N 설계·시험 | 🟢 11-15: 릴레이(Ku 4.87, Tu 0.439 s), Z-N PI, **Z-N PID**(발산 없음, 2.34 Hz 리플 지속 → 공진이 Ku·Tu를 정하는 Z-N의 한계), 같은 이득 + 2.35 Hz 노치는 리플 절반·상승 0.60 s. 11-13 모델 Z-N PID는 30°에서 진동 | UI ⑥, [data/20261115_run1](data/20261115_run1/analysis/REPORT.md), [data/20261115_run3](data/20261115_run3/analysis/REPORT.md) |
| 7 | 강화학습 제어기 | ✅ 실기 SAC(rl8): 평가 RMS 1.9° (PID 2.4°), 정상상태 편향·리플은 PID가 나음. 감쇠 보상(rl9)은 효과 없음 | `rl/`, [진행 기록](docs/RL_HISTORY.md), [firmware/aeropendulum_rl](firmware/aeropendulum_rl/aeropendulum_rl.ino), [data/20261115_rl8](data/20261115_rl8/) |
| 시연 | PID ↔ RL 시연 | 🔧 잔차 RL 정책(새 모터, rl5)을 보드에 넣은 시연용 펌웨어 2.0 + UI(시험·결과 저장). 보드 벤치 시험 통과, 실기 시연 전 | [docs/DEMO.md](docs/DEMO.md), [firmware/aeropendulum_demo](firmware/aeropendulum_demo/aeropendulum_demo.ino), `lab/demo.html` |

결과 요약과 근거는 [docs/RESULTS.md](docs/RESULTS.md), 지난 시험(11-06 기구 수정 전, 11-13 기구 수정 직후)은 그 문서의 [이전 기록](docs/RESULTS.md#4-이전-기록)에 있습니다.

### 식별된 플랜트 (2026-11-15 재식별, PWM ≈ 150, θ ≈ 20°)

```
G(s) = Δθ/ΔPWM = [ K/(0.324 s + 1) · 14.8²/(s² + 2.48 s + 14.8²) + 0.12/(3 s + 1) ] · e^(−0.024 s)     [deg/PWM]
       K ≈ 0.164,  2.35 Hz 구조 공진(ζ ≈ 0.08; 30°에서 2.26 Hz ζ 0.07),  느린 경로 0.12/(3 s+1)
```

- 권장 제어기는 **PI + 2.35 Hz 노치**(Kp 4.30, Ki 13.3)입니다. K 0.12–0.23 × 공진 ±10%에서 최대 감도 Ms ≤ 1.6이고, 하드웨어 폐루프 스텝(4.34 / 13.4)에서 상승 0.57–0.71 s(모델 0.61–0.64 s), 리플을 뺀 오버슈트 8–12%였습니다.
- 손튜닝 PID(Kp 0.9, Ki 7, Kd 0.02)도 안전하지만(최악 Ms 1.44) 상승이 약 1.2–1.5 s입니다.
- 11-13 이후 공진이 서서히 내려가(2.84 → 2.35 Hz) 한계 이득이 절반이 됐습니다(릴레이 Ku 10.2 → 4.87). **그 전 모델의 권장값(11-13: PI 5.40 / 16.9 + 2.84 Hz 노치)은 새 플랜트에서 최악 Ms 3.3이라 쓰지 마세요.**

## 빠른 시작

1. **펌웨어 업로드**: Arduino IDE에서 [firmware/aeropendulum_lab/aeropendulum_lab.ino](firmware/aeropendulum_lab/aeropendulum_lab.ino)를 열고 보드 *Arduino Uno*로 업로드합니다. 외부 라이브러리는 필요 없습니다. 명령줄로 할 때:
   ```bash
   powershell -ExecutionPolicy Bypass -File scripts/Build-Lab.ps1 -Upload -Port COM4
   ```
   업로드할 때는 모터 12V를 끄고, 실험 페이지와 시리얼 모니터를 닫으세요.
2. **실험 UI**: `Start-Lab.cmd`를 더블클릭(= Edge로 `lab/index.html`) → **보드 연결** → COM 포트 선택. Web Serial을 쓰므로 Edge나 Chrome이 필요하고, 설치·서버·인터넷은 필요 없습니다.
3. **하드웨어 없이**: 같은 페이지에서 **시뮬레이터**를 누르면, 식별 모델로 만든 가상 진자로 모든 탭을 연습할 수 있습니다.
4. 실험 순서(보정 → 개루프 시험 → 추정 → 폐루프)는 [docs/EXPERIMENT_GUIDE.md](docs/EXPERIMENT_GUIDE.md)를 따르세요.
5. **시연**(PID ↔ 강화학습): `scripts/Build-Demo.ps1 -Upload`로 시연용 펌웨어를 올리고 `lab/demo.html`을 엽니다. 자세한 것은 [docs/DEMO.md](docs/DEMO.md)에 있습니다.

## 저장소 구조

```
firmware/aeropendulum_lab/   우노 펌웨어 (센서·Madgwick·보정·가진 신호·PID·안전장치) — 단일 .ino
firmware/aeropendulum_rl/    강화학습용 펌웨어 (PC 가 보낸 PWM 증분 적용 + 안전장치·PID 복귀)
firmware/aeropendulum_demo/  시연용 펌웨어 (보드 위 고정 잔차 RL 정책 + 조정 가능한 PID) — UI 는 lab/demo.html
rl/                          Python SAC 실기 학습기, 가짜 보드, 정책 시험·분기·IMU 점검, 웹 대시보드
lab/                         실험 UI (index.html + app.js, 시뮬레이터 sim.js, 식별 계산 sysid.js, 차트 chart.js)
Start-Lab.cmd                Edge 로 lab/index.html 열기
analysis/                    Python 분석 (데이터 진단, 주파수응답, 모델 맞춤, PID 설계)
matlab/                      MATLAB 분석 (System Identification Toolbox: tfest, 검증, PID 비교)
data/                        실험 CSV 와 분석 결과 (run 별 analysis/REPORT.md)
scripts/                     빌드/업로드, 보드 자동 시험, JS 계산 시험
docs/                        사용·프로토콜·하드웨어·결과·개발 문서, 발표자료
AGENTS.md                    AI 에이전트(코딩 보조)용 작업 지침
```

## 문서

| 문서 | 대상 | 내용 |
|---|---|---|
| [docs/EXPERIMENT_GUIDE.md](docs/EXPERIMENT_GUIDE.md) | 실험하는 사람 | UI 탭별 사용법, 보정, 시험 1–6 절차, 안전 |
| [docs/HARDWARE.md](docs/HARDWARE.md) | 조립·배선 | 부품, 배선, 전원, 센서 장착, 주의사항 |
| [docs/RESULTS.md](docs/RESULTS.md) | 보고서·발표 | 실험 데이터, 식별 모델, PID 설계 결과와 근거 |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | 개발·분석 | 시리얼 명령/출력 형식, CSV 형식, EEPROM |
| [rl/README.md](rl/README.md) · [docs/RL_PROTOCOL.md](docs/RL_PROTOCOL.md) | 강화학습 | 학습기 사용법·설정·안전, RL 펌웨어 프로토콜 |
| [docs/DEMO.md](docs/DEMO.md) | 시연 | 시연용 펌웨어·UI 사용법, 보드 위 RL 정책 구현·검증, 프로토콜 |
| [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) | 코드 수정하는 팀원 | 도구, 빌드·시험, 코드 구조, 기능 추가 절차, 제약 |
| [AGENTS.md](AGENTS.md) | AI 에이전트 | 저장소 지도, 지켜야 할 규칙, 검증 방법 |

## 이전 버전

1주차에 쓰던 기구 확인용 프로그램(모터 단독 조절기, On/Off 시험, MPU6050 모니터, 1주차 웹 데모, 핀 진단 등)은 이 정리에서 삭제했습니다. 필요하면 첫 커밋 `73a69be`에서 꺼내 쓸 수 있습니다.
