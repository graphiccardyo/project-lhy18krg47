# Aeropendulum — 시스템 식별과 PID 제어 (자동제어 3조)

프로펠러 추력으로 진자 각도를 제어하는 **에어로펜듈럼**의 펌웨어, 실험 UI, 분석 도구입니다.
개루프 시험으로 전달함수를 식별하고(목표 1–3), 그 모델로 PID·Z-N 제어기를 설계해 시험합니다(목표 4–6).

- **하드웨어**: Arduino Uno + L298N(DC 모터 + 프로펠러) + MPU6050(GY-521) → [docs/HARDWARE.md](docs/HARDWARE.md)
- **입력 단위는 PWM 카운트(analogWrite 0–255)** 입니다. 전압으로 환산하지 않으며, 모든 모델의 단위는 deg/PWM입니다.
- **각도 θ**는 Madgwick 필터 자세를 진자 회전축 기준으로 투영한 값입니다. θ = 0은 모터를 끈 채 매달린 정지 자세이고, 추력 방향이 +입니다.

## 과제 진행 상황

| 목표 | 내용 | 상태 | 어디에 |
|---|---|---|---|
| 1 | 개루프 시험: 입력 PWM ↔ 출력 각도 (정적 특성) | ✅ 데이터·분석 완료 | UI ③, [data/20261106_run2](data/20261106_run2/analysis/REPORT.md) |
| 2 | 개루프 시험: 스텝, 스윕 사인 (+PRBS) | ✅ 데이터·분석 완료 | UI ④, [data/20261106_run3](data/20261106_run3/analysis/REPORT.md) |
| 3 | 전달함수 추정 → 검증 | ✅ 모델 확정 (아래) | UI ⑤, `matlab/`, `analysis/` |
| 4 | 폐루프 스텝 응답 | 🟡 구현·시뮬레이터 검증, **하드웨어 시험 전** | UI ⑥ (프리셋 P) |
| 5 | PID 설계·시험 | 🟡 설계 완료(PI + 노치 권장), **하드웨어 시험 전** | UI ⑥, [analysis/pid_design](analysis/pid_design/REPORT.md) |
| 6 | Z-N 설계·시험 | 🟡 릴레이 시험 구현, **하드웨어 시험 전** | UI ⑥ (릴레이) |
| 7 | 강화학습 제어기 | ⬜ 시작 전 | — |

현재 결과 요약은 [docs/RESULTS.md](docs/RESULTS.md)에 있습니다.

### 식별된 플랜트 (PWM ≈ 150, θ ≈ 20°)

```
G(s) = Δθ/ΔPWM = K · e^(−0.016 s) / (0.328 s + 1) · 21.8² / (s² + 5.37 s + 21.8²)     [deg/PWM]
       K = 0.12–0.21 (공칭 0.15, 세션 중 변동),  3.47 Hz 구조 공진(ζ ≈ 0.12)
```

권장 제어기는 **PI + 3.47 Hz 노치**(Kp 7.85, Ki 23.9)입니다. 모델 예측은 상승 0.4 s, 오버슈트 1% 미만이고, K 0.12–0.21 전체에서 최대 감도 Ms ≤ 1.6입니다.

## 빠른 시작

1. **펌웨어 업로드**: Arduino IDE에서 [firmware/aeropendulum_lab/aeropendulum_lab.ino](firmware/aeropendulum_lab/aeropendulum_lab.ino)를 열고 보드 *Arduino Uno*로 업로드합니다. 외부 라이브러리는 필요 없습니다. 명령줄로 할 때:
   ```bash
   powershell -ExecutionPolicy Bypass -File scripts/Build-Lab.ps1 -Upload -Port COM4
   ```
   업로드할 때는 모터 12V를 끄고, 실험 페이지와 시리얼 모니터를 닫으세요.
2. **실험 UI**: `Start-Lab.cmd`를 더블클릭(= Edge로 `lab/index.html`) → **보드 연결** → COM 포트 선택. Web Serial을 쓰므로 Edge나 Chrome이 필요하고, 설치·서버·인터넷은 필요 없습니다.
3. **하드웨어 없이**: 같은 페이지에서 **시뮬레이터**를 누르면, 식별 모델로 만든 가상 진자로 모든 탭을 연습할 수 있습니다.
4. 실험 순서(보정 → 개루프 시험 → 추정 → 폐루프)는 [docs/EXPERIMENT_GUIDE.md](docs/EXPERIMENT_GUIDE.md)를 따르세요.

## 저장소 구조

```
firmware/aeropendulum_lab/   우노 펌웨어 (센서·Madgwick·보정·가진 신호·PID·안전장치) — 단일 .ino
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
| [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) | 코드 수정하는 팀원 | 도구, 빌드·시험, 코드 구조, 기능 추가 절차, 제약 |
| [AGENTS.md](AGENTS.md) | AI 에이전트 | 저장소 지도, 지켜야 할 규칙, 검증 방법 |

## 이전 버전

1주차에 쓰던 기구 확인용 프로그램(모터 단독 조절기, On/Off 시험, MPU6050 모니터, 1주차 웹 데모, 핀 진단 등)은 이 정리에서 삭제했습니다. 필요하면 첫 커밋 `73a69be`에서 꺼내 쓸 수 있습니다.
