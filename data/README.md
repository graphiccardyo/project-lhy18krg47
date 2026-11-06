# 실험 데이터

각 폴더에는 UI가 저장한 원본 CSV(수정 금지)와 `analysis/`(분석 그림·요약·`REPORT.md`)가 있습니다. CSV 형식은 [docs/PROTOCOL.md](../docs/PROTOCOL.md#csv-ui가-저장)에 있습니다.

| 폴더 | 날짜·시각 | 펌웨어 설정 | 내용 | 결론 |
|---|---|---|---|---|
| [20261106](20261106/analysis/REPORT.md) | 11-06 13:45–13:52 | β 0.301, DLPF 2, ±4 g/±500 °/s | STAIR ×2, STEP, CHIRP (PWM 180 ± 12, 0.1–3 Hz) | ❌ 프로펠러 진동으로 각도 오염. 동특성 식별 불가 (기록용) |
| [20261106_run2](20261106_run2/analysis/REPORT.md) | 11-06 14:15–14:22 | β 0.02, DLPF 4, ±16 g/±2000 °/s | MANUAL, STAIR ×2 (50→250, 3 s), STEP ×2 (150→162), CHIRP ×2 (150 ± 12, 0.1–3 Hz) | ✅ 정적 특성, 1차 + 지연 모델 (K 0.15–0.21) |
| [20261106_run3](20261106_run3/analysis/REPORT.md) | 11-06 14:34–14:40 | 같음 | CHIRP ×2 (150 ± 12, 0.1–10 Hz), CHIRP ×2 (150 ± **25**, 0.1–7 Hz) | ✅ **3.47 Hz 공진 확인, 최종 모델**. ±25가 가장 좋은 데이터 (SNR 25) |
| [calibration](calibration/) | 10-31 | (1주차 펌웨어) | MPU6050 수평 보정 오프셋 | 펌웨어의 가속도 오프셋 기본값 |

공통 조건: 축 Z−, Ts = 10 ms, 입력은 PWM, 모터 전원 12 V(전압은 기록하지 않음 — 다음부터 기록 권장).

## 새 데이터를 올릴 때

1. `data/<YYYYMMDD>_run<N>/`에 CSV를 그대로 복사합니다(다운로드 폴더의 `aero_*.csv`).
2. 위 표에 한 줄을 추가합니다. 펌웨어 설정, 시험 조건, **모터 전원 전압**, 장착·기구 변경 사항을 적습니다.
3. 분석:
   ```bash
   python analysis/aero_analyze.py data/<폴더>          # 품질·재현성·정적 특성·교차검증
   python analysis/aero_freqresp.py data/<폴더>         # CHIRP 주파수응답
   python analysis/aero_model_fit.py --est <CHIRP들> --val <나머지>   # 공진 포함 모델
   ```
4. 결론을 `data/<폴더>/analysis/REPORT.md`에 쓰고, 큰 결론이 바뀌면 [docs/RESULTS.md](../docs/RESULTS.md)도 갱신합니다.
