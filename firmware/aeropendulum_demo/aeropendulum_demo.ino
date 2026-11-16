// Aeropendulum DEMO firmware — Arduino Uno + L298N + MPU6050 (GY-521)
//
// 시연용. 보드 혼자서 두 제어기 중 하나로 기준 각도를 유지합니다 (PC 의 Python/PyTorch 불필요).
//   - PID (+ 선택 노치): 이득은 시리얼로 조정 (PID / NOTCH 명령, 실험용 펌웨어와 같은 식)
//   - RL: 고정 정책 = 11-15 rl8 best (ep 120). 정수 양자화한 MLP 8-64-64-1 이 policy.h 에 들어 있음
//         (python -m rl.demo_policy 로 생성). 관측·행동·적분 보조·시그마-델타는 rl/env.py 와 같은 식.
// RL 펌웨어(firmware/aeropendulum_rl, AEROPEND_RL 1.0)에서 갈라져 나옴. EEPROM 형식은 같아 보정·영점을 공유함.
// 프로토콜: docs/DEMO.md
//
// 정책 추론(곱셈 약 4 700 번)은 한 틱(5 ms)에 넣기엔 길어서, 틱 사이 빈 시간에 조각(뉴런 1–8개)으로 나눠 계산합니다.
// 결정 샘플 k 에서 시작한 추론 결과는 k+2 샘플(20 ms 뒤)에 적용 — 학습 때의 PC 경로(dec 2, lag 2)와 같은 시간 구조.
// 그때까지 못 끝내면 PWM 을 유지하고 miss 를 셉니다(학습기의 누락과 같은 처리).
//
// 배선 (기존 프로젝트와 동일)
//   D5 → L298N ENA (ENA 점퍼 제거),  D7 → IN1,  D8 → IN2,  GND 공통
//   MPU6050: VCC→3.3V, GND→GND, SDA→A4(SDA), SCL→A5(SCL)  (모듈 풀업 사용)
//
// 시리얼 250000 baud, 줄 단위 텍스트. 입력 단위: PWM 카운트 (analogWrite 0..255). 전압으로 환산하지 않음.

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "policy.h"

#define FW_ID "AEROPEND_DEMO 1.0"
#define USE_WATCHDOG 1
#ifndef NN_TEST
#define NN_TEST 0               // 1 = `NN` 정책 계산 시험 명령 (플래시 부족으로 기본 끔, rl/demo_test.py --nn 용)
#endif
#ifndef DEMO_CAL
#define DEMO_CAL 1
#endif

// ---------------------------------------------------------------- hardware
const uint8_t ENA_PIN = 5;
const uint8_t IN1_PIN = 7;
const uint8_t IN2_PIN = 8;
const uint32_t BAUD = 250000;
const uint32_t I2C_CLOCK = 400000;
const uint16_t TICK_US = 5000;          // 200 Hz 센서/필터 주기
const float TICK_S = TICK_US * 1e-6f;
const uint8_t STREAM_DIV = 2;           // 샘플 주기 Ts = 10 ms (정책이 학습된 주기 — 바꾸지 말 것)
const float TS = TICK_S * STREAM_DIV;
const unsigned long LINK_TIMEOUT_MS = 1000;
const float GYRO_LSB = 16.4f;           // ±2000 dps
const float ACC_LSB = 2048.0f;          // ±16 g
const uint8_t GYRO_FS_REG = 0x18;
const uint8_t ACC_FS_REG = 0x18;
const int16_t SAT_RAW = 32000;
const float ACC_LPF_ALPHA = 0.145f;     // 1 - exp(-2π·5 Hz·5 ms)
const float ACC_GATE_G = 0.15f;
const float DEG = 57.2957795f;
const float RAD = 0.01745329252f;

// ---------------------------------------------------------------- persistent config (실험용·RL 펌웨어와 같은 형식)
struct Config {
  uint16_t magic;
  uint8_t version;
  uint8_t calValid;
  uint8_t zeroValid;
  uint8_t axis;          // 0=X 1=Y 2=Z
  int8_t sign;
  uint8_t dlpf;
  uint16_t betaMilli;
  float gyroBias[3];
  float accOff[3];
  float g0[3];
  uint8_t crc;
};
const uint16_t CFG_MAGIC = 0xAE01;
const uint8_t CFG_VERSION = 4;
Config cfg;
bool cfgValid = false;

void printFix(float x, uint8_t d);

uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
  }
  return crc;
}

void saveConfig() {
  cfg.crc = crc8(reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg) - 1);
  EEPROM.put(0, cfg);
}

// EEPROM 이 유효하지 않으면(실험용 펌웨어로 보정한 적이 없으면) 기본값으로 RAM 에서만 돌고 저장하지 않음
void loadConfig() {
  EEPROM.get(0, cfg);
  cfgValid = cfg.magic == CFG_MAGIC && cfg.version == CFG_VERSION &&
             cfg.crc == crc8(reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg) - 1) &&
             cfg.axis < 3 && (cfg.sign == 1 || cfg.sign == -1) && cfg.dlpf <= 6 &&
             cfg.betaMilli >= 1 && cfg.betaMilli <= 1000;
  if (!cfgValid) {
    memset(&cfg, 0, sizeof(cfg));
    cfg.magic = CFG_MAGIC;
    cfg.version = CFG_VERSION;
    cfg.sign = 1;
    cfg.dlpf = 4;
    cfg.betaMilli = 10;
    cfg.g0[2] = 1.0f;
  }
}

// ---------------------------------------------------------------- Madgwick IMU (6축) — RL 펌웨어와 같음
struct Madgwick {
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  float beta = 0.05f;

  void seed(float ax, float ay, float az) {
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));
    float cr = cos(roll * 0.5f), sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f), sp = sin(pitch * 0.5f);
    q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
  }

  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    float an = ax * ax + ay * ay + az * az;
    if (an > 1e-6f) {
      float r = 1.0f / sqrt(an);
      ax *= r; ay *= r; az *= r;
      float _2q0 = 2 * q0, _2q1 = 2 * q1, _2q2 = 2 * q2, _2q3 = 2 * q3;
      float _4q0 = 4 * q0, _4q1 = 4 * q1, _4q2 = 4 * q2;
      float _8q1 = 8 * q1, _8q2 = 8 * q2;
      float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;
      float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      float s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      float s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      float s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay;
      float sn = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
      if (sn > 1e-12f) {
        r = beta / sqrt(sn);
        qDot1 -= r * s0; qDot2 -= r * s1; qDot3 -= r * s2; qDot4 -= r * s3;
      }
    }
    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
    float n = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    if (!(n > 0.25f) || !isfinite(n)) { q0 = 1; q1 = q2 = q3 = 0; return; }
    n = 1.0f / sqrt(n);
    q0 *= n; q1 *= n; q2 *= n; q3 *= n;
  }

  void gravity(float v[3]) const {
    v[0] = 2 * (q1 * q3 - q0 * q2);
    v[1] = 2 * (q0 * q1 + q2 * q3);
    v[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
  }
};
Madgwick ahrs;

// ---------------------------------------------------------------- state
uint8_t imuAddr = 0;
bool imuOk = false;
bool seedPending = true;
uint8_t readFails = 0;
unsigned long lastImuRetry = 0;

float gyr[3], acc[3];
float theta = 0, omega = 0, thetaAcc = 0;   // deg, deg/s

uint8_t divCount = 0;
uint8_t outDiv = 2, outCount = 0;   // D 줄은 outDiv 샘플마다 (기본 20 ms — 직렬 출력 시간을 추론에 돌림). OUT 1 = 10 ms
int16_t uMaxPwm = 255;
int16_t angleLimitCdeg = 10000;
bool bench = false;            // 벤치(드라이런): 모터 핀 항상 LOW, 값만 기록

int16_t uOut = 0;
unsigned long lastHeartbeat = 0;
unsigned long nextTickUs = 0, lastTickUs = 0;
uint16_t maxTickUs = 0, overruns = 0, i2cErrors = 0, satCount = 0;
uint16_t tickCount = 0, gatedCount = 0;
uint32_t tickSumUs = 0;       // 1 s 동안 틱에 쓴 시간 (S 줄 load_pct)
float accF[3] = {0, 0, 1};
unsigned long lastStatusMs = 0;

struct CalState {
  bool active;
  uint16_t n;
  float sum[3], lo[3], hi[3];
} cal;

// ---------------------------------------------------------------- 제어 상태
// hold = 기준 유지 중 (REF 로 시작, STOP 으로 끝). want = 사용자가 고른 제어기. rlOn = RL 이 실제로 출력 중.
// want == RL 이고 rlOn 이 아니면 PID 로 기준까지 가서 |e| < 2° 를 1 s 유지하면 RL 로 넘김 (학습 때 에피소드 시작과 같음).
bool hold = false;
bool wantRl = false;
bool rlOn = false;
uint8_t settleCnt = 0;
float refDeg = 0;
int16_t refCdeg = 0;

// ---------------------------------------------------------------- PID (실험용·RL 펌웨어와 같은 식)
// u = ff + Kp·e + Ki·∫e − Kd·ω_f, 출력단 선택 노치, 포화 + 조건부 적분
struct Pid {
  float kp = 4.30f, ki = 13.3f, kd = 0;   // 기본 = 권장 PI + 2.35 Hz 노치 (RESULTS.md §2)
  int16_t ff = 150;
  uint16_t nfc = 235;
  uint16_t nzz = 150, nzp = 700;
  uint8_t umin = 40;
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
} pid;
float pidI = 0, omF = 0, nz1 = 0, nz2 = 0;

void notchCoeffs() {
  pid.b0 = 1; pid.b1 = pid.b2 = pid.a1 = pid.a2 = 0;
  if (!pid.nfc) return;
  float w = 6.2831853f * pid.nfc * 0.01f;
  float k = w / tan(w * TS * 0.5f), k2 = k * k, w2 = w * w;
  float zz = pid.nzz * 0.001f, zp = pid.nzp * 0.001f;
  float a0 = k2 + 2 * zp * w * k + w2;
  pid.b0 = (k2 + 2 * zz * w * k + w2) / a0;
  pid.b1 = 2 * (w2 - k2) / a0;
  pid.b2 = (k2 - 2 * zz * w * k + w2) / a0;
  pid.a1 = pid.b1;
  pid.a2 = (k2 - 2 * zp * w * k + w2) / a0;
}

void pidReset() { pidI = 0; omF = omega; nz1 = nz2 = 0; notchCoeffs(); }

int16_t pidStep(float r) {
  float e = r - theta;
  omF += TS / (0.0159f + TS) * (omega - omF);
  float v = pid.kp * e + pidI - pid.kd * omF;
  if (pid.nfc) {
    float y = pid.b0 * v + nz1;
    nz1 = pid.b1 * v - pid.a1 * y + nz2;
    nz2 = pid.b2 * v - pid.a2 * y;
    v = y;
  }
  float u = pid.ff + v;
  float hi = uMaxPwm, lo = pid.umin < hi ? pid.umin : hi;
  float us = u < lo ? lo : (u > hi ? hi : u);
  if (u == us || (u > us && e < 0) || (u < us && e > 0)) {
    pidI += pid.ki * e * TS;
    if (pidI > 255) pidI = 255;
    if (pidI < -255) pidI = -255;
  }
  return (int16_t)lround(us);
}

// ---------------------------------------------------------------- RL 정책 (rl/env.py 와 같은 관측·행동, config = data/20261115_rl8/config.json)
const uint8_t RL_DUMAX = 10;            // 한 결정의 최대 PWM 증분
const int16_t RL_ULO = 40, RL_UHI = 255;
const int16_t SAFE_THLO = -1000, SAFE_THHI = 7000;   // [cdeg] 벗어나면 PID 로 복귀 (학습 때 SAFE 와 같음)
const float SAFE_WMAX = 600.0f;                       // [deg/s]
const float RL_AW = 0.46651200f;        // 1 − exp(−2π·10 Hz·Ts)  ω 저역통과
const float RL_AI = 0.00332779f;        // 1 − exp(−Ts/3 s)       오차 느린 평균
const float RL_KIX = 0.012f;            // k_I·dec·Ts/du_max = 6·0.02/10  고정 적분 보조 (정규화, 1° 당)
const int16_t DU_NONE = -32768;         // 직전 결정 추론이 늦어 du 가 없었음

uint8_t rlN = 0;              // 결정 위상 (0 = 결정·적용 샘플, 1 = 그 사이)
bool rlFirst = true;          // 첫 결정 (직전 값 없음)
int16_t rlU = 0;              // 정책 PWM (영차 유지)
int16_t rlPendDu = 0;         // 마지막 추론이 고른 증분 (다음 결정 샘플에 적용)
bool rlPendOk = false;
float rlWf = 0, rlEi = 0, rlResid = 0;
float rlPrevA = 0, rlPrevXi = 0, rlPrevTh = 0, rlA = 0;
int16_t rlPrevDu = 0, rlPrevU = 0;
uint16_t rlMiss = 0;
float rlXi = 0;               // 진행 중인 결정의 적분 보조 몫

// 정수 추론 엔진 (계산 방식: rl/demo_policy.py). C 로 쓰면 곱 하나에 약 70 사이클(추론 21 ms)이라
// 곱셈-누산을 어셈블리로 씀: 1층 16×16 비트(곱 4번, AVR201 의 부호 확장 방법), 2층은 0 이 아닌 h1 만 두 행씩.
int16_t nnX[NN_IN];
uint8_t nnNz[3 * NN_H];       // 0 이 아닌 h1: [k·2, h 하위, h 상위] (h ≤ 32767 이라 상위 바이트 ≤ 127)
uint8_t nnNzN = 0;
int32_t nnAcc3 = 0;
uint8_t nnStage = 0;          // 0 쉼, 1 1층, 2 2층, 3 끝(결과 대기)
uint8_t nnRow = 0;
unsigned long nnStartUs = 0;
uint16_t nnMaxUs = 0;         // 추론 시작 → 끝 (틱 포함 벽시계) 최댓값, S 줄

// acc += Σ_i w[i]·x[i]  (w: PROGMEM int16 n 개, x: RAM int16) — 부호 있는 16×16 → 32 비트
int32_t mac16(const int16_t *w, const int16_t *x, uint8_t n, int32_t acc) {
  uint8_t z = 0;
  asm volatile(
    "1:\n\t"
    "lpm  r18, Z+\n\t"
    "lpm  r19, Z+\n\t"
    "ld   r20, X+\n\t"
    "ld   r21, X+\n\t"
    "muls  r19, r21\n\t"             // wH·xH → 바이트 2–3
    "add  %C[a], r0\n\t"
    "adc  %D[a], r1\n\t"
    "mul  r18, r20\n\t"              // wL·xL → 바이트 0–1
    "add  %A[a], r0\n\t"
    "adc  %B[a], r1\n\t"
    "adc  %C[a], %[z]\n\t"
    "adc  %D[a], %[z]\n\t"
    "mulsu r19, r20\n\t"             // wH·xL → 바이트 1–2, C = 부호 → 바이트 3 에서 빼서 부호 확장
    "sbc  %D[a], %[z]\n\t"
    "add  %B[a], r0\n\t"
    "adc  %C[a], r1\n\t"
    "adc  %D[a], %[z]\n\t"
    "mulsu r21, r18\n\t"             // xH·wL
    "sbc  %D[a], %[z]\n\t"
    "add  %B[a], r0\n\t"
    "adc  %C[a], r1\n\t"
    "adc  %D[a], %[z]\n\t"
    "dec  %[n]\n\t"
    "brne 1b\n\t"
    "clr  __zero_reg__\n\t"
    : [a] "+r"(acc), [n] "+r"(n), "+z"(w), "+x"(x)
    : [z] "r"(z)
    : "r18", "r19", "r20", "r21", "r0", "memory");
  return acc;
}

// 두 행 (2p, 2p+1) 을 0 이 아닌 h1 목록으로: a0 += Σ w0[k]·h[k], a1 += Σ w1[k]·h[k]
// w: PROGMEM [k][2] int8 (쌍 하나 128 바이트), 목록 항목 [2k, hL, hH]. h ≥ 0 이라 mulsu(부호 w × 부호 없는 바이트)
void mac2(const int8_t *w, int32_t &a0, int32_t &a1) {
  uint8_t n = nnNzN;
  if (!n) return;
  uint8_t z = 0;
  const uint8_t *lp = nnNz;
  const int8_t *zp;
  asm volatile(
    "1:\n\t"
    "ld   r22, X+\n\t"               // 2k
    "ld   r20, X+\n\t"               // hL
    "ld   r21, X+\n\t"               // hH
    "movw r30, %[w]\n\t"
    "add  r30, r22\n\t"
    "adc  r31, %[z]\n\t"
    "lpm  r18, Z+\n\t"               // w0
    "lpm  r19, Z\n\t"                // w1
    "mulsu r18, r20\n\t"             // w0·hL → 바이트 0–1, 부호 확장은 2–3 에서 빼기
    "sbc  %C[a], %[z]\n\t"
    "sbc  %D[a], %[z]\n\t"
    "add  %A[a], r0\n\t"
    "adc  %B[a], r1\n\t"
    "adc  %C[a], %[z]\n\t"
    "adc  %D[a], %[z]\n\t"
    "mulsu r18, r21\n\t"             // w0·hH → 바이트 1–2
    "sbc  %D[a], %[z]\n\t"
    "add  %B[a], r0\n\t"
    "adc  %C[a], r1\n\t"
    "adc  %D[a], %[z]\n\t"
    "mulsu r19, r20\n\t"             // w1·hL
    "sbc  %C[b], %[z]\n\t"
    "sbc  %D[b], %[z]\n\t"
    "add  %A[b], r0\n\t"
    "adc  %B[b], r1\n\t"
    "adc  %C[b], %[z]\n\t"
    "adc  %D[b], %[z]\n\t"
    "mulsu r19, r21\n\t"             // w1·hH
    "sbc  %D[b], %[z]\n\t"
    "add  %B[b], r0\n\t"
    "adc  %C[b], r1\n\t"
    "adc  %D[b], %[z]\n\t"
    "dec  %[n]\n\t"
    "brne 1b\n\t"
    "clr  __zero_reg__\n\t"
    : [a] "+r"(a0), [b] "+r"(a1), [n] "+r"(n), "+x"(lp), "=&z"(zp)
    : [w] "r"(w), [z] "r"(z)
    : "r18", "r19", "r20", "r21", "r22", "r0", "memory");
}

// 산술 시프트 (변수 시프트는 비트마다 반복이라 16·8 비트는 바이트 이동으로 먼저)
int32_t shr(int32_t v, uint8_t s) {
  if (s >= 16) { v >>= 16; s -= 16; }
  if (s >= 8) { v >>= 8; s -= 8; }
  while (s--) v >>= 1;
  return v;
}

int16_t clampH(int32_t v) { return v < 0 ? 0 : (v > 32767 ? 32767 : (int16_t)v); }

// 한 조각 계산 (약 0.2–0.35 ms). 1층은 16 뉴런, 2층은 4 뉴런씩. 끝나면 nnStage = 3
void nnWork() {
  if (nnStage == 1) {
    for (uint8_t r = 0; r < 16; ++r, ++nnRow) {
      int32_t a = mac16(NN_W1 + nnRow * NN_IN, nnX, NN_IN, (int32_t)pgm_read_dword(&NN_B1[nnRow]));
      int16_t h = clampH(shr(a, pgm_read_byte(&NN_S1[nnRow])));
      if (h > 0) {
        uint8_t *e = nnNz + 3 * nnNzN++;
        e[0] = nnRow * 2; e[1] = h & 0xFF; e[2] = h >> 8;
      }
    }
    if (nnRow >= NN_H) { nnStage = 2; nnRow = 0; nnAcc3 = 0; }
  } else if (nnStage == 2) {
    for (uint8_t c = 0; c < 2 && nnRow < NN_H; ++c) {
    int32_t a0 = (int32_t)pgm_read_dword(&NN_B2[nnRow]), a1 = (int32_t)pgm_read_dword(&NN_B2[nnRow + 1]);
    mac2(NN_W2P + nnRow * NN_H, a0, a1);      // 쌍 p = nnRow/2 의 시작 = p·128 = nnRow·64
    for (uint8_t q = 0; q < 2; ++q) {
      int16_t h = clampH(shr(q ? a1 : a0, pgm_read_byte(&NN_S2[nnRow + q])));
      if (h) nnAcc3 += (int32_t)(int16_t)pgm_read_word(&NN_W3[nnRow + q]) * h;
    }
    nnRow += 2;
    }
    if (nnRow >= NN_H) nnStage = 3;
  }
}

float nnMu() { return (float)nnAcc3 * NN_M3 + NN_C3; }

int16_t qx(float v) {
  float s = v * (float)(1 << NN_XS_BITS);
  return s > 32767.0f ? 32767 : (s < -32767.0f ? -32767 : (int16_t)lround(s));
}

void nnStart() { nnStage = 1; nnRow = 0; nnNzN = 0; nnStartUs = micros(); }

// 추론이 끝났으면 행동 → 정수 증분 (행동 곡선 a·|a|, 고정 적분 보조, 시그마-델타). env.on_D 의 뒷부분
void nnFinish() {
  if (nnStage != 3 || !rlOn) return;
  nnStage = 0;
  unsigned long took = micros() - nnStartUs;
  if (took > 65535UL) took = 65535UL;
  if (took > nnMaxUs) nnMaxUs = took;
  float a = tanh(nnMu());
  if (a > 1) a = 1;
  if (a < -1) a = -1;
  float x = a * fabs(a) + rlXi;
  rlResid += x * RL_DUMAX;
  long du = lround(rlResid);
  if (du > RL_DUMAX) du = RL_DUMAX;
  if (du < -RL_DUMAX) du = -RL_DUMAX;
  rlResid -= du;
  if (rlResid > 1) rlResid = 1;
  if (rlResid < -1) rlResid = -1;
  rlA = a;
  rlPrevA = a;
  rlPrevDu = du;
  rlPendDu = du;
  rlPendOk = true;
}

// 정책 PWM 범위: u_lo 먼저, min(u_hi, LIM) 나중 (LIM 이 u_lo 보다 낮으면 LIM 이 이김 — RL 펌웨어와 같음)
int16_t clampRl(int16_t u) {
  if (u < RL_ULO) u = RL_ULO;
  int16_t hi = min(RL_UHI, uMaxPwm);
  return u > hi ? hi : u;
}

// u0 = 이번 샘플에 나가는 PID 출력 → 다음 샘플(n = 0)부터 RL 이 이어받음
void rlStart(int16_t u0) {
  rlOn = true;
  rlFirst = true;
  rlN = 0;
  rlU = clampRl(u0);
  rlPendOk = false;
  rlResid = 0;
  rlEi = 0;
  rlA = 0;
  rlMiss = 0;
  nnStage = 0;
  Serial.println(F("EVT RL ON"));
}

// RL → PID (bumpless: 적분항 I = u − ff)
void rlToPid() {
  rlOn = false;
  nnStage = 0;
  settleCnt = 0;
  pidReset();
  float i = uOut - pid.ff;
  pidI = i > 255 ? 255 : (i < -255 ? -255 : i);
}

// RL 한 샘플. thC/omC = D 줄과 같은 cdeg 반올림 값 (학습기도 D 줄 값으로 관측을 만듦)
int16_t rlStep(long thC, long omC) {
  float th = thC * 0.01f, om = omC * 0.01f;
  if (rlN == 0 && !rlFirst) {                 // 적용 시점 = 다음 결정 샘플 (lag = dec = 2)
    if (rlPendOk) {
      rlU = clampRl(rlU + rlPendDu);
    } else {
      rlMiss++;
      nnStage = 0;                            // 늦은 추론은 버림
      rlPrevDu = DU_NONE;
    }
    rlPendOk = false;
  }
  bool badTh = thC < SAFE_THLO || thC > SAFE_THHI;
  if (badTh || fabs(om) > SAFE_WMAX) {
    Serial.print(F("EVT FALLBACK ")); Serial.println(badTh ? F("THETA") : F("OMEGA"));
    wantRl = false;
    rlToPid();
    return pidStep(refDeg);
  }
  float e = refCdeg * 0.01f - th;
  if (rlFirst) rlWf = om;
  else rlWf += RL_AW * (om - rlWf);
  rlEi += RL_AI * (e - rlEi);
  if (rlN == 0) {                             // 결정 샘플: 관측 → 추론 시작
    float aStore = 0, dth = 0;
    if (!rlFirst) {
      int16_t dEff = rlU - rlPrevU;
      if (dEff == rlPrevDu) {
        aStore = rlPrevA;                     // 보낸 그대로 적용됨 → 정책이 고른 연속값
      } else {                                // 누락·범위 자르기 → 실제 정책 몫을 정책 공간으로
        float xp = (float)dEff / RL_DUMAX - rlPrevXi;
        if (xp > 1) xp = 1;
        if (xp < -1) xp = -1;
        aStore = xp < 0 ? -sqrt(-xp) : sqrt(xp);
      }
      dth = th - rlPrevTh;
      if (dth > 5) dth = 5;
      if (dth < -5) dth = -5;
    }
    nnX[0] = qx(e / 10.0f);
    nnX[1] = qx((th - 25.0f) / 10.0f);
    nnX[2] = qx(rlWf / 100.0f);
    nnX[3] = qx(dth);
    nnX[4] = qx((rlU - 125.0f) / 85.0f);
    nnX[5] = qx(aStore);
    nnX[6] = qx(rlEi / 2.0f);
    nnX[7] = qx(tanh(e / 0.5f));
    rlXi = RL_KIX * e;
    rlPrevXi = rlXi;
    rlPrevU = rlU;
    rlPrevTh = th;
    rlPrevDu = DU_NONE;                       // 추론이 끝나면 nnFinish 가 채움
    nnStart();
  }
  rlFirst = false;
  if (++rlN >= 2) rlN = 0;
  return rlU;
}

// ---------------------------------------------------------------- motor
void applyMotor(int16_t u) {
  if (u < 0) u = 0;
  if (u > uMaxPwm) u = uMaxPwm;
  if (bench || u == 0) {
    analogWrite(ENA_PIN, 0);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  } else {
    if (uOut == 0) {
      digitalWrite(IN1_PIN, HIGH);
      digitalWrite(IN2_PIN, LOW);
    }
    analogWrite(ENA_PIN, (uint8_t)u);
  }
  uOut = u;
}

void stopAll() {
  hold = false;
  rlOn = false;
  nnStage = 0;
  settleCnt = 0;
  pidI = 0;
  applyMotor(0);
}

void abortRun(const __FlashStringHelper *why) {
  bool was = hold || uOut != 0;
  stopAll();
  if (was) { Serial.print(F("EVT ABORT ")); Serial.println(why); }
}

// ---------------------------------------------------------------- MPU6050
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(imuAddr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) return false;
  if (Wire.requestFrom(addr, len) != len || Wire.getWireTimeoutFlag()) return false;
  for (uint8_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

bool imuErrReported = false;
uint8_t probeImu(uint32_t clock) {
  Wire.begin();
  Wire.setClock(clock);
  Wire.setWireTimeout(25000, true);
  digitalWrite(SDA, LOW);   // 우노 내부 5V 풀업 끔
  digitalWrite(SCL, LOW);
  const uint8_t addrs[] = {0x68, 0x69};
  for (uint8_t a : addrs) {
    uint8_t id = 0;
    if (!readRegs(a, 0x75, &id, 1)) continue;
    if (id == 0x68 || id == 0x72 || id == 0x70 || id == 0x98) return a;
  }
  return 0;
}

bool imuInit() {
  imuAddr = probeImu(I2C_CLOCK);
  if (!imuAddr) imuAddr = probeImu(100000);
  if (!imuAddr) {
    if (!imuErrReported) Serial.println(F("IMU ERROR NOT_FOUND"));
    imuErrReported = true;
    return false;
  }
  writeReg(0x6B, 0x80);
  delay(100);
  bool ok = writeReg(0x6B, 0x01) && writeReg(0x6C, 0x00) && writeReg(0x19, 0x00) &&
            writeReg(0x1A, cfg.dlpf) && writeReg(0x1B, GYRO_FS_REG) && writeReg(0x1C, ACC_FS_REG);
  uint8_t chk[2];
  ok = ok && readRegs(imuAddr, 0x1B, chk, 2) && chk[0] == GYRO_FS_REG && chk[1] == ACC_FS_REG;
  if (!ok) {
    if (!imuErrReported) Serial.println(F("IMU ERROR CONFIG"));
    imuErrReported = true;
    return false;
  }
  delay(50);
  readFails = 0;
  seedPending = true;
  Wire.setWireTimeout(3000, true);
  imuErrReported = false;
  Serial.println(F("IMU READY"));
  return true;
}

bool imuRead() {
  uint8_t b[14];
  if (!readRegs(imuAddr, 0x3B, b, 14)) return false;
  int16_t v[7];
  for (uint8_t i = 0; i < 7; ++i) v[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  for (uint8_t i = 0; i < 7; ++i) {
    if (i == 3) continue;
    if (v[i] >= SAT_RAW || v[i] <= -SAT_RAW) { satCount++; break; }
  }
  for (uint8_t i = 0; i < 3; ++i) {
    acc[i] = v[i] / ACC_LSB - cfg.accOff[i];
    gyr[i] = v[4 + i] / GYRO_LSB - cfg.gyroBias[i];
  }
  return true;
}

float pivotAngle(const float g[3]) {
  uint8_t k = cfg.axis, i = (k + 1) % 3, j = (k + 2) % 3;
  float cross = cfg.g0[i] * g[j] - cfg.g0[j] * g[i];
  float dot = cfg.g0[i] * g[i] + cfg.g0[j] * g[j];
  return -cfg.sign * atan2(cross, dot) * DEG;
}

// ---------------------------------------------------------------- sampling
char *putNum(char *p, long v) {
  *p++ = ',';
  ltoa(v, p, 10);
  return p + strlen(p);
}

// 제어 모드: 0 대기, 1 PID 유지, 2 RL 대기(PID 로 정착 중), 3 RL
uint8_t ctrlMode() { return !hold ? 0 : (rlOn ? 3 : (wantRl ? 2 : 1)); }

// mode = 이 줄의 pwm 을 실제로 만든 제어기 (RL 이 시작·복귀하는 샘플도 정확히)
void sendData(long thC, long omC, uint8_t mode) {
  // D,t_ms,pwm,theta_cdeg,omega_cdps,thetaAcc_cdeg,mode,ref_cdeg,a_milli
  char b[72];
  char *p = b;
  *p++ = 'D';
  p = putNum(p, (long)millis());
  p = putNum(p, uOut);
  p = putNum(p, thC);
  p = putNum(p, omC);
  p = putNum(p, lround(thetaAcc * 100));
  p = putNum(p, mode);
  p = putNum(p, refCdeg);
  p = putNum(p, mode == 3 ? lround(rlA * 1000) : 0);
  *p++ = '\r'; *p++ = '\n';
  Serial.write(reinterpret_cast<const uint8_t *>(b), p - b);
}

void sendStatus() {
  // S,state,imu,cal,zero,maxTick_us,overruns,i2cErr,gate_pct,sat,nn_us,miss,load_pct
  Serial.print(F("S,")); Serial.print(cal.active ? 3 : (ctrlMode() ? 4 : 0));
  Serial.print(','); Serial.print(imuOk);
  Serial.print(','); Serial.print(cfg.calValid);
  Serial.print(','); Serial.print(cfg.zeroValid);
  Serial.print(','); Serial.print(maxTickUs);
  Serial.print(','); Serial.print(overruns);
  Serial.print(','); Serial.print(i2cErrors);
  Serial.print(','); Serial.print(tickCount ? (uint16_t)((uint32_t)gatedCount * 100 / tickCount) : 0);
  Serial.print(','); Serial.print(satCount);
  Serial.print(','); Serial.print(nnMaxUs);
  Serial.print(','); Serial.print(rlMiss);
  Serial.print(','); Serial.println(tickSumUs / 10000);
  maxTickUs = 0; nnMaxUs = 0; tickSumUs = 0;
  satCount = 0; tickCount = 0; gatedCount = 0;
}

void sampleStep() {
  float g[3];
  ahrs.gravity(g);
  theta = pivotAngle(g);
  omega = cfg.sign * gyr[cfg.axis];
  if (outCount + 1 >= outDiv) {                // θ_acc(진단용)는 D 줄을 보내는 샘플에서만
    float an = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if (an > 0.1f) {
      float ga[3] = {acc[0] / an, acc[1] / an, acc[2] / an};
      thetaAcc = pivotAngle(ga);
    }
  }
  long thC = lround(theta * 100), omC = lround(omega * 100);

  int16_t u = 0;
  uint8_t mode = 0;
  if (hold) {
    if (rlOn) {
      u = rlStep(thC, omC);
      mode = rlOn ? 3 : 1;                     // FALLBACK 이면 이 샘플부터 PID
    } else {
      u = pidStep(refDeg);
      mode = wantRl ? 2 : 1;
      if (wantRl) {                            // RL 대기: |e| < 2° 를 1 s 유지하면 RL 로
        if (fabs(refDeg - theta) < 2.0f) { if (++settleCnt >= 100) rlStart(u); }
        else settleCnt = 0;
      }
    }
  }
  if (u != 0 && fabs(theta) * 100 > angleLimitCdeg) {   // 안전: 각도 한계
    abortRun(F("ANGLE_LIMIT"));
    u = 0;
    mode = 0;
  }
  applyMotor(u);
  if (++outCount >= outDiv) { outCount = 0; sendData(thC, omC, mode); }
}

#if DEMO_CAL
// 정지 보정 (자이로 바이어스만): 1000 틱(5 s) 평균. 세 축 중 하나라도 최대−최소 > 2.5 °/s 면 움직임으로 보고 실패
// (실험용 펌웨어의 σ > 0.4 °/s 판정과 비슷한 수준. 플래시 때문에 Welford·가속도 판정은 뺌)
void calAccumulate() {
  for (uint8_t i = 0; i < 3; ++i) {
    float g = gyr[i] + cfg.gyroBias[i];       // 보정 전 값
    cal.sum[i] += g;
    if (cal.n == 0 || g < cal.lo[i]) cal.lo[i] = g;
    if (cal.n == 0 || g > cal.hi[i]) cal.hi[i] = g;
  }
  if (++cal.n < 1000) return;
  cal.active = false;
  float rng = 0;
  for (uint8_t i = 0; i < 3; ++i) rng = max(rng, cal.hi[i] - cal.lo[i]);
  Serial.print(rng > 2.5f ? F("CAL FAIL MOVING range=") : F("CAL OK range=")); printFix(rng, 2); Serial.println();
  if (rng > 2.5f) return;
  for (uint8_t i = 0; i < 3; ++i) cfg.gyroBias[i] = cal.sum[i] * 0.001f;
  cfg.calValid = 1;
  saveConfig();
  seedPending = true;
}
#endif

void tick(float dt) {
  if (!imuOk) {                // 벤치 모드 + 센서 없음: θ=0
    for (uint8_t i = 0; i < 3; ++i) gyr[i] = acc[i] = 0;
    if (++divCount >= STREAM_DIV) { divCount = 0; sampleStep(); }
    return;
  }
  if (!imuRead()) {
    i2cErrors++;
    if (++readFails >= 3) {
      imuOk = false;
      cal.active = false;
      abortRun(F("IMU_FAIL"));
      Serial.println(F("IMU ERROR READ"));
      lastImuRetry = millis();
    }
    return;
  }
  readFails = 0;
  if (seedPending) {
    for (uint8_t i = 0; i < 3; ++i) accF[i] = acc[i];
    ahrs.seed(acc[0], acc[1], acc[2]);
    seedPending = false;
  }
  for (uint8_t i = 0; i < 3; ++i) accF[i] += ACC_LPF_ALPHA * (acc[i] - accF[i]);
  float aN = sqrt(accF[0] * accF[0] + accF[1] * accF[1] + accF[2] * accF[2]);
  tickCount++;
  if (fabs(aN - 1.0f) <= ACC_GATE_G) {
    ahrs.update(gyr[0] * RAD, gyr[1] * RAD, gyr[2] * RAD, accF[0], accF[1], accF[2], dt);
  } else {
    gatedCount++;
    ahrs.update(gyr[0] * RAD, gyr[1] * RAD, gyr[2] * RAD, 0, 0, 0, dt);
  }
#if DEMO_CAL
  if (cal.active) calAccumulate();
#endif
  if (++divCount >= STREAM_DIV) { divCount = 0; sampleStep(); }
}

// ---------------------------------------------------------------- commands
void printConfig() {
  Serial.print(F("CFG axis=")); Serial.print((char)('X' + cfg.axis));
  Serial.print(F(" sign=")); Serial.print(cfg.sign > 0 ? '+' : '-');
  Serial.print(F(" beta=")); Serial.print(cfg.betaMilli);
  Serial.print(F(" dlpf=")); Serial.print(cfg.dlpf);
  Serial.print(F(" ts_ms=10 out_ms=")); Serial.print(10 * outDiv);
  Serial.print(F(" umax=")); Serial.print(uMaxPwm);
  Serial.print(F(" alim=")); Serial.print(angleLimitCdeg);
  Serial.print(F(" bench=")); Serial.print(bench);
  Serial.print(F(" cal=")); Serial.print(cfg.calValid);
  Serial.print(F(" zero=")); Serial.print(cfg.zeroValid);
  Serial.print(F(" eeprom=")); Serial.println(cfgValid);
}

void printFix(float x, uint8_t d) {
  long m = 1;
  for (uint8_t i = 0; i < d; ++i) m *= 10;
  long v = lround(x * m);
  if (v < 0) { Serial.write('-'); v = -v; }
  Serial.print(v / m);
  Serial.write('.');
  long f = v % m;
  for (long q = m / 10; q > 0; q /= 10) Serial.write('0' + (f / q) % 10);
}

void printPid() {
  Serial.print(F("PIDCFG kp=")); printFix(pid.kp, 3);
  Serial.print(F(" ki=")); printFix(pid.ki, 3);
  Serial.print(F(" kd=")); printFix(pid.kd, 3);
  Serial.print(F(" ff=")); Serial.print(pid.ff);
  Serial.print(F(" notch=")); Serial.print(pid.nfc);
  Serial.print(F(" zz=")); Serial.print(pid.nzz);
  Serial.print(F(" zp=")); Serial.print(pid.nzp);
  Serial.print(F(" umin=")); Serial.println(pid.umin);
}

void printMode() {
  Serial.print(F("MODE ")); Serial.println(wantRl ? F("RL") : F("PID"));
}

void err(const __FlashStringHelper *code) {
  Serial.print(F("ERR ")); Serial.println(code);
}

uint8_t parseInts(char *s, long *out, uint8_t maxN) {
  uint8_t n = 0;
  for (;;) {
    while (*s == ' ') ++s;
    if (!*s) return n;
    if (n >= maxN) return 255;
    bool neg = false;
    if (*s == '-' || *s == '+') { neg = *s == '-'; ++s; }
    if (*s < '0' || *s > '9') return 255;
    long v = 0;
    while (*s >= '0' && *s <= '9') {
      v = v * 10 + (*s++ - '0');
      if (v > 100000000L) return 255;
    }
    if (*s && *s != ' ') return 255;
    out[n++] = neg ? -v : v;
  }
}

bool busy() { return hold || cal.active || uOut != 0; }

void command(char *line) {
  lastHeartbeat = millis();
  long v[8];
  if (strcmp(line, "PING") == 0) {
    return;
  } else if (strcmp(line, "HELLO") == 0) {
    stopAll();
    Serial.print(F("READY ")); Serial.println(F(FW_ID));
    Serial.println(imuOk ? F("IMU READY") : F("IMU ERROR NOT_READY"));
    printConfig();
    printPid();
    printMode();
  } else if (strcmp(line, "STOP") == 0) {
    bool calWas = cal.active;
    cal.active = false;
    stopAll();
    if (calWas) Serial.println(F("CAL FAIL CANCELLED"));
    Serial.println(F("EVT STOP"));
  } else if (strcmp(line, "GET") == 0) {
    printConfig();
    printPid();
    printMode();
  } else if (strncmp(line, "REF ", 4) == 0) {      // REF r_cdeg : 고른 제어기로 기준 유지 시작 / 기준 변경
    if (cal.active) { err(F("BUSY")); return; }
    if (!imuOk && !bench) { err(F("IMU_NOT_READY")); return; }
    if (parseInts(line + 4, v, 1) != 1 || labs(v[0]) > 9000) { err(F("BAD_ARGS")); return; }
    if (!hold) { pidReset(); hold = true; settleCnt = 0; }
    refCdeg = v[0];
    refDeg = refCdeg * 0.01f;
    Serial.print(F("EVT REF ")); Serial.println(v[0]);
  } else if (strcmp(line, "MODE RL") == 0) {       // 제어기 고르기 (유지 중이면 RL 은 정착 후 넘어감)
    wantRl = true;
    settleCnt = 0;
    printMode();
  } else if (strcmp(line, "MODE PID") == 0) {
    wantRl = false;
    if (rlOn) { rlToPid(); Serial.println(F("EVT RL OFF")); }
    printMode();
  } else if (strncmp(line, "PID ", 4) == 0) {      // PID kp_milli ki_milli kd_milli ff_pwm [umin_pwm] (실행 중 변경 가능)
    uint8_t k = parseInts(line + 4, v, 5);
    if (k == 4) v[4] = pid.umin;
    if ((k != 4 && k != 5) || v[0] < 0 || v[1] < 0 || v[2] < 0 || v[0] > 200000 || v[1] > 500000 ||
        v[2] > 50000 || v[3] < 0 || v[3] > 255 || v[4] < 0 || v[4] > 255) { err(F("BAD_ARGS")); return; }
    pid.kp = v[0] * 0.001f; pid.ki = v[1] * 0.001f; pid.kd = v[2] * 0.001f; pid.ff = v[3]; pid.umin = v[4];
    printPid();
  } else if (strncmp(line, "NOTCH ", 6) == 0) {    // NOTCH f_centiHz zz_milli zp_milli  /  NOTCH 0
    uint8_t k = parseInts(line + 6, v, 3);
    if (k == 1 && v[0] == 0) pid.nfc = 0;
    else if (k == 3 && v[0] >= 10 && v[0] <= 2000 && v[1] >= 0 && v[1] <= 2000 && v[2] > 0 && v[2] <= 2000) {
      pid.nfc = v[0]; pid.nzz = v[1]; pid.nzp = v[2];
    } else { err(F("BAD_ARGS")); return; }
    nz1 = nz2 = 0;
    notchCoeffs();
    printPid();
  } else if (strncmp(line, "LIM ", 4) == 0) {      // LIM umax_pwm alim_cdeg
    if (parseInts(line + 4, v, 2) != 2 || v[0] < 0 || v[0] > 255 || v[1] < 500 || v[1] > 18000) { err(F("BAD_ARGS")); return; }
    uMaxPwm = v[0]; angleLimitCdeg = v[1];
    printConfig();
  } else if (strncmp(line, "OUT ", 4) == 0) {      // OUT 1|2 : D 줄 주기 10 / 20 ms
    if (parseInts(line + 4, v, 1) != 1 || v[0] < 1 || v[0] > 2) { err(F("BAD_ARGS")); return; }
    outDiv = v[0];
    outCount = 0;
    printConfig();
  } else if (strncmp(line, "BENCH ", 6) == 0) {
    if (parseInts(line + 6, v, 1) != 1 || v[0] < 0 || v[0] > 1) { err(F("BAD_ARGS")); return; }
    if (busy()) { err(F("BUSY")); return; }
    bench = v[0] != 0;
    applyMotor(0);
    if (bench && !imuOk) { nextTickUs = micros(); lastTickUs = nextTickUs; }
    printConfig();
  } else if (strncmp(line, "BETA ", 5) == 0) {
    if (parseInts(line + 5, v, 1) != 1 || v[0] < 1 || v[0] > 1000) { err(F("BAD_ARGS")); return; }
    if (!cfgValid) { err(F("NO_EEPROM")); return; }
    cfg.betaMilli = v[0];
    ahrs.beta = cfg.betaMilli * 0.001f;
    saveConfig();
    printConfig();
  } else if (strcmp(line, "ZERO") == 0) {          // 진자가 정지해 매달린 자세 = θ 0
    if (!imuOk) { err(F("IMU_NOT_READY")); return; }
    if (!cfgValid) { err(F("NO_EEPROM")); return; }
    if (busy()) { err(F("BUSY")); return; }
    float g[3];
    ahrs.gravity(g);
    uint8_t i = (cfg.axis + 1) % 3, j = (cfg.axis + 2) % 3;
    if (g[i] * g[i] + g[j] * g[j] < 0.25f) { err(F("AXIS_PARALLEL_TO_GRAVITY")); return; }
    for (uint8_t k = 0; k < 3; ++k) cfg.g0[k] = g[k];
    cfg.zeroValid = 1;
    saveConfig();
    Serial.println(F("ZERO OK"));
#if DEMO_CAL
  } else if (strcmp(line, "CAL") == 0) {           // 정지 보정 (자이로 바이어스)
    if (!imuOk) { err(F("IMU_NOT_READY")); return; }
    if (!cfgValid) { err(F("NO_EEPROM")); return; }
    if (busy()) { err(F("BUSY")); return; }
    memset(&cal, 0, sizeof(cal));
    cal.active = true;
    Serial.println(F("CAL START"));
#endif
#if NN_TEST
  } else if (strncmp(line, "NN ", 3) == 0) {       // NN x0..x7 (o·2^11) : 정책 계산 시험 → NN acc3 mu×1e6 us
    if (rlOn) { err(F("BUSY")); return; }
    if (parseInts(line + 3, v, 8) != 8) { err(F("BAD_ARGS")); return; }
    for (uint8_t i = 0; i < NN_IN; ++i) nnX[i] = v[i] < -32767 ? -32767 : (v[i] > 32767 ? 32767 : v[i]);
    unsigned long t0 = micros();
    nnStart();
    while (nnStage != 3) nnWork();
    unsigned long t1 = micros();
    nnStage = 0;
    Serial.print(F("NN ")); Serial.print(nnAcc3);
    Serial.print(' '); Serial.print(lround(nnMu() * 1e6f));
    Serial.print(' '); Serial.println(t1 - t0);
#endif
  } else {
    err(F("UNKNOWN_COMMAND"));
  }
}

char input[72];
uint8_t inputLen = 0;
bool discarding = false;
unsigned long lastByteAt = 0;

void readCommands() {
  if ((inputLen || discarding) && millis() - lastByteAt > 250) {
    inputLen = 0;
    discarding = false;
    err(F("COMMAND_TIMEOUT"));
  }
  for (uint8_t budget = 0; budget < 64 && Serial.available(); ++budget) {
    char c = Serial.read();
    lastByteAt = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!discarding && inputLen) { input[inputLen] = '\0'; command(input); }
      inputLen = 0;
      discarding = false;
    } else if (!discarding) {
      if (c < 32 || c > 126 || inputLen >= sizeof(input) - 1) {
        discarding = true;
        inputLen = 0;
        abortRun(F("INVALID_LINE"));
        err(F("INVALID_LINE"));
      } else {
        input[inputLen++] = c;
      }
    }
  }
}

// ---------------------------------------------------------------- main
void setup() {
  MCUSR = 0;
  wdt_disable();
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  Serial.begin(BAUD);
  loadConfig();
  ahrs.beta = cfg.betaMilli * 0.001f;
  notchCoeffs();
  delay(200);
  Serial.print(F("READY ")); Serial.println(F(FW_ID));
  if (!cfgValid) Serial.println(F("NOTE NO_EEPROM_CONFIG (calibrate with lab firmware first)"));
  imuOk = imuInit();
  lastImuRetry = millis();
  printConfig();
  printPid();
#if USE_WATCHDOG
  wdt_enable(WDTO_500MS);
#endif
  lastHeartbeat = millis();
  nextTickUs = micros();
  lastTickUs = nextTickUs;
}

void loop() {
#if USE_WATCHDOG
  wdt_reset();
#endif
  readCommands();

  if ((uOut != 0 || hold) && millis() - lastHeartbeat > LINK_TIMEOUT_MS) abortRun(F("LINK_TIMEOUT"));
  if (!imuOk && !bench && uOut != 0) applyMotor(0);
  if (!imuOk && !(bench && hold) && millis() - lastImuRetry >= 1000) {
    lastImuRetry = millis();
    imuOk = imuInit();
    if (imuOk) { seedPending = true; nextTickUs = micros(); lastTickUs = nextTickUs; }
  }
  bool ticked = false;
  if (imuOk || bench) {
    unsigned long now = micros();
    if ((long)(now - nextTickUs) >= 0) {
      nextTickUs += TICK_US;
      if ((long)(now - nextTickUs) >= 0) { overruns++; nextTickUs = now + TICK_US; }
      float dt = (now - lastTickUs) * 1e-6f;
      lastTickUs = now;
      if (dt <= 0 || dt > 0.05f) dt = TICK_S;
      tick(dt);
      unsigned long tookL = micros() - now;
      uint16_t took = tookL > 65535UL ? 65535 : (uint16_t)tookL;
      if (took > maxTickUs) maxTickUs = took;
      tickSumUs += took;
      ticked = true;
    }
  }
  // 틱이 없을 때만 추론 한 조각 (틱 지연 최대 = 한 조각 시간)
  if (!ticked && nnStage && rlOn) {
    if (nnStage < 3) nnWork();
    if (nnStage == 3) nnFinish();
  }

  // S 줄은 1 s 마다, 추론 중에는 미룸 (전송 버퍼가 차면 Serial 이 막혀 추론이 20 ms 를 넘김)
  if (millis() - lastStatusMs >= 1000 && !nnStage) {
    lastStatusMs = millis();
    sendStatus();
  }
}
