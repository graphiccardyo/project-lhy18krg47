// Aeropendulum experiment firmware — Arduino Uno + L298N + MPU6050 (GY-521)
//
// 목표: 개루프 시스템 식별 실험 (1) 입력-각도 정적 특성, (2) 스텝/스윕 사인/PRBS 응답,
//       (3) 전달함수 추정용 데이터 기록.
// - 외부 라이브러리 없음 (Wire, EEPROM만 사용). MPU6050 드라이버와 Madgwick IMU 필터를 직접 포함.
// - 200 Hz로 센서를 읽고 Madgwick 필터를 갱신, 샘플 주기 Ts(기본 10 ms)마다 입력을 갱신하고 1줄 전송.
// - 수평면 보정(CAL): 센서 +Z가 위를 향하게 평평하게 놓고 정지 → 자이로 바이어스 + 가속도 오프셋.
// - 영점(ZERO): 진자가 정지해 매달린 자세의 중력 방향을 θ=0 기준으로 저장.
// - 보정·설정은 EEPROM에 저장되어 재부팅 후에도 유지.
//
// 배선 (기존 프로젝트와 동일)
//   D5 → L298N ENA (ENA 점퍼 제거),  D7 → IN1,  D8 → IN2,  GND 공통
//   MPU6050: VCC→3.3V, GND→GND, SDA→A4(SDA), SCL→A5(SCL)  (모듈 풀업 사용)
//
// 시리얼 250000 baud, 줄 단위 텍스트. 프로토콜: docs/PROTOCOL.md
// 입력 단위: PWM 카운트 (analogWrite 0..255, D5 ≈ 980 Hz). 전압으로 환산하지 않음.

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define FW_ID "AEROPEND_LAB 2.1"   // 2.1 = 폐루프(PID/NOTCH/REF/CLSTEP/RELAY) 포함. UI 가 버전으로 구분
#define USE_WATCHDOG 1

// ---------------------------------------------------------------- hardware
const uint8_t ENA_PIN = 5;
const uint8_t IN1_PIN = 7;
const uint8_t IN2_PIN = 8;
const uint32_t BAUD = 250000;
const uint32_t I2C_CLOCK = 400000;      // 배선이 길거나 노이즈가 심하면 100000으로
const uint16_t TICK_US = 5000;          // 200 Hz 센서/필터 주기
const float TICK_S = TICK_US * 1e-6f;
const unsigned long LINK_TIMEOUT_MS = 1000;
// 프로펠러 진동이 센서 내부(디지털 필터 이전)에서 포화되지 않도록 최대 범위 사용 (2026-11-06 데이터 분석 결과)
const float GYRO_LSB = 16.4f;           // ±2000 dps
const float ACC_LSB = 2048.0f;          // ±16 g
const uint8_t GYRO_FS_REG = 0x18;
const uint8_t ACC_FS_REG = 0x18;
const int16_t SAT_RAW = 32000;          // 이 이상이면 포화로 집계
// Madgwick 가속도 입력 전처리: 1차 저역통과(약 5 Hz) + |a|가 1 g에서 벗어나면 보정 생략(자이로만 적분)
const float ACC_LPF_ALPHA = 0.145f;     // 1 - exp(-2π·5 Hz·5 ms)
const float ACC_GATE_G = 0.15f;
const float DEG = 57.2957795f;
const float RAD = 0.01745329252f;

// ---------------------------------------------------------------- persistent config
struct Config {
  uint16_t magic;
  uint8_t version;
  uint8_t calValid;      // 자이로 바이어스 보정됨
  uint8_t zeroValid;
  uint8_t axis;          // 0=X 1=Y 2=Z (회전축 = 진자 피벗 축)
  int8_t sign;           // +1 / -1
  uint8_t dlpf;          // MPU6050 CONFIG.DLPF_CFG 0..6
  uint16_t betaMilli;    // Madgwick beta × 1000
  float gyroBias[3];     // dps
  float accOff[3];       // g
  float g0[3];           // θ=0 일 때 센서 좌표계 중력 방향 (단위벡터)
  uint8_t crc;
};
const uint16_t CFG_MAGIC = 0xAE01;
const float DEFAULT_ACC_OFF[3] = {-0.017059f, -0.022036f, 0.147525f};   // g
const uint8_t CFG_VERSION = 4;
const int CFG_BACKUP_ADDR = 128;   // CFG BACKUP/RESTORE 예비 영역
Config cfg;

void printFix(float x, uint8_t d);
void printCal();

uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
  }
  return crc;
}

void defaultConfig() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CFG_MAGIC;
  cfg.version = CFG_VERSION;
  cfg.axis = 0;
  cfg.sign = 1;
  cfg.dlpf = 4;          // 21 Hz
  cfg.betaMilli = 20;    // β = 0.02 (자이로 위주)
  cfg.g0[2] = 1.0f;
  // 가속도 오프셋 기본값 = 이전 수동 보정 저장값 (data/calibration/mpu6050-calibration-20261031.json, 같은 GY-521 모듈)
  for (uint8_t i = 0; i < 3; ++i) cfg.accOff[i] = DEFAULT_ACC_OFF[i];
}

void saveConfig() {
  cfg.crc = crc8(reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg) - 1);
  EEPROM.put(0, cfg);   // put()은 바뀐 바이트만 씀
}

bool cfgMigrated = false;
void loadConfig() {
  EEPROM.get(0, cfg);
  // v3 → v4: 축/부호/보정/영점/가속도 오프셋은 유지하고 필터 설정만 새 기본값으로
  if (cfg.magic == CFG_MAGIC && cfg.version == 3 &&
      cfg.crc == crc8(reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg) - 1)) {
    cfg.version = CFG_VERSION;
    cfg.dlpf = 4;
    cfg.betaMilli = 20;
    saveConfig();
    cfgMigrated = true;
  }
  bool ok = cfg.magic == CFG_MAGIC && cfg.version == CFG_VERSION &&
            cfg.crc == crc8(reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg) - 1) &&
            cfg.axis < 3 && (cfg.sign == 1 || cfg.sign == -1) && cfg.dlpf <= 6 &&
            cfg.betaMilli >= 1 && cfg.betaMilli <= 1000;
  if (!ok) defaultConfig();
}

// ---------------------------------------------------------------- Madgwick IMU (6축)
// S. Madgwick, "An efficient orientation filter for inertial and inertial/magnetic sensor arrays", 2010.
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

  // gyro: rad/s, acc: 임의 단위, dt: s
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

  // 센서 좌표계에서 본 중력(지구 +Z) 방향
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

float gyr[3];       // dps, 바이어스 보정 후
float acc[3];       // g, 오프셋 보정 후
float rawG[3];      // dps, 보정 전
float rawA[3];      // g, 보정 전
float theta = 0, omega = 0, thetaAcc = 0;   // deg, deg/s

uint8_t streamDiv = 2;      // Ts = 5 ms × streamDiv
uint8_t divCount = 0;
uint8_t auxCount = 0;
int16_t uMaxPwm = 255;        // 입력 상한 (PWM 카운트)
int16_t angleLimitCdeg = 12000;
bool allowReverse = false;     // 기본: 역추력(음수 입력) 금지
// 벤치(드라이런) 모드: 센서 없이 시험 로직만 실행. PWM 값은 기록되지만 모터 핀은 항상 LOW.
bool bench = false;

// 입력 u 의 단위는 PWM 카운트 (analogWrite 값, 0..255). 음수 = 역방향 (allowReverse 일 때만)
int16_t uManual = 0;        // −255..255
int16_t uOut = 0;           // 실제 출력 PWM
uint8_t phase = 0;          // 0 대기/수동, 1.. 프로파일 구간
unsigned long lastHeartbeat = 0;

unsigned long nextTickUs = 0;
unsigned long lastTickUs = 0;
uint16_t maxTickUs = 0;
uint16_t overruns = 0;
uint16_t i2cErrors = 0;
uint16_t satCount = 0;        // 1초 동안 포화된 샘플 수
uint16_t tickCount = 0, gatedCount = 0;   // 가속도 보정 생략 비율 집계
float accF[3] = {0, 0, 1};     // 저역통과된 가속도 (Madgwick 입력)
unsigned long lastStatusMs = 0;

// 수평 보정 (Welford)
// 보정 모드: 정지 자세(기본, 어떤 각도든 정지만 하면 됨 → 자이로 바이어스만)
//            수평면(CAL FLAT, +Z 위로 평평하게 → 자이로 바이어스 + 가속도 오프셋)
struct CalState {
  bool active = false;
  bool flat = false;
  uint16_t n = 0, target = 1000;
  float mean[6], m2[6];
  uint8_t lastPct = 0;
} cal;

// ---------------------------------------------------------------- excitation profiles
enum ProfileType : uint8_t { P_NONE, P_STEP, P_STAIR, P_CHIRP, P_PRBS, P_CLSTEP, P_RELAY };
struct Profile {
  ProfileType type = P_NONE;
  uint32_t n = 0, total = 0;
  uint32_t a = 0, b = 0, c = 0;   // 구간 길이(샘플)
  int16_t u0 = 0, u1 = 0, amp = 0, inc = 0;
  float f = 0, df = 0, kf = 1, ph = 0;
  bool logSweep = false;
  uint16_t lfsr = 0x1FF;
  int8_t level = 1;
} prof;

// ---------------------------------------------------------------- closed-loop control
// u = ff + Kp·e + Ki·∫e − Kd·ω_f  (D 는 자이로 각속도를 10 Hz 저역통과해 측정값에 적용 → 기준 변화 시 킥 없음)
// 선택: 출력단 2차 노치 (구조 공진 억제), 출력 포화 + 조건부 적분(anti-windup). 샘플 주기 Ts 마다 실행.
struct Pid {
  float kp = 0, ki = 0, kd = 0;
  int16_t ff = 150;               // 피드포워드 PWM (운전점)
  uint16_t nfc = 0;               // 노치 중심 [0.01 Hz], 0 = 끔
  uint16_t nzz = 150, nzp = 700;  // 노치 ζz, ζp ×1000
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
} pid;
float pidI = 0, omF = 0, nz1 = 0, nz2 = 0, refDeg = 0;
bool clHold = false;              // REF 명령으로 기준 유지 중

void notchCoeffs() {               // 쌍선형 변환 + 중심주파수 프리워핑
  pid.b0 = 1; pid.b1 = pid.b2 = pid.a1 = pid.a2 = 0;
  if (!pid.nfc) return;
  float ts = TICK_S * streamDiv, w = 6.2831853f * pid.nfc * 0.01f;
  float k = w / tan(w * ts * 0.5f), k2 = k * k, w2 = w * w;
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
  float ts = TICK_S * streamDiv;
  float e = r - theta;
  omF += ts / (0.0159f + ts) * (omega - omF);
  float v = pid.kp * e + pidI - pid.kd * omF;
  if (pid.nfc) {                   // 2차 IIR (전치 직접형 II)
    float y = pid.b0 * v + nz1;
    nz1 = pid.b1 * v - pid.a1 * y + nz2;
    nz2 = pid.b2 * v - pid.a2 * y;
    v = y;
  }
  float u = pid.ff + v;
  float lo = allowReverse ? -uMaxPwm : 0, hi = uMaxPwm;
  float us = u < lo ? lo : (u > hi ? hi : u);
  if (u == us || (u > us && e < 0) || (u < us && e > 0)) {
    pidI += pid.ki * e * ts;
    if (pidI > 255) pidI = 255;
    if (pidI < -255) pidI = -255;
  }
  return (int16_t)lround(us);
}

float tsSeconds() { return TICK_S * streamDiv; }
uint32_t msToSamples(long ms) {
  long ts = 5L * streamDiv;
  return ms <= 0 ? 0 : (uint32_t)((ms + ts / 2) / ts);
}

// ---------------------------------------------------------------- motor

void applyMotor(int16_t u) {
  if (u < 0 && !allowReverse) u = 0;
  if (u > uMaxPwm) u = uMaxPwm;
  if (u < -uMaxPwm) u = -uMaxPwm;
  if (bench) {                 // 드라이런: 출력 핀은 건드리지 않고 값만 기록
    analogWrite(ENA_PIN, 0);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    uOut = u;
    return;
  }
  if (u == 0) {
    analogWrite(ENA_PIN, 0);
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  } else {
    bool fwd = u > 0;
    if ((uOut > 0) != fwd || uOut == 0) {   // 방향 전환 시 먼저 출력 차단
      analogWrite(ENA_PIN, 0);
      digitalWrite(IN1_PIN, fwd ? HIGH : LOW);
      digitalWrite(IN2_PIN, fwd ? LOW : HIGH);
    }
    analogWrite(ENA_PIN, (uint8_t)(u < 0 ? -u : u));
  }
  uOut = u;
}

void stopAll() {
  prof.type = P_NONE;
  clHold = false;
  pidI = 0;
  uManual = 0;
  phase = 0;
  applyMotor(0);
}

void event(const __FlashStringHelper *what) {
  Serial.print(F("EVT ")); Serial.println(what);
}

void abortRun(const __FlashStringHelper *why) {
  bool was = prof.type != P_NONE || uOut != 0;
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

uint32_t i2cClock = I2C_CLOCK;

// 0x68/0x69 에서 WHO_AM_I 확인. 반환: 찾은 주소(0=없음), lastId = 마지막으로 읽은 ID (0xFF=무응답)
uint8_t lastId = 0xFF;
bool imuErrReported = false;   // 같은 오류를 1초마다 반복 출력하지 않음 (상태는 S 줄로 계속 보고)
uint8_t probeImu(uint32_t clock) {
  Wire.begin();
  Wire.setClock(clock);
  Wire.setWireTimeout(25000, true);
  digitalWrite(SDA, LOW);   // 우노 내부 5V 풀업 끔 (모듈의 3.3V 풀업 사용)
  digitalWrite(SCL, LOW);
  const uint8_t addrs[] = {0x68, 0x69};
  for (uint8_t a : addrs) {
    uint8_t id = 0;
    if (!readRegs(a, 0x75, &id, 1)) continue;
    lastId = id;
    if (id == 0x68 || id == 0x72 || id == 0x70 || id == 0x98) return a;   // MPU6050/호환칩
  }
  return 0;
}

bool imuInit() {
  lastId = 0xFF;
  imuAddr = probeImu(I2C_CLOCK);
  i2cClock = I2C_CLOCK;
  if (!imuAddr && I2C_CLOCK > 100000) {   // 배선이 길거나 풀업이 약하면 100 kHz로 재시도
    imuAddr = probeImu(100000);
    i2cClock = 100000;
  }
  if (!imuAddr) {
    if (imuErrReported) return false;
    imuErrReported = true;
    Serial.print(F("IMU ERROR NOT_FOUND "));
    if (lastId == 0xFF) Serial.println(F("no_ack(0x68/0x69) - check VCC/GND/SDA/SCL"));
    else { Serial.print(F("whoami=0x")); Serial.println(lastId, HEX); }
    return false;
  }
  writeReg(0x6B, 0x80);                 // device reset
  delay(100);
  bool ok = writeReg(0x6B, 0x01) &&     // PLL (gyro X) clock, wake
            writeReg(0x6C, 0x00) &&
            writeReg(0x19, 0x00) &&     // SMPLRT_DIV: 1 kHz (DLPF on)
            writeReg(0x1A, cfg.dlpf) && // DLPF
            writeReg(0x1B, GYRO_FS_REG) &&   // ±2000 dps
            writeReg(0x1C, ACC_FS_REG);      // ±16 g
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
  Serial.print(F("IMU READY addr=0x")); Serial.print(imuAddr, HEX);
  Serial.print(F(" i2c=")); Serial.println(i2cClock / 1000);
  return true;
}

bool imuRead() {
  uint8_t b[14];
  if (!readRegs(imuAddr, 0x3B, b, 14)) return false;
  int16_t v[7];
  for (uint8_t i = 0; i < 7; ++i) v[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  for (uint8_t i = 0; i < 7; ++i) {
    if (i == 3) continue;   // 온도
    if (v[i] >= SAT_RAW || v[i] <= -SAT_RAW) { satCount++; break; }
  }
  for (uint8_t i = 0; i < 3; ++i) {
    rawA[i] = v[i] / ACC_LSB;
    rawG[i] = v[4 + i] / GYRO_LSB;
    acc[i] = rawA[i] - cfg.accOff[i];
    gyr[i] = rawG[i] - cfg.gyroBias[i];
  }
  return true;
}

// ---------------------------------------------------------------- angle about pivot axis
// 기준 중력 g0와 현재 중력 g를 피벗 축에 수직인 평면에 투영해 사이 각을 구함.
// 센서가 축 +방향으로 φ 회전하면 θ=+φ 가 되어 자이로 각속도와 부호가 일치.
float pivotAngle(const float g[3]) {
  uint8_t k = cfg.axis, i = (k + 1) % 3, j = (k + 2) % 3;
  float cross = cfg.g0[i] * g[j] - cfg.g0[j] * g[i];
  float dot = cfg.g0[i] * g[i] + cfg.g0[j] * g[j];
  return -cfg.sign * atan2(cross, dot) * DEG;
}

float perpNorm(const float g[3], uint8_t k) {
  uint8_t i = (k + 1) % 3, j = (k + 2) % 3;
  return sqrt(g[i] * g[i] + g[j] * g[j]);
}

// ---------------------------------------------------------------- profiles
int16_t profileStep() {
  int16_t u = 0;
  uint32_t n = prof.n;
  switch (prof.type) {
    case P_STEP:
      if (n < prof.a) { u = prof.u0; phase = 1; }
      else if (n < prof.a + prof.b) { u = prof.u1; phase = 2; }
      else { u = prof.u0; phase = 3; }
      break;
    case P_STAIR: {
      uint32_t idx = n / prof.a;
      u = prof.u0 + (int16_t)((long)idx * prof.inc);
      phase = idx < 254 ? (uint8_t)(idx + 1) : 255;
      break;
    }
    case P_CHIRP:
      if (n < prof.a) { u = prof.u0; phase = 1; }
      else {
        u = prof.u0 + (int16_t)lround(prof.amp * sin(prof.ph));
        float ts = tsSeconds();
        prof.ph += 6.2831853f * prof.f * ts;
        if (prof.ph > 6.2831853f) prof.ph -= 6.2831853f;
        if (prof.logSweep) prof.f *= prof.kf; else prof.f += prof.df;
        phase = 2;
      }
      break;
    case P_PRBS:
      if (n < prof.a) { u = prof.u0; phase = 1; }
      else {
        if ((n - prof.a) % prof.b == 0) {   // 9비트 LFSR, x^9 + x^5 + 1 (주기 511)
          uint16_t bit = ((prof.lfsr >> 8) ^ (prof.lfsr >> 4)) & 1;
          prof.lfsr = ((prof.lfsr << 1) | bit) & 0x1FF;
          prof.level = bit ? 1 : -1;
        }
        u = prof.u0 + prof.level * prof.amp;
        phase = 2;
      }
      break;
    case P_CLSTEP:                 // u0/u1 = 기준 [cdeg]
      if (n < prof.a) { refDeg = prof.u0 * 0.01f; phase = 1; }
      else if (n < prof.a + prof.b) { refDeg = prof.u1 * 0.01f; phase = 2; }
      else { refDeg = prof.u0 * 0.01f; phase = 3; }
      u = pidStep(refDeg);
      break;
    case P_RELAY:                  // pre: PID 로 기준 유지하며 출력 평균 → 릴레이 중심, 이후 중심 ± d
      refDeg = prof.u0 * 0.01f;
      if (n < prof.a) {
        u = pidStep(refDeg); phase = 1;
        if (n >= prof.a / 2) { prof.f += u; prof.df += 1; }
      } else {
        if (n == prof.a) { prof.kf = prof.df > 0 ? prof.f / prof.df : pid.ff; prof.level = 1; }
        float e = refDeg - theta, h = prof.inc * 0.01f;
        if (e > h) prof.level = 1;
        else if (e < -h) prof.level = -1;
        u = (int16_t)lround(prof.kf + prof.level * prof.amp);
        phase = 2;
      }
      break;
    default: break;
  }
  prof.n++;
  return u;
}

// ---------------------------------------------------------------- sampling
// 숫자 변환이 루프 시간의 큰 부분이라 avr-libc ltoa(빠른 어셈블리 구현)로 한 줄을 만든 뒤 한 번에 전송
char *putNum(char *p, long v) {
  *p++ = ',';
  ltoa(v, p, 10);
  return p + strlen(p);
}

void sendData() {
  // D,t_ms,u_pwm,theta_cdeg,omega_cdps,thetaAcc_cdeg,phase[,ref_cdeg]
  char b[64];
  char *p = b;
  *p++ = 'D';
  p = putNum(p, (long)millis());
  p = putNum(p, uOut);
  p = putNum(p, lround(theta * 100));
  p = putNum(p, lround(omega * 100));
  p = putNum(p, lround(thetaAcc * 100));
  p = putNum(p, phase);
  if (clHold || prof.type == P_CLSTEP || prof.type == P_RELAY) p = putNum(p, lround(refDeg * 100));   // 8번째: 기준 [cdeg]
  *p++ = '\r'; *p++ = '\n';
  Serial.write(reinterpret_cast<const uint8_t *>(b), p - b);
}

void sendAux() {
  // A,ax,ay,az(mg, 보정 전),gx,gy,gz(cdps, 보정 전)
  char b[72];
  char *p = b;
  *p++ = 'A';
  for (uint8_t i = 0; i < 3; ++i) p = putNum(p, lround(rawA[i] * 1000));
  for (uint8_t i = 0; i < 3; ++i) p = putNum(p, lround(rawG[i] * 100));
  *p++ = '\r'; *p++ = '\n';
  Serial.write(reinterpret_cast<const uint8_t *>(b), p - b);
}

// A 줄은 샘플(D 줄) 주기와 겹치지 않는 다음 틱에 보냄 → 샘플 틱의 소요 시간을 줄임
bool auxPending = false;
void flushAux() {
  if (auxPending && (divCount != 0 || streamDiv == 1)) { auxPending = false; sendAux(); }
}

void sendStatus() {
  // S,state,imu,cal,zero,maxTick_us,overruns,i2cErr,gate_pct,sat
  uint8_t state = cal.active ? 3 : (prof.type != P_NONE ? 2 : (clHold ? 4 : (uOut != 0 ? 1 : 0)));
  Serial.print(F("S,")); Serial.print(state);
  Serial.print(','); Serial.print(imuOk);
  Serial.print(','); Serial.print(cfg.calValid);
  Serial.print(','); Serial.print(cfg.zeroValid);
  Serial.print(','); Serial.print(maxTickUs);
  Serial.print(','); Serial.print(overruns);
  Serial.print(','); Serial.print(i2cErrors);
  Serial.print(','); Serial.print(tickCount ? (uint16_t)((uint32_t)gatedCount * 100 / tickCount) : 0);
  Serial.print(','); Serial.println(satCount);
  maxTickUs = 0;
  satCount = 0; tickCount = 0; gatedCount = 0;
}

void sampleStep() {
  float g[3];
  ahrs.gravity(g);
  theta = pivotAngle(g);
  omega = cfg.sign * gyr[cfg.axis];
  float an = sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if (an > 0.1f) {
    float ga[3] = {acc[0] / an, acc[1] / an, acc[2] / an};
    thetaAcc = pivotAngle(ga);
  }

  int16_t u;
  if (prof.type != P_NONE) {
    if (prof.n >= prof.total) {
      stopAll();
      event(F("DONE"));
      u = 0;
    } else {
      u = profileStep();
    }
  } else if (clHold) {
    u = pidStep(refDeg);
    phase = 0;
  } else {
    u = uManual;
    phase = 0;
  }

  // 안전: 각도 한계 (출력 중일 때만)
  if (u != 0 && fabs(theta) * 100 > angleLimitCdeg) {
    abortRun(F("ANGLE_LIMIT"));
    u = 0;
  }
  applyMotor(u);
  sendData();
  if (++auxCount >= 10) { auxCount = 0; auxPending = true; }
}

void calAccumulate() {
  float x[6] = {rawG[0], rawG[1], rawG[2], rawA[0], rawA[1], rawA[2]};
  cal.n++;
  for (uint8_t i = 0; i < 6; ++i) {
    float d = x[i] - cal.mean[i];
    cal.mean[i] += d / cal.n;
    cal.m2[i] += d * (x[i] - cal.mean[i]);
  }
  uint8_t pct = (uint32_t)cal.n * 100 / cal.target;
  if (pct / 10 != cal.lastPct / 10) {
    cal.lastPct = pct;
    Serial.print(F("CAL PROG ")); Serial.println(pct);
  }
  if (cal.n < cal.target) return;

  cal.active = false;
  float gStd = 0, aStd = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    gStd = max(gStd, sqrt(cal.m2[i] / (cal.n - 1)));
    aStd = max(aStd, sqrt(cal.m2[3 + i] / (cal.n - 1)));
  }
  const float *m = cal.mean;
  float ax = m[3] - cfg.accOff[0], ay = m[4] - cfg.accOff[1], az = m[5] - cfg.accOff[2];
  float aNorm = sqrt(ax * ax + ay * ay + az * az);
  const __FlashStringHelper *fail = nullptr;
  if (gStd > 0.4f || aStd > 0.03f) fail = F("MOVING");
  else if (cal.flat && (m[5] < 0.75f || m[5] > 1.25f)) fail = F("NOT_Z_UP");
  else if (cal.flat && (fabs(m[3]) > 0.2f || fabs(m[4]) > 0.2f)) fail = F("NOT_FLAT");
  else if (!cal.flat && (aNorm < 0.85f || aNorm > 1.15f)) fail = F("ACC_NORM");
  else if (fabs(m[0]) > 25 || fabs(m[1]) > 25 || fabs(m[2]) > 25) fail = F("GYRO_BIAS_RANGE");
  if (fail) {
    Serial.print(F("CAL FAIL ")); Serial.print(fail);
    Serial.print(F(" gstd=")); printFix(gStd, 3);
    Serial.print(F(" astd=")); printFix(aStd, 4);
    Serial.print(F(" a=")); printFix(m[3], 3); Serial.print(' ');
    printFix(m[4], 3); Serial.print(' '); printFix(m[5], 3); Serial.println();
    return;
  }
  for (uint8_t i = 0; i < 3; ++i) cfg.gyroBias[i] = m[i];
  if (cal.flat) { cfg.accOff[0] = m[3]; cfg.accOff[1] = m[4]; cfg.accOff[2] = m[5] - 1.0f; }
  cfg.calValid = 1;
  saveConfig();
  seedPending = true;
  Serial.print(F("CAL OK mode=")); Serial.print(cal.flat ? F("FLAT") : F("STILL"));
  Serial.print(F(" gstd=")); printFix(gStd, 3);
  Serial.print(F(" astd=")); printFix(aStd, 4);
  Serial.print(F(" anorm=")); printFix(aNorm, 3); Serial.println();
  printCal();
}

void tick(float dt) {
  if (!imuOk) {                // 벤치 모드 + 센서 없음: θ=0, 시험 로직만 진행
    for (uint8_t i = 0; i < 3; ++i) { gyr[i] = acc[i] = rawG[i] = rawA[i] = 0; }
    if (++divCount >= streamDiv) { divCount = 0; sampleStep(); }
    else flushAux();
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
  } else {                     // 진동/가속 중: 가속도 보정 없이 자이로만 적분
    gatedCount++;
    ahrs.update(gyr[0] * RAD, gyr[1] * RAD, gyr[2] * RAD, 0, 0, 0, dt);
  }
  if (cal.active) calAccumulate();
  if (++divCount >= streamDiv) { divCount = 0; sampleStep(); if (streamDiv == 1) flushAux(); }
  else flushAux();
}

// ---------------------------------------------------------------- commands
void printConfig() {
  Serial.print(F("CFG axis=")); Serial.print((char)('X' + cfg.axis));
  Serial.print(F(" sign=")); Serial.print(cfg.sign > 0 ? '+' : '-');
  Serial.print(F(" beta=")); Serial.print(cfg.betaMilli);
  Serial.print(F(" dlpf=")); Serial.print(cfg.dlpf);
  Serial.print(F(" div=")); Serial.print(streamDiv);
  Serial.print(F(" ts_ms=")); Serial.print(5 * streamDiv);
  Serial.print(F(" umax=")); Serial.print(uMaxPwm);
  Serial.print(F(" alim=")); Serial.print(angleLimitCdeg);
  Serial.print(F(" rev=")); Serial.print(allowReverse);
  Serial.print(F(" bench=")); Serial.print(bench);
  Serial.print(F(" arng=16 grng=2000"));
  Serial.print(F(" cal=")); Serial.print(cfg.calValid);
  Serial.print(F(" zero=")); Serial.println(cfg.zeroValid);
}

void printCal() {
  Serial.print(F("CALVAL "));
  Serial.print(cfg.calValid);
  for (uint8_t i = 0; i < 3; ++i) { Serial.print(' '); printFix(cfg.gyroBias[i], 4); }
  for (uint8_t i = 0; i < 3; ++i) { Serial.print(' '); printFix(cfg.accOff[i], 4); }
  Serial.print(F(" g0"));
  for (uint8_t i = 0; i < 3; ++i) { Serial.print(' '); printFix(cfg.g0[i], 4); }
  Serial.println();
}

// 고정소수점 출력 (Print::printFloat 대신 — 플래시 절약, 출력 형식 동일)
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
  Serial.print(F(" zp=")); Serial.println(pid.nzp);
}

void err(const __FlashStringHelper *code) {
  Serial.print(F("ERR ")); Serial.println(code);
}

// 공백으로 구분된 정수 인자 파싱 (strtol 대신 직접 구현 — 플래시 절약)
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

bool busy() { return prof.type != P_NONE || cal.active; }
bool inRangeU(long u) { return u >= -255 && u <= 255; }

void startProfile(const __FlashStringHelper *name) {
  prof.n = 0;
  uManual = 0;
  divCount = 0;
  Serial.print(F("EVT RUN ")); Serial.print(name);
  Serial.print(F(" total=")); Serial.print(prof.total);
  Serial.print(F(" ts_ms=")); Serial.println(5 * streamDiv);
}

void runCommand(char *args) {
  if (busy()) { err(F("BUSY")); return; }
  if (!imuOk && !bench) { err(F("IMU_NOT_READY")); return; }
  long v[8];
  char *sp = strchr(args, ' ');
  char *rest = sp ? sp + 1 : args + strlen(args);
  if (sp) *sp = '\0';
  uint8_t n = parseInts(rest, v, 8);
  const uint32_t MAX_SAMPLES = 120000UL;   // 10 ms 기준 20분

  if (strcmp(args, "STEP") == 0) {           // RUN STEP u0 u1 pre_ms step_ms post_ms
    if (n != 5 || !inRangeU(v[0]) || !inRangeU(v[1]) || v[2] < 0 || v[3] <= 0 || v[4] < 0) { err(F("BAD_ARGS")); return; }
    prof = Profile();
    prof.type = P_STEP; prof.u0 = v[0]; prof.u1 = v[1];
    prof.a = msToSamples(v[2]); prof.b = msToSamples(v[3]); prof.c = msToSamples(v[4]);
    prof.total = prof.a + prof.b + prof.c;
  } else if (strcmp(args, "STAIR") == 0) {   // RUN STAIR u_start u_end u_inc dwell_ms
    if (n != 4 || !inRangeU(v[0]) || !inRangeU(v[1]) || v[2] <= 0 || v[2] > 255 || v[3] <= 0) { err(F("BAD_ARGS")); return; }
    long steps = labs(v[1] - v[0]) / v[2] + 1;
    if (steps > 250) { err(F("TOO_MANY_STEPS")); return; }
    prof = Profile();
    prof.type = P_STAIR; prof.u0 = v[0]; prof.inc = (v[1] >= v[0]) ? v[2] : -v[2];
    prof.a = msToSamples(v[3]);
    if (prof.a == 0) prof.a = 1;
    prof.total = prof.a * steps;
  } else if (strcmp(args, "CHIRP") == 0) {   // RUN CHIRP u0 amp f0_mHz f1_mHz pre_ms dur_ms log(0/1)
    if (n != 7 || !inRangeU(v[0]) || v[1] < 0 || v[1] > 255 || v[2] <= 0 || v[3] <= 0 ||
        v[3] > 50000 || v[4] < 0 || v[5] <= 0) { err(F("BAD_ARGS")); return; }
    prof = Profile();
    prof.type = P_CHIRP; prof.u0 = v[0]; prof.amp = v[1];
    prof.a = msToSamples(v[4]); prof.b = msToSamples(v[5]);
    prof.total = prof.a + prof.b;
    prof.logSweep = v[6] != 0;
    float f0 = v[2] * 0.001f, f1 = v[3] * 0.001f;
    prof.f = f0;
    prof.df = prof.b ? (f1 - f0) / prof.b : 0;
    prof.kf = 1;
    if (prof.b) {                 // kf = (f1/f0)^(1/b), ln x ≈ 4096·(ε − ε²/2), ε = x^(1/4096) − 1  (pow 대신 — 플래시 절약)
      float x = f1 / f0;
      for (uint8_t i = 0; i < 12; ++i) x = sqrt(x);
      float e = x - 1, y = 4096.0f * (e - 0.5f * e * e) / prof.b;
      prof.kf = 1 + y + 0.5f * y * y;
    }
  } else if (strcmp(args, "PRBS") == 0) {    // RUN PRBS u0 amp bit_ms pre_ms dur_ms
    if (n != 5 || !inRangeU(v[0]) || v[1] < 0 || v[1] > 255 || v[2] <= 0 || v[3] < 0 || v[4] <= 0) { err(F("BAD_ARGS")); return; }
    prof = Profile();
    prof.type = P_PRBS; prof.u0 = v[0]; prof.amp = v[1];
    prof.a = msToSamples(v[3]);
    prof.b = msToSamples(v[2]);
    if (prof.b == 0) prof.b = 1;
    prof.total = prof.a + msToSamples(v[4]);
  } else if (strcmp(args, "CLSTEP") == 0) {  // RUN CLSTEP r0_cdeg r1_cdeg pre_ms step_ms post_ms
    if (n != 5 || labs(v[0]) > 17000 || labs(v[1]) > 17000 || v[2] < 0 || v[3] <= 0 || v[4] < 0) { err(F("BAD_ARGS")); return; }
    prof = Profile();
    prof.type = P_CLSTEP; prof.u0 = v[0]; prof.u1 = v[1];
    prof.a = msToSamples(v[2]); prof.b = msToSamples(v[3]); prof.c = msToSamples(v[4]);
    prof.total = prof.a + prof.b + prof.c;
  } else if (strcmp(args, "RELAY") == 0) {   // RUN RELAY r_cdeg d_pwm hyst_cdeg pre_ms dur_ms
    if (n != 5 || labs(v[0]) > 17000 || v[1] <= 0 || v[1] > 255 || v[2] < 0 || v[2] > 1000 || v[3] < 0 || v[4] <= 0) { err(F("BAD_ARGS")); return; }
    prof = Profile();
    prof.type = P_RELAY; prof.u0 = v[0]; prof.amp = v[1]; prof.inc = v[2];
    prof.a = msToSamples(v[3]);
    prof.total = prof.a + msToSamples(v[4]);
    prof.f = 0; prof.df = 0;
  } else { err(F("UNKNOWN_PROFILE")); return; }
  clHold = false;
  pidReset();

  if (prof.total == 0 || prof.total > MAX_SAMPLES) { prof.type = P_NONE; err(F("BAD_DURATION")); return; }
  switch (prof.type) {
    case P_STEP: startProfile(F("STEP")); break;
    case P_CLSTEP: startProfile(F("CLSTEP")); break;
    case P_RELAY: startProfile(F("RELAY")); break;
    case P_STAIR: startProfile(F("STAIR")); break;
    case P_CHIRP: startProfile(F("CHIRP")); break;
    default: startProfile(F("PRBS")); break;
  }
}

void command(char *line) {
  lastHeartbeat = millis();
  long v[4];
  if (strcmp(line, "PING") == 0) {
    return;
  } else if (strcmp(line, "HELLO") == 0) {
    stopAll();
    Serial.print(F("READY ")); Serial.println(F(FW_ID));
    Serial.println(imuOk ? F("IMU READY") : F("IMU ERROR NOT_READY"));
    printConfig();
    printCal();
  } else if (strcmp(line, "STOP") == 0) {
    bool calWas = cal.active;
    cal.active = false;
    stopAll();
    if (calWas) Serial.println(F("CAL FAIL CANCELLED"));
    event(F("STOP"));
  } else if (strcmp(line, "GET") == 0) {
    printConfig();
    printCal();
    printPid();
  } else if (strncmp(line, "PID ", 4) == 0) {      // PID kp_milli ki_milli kd_milli ff_pwm (실행 중 변경 가능)
    long w[4];
    if (parseInts(line + 4, w, 4) != 4 || w[0] < 0 || w[1] < 0 || w[2] < 0 || w[0] > 200000 || w[1] > 500000 ||
        w[2] > 50000 || w[3] < 0 || w[3] > 255) { err(F("BAD_ARGS")); return; }
    pid.kp = w[0] * 0.001f; pid.ki = w[1] * 0.001f; pid.kd = w[2] * 0.001f; pid.ff = w[3];
    printPid();
  } else if (strncmp(line, "NOTCH ", 6) == 0) {    // NOTCH f_centiHz zz_milli zp_milli  /  NOTCH 0
    long w[3];
    uint8_t k = parseInts(line + 6, w, 3);
    if (k == 1 && w[0] == 0) pid.nfc = 0;
    else if (k == 3 && w[0] >= 10 && w[0] <= 2000 && w[1] >= 0 && w[1] <= 2000 && w[2] > 0 && w[2] <= 2000) {
      pid.nfc = w[0]; pid.nzz = w[1]; pid.nzp = w[2];
    } else { err(F("BAD_ARGS")); return; }
    nz1 = nz2 = 0;
    notchCoeffs();
    printPid();
  } else if (strncmp(line, "REF ", 4) == 0) {      // REF r_cdeg : 폐루프로 기준 유지 (STOP 으로 해제)
    if (busy()) { err(F("BUSY")); return; }
    if (!imuOk && !bench) { err(F("IMU_NOT_READY")); return; }
    if (parseInts(line + 4, v, 1) != 1 || labs(v[0]) > 17000) { err(F("BAD_ARGS")); return; }
    if (!clHold) { pidReset(); uManual = 0; clHold = true; }
    refDeg = v[0] * 0.01f;
    Serial.print(F("EVT REF ")); Serial.println(v[0]);
  } else if (strncmp(line, "U ", 2) == 0) {
    if (busy()) { err(F("BUSY")); return; }
    if (!imuOk && !bench) { err(F("IMU_NOT_READY")); return; }
    if (parseInts(line + 2, v, 1) != 1 || !inRangeU(v[0])) { err(F("BAD_ARGS")); return; }
    clHold = false;
    uManual = v[0];
  } else if (strncmp(line, "LIM ", 4) == 0) {
    uint8_t n = parseInts(line + 4, v, 3);
    if ((n != 2 && n != 3) || v[0] < 0 || v[0] > 255 || v[1] < 500 || v[1] > 18000) { err(F("BAD_ARGS")); return; }
    uMaxPwm = v[0]; angleLimitCdeg = v[1];
    if (n == 3) allowReverse = v[2] != 0;
    printConfig();
  } else if (strncmp(line, "STREAM ", 7) == 0) {
    if (busy()) { err(F("BUSY")); return; }
    if (parseInts(line + 7, v, 1) != 1 || v[0] < 1 || v[0] > 20) { err(F("BAD_ARGS")); return; }
    streamDiv = v[0];
    notchCoeffs();                 // 노치 계수는 샘플 주기에 의존
    printConfig();
  } else if (strncmp(line, "AXIS ", 5) == 0) {
    char a = line[5], s = line[6] == ' ' ? line[7] : 0;
    if (a < 'X' || a > 'Z' || (s != '+' && s != '-') || line[8] != '\0') { err(F("BAD_ARGS")); return; }
    if (uOut != 0 || busy()) { err(F("BUSY")); return; }
    cfg.axis = a - 'X';
    cfg.sign = s == '+' ? 1 : -1;
    saveConfig();
    printConfig();
  } else if (strncmp(line, "BETA ", 5) == 0) {
    if (parseInts(line + 5, v, 1) != 1 || v[0] < 1 || v[0] > 1000) { err(F("BAD_ARGS")); return; }
    cfg.betaMilli = v[0];
    ahrs.beta = cfg.betaMilli * 0.001f;
    saveConfig();
    printConfig();
  } else if (strncmp(line, "DLPF ", 5) == 0) {
    if (parseInts(line + 5, v, 1) != 1 || v[0] < 0 || v[0] > 6) { err(F("BAD_ARGS")); return; }
    cfg.dlpf = v[0];
    if (imuOk && !writeReg(0x1A, cfg.dlpf)) i2cErrors++;
    saveConfig();
    printConfig();
  } else if (strcmp(line, "ZERO") == 0) {
    if (!imuOk) { err(F("IMU_NOT_READY")); return; }
    if (uOut != 0 || busy()) { err(F("BUSY")); return; }
    float g[3];
    ahrs.gravity(g);
    if (perpNorm(g, cfg.axis) < 0.5f) { err(F("AXIS_PARALLEL_TO_GRAVITY")); return; }
    for (uint8_t i = 0; i < 3; ++i) cfg.g0[i] = g[i];
    cfg.zeroValid = 1;
    saveConfig();
    Serial.println(F("ZERO OK"));
    printConfig();
    printCal();
  } else if (strcmp(line, "CAL") == 0 || strcmp(line, "CAL FLAT") == 0) {
    if (!imuOk) { err(F("IMU_NOT_READY")); return; }
    if (uOut != 0 || busy()) { err(F("BUSY")); return; }
    cal = CalState();
    memset(cal.mean, 0, sizeof(cal.mean));
    memset(cal.m2, 0, sizeof(cal.m2));
    cal.flat = line[3] == ' ';
    cal.active = true;
    Serial.print(F("CAL START mode=")); Serial.print(cal.flat ? F("FLAT") : F("STILL"));
    Serial.print(F(" n=")); Serial.println(cal.target);
  } else if (strncmp(line, "ACCOFF ", 7) == 0 && strcmp(line + 7, "DEFAULT") != 0) {   // 가속도 오프셋 직접 지정 (mg)
    if (busy()) { err(F("BUSY")); return; }
    if (parseInts(line + 7, v, 3) != 3 || labs(v[0]) > 500 || labs(v[1]) > 500 || labs(v[2]) > 500) { err(F("BAD_ARGS")); return; }
    for (uint8_t i = 0; i < 3; ++i) cfg.accOff[i] = v[i] * 0.001f;
    saveConfig();
    seedPending = true;
    printCal();
  } else if (strcmp(line, "ACCOFF DEFAULT") == 0) {
    if (busy()) { err(F("BUSY")); return; }
    for (uint8_t i = 0; i < 3; ++i) cfg.accOff[i] = DEFAULT_ACC_OFF[i];
    saveConfig();
    seedPending = true;
    printCal();
  } else if (strcmp(line, "CAL CLEAR") == 0) {     // 자이로 바이어스만 해제 (가속도 오프셋 유지)
    if (busy()) { err(F("BUSY")); return; }
    for (uint8_t i = 0; i < 3; ++i) cfg.gyroBias[i] = 0;
    cfg.calValid = 0;
    saveConfig();
    seedPending = true;
    printConfig();
    printCal();
  } else if (strncmp(line, "BENCH ", 6) == 0) {
    if (parseInts(line + 6, v, 1) != 1 || v[0] < 0 || v[0] > 1) { err(F("BAD_ARGS")); return; }
    if (uOut != 0 || busy()) { err(F("BUSY")); return; }
    stopAll();
    bench = v[0] != 0;
    applyMotor(0);
    if (bench && !imuOk) { nextTickUs = micros(); lastTickUs = nextTickUs; }
    printConfig();
  } else if (strncmp(line, "RUN ", 4) == 0) {
    runCommand(line + 4);
  } else if (strcmp(line, "CFG BACKUP") == 0) {    // 현재 설정을 EEPROM 예비 영역에 복사
    if (busy()) { err(F("BUSY")); return; }
    saveConfig();
    EEPROM.put(CFG_BACKUP_ADDR, cfg);
    Serial.println(F("CFG BACKUP OK"));
  } else if (strcmp(line, "CFG RESTORE") == 0) {   // 예비 영역의 설정으로 되돌림
    if (uOut != 0 || busy()) { err(F("BUSY")); return; }
    Config b;
    EEPROM.get(CFG_BACKUP_ADDR, b);
    if (b.magic != CFG_MAGIC || b.version != CFG_VERSION ||
        b.crc != crc8(reinterpret_cast<const uint8_t *>(&b), sizeof(b) - 1)) { err(F("NO_BACKUP")); return; }
    cfg = b;
    saveConfig();
    ahrs.beta = cfg.betaMilli * 0.001f;
    if (imuOk) writeReg(0x1A, cfg.dlpf);
    seedPending = true;
    Serial.println(F("CFG RESTORE OK"));
    printConfig();
    printCal();
  } else if (strcmp(line, "FACTORY") == 0) {
    if (uOut != 0 || busy()) { err(F("BUSY")); return; }
    defaultConfig();
    saveConfig();
    ahrs.beta = cfg.betaMilli * 0.001f;
    if (imuOk) writeReg(0x1A, cfg.dlpf);
    seedPending = true;
    printConfig();
    printCal();
  } else {
    err(F("UNKNOWN_COMMAND"));
  }
}

char input[72];
uint8_t inputLen = 0;
bool discarding = false;
unsigned long lastByteAt = 0;

void readCommands() {
  if ((inputLen || discarding) && millis() - lastByteAt > 250) {   // 잘린 명령 폐기
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
  delay(200);
  Serial.print(F("READY ")); Serial.println(F(FW_ID));
  if (cfgMigrated) Serial.println(F("NOTE CFG_MIGRATED v3->v4: dlpf=4 beta=20 (axis/cal/zero kept)"));
  imuOk = imuInit();
  lastImuRetry = millis();
  printConfig();
  printCal();
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

  // 통신 두절 시 정지 (센서 상태와 무관하게 먼저 검사)
  if ((uOut != 0 || prof.type != P_NONE || clHold) && millis() - lastHeartbeat > LINK_TIMEOUT_MS) {
    abortRun(F("LINK_TIMEOUT"));
  }

  if (!imuOk && !bench) {
    if (uOut != 0) applyMotor(0);
  }
  // 센서 재연결 시도 (벤치 시험 중에는 타이밍을 흔들지 않도록 쉼)
  if (!imuOk && !(bench && (prof.type != P_NONE || uOut != 0)) && millis() - lastImuRetry >= 1000) {
    lastImuRetry = millis();
    imuOk = imuInit();
    if (imuOk) { seedPending = true; nextTickUs = micros(); lastTickUs = nextTickUs; }
  }
  if (imuOk || bench) {
    unsigned long now = micros();
    if ((long)(now - nextTickUs) >= 0) {
      nextTickUs += TICK_US;
      if ((long)(now - nextTickUs) >= 0) {   // 한 주기 이상 밀림
        overruns++;
        nextTickUs = now + TICK_US;
      }
      float dt = (now - lastTickUs) * 1e-6f;
      lastTickUs = now;
      if (dt <= 0 || dt > 0.05f) dt = TICK_S;
      tick(dt);
      unsigned long tookL = micros() - now;
      uint16_t took = tookL > 65535UL ? 65535 : (uint16_t)tookL;
      if (took > maxTickUs) maxTickUs = took;
    }
  }

  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    sendStatus();
  }
}
