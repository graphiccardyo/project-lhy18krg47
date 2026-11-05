#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Week 1 browser demo: L298N motor control and MPU6050 attitude on one Uno.
// Motor protocol follows uno_l298n; attitude follows uno_mpu6050_attitude.
// New calibration is not performed here; saved offsets arrive through RESTORE.
const uint8_t ENA_PIN = 5;
const uint8_t IN1_PIN = 7;
const uint8_t IN2_PIN = 8;
const unsigned long LINK_TIMEOUT_MS = 1500;
const unsigned long SAMPLE_INTERVAL_MS = 40;

MPU6050 mpu(Wire);
Adafruit_Madgwick fusion;
float latest[6];
float latestDt = 0;
unsigned long fusionMicros = 0;
bool seedAttitude = true;
bool configured = false;
uint8_t sensorAddress = 0;
uint16_t calibrationId = 0;
unsigned long lastSample = 0;
unsigned long lastRetry = 0;

char input[96];
uint8_t inputLength = 0;
bool discardLine = false;
unsigned long inputLastByteAt = 0;
uint8_t percent = 0;
char direction = 'F';
unsigned long lastHeartbeat = 0;

uint8_t pwmValue(uint8_t pct) {
  return (static_cast<uint16_t>(pct) * 255 + 50) / 100;
}

void report(const __FlashStringHelper *reason) {
  Serial.print(F("STATE pct=")); Serial.print(percent);
  Serial.print(F(" pwm=")); Serial.print(pwmValue(percent));
  Serial.print(F(" dir=")); Serial.print(direction);
  Serial.print(F(" d7=")); Serial.print(digitalRead(IN1_PIN));
  Serial.print(F(" d8=")); Serial.print(digitalRead(IN2_PIN));
  Serial.print(F(" reason=")); Serial.println(reason);
}

void stopMotor() {
  analogWrite(ENA_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  percent = 0;
}

void reject(const __FlashStringHelper *error) {
  stopMotor();
  Serial.print(F("ERR ")); Serial.println(error);
  report(F("ERROR"));
}

void resetAttitude() {
  fusion.setQuaternion(1, 0, 0, 0);
  fusionMicros = 0;
  seedAttitude = true;
}

void beginBus() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);  // use module pull-ups, not Uno's internal 5 V pull-ups
}

bool readReg(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) return false;
  if (Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1 || Wire.getWireTimeoutFlag()) return false;
  value = Wire.read();
  return true;
}

bool checkReg(uint8_t reg, uint8_t expected) {
  uint8_t value = 0;
  return readReg(sensorAddress, reg, value) && value == expected;
}

bool configureSensor() {
  beginBus();
  sensorAddress = 0;
  const uint8_t addresses[] = {0x68, 0x69};
  for (uint8_t address : addresses) {
    uint8_t id = 0;
    if (readReg(address, 0x75, id) && id == 0x68) { sensorAddress = address; break; }
  }
  if (!sensorAddress) { Serial.println(F("IMU ERROR NOT_FOUND")); return false; }
  mpu.setAddress(sensorAddress);
  mpu.setAccOffsets(0, 0, 0);
  mpu.setGyroOffsets(0, 0, 0);
  calibrationId = 0;
  if (mpu.begin(0, 0) != 0 || mpu.writeData(0x6C, 0x00) != 0 ||
      mpu.writeData(0x1A, 0x03) != 0 || mpu.writeData(0x19, 0x00) != 0) {
    Serial.println(F("IMU ERROR CONFIG_WRITE")); return false;
  }
  if (!checkReg(0x6B, 0x01) || !checkReg(0x6C, 0x00) || !checkReg(0x1A, 0x03) ||
      !checkReg(0x19, 0x00) || !checkReg(0x1B, 0x00) || !checkReg(0x1C, 0x00)) {
    Serial.println(F("IMU ERROR CONFIG_VERIFY")); return false;
  }
  delay(400);
  fusion.begin(100);
  resetAttitude();
  Serial.println(F("IMU READY"));
  return true;
}

bool restoreOffsets(const char *line) {
  const char *cursor = line;
  char *end = nullptr;
  unsigned long id = strtoul(cursor, &end, 10);
  if (end == cursor || id == 0 || id > 65535) return false;
  cursor = end;
  float offsets[6];
  for (uint8_t i = 0; i < 6; ++i) {
    if (*cursor != ' ') return false;
    ++cursor;
    offsets[i] = strtod(cursor, &end);
    if (end == cursor || !isfinite(offsets[i]) || fabs(offsets[i]) > (i < 3 ? 0.5f : 20.0f)) return false;
    cursor = end;
  }
  if (*cursor != '\0') return false;
  mpu.setAccOffsets(offsets[0], offsets[1], offsets[2]);
  mpu.setGyroOffsets(offsets[3], offsets[4], offsets[5]);
  calibrationId = static_cast<uint16_t>(id);
  resetAttitude();
  return true;
}

void command(char *line) {
  if (strcmp(line, "HELLO") == 0) {
    stopMotor();
    lastHeartbeat = millis();
    Serial.println(F("READY UNO_DEMO 1.0"));
    Serial.println(configured ? F("IMU READY") : F("IMU ERROR NOT_READY"));
    report(F("CONNECTED"));
  } else if (strcmp(line, "PING") == 0) {
    lastHeartbeat = millis();
    Serial.println(F("PONG"));
  } else if (strcmp(line, "STOP") == 0) {
    stopMotor();
    report(F("STOP"));
  } else if (strcmp(line, "STATUS") == 0) {
    report(F("STATUS"));
  } else if (strncmp(line, "RESTORE ", 8) == 0) {
    if (configured && restoreOffsets(line + 8)) {
      Serial.print(F("CAL RESTORED id=")); Serial.println(calibrationId);
    } else {
      Serial.println(F("CAL REJECTED"));
    }
  } else if (strncmp(line, "SET ", 4) == 0) {
    char *number = line + 4;
    char *end;
    if (*number < '0' || *number > '9') {
      reject(F("BAD_PERCENT")); return;
    }
    long value = strtol(number, &end, 10);
    if (value < 0 || value > 100 || *end != ' ' ||
        (end[1] != 'F' && end[1] != 'R') || end[2] != '\0') {
      reject(F("BAD_SET")); return;
    }
    char requestedDirection = end[1];
    if (percent != 0 && requestedDirection != direction) {
      reject(F("STOP_BEFORE_REVERSE")); return;
    }
    if (requestedDirection != direction) {
      stopMotor();
      delay(3);
    }
    direction = requestedDirection;
    percent = static_cast<uint8_t>(value);
    if (percent == 0) {
      stopMotor();
    } else {
      digitalWrite(IN1_PIN, direction == 'F' ? HIGH : LOW);
      digitalWrite(IN2_PIN, direction == 'F' ? LOW : HIGH);
      analogWrite(ENA_PIN, pwmValue(percent));
    }
    lastHeartbeat = millis();
    report(F("SET"));
  } else {
    reject(F("UNKNOWN_COMMAND"));
  }
}

void readCommands() {
  // Never join a new command onto an old line whose newline was lost.
  if ((inputLength || discardLine) && millis() - inputLastByteAt > 250) {
    inputLength = 0;
    discardLine = false;
    reject(F("COMMAND_TIMEOUT"));
  }
  // Bound each batch so even a continuous malformed stream cannot starve timeout.
  for (uint8_t budget = 0; budget < 64 && Serial.available(); ++budget) {
    char c = Serial.read();
    inputLastByteAt = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!discardLine && inputLength) {
        input[inputLength] = '\0';
        command(input);
      }
      inputLength = 0;
      discardLine = false;
    } else if (!discardLine) {
      if (c < 32 || c > 126 || inputLength >= sizeof(input) - 1) {
        discardLine = true;
        inputLength = 0;
        reject(F("INVALID_LINE"));
      } else {
        input[inputLength++] = c;
      }
    }
  }
}

bool updateAttitude(unsigned long now) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(sensorAddress);
  if (Wire.endTransmission() != 0) return false;
  mpu.fetchData();
  if (Wire.getWireTimeoutFlag()) return false;
  latest[0] = mpu.getAccX(); latest[1] = mpu.getAccY(); latest[2] = mpu.getAccZ();
  latest[3] = mpu.getGyroX(); latest[4] = mpu.getGyroY(); latest[5] = mpu.getGyroZ();
  for (uint8_t i = 0; i < 6; ++i) if (!isfinite(latest[i])) return false;
  float dt = fusionMicros ? (now - fusionMicros) * 0.000001f : 0.01f;
  fusionMicros = now;
  if (dt > 0.1f || dt < 0.002f) { seedAttitude = true; dt = 0.01f; }
  if (seedAttitude) {
    float roll = atan2(latest[1], latest[2]);
    float pitch = atan2(-latest[0], sqrt(latest[1] * latest[1] + latest[2] * latest[2]));
    float cr = cos(roll * 0.5f), sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f), sp = sin(pitch * 0.5f);
    fusion.setQuaternion(cr * cp, sr * cp, cr * sp, -sr * sp);
    seedAttitude = false;
  }
  // Adafruit updateIMU takes gyro in degrees/sec and accepts actual dt in seconds.
  fusion.updateIMU(latest[3], latest[4], latest[5], latest[0], latest[1], latest[2], dt);
  latestDt = dt;
  return true;
}

// S,ms,ax,ay,az,gx,gy,gz,qw,qx,qy,qz,dt_ms,cal_id
void streamSample() {
  Serial.print(F("S,")); Serial.print(millis());
  for (uint8_t i = 0; i < 3; ++i) { Serial.print(','); Serial.print(latest[i], 3); }
  for (uint8_t i = 3; i < 6; ++i) { Serial.print(','); Serial.print(latest[i], 2); }
  float w, x, y, z; fusion.getQuaternion(&w, &x, &y, &z);
  float norm = sqrt(w*w + x*x + y*y + z*z);
  if (!isfinite(norm) || norm < 0.5f) { resetAttitude(); w = 1; x = y = z = 0; }
  else { w /= norm; x /= norm; y /= norm; z /= norm; }
  const float quaternion[] = {w, x, y, z};
  for (float value : quaternion) { Serial.print(','); Serial.print(value, 4); }
  Serial.print(','); Serial.print(latestDt * 1000, 1);
  Serial.print(','); Serial.println(calibrationId);
}

void setup() {
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  stopMotor();
  Serial.begin(115200);
  delay(300);
  lastHeartbeat = millis();
  Serial.println(F("READY UNO_DEMO 1.0"));
  report(F("BOOT"));
  configured = configureSensor();
  lastRetry = millis();
}

void loop() {
  readCommands();
  // The motor watchdog runs first so a lost sensor can never hold the motor on.
  if (percent != 0 && millis() - lastHeartbeat > LINK_TIMEOUT_MS) {
    stopMotor();
    report(F("TIMEOUT"));
  }
  if (!configured) {
    if (millis() - lastRetry >= 3000) { configured = configureSensor(); lastRetry = millis(); }
    return;
  }
  unsigned long now = micros();
  if (!fusionMicros || now - fusionMicros >= 10000) {
    if (!updateAttitude(now)) {
      configured = false;
      lastRetry = millis();
      Serial.println(F("IMU ERROR SAMPLE_READ"));
    } else if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
      lastSample = millis();
      streamSample();
    }
  }
}
