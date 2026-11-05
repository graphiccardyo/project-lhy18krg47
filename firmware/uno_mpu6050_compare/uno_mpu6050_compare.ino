#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

struct Statistics { float mean[6]; float stddev[6]; };
bool collectStatistics(Statistics &stats);
bool flatAndStill(const Statistics &stats);

// MPU6050_light 1.2.1. Manual single-position OFFSET calibration only.
// No automatic calibration or EEPROM writes. Motor outputs always LOW.
MPU6050 mpu(Wire);
bool configured = false;
uint8_t sensorAddress = 0;
uint16_t calibrationId = 0;
unsigned long lastSample = 0;
unsigned long lastRetry = 0;
char input[24];
uint8_t inputLength = 0;
bool discardLine = false;

void holdMotorOff() {
  const uint8_t pins[] = {5, 7, 8};
  for (uint8_t pin : pins) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }
}

void beginBus() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);  // use module pull-ups, not Uno's internal 5 V pull-ups
}

void hexByte(uint8_t value) {
  Serial.print(F("0x"));
  if (value < 16) Serial.print('0');
  Serial.print(value, HEX);
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

void getOffsets(float *offsets) {
  offsets[0] = mpu.getAccXoffset(); offsets[1] = mpu.getAccYoffset(); offsets[2] = mpu.getAccZoffset();
  offsets[3] = mpu.getGyroXoffset(); offsets[4] = mpu.getGyroYoffset(); offsets[5] = mpu.getGyroZoffset();
}

void setOffsets(const float *offsets) {
  mpu.setAccOffsets(offsets[0], offsets[1], offsets[2]);
  mpu.setGyroOffsets(offsets[3], offsets[4], offsets[5]);
}

void clearOffsets() {
  mpu.setAccOffsets(0, 0, 0);
  mpu.setGyroOffsets(0, 0, 0);
  calibrationId = 0;
}

void reportOffsets() {
  float offsets[6]; getOffsets(offsets);
  Serial.print(F("# OFFSETS cal_id=")); Serial.print(calibrationId);
  for (uint8_t i = 0; i < 6; ++i) { Serial.print(','); Serial.print(offsets[i], 6); }
  Serial.println();
}

bool configureSensor() {
  beginBus();
  sensorAddress = 0;
  const uint8_t addresses[] = {0x68, 0x69};
  for (uint8_t address : addresses) {
    uint8_t id = 0;
    if (readReg(address, 0x75, id)) {
      Serial.print(F("# WHO_AM_I address=")); hexByte(address);
      Serial.print(F(" id=")); hexByte(id); Serial.println();
      if (id == 0x68) { sensorAddress = address; break; }
    }
  }
  if (!sensorAddress) { Serial.println(F("# ERROR MPU6050_NOT_FOUND")); return false; }
  mpu.setAddress(sensorAddress);
  clearOffsets();
  if (mpu.begin(0, 0) != 0 || mpu.writeData(0x6C, 0x00) != 0 ||
      mpu.writeData(0x1A, 0x03) != 0 || mpu.writeData(0x19, 0x00) != 0) {
    Serial.println(F("# ERROR CONFIG_WRITE")); return false;
  }
  if (!checkReg(0x6B, 0x01) || !checkReg(0x6C, 0x00) || !checkReg(0x1A, 0x03) ||
      !checkReg(0x19, 0x00) || !checkReg(0x1B, 0x00) || !checkReg(0x1C, 0x00)) {
    Serial.println(F("# ERROR CONFIG_VERIFY")); return false;
  }
  delay(400);
  Serial.print(F("# READY UNO_MPU6050 2.0 address=")); hexByte(sensorAddress);
  Serial.println(F(" library=MPU6050_light-1.2.1 stream_hz=20 MOTOR_OFF"));
  Serial.println(F("ms,cal_id,ax_before_g,ay_before_g,az_before_g,gx_before_dps,gy_before_dps,gz_before_dps,ax_after_g,ay_after_g,az_after_g,gx_after_dps,gy_after_dps,gz_after_dps,temp_c"));
  Serial.println(F("# CAL NONE manual_CAL_only flat_Z_up_still_required"));
  reportOffsets();
  return true;
}

// Both before/after values come from ONE library fetch of the same 14-byte frame.
// Library getters already subtract offsets; adding those offsets recovers before.
bool fetchFrame(float *before, float *after) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(sensorAddress);
  if (Wire.endTransmission() != 0) return false;
  mpu.fetchData();
  if (Wire.getWireTimeoutFlag()) return false;
  after[0] = mpu.getAccX(); after[1] = mpu.getAccY(); after[2] = mpu.getAccZ();
  after[3] = mpu.getGyroX(); after[4] = mpu.getGyroY(); after[5] = mpu.getGyroZ();
  float offsets[6]; getOffsets(offsets);
  for (uint8_t i = 0; i < 6; ++i) {
    before[i] = after[i] + offsets[i];
    if (!isfinite(before[i]) || !isfinite(after[i])) return false;
  }
  return true;
}

bool collectStatistics(Statistics &stats) {
  const uint8_t count = 60;
  float sums[6] = {}, squares[6] = {};
  for (uint8_t sample = 0; sample < count; ++sample) {
    float before[6], after[6];
    if (!fetchFrame(before, after)) return false;
    for (uint8_t i = 0; i < 6; ++i) { sums[i] += before[i]; squares[i] += before[i] * before[i]; }
    delay(10);
  }
  for (uint8_t i = 0; i < 6; ++i) {
    stats.mean[i] = sums[i] / count;
    float variance = squares[i] / count - stats.mean[i] * stats.mean[i];
    stats.stddev[i] = sqrt(max(0.0f, variance));
  }
  return true;
}

bool flatAndStill(const Statistics &stats) {
  if (fabs(stats.mean[0]) > 0.15f || fabs(stats.mean[1]) > 0.15f ||
      stats.mean[2] < 0.7f || stats.mean[2] > 1.35f) return false;
  for (uint8_t i = 0; i < 6; ++i) {
    if (stats.stddev[i] > (i < 3 ? 0.015f : 0.8f)) return false;
    if (i >= 3 && fabs(stats.mean[i]) > 8.0f) return false;
  }
  return true;
}

void calibrate() {
  Serial.println(F("# CAL BEGIN checking_flat_Z_up_and_still"));
  Statistics pre, post;
  if (!collectStatistics(pre) || !flatAndStill(pre)) {
    Serial.println(F("# CAL REJECTED place_flat_Z_up_and_keep_still"));
    reportOffsets(); return;
  }
  float previous[6]; getOffsets(previous);
  Wire.clearWireTimeoutFlag();
  // Upstream library's 500-sample accel+gyro OFFSET calibration.
  // Its Z target is +1 g, so gravity is retained rather than zeroed.
  mpu.calcOffsets(true, true);
  bool valid = !Wire.getWireTimeoutFlag() && collectStatistics(post) && flatAndStill(post);
  float offsets[6]; getOffsets(offsets);
  for (uint8_t i = 0; i < 6 && valid; ++i) {
    float reference = i == 2 ? 1.0f : 0.0f;
    float residual = post.mean[i] - offsets[i] - reference;
    if (!isfinite(offsets[i]) || fabs(residual) > (i < 3 ? 0.025f : 0.25f) ||
        fabs(post.mean[i] - pre.mean[i]) > (i < 3 ? 0.03f : 1.0f)) valid = false;
  }
  if (!valid) {
    setOffsets(previous);
    Serial.println(F("# CAL REJECTED movement_or_read_error_previous_offsets_kept"));
    reportOffsets(); return;
  }
  if (++calibrationId == 0) calibrationId = 1;
  Serial.println(F("# CAL DONE offset_only_target_acc_0_0_1g_gyro_0_0_0"));
  reportOffsets();
}

void handleCommand(const char *line) {
  if (strcmp(line, "STATUS") == 0) { reportOffsets(); return; }
  if (strcmp(line, "CLEAR") == 0) {
    clearOffsets(); Serial.println(F("# CAL CLEARED")); reportOffsets(); return;
  }
  if (strcmp(line, "CAL") == 0) {
    if (configured) calibrate(); else Serial.println(F("# CAL REJECTED sensor_not_ready"));
    return;
  }
  Serial.println(F("# ERROR UNKNOWN_COMMAND"));
}

void readCommands() {
  for (uint8_t budget = 0; budget < 64 && Serial.available(); ++budget) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!discardLine && inputLength) { input[inputLength] = '\0'; handleCommand(input); }
      inputLength = 0; discardLine = false;
    } else if (!discardLine) {
      if (c < 32 || c > 126 || inputLength >= sizeof(input) - 1) { discardLine = true; inputLength = 0; }
      else input[inputLength++] = c;
    }
  }
}

void streamSample() {
  float before[6], after[6];
  if (!fetchFrame(before, after)) {
    configured = false; lastRetry = millis();
    Serial.println(F("# ERROR SAMPLE_READ calibration_will_reset_on_reconnect")); return;
  }
  Serial.print(millis()); Serial.print(','); Serial.print(calibrationId);
  for (uint8_t i = 0; i < 6; ++i) { Serial.print(','); Serial.print(before[i], 5); }
  for (uint8_t i = 0; i < 6; ++i) { Serial.print(','); Serial.print(after[i], 5); }
  Serial.print(','); Serial.println(mpu.getTemp(), 2);
}

void setup() {
  holdMotorOff();
  Serial.begin(115200);
  delay(300);
  Serial.println(F("# UNO_MPU6050 BOOT protocol=2 manual_calibration_only"));
  configured = configureSensor();
  lastRetry = millis();
}

void loop() {
  readCommands();
  if (!configured) {
    if (millis() - lastRetry >= 3000) { configured = configureSensor(); lastRetry = millis(); }
    return;
  }
  if (millis() - lastSample >= 50) { lastSample = millis(); streamSample(); }
}
