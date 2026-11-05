#include <Arduino.h>
#include <Wire.h>

// GY-521 external pull-ups are used; do not pull the I2C bus up to Uno's 5 V.
// Dedicated SDA/SCL header and A4/A5 are the same bus on Uno R3.
const uint8_t WHO_AM_I = 0x75;
const unsigned long SAMPLE_INTERVAL_MS = 50;  // 20 Hz host stream
uint8_t sensorAddress = 0;
bool configured = false;
unsigned long lastSample = 0;
unsigned long lastRetry = 0;
uint8_t readFailures = 0;

void hexByte(uint8_t value) {
  Serial.print(F("0x"));
  if (value < 16) Serial.print('0');
  Serial.print(value, HEX);
}

void holdMotorOff() {
  digitalWrite(5, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
}

void beginBus() {
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  digitalWrite(SDA, LOW);  // disable internal 5 V pull-ups
  digitalWrite(SCL, LOW);
}

bool readBytes(uint8_t address, uint8_t reg, uint8_t *data, uint8_t count) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) return false;
  uint8_t received = Wire.requestFrom(address, count, static_cast<uint8_t>(true));
  if (received != count || Wire.getWireTimeoutFlag()) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(sensorAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0 && !Wire.getWireTimeoutFlag();
}

bool checkReg(uint8_t reg, uint8_t expected) {
  uint8_t actual = 0;
  return readBytes(sensorAddress, reg, &actual, 1) && actual == expected;
}

bool findAndConfigure() {
  beginBus();
  sensorAddress = 0;
  Serial.print(F("# BUS SDA=")); Serial.print(digitalRead(SDA));
  Serial.print(F(" SCL=")); Serial.println(digitalRead(SCL));
  // Scan all normal 7-bit peripheral addresses, but only configure 6050 IDs.
  for (uint8_t address = 8; address < 120; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (Wire.getWireTimeoutFlag()) {
      Serial.println(F("# ERROR I2C_TIMEOUT check_power_ground_SDA_SCL"));
      Wire.clearWireTimeoutFlag();
      return false;
    }
    if (error == 0) {
      Serial.print(F("# I2C_FOUND address=")); hexByte(address); Serial.println();
      if (address == 0x68 || address == 0x69) {
        uint8_t id = 0;
        if (readBytes(address, WHO_AM_I, &id, 1)) {
          Serial.print(F("# WHO_AM_I address=")); hexByte(address);
          Serial.print(F(" id=")); hexByte(id); Serial.println();
          if (id == 0x68 && sensorAddress == 0) sensorAddress = address;
        }
      }
    }
  }
  if (sensorAddress == 0) {
    Serial.println(F("# ERROR MPU6050_NOT_FOUND expected_address_0x68_or_0x69_id_0x68"));
    return false;
  }
  // Wake device with X gyro PLL; enable all axes; +-250 dps / +-2 g.
  if (!writeReg(0x6B, 0x01)) return false;
  delay(100);
  if (!writeReg(0x6C, 0x00) || !writeReg(0x1A, 0x03) ||
      !writeReg(0x19, 0x09) || !writeReg(0x1B, 0x00) || !writeReg(0x1C, 0x00)) {
    Serial.println(F("# ERROR CONFIG_WRITE"));
    return false;
  }
  if (!checkReg(0x6B, 0x01) || !checkReg(0x6C, 0x00) ||
      !checkReg(0x1A, 0x03) || !checkReg(0x19, 0x09) ||
      !checkReg(0x1B, 0x00) || !checkReg(0x1C, 0x00)) {
    Serial.println(F("# ERROR CONFIG_VERIFY"));
    return false;
  }
  delay(250);  // allow gyro/temperature startup readings to settle
  Serial.print(F("# READY UNO_MPU6050 1.0 address=")); hexByte(sensorAddress);
  Serial.println(F(" accel_range_g=2 gyro_range_dps=250 stream_hz=20 MOTOR_OFF"));
  Serial.println(F("ms,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c"));
  readFailures = 0;
  return true;
}

int16_t signed16(const uint8_t *p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

void sample() {
  uint8_t bytes[14];
  if (!readBytes(sensorAddress, 0x3B, bytes, sizeof(bytes))) {
    Serial.println(F("# ERROR SAMPLE_READ"));
    if (++readFailures >= 3) { configured = false; lastRetry = millis(); }
    return;
  }
  readFailures = 0;
  int16_t values[7];
  for (uint8_t i = 0; i < 7; ++i) values[i] = signed16(bytes + 2 * i);
  Serial.print(millis());
  for (uint8_t i = 0; i < 7; ++i) { Serial.print(','); Serial.print(values[i]); }
  // Units are g and degrees/second, not m/s^2 and rad/s.
  for (uint8_t i = 0; i < 3; ++i) { Serial.print(','); Serial.print(values[i] / 16384.0f, 5); }
  for (uint8_t i = 4; i < 7; ++i) { Serial.print(','); Serial.print(values[i] / 131.0f, 3); }
  Serial.print(','); Serial.println(values[3] / 340.0f + 36.53f, 2);
}

void setup() {
  holdMotorOff();
  Serial.begin(115200);
  delay(300);
  Serial.println(F("# UNO_MPU6050 BOOT SDA=A4 SCL=A5 115200baud"));
  configured = findAndConfigure();
  lastRetry = millis();
}

void loop() {
  if (!configured) {
    if (millis() - lastRetry >= 3000) {
      configured = findAndConfigure();
      lastRetry = millis();
    }
    return;
  }
  if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = millis();
    sample();
  }
}
