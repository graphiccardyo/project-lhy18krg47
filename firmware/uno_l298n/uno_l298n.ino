#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

// One brushed DC motor on L298N bridge A. Remove the ENA jumper.
const uint8_t ENA_PIN = 5;
const uint8_t IN1_PIN = 7;
const uint8_t IN2_PIN = 8;
const unsigned long LINK_TIMEOUT_MS = 1500;
char input[48];
uint8_t inputLength = 0;
bool discarding = false;
uint8_t percent = 0;
char direction = 'F';
unsigned long lastHeartbeat = 0;

uint8_t pwmValue(uint8_t pct) {
  return (static_cast<uint16_t>(pct) * 255 + 50) / 100;
}

void reportPins() {
  const uint8_t mask7 = digitalPinToBitMask(IN1_PIN);
  const uint8_t mask8 = digitalPinToBitMask(IN2_PIN);
  const uint8_t port7 = digitalPinToPort(IN1_PIN);
  const uint8_t port8 = digitalPinToPort(IN2_PIN);
  Serial.print(F("PINS d7=")); Serial.print(digitalRead(IN1_PIN));
  Serial.print(F(" d8=")); Serial.print(digitalRead(IN2_PIN));
  Serial.print(F(" out7=")); Serial.print(!!(*portOutputRegister(port7) & mask7));
  Serial.print(F(" out8=")); Serial.print(!!(*portOutputRegister(port8) & mask8));
  Serial.print(F(" mode7=")); Serial.print(!!(*portModeRegister(port7) & mask7));
  Serial.print(F(" mode8=")); Serial.println(!!(*portModeRegister(port8) & mask8));
}

void report(const __FlashStringHelper *reason) {
  Serial.print(F("STATE pct=")); Serial.print(percent);
  Serial.print(F(" pwm=")); Serial.print(pwmValue(percent));
  Serial.print(F(" dir=")); Serial.print(direction);
  Serial.print(F(" reason=")); Serial.println(reason);
  reportPins();
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

void command(char *line) {
  if (strcmp(line, "HELLO") == 0) {
    stopMotor();
    lastHeartbeat = millis();
    Serial.println(F("READY UNO_L298N_PWM 1.0"));
    report(F("CONNECTED"));
  } else if (strcmp(line, "PING") == 0) {
    lastHeartbeat = millis();
    Serial.println(F("PONG"));
  } else if (strcmp(line, "STOP") == 0) {
    stopMotor();
    report(F("STOP"));
  } else if (strcmp(line, "STATUS") == 0) {
    report(F("STATUS"));
  } else if (strcmp(line, "PINS") == 0) {
    reportPins();
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

void setup() {
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  stopMotor();
  Serial.begin(115200);
  lastHeartbeat = millis();
  Serial.println(F("READY UNO_L298N_PWM 1.0"));
  report(F("BOOT"));
}

void loop() {
  // Bound each batch so even a continuous malformed stream cannot starve timeout.
  for (uint8_t budget = 0; budget < 64 && Serial.available(); ++budget) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!discarding && inputLength != 0) {
        input[inputLength] = '\0';
        command(input);
      }
      inputLength = 0;
      discarding = false;
    } else if (!discarding) {
      if (c < 32 || c > 126 || inputLength >= sizeof(input) - 1) {
        discarding = true;
        inputLength = 0;
        reject(F("INVALID_LINE"));
      } else {
        input[inputLength++] = c;
      }
    }
  }
  if (percent != 0 && millis() - lastHeartbeat > LINK_TIMEOUT_MS) {
    stopMotor();
    report(F("TIMEOUT"));
  }
}
