#include <Arduino.h>
#include <string.h>

// One brushed DC motor on L298N bridge A, full on/off only (no PWM).
// ENA is driven with digitalWrite, so the bridge is either fully on or off.
// Remove the ENA jumper; wiring matches uno_l298n.
const uint8_t ENA_PIN = 5;
const uint8_t IN1_PIN = 7;
const uint8_t IN2_PIN = 8;
const unsigned long LINK_TIMEOUT_MS = 1500;
char input[32];
uint8_t inputLength = 0;
bool discarding = false;
unsigned long inputLastByteAt = 0;
char running = 0;  // 'F', 'R' or 0 when stopped
unsigned long lastHeartbeat = 0;

void report(const __FlashStringHelper *reason) {
  Serial.print(F("STATE run=")); Serial.print(running ? running : '0');
  Serial.print(F(" ena=")); Serial.print(digitalRead(ENA_PIN));
  Serial.print(F(" d7=")); Serial.print(digitalRead(IN1_PIN));
  Serial.print(F(" d8=")); Serial.print(digitalRead(IN2_PIN));
  Serial.print(F(" reason=")); Serial.println(reason);
}

void stopMotor() {
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  running = 0;
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
    Serial.println(F("READY UNO_L298N_ONOFF 1.0"));
    report(F("CONNECTED"));
  } else if (strcmp(line, "PING") == 0) {
    lastHeartbeat = millis();
    Serial.println(F("PONG"));
  } else if (strcmp(line, "STOP") == 0) {
    stopMotor();
    report(F("STOP"));
  } else if (strcmp(line, "STATUS") == 0) {
    report(F("STATUS"));
  } else if (strcmp(line, "RUN F") == 0 || strcmp(line, "RUN R") == 0) {
    char requested = line[4];
    if (running && running != requested) {
      reject(F("STOP_BEFORE_REVERSE")); return;
    }
    digitalWrite(IN1_PIN, requested == 'F' ? HIGH : LOW);
    digitalWrite(IN2_PIN, requested == 'F' ? LOW : HIGH);
    digitalWrite(ENA_PIN, HIGH);
    running = requested;
    lastHeartbeat = millis();
    report(F("RUN"));
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
  Serial.println(F("READY UNO_L298N_ONOFF 1.0"));
  report(F("BOOT"));
}

void loop() {
  // Never join a new command onto an old line whose newline was lost.
  if ((inputLength || discarding) && millis() - inputLastByteAt > 250) {
    inputLength = 0;
    discarding = false;
    reject(F("COMMAND_TIMEOUT"));
  }
  // Bound each batch so even a continuous malformed stream cannot starve timeout.
  for (uint8_t budget = 0; budget < 64 && Serial.available(); ++budget) {
    char c = Serial.read();
    inputLastByteAt = millis();
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
  if (running && millis() - lastHeartbeat > LINK_TIMEOUT_MS) {
    stopMotor();
    report(F("TIMEOUT"));
  }
}
