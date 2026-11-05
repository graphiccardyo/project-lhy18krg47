#include <Arduino.h>

// Diagnostic only. Motor 12 V must remain OFF and IN1/IN2 disconnected.
// ENA is held LOW. D7 and D8 switch together; LED L marks the HIGH phase.
const uint8_t ENA = 5;
const uint8_t REFERENCE = 7;
const uint8_t SUSPECT = 8;
const unsigned long PHASE_MS = 4000;
bool highPhase = false;
unsigned long phaseStarted = 0;
unsigned long lastReport = 0;

uint8_t outputLatch(uint8_t pin) {
  return !!(*portOutputRegister(digitalPinToPort(pin)) & digitalPinToBitMask(pin));
}

uint8_t outputMode(uint8_t pin) {
  return !!(*portModeRegister(digitalPinToPort(pin)) & digitalPinToBitMask(pin));
}

void setPhase(bool high) {
  digitalWrite(ENA, LOW);
  digitalWrite(REFERENCE, high ? HIGH : LOW);
  digitalWrite(SUSPECT, high ? HIGH : LOW);
  digitalWrite(LED_BUILTIN, high ? HIGH : LOW);
  highPhase = high;
  phaseStarted = millis();
}

void report() {
  Serial.print(F("PIN_DIAG phase=")); Serial.print(highPhase ? F("HIGH") : F("LOW"));
  Serial.print(F(" ENA=")); Serial.print(digitalRead(ENA));
  Serial.print(F(" D7=")); Serial.print(digitalRead(REFERENCE));
  Serial.print(F(" D8=")); Serial.print(digitalRead(SUSPECT));
  Serial.print(F(" OUT7=")); Serial.print(outputLatch(REFERENCE));
  Serial.print(F(" OUT8=")); Serial.print(outputLatch(SUSPECT));
  Serial.print(F(" MODE7=")); Serial.print(outputMode(REFERENCE));
  Serial.print(F(" MODE8=")); Serial.println(outputMode(SUSPECT));
}

void setup() {
  digitalWrite(ENA, LOW);
  digitalWrite(REFERENCE, LOW);
  digitalWrite(SUSPECT, LOW);
  pinMode(ENA, OUTPUT);
  pinMode(REFERENCE, OUTPUT);
  pinMode(SUSPECT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("UNO_PIN_DIAG 1.0 MOTOR_ENA_ALWAYS_LOW"));
  setPhase(true);
}

void loop() {
  if (millis() - phaseStarted >= PHASE_MS) setPhase(!highPhase);
  // Read after the output has settled, rather than at the transition.
  if (millis() - phaseStarted >= 50 && millis() - lastReport >= 500) {
    report();
    lastReport = millis();
  }
}
