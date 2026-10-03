// Blink test: toggles a GPIO every 500 ms and reports it over USB, the very
// first "is the board alive" check: the onboard LED blinks.

#include <Arduino.h>
#include "config.h"

static constexpr uint8_t BLINK_PIN = BoardConfig::LED_PIN;  // onboard LED

void setup() {
  Serial.begin(115200);
  pinMode(BLINK_PIN, OUTPUT);
  delay(1000);
  Serial.println("Neffy Mainboard: blink test started");
}

void loop() {
  digitalWrite(BLINK_PIN, HIGH);
  Serial.println("PIN HIGH");
  delay(500);

  digitalWrite(BLINK_PIN, LOW);
  Serial.println("PIN LOW");
  delay(500);
}