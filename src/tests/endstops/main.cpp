// Endstop test: shows which endstop switch is pressed, nothing moves. Prints a
// line on every change of GPIO 11 or 12 (active-low, pull-up) and every 2 s,
// so pressing a switch by hand shows which pin belongs to which axis.

#include <Arduino.h>
#include "config.h"

static const uint8_t PINS[] = { 11, 12 };

static void print(int a, int b) {
    Serial.printf("GPIO 11: %s   GPIO 12: %s   (config: head = %u, body = %u)\n",
                  a == LOW ? "PRESSED" : "free   ", b == LOW ? "PRESSED" : "free   ",
                  BoardConfig::BTN_HEAD_PIN, BoardConfig::BTN_BODY_PIN);
}

void setup() {
    Serial.begin(115200);
    for (uint8_t p : PINS) pinMode(p, INPUT_PULLUP);
    delay(1500);
    Serial.println("=== Endstop test: press each switch by hand ===");
}

void loop() {
    static int lastA = -1, lastB = -1;
    static uint32_t lastPrintMs = 0;
    int a = digitalRead(PINS[0]), b = digitalRead(PINS[1]);
    if (a != lastA || b != lastB || millis() - lastPrintMs >= 2000) {
        print(a, b);
        lastA = a; lastB = b; lastPrintMs = millis();
    }
    delay(20);
}
