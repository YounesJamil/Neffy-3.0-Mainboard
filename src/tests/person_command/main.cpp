// GET_PERSON_READING (0x03) tester for the mainboard (ESP32-S3).
// Sends the command every 5 seconds, alternating NONE -> PRESENT,
// completely independent of the physical D6T sensor. Lets you test the other
// board's reaction in isolation, without anyone walking past the sensor.
//
// Onboard LED mirrors the currently-sent state: off = NONE, on = PRESENT.

#include <Arduino.h>
#include "Neffy_Interface.h"
#include "config.h"
#include "driver/gpio.h"

static constexpr uint32_t SEND_INTERVAL_MS  = 5000;
static const char* STATE_NAMES[2] = { "NONE", "PRESENT" };

NeffyInterface interface;
Message        msg;
uint8_t        nextState    = 0;
uint8_t        currentState = 0;

unsigned long lastSendMs = 0;

void setup() {
    Serial.begin(BoardConfig::BAUDRATE);
    gpio_reset_pin(BoardConfig::LED_PIN);
    gpio_set_direction(BoardConfig::LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BoardConfig::LED_PIN, 0);

    interface.initUART(BoardConfig::RX_PIN, BoardConfig::TX_PIN, BoardConfig::BAUDRATE, UART_NUM_0);

    msg.command         = NeffyCommands::GET_PERSON_READING.id;
    msg.bytes_in_buffer = NeffyCommands::GET_PERSON_READING.payloadLength;

    Serial.println("=== GET_PERSON_READING Tester ===");
}

void loop() {
    interface.update();
    unsigned long nowMs = millis();

    if (nowMs - lastSendMs >= SEND_INTERVAL_MS) {
        lastSendMs   = nowMs;
        currentState = nextState;

        msg.buffer[0] = currentState;
        interface.writeMessage(msg);
        Serial.printf("[TEST] sent GET_PERSON_READING = %u (%s)\n", currentState, STATE_NAMES[currentState]);

        nextState = (nextState + 1) % 2;
    }

    gpio_set_level(BoardConfig::LED_PIN, currentState);
}
