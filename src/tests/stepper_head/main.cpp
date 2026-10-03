// Head test: home against the endstop, then move the head up and down forever.
// LED brightness follows the head position: down = almost off, up = full.
//
// The motor runs in a task on core 0, the LEDs on core 1, and they must stay
// apart: FastLED refills its RMT buffer from an interrupt several times per frame
// (12 LEDs need 288 pulses, the buffer holds 96). The stepper's 50 kHz timer
// interrupt on the same core delays those refills, the frame breaks off, and the
// strip keeps showing its last complete frame.

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <FastLED.h>
#include "MotorController.h"
#include "config.h"

static constexpr float   TRAVEL_DEG   = BoardConfig::MAX_POSITION_HEAD_DEG;  // full range
static constexpr float   MOVE_SECONDS = 4.0f;
static constexpr uint8_t STRIP_PIN    = LedConfig::RING_PIN;
static constexpr uint8_t NUM_LEDS     = LedConfig::RING_LEDS;

static volatile bool  g_homed      = false;
static volatile float g_headPosDeg = 0.0f;
static CRGB           leds[NUM_LEDS];

// Core 0: owns the motor and its step interrupt. Homes, then moves up and down.
static void motorTask(void*) {
    esp_task_wdt_init(30, false);

    AngularActuator head(BoardConfig::STEPPER_MICROSTEPS,
                         BoardConfig::HEAD_GEAR_RATIO,
                         BoardConfig::STEP_HEAD_PIN,
                         BoardConfig::DIR_HEAD_PIN,
                         BoardConfig::EN_HEAD_PIN,
                         -1, -1,
                         BoardConfig::BTN_HEAD_PIN);
    MotorController controller;
    controller.addMotor(&head);
    controller.begin();             // attaches the step interrupt to this core
    controller.enableStepping();

    head.invertDirection(BoardConfig::HEAD_DIR_INVERTED);
    head.configHoming(BoardConfig::HOME_DIR_HEAD == 1 ? BACKWARD : FORWARD,
                      BoardConfig::HOMING_SPEED_HEAD_DEG_S,
                      BoardConfig::HOME_BACKOFF_HEAD_DEG,
                      BoardConfig::MAX_POSITION_HEAD_DEG);
    while (!head.home()) {}
    head.tare();
    g_homed = true;

    float target = TRAVEL_DEG;
    for (;;) {
        head.moveToPositionInTime(target, MOVE_SECONDS);
        MotorState state;
        do {
            state        = head.update();
            g_headPosDeg = head.getPosition();
        } while (state == MOVING_TO_TARGET || state == MOVING_AT_SPEED);

        target = (target == 0.0f) ? TRAVEL_DEG : 0.0f;
    }
}

// level 0 = almost off, 1 = full. Cubic because the eye perceives brightness
// logarithmically; a linear ramp looks fully on almost immediately.
static void setLeds(float level) {
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    FastLED.setBrightness(1 + (uint8_t)(254.0f * level * level * level));
    fill_solid(leds, NUM_LEDS, CRGB(255, 150, 60));
    FastLED.show();
}

void setup() {
    FastLED.addLeds<WS2812B, STRIP_PIN, GRB>(leds, NUM_LEDS);
    setLeds(1.0f);      // strip check: full brightness for one second at power-up
    delay(1000);
    setLeds(0.0f);

    // The motor task never yields: without this, core 0's idle watchdog prints
    // an error onto UART0 every 30 s.
    disableCore0WDT();
    xTaskCreatePinnedToCore(&motorTask, "MotorLoop", 4096, nullptr, 2, nullptr, 0);
}

// Core 1: only the LEDs.
void loop() {
    setLeds(g_homed ? g_headPosDeg / TRAVEL_DEG : 0.0f);
    delay(20);
}
