// Stepper test for the body (linear) axis, with breathing LEDs.
//
// Structured like MotorControlLoop: the motor lives in its own tight loop on
// core 0, fed target positions through a queue. The LED strip is driven from
// core 1 and its brightness tracks the body position, so the light breathes in
// sync with the motion: moving out = inhale = brighter, back = exhale = darker.
//
// Motor geometry and pins come from config.h; the LEDs are the 12 LED ring.

#include <Arduino.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
// Let the stepper's 50 kHz timer ISR run during LED output. Without this FastLED
// blocks interrupts for the whole frame and the motor loses step pulses, which
// shows up as audible jerking. The cost is a rare LED glitch.
#define FASTLED_ALLOW_INTERRUPTS 1
#define FASTLED_INTERRUPT_RETRY_COUNT 0
#include <FastLED.h>
#include "MotorController.h"
#include "config.h"

static constexpr float    TRAVEL_MM    = 20.0f; // stroke length
static constexpr float    MOVE_SECONDS = 4.0f;  // time budget per stroke
static constexpr uint32_t PAUSE_MS     = 1000;  // hold time at each end

// WS2812B ring
static constexpr uint8_t  STRIP_PIN    = LedConfig::RING_PIN;
static constexpr uint8_t  NUM_LEDS     = LedConfig::RING_LEDS;
static constexpr uint8_t  MIN_BRIGHT   = 8;     // never fully off at the bottom
static constexpr uint8_t  MAX_BRIGHT   = 255;
static constexpr uint32_t LED_FRAME_MS = 33;    // 30 Hz; every frame disturbs the step ISR a little
static const     CRGB     BREATH_COLOR = CRGB(255, 150, 60);   // warm amber

struct MoveCmd { float position; float seconds; };

static QueueHandle_t  g_bodyQueue  = nullptr;
static volatile float g_bodyPosMm  = 0.0f;   // published by the motor task
static CRGB           g_leds[NUM_LEDS];

// ---- motor task: core 0, nothing else, update() as tight as possible ----
static void motorTask(void*) {
    esp_task_wdt_init(30, false);
    g_bodyQueue = xQueueCreate(1, sizeof(MoveCmd));

    LinearActuator body(BoardConfig::STEPPER_MICROSTEPS,
                        BoardConfig::BODY_MM_PER_ROTATION,
                        BoardConfig::STEP_BODY_PIN,
                        BoardConfig::DIR_BODY_PIN,
                        BoardConfig::EN_BODY_PIN,
                        -1, -1,
                        BoardConfig::BTN_BODY_PIN);
    MotorController controller;

    controller.addMotor(&body);
    controller.begin();
    controller.enableStepping();

    Serial.println("homing...");
    body.configHoming(BoardConfig::HOME_DIR_BODY == 1 ? BACKWARD : FORWARD,
                      BoardConfig::HOMING_SPEED_BODY_MM_S,
                      BoardConfig::HOME_BACKOFF_BODY_MM,
                      BoardConfig::MAX_POSITION_BODY_MM);
    while (!body.home()) {}
    body.tare();
    Serial.println("homed, position = 0");

    MoveCmd cmd;
    for (;;) {
        if (xQueueReceive(g_bodyQueue, &cmd, 0) == pdTRUE)
            body.moveToPositionInTime(cmd.position, cmd.seconds);

        body.update();
        g_bodyPosMm = body.getPosition();
    }
}

// Maps the body position onto strip brightness and pushes a frame.
static void showBreathFrame() {
    float t = g_bodyPosMm / TRAVEL_MM;            // 0 at home, 1 fully out
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    uint8_t bright = MIN_BRIGHT + (uint8_t)((MAX_BRIGHT - MIN_BRIGHT) * t);

    // A parked body must not cost the motor anything: skip identical frames.
    static int16_t lastBright = -1;
    if (bright == lastBright) return;
    lastBright = bright;

    FastLED.setBrightness(bright);
    fill_solid(g_leds, NUM_LEDS, BREATH_COLOR);
    FastLED.show();
}

// Runs the LED fade for `ms`, keeping the strip in step with the motor.
static void breatheFor(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        showBreathFrame();
        delay(LED_FRAME_MS);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.printf("=== Stepper Test (body, %.0f mm, %.1f s/stroke + breathing LEDs) ===\n",
                  TRAVEL_MM, MOVE_SECONDS);

    FastLED.addLeds<WS2812B, STRIP_PIN, GRB>(g_leds, NUM_LEDS);
    FastLED.setMaxPowerInVoltsAndMilliamps(5, 800);   // lower this if the motor rail browns out
    showBreathFrame();

    // The motor task never yields: without this, core 0's idle watchdog prints
    // an error onto UART0 every 30 s.
    disableCore0WDT();
    xTaskCreatePinnedToCore(&motorTask, "MotorLoop", 4096, nullptr, 2, nullptr, 0);
}

void loop() {
    if (!g_bodyQueue) { delay(50); return; }   // wait for the motor task to come up

    MoveCmd out  = { TRAVEL_MM, MOVE_SECONDS };
    MoveCmd back = { 0.0f,      MOVE_SECONDS };

    Serial.println("-> inhale (out, brightening)");
    xQueueOverwrite(g_bodyQueue, &out);
    breatheFor((uint32_t)(MOVE_SECONDS * 1000) + PAUSE_MS);
    Serial.printf("   at %.2f mm\n", g_bodyPosMm);

    Serial.println("-> exhale (back, dimming)");
    xQueueOverwrite(g_bodyQueue, &back);
    breatheFor((uint32_t)(MOVE_SECONDS * 1000) + PAUSE_MS);
    Serial.printf("   at %.2f mm\n", g_bodyPosMm);
}
