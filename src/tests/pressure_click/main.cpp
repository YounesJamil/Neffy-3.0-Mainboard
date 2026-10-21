// Pocket test: the firmware's click detection with light response, plus a
// voltage log for diagnosing the pressure sensors.
//
// Every 20 ms both pockets are read (8x averaged, no smoothing, so fast spikes
// stay visible) and fed into the same PressureDetector as in the firmware, with
// the sensitivities from config.h. A detected click flashes that pocket's LED
// strip to full and fades it out, as in the firmware. Nothing is sent over UART.
//
// Serial output, one CSV line per sample:
//   t_ms,L_V,R_V,mark
// A key typed in the serial monitor is logged in the mark column of the next
// line (see the legend printed at start); detected clicks show as "click L/R".
// For recording with markers use tools/pocket_log.py.

#include <Arduino.h>
#include <FastLED.h>
#include "config.h"
#include "Neffy_Interface.h"
#include "PressureDetector.h"
#include "VoltageReader.h"

static constexpr uint32_t SAMPLE_INTERVAL_MS = 20;
static constexpr float    POCKET_IDLE_LEVEL  = 0.3f;   // steady glow between clicks (0..1)
static constexpr float    CLICK_FADE_S       = 0.3f;   // click flash fades out within this

static NeffyInterface   interface;   // the detector needs one; it stays muted
static VoltageReader    voltageL(BoardConfig::PRESSURE_PIN_L, BoardConfig::PRESSURE_SAMPLES);
static VoltageReader    voltageR(BoardConfig::PRESSURE_PIN_R, BoardConfig::PRESSURE_SAMPLES);
static PressureDetector pocketL(interface, voltageL, true,  BoardConfig::PRESSURE_INVERT_L, BoardConfig::PRESSURE_SENSITIVITY_L);
static PressureDetector pocketR(interface, voltageR, false, BoardConfig::PRESSURE_INVERT_R, BoardConfig::PRESSURE_SENSITIVITY_R);
static CRGB ledsL[LedConfig::POCKET_LEDS];
static CRGB ledsR[LedConfig::POCKET_LEDS];

static uint32_t nextSampleMs = 0;

// level 0..1 -> brightness, cubic like the firmware (the eye is logarithmic)
static void fillLevel(CRGB* strip, float level) {
    using namespace LedConfig;
    float   perceived  = level * level * level;
    uint8_t brightness = MIN_BRIGHT + (uint8_t)((255 - MIN_BRIGHT) * perceived);
    CRGB color(COLOR_R, COLOR_G, COLOR_B);
    color.nscale8(brightness);
    fill_solid(strip, POCKET_LEDS, color);
}

void setup() {
    Serial.begin(115200);
    FastLED.addLeds<WS2812B, LedConfig::POCKET_L_PIN, GRB>(ledsL, LedConfig::POCKET_LEDS);
    FastLED.addLeds<WS2812B, LedConfig::POCKET_R_PIN, GRB>(ledsR, LedConfig::POCKET_LEDS);

    voltageL.init();
    voltageR.init();
    pocketL.init();
    pocketR.init();
    pocketL.setMuted(true);              // test only: no UART messages
    pocketR.setMuted(true);
    delay(1500); // give the monitor time to attach

    Serial.println("# Pocket test (L = pin 13, R = pin 14), every 20 ms; a click flashes its LEDs");
    Serial.println("# Keys: r = rest    h = hand on (no press)   p = press + hold");
    Serial.println("#       l = let go  k = single click         x = pressed with plastic");
    Serial.println("#       any other key is logged as typed");
    Serial.println("t_ms,L_V,R_V,mark");
    nextSampleMs = millis();
}

void loop() {
    uint32_t now = millis();
    if ((int32_t)(now - nextSampleMs) < 0) return;
    nextSampleMs += SAMPLE_INTERVAL_MS; // fixed grid, no drift from print time

    char mark = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c > ' ') mark = c; // skip CR/LF/space
    }

    pocketL.update(now);                 // reads the pocket and detects clicks
    pocketR.update(now);
    bool clickL = pocketL.click();
    bool clickR = pocketR.click();

    // a click jumps to full, then fades out within CLICK_FADE_S
    static float flashL = 0.0f, flashR = 0.0f;
    float step = SAMPLE_INTERVAL_MS / 1000.0f / CLICK_FADE_S;
    flashL = clickL ? 1.0f : fmaxf(flashL - step, 0.0f);
    flashR = clickR ? 1.0f : fmaxf(flashR - step, 0.0f);
    fillLevel(ledsL, fmaxf(POCKET_IDLE_LEVEL, flashL));
    fillLevel(ledsR, fmaxf(POCKET_IDLE_LEVEL, flashR));
    FastLED.show();

    Serial.printf("%lu,%.3f,%.3f,", (unsigned long)now, pocketL.lastVoltage(), pocketR.lastVoltage());
    if (mark)   Serial.print(mark);
    if (clickL) Serial.print(" click L");
    if (clickR) Serial.print(" click R");
    Serial.println();
}
