// LED test: lights the 12 LED ring in full yellow, as a quick wiring check.
// Pins and LED count come from config.h.

#include <FastLED.h>
#include "config.h"

CRGB leds[LedConfig::RING_LEDS];

void setup() {
  FastLED.addLeds<WS2812B, LedConfig::RING_PIN, GRB>(leds, LedConfig::RING_LEDS);
  FastLED.setBrightness(255);                          // 0..255
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 800);     // hard current limit
}

void loop() {
  fill_solid(leds, LedConfig::RING_LEDS, CRGB::Yellow);
  FastLED.show();
}
