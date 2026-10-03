// LED fade test: the 12 LED ring fades in and out in yellow (about 4 s per
// cycle). Pins and LED count come from config.h.

#include <FastLED.h>
#include "config.h"

CRGB leds[LedConfig::RING_LEDS];

void setup() {
  FastLED.addLeds<WS2812B, LedConfig::RING_PIN, GRB>(leds, LedConfig::RING_LEDS);
  fill_solid(leds, LedConfig::RING_LEDS, CRGB::Yellow);
}

void loop() {
  for (int b = 0; b <= 255; b++) {
    FastLED.setBrightness(b);
    FastLED.show();
    delay(15);
  }
  for (int b = 255; b >= 0; b--) {
    FastLED.setBrightness(b);
    FastLED.show();
    delay(15);
  }
}
