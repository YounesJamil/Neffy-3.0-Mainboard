// Thermal sensor raw readout: one CSV line per read "t_ms,ptat_C,pixel_C,read_us,raw"
// (raw = the 5 bytes as received, the last one is the PEC checksum).
// Read interval and I2C clock can be switched over USB to find out how often the
// D6T delivers new values:
//   '1' 20 ms   '2' 50 ms   '3' 100 ms   '4' 200 ms   '5' 300 ms
//   'k' 10 kHz  'K' 100 kHz
// Each switch prints a line "# interval <ms> clock <Hz>".

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

static uint32_t intervalMs = 200;
static uint32_t clockHz    = D6TConfig::I2C_CLOCK;

static void announce() { Serial.printf("# interval %lu clock %lu\n", (unsigned long)intervalMs, (unsigned long)clockHz); }

void setup() {
    Serial.begin(115200);
    Wire.begin(D6TConfig::SDA_PIN, D6TConfig::SCL_PIN);
    Wire.setClock(clockHz);
    delay(1500);
    Serial.println("t_ms,ptat_C,pixel_C,read_us,raw");
    announce();
}

void loop() {
    while (Serial.available()) {
        switch (Serial.read()) {
            case '1': intervalMs = 20;  break;
            case '2': intervalMs = 50;  break;
            case '3': intervalMs = 100; break;
            case '4': intervalMs = 200; break;
            case '5': intervalMs = 300; break;
            case 'k': clockHz = 10000;  Wire.setClock(clockHz); break;
            case 'K': clockHz = 100000; Wire.setClock(clockHz); break;
            default: continue;
        }
        announce();
    }

    static uint32_t last = 0;
    if (millis() - last < intervalMs) return;
    last = millis();

    uint32_t t0 = micros();
    Wire.beginTransmission(D6TConfig::I2C_ADDR);
    Wire.write(D6TConfig::I2C_CMD);
    if (Wire.endTransmission(true) != 0) { Serial.println("# i2c error"); return; }
    uint8_t buf[5] = {0};
    Wire.requestFrom((uint8_t)D6TConfig::I2C_ADDR, (uint8_t)5);
    for (int i = 0; i < 5; i++) buf[i] = Wire.read();
    uint32_t us = micros() - t0;

    float ptat  = (int16_t)((buf[1] << 8) | buf[0]) * 0.1f;
    float pixel = (int16_t)((buf[3] << 8) | buf[2]) * 0.1f;
    Serial.printf("%lu,%.1f,%.1f,%lu,%02X %02X %02X %02X %02X\n", (unsigned long)millis(), ptat, pixel,
                  (unsigned long)us, buf[0], buf[1], buf[2], buf[3], buf[4]);
}
