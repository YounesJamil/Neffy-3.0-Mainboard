#include "VoltageReader.h"

VoltageReader::VoltageReader(uint8_t pin, uint8_t samples)
    : _pin(pin), _samples(samples) {}

void VoltageReader::init() {
    analogSetAttenuation(ADC_11db);
    analogReadResolution(12);
}

uint32_t VoltageReader::readMv() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < _samples; i++) {
        // calibrated with the chip's eFuse data; raw * 3300 / 4095 is off because
        // the ESP32-S3 ADC is non-linear and saturates around 3.1 V at 11 dB
        sum += analogReadMilliVolts(_pin);
    }
    return sum / _samples;
}

float VoltageReader::readV() {
    return readMv() / 1000.0f;
}

