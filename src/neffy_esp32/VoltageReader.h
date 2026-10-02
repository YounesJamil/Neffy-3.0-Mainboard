#pragma once
#include <Arduino.h>

/** ADC voltage reader with multi-sample averaging (calibrated millivolts). */
class VoltageReader {
public:
    /**
     * @param pin     ADC pin number to read from
     * @param samples Number of samples averaged per reading
     */
    VoltageReader(uint8_t pin, uint8_t samples = 16);

    /** Configures ADC attenuation and resolution. Call once before first read. */
    void init();

    /** Returns the averaged voltage in volts. */
    float readV();

    /** Returns the averaged voltage in millivolts. */
    uint32_t readMv();

private:
    uint8_t _pin;
    uint8_t _samples;
};
