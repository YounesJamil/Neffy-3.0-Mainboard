#include "PersonSensor.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

// Tuned offline against a guided recording (empty room, approaching, standing in
// front, 1 m away, walking past, leaving).
//
// Someone is there as soon as it is ENTER warmer than the learned room
// temperature, and gone as soon as it is less than LEAVE_FRAC of that: both on
// a single reading, the gap only stops flicker around the threshold.
// ENTER = max(K_NOISE * jitter, ENTER_MIN_C), so a noisy spot raises it.
// Standing in front gives +4..6 C, 1 m away about +2 C.
static constexpr float ENTER_MIN_C = 1.0f;
static constexpr float K_NOISE     = 8.0f;
static constexpr float LEAVE_FRAC  = 0.6f;

// Room temperature: follows a drop at once (a hand or person can only warm the
// sensor) and rises slowly, so someone approaching slowly or standing a bit
// away is not learned away as "room". Time constants in seconds.
static constexpr float TAU_DOWN_S    = 1.0f;   // clearly colder than the room
static constexpr float TAU_ROOM_S    = 10.0f;  // within the noise: room drift
static constexpr float TAU_WARM_S    = 120.0f; // warmer, but below ENTER
static constexpr float TAU_PRESENT_S = 300.0f; // someone there
static constexpr float TAU_NOISE_S   = 5.0f;

// Safety net for a wrongly learned room temperature (it showed up as "someone
// there" for minutes): a person moves the reading by >= 0.5 C within seconds,
// a wrong room temperature leaves it flat. Flat for FLAT_WINDOW_MS while
// "someone there" -> relearn with TAU_FLAT_S instead of TAU_PRESENT_S.
static constexpr uint32_t FLAT_WINDOW_MS = 20000;
static constexpr float    FLAT_RANGE_C   = 0.4f;
static constexpr float    TAU_FLAT_S     = 20.0f;

// The first readings after power-up can be off: take the room temperature
// straight from the readings for this long.
static constexpr uint32_t WARMUP_MS = 3000;

static constexpr uint8_t TEMP_BYTE_COUNT = 5;

// CRC-8 (polynomial 0x07) over the read address and the 4 data bytes, as the
// D6T-1A sends it in its last byte (PEC). Checked against raw readings.
static uint8_t crc8(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    return crc;
}

float PersonSensor::readTemp() {
    Wire.beginTransmission(D6TConfig::I2C_ADDR);
    Wire.write(D6TConfig::I2C_CMD);
    if (Wire.endTransmission(true) != 0) return NAN;

    if (Wire.requestFrom((uint8_t)D6TConfig::I2C_ADDR, (uint8_t)TEMP_BYTE_COUNT) != TEMP_BYTE_COUNT) return NAN;
    uint8_t buf[TEMP_BYTE_COUNT];
    for (int i = 0; i < TEMP_BYTE_COUNT; i++) buf[i] = Wire.read();

    uint8_t crc = crc8(0, (D6TConfig::I2C_ADDR << 1) | 1);
    for (int i = 0; i < TEMP_BYTE_COUNT - 1; i++) crc = crc8(crc, buf[i]);
    if (crc != buf[TEMP_BYTE_COUNT - 1]) return NAN;  // corrupted reading

    _lastPtat = (int16_t)((buf[1] << 8) | buf[0]) * 0.1f;  // sensor's own temperature
    return (int16_t)((buf[3] << 8) | buf[2]) * 0.1f;
}

void PersonSensor::init() {
    Wire.begin(D6TConfig::SDA_PIN, D6TConfig::SCL_PIN);
    Wire.setClock(D6TConfig::I2C_CLOCK);
    float t = readTemp();
    _baseline = _lastTemp = _prevTemp = isnan(t) ? 20.0f : t;
    _lastMs   = _startMs = _flatStartMs = millis();
    _flatMin  = _flatMax = _baseline;
}

PersonState PersonSensor::update() {
    float temp = readTemp();
    if (isnan(temp)) return _present ? PersonState::PRESENT : PersonState::NONE; // I2C error: keep state

    unsigned long now = millis();
    float dt = fminf((now - _lastMs) / 1000.0f, 1.0f);
    _lastMs  = now;

    _lastTemp    = temp;
    float jitter = fabsf(temp - _prevTemp);
    _prevTemp    = temp;

    if (now - _startMs < WARMUP_MS) {
        _baseline = temp;
        return PersonState::NONE;
    }

    // flatness over fixed windows (see FLAT_WINDOW_MS)
    _flatMin = fminf(_flatMin, temp);
    _flatMax = fmaxf(_flatMax, temp);
    if (now - _flatStartMs >= FLAT_WINDOW_MS) {
        _flat        = _flatMax - _flatMin < FLAT_RANGE_C;
        _flatStartMs = now;
        _flatMin     = _flatMax = temp;
    }

    float d = temp - _baseline;

    float enter = fmaxf(K_NOISE * _noise, ENTER_MIN_C);
    if      (!_present && d >= enter)              _present = true;
    else if ( _present && d <  enter * LEAVE_FRAC) _present = false;

    bool quiet = fabsf(d) < 3.0f * _noise + 0.2f;
    if (!_present && quiet) _noise += (fminf(jitter, 0.5f) - _noise) * dt / TAU_NOISE_S;

    float tau;
    if      (d < 0.0f) tau = d < -2.0f * _noise ? TAU_DOWN_S : TAU_ROOM_S;
    else if (_present) tau = _flat ? TAU_FLAT_S : TAU_PRESENT_S;
    else if (quiet)    tau = TAU_ROOM_S;
    else               tau = TAU_WARM_S;
    _baseline += d * fminf(dt / tau, 1.0f);

    return _present ? PersonState::PRESENT : PersonState::NONE;
}
