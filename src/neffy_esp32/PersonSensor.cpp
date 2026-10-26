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
static constexpr float TAU_NOISE_S   = 5.0f;

// While someone is there the room temperature still creeps towards the reading,
// so a wrongly learned room level (shows "someone there" with nobody in front)
// is released after ~30 min. Someone sitting completely still is held ~20 min
// (for a room temperature only guessed at power-up see TAU_GUESSED_S).
// (A 20 s flatness check released wrong levels within a minute, but a person
// sitting still reads just as flat (0.3 C range) and was lost after 2.5 min.)
static constexpr float TAU_PRESENT_S = 900.0f;

// Leaving shows as a fast drop, but the spot stays up to ~1 C warmer for a
// while (measured 0.8 C after standing 20 s), which the slow TAU_PRESENT_S does
// not follow. So: dropped by LEAVE_DROP_FRAC of the height above the room
// within 1..2 s, or LEAVE_BELOW_MIN_C below the lowest reading since someone
// came -> gone, room temperature taken from the reading. The second catches
// leaving when the room temperature was only guessed at power-up (see
// BOOT_PERSON_MAX_C) and lies far below; leaning in and back does not go below.
static constexpr float    LEAVE_DROP_FRAC   = 0.6f;
static constexpr float    LEAVE_BELOW_MIN_C = 1.0f;
static constexpr uint32_t PEAK_BUCKET_MS    = 1000;

// The first readings after power-up can be off: take the room temperature
// straight from the readings for this long.
static constexpr uint32_t WARMUP_MS = 3000;

// A cold sensor (robot stored cold, switched on in a warm room) reads ~0.7 C
// low at first and settles within ~20 s. Until SETTLE_MS a slow rise is
// followed as room drift (TAU_ROOM_S instead of TAU_WARM_S); it gave a 47 s
// false "someone there" before. Someone stepping in rises far faster.
static constexpr uint32_t SETTLE_MS = 30000;

// Someone already sitting in front at power-up must not be learned as the room.
// Right after power-up the sensor's own temperature (PTAT) is at most a few C
// above the room (cold start: 22.9 C with the room at 19.3 C; it warms to ~8 C
// above within minutes), while a person reads 25..26 C. A reading this much
// above PTAT can only be something warm in view: PTAT is taken as the room then.
// After a quick restart PTAT is still warm and this does not trigger.
// A robot colder than the room (stored cold) also reads above PTAT with nobody
// there: more than BOOT_PERSON_MAX_C above (measured 8.5 C with PTAT 11.5 C)
// means a cold sensor, not a person, and the reading is taken as the room. A
// cold robot closer to the room temperature can still pass as someone there,
// so a guessed room temperature is relearned faster (TAU_GUESSED_S): someone
// sitting still is held ~3 min, a wrong guess released after ~3 min.
static constexpr float BOOT_PERSON_MARGIN_C = 1.0f;
static constexpr float BOOT_PERSON_MAX_C    = 5.0f;
static constexpr float TAU_GUESSED_S        = 120.0f;

// After its own restart (power dip or loose contact while the board keeps
// running) the sensor sends exactly 0.0 C with a valid PEC and then ramps up
// within ~0.5 s (0.0, 10.5, 20.3, 21.6, 23.5 C). Taken as real it set the room
// temperature to 0 C and showed "someone there" for good. So on 0.0 C the
// readings of the next RESTART_SKIP_MS are skipped; room and state stay.
static constexpr uint32_t RESTART_SKIP_MS = 2000;

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
    _baseline = _lastTemp = _prevTemp = isnan(t) ? 20.0f : t;  // no reading: 20 C until warm-up
    _lastMs   = _startMs = _peakStartMs = millis();
    _peak     = _prevPeak = _baseline;
}

PersonState PersonSensor::update() {
    float temp = readTemp();
    if (isnan(temp)) return _present ? PersonState::PRESENT : PersonState::NONE; // I2C error: keep state

    unsigned long now = millis();
    if (temp == 0.0f) {  // sensor restarted, see RESTART_SKIP_MS
        _restarting = true;
        _restartMs  = now;
    }
    if (_restarting) {
        if (now - _restartMs < RESTART_SKIP_MS) return _present ? PersonState::PRESENT : PersonState::NONE;
        _restarting = false;
    }

    float dt = fminf((now - _lastMs) / 1000.0f, 1.0f);  // seconds since the last reading, at most 1
    _lastMs  = now;

    _lastTemp    = temp;
    float jitter = fabsf(temp - _prevTemp);
    _prevTemp    = temp;

    if (now - _startMs < WARMUP_MS) {
        float overPtat = temp - _lastPtat;  // see BOOT_PERSON_MARGIN_C and BOOT_PERSON_MAX_C
        _roomGuessed = overPtat > BOOT_PERSON_MARGIN_C && overPtat <= BOOT_PERSON_MAX_C;
        _baseline    = _roomGuessed ? _lastPtat : temp;
        return PersonState::NONE;
    }

    // highest reading of the last 1..2 s, in two PEAK_BUCKET_MS buckets
    if (now - _peakStartMs >= PEAK_BUCKET_MS) {
        _prevPeak    = _peak;
        _peak        = temp;
        _peakStartMs = now;
    }
    _peak = fmaxf(_peak, temp);
    float recentPeak = fmaxf(_peak, _prevPeak);

    float aboveRoom = temp - _baseline;

    float enter = fmaxf(K_NOISE * _noise, ENTER_MIN_C);
    bool  left  = recentPeak - _baseline >= enter                                    // see LEAVE_DROP_FRAC
               && (recentPeak - temp >= LEAVE_DROP_FRAC * (recentPeak - _baseline)
                   || temp < _presentMin - LEAVE_BELOW_MIN_C);
    if (_present) _presentMin = fminf(_presentMin, temp);

    if (!_present && aboveRoom >= enter) {
        _present    = true;
        _presentMin = temp;
    } else if (_present && aboveRoom < enter * LEAVE_FRAC) {
        _present     = false;
        _roomGuessed = false;
    } else if (_present && left) {
        _present     = false;
        _roomGuessed = false;
        _baseline    = temp;
        return PersonState::NONE;
    }

    // within the noise band around the room temperature (0.2 C margin for the 0.1 C steps)
    bool quiet = fabsf(aboveRoom) < 3.0f * _noise + 0.2f;
    if (!_present && quiet) _noise += (fminf(jitter, 0.5f) - _noise) * dt / TAU_NOISE_S;  // 0.5: ignore jumps

    // how fast the room temperature follows the reading (see the TAU_* constants)
    float tau;
    if (aboveRoom < -2.0f * _noise) tau = TAU_DOWN_S;   // clearly colder: follow at once
    else if (aboveRoom < 0.0f)      tau = TAU_ROOM_S;
    else if (_present)              tau = _roomGuessed ? TAU_GUESSED_S : TAU_PRESENT_S;
    else if (quiet)                 tau = TAU_ROOM_S;
    else if (now - _startMs < SETTLE_MS) tau = TAU_ROOM_S;  // cold sensor settling
    else                            tau = TAU_WARM_S;
    _baseline += aboveRoom * fminf(dt / tau, 1.0f);  // at most up to the reading

    return _present ? PersonState::PRESENT : PersonState::NONE;
}
