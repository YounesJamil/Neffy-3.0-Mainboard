#include "PersonSensor.h"
#include "config.h"
#include <Wire.h>
#include <math.h>

// Tuned offline against a guided recording (empty room, approaching, standing in
// front, 1 m away, walking past, leaving).
//
// Someone is there as soon as it is ENTER warmer than the learned room
// temperature (a single reading), and gone when it stays below LEAVE_FRAC of
// that for LEAVE_CONFIRM_MS (see there); the gap stops flicker around the threshold.
// ENTER = max(K_NOISE * jitter, ENTER_MIN_C), so a noisy spot raises it.
// Standing in front gives +4..6 C, 1 m away about +2 C. Distance test 26 Oct
// (room 21 C): 3 m +0.85, 2 m +1.06, 1.5 m +1.8, 1 m +2.7, 0.5 m +3.8 C.
// 0.8 instead of 1.0 holds 3 m 95 % of the time instead of 66 %, with nearly
// no more false "someone there" in an empty room (2.6 % instead of 2.4 %).
static constexpr float ENTER_MIN_C = 0.8f;
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

// After leaving, the spot stays up to ~1 C warmer for a while (measured 0.8 C
// after standing 20 s), which the slow TAU_PRESENT_S does not follow, so the
// reading may never get below LEAVE_FRAC. Also gone: LEAVE_BELOW_MIN_C below
// the lowest reading while someone was there (taken over the previous complete
// MIN_BUCKET_MS bucket, from LEAVE_MIN_AFTER_MS after they came, so neither
// the way in nor the way out counts), and either less than LEAVE_NEAR_C above
// the room or the room temperature only guessed at power-up (see
// BOOT_PERSON_MAX_C; it lies far below then). The room temperature is taken
// from the reading then. Leaning back while seated (25 -> 23.8 C, room 21 C)
// stays well above LEAVE_NEAR_C.
static constexpr float    LEAVE_BELOW_MIN_C  = 1.0f;
static constexpr float    LEAVE_NEAR_C       = 1.5f;
static constexpr uint32_t LEAVE_MIN_AFTER_MS = 2000;
static constexpr uint32_t MIN_BUCKET_MS      = 15000;

// The other board sends Neffy to sleep on the first "nobody". Leaning back or
// turning away read close to the room for 1..3 s, and Neffy went to sleep and
// woke up again every ~10 s with someone sitting in front. So gone is only
// reported when it holds for LEAVE_CONFIRM_MS, and leaving is called off only
// when the reading is clearly back: ENTER above the room and within
// LEAVE_BACK_C of the lowest reading while there.
static constexpr uint32_t LEAVE_CONFIRM_MS = 2000;
static constexpr float    LEAVE_BACK_C     = 0.5f;

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
// temperature to 0 C and showed "someone there" for good. So on 0.0 C, or a
// drop by more than RESTART_DROP_C from one reading to the next (real ones drop
// at most 2.2 C), the readings of the next RESTART_SKIP_MS are skipped; room
// and state stay. A restart during the warm-up starts the warm-up again (a
// room temperature of 10 C was learned from the ramp at power-up).
static constexpr uint32_t RESTART_SKIP_MS = 2000;
static constexpr float    RESTART_DROP_C  = 8.0f;

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
    _lastMs   = _startMs = _restartMs = millis();
    _restarting = t == 0.0f;  // sensor just started, see RESTART_SKIP_MS
}

PersonState PersonSensor::update() {
    float temp = readTemp();
    if (isnan(temp)) return _present ? PersonState::PRESENT : PersonState::NONE; // I2C error: keep state

    unsigned long now = millis();
    if (temp == 0.0f || _prevTemp - temp > RESTART_DROP_C) {  // sensor restarted, see RESTART_SKIP_MS
        _restarting = true;
        _restartMs  = now;
    }
    if (_restarting) {
        _prevTemp = temp;  // compare the next reading with this one
        if (now - _restartMs < RESTART_SKIP_MS) return _present ? PersonState::PRESENT : PersonState::NONE;
        _restarting = false;
        if (_restartMs - _startMs < WARMUP_MS) _startMs = now;  // restarted during the warm-up: warm up again
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

    float aboveRoom = temp - _baseline;
    float enter     = fmaxf(K_NOISE * _noise, ENTER_MIN_C);

    // leaving (see LEAVE_BELOW_MIN_C and LEAVE_CONFIRM_MS); _prevMin is NAN until known
    bool belowMin = !isnan(_prevMin) && temp < _prevMin - LEAVE_BELOW_MIN_C
                 && (_roomGuessed || aboveRoom < LEAVE_NEAR_C);
    bool goneNow  = aboveRoom < enter * LEAVE_FRAC || belowMin;
    bool back     = aboveRoom >= enter && (isnan(_prevMin) || temp >= _prevMin - LEAVE_BACK_C);

    if (!_present) {
        if (aboveRoom >= enter) {
            _present  = true;
            _leaving  = false;
            _enterMs  = _minStartMs = now;
            _curMin   = _prevMin = NAN;
        }
    } else {
        if (_leaving && back)          _leaving = false;
        else if (goneNow && !_leaving) { _leaving = true; _leavingMs = now; }

        if (_leaving && now - _leavingMs >= LEAVE_CONFIRM_MS) {
            _present = _leaving = _roomGuessed = false;
            if (belowMin) {
                _baseline = temp;
                return PersonState::NONE;
            }
        } else if (!_leaving && now - _enterMs >= LEAVE_MIN_AFTER_MS) {
            // lowest reading while there, in MIN_BUCKET_MS buckets
            if (now - _minStartMs >= MIN_BUCKET_MS) {
                _prevMin    = _curMin;
                _curMin     = NAN;
                _minStartMs = now;
            }
            _curMin = isnan(_curMin) ? temp : fminf(_curMin, temp);
        }
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
