#include "PressureDetector.h"
#include "Arduino.h"

// Tuned offline against recorded pocket logs (20 ms samples, both pockets,
// clicks, resting hands, long presses). All rates assume READ_INTERVAL_MS.
static constexpr uint32_t READ_INTERVAL_MS = 20;
static constexpr float    DT               = READ_INTERVAL_MS / 1000.0f;

static constexpr float A_SMOOTH   = 0.5f;
static constexpr float A_LOCAL_UP = DT / 0.6f; // local baseline follows a resting hand
static constexpr float A_LOCAL_DN = DT / 0.1f; // ... and the undershoot after a click at once
static constexpr float A_NOISE    = DT / 2.0f;
static constexpr float A_REST_UP  = DT / 1.0f; // sensor recovering after release is slow, a hand is a jump
static constexpr float A_REST_DN  = DT / 1.0f; // a hand only raises the level, so lower = new rest

// Pulse threshold = max(K * jitter, floor), plus a share of the learned click
// height while a hand rests on the pocket: clicks under a resting hand barely
// show (the pad is smaller than a hand), so only clear ones count there.
static constexpr float    K_CLICK       = 6.0f;
static constexpr float    CLICK_FRAC    = 0.5f;
static constexpr float    A_CLICK_AMP   = 0.2f;   // per click
static constexpr float    CLICK_MIN_V   = 0.015f;
static constexpr float    CLICK_MAX_V   = 0.25f;
static constexpr uint32_t CLICK_MAX_MS  = 600;    // longer = press/hand, not a click
static constexpr uint32_t PULSE_REPEAT_MS = 200;  // min gap between two pulse clicks
// A pulse click is only sent once the level is back down (to the rest level, or
// with a hand on the pocket to the hand level); a hand or press stays up and is
// dropped after CONFIRM_MS. Real clicks drop at once, so no delay.
static constexpr uint32_t CONFIRM_MS    = 300;

// Light clicks are often no pulse but a one-sample jump (down on some pockets)
// with a rebound. Any jump >= the step threshold starts an event; jumps closer
// than EVENT_GAP_MS belong to it. The event is a click if it jumped both ways
// (a single step is crosstalk or the release of a press), the level is not held
// up afterwards and no pulse click was already counted for it.
static constexpr float    K_STEP        = 8.0f;
static constexpr float    STEP_MIN_V    = 0.012f;
static constexpr uint32_t EVENT_GAP_MS  = 250;
static constexpr uint32_t SAME_EVENT_MS = 30;     // pulse click this close before a jump = same click

static constexpr float    K_HAND        = 8.0f;
static constexpr float    HAND_MIN_V    = 0.010f;
static constexpr uint32_t HAND_ON_MS    = 700;    // > CLICK_MAX_MS: a long click is no hand
static constexpr uint32_t HAND_OFF_MS   = 1000;
static constexpr uint32_t PRESS_ON_MS   = 140;    // pressed(): fast variant for breathing
static constexpr uint32_t PRESS_OFF_MS  = 200;
// "Hand" that is barely above rest and perfectly still for this long is sensor
// drift: re-learn the rest level. A real hand jitters; a press lifts far more.
static constexpr uint32_t STUCK_MS      = 10000;
static constexpr float    STUCK_MAX_V   = 0.05f;

PressureDetector::PressureDetector(NeffyInterface& iface, VoltageReader& voltage, bool isLeft, bool invert,
                                   float sensitivity)
    : _iface(iface), _voltage(voltage), _isLeft(isLeft), _invert(invert), _thrScale(1.0f / sensitivity) {}

float PressureDetector::readSigned() {
    float v = _voltage.readV();
    return _invert ? -v : v;  // pressing always raises the signal
}

void PressureDetector::init() {
    _smoothed = _local = _rest = _prevSig = readSigned();

    if (_isLeft) {
        _clickMsg.command         = NeffyCommands::PRESSURE_CLICK_L.id;
        _clickMsg.bytes_in_buffer = NeffyCommands::PRESSURE_CLICK_L.payloadLength;
    } else {
        _clickMsg.command         = NeffyCommands::PRESSURE_CLICK_R.id;
        _clickMsg.bytes_in_buffer = NeffyCommands::PRESSURE_CLICK_R.payloadLength;
    }
}

void PressureDetector::setMuted(bool muted) { _muted = muted; }

void PressureDetector::sendClick() {
    _clickPending = true;
    if (!_muted) _iface.writeMessage(_clickMsg);
}

bool PressureDetector::click() {
    if (_clickPending) { _clickPending = false; return true; }
    return false;
}

void PressureDetector::update(unsigned long nowMs) {
    if (nowMs - _prevReadMs < READ_INTERVAL_MS) return;
    _prevReadMs = nowMs;

    float raw  = _lastV = readSigned();
    _smoothed += (raw - _smoothed) * A_SMOOTH;

    // Clicks use the raw signal while no hand rests (quiet; light clicks are only
    // 1-2 samples long and smoothing would halve them), the smoothed one with a hand.
    float sig        = _hand ? _smoothed : raw;
    float step       = sig - _prevSig;      // change since the last reading
    float jitter     = fabsf(step);
    _prevSig         = sig;
    float aboveLocal = sig - _local;

    _clickThr = fmaxf(K_CLICK * _noise, CLICK_MIN_V);
    if (_hand) _clickThr = fmaxf(_clickThr, CLICK_FRAC * _clickAmp);
    _clickThr = fminf(_clickThr * _thrScale, CLICK_MAX_V);

    // --- click: short pulse above the local baseline
    if (!_inPulse) {
        if (aboveLocal > _clickThr) {
            _inPulse      = true;
            _pulseStartMs = nowMs;
            _pulsePeak    = aboveLocal;
        } else {
            if (aboveLocal < 0.0f) _local += aboveLocal * A_LOCAL_DN;  // follow a drop at once
            else                   _local += aboveLocal * A_LOCAL_UP;  // and a rise slowly
            float capped = fminf(jitter, _clickThr); // one big step must not blow up the estimate
            _noise += (capped - _noise) * A_NOISE;
            if (!_hand) _restNoise += (capped - _restNoise) * A_NOISE;
        }
    } else {
        _pulsePeak = fmaxf(_pulsePeak, aboveLocal);
        if (nowMs - _pulseStartMs > CLICK_MAX_MS) {
            _inPulse = false; // long press: let the local baseline follow
            _local   = sig;
        } else if (aboveLocal < _clickThr / 2.0f) {
            _inPulse   = false;
            if (_clickAmp == 0.0f) _clickAmp = _pulsePeak;                             // first click
            else                   _clickAmp += (_pulsePeak - _clickAmp) * A_CLICK_AMP;  // then average
            if (!_anyClick || nowMs - _lastClickMs > PULSE_REPEAT_MS) {
                _pendingClick   = true;
                _pendingSinceMs = nowMs;
                _anyClick       = true;
                _lastClickMs    = nowMs; // also blocks the jump event of this click
            }
            if (_evActive) _evSkip = true;
        }
    }

    // --- click: jump event (light clicks)
    float stepThr = fmaxf(K_STEP * _restNoise, STEP_MIN_V) * _thrScale;
    if (!_hand) {
        if (jitter >= stepThr) {
            if (!_evActive) {
                _evActive  = true;
                _evSkip    = _pressed; // release of a press
                _evUp      = 0.0f;
                _evDn      = 0.0f;
                _evStartMs = nowMs;
            }
            _evLastMs = nowMs;
        }
        if (_evActive) {
            _evUp = fmaxf(_evUp, step);
            _evDn = fmaxf(_evDn, -step);
        }
    }
    if (_evActive && nowMs - _evLastMs >= EVENT_GAP_MS) {
        _evActive = false;
        bool bothWays = fminf(_evUp, _evDn) >= stepThr;
        bool counted  = _anyClick && _lastClickMs + SAME_EVENT_MS >= _evStartMs;  // pulse click got it
        if (bothWays && !_evSkip && !_pressed && !_hand && !counted) {
            _anyClick    = true;
            _lastClickMs = nowMs;
            sendClick();
        }
    }

    // --- hand: sustained rise above the rest level
    float aboveRest       = _smoothed - _rest;
    float handThr = fmaxf(K_HAND * _restNoise, HAND_MIN_V);
    if (_pendingClick) {
        bool back;
        if (_hand) back = sig - _local < _clickThr;  // back to the hand level
        else       back = sig - _rest  < handThr;    // back to the rest level
        if (back) {                                        // back down: a real click
            _pendingClick = false;
            sendClick();
        } else if (nowMs - _pendingSinceMs > CONFIRM_MS) { // stays up: hand or press
            _pendingClick = false;
        }
    }
    if (_smoothed < _rest) {
        _rest += (_smoothed - _rest) * A_REST_DN;
    } else if (!_hand && !_inPulse && aboveRest < handThr) {
        _rest += (_smoothed - _rest) * A_REST_UP;
    }

    // how long the level has been above / below the hand threshold
    if (aboveRest > handThr) { _aboveMs += READ_INTERVAL_MS; _belowMs = 0; }
    else                     { _belowMs += READ_INTERVAL_MS; _aboveMs = 0; }

    if      (!_pressed && _aboveMs >= PRESS_ON_MS)  _pressed = true;
    else if ( _pressed && _belowMs >= PRESS_OFF_MS) _pressed = false;

    if (!_hand && _aboveMs >= HAND_ON_MS) {
        _hand    = true;
        _stillMs = 0;
    } else if (_hand && _belowMs >= HAND_OFF_MS) {
        _hand = false;
    }

    if (_hand) {
        bool drifting = _noise < 2.0f * _restNoise && aboveRest < STUCK_MAX_V;
        if (drifting) _stillMs += READ_INTERVAL_MS;
        else          _stillMs  = 0;
        if (_stillMs >= STUCK_MS) {
            _hand    = false;
            _pressed = false;
            _rest    = _smoothed;
        }
    }
}
