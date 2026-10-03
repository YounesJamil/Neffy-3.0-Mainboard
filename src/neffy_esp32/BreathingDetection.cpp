#include "BreathingDetection.h"

static constexpr uint32_t SESSION_MS = 30000;

// Adults breathe 12-20 times per minute at rest (6-10 per 30 s). Slower is
// relaxed breathing (exercises, meditation), faster is typical for stress.
// Bounds are on the rate scaled to 30 s, so they sit between whole breaths.
static constexpr float RELAXED_MAX_PER_30S = 5.5f;  // <= 5 breaths  -> relaxed
static constexpr float NORMAL_MAX_PER_30S  = 10.5f; // 6..10 breaths -> normal, more -> stressed

BreathingDetection::BreathingDetection(NeffyInterface& iface) : _iface(iface) {}

void BreathingDetection::start(unsigned long nowMs) {
    _running       = true;
    _wasPressed    = true; // a press already going on at the start is no inhale
    _startMs       = nowMs;
    _inhales       = 0;
    _result        = BreathingResult::NONE;
    _breathsPer30s = 0.0f;
}

bool BreathingDetection::update(unsigned long nowMs, bool pressed) {
    if (!_running) return false;

    if (pressed && !_wasPressed) {        // press starts = inhale starts
        if (_inhales == 0) _firstInhaleMs = nowMs;
        _lastInhaleMs = nowMs;
        _inhales++;
    }
    _wasPressed = pressed;

    if (nowMs - _startMs < SESSION_MS) return false;
    _running = false;

    if (_inhales < 2) return true; // no rate without two inhales: send nothing

    // n inhales = n - 1 full breaths between the first and the last one
    float meanCycleMs = (float)(_lastInhaleMs - _firstInhaleMs) / (_inhales - 1);
    _breathsPer30s    = SESSION_MS / meanCycleMs;  // e.g. one breath per 6 s -> 5 per 30 s

    MessageMetadata cmd;
    if (_breathsPer30s <= RELAXED_MAX_PER_30S) {
        _result = BreathingResult::RELAXED;
        cmd     = NeffyCommands::BREATHING_DETECTION_RELAXED;
    } else if (_breathsPer30s <= NORMAL_MAX_PER_30S) {
        _result = BreathingResult::NORMAL;
        cmd     = NeffyCommands::BREATHING_DETECTION_NORMAL;
    } else {
        _result = BreathingResult::STRESSED;
        cmd     = NeffyCommands::BREATHING_DETECTION_STRESSED;
    }

    Message msg;
    msg.command         = cmd.id;
    msg.bytes_in_buffer = cmd.payloadLength;
    msg.buffer[0]       = 0x00;
    _iface.writeMessage(msg);
    return true;
}
