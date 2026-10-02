#pragma once
#include "Neffy_Interface.h"

enum class BreathingResult : uint8_t { NONE, RELAXED, NORMAL, STRESSED };

/**
 * 30 s breathing detection, started by BREATHING_DETECTION_START from the other board.
 * The user presses the pockets while inhaling and lets go while exhaling. The
 * breathing rate is taken from the mean time between inhales (more precise than
 * counting whole breaths in such a short window), classified and sent as
 * BREATHING_DETECTION_RELAXED / _NORMAL / _STRESSED. With fewer than two inhales nothing
 * is sent.
 */
class BreathingDetection {
public:
    explicit BreathingDetection(NeffyInterface& iface);

    /** Starts (or restarts) a session. */
    void start(unsigned long nowMs);

    /**
     * Feeds the current pocket state. Call every loop iteration while running().
     * @param pressed true while the user presses a pocket (inhaling)
     * @return true once, when the session has just finished
     */
    bool update(unsigned long nowMs, bool pressed);

    bool            running() const { return _running; }
    BreathingResult result()  const { return _result; }
    /** Breathing rate of the last session, scaled to breaths per 30 s. */
    float           breathsPer30s() const { return _breathsPer30s; }

private:
    NeffyInterface& _iface;

    bool            _running       = false;
    bool            _wasPressed    = false;
    unsigned long   _startMs       = 0;
    unsigned long   _firstInhaleMs = 0;
    unsigned long   _lastInhaleMs  = 0;
    uint16_t        _inhales       = 0;
    BreathingResult _result        = BreathingResult::NONE;
    float           _breathsPer30s = 0.0f;
};
