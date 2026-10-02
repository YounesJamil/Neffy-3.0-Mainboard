#pragma once
#include "Neffy_Interface.h"
#include "VoltageReader.h"

/**
 * Detects clicks on a velostat pocket and sends PRESSURE_CLICK_L/R. Self-
 * calibrating: the sensors drift with use and time, so nothing is tuned per
 * pocket. The detector keeps learning the rest level, the signal jitter and the
 * typical click height, and derives its thresholds from them.
 *
 *  - click: short pulse (<= 600 ms) above a local baseline, or a fast jump both
 *    ways (light clicks, only without a hand on the pocket)
 *  - a resting hand (sustained rise above the rest level) is tracked internally
 *    only: clicks under it need to be clearer
 */
class PressureDetector {
public:
    /**
     * @param iface   Reference to the UART interface
     * @param voltage VoltageReader for the pocket pin (only readV() is used)
     * @param isLeft  true = left pocket (sends PRESSURE_CLICK_L), false = right pocket (_R)
     * @param invert  true if pressing lowers the voltage (pocket assembled the other way round)
     * @param sensitivity 1 = as calibrated; below 1 clicks must be stronger (thresholds / sensitivity)
     */
    PressureDetector(NeffyInterface& iface, VoltageReader& voltage, bool isLeft, bool invert = false,
                     float sensitivity = 1.0f);

    /** Seeds the baselines. Call once after VoltageReader::init(). */
    void init();

    /**
     * Reads the sensor and updates detection state. Call every loop iteration.
     * @param nowMs Current time from millis().
     */
    void update(unsigned long nowMs);

    /**
     * Returns true while the pocket is pressed (reacts within ~150 ms, does not
     * tell clicks apart). Used for breathing, where a fast inhale can be shorter
     * than a second.
     */
    bool pressed() const { return _pressed; }

    /**
     * Stops (true) or resumes (false) the click UART messages; detection keeps
     * running.
     */
    void setMuted(bool muted);

    /** Returns true and consumes the event if a click was detected since last call. */
    bool click();

    /** Last raw reading in volts (for the 'D' debug stream). */
    float lastVoltage() const { return _invert ? -_lastV : _lastV; }

private:
    float readSigned();
    void  sendClick();

    NeffyInterface& _iface;
    VoltageReader&  _voltage;
    bool            _isLeft;
    bool            _invert;
    float           _thrScale;  // 1 / sensitivity
    Message         _clickMsg;

    float _lastV     = 0.0f; // last raw reading (signed)
    float _x         = 0.0f; // smoothed signal
    float _prevC     = 0.0f; // previous click signal (raw or smoothed, see update())
    float _local     = 0.0f; // local baseline, clicks are measured against it
    float _rest      = 0.0f; // rest level without hand
    float _noise     = 0.001f; // jitter, always
    float _restNoise = 0.001f; // jitter without hand
    float _clickAmp  = 0.0f; // learned typical click height, 0 = none yet
    float _clickThr  = 0.0f;

    bool          _inPulse      = false;
    unsigned long _pulseStartMs = 0;
    float         _pulsePeak    = 0.0f;
    bool          _clickPending = false;
    bool          _pendingClick   = false; // pulse click waiting for the level to drop
    unsigned long _pendingSinceMs = 0;
    bool          _anyClick       = false;
    unsigned long _lastClickMs    = 0;

    bool          _evActive  = false;  // jump event in progress
    bool          _evSkip    = false;
    unsigned long _evStartMs = 0;
    unsigned long _evLastMs  = 0;
    float         _evUp      = 0.0f;   // biggest jump up / down in the event
    float         _evDn      = 0.0f;

    bool          _muted    = false;
    bool          _pressed  = false;
    bool          _hand     = false;
    unsigned long _aboveMs  = 0;
    unsigned long _belowMs  = 0;
    unsigned long _stillMs  = 0;

    unsigned long _prevReadMs = 0;
};
