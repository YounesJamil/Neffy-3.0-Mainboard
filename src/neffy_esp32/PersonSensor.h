#pragma once
#include <Arduino.h>

// Values are the GET_PERSON_READING payload: the other board only knows 1 = someone there.
enum class PersonState : uint8_t { NONE = 0, PRESENT = 1 };

/**
 * Combines the single-pixel D6T thermal sensor with person detection logic.
 * Reads the temperature via I2C and reports PRESENT while it is clearly warmer
 * than the learned room temperature, NONE otherwise. The threshold follows the
 * sensor noise and the room temperature is learned slowly (see PersonSensor.cpp).
 */
class PersonSensor {
public:
    /** Initialises I2C and seeds the room temperature (refined during the first 3 s). */
    void init();

    /**
     * Reads the temperature and updates the detection. Call every PERSON_READ_MS (not faster).
     * @return Current PersonState
     */
    PersonState update();

    /** Returns the raw temperature from the last update() call in degrees Celsius. */
    float lastTemp() const { return _lastTemp; }

    /** Returns the sensor's own (package) temperature from the last update() call. */
    float lastPtat() const { return _lastPtat; }

    /** Returns the learned ambient baseline in degrees Celsius. */
    float baseline() const { return _baseline; }

private:
    float         _baseline       = 0.0f;
    float         _lastTemp       = 0.0f;
    float         _lastPtat       = 0.0f;
    float         _prevTemp       = 0.0f;
    float         _noise          = 0.1f;   // reading-to-reading jitter while nobody is there
    bool          _present        = false;
    bool          _roomGuessed    = false; // room temperature guessed from PTAT at power-up
    float         _presentMin     = 0.0f;  // lowest reading since someone came
    unsigned long _lastMs         = 0;
    unsigned long _startMs        = 0;
    unsigned long _restartMs      = 0;     // sensor sent 0.0 C (restarted) at this time
    bool          _restarting     = false; // skipping its first readings after a restart
    unsigned long _peakStartMs    = 0;     // start of the current peak bucket
    float         _peak           = 0.0f;  // highest reading in the current bucket
    float         _prevPeak       = 0.0f;  // ... and in the one before

    float readTemp();
};
