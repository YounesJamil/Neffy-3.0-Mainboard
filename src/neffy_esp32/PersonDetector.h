#pragma once
#include "Neffy_Interface.h"
#include "PersonSensor.h"

/**
 * Wraps PersonSensor and sends NeffyCommands::GET_PERSON_READING over UART
 * whenever the detected state (NONE = 0 / PRESENT = 1) changes, and on demand
 * via sendState() so a lost message does not go unnoticed.
 */
class PersonDetector {
public:
    /** @param iface Reference to the UART interface */
    PersonDetector(NeffyInterface& iface);

    /** Initialises the underlying sensor and the outgoing UART message. Call once in setup(). */
    void init();

    /**
     * Reads the sensor and sends an update over UART on state change. Call every loop iteration.
     * @return The current PersonState.
     */
    PersonState update();

    /** Sends the current state over UART again, whether it changed or not. */
    void sendState();

    /** Last measured temperature and learned baseline, in degrees Celsius. */
    float temperature() const { return _sensor.lastTemp(); }
    float baseline() const    { return _sensor.baseline(); }
    float ptat() const        { return _sensor.lastPtat(); }

private:
    NeffyInterface& _iface;
    PersonSensor    _sensor;
    Message         _personMsg;
    PersonState     _lastSent = PersonState::NONE;
};
