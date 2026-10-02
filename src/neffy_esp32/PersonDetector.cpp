#include "PersonDetector.h"

PersonDetector::PersonDetector(NeffyInterface& iface) : _iface(iface) {}

void PersonDetector::init() {
    _sensor.init();
    _personMsg.command         = NeffyCommands::GET_PERSON_READING.id;
    _personMsg.bytes_in_buffer = NeffyCommands::GET_PERSON_READING.payloadLength;
}

PersonState PersonDetector::update() {
    PersonState state = _sensor.update();
    if (state != _lastSent) {
        _lastSent = state;
        sendState();
    }
    return state;
}

void PersonDetector::sendState() {
    _personMsg.buffer[0] = static_cast<uint8_t>(_lastSent);
    _iface.writeMessage(_personMsg);
}
