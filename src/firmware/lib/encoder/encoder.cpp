#include "encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB, int ppr)
    : _pinA(pinA), _pinB(pinB), _ppr(ppr), _pulseCount(0),
      _lastReadTime(0), _lastPulseCount(0) {}

void Encoder::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    _pulseCount = 0;
}

void Encoder::update() {
    int stateA = digitalRead(_pinA);
    int stateB = digitalRead(_pinB);

    if (stateA == stateB)
        _pulseCount++;
    else
        _pulseCount--;
}

long Encoder::getPulses() {
    return _pulseCount;
}

float Encoder::getRPM(unsigned long interval_ms) {
    unsigned long now = millis();
    long pulseDiff = _pulseCount - _lastPulseCount;
    float rpm = (pulseDiff * 60000.0) / (_ppr * interval_ms);

    _lastPulseCount = _pulseCount;
    _lastReadTime = now;

    return rpm;
}

void Encoder::reset() {
    _pulseCount = 0;
    _lastPulseCount = 0;
}
