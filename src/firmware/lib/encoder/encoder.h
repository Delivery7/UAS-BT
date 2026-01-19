#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB, int ppr);

    void begin();
    void update();  // dipanggil dari ISR
    long getPulses();
    float getRPM(unsigned long interval_ms);
    void reset();

private:
    uint8_t _pinA, _pinB;
    volatile long _pulseCount;
    int _ppr;  // pulses per revolution
    unsigned long _lastReadTime;
    long _lastPulseCount;
};

#endif
