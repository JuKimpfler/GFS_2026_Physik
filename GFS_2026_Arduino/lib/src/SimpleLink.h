#pragma once
#include <Arduino.h>

class SimpleLinkSender {
public:
    explicit SimpleLinkSender(uint8_t pin);
    void begin();
    bool send(float value);

private:
    uint8_t _pin;
    int Faktor = 300;
};
