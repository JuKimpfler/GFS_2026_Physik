#pragma once
#include <Arduino.h>

// ── Timing Sender (µs) ───────────────────────────────────────────────────
#define SL_SYNC_HIGH   400
#define SL_ONE_HIGH    160
#define SL_ZERO_HIGH    60
#define SL_GAP          80

// ── Schwellen Empfänger (µs) ─────────────────────────────────────────────
#define SL_SYNC_MIN    250   // > 250 µs  →  SYNC
#define SL_ONE_MIN     100   // > 100 µs  →  Bit = 1

// ── Datenbits ────────────────────────────────────────────────────────────
#define SL_BITS         15   // 2^15 = 32768 > 20001 Stufen


class SimpleLinkSender {
public:
    explicit SimpleLinkSender(uint8_t pin);
    void begin();
    bool send(float value);

private:
    uint8_t _pin;
};


// ══════════════════════════════════════════════════════════════════════════
//  EMPFÄNGER  (Raspberry Pi Pico)
// ══════════════════════════════════════════════════════════════════════════
class SimpleLinkReceiver {
public:
    explicit SimpleLinkReceiver(uint8_t pin);
    void begin();
    bool receive();

    float getValue() const;
    bool  isValid()  const;

private:
    uint8_t  _pin;

    int last_Hight;

    float    _value;
    bool     _valid;
};
