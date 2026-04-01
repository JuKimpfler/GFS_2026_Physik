// ============================================================
//  AccelLink_RX.cpp – DIY bit-bang UART receiver implementation
//  See AccelLink_RX.h for protocol details.
// ============================================================

#include "AccelLink_RX.h"

AccelLinkRX AccelRX;

// ── Singleton pointer for static ISR wrappers ─────────────────
static AccelLinkRX *_rxInst = nullptr;

static void _dataISRWrapper() { if (_rxInst) _rxInst->_handleDataISR(); }
static void _calISRWrapper()  { if (_rxInst) _rxInst->_handleCalISR();  }

// ── Initialisation ────────────────────────────────────────────
void AccelLinkRX::begin(uint8_t dataPin, uint8_t calPin) {
    _rxInst  = this;
    _dataPin = dataPin;
    _calPin  = calPin;

    pinMode(_dataPin, INPUT);
    pinMode(_calPin,  INPUT);

    attachInterrupt(digitalPinToInterrupt(_dataPin), _dataISRWrapper, FALLING);
    attachInterrupt(digitalPinToInterrupt(_calPin),  _calISRWrapper,  RISING);
}

// ── Public accessors ──────────────────────────────────────────
bool AccelLinkRX::hasNewData() const {
    return _newData;
}

float AccelLinkRX::getX() {
    _newData = false;   // clear flag on first-axis read
    return _valX;
}

float AccelLinkRX::getY() const {
    return _valY;
}

float AccelLinkRX::getZ() const {
    return _valZ;
}

bool AccelLinkRX::getCalFlag() {
    if (!_calFlag) return false;
    _calFlag = false;
    return true;
}

// ── Calibration ISR ───────────────────────────────────────────
void AccelLinkRX::_handleCalISR() {
    _calFlag = true;
}

// ── Data ISR ─────────────────────────────────────────────────
// Receives the complete 15-byte packet with interrupts disabled.
// Maximum blocking time: 15 × 10 × ACCELLINK_BIT_US µs = 600 µs @ 250 kbaud.
void AccelLinkRX::_handleDataISR() {
    // Prevent re-entry while we are busy receiving.
    detachInterrupt(digitalPinToInterrupt(_dataPin));

    noInterrupts();

    uint8_t rxBuf[15];
    bool    valid = true;

    // ── Byte 0: ISR fired at its falling edge ────────────────
    uint32_t edgeTime = micros();   // ≈ time of the falling edge
    rxBuf[0] = _receiveByte(_dataPin, edgeTime, valid);

    // ── Bytes 1–14: detect falling edge of each start bit ────
    for (uint8_t idx = 1; idx < 15 && valid; idx++) {
        uint32_t deadline = micros() + (uint32_t)ACCELLINK_BYTE_GAP_US;

        // Wait for stop bit of previous byte (pin goes HIGH)
        while (!digitalRead(_dataPin)) {
            if (micros() > deadline) { valid = false; break; }
        }
        if (!valid) break;

        // Wait for falling edge of next start bit (pin goes LOW)
        while (digitalRead(_dataPin)) {
            if (micros() > deadline) { valid = false; break; }
        }
        if (!valid) break;

        edgeTime = micros();    // ≈ start of this start bit
        rxBuf[idx] = _receiveByte(_dataPin, edgeTime, valid);
    }

    interrupts();

    // Check calibration pin level in case the RISING edge was missed
    // during the noInterrupts() window (pulse is >>600 µs so still HIGH).
    if (digitalRead(_calPin)) {
        _calFlag = true;
    }

    // ── Validate sync bytes and CRC ──────────────────────────
    if (valid
            && rxBuf[0] == ACCELLINK_SYNC1
            && rxBuf[1] == ACCELLINK_SYNC2
            && _crc8(&rxBuf[2], 12) == rxBuf[14]) {
        _valX    = _decodeFloat(&rxBuf[2]);
        _valY    = _decodeFloat(&rxBuf[6]);
        _valZ    = _decodeFloat(&rxBuf[10]);
        _newData = true;
    }
    // On sync / CRC mismatch the packet is silently discarded;
    // the next FALLING edge will re-trigger reception.

    // Re-arm interrupt for next packet.
    attachInterrupt(digitalPinToInterrupt(_dataPin), _dataISRWrapper, FALLING);
}

// ── Private: receive one byte ─────────────────────────────────
// Called with interrupts already disabled.
// edgeTime: micros() value at/near the falling edge of the start bit.
uint8_t AccelLinkRX::_receiveByte(uint8_t pin, uint32_t edgeTime, bool &valid) {
    uint8_t b = 0;
    for (uint8_t bit = 0; bit < 8; bit++) {
        // Sample point: centre of each data bit.
        // Offset from start-bit falling edge:
        //   skip start bit (1 × BIT_US) + advance to bit centre (+ ½ BIT_US)
        //   + bit index × BIT_US
        uint32_t sampleAt = edgeTime
                            + (uint32_t)ACCELLINK_BIT_US                    // start bit
                            + ((uint32_t)ACCELLINK_BIT_US >> 1u)            // ½ bit to centre
                            + (uint32_t)ACCELLINK_BIT_US * (uint32_t)bit;   // bit offset

        // Busy-wait until the sample point is reached.
        // micros() has 1 µs resolution; exit as soon as timer ≥ sampleAt.
        while (micros() < sampleAt);

        if (digitalRead(pin)) {
            b |= (1u << bit);
        }
    }

    // Sanity-check: stop bit should be HIGH.  If not, framing error.
    uint32_t stopSample = edgeTime
                          + (uint32_t)ACCELLINK_BIT_US * 9u  // 1 start + 8 data + ½
                          + ((uint32_t)ACCELLINK_BIT_US >> 1u);
    while (micros() < stopSample);
    if (!digitalRead(pin)) {
        valid = false;
    }

    return b;
}

// ── Private: decode 4-byte payload to float ───────────────────
float AccelLinkRX::_decodeFloat(const uint8_t *in4) {
    int32_t ival = (int32_t)(  (uint32_t)in4[0]
                              | ((uint32_t)in4[1] <<  8)
                              | ((uint32_t)in4[2] << 16)
                              | ((uint32_t)in4[3] << 24));
    return ival / 10000.0f;
}

// ── Private: CRC-8/SMBUS ──────────────────────────────────────
uint8_t AccelLinkRX::_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00u;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80u)
                      ? (uint8_t)((crc << 1u) ^ 0x07u)
                      : (uint8_t)(crc << 1u);
        }
    }
    return crc;
}
