// ============================================================
//  AccelLink_RX.cpp -- PIO-UART receiver implementation
//  See AccelLink_RX.h for protocol details.
// ============================================================

#include "AccelLink_RX.h"
#include <new>

AccelLinkRX AccelRX;

// ── Initialisation ────────────────────────────────────────────
void AccelLinkRX::begin(uint8_t dataPin, uint8_t calPin) {
    _calPin = calPin;
    _rxIdx  = 0;

    // Construct SerialPIO in pre-allocated buffer (RX only, no TX pin needed).
    _serial = new(_serialBuf) SerialPIO(SerialPIO::NOPIN, dataPin);
    _serial->begin(ACCELLINK_BAUD);

    pinMode(_calPin, OUTPUT);
    digitalWrite(_calPin, LOW);     // calibration flag idle LOW
}

// ── Public: drain PIO FIFO and parse packets ──────────────────
void AccelLinkRX::update() {
    if (!_serial) return;

    while (_serial->available()) {
        uint8_t b = (uint8_t)_serial->read();

        if (_rxIdx == 0) {
            // Waiting for SYNC1
            if (b == ACCELLINK_SYNC1) {
                _rxBuf[_rxIdx++] = b;
            }
        } else if (_rxIdx == 1) {
            // Waiting for SYNC2
            if (b == ACCELLINK_SYNC2) {
                _rxBuf[_rxIdx++] = b;
            } else if (b == ACCELLINK_SYNC1) {
                // Consecutive SYNC1 -- keep index at 1, overwrite slot 0
                _rxBuf[0] = b;
            } else {
                // Bad byte -- reset
                _rxIdx = 0;
            }
        } else {
            // Collecting payload + CRC
            _rxBuf[_rxIdx++] = b;

            if (_rxIdx == ACCELLINK_PACKET_LEN) {
                _rxIdx = 0;

                // Validate CRC over the 12 payload bytes [2..13]
                if (_crc8(&_rxBuf[2], 12) == _rxBuf[14]) {
                    _valX    = _decodeFloat(&_rxBuf[2]);
                    _valY    = _decodeFloat(&_rxBuf[6]);
                    _valZ    = _decodeFloat(&_rxBuf[10]);
                    _newData = true;
                }
                // On CRC mismatch the packet is silently discarded;
                // the parser will re-sync on the next SYNC1 byte.
            }
        }
    }
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

// ── Public: trigger calibration on Arduino ────────────────────
void AccelLinkRX::triggerCalibration(uint32_t pulseDurationMs) {
    digitalWrite(_calPin, HIGH);
    delay(pulseDurationMs);
    digitalWrite(_calPin, LOW);
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
