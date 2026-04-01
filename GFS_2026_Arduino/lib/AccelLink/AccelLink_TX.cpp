// ============================================================
//  AccelLink_TX.cpp – DIY bit-bang UART transmitter implementation
//  See AccelLink_TX.h for protocol details.
// ============================================================

#include "AccelLink_TX.h"
#include <math.h>

AccelLinkTX AccelTX;

// ── Initialisation ────────────────────────────────────────────
void AccelLinkTX::begin(uint8_t dataPin, uint8_t calPin) {
    _dataPin = dataPin;
    _calPin  = calPin;
    pinMode(_dataPin, OUTPUT);
    digitalWrite(_dataPin, HIGH);   // idle HIGH (UART convention)
    pinMode(_calPin, OUTPUT);
    digitalWrite(_calPin, LOW);     // calibration flag idle LOW
}

// ── Public: send 3-axis packet ────────────────────────────────
void AccelLinkTX::sendValues(float x, float y, float z) {
    // Build 12-byte payload (3 × int32_t little-endian)
    uint8_t payload[12];
    _encodeFloat(x, &payload[0]);
    _encodeFloat(y, &payload[4]);
    _encodeFloat(z, &payload[8]);

    uint8_t crc = _crc8(payload, 12);

    // Transmit: [SYNC1][SYNC2][payload × 12][CRC8]  — 15 bytes total
    _sendByte(ACCELLINK_SYNC1);
    _sendByte(ACCELLINK_SYNC2);
    for (uint8_t i = 0; i < 12; i++) {
        _sendByte(payload[i]);
    }
    _sendByte(crc);
}

// ── Public: calibration flag ──────────────────────────────────
void AccelLinkTX::triggerCalibration(uint32_t pulseDurationMs) {
    digitalWrite(_calPin, HIGH);
    delay(pulseDurationMs);
    digitalWrite(_calPin, LOW);
}

// ── Private: send one byte as 8N1 ────────────────────────────
void AccelLinkTX::_sendByte(uint8_t b) {
    // Start bit (LOW)
    digitalWrite(_dataPin, LOW);
    delayMicroseconds(ACCELLINK_BIT_US);

    // 8 data bits, LSB first
    for (uint8_t i = 0; i < 8; i++) {
        digitalWrite(_dataPin, (b >> i) & 1u ? HIGH : LOW);
        delayMicroseconds(ACCELLINK_BIT_US);
    }

    // Stop bit (HIGH) – restore idle state
    digitalWrite(_dataPin, HIGH);
    delayMicroseconds(ACCELLINK_BIT_US);
}

// ── Private: encode float → 4 bytes ──────────────────────────
void AccelLinkTX::_encodeFloat(float v, uint8_t *out4) {
    // Clamp to transmission range
    if (v >  100.0f) v =  100.0f;
    if (v < -100.0f) v = -100.0f;

    // Scale to fixed-point (4 decimal places) and round
    int32_t ival = (int32_t)roundf(v * 10000.0f);

    // Little-endian byte order
    out4[0] = (uint8_t)( ival        & 0xFF);
    out4[1] = (uint8_t)((ival >>  8) & 0xFF);
    out4[2] = (uint8_t)((ival >> 16) & 0xFF);
    out4[3] = (uint8_t)((ival >> 24) & 0xFF);
}

// ── Private: CRC-8/SMBUS ──────────────────────────────────────
uint8_t AccelLinkTX::_crc8(const uint8_t *data, uint8_t len) {
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
