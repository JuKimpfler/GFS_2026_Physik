// ============================================================
//  AccelLink_TX.cpp – PIO-UART transmitter implementation
//  See AccelLink_TX.h for protocol details.
// ============================================================

#include "AccelLink_TX.h"
#include <math.h>
#include <new>

AccelLinkTX AccelTX;

// ── Singleton pointer for static ISR wrapper ──────────────────
static AccelLinkTX *_txInst = nullptr;

static void _calISRWrapper() { if (_txInst) _txInst->_handleCalISR(); }

// ── Initialisation ────────────────────────────────────────────
void AccelLinkTX::begin(uint8_t dataPin, uint8_t calPin) {
    _txInst  = this;
    _calPin  = calPin;

    // Construct SerialPIO in pre-allocated buffer (TX only, no RX pin).
    _serial = new(_serialBuf) SerialPIO(dataPin, SerialPIO::NOPIN);
    _serial->begin(ACCELLINK_BAUD);

    pinMode(_calPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_calPin), _calISRWrapper, RISING);
}

// ── Public: send 3-axis packet ────────────────────────────────
void AccelLinkTX::sendValues(float x, float y, float z) {
    if (!_serial) return;

    // Build 12-byte payload (3 × int32_t little-endian)
    uint8_t payload[12];
    _encodeFloat(x, &payload[0]);
    _encodeFloat(y, &payload[4]);
    _encodeFloat(z, &payload[8]);

    uint8_t crc = _crc8(payload, 12);

    // Assemble and transmit: [SYNC1][SYNC2][payload × 12][CRC8] — 15 bytes total
    uint8_t packet[15];
    packet[0] = ACCELLINK_SYNC1;
    packet[1] = ACCELLINK_SYNC2;
    memcpy(&packet[2], payload, 12);
    packet[14] = crc;

    _serial->write(packet, 15);
}

// ── Public: calibration flag ──────────────────────────────────
bool AccelLinkTX::getCalFlag() {
    if (!_calFlag) return false;
    _calFlag = false;
    return true;
}

// ── Calibration ISR ───────────────────────────────────────────
void AccelLinkTX::_handleCalISR() {
    _calFlag = true;
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
