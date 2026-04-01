#pragma once
// ============================================================
//  AccelLink_TX – PIO-UART transmitter
//  Arduino Nano RP2040 Connect → Raspberry Pi Pico
//
//  Wire 1 (dataPin = D18):
//    SerialPIO 8N1 UART at ACCELLINK_BAUD baud (115200).
//    Uses the RP2040 PIO for hardware-timed bit generation –
//    no blocking, no noInterrupts(), no ISR timing issues.
//    Packet (15 bytes):
//      [SYNC1=0x55][SYNC2=0xAA]
//      [X0..X3]  int32_t (value_x × 10000), little-endian
//      [Y0..Y3]  int32_t (value_y × 10000), little-endian
//      [Z0..Z3]  int32_t (value_z × 10000), little-endian
//      [CRC8]    CRC-8/SMBUS over bytes 2–13
//    Timing: 15 bytes × 10 bits / 115200 baud ≈ 1.3 ms << 20 ms limit.
//
//  Wire 2 (calPin = D19):
//    Calibration flag input, idle LOW.
//    A RISING edge (HIGH pulse from the Pico) sets an internal flag.
//    Call getCalFlag() in loop() – returns true once and auto-clears.
//    The Arduino should then re-calibrate its IMU.
// ============================================================

#include <Arduino.h>
#include <SerialPIO.h>

#ifndef ACCELLINK_BAUD
#define ACCELLINK_BAUD 115200UL
#endif

#define ACCELLINK_SYNC1 0x55u
#define ACCELLINK_SYNC2 0xAAu

class AccelLinkTX {
public:
    // Initialize the link.
    // dataPin: PIO UART TX (default D18)
    // calPin:  calibration flag input (default D19)
    void begin(uint8_t dataPin = 18, uint8_t calPin = 19);

    // Returns true after begin() has successfully initialised the PIO UART.
    bool isReady() const { return _serial != nullptr; }

    // Transmit X, Y, Z acceleration values (clamped to [-100.0, +100.0]).
    // Non-blocking: bytes are queued in the PIO FIFO.
    void sendValues(float x, float y, float z);

    // Returns true (and clears the flag) when the Pico has sent a
    // calibration pulse on calPin.  Poll this in loop().
    bool getCalFlag();

    // ── ISR handler – do not call directly ──────────────────
    void _handleCalISR();

private:
    uint8_t _calPin = 19;

    volatile bool _calFlag = false;

    // Storage for in-place SerialPIO construction (avoids heap allocation).
    alignas(SerialPIO) uint8_t _serialBuf[sizeof(SerialPIO)];
    SerialPIO *_serial = nullptr;

    // Encode one float into 4 bytes (int32_t × 10000, little-endian).
    static void _encodeFloat(float v, uint8_t *out4);

    // CRC-8/SMBUS (polynomial 0x07) over len bytes.
    static uint8_t _crc8(const uint8_t *data, uint8_t len);
};

// Global singleton instance (matches arduino-library convention).
extern AccelLinkTX AccelTX;
