#pragma once
// ============================================================
//  AccelLink_TX – DIY bit-bang UART transmitter
//  Arduino Nano RP2040 Connect → Raspberry Pi Pico
//
//  Wire 1 (dataPin = D18):
//    Bit-bang 8N1 UART, idle HIGH, start bit LOW.
//    Baud rate: ACCELLINK_BIT_US µs / bit (default 4 µs → 250 kbaud).
//    Packet (15 bytes):
//      [SYNC1=0x55][SYNC2=0xAA]
//      [X0..X3]  int32_t (value_x × 10000), little-endian
//      [Y0..Y3]  int32_t (value_y × 10000), little-endian
//      [Z0..Z3]  int32_t (value_z × 10000), little-endian
//      [CRC8]    CRC-8/SMBUS over bytes 2–13
//    Timing: 15 bytes × 10 bits × 4 µs = 600 µs << 4 ms limit.
//
//  Wire 2 (calPin = D19):
//    Calibration flag output, idle LOW.
//    HIGH pulse (default 10 ms) signals the Pico to re-tare its HX711.
// ============================================================

#include <Arduino.h>

// Bit period in microseconds. Decrease to raise baud rate.
// 4 µs → 250 kbaud; safe range on RP2040: 3–10 µs.
#ifndef ACCELLINK_BIT_US
#define ACCELLINK_BIT_US 4u
#endif

#define ACCELLINK_SYNC1 0x55u
#define ACCELLINK_SYNC2 0xAAu

class AccelLinkTX {
public:
    // Initialize the two pins.
    // dataPin: bit-bang UART TX (default D18)
    // calPin:  calibration flag output (default D19)
    void begin(uint8_t dataPin = 18, uint8_t calPin = 19);

    // Transmit X, Y, Z acceleration values clamped to [-100.0, +100.0].
    // Blocks for ~600 µs while transmitting the 15-byte packet.
    void sendValues(float x, float y, float z);

    // Pulse the calibration flag pin HIGH for pulseDurationMs milliseconds,
    // then return it to LOW.  The Pico should re-tare / calibrate upon receipt.
    void triggerCalibration(uint32_t pulseDurationMs = 10);

private:
    uint8_t _dataPin = 18;
    uint8_t _calPin  = 19;

    // Transmit one byte as 8N1 (start bit + 8 data bits LSB-first + stop bit).
    void _sendByte(uint8_t b);

    // Encode one float into 4 bytes (int32_t × 10000, little-endian).
    static void _encodeFloat(float v, uint8_t *out4);

    // CRC-8/SMBUS (polynomial 0x07) over len bytes.
    static uint8_t _crc8(const uint8_t *data, uint8_t len);
};

// Global singleton instance (matches arduino-library convention).
extern AccelLinkTX AccelTX;
