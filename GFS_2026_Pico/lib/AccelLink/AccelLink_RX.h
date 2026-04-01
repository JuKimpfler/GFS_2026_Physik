#pragma once
// ============================================================
//  AccelLink_RX – DIY bit-bang UART receiver
//  Raspberry Pi Pico ← Arduino Nano RP2040 Connect
//
//  Wire 1 (dataPin = GPIO2):
//    Interrupt-driven bit-bang 8N1 receiver.
//    A FALLING edge on the data pin triggers an ISR that
//    receives the complete 15-byte packet with interrupts
//    disabled (~600 µs).  Absolute-time sampling via micros()
//    eliminates cumulative bit-timing drift.
//
//    Packet format (see AccelLink_TX.h for full details):
//      [0x55][0xAA][X0..X3][Y0..Y3][Z0..Z3][CRC8]
//    Self-synchronising: wrong sync bytes → packet discarded,
//    interrupt re-armed immediately for the next attempt.
//
//  Wire 2 (calPin = GPIO3):
//    Calibration flag output, idle LOW.
//    Call triggerCalibration() to pulse the pin HIGH for a
//    configurable duration.  The Arduino detects the RISING edge
//    and re-calibrates its IMU.
// ============================================================

#include <Arduino.h>

#ifndef ACCELLINK_BIT_US
#define ACCELLINK_BIT_US 4u
#endif

#define ACCELLINK_SYNC1 0x55u
#define ACCELLINK_SYNC2 0xAAu

// Maximum time to wait for inter-byte stop + start bit sequence (µs).
// Must be larger than 2 × ACCELLINK_BIT_US.  5× gives generous margin.
#define ACCELLINK_BYTE_GAP_US (ACCELLINK_BIT_US * 5u)

class AccelLinkRX {
public:
    // Attach interrupts and configure pins.
    // dataPin: bit-bang UART RX (default GPIO2)
    // calPin:  calibration flag output (default GPIO3)
    void begin(uint8_t dataPin = 2, uint8_t calPin = 3);

    // Returns true when a new, CRC-validated packet has been received.
    bool hasNewData() const;

    // Return the latest X, Y, Z values (call after hasNewData()).
    // hasNewData() resets to false after getX() is called.
    float getX();
    float getY() const;
    float getZ() const;

    // Pulse the calibration flag pin HIGH for pulseDurationMs milliseconds,
    // then return it to LOW.  The Arduino will detect the RISING edge and
    // re-calibrate its IMU.
    void triggerCalibration(uint32_t pulseDurationMs = 10);

    // ── ISR handler – do not call directly ──────────────────
    void _handleDataISR();

private:
    uint8_t _dataPin = 2;
    uint8_t _calPin  = 3;

    volatile float _valX    = 0.0f;
    volatile float _valY    = 0.0f;
    volatile float _valZ    = 0.0f;
    volatile bool  _newData = false;

    // Receive one byte: waits for falling edge (start bit) then samples 8 bits.
    // edgeTime: µs timestamp of the falling edge (from micros()).
    // Returns decoded byte; sets valid=false on timeout.
    static uint8_t _receiveByte(uint8_t pin, uint32_t edgeTime, bool &valid);

    // Decode 4-byte little-endian int32_t payload back to float.
    static float _decodeFloat(const uint8_t *in4);

    // CRC-8/SMBUS (polynomial 0x07) over len bytes.
    static uint8_t _crc8(const uint8_t *data, uint8_t len);
};

// Global singleton instance.
extern AccelLinkRX AccelRX;
