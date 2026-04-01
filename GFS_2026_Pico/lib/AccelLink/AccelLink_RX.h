#pragma once
// ============================================================
//  AccelLink_RX -- PIO-UART receiver
//  Raspberry Pi Pico <- Arduino Nano RP2040 Connect
//
//  Wire 1 (dataPin = GPIO2):
//    SerialPIO 8N1 UART at ACCELLINK_BAUD baud (115200).
//    Uses the RP2040 PIO for hardware-timed bit sampling on any
//    GPIO pin -- no ISR timing issues, no noInterrupts() blocking.
//    Call update() frequently in loop() to drain the PIO RX FIFO
//    and run the packet parser state machine.
//
//    Packet format:
//      [0x55][0xAA][X0..X3][Y0..Y3][Z0..Z3][CRC8]  (15 bytes)
//    Self-synchronising: bytes that do not form a valid header or
//    fail the CRC are discarded and the parser re-syncs.
//
//  Wire 2 (calPin = GPIO3):
//    Calibration flag output, idle LOW.
//    Call triggerCalibration() to pulse the pin HIGH for a
//    configurable duration.  The Arduino detects the RISING edge
//    and re-calibrates its IMU.
// ============================================================

#include <Arduino.h>
#include <SerialPIO.h>

#ifndef ACCELLINK_BAUD
#define ACCELLINK_BAUD 115200UL
#endif

#define ACCELLINK_SYNC1      0x55u
#define ACCELLINK_SYNC2      0xAAu
#define ACCELLINK_PACKET_LEN 15

class AccelLinkRX {
public:
    // Attach PIO UART and configure pins.
    // dataPin: PIO UART RX (default GPIO2)
    // calPin:  calibration flag output (default GPIO3)
    void begin(uint8_t dataPin = 2, uint8_t calPin = 3);

    // Returns true after begin() has successfully initialised the PIO UART.
    bool isReady() const { return _serial != nullptr; }

    // Drain the PIO RX FIFO and advance the packet parser.
    // Must be called frequently (every loop iteration).
    void update();

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

private:
    uint8_t _calPin = 3;

    // Storage for in-place SerialPIO construction (avoids heap allocation).
    alignas(SerialPIO) uint8_t _serialBuf[sizeof(SerialPIO)];
    SerialPIO *_serial = nullptr;

    // Packet parser state machine
    uint8_t _rxBuf[ACCELLINK_PACKET_LEN];
    uint8_t _rxIdx = 0;

    float _valX    = 0.0f;
    float _valY    = 0.0f;
    float _valZ    = 0.0f;
    bool  _newData = false;

    // Decode 4-byte little-endian int32_t payload back to float.
    static float _decodeFloat(const uint8_t *in4);

    // CRC-8/SMBUS (polynomial 0x07) over len bytes.
    static uint8_t _crc8(const uint8_t *data, uint8_t len);
};

// Global singleton instance.
extern AccelLinkRX AccelRX;
