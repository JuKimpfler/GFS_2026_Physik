#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include "AccelLink_TX.h"

// ── Pin configuration ─────────────────────────────────────────
// D18: bit-bang UART data line → Pico GPIO2
// D19: calibration flag output → Pico GPIO3
#define ACCELLINK_DATA_PIN 18
#define ACCELLINK_CAL_PIN  19

// Send IMU data every 20 ms (50 Hz)
#define SEND_INTERVAL_MS 20

// ── Calibration offsets ───────────────────────────────────────
static float offsetX = 0.0f;
static float offsetY = 0.0f;
static float offsetZ = 0.0f;

// Average the IMU over N samples to find the resting offset.
// Afterwards pulses the calibration flag so the Pico re-tares its HX711.
static void calibrateIMU() {
    const uint8_t N = 50;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;

    Serial.println("Kalibrierung... Sensor bitte ruhig halten.");
    for (uint8_t i = 0; i < N; i++) {
        float x, y, z;
        while (!IMU.accelerationAvailable());
        IMU.readAcceleration(x, y, z);
        sx += x;  sy += y;  sz += z;
        delay(10);
    }
    offsetX =  sx / N;
    offsetY =  sy / N;
    offsetZ = (sz / N) - 1.0f;   // remove 1 g gravity on Z

    Serial.print("Offsets  X=");  Serial.print(offsetX, 4);
    Serial.print("  Y=");         Serial.print(offsetY, 4);
    Serial.print("  Z=");         Serial.println(offsetZ, 4);

    // Signal Pico to re-tare its HX711 at the same moment
    AccelTX.triggerCalibration(10);
}

// ── Arduino setup ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    // Initialise the AccelLink transmitter
    AccelTX.begin(ACCELLINK_DATA_PIN, ACCELLINK_CAL_PIN);

    // Initialise the IMU
    if (!IMU.begin()) {
        Serial.println("FEHLER: LSM6DSOX konnte nicht initialisiert werden!");
        while (true) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }

    Serial.println("IMU bereit.");
    Serial.print("Abtastrate: ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    pinMode(LED_BUILTIN, OUTPUT);

    // Perform initial calibration
    delay(500);
    calibrateIMU();
    Serial.println("Bereit – sende Beschleunigungsdaten.");
}

// ── Arduino main loop ─────────────────────────────────────────
void loop() {
    static uint32_t lastSend = 0;
    uint32_t now = millis();

    if (now - lastSend >= SEND_INTERVAL_MS) {
        lastSend = now;

        if (IMU.accelerationAvailable()) {
            float rx, ry, rz;
            IMU.readAcceleration(rx, ry, rz);

            // Remove resting offsets
            float ax = (rx - offsetX) * 9.81f;   // convert g → m/s²
            float ay = (ry - offsetY) * 9.81f;
            float az = (rz - offsetZ) * 9.81f;

            // Transmit via AccelLink (~600 µs)
            AccelTX.sendValues(ax, ay, az);

            // LED heartbeat
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

            // USB-serial debug output
            Serial.print("ax="); Serial.print(ax, 4);
            Serial.print(" ay="); Serial.print(ay, 4);
            Serial.print(" az="); Serial.println(az, 4);
        }
    }
}