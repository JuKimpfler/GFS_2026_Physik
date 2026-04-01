#include <Arduino.h>
#include "HX711.h"
#include "AccelLink_RX.h"
#include "BotConnect.h"

// ── HX711 pin configuration (unchanged from original) ─────────
#define HX711_DOUT  4
#define HX711_SCK   5

// ── AccelLink pin configuration ───────────────────────────────
// GPIO2: data line from Arduino D18
// GPIO3: calibration flag output → Arduino D19
#define ACCELLINK_DATA_PIN 2
#define ACCELLINK_CAL_PIN  3

// ── Physical parameter ────────────────────────────────────────
// Mass of the pendulum / proof-mass in kg.  Set to your actual mass.
// Must be > 0 to avoid division-by-zero in acceleration calculation.
static float MASS_KG = 0.100f;

// ── HX711 scale factor (raw → Newton).  Calibrate experimentally. ──
static float SCALE_FACTOR = 1.0f;

// ── Objects ───────────────────────────────────────────────────
HX711 scale;

// ── Helper: trigger calibration on both sensors ───────────────
// Pulses GPIO3 HIGH (→ Arduino re-calibrates IMU) and
// simultaneously re-tares the local HX711.
static void triggerFullCalibration() {
    Serial.println("Kalibrierung beider Sensoren...");
    scale.tare(10);                         // re-tare HX711 first
    AccelRX.triggerCalibration(10);         // then signal Arduino
    Serial.println("Tarierung abgeschlossen.");
}

// ── Arduino setup ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    BC.begin(Serial1);

    delay(10000);

    Serial.println("Raspberry Pi Pico – Beschleunigungssensor");
    Serial.println("=========================================");

    // Initialise AccelLink receiver (interrupt on GPIO2, cal output on GPIO3)
    AccelRX.begin(ACCELLINK_DATA_PIN, ACCELLINK_CAL_PIN);
    Serial.println("AccelLink RX bereit (GPIO2=Daten, GPIO3=Cal-Out).");

    // Initialise HX711
    scale.begin(HX711_DOUT, HX711_SCK);
    delay(500);

    if (scale.is_ready()) {
        Serial.println("HX711 gefunden.");
        Serial.println("Tarierung... Bitte System ruhig halten.");
        scale.tare(20);
        scale.set_scale(SCALE_FACTOR);
        Serial.println("Tarierung abgeschlossen.");
    } else {
        Serial.println("WARNUNG: HX711 nicht bereit!");
    }

    if (MASS_KG <= 0.0f) {
        Serial.println("WARNUNG: MASS_KG ist 0 oder negativ – Beschleunigung kann nicht berechnet werden!");
    }

    // Trigger initial calibration: re-tare HX711 and tell Arduino to
    // calibrate its IMU at the same time.
    delay(500);
    triggerFullCalibration();

    Serial.println();
    Serial.println("Zeit_s, Kraft_N, Accel_DIY_ms2, Accel_IMU_X, Accel_IMU_Y, Accel_IMU_Z, Diff");
    Serial.println("----------------------------------------------------------------------------");

    pinMode(LED_BUILTIN, OUTPUT);
}

// ── Arduino main loop ─────────────────────────────────────────
void loop() {
    BC.process();
    static uint32_t lastPrint = 0;
    const uint32_t PRINT_INTERVAL_MS = 20;   // match Arduino send rate

    uint32_t now = millis();
    if (now - lastPrint < PRINT_INTERVAL_MS) return;
    lastPrint = now;

    // ── Read HX711 (DIY accelerometer) ───────────────────────
    float force_N   = 0.0f;
    float accel_diy = 0.0f;
    if (scale.is_ready()) {
        force_N   = scale.get_units(1);
        accel_diy = (MASS_KG > 0.0f) ? (force_N / MASS_KG) : 0.0f;
    }

    // ── Read AccelLink IMU data ───────────────────────────────
    float imu_x = 0.0f, imu_y = 0.0f, imu_z = 0.0f;
    if (AccelRX.hasNewData()) {
        imu_x = AccelRX.getX();   // clears hasNewData flag
        imu_y = AccelRX.getY();
        imu_z = AccelRX.getZ();
    }

    float diff = accel_diy - imu_x;

    // ── CSV output ────────────────────────────────────────────
    float time_s = now / 1000.0f;
    BC.sendTelemetryFloat("IMU-Acceleration",imu_y);
    BC.sendTelemetryFloat("DIY-Acceleration",accel_diy);
    BC.sendTelemetryFloat("Diff",diff);
    Serial.print(time_s, 3);   Serial.print(", ");
    Serial.print(force_N, 4);  Serial.print(", ");
    Serial.print(accel_diy, 4); Serial.print(", ");
    Serial.print(imu_x, 4);   Serial.print(", ");
    Serial.print(imu_y, 4);   Serial.print(", ");
    Serial.print(imu_z, 4);   Serial.print(", ");
    Serial.println(diff, 4);

    // LED heartbeat
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}