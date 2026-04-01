#include <Arduino.h>
#include "HX711.h"
#include "AccelLink_RX.h"

// ── HX711 pin configuration (unchanged from original) ─────────
#define HX711_DOUT  4
#define HX711_SCK   5

// ── AccelLink pin configuration ───────────────────────────────
// GPIO2: data line from Arduino D18
// GPIO3: calibration flag from Arduino D19
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

// ── Helper: re-tare the HX711 (called on calibration flag) ────
static void retareScale() {
    Serial.println("Kalibrierungssignal empfangen – Tarierung...");
    scale.tare(10);
    Serial.println("Tarierung abgeschlossen.");
}

// ── Arduino setup ─────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("Raspberry Pi Pico – Beschleunigungssensor");
    Serial.println("=========================================");

    // Initialise AccelLink receiver (interrupts on GPIO2 / GPIO3)
    AccelRX.begin(ACCELLINK_DATA_PIN, ACCELLINK_CAL_PIN);
    Serial.println("AccelLink RX bereit (GPIO2=Daten, GPIO3=Cal).");

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

    Serial.println();
    Serial.println("Zeit_s, Kraft_N, Accel_DIY_ms2, Accel_IMU_X, Accel_IMU_Y, Accel_IMU_Z, Diff");
    Serial.println("----------------------------------------------------------------------------");

    pinMode(LED_BUILTIN, OUTPUT);
}

// ── Arduino main loop ─────────────────────────────────────────
void loop() {
    static uint32_t lastPrint = 0;
    const uint32_t PRINT_INTERVAL_MS = 20;   // match Arduino send rate

    // ── Handle calibration flag ──────────────────────────────
    if (AccelRX.getCalFlag()) {
        retareScale();
    }

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