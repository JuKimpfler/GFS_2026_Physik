#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include "SimpleLink.h"



#define FILTER_SIZE 5
float filtered_y[FILTER_SIZE] = {0};
int filter_index = 0;
float filtered_accel_y;

float offsetX = 0.0f;
float offsetY = 0.0f;
float offsetZ = 0.0f;

uint32_t IMU_SAMPLE_TIMEOUT_MS = 100;

SimpleLinkSender link(15);

// Average the IMU over N samples to find the resting offset.
// Assumes the Z-axis is aligned vertically (reads +1 g at rest).
void calibrateIMU() {
    uint8_t N = 50;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    uint8_t count = 0;

    Serial.println("Kalibrierung... Sensor bitte ruhig halten.");
    for (uint8_t i = 0; i < N; i++) {
        uint32_t t0 = millis();
        while (!IMU.accelerationAvailable()) {
            if (millis() - t0 > IMU_SAMPLE_TIMEOUT_MS) break;
        }
        if (IMU.accelerationAvailable()) {
            float x, y, z;
            IMU.readAcceleration(x, y, z);
            sx += x;  sy += y;  sz += z;
            count++;
        }
        delay(10);
    }
    if (count > 0) {
        offsetX =  sx / count;
        offsetY =  sy / count;
        offsetZ = (sz / count) - 1.0f;
    }
}

float apply_filter(float new_value, float* filter_array) {
  // Füge neuen Wert hinzu
  filter_array[filter_index] = new_value;
  
  // Berechne Durchschnitt
  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += filter_array[i];
  }
  
  return sum / FILTER_SIZE;
}

void setup() {
    Serial.begin(115200);

    IMU.begin();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(14,INPUT_PULLDOWN);
    pinMode(15,OUTPUT);

    // Perform initial calibration
    delay(1000);
    while(!IMU.accelerationAvailable());
    IMU.readAcceleration(offsetX, offsetY, offsetZ);
    Serial.println("Bereit – sende Beschleunigungsdaten.");
}

uint32_t lastSend  = 0;

void loop() {

    //if(digitalRead(14)){
    //    calibrateIMU();
    //}

    if (IMU.accelerationAvailable()) {
        float rx, ry, rz;
        IMU.readAcceleration(rx, ry, rz);
        filtered_accel_y = apply_filter(((ry - offsetY)*-10), filtered_y);
        filter_index = (filter_index + 1) % FILTER_SIZE;
        link.send(filtered_accel_y/2);
    }
}