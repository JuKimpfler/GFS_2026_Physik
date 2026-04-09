#include <Arduino.h>
#include <Arduino_LSM6DSOX.h>
#include "SimpleLink.h"

#define FILTER_SIZE 5

double filteredY[FILTER_SIZE] = {0};
int16_t filterIndex   = 0;
double filteredAccelY = 0;

double offsetX = 0.0;
double offsetY = 0.0;
double offsetZ = 0.0;

long imuSampleTimeoutMs = 100;
long lastSend           = 0;

SimpleLinkSender link(15);

void calibrateIMU() {
    uint16_t sampleCount = 50;
    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    uint16_t count = 0;

    for (uint16_t i = 0; i < sampleCount; i++) {
        long startTime = millis();
        while (!IMU.accelerationAvailable()) {
            if (millis() - startTime > imuSampleTimeoutMs) break;
            }
        if (IMU.accelerationAvailable()) {
            float fx, fy, fz;
            IMU.readAcceleration(fx, fy, fz);
            sumX += fx; sumY += fy; sumZ += fz;
            count++;
            }
        delay(10);
        }
    if (count > 0) {
        offsetX =  sumX / count;
        offsetY =  sumY / count;
        offsetZ = (sumZ / count) - 1.0;
        }
    }

double applyFilter(double newValue, double* filterArray) {
    filterArray[filterIndex] = newValue;
    double sum = 0;
    for (int16_t i = 0; i < FILTER_SIZE; i++) {
        sum += filterArray[i];
        }
    return sum / FILTER_SIZE;
    }

void setup() {
    Serial.begin(115200);

    IMU.begin();

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(14, INPUT_PULLDOWN);
    pinMode(15, OUTPUT);

    delay(1000);
    while (!IMU.accelerationAvailable());
    float fx, fy, fz;
    IMU.readAcceleration(fx, fy, fz);
    offsetX = fx;
    offsetY = fy;
    offsetZ = fz;
    Serial.println("Ready - sending acceleration data.");
    }

void loop() {
    if (IMU.accelerationAvailable()) {
        float fx, fy, fz;
        IMU.readAcceleration(fx, fy, fz);
        filteredAccelY = applyFilter(((static_cast<double>(fy) - offsetY) * -10), filteredY);
        filterIndex    = (filterIndex + 1) % FILTER_SIZE;
        link.send(filteredAccelY / 2);
        }
    }