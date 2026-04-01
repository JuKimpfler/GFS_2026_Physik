#include <Wire.h>
//#include <Arduino_LSM6DSOX.h>

#define I2C_ADDR 0x08

volatile float accelY = 0.0f;
volatile float offset  = 0.0f;
volatile bool  doTare  = false;

void onReceive(int n) {
  if (!Wire.available()) return;
  uint8_t cmd = Wire.read();
  if (cmd == 0x02) doTare = true;
}

void onRequest() {
  float v = accelY - offset;
  Wire.write((uint8_t*)&v, sizeof(v));
}

void setup() {
  //IMU.begin();
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
}

void loop() {
  float x, y, z;
  //if (IMU.accelerationAvailable()) {
    //IMU.readAcceleration(x, y, z);
  //  accelY = y * 9.80665f;
  //}
  if (doTare) {
    offset = accelY;
    doTare = false;
  }
}