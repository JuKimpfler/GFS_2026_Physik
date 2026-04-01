#include "Arduino.h"
#include <Wire.h>
#define I2C_ADDR 0x08

void onRequest() {
  float v = 1.23f;
  Wire.write((uint8_t*)&v, sizeof(v));
}

void setup() {
  Wire.begin(I2C_ADDR);
  Wire.onRequest(onRequest);
}

void loop() {}