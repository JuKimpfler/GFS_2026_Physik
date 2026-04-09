#include <Arduino.h>
#include "HX711.h"
#include "BotConnect.h"

#define HX711_DOUT 2
#define HX711_SCK  3
#define FILTER_SIZE 5

static double scaleFactor = 220;
HX711 scale;

double forceN        = 0;
double force_kiloN   = 0;
double accelDiy      = 0;
int16_t count        = 0;
int16_t count2       = 0;
int last             = 0;

const int16_t RX_PIN = 9;

enum ReceiverState { RESET_WAIT, IDLE, START_MEASUREMENT, WAIT_PAUSE, READ_BIT };
ReceiverState state = RESET_WAIT;

double value              = 0;
unsigned long timestamp   = 0;
int16_t bitCounter        = 0;
int16_t bits[13];
bool dataReady            = false;
long factor               = 300;

double filteredY[FILTER_SIZE] = {0};
int16_t filterIndex           = 0;
double filteredAccelY         = 0;

double applyFilter(double newValue, double* filterArray) {
    filterArray[filterIndex] = newValue;
    double sum = 0;
    for (int16_t i = 0; i < FILTER_SIZE; i++) {
        sum += filterArray[i];
    }
    return sum / FILTER_SIZE;
}

void updateReceiver() {
    bool pinState        = digitalRead(RX_PIN);
    unsigned long now    = micros();

    switch (state) {
        case RESET_WAIT:
            state = IDLE;
            break;

        case IDLE:
            break;

        case START_MEASUREMENT:
            if (pinState == LOW) {
                unsigned long duration = now - timestamp;
                if (duration >= (unsigned long)(13 * factor) && duration <= (unsigned long)(17 * factor)) {
                    timestamp = now;
                    state = WAIT_PAUSE;
                }
                else {
                    state = RESET_WAIT;
                }
            }
            break;

        case WAIT_PAUSE:
            if (now - timestamp >= (unsigned long)(10 * factor)) {
                timestamp  = now;
                bitCounter = 0;
                state      = READ_BIT;
            }
            break;

        case READ_BIT: {
            unsigned long interval = (bitCounter == 0)
                ? (unsigned long)(5 * factor)
                : (unsigned long)(10 * factor);
            if (now - timestamp >= interval) {
                bits[bitCounter] = digitalRead(RX_PIN);
                bitCounter++;
                timestamp += interval;
                if (bitCounter >= 13) {
                    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                    dataReady = true;
                    state     = RESET_WAIT;
                    timestamp = micros();
                }
            }
            break;
        }
    }
}

void startMeasurement() {
    if (state == IDLE) {
        unsigned long now = micros();
        timestamp = now;
        state     = START_MEASUREMENT;
        count     = count + 1;
        count2    = count2 + 1;
    }
}

static void calibrate() {
    scale.tare(20);
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RX_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RX_PIN), startMeasurement, RISING);

    BC.begin(Serial1);

    delay(3000);

    scale.begin(HX711_DOUT, HX711_SCK);

    delay(500);

    while (!scale.is_ready());
    Serial.println("HX711 ready.");
    scale.tare(20);
    scale.set_scale(scaleFactor);
    Serial.println("Calibration done.");

    delay(500);

    calibrate();
}

void loop() {
    if (state == IDLE || state == RESET_WAIT) {
        interrupts();
    }
    else {
        noInterrupts();
    }

    if (state != IDLE) {
        noInterrupts();
        updateReceiver();
    }
    else {
        if (scale.is_ready()) {
            force_kiloN = scale.get_units(1);
            forceN = force_kiloN/1000;
            accelDiy = forceN/0.540; // Masse des Messgewichts: 540g
        }

        if (dataReady == true) {
            count--;
            uint16_t intValue = 0;
            for (int16_t i = 0; i < 12; i++) {
                if (bits[i + 1] == 1) {
                    intValue |= (1 << i);
                    }
                }
            value = intValue;
            if (bits[0] == 1) {
                value = value * -1;
                }
            dataReady = false;
            value = value/1000;
        }

        if (digitalRead(15)) {
            calibrate();
        }

        if(millis()-last>40){
            last = millis();
            filteredAccelY = applyFilter(accelDiy, filteredY);
            filterIndex    = (filterIndex + 1) % FILTER_SIZE;
            BC.sendP2P(String(millis()) + " " + String(filteredAccelY) + " " + String(value) + "\n");
        }
    }
}