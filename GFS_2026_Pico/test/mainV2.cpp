#include <Arduino.h>
#include "HX711.h"
#include "Wire.h"
#include "BotConnect.h"

// HX711 Pin-Konfiguration
#define HX711_1_DOUT 4
#define HX711_1_SCK  5

#define UART_BAUD 115200

#define LED_PIN 25

// Parameter
float SCALE_FACTOR_1 = 1.0;  

HX711 scale_1;

// Variablen für Arduino-Daten
float Ardu_accel = 0.0;
float Pico_accel = 0.0;

bool Read_timer = true;
bool Send_timer = true;
bool HX_timer = true;

float Send_freq = 20;
float Read_freq = 20;
float HX_freq = 10;

// Betriebs Variablen
int r_timer;
int s_timer;
int h_timer;
int last_r;
int last_s;
int last_h;
float raw_ges=0;
float raw_1=0;

float Offset1 = 0;

void read_I2C(){
    Wire.requestFrom(0x08, (uint8_t)4);
    delay(2);
    if (Wire.available() == 4) {
      uint8_t buf[4];
      for (uint8_t i = 0; i < 4; i++) buf[i] = Wire.read();
      float v;
      memcpy(&v, buf, sizeof(v));
      Ardu_accel = v;
    }
}

void setup() {
    // USB Serial für Debugging/Datenausgabe
    Serial.begin(115200);
    Serial1.begin(UART_BAUD);
    BC.begin(Serial1);

    Wire.begin();

    // LED Setup
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    pinMode(15,INPUT);

    scale_1.begin(HX711_1_DOUT, HX711_1_SCK);
    
    // Warte auf Stabilisierung
    delay(3000);
    
    scale_1.tare(20);  // 20 Messungen für Durchschnitt
    
    scale_1.set_scale(SCALE_FACTOR_1);
    
    delay(1000);
    digitalWrite(LED_PIN, LOW);
}

void handle_timer(){
    h_timer = millis() - last_h;
    if((h_timer > (1000/HX_freq))){
        last_h = millis();
        HX_timer = true;
    }
}

void Calibrate(){
    Offset1 = raw_1;
    digitalWrite(LED_PIN,HIGH);

    Wire.beginTransmission(0x08); // Calibrate Arduino
    Wire.write(0x02);
    Wire.endTransmission();
}


void loop() {
    handle_timer();
    BC.process();

    if (scale_1.is_ready() ) {
        raw_1 = scale_1.get_units(1)-Offset1;
        //read_I2C();
    }

    raw_ges = raw_1*-1;
    Pico_accel = (raw_ges/10000);

    if(HX_timer){
        // Ausgabe im CSV-Format
        BC.sendTelemetryFloat("IMU-Acceleration",Ardu_accel);
        BC.sendTelemetryFloat("DIY-Acceleration",Pico_accel);
        Serial.print("> raw_1: ");
        Serial.print(raw_1);
        Serial.print(" ,raw_ges: ");
        Serial.print(raw_ges);
        Serial.print(" ,Pico_ges: ");
        Serial.print(Pico_accel);
        Serial.print(" ,Ardu_ges: ");
        Serial.println(Ardu_accel);
        HX_timer = false;
    }

    if(digitalRead(15) || BC.start){
        Calibrate();
    }
    else{
        digitalWrite(LED_PIN,LOW);
    }
}
