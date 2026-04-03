#include <Arduino.h>
#include "HX711.h"
#include "BotConnect.h"

// ── HX711 pin configuration (unchanged from original) ─────────
#define HX711_DOUT  2
#define HX711_SCK   3
static float SCALE_FACTOR = 1.0f;
HX711 scale;

float force_N   = 0;
float accel_diy = 0;

int count = 0;
int count2 = 0;

long lastSend = 0;

const int rxPin = 9;
enum EmpfangsStatus { RESET_WAIT, IDLE, START_MESSUNG, PAUSE_WARTEN, BIT_LESEN };
EmpfangsStatus status = RESET_WAIT;
float value=0;
unsigned long zeitStempel = 0;
int bitZahler = 0;
int bits[13];
bool datenBereit = false;

int Faktor = 300;

void updateReceiver1() {
  bool pinState = digitalRead(rxPin);
  unsigned long jetzt = micros();

  switch (status) {
    case RESET_WAIT:
        status = IDLE;
        break;

    case IDLE:
      break;

    case START_MESSUNG:
      // Wir warten auf die fallende Flanke des 150µs Impulses
      if (pinState == LOW) {
        unsigned long dauer = jetzt - zeitStempel;
        if (dauer >= 13*Faktor && dauer <= 17*Faktor) {
          zeitStempel = jetzt; // Start der 50µs Pause (LOW-Phase)
          status = PAUSE_WARTEN;
        } else {
          status = RESET_WAIT;
        }
      }
      break;

    case PAUSE_WARTEN:
      // Warte das Ende der 50µs LOW-Phase ab
      if (jetzt - zeitStempel >= 10*Faktor) {
        zeitStempel = jetzt;
        bitZahler = 0;
        status = BIT_LESEN;
      }
      break;

    case BIT_LESEN:
      // Das erste Bit nach 50µs lesen (Mitte), alle weiteren nach 100µs
      unsigned long intervall = (bitZahler == 0) ? 5*Faktor : 10*Faktor;

      if (jetzt - zeitStempel >= intervall) {
        bits[bitZahler] = digitalRead(rxPin);
        bitZahler++;
        
        zeitStempel += intervall;

        if (bitZahler >= 13) {
          digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
          datenBereit = true;
          status = RESET_WAIT;
          zeitStempel = micros(); 
        }
      }
      break;
  }
}

void start1(){
    if(status==1){
        unsigned long jetzt = micros();
        zeitStempel = jetzt;
        status = START_MESSUNG;
        count = count+1;
        count2 = count2+1;
    }
}

static void Calibration() {
    scale.tare(10); 
    //digitalWrite(8,HIGH);
    //delay(1000);
    //digitalWrite(8,LOW);        
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(rxPin,INPUT_PULLUP);
    pinMode(8,OUTPUT);
    attachInterrupt(digitalPinToInterrupt(rxPin), start1, RISING);

    BC.begin(Serial1);

    delay(3000);

    scale.begin(HX711_DOUT, HX711_SCK);

    delay(500);

    while(!scale.is_ready());
    Serial.println("HX711 gefunden.");
    scale.tare(20);
    scale.set_scale(SCALE_FACTOR);
    Serial.println("Calibriert abgeschlossen.");

    delay(500);

    Calibration();
}

#define FILTER_SIZE 5
float filtered_y[FILTER_SIZE] = {0};
int filter_index = 0;
float filtered_accel_y;

float apply_filter(float new_value, float* filter_array) {
  // Füge neuen Wert hinzu
  filter_array[filter_index] = new_value;

  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += filter_array[i];
  }
  
  return sum / FILTER_SIZE;
}

// ── Arduino main loop ─────────────────────────────────────────
void loop() {
    if (status == 1 || status == 0){interrupts();}
    else{noInterrupts();}

    if (status != 1 ){
        noInterrupts();
        updateReceiver1();
    }
    else{
      BC.process();

      if (scale.is_ready()) {

          force_N   = scale.get_units(1)*-1;
          accel_diy = force_N/10000;
      }

      if (datenBereit==true) {
          count = count-1;
          uint16_t Ganzzahl = 0;
          for (int i = 0; i < 12; i++) {
              if (bits[i+1] == 1) {
                  Ganzzahl |= (1 << i);
              }
          }
          value = (Ganzzahl / 1000.0)*2;
          if (bits[0]==1){value = value*-1;}
          datenBereit = false;
      }  

      if(digitalRead(15)){Calibration();}

      if (millis() - lastSend >= 30) {
          filtered_accel_y = apply_filter((accel_diy), filtered_y);
          filter_index = (filter_index + 1) % FILTER_SIZE;

          //BC.sendTelemetryFast("IMU-Acce",value);
          //BC.sendTelemetryFast("DIY-Acce",filtered_accel_y);

          float t = millis()/1000.0;

          BC.sendP2P(String(t)+"\t"+String(filtered_accel_y)+"\t"+String(value)+"\r\n");
          Serial.print(String(t)+"\t"+String(filtered_accel_y)+"\t"+String(value)+"\r\n");

          
      }      
    }   
}