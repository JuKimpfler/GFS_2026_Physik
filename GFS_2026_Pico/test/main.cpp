#include <Arduino.h>

/*
  Raspberry Pi Pico - 2-Achsen Beschleunigungssensor mit HX711 Wägezellen
  Programmiert mit Arduino IDE (C++)
  Misst Beschleunigung über Kraftmessung und vergleicht mit Arduino Nano RP2040
*/

#include "HX711.h"

// HX711 Pin-Konfiguration
// HX711 #1 (X-Achse)
#define HX711_1_DOUT 2
#define HX711_1_SCK  3

// HX711 #2 (Y-Achse)
#define HX711_2_DOUT 4
#define HX711_2_SCK  5

// UART Konfiguration (Serial1)
// TX: GPIO 0, RX: GPIO 1
#define UART_BAUD 115200

// LED Pin (eingebaute LED am Pico)
#define LED_PIN LED_BUILTIN

// Physikalische Parameter
float MASS = 0.100;  // Masse in kg (100g) - ANPASSEN!

// Kalibrierungsfaktoren (HX711 Rohwert zu Newton)
// Diese Werte müssen experimentell bestimmt werden!
float SCALE_FACTOR_X = 1.0;  // ANPASSEN!
float SCALE_FACTOR_Y = 1.0;  // ANPASSEN!

// HX711 Objekte
HX711 scale_x;
HX711 scale_y;

// Variablen für Arduino-Daten
float arduino_accel_x = 0.0;
float arduino_accel_y = 0.0;
bool arduino_data_received = false;

// Timing
unsigned long last_measurement = 0;
const unsigned long MEASUREMENT_INTERVAL = 100; // 100ms = 10Hz

// Statistik
unsigned long measurement_count = 0;

// Filter für HX711 Werte (gleitender Durchschnitt)
#define FILTER_SIZE 3
float filter_x[FILTER_SIZE] = {0};
float filter_y[FILTER_SIZE] = {0};
int filter_index = 0;

float apply_filter(float new_value, float* filter_array) {
  // Füge neuen Wert zum Filter hinzu
  filter_array[filter_index] = new_value;
  
  // Berechne Durchschnitt
  float sum = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    sum += filter_array[i];
  }
  
  return sum / FILTER_SIZE;
}

void read_arduino_data() {
  // Liest Beschleunigungsdaten vom Arduino Nano RP2040
  // Erwartetes Format: "X:1.234,Y:5.678\n"
  
  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    
    // Parse die Daten
    int comma_pos = line.indexOf(',');
    if (comma_pos > 0) {
      String x_part = line.substring(0, comma_pos);
      String y_part = line.substring(comma_pos + 1);
      
      int x_colon = x_part.indexOf(':');
      int y_colon = y_part.indexOf(':');
      
      if (x_colon > 0 && y_colon > 0) {
        arduino_accel_x = x_part.substring(x_colon + 1).toFloat();
        arduino_accel_y = y_part.substring(y_colon + 1).toFloat();
        arduino_data_received = true;
      }
    }
  }
}

float calculate_acceleration(float force, float mass) {
  // Berechnet Beschleunigung: a = F / m
  if (mass > 0) {
    return force / mass;
  }
  return 0.0;
}

void setup() {
  // USB Serial für Debugging/Datenausgabe
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Warte max 3 Sekunden
  
  // UART für Kommunikation mit Arduino Nano RP2040
  Serial1.begin(UART_BAUD);
  
  // LED Setup
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.println("=====================================");
  Serial.println("Raspberry Pi Pico");
  Serial.println("2-Achsen Beschleunigungssensor");
  Serial.println("mit HX711 Waegezellen");
  Serial.println("=====================================");
  Serial.println();
  
  // Initialisiere HX711 Sensoren
  Serial.println("Initialisiere HX711 Sensoren...");
  
  scale_x.begin(HX711_1_DOUT, HX711_1_SCK);
  scale_y.begin(HX711_2_DOUT, HX711_2_SCK);
  
  // Warte auf Stabilisierung
  delay(2000);
  
  // Prüfe ob HX711 bereit sind
  if (scale_x.is_ready() && scale_y.is_ready()) {
    Serial.println("HX711 Sensoren gefunden!");
  } else {
    Serial.println("WARNUNG: HX711 nicht bereit!");
  }
  
  // Tarierung (Nullpunkt setzen)
  Serial.println("\nTariere Sensoren...");
  Serial.println("Bitte System nicht bewegen!");
  delay(1000);
  
  scale_x.tare(20);  // 20 Messungen für Durchschnitt
  scale_y.tare(20);
  
  Serial.println("Tarierung abgeschlossen!");
  
  // Optional: Kalibrierung mit bekanntem Gewicht
  /*
  Serial.println("\nKalibrierung X-Achse:");
  Serial.println("Lege bekanntes Gewicht (z.B. 100g) auf X-Achse...");
  delay(5000);
  scale_x.calibrate_scale(100.0, 20);  // 100g
  SCALE_FACTOR_X = scale_x.get_scale();
  Serial.print("Kalibrierungsfaktor X: ");
  Serial.println(SCALE_FACTOR_X, 6);
  
  Serial.println("\nKalibrierung Y-Achse:");
  Serial.println("Lege bekanntes Gewicht (z.B. 100g) auf Y-Achse...");
  delay(5000);
  scale_y.calibrate_scale(100.0, 20);  // 100g
  SCALE_FACTOR_Y = scale_y.get_scale();
  Serial.print("Kalibrierungsfaktor Y: ");
  Serial.println(SCALE_FACTOR_Y, 6);
  */
  
  // Setze Skalierungsfaktoren
  scale_x.set_scale(SCALE_FACTOR_X);
  scale_y.set_scale(SCALE_FACTOR_Y);
  
  Serial.println("\n=== System bereit ===");
  Serial.println("\nDatenformat:");
  Serial.println("Zeit, Kraft_X, Kraft_Y, Accel_X(exp), Accel_Y(exp), Accel_X(ref), Accel_Y(ref), Diff_X, Diff_Y");
  Serial.println("-------------------------------------------------------------------------------------");
  
  digitalWrite(LED_PIN, LOW);
  last_measurement = millis();
}



void loop() {
  unsigned long current_time = millis();
  
  // Prüfe ob Messintervall erreicht
  if (current_time - last_measurement >= MEASUREMENT_INTERVAL) {
    last_measurement = current_time;
    
    // LED Toggle
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
    // Lese HX711 Werte
    if (scale_x.is_ready() && scale_y.is_ready()) {
      
      // Lese Rohwerte
      float raw_x = scale_x.get_units(1);  // 1 Messung (schneller)
      float raw_y = scale_y.get_units(1);
      
      // Wende Filter an
      float force_x = apply_filter(raw_x, filter_x);
      float force_y = apply_filter(raw_y, filter_y);
      
      // Aktualisiere Filter-Index
      filter_index = (filter_index + 1) % FILTER_SIZE;
      
      // Berechne Beschleunigung: a = F/m
      float accel_x_exp = calculate_acceleration(force_x, MASS);
      float accel_y_exp = calculate_acceleration(force_y, MASS);
      
      // Lese Referenzdaten vom Arduino
      read_arduino_data();
      
      // Berechne Differenzen
      float diff_x = accel_x_exp - arduino_accel_x;
      float diff_y = accel_y_exp - arduino_accel_y;
      
      // Zeitstempel (Sekunden seit Start)
      float time_sec = current_time / 1000.0;
      
      // Ausgabe im CSV-Format
      Serial.print(time_sec, 3);
      Serial.print(", ");
      Serial.print(force_x, 4);
      Serial.print(", ");
      Serial.print(force_y, 4);
      Serial.print(", ");
      Serial.print(accel_x_exp, 4);
      Serial.print(", ");
      Serial.print(accel_y_exp, 4);
      Serial.print(", ");
      Serial.print(arduino_accel_x, 4);
      Serial.print(", ");
      Serial.print(arduino_accel_y, 4);
      Serial.print(", ");
      Serial.print(diff_x, 4);
      Serial.print(", ");
      Serial.println(diff_y, 4);
      
      // Sende Feedback an Arduino (optional)
      Serial1.print("PICO:");
      Serial1.print(accel_x_exp, 3);
      Serial1.print(",");
      Serial1.println(accel_y_exp, 3);
      
      measurement_count++;
      
    } else {
      Serial.println("FEHLER: HX711 nicht bereit!");
    }
  }
  
  // Lese kontinuierlich Arduino-Daten
  read_arduino_data();
}
