#include <Arduino.h>

/*
  Arduino Nano RP2040 Connect - Referenz-Beschleunigungssensor
  Liest den internen LSM6DS3 IMU und sendet Daten an Raspberry Pi Pico
*/

#include <Arduino_LSM6DSOX.h>

// UART Konfiguration (Serial1)
// TX: Pin 1, RX: Pin 0
#define BAUD_RATE 115200

// Sampling Rate
#define SAMPLE_RATE_HZ 10
#define SAMPLE_INTERVAL_MS (1000 / SAMPLE_RATE_HZ)

// Filter für Beschleunigungswerte (gleitender Durchschnitt)
#define FILTER_SIZE 5

// Variablen
float accel_x, accel_y, accel_z;
float filtered_x[FILTER_SIZE] = {0};
float filtered_y[FILTER_SIZE] = {0};
int filter_index = 0;

unsigned long last_sample_time = 0;
unsigned long measurement_count = 0;

// Kalibrierungsoffsets (bei Stillstand bestimmen)
float offset_x = 0.0;
float offset_y = 0.0;
float offset_z = 0.0;

void calibrate_imu() {
  const int samples = 100;
  float sum_x = 0, sum_y = 0, sum_z = 0;
  
  for (int i = 0; i < samples; i++) {
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(accel_x, accel_y, accel_z);
      sum_x += accel_x;
      sum_y += accel_y;
      sum_z += accel_z;
      delay(10);
    }
  }
  
  offset_x = sum_x / samples;
  offset_y = sum_y / samples;
  offset_z = (sum_z / samples) - 1.0; // Z sollte 1g zeigen (Gravitation)
  
  Serial.print("Offsets - X: ");
  Serial.print(offset_x, 4);
  Serial.print(" Y: ");
  Serial.print(offset_y, 4);
  Serial.print(" Z: ");
  Serial.println(offset_z, 4);
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
  // USB Serial für Debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Warte max 3 Sekunden
  
  // UART für Kommunikation mit Pico
  Serial1.begin(BAUD_RATE);
  
  Serial.println("Arduino Nano RP2040 - Beschleunigungssensor");
  Serial.println("==========================================");
  
  // Initialisiere IMU
  if (!IMU.begin()) {
    Serial.println("FEHLER: LSM6DSOX IMU konnte nicht initialisiert werden!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  
  Serial.println("IMU erfolgreich initialisiert");
  Serial.print("Beschleunigungsmesser Sample Rate: ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  
  // LED Setup
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Kalibrierung - Mittelwert bei Stillstand
  Serial.println("\nKalibrierung... Bitte Sensor ruhig halten!");
  delay(2000);
  calibrate_imu();
  
  Serial.println("Kalibrierung abgeschlossen");
  Serial.println("\nStarte Messungen...");
  Serial.println("Format: X-Accel, Y-Accel, Z-Accel (m/s²)");
  Serial.println("---------------------------------------");
  
  last_sample_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  
  // Prüfe ob Sample-Intervall erreicht
  if (current_time - last_sample_time >= SAMPLE_INTERVAL_MS) {
    last_sample_time = current_time;
    
    // Lese Beschleunigungswerte
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(accel_x, accel_y, accel_z);
      
      // Entferne Offsets
      accel_x -= offset_x;
      accel_y -= offset_y;
      accel_z -= offset_z;
      
      // Wende Filter an (optional)
      float filtered_accel_x = apply_filter(accel_x, filtered_x);
      float filtered_accel_y = apply_filter(accel_y, filtered_y);
      
      // Aktualisiere Filter-Index
      filter_index = (filter_index + 1) % FILTER_SIZE;
      
      // Konvertiere von g zu m/s²
      float accel_x_ms2 = filtered_accel_x * 9.81;
      float accel_y_ms2 = filtered_accel_y * 9.81;
      float accel_z_ms2 = accel_z * 9.81;
      
      // Sende an Pico via UART (Format: "X:value,Y:value")
      Serial1.print("X:");
      Serial1.print(accel_x_ms2, 3);
      Serial1.print(",Y:");
      Serial1.println(accel_y_ms2, 3);
      
      // Debug-Ausgabe über USB Serial
      Serial.print(measurement_count);
      Serial.print(": X=");
      Serial.print(accel_x_ms2, 4);
      Serial.print(" m/s²  Y=");
      Serial.print(accel_y_ms2, 4);
      Serial.print(" m/s²  Z=");
      Serial.print(accel_z_ms2, 4);
      Serial.println(" m/s²");
      
      // LED Blinken
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      
      measurement_count++;
    }
  }
  
  // Empfange optional Daten vom Pico
  if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    if (data.startsWith("PICO:")) {
      Serial.print("Empfangen vom Pico: ");
      Serial.println(data);
    }
  }
}
