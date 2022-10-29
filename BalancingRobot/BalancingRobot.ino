/* 
 *  Fall 2022
 *  ME-134 Robotics
 *  Balancing Robot project by Kyu Rae Kim, Selina Spry, and Michelle Kim
 *  A scale that self-balances with uneven weights
*/

#include <JY901.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lidar = Adafruit_VL53L0X();

// Pins
const int ENABLEPIN = 12;
const int MOTORPIN2 = 14;
const int MOTORPIN1 = 27;
const int RLEDPIN = 32;
const int YLEDPIN = 33;
const int GLEDPIN = 25;

// PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 255;

// PID
const float Kp = 14.0;           // P control gain
const float Ki = 0.1;            // I control gain
const float Kd = 4.0;            // D control gain
const int bal_angle = 0;         // Angle when balanced
int distance_sum = 0;            // Accumulated distance

// Lidar
const float distance_r = 40;     // Lidar threshold for red LED
const float distance_y = 50;     // Lidar threshold for yellow LED
const float distance_g = 80;     // Lidar threshold for green LED
float range_mm;                  // Lidar measurement in mm

void PID() {
  JY901.receiveSerialData();
  int distance = round(bal_angle - JY901.getRoll());
  float ang_vel = JY901.getGyroX();
  float control;
  distance_sum += distance;

  control = Kp * distance + Ki * distance_sum + Kd * (0 - ang_vel);

  if (control > 0) {
    digitalWrite(MOTORPIN1, LOW);
    digitalWrite(MOTORPIN2, HIGH);
  }
  else if (control < 0) {
    digitalWrite(MOTORPIN1, HIGH);
    digitalWrite(MOTORPIN2, LOW);
  }

  if (distance >= 20 + bal_angle || distance <= -20  + bal_angle) {
    dutyCycle = 150;
  }
  else if (distance <= 3 && distance >= -3) {
    digitalWrite(MOTORPIN1, LOW);
    digitalWrite(MOTORPIN2, LOW);
  }
  else {
    dutyCycle = abs(control);
  }

  if (dutyCycle >= 150) {
    dutyCycle = 150;
  }

  ledcWrite(pwmChannel, dutyCycle);
  Serial.print("distance: "); Serial.print(distance); Serial.println();
//  Serial.print("duty cycle: "); Serial.println(dutyCycle); Serial.println("\n");
}

void ledControl() {
  VL53L0X_RangingMeasurementData_t measure;
  lidar.rangingTest(&measure, false);
  range_mm = measure.RangeMilliMeter;
  
  if (measure.RangeStatus == 4) {
    digitalWrite(RLEDPIN, LOW);
    digitalWrite(YLEDPIN, LOW);
    digitalWrite(GLEDPIN, LOW);
  }
  else if (range_mm <= distance_r) {
    digitalWrite(RLEDPIN, HIGH);
    digitalWrite(YLEDPIN, LOW);
    digitalWrite(GLEDPIN, LOW);
  }
  else if (range_mm <= distance_y) {
    digitalWrite(RLEDPIN, LOW);
    digitalWrite(YLEDPIN, HIGH);
    digitalWrite(GLEDPIN, LOW);
  }
  else {
    digitalWrite(RLEDPIN, LOW);
    digitalWrite(YLEDPIN, LOW);
    digitalWrite(GLEDPIN, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600);
  JY901.attach(Serial2);
  lidar.begin();

  pinMode(MOTORPIN1, OUTPUT);
  pinMode(MOTORPIN2, OUTPUT);
  pinMode(ENABLEPIN, OUTPUT);
  pinMode(RLEDPIN, OUTPUT);
  pinMode(YLEDPIN, OUTPUT);
  pinMode(GLEDPIN, OUTPUT);

  digitalWrite(RLEDPIN, LOW);
  digitalWrite(YLEDPIN, LOW);
  digitalWrite(GLEDPIN, LOW);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(ENABLEPIN, pwmChannel);
}

void loop() {
  PID();
  ledControl();
  delay(30);
}
