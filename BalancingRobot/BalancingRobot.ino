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
const int MOTORPIN1 = 14;
const int MOTORPIN2 = 27;
const int RLEDPIN = 32;
const int YLEDPIN = 33;
const int GLEDPIN = 25;

// PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 255;

// PID
const float Kp = 0.6;            // P control gain
const float Ki = 0.0;            // I control gain
const float Kd = 0.4;            // D control gain
const int bal_angle = 0;         // Angle when balanced
int distance_sum = 0;            // Accumulated distance

// Lidar
const float distance_r = 50;     // Lidar threshold for red LED
const float distance_y = 80;     // Lidar threshold for yellow LED
const float distance_g = 110;    // Lidar threshold for green LED
float range_mm;                  // Lidar measurement in mm

void PID() {
  JY901.receiveSerialData();
  int distance = round(JY901.getRoll()) - bal_angle;
  float ang_vel = JY901.getGyroX();
  distance_sum += distance;

  if (distance <= 5 && distance >= -5) {
    digitalWrite(MOTORPIN1, LOW);
    digitalWrite(MOTORPIN2, LOW);
  }
//  else if (distance <= -10 && distance >= -30) {
//    digitalWrite(MOTORPIN1, LOW);
//    digitalWrite(MOTORPIN2, HIGH);
//    dutyCycle = 128;
//  }
//  else if (distance >= 10 && distance <= 30) {
//    digitalWrite(MOTORPIN1, HIGH);
//    digitalWrite(MOTORPIN2, LOW);
//    dutyCycle = 128;
//  }
  else if (distance <= -30) {
    digitalWrite(MOTORPIN1, LOW);
    digitalWrite(MOTORPIN2, HIGH);
    dutyCycle = 255;
  }
  else if (distance >= 30) {
    digitalWrite(MOTORPIN1, HIGH);
    digitalWrite(MOTORPIN2, LOW);
    dutyCycle = 128;
  }
  else if (distance <= 0) {
    digitalWrite(MOTORPIN1, LOW);
    digitalWrite(MOTORPIN2, HIGH);
    dutyCycle = Kp * map(distance, bal_angle, -60, 0, 255) - Ki * distance_sum + Kd * ang_vel;
  }
  else {
    digitalWrite(MOTORPIN1, HIGH);
    digitalWrite(MOTORPIN2, LOW);
    dutyCycle = Kp * map(distance, bal_angle, 60, 0, 255) - Ki * distance_sum - Kd * ang_vel;
  }
  ledcWrite(pwmChannel, dutyCycle);
  Serial.println(distance);
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
