#define RLEDPIN 32
#define YLEDPIN 33
#define GLEDPIN 25

#include "Adafruit_VL53L0X.h"

const float distance_r = 50;
const float distance_y = 80;
const float distance_g = 110;
float mm;


Adafruit_VL53L0X lidar = Adafruit_VL53L0X();

void RYG() {
  VL53L0X_RangingMeasurementData_t measure;
  lidar.rangingTest(&measure, false);
  mm = measure.RangeMilliMeter;
  if (measure.RangeStatus == 4) {
    digitalWrite(RLEDPIN, LOW);
    digitalWrite(YLEDPIN, LOW);
    digitalWrite(GLEDPIN, LOW);
  }
  else if (mm <= distance_r) {
    digitalWrite(RLEDPIN, HIGH);
    digitalWrite(YLEDPIN, LOW);
    digitalWrite(GLEDPIN, LOW);
  }
  else if (mm > distance_r && mm <= distance_y) {
    digitalWrite(RLEDPIN, LOW);
    digitalWrite(YLEDPIN, HIGH);
    digitalWrite(GLEDPIN, LOW);
  }
  else if (mm > distance_y) {
    digitalWrite(RLEDPIN, LOW);
    digitalWrite(YLEDPIN, LOW);
    digitalWrite(GLEDPIN, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  lidar.begin();
  pinMode(RLEDPIN, OUTPUT);
  pinMode(YLEDPIN, OUTPUT);
  pinMode(GLEDPIN, OUTPUT);
  digitalWrite(RLEDPIN, LOW);
  digitalWrite(YLEDPIN, LOW);
  digitalWrite(GLEDPIN, LOW);
}


void loop() {
  RYG();
}
