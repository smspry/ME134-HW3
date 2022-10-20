/* 
 *  Fall 2022
 *  ME-134 Robotics
 *  Balancing Robot project by Kyu Rae Kim, Selina Spry, and Michelle Kim
 *  A scale that self-balances with uneven weights
*/

#include <JY901.h>

// Motor pins
const int MOTORPIN1 = 26;
const int MOTORPIN2 = 25;
const int ENABLEPIN = 27;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

const float Kp = 1.0;
const float Ki = 0;
const float Kd = 1;
const int flat = 0;
int acc = 0;

void PID() {
  int distance = round(JY901.getRoll()) - flat;
//  float ang_vel = JY901.getGyroZ();
//  acc += distance;

  if (distance >= 0) {
    digitalWrite(MOTORPIN1, LOW);
    digitalWrite(MOTORPIN2, HIGH);
    dutyCycle = Kp * map(distance, flat, 90, 0, 200);
  }
  else {
    digitalWrite(MOTORPIN1, HIGH);
    digitalWrite(MOTORPIN2, LOW);
    dutyCycle = Kp * map(distance, flat, -90, 0, 200);
  }
  ledcWrite(pwmChannel, dutyCycle);
  Serial.println(dutyCycle);
  Serial.println(JY901.getRoll());
}

void setup() {
  Serial.begin(115200);
  JY901.startIIC();

  pinMode(MOTORPIN1, OUTPUT);
  pinMode(MOTORPIN2, OUTPUT);
  pinMode(ENABLEPIN, OUTPUT);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(ENABLEPIN, pwmChannel);
//  ledcWrite(pwmChannel, dutyCycle);
}

void loop() {
  PID();
  delay(30);
}
