/* 
 *  Fall 2022
 *  ME-134 Robotics
 *  Balancing Robot project by Kyu Rae Kim, Selina Spry, and Michelle Kim
 *  A scale that self-balances with uneven weights
 *  Target goal:  balance with a weight on only one side
 *  Stretch goal: while balancing, put weights on either sides to make the system more dynamic
*/

#include <math.h>
#include <JY901.h>
#include <WiFi.h>
#include "Adafruit_VL53L0X.h"
#include "ESPAsyncWebServer.h"

// ESP32 Wifi SSID and Password
const char* ssid = "ESP32-team1";
const char* password = "team1"; 

// MATLAB communication settings
AsyncWebServer server(80);
String val = "0";

// Pins
const int ENABLEPIN = 12;
const int MOTORPIN1 = 27;
const int MOTORPIN2 = 14;
const int RLEDPIN = 32;
const int YLEDPIN = 33;
const int GLEDPIN = 25;

// PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 255;

// General measurements
float radius = 197;
float plate_height = 92.0;

// PID
const float Kp = 14.0;           // P control gain
const float Ki = 0.1;            // I control gain
const float Kd = 4.0;            // D control gain
const int bal_angle = 0;         // Angle when balanced
int distance;                    // Angle between measured and desired points
int distance_sum = 0;            // Accumulated distance
float ang_vel;                   // Angular velocity
float control;                   // Control output

// Lidar
const float distance_r = 40;     // Lidar threshold for red LED
const float distance_y = 50;     // Lidar threshold for yellow LED
const float distance_g = 80;     // Lidar threshold for green LED
float range_mm;                  // Lidar measurement in mm
Adafruit_VL53L0X lidar = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;


/* PID control function */
void PID() {
  JY901.receiveSerialData();
  distance = round(bal_angle - JY901.getRoll());
  ang_vel = JY901.getGyroX();
  distance_sum += distance;
  control = Kp * distance + Ki * distance_sum + Kd * (0 - ang_vel);

  // Determine motor direction according to the control output signs
  if (control > 0) {
    digitalWrite(MOTORPIN1, LOW);
    digitalWrite(MOTORPIN2, HIGH);
  }
  else if (control < 0) {
    digitalWrite(MOTORPIN1, HIGH);
    digitalWrite(MOTORPIN2, LOW);
  }

  // High speed in this angle range to have enough torque to initially lift up weights
  if (distance >= 20 + bal_angle || distance <= -20  + bal_angle) {
    dutyCycle = 150;
  }
  // Differential gap to prevent excessive oscillation
  else if (distance <= 3 && distance >= -3) {
    digitalWrite(MOTORPIN1, LOW);
    digitalWrite(MOTORPIN2, LOW);
  }
  // Speed of the motor determined by the control output
  else {
    dutyCycle = abs(control);
  }

  // Limit the maximum speed
  if (dutyCycle >= 150) {
    dutyCycle = 150;
  }

  ledcWrite(pwmChannel, dutyCycle);
//  Serial.print("distance: "); Serial.print(distance); Serial.println();
//  Serial.print("duty cycle: "); Serial.println(dutyCycle); Serial.println("\n");
}

/* Function to control LEDs according to the Lidar measurements */
void ledControl() {
  lidar.rangingTest(&measure, false);
  range_mm = measure.RangeMilliMeter;
  
  if (measure.RangeStatus == 4) {      // out of range
    digitalWrite(RLEDPIN, LOW);
    digitalWrite(YLEDPIN, LOW);
    digitalWrite(GLEDPIN, LOW);
  }
  else if (range_mm <= distance_r) {   // turn on red LED
    digitalWrite(RLEDPIN, HIGH);
    digitalWrite(YLEDPIN, LOW);
    digitalWrite(GLEDPIN, LOW);
  }
  else if (range_mm <= distance_y) {   // turn on yellow LED
    digitalWrite(RLEDPIN, LOW);
    digitalWrite(YLEDPIN, HIGH);
    digitalWrite(GLEDPIN, LOW);
  }
  else {                               // turn on green LED
    digitalWrite(RLEDPIN, LOW);
    digitalWrite(YLEDPIN, LOW);
    digitalWrite(GLEDPIN, HIGH);
  }
}

/* Function to compare Lidar and IMU measurements */
void compareSensorData() {
  JY901.receiveSerialData();
  lidar.rangingTest(&measure, false);
  range_mm = measure.RangeMilliMeter;
  
  // Measured angle from IMU
  float measured_angle = round(bal_angle - JY901.getRoll());
  
  // Cosine Rule to calculate the angle from linear Lidar measurement
  float A = sqrt(pow(radius,2) + pow(range_mm + plate_height,2) - 2*radius*(range_mm + plate_height)*cos(0.418879));
  float calculated_angle = acos((pow(A,2) + pow(radius,2) - pow(range_mm + plate_height,2))/(2*A*radius));
  calculated_angle = 66 - calculated_angle * 180 / M_PI;

  // Calculate error
  float error = abs(calculated_angle - measured_angle);
  float percent_err = error / measured_angle * 100;

  Serial.print("Measured Angle:   "); Serial.print(measured_angle); Serial.println();
  Serial.print("Calculated Angle: "); Serial.print(calculated_angle); Serial.println();
  Serial.print("Difference:       "); Serial.print(error); Serial.println();
  Serial.print("Percent Error:    "); Serial.print(percent_err); Serial.print("%"); Serial.println("\n");
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

  // Setting up the ESP32 hosted WiFi
  Serial.println("Setting Access Point…");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  // Displays the IP address for the ESP32 (default: 192.168.4.1)
  Serial.println(IP);
  server.on("/wifi_comms",HTTP_POST,[](AsyncWebServerRequest * request){},
  NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {
    val = String((char *)data_in, len);
    request->send_P(200, "text/plain", "message received");
  });
  server.begin();
}

void loop() {
  if (val != "1") { return; }
  PID();
  ledControl();
  compareSensorData();
  delay(30);
}
