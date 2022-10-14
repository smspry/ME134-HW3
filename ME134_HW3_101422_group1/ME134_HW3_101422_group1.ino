#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <ESP32Servo.h>

///***** You must install the ESP async library for this to function (instructions in class doc)***


// Change the ssid and password to something else 
const char* ssid = "ESP32-team1";
const char* password = "team1"; 

AsyncWebServer server(80);

// set up motor
Servo motor;

int motor_pin = 5;


void setup() {
  Serial.begin(115200); // basic setup for serta communication when ESp32 is connected to pc via USB
  
  // attached the motor to a pin on the ESP32
  motor.attach(motor_pin);
  pinMode(motor_pin, OUTPUT);

  // Setting up the ESP32 Hosted wifi (IE you connect to a network being produced by the ESP32) 
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: "); Serial.println(IP); // displays the IP address for the ESP32, by default it is 192.168.4.1
  // you can confirm the IP by plugging in the ESP32, opening the serial monitor then resetting the ESP32, the above lines print the IP address
  
  server.on("/wifi_comms",HTTP_POST,[](AsyncWebServerRequest * request){},
  NULL,[](AsyncWebServerRequest * request, uint8_t *data_in, size_t len, size_t index, size_t total) {
    String val = String((char *)data_in, len);
    if(val == "1"){
      request->send_P(200, "text/plain", balancing_function().c_str());
    }
    if(val == "0"){
      request->send_P(200, "text/plain", stop_balancing_function().c_str());      
    }
    
    
  });

  // Start server (needed)
  server.begin();

}

void loop() { 

}

// some function for balancing the robot
String balancing_function(){


  return String("Robot is actively balancing");
  
}

// function to stop actively balancing the robot
String stop_balancing_function(){

  
  return String("Robot stopped balancing");
  
}

