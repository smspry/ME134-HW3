/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

// Motor A
int motor1Pin1 = 26;        // out1
int motor1Pin2 = 25;        // out2
int enable1Pin = 27;       

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);    // out1
  pinMode(motor1Pin2, OUTPUT);    // out2
  pinMode(enable1Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  Serial.begin(115200);

  // testing
  Serial.print("Testing DC Motor...");
  ledcWrite(pwmChannel, dutyCycle); 

}

void loop() {
  // Move the DC motor forward at maximum speed
  Serial.println("Moving Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  delay(2000);
}
