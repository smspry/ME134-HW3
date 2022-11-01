#define RLEDPIN 32
#define YLEDPIN 33
#define GLEDPIN 25

void setup() {
  pinMode(RLEDPIN, OUTPUT);
  pinMode(YLEDPIN, OUTPUT);
  pinMode(GLEDPIN, OUTPUT);
  digitalWrite(RLEDPIN, LOW);
  digitalWrite(YLEDPIN, LOW);
  digitalWrite(GLEDPIN, LOW);
}

void loop() {
  digitalWrite(GLEDPIN, LOW);
  digitalWrite(RLEDPIN, HIGH);
  delay(2000);
  digitalWrite(RLEDPIN, LOW);
  digitalWrite(YLEDPIN, HIGH);
  delay(2000);
  digitalWrite(YLEDPIN, LOW);
  digitalWrite(GLEDPIN, HIGH);
  delay(2000);
}
