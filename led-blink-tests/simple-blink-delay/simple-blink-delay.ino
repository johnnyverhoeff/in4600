#define led 13

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  Serial.begin(250000);

 
}

void loop() {

  digitalWrite(led, HIGH);
  //delayMicroseconds(100000);
  delay(5);

  digitalWrite(led, LOW);
  //delayMicroseconds(100000);
  delay(5);

  
}

