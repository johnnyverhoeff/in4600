#define led 13

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  Serial.begin(250000);

 
}

void loop() {

  digitalWrite(led, HIGH);
  delayMicroseconds(5000);
  //delay(5);

  for (int i = 0; i < 3; i++) {
    digitalWrite(led, 1 ^ digitalRead(led));
    delayMicroseconds(1000);
    //delay(1);
  }

  digitalWrite(led, HIGH);
  delayMicroseconds(2000);
  
  

  
}

