#define led 13
#define bypass_cap_pin 8
#define interrupt_pin 3


void setup() {
  pinMode(led, OUTPUT);
  pinMode(bypass_cap_pin, OUTPUT);
  pinMode(interrupt_pin, INPUT);
  
  Serial.begin(250000);

  digitalWrite(bypass_cap_pin, HIGH);

}

void loop() {
  if (digitalRead(interrupt_pin) == 0) {
    digitalWrite(led, HIGH);
    delayMicroseconds(1000);
    digitalWrite(led, LOW);
    delayMicroseconds(1000);
  }

}






