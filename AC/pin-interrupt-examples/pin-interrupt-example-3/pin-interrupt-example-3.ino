#define led 13
#define interrupt_pin 3

void setup() {
  pinMode(led, OUTPUT);
  pinMode(interrupt_pin, INPUT);

  Serial.begin(250000);

  digitalWrite(led, HIGH);

}

void loop() {

  digitalWrite(led, digitalRead(interrupt_pin));
}






