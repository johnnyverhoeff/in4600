#define led 2
#define interrupt_pin 3

#define OFF LOW
#define ON HIGH

void setup() {
  pinMode(led, OUTPUT);
  pinMode(interrupt_pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr_rising, RISING );
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr_falling, FALLING );
  
  Serial.begin(250000);

  digitalWrite(led, ON);

}

void loop() {


}

void isr_rising(void) {
  digitalWrite(led, OFF);
}

void isr_falling(void) {
  digitalWrite(led, ON);
}




