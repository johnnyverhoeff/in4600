#define led 2
#define interrupt_pin 3

#define OFF HIGH
#define ON LOW

void setup() {
  pinMode(led, OUTPUT);
  pinMode(interrupt_pin, INPUT);

  //attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr_rising, RISING );
  //attachInterrupt(digitalPinToInterrupt(interrupt_pin), isr_falling, FALLING );
  
  Serial.begin(115200);

  digitalWrite(led, ON);

}

void loop() {

  // LOW for ~2 ms
  // HIGH for ~8 ms
}

void isr_rising(void) {
  digitalWrite(led, ON);
}

void isr_falling(void) {
  digitalWrite(led, OFF);
}




