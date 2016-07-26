#define led 13
#define bypass_cap_pin 8
#define interrupt_pin 3


void setup() {
  pinMode(led, OUTPUT);
  pinMode(bypass_cap_pin, OUTPUT);
  pinMode(interrupt_pin, INPUT);
  
  Serial.begin(250000);



}

void loop() {

  digitalWrite(led, HIGH);
  digitalWrite(bypass_cap_pin, LOW);

  delay(1000);

  digitalWrite(led, HIGH);
  digitalWrite(bypass_cap_pin, HIGH);

  delay(1000);

}






