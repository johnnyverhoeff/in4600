#define sensing_pin 7


void setup() {
  pinMode(sensing_pin, OUTPUT);
  digitalWrite(sensing_pin, LOW);

  Serial.begin(250000);

  

}

void loop() {

  uint32_t t = micros();
  digitalWrite(sensing_pin, HIGH);
  uint16_t adc = analogRead(A0);
  digitalWrite(sensing_pin, LOW);

  
  Serial.print(t); 
  Serial.print(": "); 
  Serial.println(adc);

  
}


