#define SIZE 15


uint32_t *time_buffer;
uint16_t *adc_buffer;



void setup() {
  Serial.begin(250000);


  time_buffer = new uint32_t[SIZE];
  adc_buffer = new uint16_t[SIZE];
  
}


uint32_t t = 0;

void loop() {
  for (uint32_t i = 0; i < SIZE; i++) {
    uint32_t it = micros();

    
    //digitalWrite(5, digitalRead(5) ^ 1);

    //time_buffer[i] = micros();
    //adc_buffer[i] = analogRead(A0);
    Serial.print(time_buffer[0]); Serial.print(": "); Serial.println(adc_buffer[0]);
    t += (micros() - it);
  }

  Serial.println();
  Serial.println((t / SIZE));
  t = 0;

  delay(5000);
  Serial.println();

  
}
