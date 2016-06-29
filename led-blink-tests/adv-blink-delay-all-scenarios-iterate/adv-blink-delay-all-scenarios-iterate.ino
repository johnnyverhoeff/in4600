#define led 13

#define NUM_TIMES_MODULATE 4


void setup() {
  Serial.begin(250000);

  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  


}

void loop() {

  for (int i = 0; i < (1 << NUM_TIMES_MODULATE); i++) {

    Serial.print("i: "); Serial.print(i); Serial.print(" -> ");
    for (int k = 0; k < NUM_TIMES_MODULATE; k++) {
      Serial.print((i & (1 << k)) > 0); Serial.print(" ");
    }
    Serial.println();
    delay(2000);

    for (int j = 0; j < 100; j++) {
      digitalWrite(led, 1);
      delay(5);
    
      for (int k = 0; k < NUM_TIMES_MODULATE; k++) {
        digitalWrite(led, (i & (1 << k)) > 0);
        delayMicroseconds(1000);
      }

      digitalWrite(led, 1);
      delayMicroseconds(1000);
        
    }
  }
  

}
