#define led 13

#define NUM_TIMES_MODULATE 4


void setup() {
  Serial.begin(250000);

  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  randomSeed(analogRead(A15));


}

void loop() {


  digitalWrite(led, 1);
  delay(5);
  
  int i = (int)random(0, (1 << NUM_TIMES_MODULATE));
  
  if ((i < 0) || (i > 15)) {
    Serial.println("!!!!");
  }
  
  for (int k = 0; k < NUM_TIMES_MODULATE; k++) {
    digitalWrite(led, (i & (1 << k)) > 0);
    delayMicroseconds(1000);
  }
  
  digitalWrite(led, 1);
  delayMicroseconds(1000);
        
  

}
