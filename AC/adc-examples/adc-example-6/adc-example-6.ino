#define modulate_enable 3

#define led 8 

int done_modulating = 0;

void setup() {


  Serial.begin(250000);

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  pinMode(modulate_enable, INPUT);

}

void loop() {
  if (!done_modulating && digitalRead(modulate_enable) == 0) {
    
    for (int i = 0; i < 4; i++) {
      digitalWrite(led, digitalRead(led) ^ 1);
      delayMicroseconds(700);
      Serial.println(analogRead(A0));
      
    }
    done_modulating = 1;
    digitalWrite(led, HIGH);

  }

  while (digitalRead(modulate_enable) == 0);
  done_modulating = 0;
}


