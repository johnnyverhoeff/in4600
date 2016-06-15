#define modulate_enable 3

#define led 8 

void setup() {


  Serial.begin(250000);

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  pinMode(modulate_enable, INPUT);

}

void loop() {
  if (digitalRead(modulate_enable) == 0)
    Serial.println(analogRead(A0));

  
}


