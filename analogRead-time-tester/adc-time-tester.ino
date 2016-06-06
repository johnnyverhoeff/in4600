void setup() {
  Serial.begin(115200);
}

uint32_t t = 0;

void loop() {
  for (uint32_t i = 0; i < 10000; i++) {
    uint32_t it = micros();
    analogRead(A0);
    t += (micros() - it);
  }

  Serial.println((t / 10000));
  t = 0;
}
