

#define HI_BYTE(x)  ((x) >> 8)
#define LO_BYTE(x)  ((x) & 0xFF)

void setup() {
  Serial.begin(115200);
}

void loop() {
  
  if (Serial.available() > 0) {
    Serial.read();
    uint16_t adcs = analogRead(A0) + analogRead(A1);
    Serial.write(HI_BYTE(adcs));
    Serial.write(LO_BYTE(adcs));
  }
}
