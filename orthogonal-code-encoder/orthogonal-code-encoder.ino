#include "encoder.h"

#define DATA_LENGTH 3
#define CODE_LENGTH 4

uint8_t code[] = {0, 1, 1, 0};

uint8_t data[] = {0, 1, 0};

encoder enc(code, CODE_LENGTH, data, DATA_LENGTH);

void setup() {
  Serial.begin(115200);
  Serial.println("Begun");
}

void loop() {
  for (uint8_t i = 0; i < DATA_LENGTH * CODE_LENGTH; i++) {
    Serial.print(enc.get_next_encoded_bit());
    Serial.print(" ");
  }
  Serial.println();
  delay(1000);
}
