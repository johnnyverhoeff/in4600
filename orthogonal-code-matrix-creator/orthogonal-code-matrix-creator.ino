#include "hadamard_matrix_generator.h"

#define START_CHIP 1
#define CODE_LENGTH 8

hadamard_matrix_generator hmg(START_CHIP, CODE_LENGTH);

void print_matrix() {

  for (uint8_t i = 0; i < 2 * CODE_LENGTH - 1; i++)
    Serial.print("*");
  Serial.println();

  for (uint8_t i = 0; i < CODE_LENGTH; i++) {
    for (uint8_t j = 0; j < CODE_LENGTH; j++) {
      Serial.print(hmg.get_code_matrix()[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }

  for (uint8_t i = 0; i < 2 * CODE_LENGTH - 1; i++)
    Serial.print("*");
  Serial.println();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Begun!");

  Serial.print("code_length: "); Serial.println(hmg.get_code_length());
  Serial.print("start_chip: "); Serial.println(hmg.get_start_chip());

  print_matrix();

  Serial.println("Done!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
