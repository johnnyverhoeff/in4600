#include "hadamard_matrix_generator.h"

hadamard_matrix_generator::hadamard_matrix_generator(uint8_t start_chip, const uint8_t code_length):
  _code_length(code_length), _start_chip(start_chip)
{

  uint8_t** matrix = new uint8_t*[code_length];
  for (uint8_t i = 0; i < code_length; i++)
    matrix[i] = new uint8_t[code_length];

  matrix[0][0] = start_chip;

  /*
  inspiration taken from:
  http://stackoverflow.com/questions/18604659/hadamard-matrix-code
  */

  for (int i = 2; i <= code_length; i *= 2) {
    for (int x = 0; x < (i / 2); x++) {
      for (int y = i / 2; y < i; y++) {
        matrix[x][y] = matrix[x][y - i / 2];
      }
    }

    for (int y = 0; y < (i / 2); y++) {
      for (int x = i / 2; x < i; x++) {
        matrix[x][y] = matrix[x - (i / 2)][y];
      }
    }

    for (int x = i / 2; x < i; x++) {
      for (int y = i / 2; y < i; y++){
        matrix[x][y] = !matrix[x - i / 2][y - i / 2];

      }
    }
  }

  _code_matrix = matrix;
}

uint8_t** hadamard_matrix_generator::get_code_matrix(void) {
  return _code_matrix;
}

uint8_t hadamard_matrix_generator::get_start_chip(void) {
  return _start_chip;
}

const uint8_t hadamard_matrix_generator::get_code_length(void) {
  return _code_length;
}
