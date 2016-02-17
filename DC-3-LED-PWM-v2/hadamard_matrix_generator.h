/*
  hadamard_matrix.h - class to create an hadamard_matrix.
  Created by Johnny Verhoeff - 16-2-2016
*/

#ifndef HADAMARD_MATRIX_GENERATOR_H
#define HADAMARD_MATRIX_GENERATOR_H

#include <stdint.h>

class hadamard_matrix_generator {
public:
  hadamard_matrix_generator(uint8_t start_chip, const uint8_t code_length);

  uint8_t** get_code_matrix(void);
  uint8_t get_start_chip(void);
  const uint8_t get_code_length(void);

private:
  uint8_t** _code_matrix;
  uint8_t _start_chip;
  const uint8_t _code_length;
};

#endif // HADAMARD_MATRIX_GENERATOR_H
