#include "encoder.h"

encoder::encoder(
  uint8_t** orthogonal_code_matrix, uint8_t code_number, const uint8_t code_length,
  uint8_t* data, const uint8_t data_length
): _orthogonal_code(orthogonal_code_matrix[code_number]),
   _code_length(code_length),
   _data(data),
   _data_length(data_length),
   _code_number(code_number) {

  _data_bit_position = 0;
  _code_bit_position = 0;
  
}

uint8_t encoder::get_next_encoded_bit(void) {
  if (_code_bit_position >= _code_length) {

    _code_bit_position = 0;
    _data_bit_position++;

    if (_data_bit_position >= _data_length) {
      _data_bit_position = 0;
    }
  }

  return _orthogonal_code[_code_bit_position++] ^ _data[_data_bit_position];
}

const uint8_t encoder::get_code_length(void) {
  return _code_length;
}

uint8_t* encoder::get_orthogonal_code(void) {
  return _orthogonal_code;
}

const uint8_t encoder::get_data_length(void) {
  return _data_length;
}

uint8_t* encoder::get_data(void) {
  return _data;
}

uint8_t encoder::get_code_number(void) {
  return _code_number;
}

