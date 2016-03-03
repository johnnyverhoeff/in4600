/*
  encoder.h - class to encode an array of bits with an orthogonal code.
  Created by Johnny Verhoeff - 16-2-2016
*/

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

class encoder {

public:
  encoder(
    uint8_t** orthogonal_code_matrix,
    uint8_t code_number,
    const uint8_t code_length,
    uint8_t* data,
    const uint8_t data_length
  );

  uint8_t get_next_encoded_bit(void);

  const uint8_t get_code_length(void);
  uint8_t get_code_number(void);
  uint8_t* get_orthogonal_code(void);

  const uint8_t get_data_length(void);
  uint8_t* get_data(void);

private:
  uint8_t _data_bit_position;
  uint8_t _code_bit_position;

  uint8_t* _orthogonal_code;
  uint8_t _code_number;
  const uint8_t _code_length;

  uint8_t* _data;
  const uint8_t _data_length;
};


#endif //ENCODER_H
