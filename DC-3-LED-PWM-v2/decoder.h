/*
  decoder.h - class to decode an array of measured currents which encode 0 or more orthogonal codes.
  Created by Johnny Verhoeff - 18-2-2016
*/

#ifndef DECODER_H
#define DECODER_H

#include <stdint.h>
#include "hadamard_matrix_generator.h"
#include "Arduino.h"

#define LOGICAL_UNUSED 2

class decoder {

public:
  decoder(const uint8_t adc_pin, const uint8_t data_length, hadamard_matrix_generator *hmg);

  void measure(void);

  uint8_t is_decoded_data_ready(void);
  uint8_t **get_decoded_led_data(void);

  uint8_t is_code_white_listed(uint8_t code_number);
  uint8_t is_code_black_listed(uint8_t code_number);

  void black_list_code(uint8_t code_number);

private:
  const uint8_t _adc_pin;

  hadamard_matrix_generator *_hmg;

  const uint8_t _data_length;

  uint16_t *_measured_data;
  uint8_t _code_bit_position;
  uint8_t _data_bit_position;
  uint8_t **_decoded_led_data;

  uint8_t _data_ready;

  uint8_t *_white_list;
  uint8_t *_black_list;

  void _init_access_lists(void);

  void _initialize_decoded_matrix(void);
  void _decode_led_data(void);

};


#endif // DECODER_H
