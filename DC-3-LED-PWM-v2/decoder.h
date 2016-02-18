/*
  decoder.h - class to decode an array of measured currents which encode 0 or more orthogonal codes.
  Created by Johnny Verhoeff - 18-2-2016
*/

#ifndef DECODER_H
#define DECODER_H

#include <stdint.h>
#include "hadamard_matrix_generator.h"
#include "Arduino.h"

#define IDEAL_1_LED_ADC_VAL 200
#define MIN_1_LED_ADC_VAL 180
#define MAX_1_LED_ADC_VAL 220

#define ENCODED_UNUSED 1
#define ENCODED_0 2
#define ENCODED_1 0

#define LOGICAL_UNUSED 2
#define LOGICAL_0 0
#define LOGICAL_1 1

class decoder {

public:
  decoder(const uint8_t adc_pin, const uint8_t data_length, hadamard_matrix_generator *hmg);

  void measure(void);

  uint8_t is_decoded_data_ready(void);
  uint8_t **get_decoded_led_data(void);

private:
  const uint8_t _adc_pin;

  hadamard_matrix_generator *_hmg;

  const uint8_t _data_length;

  uint16_t *_measured_data;
  uint8_t _code_bit_position;
  uint8_t _data_bit_position;
  uint8_t **_decoded_led_data;

  uint8_t _data_ready;

  void _initialize_decoded_matrix(void);
  uint8_t _from_encoded_val_to_logical_val(uint8_t encoded_val);
  void _decode_led_data(void);

  uint16_t _clamp_measurements(uint16_t m);
};


#endif // DECODER_H
