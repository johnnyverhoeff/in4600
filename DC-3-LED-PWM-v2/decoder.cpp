#include "decoder.h"

decoder::decoder(
  const uint8_t adc_pin,
  const uint8_t data_length,
  hadamard_matrix_generator *hmg
):
  _adc_pin(adc_pin),
  _hmg(hmg),
  _data_length(data_length) {

  _measured_data = new uint8_t[_hmg->get_code_length()];

  _code_bit_position = 0;
  _data_bit_position = 0;

  _initialize_decoded_matrix();
}

void decoder::_initialize_decoded_matrix(void) {

  _decoded_led_data = new uint8_t*[_hmg->get_code_length()];
  for (uint8_t code = 0; code < _hmg->get_code_length(); code++)
    _decoded_led_data[code] = new uint8_t[_data_length];

  for (uint8_t code = 0; code < _hmg->get_code_length(); code++) {
    for (uint8_t d = 0; d < _hmg->get_code_length(); d++) {
      _decoded_led_data[code][d] = LOGICAL_UNUSED;
    }
  }
}

void decoder::measure(void) {
  _measured_data[_code_bit_position++] = analogRead(_adc_pin);

  if (_code_bit_position >= _hmg->get_code_length()) {
    _code_bit_position = 0;

    _decode_led_data();

    _data_bit_position++;
    if (_data_bit_position >= _data_length) {
      _data_bit_position = 0;

      // can show complete result now;
      for (uint8_t code = 0; code < _hmg->get_code_length(); code++) {
        Serial.print("code "); Serial.print(code); Serial.print(": ");
        for (uint8_t d = 0; d < _data_length; d++) {
          Serial.print(_decoded_led_data[code][d]);
          Serial.print(" ");
        }
        Serial.println();
      }
      Serial.println();
    }
  }
}

void decoder::_decode_led_data(void) {
  uint32_t sum_of_measured_vals = 0;
  for (uint8_t val = 0; val < _hmg->get_code_length(); val++)
    sum_of_measured_vals += _measured_data[val];

  uint8_t calculated_num_of_leds = sum_of_measured_vals / IDEAL_1_LED_ADC_VAL / (_hmg->get_code_length() / 2);

  for (uint8_t code = 0; code < _hmg->get_code_length(); code++) {
    uint32_t avg_val = 0;

    for (uint8_t chip = 0; chip < _hmg->get_code_length(); chip++) {
      uint32_t orthogonal_code_representation = _hmg->get_code_matrix()[code][chip] * IDEAL_1_LED_ADC_VAL;
      uint32_t product = orthogonal_code_representation * _measured_data[chip];
      avg_val += product;
    }

    avg_val /= _hmg->get_code_length();

    // TODO: figure out WHY 10000
    uint8_t encoded_val = (avg_val / 10000) - (calculated_num_of_leds - 1);

    _decoded_led_data[code][_data_bit_position] = _from_encoded_val_to_logical_val(encoded_val);
  }
}

uint8_t decoder::_from_encoded_val_to_logical_val(uint8_t encoded_val) {
  switch (encoded_val) {
    case ENCODED_UNUSED:
      return LOGICAL_UNUSED;

    case ENCODED_0:
      return LOGICAL_0;

    case ENCODED_1:
      return LOGICAL_1;

    default:
      return LOGICAL_UNUSED;
  }
}
