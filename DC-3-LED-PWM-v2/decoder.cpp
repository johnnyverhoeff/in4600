#include "decoder.h"

decoder::decoder(
  const uint8_t adc_pin,
  const uint8_t data_length,
  hadamard_matrix_generator *hmg
):
  _adc_pin(adc_pin),
  _hmg(hmg),
  _data_length(data_length) {

  _measured_data = new uint16_t[_hmg->get_code_length()];

  _code_bit_position = 0;
  _data_bit_position = 0;

  _data_ready = 0;

  _white_list = new uint8_t[_hmg->get_code_length()];
  _black_list = new uint8_t[_hmg->get_code_length()];

  _initialize_decoded_matrix();
  _init_access_lists();
}

void decoder::_init_access_lists(void) {
  for (uint8_t i = 0; i < _hmg->get_code_length(); i++) {
    _white_list[i] = 1;
    _black_list[i] = 0;
  }
}

void decoder::_initialize_decoded_matrix(void) {

  _decoded_led_data = new uint8_t*[_hmg->get_code_length()];
  for (uint8_t code = 0; code < _hmg->get_code_length(); code++)
    _decoded_led_data[code] = new uint8_t[_data_length];

  for (uint8_t code = 0; code < _hmg->get_code_length(); code++) {
    for (uint8_t d = 0; d < _data_length; d++) {
      _decoded_led_data[code][d] = LOGICAL_UNUSED;
    }
  }
}

uint16_t decoder::_clamp_measurements(uint16_t m) {
  for (uint8_t i = 0; i < _hmg->get_code_length(); i++) {
    if (m >= i * MIN_1_LED_ADC_VAL && m <= i * MAX_1_LED_ADC_VAL)
      return i * IDEAL_1_LED_ADC_VAL;
  }

  return m;
}

void decoder::measure(void) {
  _measured_data[_code_bit_position++] = _clamp_measurements(analogRead(_adc_pin));

  if (_code_bit_position >= _hmg->get_code_length()) {
    _code_bit_position = 0;

    _decode_led_data();

    _data_bit_position++;
    if (_data_bit_position >= _data_length) {
      _data_bit_position = 0;
      _data_ready = 1;

    }



  }
}

uint8_t decoder::is_decoded_data_ready(void) {
  if (_data_ready) {
    _data_ready = 0;
    return 1;
  } else
    return 0;
}

uint8_t** decoder::get_decoded_led_data(void) {
  return _decoded_led_data;
}

void decoder::_decode_led_data(void) {

  for (uint8_t code = 0; code < (_hmg->get_code_length() - 1); code++) {

    int32_t correlation = 0;
    
    for (uint8_t chip = 0; chip < _hmg->get_code_length(); chip++) {

      int8_t radio_rep = 1 - 2 * _hmg->get_code_matrix()[code + 1][chip];

      int32_t corr_per_chip = ((int16_t)radio_rep * (int16_t)_measured_data[chip]);
      correlation += corr_per_chip;
     
    }

    correlation = -2 * correlation / IDEAL_1_LED_ADC_VAL;

    Serial.print("correlation: "); Serial.println(correlation);

    int8_t encoded_val = 2;
    
    if (correlation > (0.5 * _hmg->get_code_length()))
      encoded_val = 0;
    else if (correlation < (-0.5 * _hmg->get_code_length()))
      encoded_val = 1;
    else
      encoded_val = 2;
    
    
  
    _decoded_led_data[code][_data_bit_position] = encoded_val;
  }

}

uint8_t decoder::_from_encoded_val_to_logical_val(int8_t encoded_val) {

  if (encoded_val == ENCODED_UNUSED)
    return LOGICAL_UNUSED;
  else if (encoded_val >= ENCODED_0)
    return LOGICAL_0;
  else if (encoded_val <= ENCODED_1)
    return LOGICAL_1;

}

uint8_t decoder::is_code_white_listed(uint8_t code_number) {
  if (code_number < _hmg->get_code_length()) {
    return _white_list[code_number];
  } else {
    return 0;
  }
}

uint8_t decoder::is_code_black_listed(uint8_t code_number) {
  if (code_number < _hmg->get_code_length()) {
    return _black_list[code_number];
  } else {
    return 0;
  }
}

void decoder::black_list_code(uint8_t code_number) {
  if (code_number < _hmg->get_code_length()) {
    _black_list[code_number] = 1;
    _white_list[code_number] = 0;
  }
}

