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

void decoder::measure(void) {
  _measured_data[_code_bit_position++] = analogRead(_adc_pin);

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

  uint8_t num_of_codes = log(_hmg->get_code_length()) / log(2);
  uint8_t half_code_length = _hmg->get_code_length() / 2;

  for (uint8_t code = 0; code < num_of_codes; code++) {

    uint8_t sum = 0;
    uint8_t code_number = (uint8_t)(pow(2, code) + 0.5);
    
    for (uint8_t chip = 0; chip < _hmg->get_code_length(); chip++) {
      sum += (_hmg->get_code_matrix()[code_number][chip] ^ (_measured_data[chip] > 1));
      // + 0.5 for rounding issues
    }


    uint8_t decoded_val;
    
    if (sum == half_code_length)
      decoded_val = 2;
    else if (sum > half_code_length)
      decoded_val = 1;
    else 
      decoded_val = 0;  

    _decoded_led_data[code_number][_data_bit_position] = decoded_val;
    
    
  }

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

