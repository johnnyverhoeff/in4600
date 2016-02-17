#include "encoder.h"
#include "hadamard_matrix_generator.h"

#define SENSOR_PIN A0
#define NUM_OF_LEDS 3

#define START_CHIP 0
#define CODE_LENGTH 4

#define DATA_LENGTH 8

uint8_t leds[] = {3, 5, 6};
uint8_t led_data[NUM_OF_LEDS][DATA_LENGTH]/* = {
    {0, 0, 1},
    {0, 1, 0},
    {1, 0, 1}
  }*/;

uint8_t enableTimer = 0;

hadamard_matrix_generator hmg(START_CHIP, CODE_LENGTH);

encoder *encoders[NUM_OF_LEDS];






uint8_t code_bit_counter = 0;
uint8_t data_bit_counter = 0;

uint8_t minVal = 190;
uint8_t maxVal = 215;

uint16_t read_values[CODE_LENGTH];

uint8_t decoded_led_data[NUM_OF_LEDS][DATA_LENGTH];


void randomize_led_data(void) {
  for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
    uint8_t random_number = random(0, 256);
    for (uint8_t bit_pos = 0; bit_pos < DATA_LENGTH; bit_pos++) {
      led_data[led][DATA_LENGTH - bit_pos] = bitRead(random_number, bit_pos);
    }
  }
}

void initializeTimer() {

  // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  //frequency of preamble
  int freq = 3000;
  int value = 16000000 / 256 /  freq / 2 ;

  OCR1A = value;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode

  TCCR1B |= (1 << CS12);    // 256 prescaler
  //TCCR1B |= (1 << CSHIGH12) | (1 << CS10); // 1024 prescaler

  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

void decode_leds() {
  for (int led = 0; led < NUM_OF_LEDS; led++) {
    uint32_t avg_value = 0;
    
    for (int i = 0; i < CODE_LENGTH; i++) {
      uint32_t orthogonal_code_representation = encoders[led]->get_orthogonal_code()[i] * 200;
      uint32_t product = orthogonal_code_representation * read_values[i];
      avg_value += product;
    }
    avg_value /= CODE_LENGTH;


    //Serial.print("led"); Serial.print(led); Serial.print(": ");
    //Serial.println(avg_value);

    if (avg_value >= 19000 && avg_value <= 21000)
      avg_value = 20000;
    else if (avg_value >= 29000 && avg_value <= 31000)
      avg_value = 30000;
    else if (avg_value >= 39000 && avg_value <= 41000)
      avg_value = 40000;

    avg_value = ((avg_value / 200) % 200);

    if (avg_value == 150) avg_value = 2;
    else avg_value /= 100;

    decoded_led_data[led][data_bit_counter] = avg_value;
  }
}

void setup() {
  for (uint8_t i = 0; i < NUM_OF_LEDS; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  Serial.begin(115200);
  Serial.println("Begun");

  for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
    encoders[led] = new encoder(
      hmg.get_code_matrix()[1 + led], 
      CODE_LENGTH, 
      led_data[led], 
      DATA_LENGTH
    );
  }

  randomize_led_data();
  
/*
  for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
    Serial.print("led"); Serial.print(led); Serial.print(": ");
    for (uint8_t c = 0; c < CODE_LENGTH; c++) {
      Serial.print(encoders[led]->get_orthogonal_code()[c]); Serial.print(" ");
    }
    Serial.println();
  }
  
  delay(1000);
*/
  initializeTimer();

  enableTimer = 1;
}

void loop() {
  if (code_bit_counter >= CODE_LENGTH) {
    enableTimer = 0;
    code_bit_counter = 0;
    
    

    decode_leds();


    data_bit_counter++;
    if (data_bit_counter >= DATA_LENGTH) {
      data_bit_counter = 0;
    
      for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
        for (uint8_t data_bit = 0; data_bit < DATA_LENGTH; data_bit++) {
          if (led_data[led][data_bit] != decoded_led_data[led][data_bit]) {
            Serial.print("Mismatch: "); 
            Serial.print("LED: "); 
            Serial.print(led);
            Serial.print(", data_bit: ");
            Serial.println(data_bit);
          }
        }
      }

      /*
      for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
        Serial.print("Led "); Serial.print(led); Serial.print(": "); 
        for (uint8_t d = 0; d < DATA_LENGTH; d++) {
          Serial.print(decoded_led_data[led][d]);
          Serial.print(" ");
        }
        Serial.println();
      }
      Serial.println();
      delay(2000);
      */

      randomize_led_data();
    }

    
    enableTimer = 1;
  }
}


ISR(TIMER1_COMPA_vect) {
  // timer compare interrupt service routine
  if(enableTimer){

    for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
      digitalWrite(leds[led], encoders[led]->get_next_encoded_bit());
    }

    read_values[code_bit_counter] = analogRead(SENSOR_PIN);
    code_bit_counter++;
  }
}
