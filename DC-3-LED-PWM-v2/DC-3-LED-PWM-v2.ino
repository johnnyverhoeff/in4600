#include "encoder.h"
#include "hadamard_matrix_generator.h"
#include "decoder.h"

#define SENSOR_PIN A0
#define NUM_OF_LEDS 3

#define START_CHIP 0
#define CODE_LENGTH 8

#define DATA_LENGTH 8

uint8_t leds[] = {3, 5, 6};
uint8_t led_data[NUM_OF_LEDS][DATA_LENGTH] = {
    {0, 1, 0, 1, 0, 1, 0, 1},
    {0, 0, 1, 1, 0, 0, 1, 1},
    {0, 0, 0, 0, 1, 1, 1, 1}
  };

uint8_t enable_timer = 0;

hadamard_matrix_generator hmg(START_CHIP, CODE_LENGTH);

encoder *encoders[NUM_OF_LEDS];


decoder d(SENSOR_PIN, DATA_LENGTH, &hmg);


void randomize_led_data(void) {
  for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
    uint8_t random_number = 195;//random(0, 128);
    for (uint8_t bit_pos = 0; bit_pos < DATA_LENGTH; bit_pos++) {
      led_data[led][bit_pos] = bitRead(random_number, bit_pos);
    }
  }
}

void clear_led_data(void) {
  for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
    for (uint8_t bit_pos = 0; bit_pos < DATA_LENGTH; bit_pos++) {
      led_data[led][bit_pos] = 0;
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





void setup() {

  randomSeed(analogRead(A1));

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

  Serial.println("total codes per led: ");
  for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
    for (int i = 0; i < CODE_LENGTH * DATA_LENGTH; i++) {
      Serial.print(encoders[led]->get_next_encoded_bit());
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();


  //clear_led_data();
  //randomize_led_data();


  Serial.println("Each led data: ");
  for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
    Serial.print("led"); Serial.print(led); Serial.print(": ");
    for (uint8_t d = 0; d < DATA_LENGTH; d++) {
      Serial.print(led_data[led][d]); Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();


  Serial.println("Each led code: ");
  for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
    Serial.print("led"); Serial.print(led); Serial.print(": ");
    for (uint8_t c = 0; c < CODE_LENGTH; c++) {
      Serial.print(encoders[led]->get_orthogonal_code()[c]); Serial.print(" ");
    }
    Serial.println();
  }

  delay(1000);

  initializeTimer();

  enable_timer = 1;
}

void loop() {



  if (d.is_decoded_data_ready()) {
    enable_timer = 0;

    for (int i = 0; i < NUM_OF_LEDS; i++)
      digitalWrite(leds[i], HIGH);

    Serial.println("Randomized led data: ");
    for (int i = 0; i < NUM_OF_LEDS; i++) {
      Serial.print("LED"); Serial.print(i); Serial.print(": ");
      for (int j = 0; j < DATA_LENGTH; j++) {
        Serial.print(led_data[i][j]); Serial.print(" ");
      }
      Serial.println();
    }
    Serial.println();


    uint8_t** t = d.get_decoded_led_data();
    Serial.println();
    Serial.println("Decoded data: ");
    for (int i = 0; i < (CODE_LENGTH - 0); i++) {
      for (int j = 0; j < DATA_LENGTH; j++) {
        Serial.print(t[i][j]); Serial.print(" ");
      }
      Serial.println();
    }
    Serial.println();


    for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
        for (uint8_t data_bit = 0; data_bit < DATA_LENGTH; data_bit++) {
          if (led_data[led][data_bit] != t[led][data_bit]) {
            Serial.print("Mismatch: ");
            Serial.print("LED: ");
            Serial.print(led);
            Serial.print(", data_bit: ");
            Serial.println(data_bit);
          }
        }
      }



    //randomize_led_data();

    delay(2000);
    enable_timer = 1;
  }
}


ISR(TIMER1_COMPA_vect) {
  // timer compare interrupt service routine
  if(enable_timer){

    for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
      digitalWrite(leds[led], encoders[led]->get_next_encoded_bit());
    }

    d.measure();
  }
}
