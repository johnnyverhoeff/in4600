
#define NUM_OF_LEDS 2

#define CODE_LENGTH 15

#define SENSOR_PIN A0

#define LOWER_THRESHOLD 6
#define UPPER_THRESHOLD 9

uint8_t pn_codes[][CODE_LENGTH] = {
  {1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0},
  {1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1}
};

uint8_t leds[] = {3, 5, 6};

uint8_t led_data[] = {
  1,
  1
};

uint8_t led_time_shift[] = {
  0,
  1
};

uint16_t *read_values;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Begun.");

  for (uint8_t i = 0; i < NUM_OF_LEDS; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  randomSeed(analogRead(A1));

  read_values = new uint16_t[CODE_LENGTH];


  /*Serial.print("Code with shift: ");Serial.print(led_time_shift[0]);
  Serial.print("=> ");
  for (uint8_t j = 0; j < CODE_LENGTH; j++) {
      Serial.print(pn_code[(j + led_time_shift[0]) % CODE_LENGTH]);
      Serial.print(" ");
      
  }
  Serial.println();*/
}

void randomize_led_data() {
  for (uint8_t i = 0; i < NUM_OF_LEDS; i++) {
    led_data[i] = random(0, 2);
  }
}

void loop() {

  //randomize_led_data();

  Serial.println("**********************");
  for (int i = 0; i < CODE_LENGTH; i++) {

    for (uint8_t led = 0; led < NUM_OF_LEDS; led++) {
      uint8_t code_index = (i + led_time_shift[led]) % CODE_LENGTH;
      digitalWrite(leds[led], led_data[led] ^ pn_codes[led][code_index]);
    }
   
    read_values[i] = analogRead(SENSOR_PIN);    
  }

  decode_led_data();
/*
  Serial.print("Read values: ");
  for (int i = 0; i < CODE_LENGTH; i++) {
    Serial.print(read_values[i]);
    Serial.print(" ");
  }
  Serial.println();*/

  
  Serial.println("**********************");
  delay(1000);
}

void decode_led_data() {

    
  for (uint8_t code = 0; code < NUM_OF_LEDS; code++) {
    for (uint8_t i = 0; i < CODE_LENGTH; i++) {
      // get measured value...
      // xor it with all cyclic shifted version of the pn code.
      // count 1s and 0s per code. ???
      // conclude result per code.
  
      uint8_t num_of_ones = 0;
  
      for (uint8_t j = 0; j < CODE_LENGTH; j++) {
        if ((read_values[j] > 1) ^ pn_codes[code][(i + j) % CODE_LENGTH])
          num_of_ones++;
      }
  
      uint8_t decoded_data = 2; // no encoded data
  
      if (num_of_ones > UPPER_THRESHOLD)
        decoded_data = 1;
      else if (num_of_ones < LOWER_THRESHOLD)
        decoded_data = 0;
  
      if (decoded_data != 2) {
  
        Serial.print("Code "); Serial.print(code); 
        Serial.print(" w/ shift = "); Serial.print(i);
        Serial.print(" decoded data: "); Serial.println(decoded_data);
      }
    }
  }
}

