uint8_t sensorPin = A0;

uint8_t led_pins[] = {3, 5, 6};
uint8_t led_size = 3;

uint8_t led_data[] = {
    0,
    1,
    1
  };

uint8_t codes[][4] = {
    {0, 1, 0, 1},
    {0, 0, 1, 1},
    {0, 1, 1, 0}
  };

uint8_t code_size = 4;

uint8_t minVal = 190;
uint8_t maxVal = 215;

uint8_t enableTimer = 0;

uint8_t code_pointer = 0;

uint16_t read_values[4];

void initializeTimer() {

  // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  //frequency of preamble
  int freq = 1;
  int value = 16000000 / 256 /  freq / 2 ;

  OCR1A = value;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode

  TCCR1B |= (1 << CS12);    // 256 prescaler
  //TCCR1B |= (1 << CSHIGH12) | (1 << CS10); // 1024 prescaler

  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}


ISR(TIMER1_COMPA_vect) {          // timer compare interrupt service routine
  if(enableTimer != 0 ){      
    for (int i = 0; i < led_size; i++) {
      digitalWrite(led_pins[i], codes[i][code_pointer] ^ led_data[i]);
    }
    //Serial.println(analogRead(sensorPin));
    read_values[code_pointer] = analogRead(sensorPin);
    code_pointer++;
  }
}

void decode_leds() {
  for (int led = 0; led < led_size; led++) {
  
    uint32_t orthogonal_code_representation[4];
    for (int i = 0; i < code_size; i++) {
      orthogonal_code_representation[i] = codes[led][i] * 200;
    }
  
    uint32_t multiply_array[4];
  
    for (int i = 0; i < code_size; i++) {
      multiply_array[i] = orthogonal_code_representation[i] * read_values[i];
    }
  
    uint32_t avg_value = 0;
    for (int i = 0; i < code_size; i++) {
      avg_value += multiply_array[i];
    }
    avg_value /= code_size;
  
  
    Serial.print("led"); Serial.print(led); Serial.print(": ");
    Serial.println(avg_value);
  }
  
  
}


void setup() {
  for (int i = 0; i < led_size; i++) {
    pinMode(led_pins[i], OUTPUT);
    digitalWrite(led_pins[i], LOW);
  }
  
  Serial.begin(115200);
  Serial.println("Begun");

  initializeTimer();

  enableTimer = 1;
}

void loop() {
  if (code_pointer >= code_size) {
    enableTimer = 0;
    code_pointer = 0;
    
    delay(2000);

    for (int i = 0; i < code_size; i++) {
      Serial.println(read_values[i]);
    }
    Serial.println();
    decode_leds();
    Serial.println();
    enableTimer = 1;
  }
}

