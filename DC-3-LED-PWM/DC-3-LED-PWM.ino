uint8_t sensorPin = A0;

uint8_t led_pins[] = {3, 5, 6};
const uint8_t led_size = 3;

uint8_t led_data[] = {
    1,
    0,
    1
  };

uint8_t decoded_led_data[led_size];

const uint8_t code_size = 8;

uint8_t codes[][code_size] = {
    {0, 1, 0, 1, 0, 1, 0, 1},
    {0, 0, 1, 1, 0, 0, 1, 1},
    {0, 1, 1, 0, 0, 1, 1, 0}
  };



uint8_t minVal = 190;
uint8_t maxVal = 215;

uint8_t enableTimer = 0;

uint8_t code_pointer = 0;

uint16_t read_values[code_size];

void initializeTimer() {

  // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  //frequency of preamble
  int freq = 10;
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
  
    uint32_t orthogonal_code_representation[code_size];
    for (int i = 0; i < code_size; i++) {
      orthogonal_code_representation[i] = codes[led][i] * 200;
    }
  
    uint32_t multiply_array[code_size];
  
    for (int i = 0; i < code_size; i++) {
      multiply_array[i] = orthogonal_code_representation[i] * read_values[i];
    }
  
    uint32_t avg_value = 0;
    for (int i = 0; i < code_size; i++) {
      avg_value += multiply_array[i];
    }
    avg_value /= code_size;
  
  
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

    decoded_led_data[led] = avg_value;
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
    

    Serial.println();
    decode_leds();
    Serial.println();

    for (int i = 0; i < led_size; i++) {
      Serial.print("Led "); Serial.print(i); Serial.print(": "); Serial.println(decoded_led_data[i]);
    }

    delay(2000);
    enableTimer = 1;
  }
}

