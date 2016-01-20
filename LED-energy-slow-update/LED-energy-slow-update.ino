
uint8_t code_0[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t code_1[] = {0, 0, 0, 0, 0, 0, 0, 1};
uint8_t code_2[] = {0, 0, 0, 0, 0, 0, 1, 1};
uint8_t code_3[] = {0, 0, 0, 0, 0, 1, 1, 1};
uint8_t code_4[] = {0, 0, 0, 0, 1, 1, 1, 1};
uint8_t code_5[] = {0, 0, 0, 1, 1, 1, 1, 1};
uint8_t code_6[] = {0, 0, 1, 1, 1, 1, 1, 1};
uint8_t code_7[] = {0, 1, 1, 1, 1, 1, 1, 1};
uint8_t code_8[] = {1, 1, 1, 1, 1, 1, 1, 1};


uint8_t *codes[] = {code_0, code_1, code_2, code_3, code_4, code_5, code_6, code_7, code_8};
uint8_t num_of_codes = 9;

uint8_t *code;
uint8_t current_code = 0;

uint8_t code_size = 8;

uint8_t ledPin = 3;

uint8_t enableTimer = 0;
uint8_t counter = 0;

unsigned long start_modulating_time;
unsigned long start_time;


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

ISR(TIMER1_COMPA_vect) {          // timer compare interrupt service routine

  if(enableTimer != 0 && counter < code_size){      
      if (counter == 0) 
        start_time = micros();
      
      digitalWrite(ledPin, code[counter++]);  
  }
}


void setup() {

  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);
    
  Serial.println("Running");

  code = codes[0];
  
  initializeTimer();
  enableTimer = 0;

  digitalWrite(ledPin, HIGH);

  start_time = millis();

}

void loop() {

  if (counter == 0) {
    enableTimer = 1;
  } else if (counter == code_size) {
    
    enableTimer = 0;
    digitalWrite(ledPin, HIGH);
    
    unsigned long modulating_time = micros() - start_modulating_time;
    delayMicroseconds(1000000 - modulating_time);
    counter = 0;
   
  }

  if (Serial.available() > 0) {
    current_code = Serial.read() - '0';

    //current_code += 1;
   
    if (current_code >= num_of_codes)
      current_code = 0;

    Serial.print("Current code: ");
    Serial.println(current_code);

    code = codes[current_code];
  }

}
