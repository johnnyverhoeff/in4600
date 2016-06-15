#define led 2
#define modulate_enable_pin 3

#define TIMER_FREQ 1000

volatile uint8_t i;

volatile uint16_t timer1_counter;

#define ENC_PER_PERIOD 6
#define CODE_SIZE 10

//uint8_t code[CODE_SIZE] = {0, 0, 1, 1, 0, 1, 0, 1, 0, 1};

uint8_t code[CODE_SIZE] = {0, 0, 1, 0, 0, 1, 0, 0, 1, 1};


volatile uint8_t code_ptr = 0;

volatile uint8_t timer_enable;

#define ADC_BUFF_SIZE 200

volatile uint16_t *adc_buffer;
volatile uint16_t adc_ptr;

volatile uint8_t dump_flag;

void setup() {
  pinMode(led, OUTPUT);
  pinMode(modulate_enable_pin, INPUT);

  Serial.begin(250000);

  attachInterrupt(digitalPinToInterrupt(modulate_enable_pin), isr_change, CHANGE );

  digitalWrite(led, HIGH);

  i = 0;

  timer_enable = 0;

  adc_buffer = new uint16_t[ADC_BUFF_SIZE];
  adc_ptr = 0;

  dump_flag = 0;





  

  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  timer1_counter = /*(1 << 16)*/ 65536 - (16 * 1000000 / 256 / TIMER_FREQ);
 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts




  

}

ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  if (i < ENC_PER_PERIOD && timer_enable == 1) {
    i++;
    digitalWrite(led, 0);//code[code_ptr++]);
    if (code_ptr > CODE_SIZE)
      code_ptr = 0;
    //digitalWrite(led, digitalRead(led) ^ 1);

    adc_buffer[adc_ptr++] = analogRead(A0);

    if (adc_ptr >= ADC_BUFF_SIZE) {
      dump_flag = 1;
      adc_ptr = 0;
    }
  } else {
    digitalWrite(led, HIGH);
  }

  
  
}

void isr_change(void) {
  if (digitalRead(modulate_enable_pin) == 0) {
    i = 0;
    //delayMicroseconds(100);
    timer_enable = 1;
  } else {
    timer_enable = 0;
  }

  
}


void loop() {
  /*if (dump_flag == 1) {
    noInterrupts();
    digitalWrite(led, HIGH);
    dump_flag = 0;


    Serial.println("*************");
    for (int idx = 0; idx < ADC_BUFF_SIZE; idx++) {
      Serial.println(adc_buffer[idx]);
    }

    
    interrupts();
  }*/
}


