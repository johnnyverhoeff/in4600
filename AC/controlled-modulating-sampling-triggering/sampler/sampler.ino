#define modulate_enable 3

#define TIMER_FREQ 1000 //Hz
uint16_t timer1_counter;


#define ADC_BUFFER_SIZE 1600
uint16_t *adc_buffer;
uint16_t adc_idx;
uint8_t adc_buffer_full;

uint8_t sample_idx = 0;

uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1

uint8_t n = sizeof(poly) / sizeof(uint8_t) - 1;
uint16_t L = (1 << n) - 1;
uint16_t N = (1 << n) + 1;

uint8_t *m_seq;
uint16_t m_seq_idx;




volatile uint8_t timer_enable = 0;



/*
This function creates an m-sequence with a given polynomial.

call like so:

uint8_t n = 5;
uint16_t L = (1 << n) - 1;
uint8_t p[] = {...};

uint8_t *ms = new uint8_t[L];

m_seq_create(p, n, 1, ms);


*/
void m_seq_create(uint8_t *poly, uint8_t n, uint16_t start_state, uint8_t *m_seq) {
  uint16_t L = (1 << n) - 1;
    
  uint16_t lfsr = start_state;
  uint16_t bit_out;
  uint16_t step_pos = 0;

  
  do {
    bit_out = 0;
    for (uint8_t i = n; i > 0; i--)
      if (poly[i] == 1)
        bit_out ^= (lfsr >> (n-i));
    bit_out &= 1;
    
    lfsr = (lfsr >> 1) | (bit_out << (n-1));
    
    m_seq[step_pos++] = bit_out;
  } while (lfsr != start_state);
  
}




void setup() {


  Serial.begin(250000);

  pinMode(modulate_enable, INPUT);

  adc_buffer = new uint16_t[ADC_BUFFER_SIZE];
  adc_idx = 0;
  adc_buffer_full = 0;

  m_seq = new uint8_t[L];
  m_seq_idx = 0;
  m_seq_create(poly, n, 1, m_seq);

  sample_idx = 0;

  // initialize timer1 -
/*  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  timer1_counter = 65536 - (16 * 1000000 / 256 / TIMER_FREQ);

  Serial.println(TIMER_FREQ);
  Serial.println(timer1_counter);

 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

  timer_enable = 1;
  
  interrupts();             // enable all interrupts
*/

  attachInterrupt(digitalPinToInterrupt(modulate_enable), isr_falling, FALLING );
}

/*
ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  if (timer_enable == 1) {
    if (digitalRead(modulate_enable) == 0 && sample_idx < 4) {
      adc_buffer[adc_idx++] = analogRead(A0);
      sample_idx++;
      if (adc_idx >= ADC_BUFFER_SIZE) {
        adc_idx = 0;
        adc_buffer_full = 1;
        noInterrupts();
        timer_enable = 0;
      }
    }

    if (digitalRead(modulate_enable) == 1) 
      sample_idx = 0;

  }
  
}
*/

void isr_falling(void) {

  for (uint8_t i = 0; i < 4; i++) {
    adc_buffer[adc_idx] = analogRead(A0);

    //Serial.println(adc_buffer[adc_idx]);

    adc_idx++;
    
    if (adc_idx >= ADC_BUFFER_SIZE) {
        adc_idx = 0;
        adc_buffer_full = 1;
        noInterrupts();
    }

    delayMicroseconds(900);
  }

}

void loop() {
  if (adc_buffer_full == 1) {
    //timer_enable = 0; 
    noInterrupts();

    for (int offset = 0; offset < (ADC_BUFFER_SIZE - L); offset++) {

      float corr_sum = 0;

      for (int i = 0; i < L; i++) {
        float r_chip = 1 - 2 * ((int8_t)m_seq[i]);
        corr_sum += (adc_buffer[i + offset] / 1023.0) * r_chip;
      }

      float normalized_l_corr = (-2 * corr_sum - 1);

      Serial.println(normalized_l_corr);

    }

  
    /*for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
      Serial.println(adc_buffer[i]);
    }
    Serial.println("-------------");*/

    /*for (int i = 0; i < L; i++) {
      Serial.println(m_seq[i]);
    }
    Serial.println("-------------");*/
    

    adc_buffer_full = 0;
    //timer_enable = 1;
    sample_idx = 0;
    interrupts();
    
  }



  

}


