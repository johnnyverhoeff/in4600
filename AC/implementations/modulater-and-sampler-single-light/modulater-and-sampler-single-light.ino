#define measuring_pin A0
#define modulate_enable_pin 3
#define led 13

#define MODULATE_TIMES 2

#define TIMER_FREQ 1000 //Hz
volatile uint16_t timer1_counter;
volatile uint8_t timer_enable = 0;

volatile uint8_t modulate_idx;

#define ADC_BUFFER_SIZE 1600
uint16_t *adc_buffer;
uint16_t adc_idx;
uint8_t adc_buffer_full;

uint8_t sample_idx;



uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1

uint8_t n = sizeof(poly) / sizeof(uint8_t) - 1;
uint16_t L = (1 << n) - 1;
uint16_t N = (1 << n) + 1;

uint8_t *m_seq;
volatile uint16_t m_seq_idx;


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

/*
 * Set the timer counter to 0 and then enables the timer.
 * So that the timer will exec. the isr immediately.
 */
void enable_timer() {
  TCNT1 = 0;
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

/*
 * Disables the timer
 */
void disable_timer() {
  TIMSK1 &= ~(1 << TOIE1); 
}


void setup() {
  Serial.begin(250000);

  pinMode(modulate_enable_pin, INPUT);

  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  adc_buffer = new uint16_t[ADC_BUFFER_SIZE];
  adc_idx = 0;
  adc_buffer_full = 0;

  for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
    adc_buffer[i] = 0;
  }

  m_seq = new uint8_t[L];
  m_seq_idx = 0;
  m_seq_create(poly, n, 1, m_seq);

  sample_idx = 0;
  modulate_idx = 0;

  // initialize timer1 -
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  timer1_counter = 65536 - (16 * 1000000 / 256 / TIMER_FREQ);
 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  disable_timer();
  
  interrupts();             // enable all interrupts

  attachInterrupt(digitalPinToInterrupt(modulate_enable_pin), modulate_enable_state_change, CHANGE );

  /*Serial.print("Timer F: "); Serial.println(TIMER_FREQ);
  Serial.print("Timer Threshold: "); Serial.println(timer1_counter);

  Serial.println(" --------- " );

  Serial.print("m_seq: ");
  for (int i = 0; i < L; i++) {
    Serial.print(m_seq[i]); Serial.print(" ");
  }
  Serial.println();*/

}


ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  if (modulate_idx < MODULATE_TIMES) {
    digitalWrite(led, m_seq[m_seq_idx]);
    
    m_seq_idx++;
    modulate_idx++;

    if (m_seq_idx >= L) {
      m_seq_idx = 0;
    }
  } else {
    digitalWrite(led, HIGH);
  }

  if (sample_idx < MODULATE_TIMES) {
    adc_buffer[adc_idx++] = analogRead(measuring_pin);
    sample_idx++;
    if (adc_idx >= ADC_BUFFER_SIZE) {
      adc_idx = 0;
      adc_buffer_full = 1;
      noInterrupts();
    }
  }

  

  
}


void modulate_enable_state_change(void) {
  uint8_t enable_state = digitalRead(modulate_enable_pin);

  if (enable_state == 1) {
    sample_idx = 0;
    modulate_idx = 0;
    
    disable_timer();

    digitalWrite(led, HIGH);
  } else {
    //delayMicroseconds(200); // less than 1 ms seems to work because that is the symbol length
    enable_timer();
  }

}



void loop() {
  if (adc_buffer_full == 1) {
    noInterrupts();

    digitalWrite(led, HIGH);
  
    //long start_time = micros();

    uint16_t sample_min = find_min(adc_buffer, ADC_BUFFER_SIZE);
    uint16_t sample_max = find_max(adc_buffer, ADC_BUFFER_SIZE);


    for (int offset = 0; offset < (ADC_BUFFER_SIZE - L); offset++) {

      float corr_sum = 0;
      float signal_sum = 0;

      for (int i = 0; i < L; i++) {

        float scaled_sample = (adc_buffer[i + offset] - sample_min) / sample_max;
        signal_sum += scaled_sample;
        
        float r_chip = 1 - 2 * ((int8_t)m_seq[i]);
        corr_sum += (scaled_sample) * r_chip;

      }

      float calc_num_of_tx = signal_sum / (1 << (n - 1));
      //Serial.println(calc_num_of_tx);

      float normalized_l_corr = (-2 * corr_sum) - calc_num_of_tx;
      Serial.println(normalized_l_corr);

    }

    //long stop_time = micros();
    //Serial.println(stop_time - start_time);
    //Serial.println((stop_time - start_time) / (ADC_BUFFER_SIZE / L));

  
    /*for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
      Serial.println(adc_buffer[i]);
    }
    Serial.println("-------------");*/

    /*for (int i = 0; i < L; i++) {
      Serial.println(m_seq[i]);
    }
    Serial.println("-------------");*/
    

    adc_buffer_full = 0;
    sample_idx = 0;
    interrupts();
    
  }
}



uint16_t find_min(uint16_t *array, uint16_t length) {
  uint16_t min = 1 << 15;

  for (uint16_t i = 0; i < length; i++) {
    if (array[i] < min)
      min = array[i];
  }

  return min;
}

uint16_t find_max(uint16_t *array, uint16_t length) {
  uint16_t max = 0;

  for (uint16_t i = 0; i < length; i++) {
    if (array[i] > max)
      max = array[i];
  }

  return max;
}

