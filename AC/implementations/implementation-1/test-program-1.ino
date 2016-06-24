#define led 8
#define modulate_enable_pin 3

#define OFF LOW
#define ON HIGH

#define TIMER_FREQ 2000 //Hz

#define TIMES 4

#define ADC_BUFFER_SIZE 750
uint16_t *adc_buffer;
uint16_t adc_idx;
uint8_t adc_buffer_full;



volatile uint16_t timer1_counter;
volatile uint8_t timer_enable;

volatile uint8_t ctr = 0;

volatile uint8_t idx = 0;



uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1

uint8_t n = sizeof(poly) / sizeof(uint8_t) - 1;
uint16_t L = (1 << n) - 1;
uint16_t N = (1 << n) + 1;

uint8_t *m_seq;
uint16_t m_seq_idx;

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
  pinMode(led, OUTPUT);
  pinMode(modulate_enable_pin, INPUT);
  
  digitalWrite(led, OFF);

  Serial.begin(250000);

  Serial.println("start!");
  Serial.println("Initializing adc buffer...");

  adc_buffer = new uint16_t[ADC_BUFFER_SIZE];
  adc_idx = 0;
  adc_buffer_full = 0;
  
  Serial.println("Adc buffer initialized, succes.");



  m_seq = new uint8_t[L];
  m_seq_idx = 0;
  m_seq_create(poly, n, 1, m_seq);











  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  timer1_counter = /*(1 << 16)*/ 65536 - (16 * 1000000 / 256 / TIMER_FREQ);

  //Serial.println(TIMER_FREQ);
  //Serial.println(timer1_counter);

 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt


  
  attachInterrupt(digitalPinToInterrupt(modulate_enable_pin), isr_change, CHANGE );

  timer_enable = 0;
  
  interrupts();             // enable all interrupts
}

ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  /*if (timer_enable == 1 && !adc_buffer_full) { 
    adc_buffer[adc_idx++] = analogRead(A0);

    if (adc_idx >= ADC_BUFFER_SIZE)
      adc_buffer_full = 1;
  }*/
    
  
  if (ctr++ % 2 == 0) {  
    if (idx >= TIMES) {
      timer_enable = 0;
    }
  
    if (timer_enable == 1) {
      idx++;
      digitalWrite(led, m_seq[m_seq_idx++]);
      if (m_seq_idx >= L)
        m_seq_idx = 0;

      if (timer_enable == 1 && !adc_buffer_full) { 
        adc_buffer[adc_idx++] = analogRead(A0);
    
        if (adc_idx >= ADC_BUFFER_SIZE)
          adc_buffer_full = 1;
      }
        
    } else {
      digitalWrite(led, ON);
      
    }
  }

}

void isr_change(void) {
  uint8_t st = digitalRead(modulate_enable_pin);
  timer_enable = st ^ 1;
  
  if (st == 0)
    idx = 0;
}



void loop() {
  if (adc_buffer_full == 1) {
    noInterrupts();

    digitalWrite(led, ON);

    timer_enable = 0;

    for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
      Serial.println(adc_buffer[i]);
    }
    Serial.println("*******************");


    /*
    float corr_sum = 0;

    for (int i = 0; i < L; i++) {
      float r_chip = 1 - 2 * ((int8_t)m_seq[i]);

      corr_sum += adc_buffer[i] * r_chip;

      Serial.print(adc_buffer[i]);
      Serial.print(" * ");
      Serial.println(r_chip);
    }

    Serial.println(corr_sum);
    Serial.println("**********");
*/
    adc_idx = 0;
    adc_buffer_full = 0;
    interrupts();
  }
}

