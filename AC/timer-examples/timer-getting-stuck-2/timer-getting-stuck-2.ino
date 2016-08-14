#define modulate_enable 3
#define led 13

#define TIME_TO_MODULATE 7 // in ms
#define TIMER_FREQ 15000 //Hz
#define MODULATE_TIMES (TIME_TO_MODULATE * (TIMER_FREQ / 1000) - 1)

volatile uint16_t timer1_counter;

volatile uint8_t modulate_idx = 0;

uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1

uint8_t n = sizeof(poly) / sizeof(uint8_t) - 1;
uint16_t L = (1 << n) - 1;
uint16_t N = (1 << n) + 1;

uint8_t *m_seq;
volatile uint16_t m_seq_idx;



volatile uint8_t timer_enable = 0;

volatile uint8_t done_modulating_one_seq = 0;

volatile uint8_t modulate_enable_flag = 0;


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


void enable_timer() {
  TCNT1 = 0;
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

void disable_timer() {
  TIMSK1 &= ~(1 << TOIE1); 
}

void setup() {
  Serial.begin(250000);

  pinMode(modulate_enable, INPUT);

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  m_seq = new uint8_t[L];
  m_seq_idx = 0;
  m_seq_create(poly, n, 1, m_seq);


  modulate_idx = 0;

  // initialize timer1 -
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  timer1_counter =  65536 - (16 * 1000000 / 256 / TIMER_FREQ);

 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  disable_timer();
  

    
  
  attachInterrupt(digitalPinToInterrupt(modulate_enable), isr_change, CHANGE );

  interrupts();       
}

ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  PORTD ^= (1 << 7);
  
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
 
}


void isr_change(void) {
    int state = digitalRead(modulate_enable);

    PORTD ^= (1 << 6);
  
    if (state == 1) {
      modulate_idx = 0;
      disable_timer(); 
      digitalWrite(led, HIGH);
    } else {
      enable_timer();
    }
}


void loop() {  

}


