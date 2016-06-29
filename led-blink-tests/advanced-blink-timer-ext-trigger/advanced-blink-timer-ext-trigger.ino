#define modulate_enable 3
#define led 13

#define TIMER_1_FREQ 1000 //Hz
#define TIMER_3_FREQ 1000 //Hz

volatile uint16_t timer1_counter;
volatile uint16_t timer3_counter;

volatile uint8_t modulate_idx = 0;

uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1

uint8_t n = sizeof(poly) / sizeof(uint8_t) - 1;
uint16_t L = (1 << n) - 1;
uint16_t N = (1 << n) + 1;

uint8_t *m_seq;
//uint8_t m_seq[] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 ,0, 1, 0 ,1 ,0 ,1 ,0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
//uint8_t m_seq[] = {0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1 ,0, 1, 0 ,0 ,0 ,1 ,0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0};

volatile uint16_t m_seq_idx;




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


void enable_timer1() {
  TCNT1 = 0;
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

void disable_timer1() {
  TIMSK1 &= ~(1 << TOIE1); 
}

void init_timer1() {
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  timer1_counter =  65536 - (16 * 1000000 / 256 / TIMER_1_FREQ);

  Serial.print("T1 f: "); Serial.println(TIMER_1_FREQ);

  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  
  disable_timer1();
  enable_timer1();
}



void enable_timer3() {
  TCNT3 = 0;
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
}

void disable_timer3() {
  TIMSK3 &= ~(1 << TOIE3); 
}

void init_timer3() {
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;

  timer3_counter =  65536 - (16 * 1000000 / 256 / TIMER_3_FREQ);

  Serial.print("T3 f: "); Serial.println(TIMER_3_FREQ);

  TCNT3 = timer3_counter;   // preload timer
  TCCR3B |= (1 << CS12);    // 256 prescaler 
  
  disable_timer3();
  enable_timer3();
}

void setup() {
  Serial.begin(250000);

  pinMode(modulate_enable, INPUT);

  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  m_seq = new uint8_t[L];
  m_seq_idx = 0;
  m_seq_create(poly, n, 1, m_seq);

  for (int i = 0; i < L; i++) {
    m_seq[i] = i % 2;
  }


  

  Serial.print("L: "); Serial.println(L);

  for (int i = 0; i < L; i++) {
    Serial.print(m_seq[i]); Serial.print(" ");
  }
  Serial.println();

  modulate_idx = 0;

  init_timer1();
  init_timer3();

  disable_timer1();
  
  interrupts();             // enable all interrupts
  
  attachInterrupt(digitalPinToInterrupt(modulate_enable), isr_change, CHANGE );

}

int zero_counter = 0;

ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  //Serial.println(micros());

  //digitalWrite(led, 1 ^ digitalRead(led));
  
  if (modulate_idx < 4) {
    
    digitalWrite(led, m_seq[m_seq_idx]);
    
    //Serial.println(m_seq_idx);
    //Serial.println(modulate_idx);
    
    //Serial.print(m_seq[m_seq_idx]); Serial.print(" " );
    //Serial.println(m_seq[m_seq_idx]);
    
    m_seq_idx++;
    modulate_idx++;

    

    if (m_seq_idx >= L) {
      m_seq_idx = 0;
    }
    
  } else {
    digitalWrite(led, HIGH);
  }

  Serial.println(digitalRead(led));
  //Serial.println(1);
 
}

ISR(TIMER3_OVF_vect) {      // interrupt service routine 
  TCNT3 = timer3_counter;   // preload timer

  //Serial.print("T3: "); Serial.println(micros());

  digitalWrite(led, 1 ^ digitalRead(led));

  Serial.println(digitalRead(led));
  //Serial.println(3);
 
}


void isr_change(void) {
 
  int state = digitalRead(modulate_enable);
  
  if (state == 1) {
    Serial.println("**");
    modulate_idx = 0;
    disable_timer1();
    digitalWrite(led, HIGH);
    enable_timer3();
    
    zero_counter = 0;
    
  } else {
    Serial.println("*");
    //delayMicroseconds(1100); // less than 1 ms seems to work because that is the symbol length
    disable_timer3();

    enable_timer1();
  }
}


void loop() {  

}


