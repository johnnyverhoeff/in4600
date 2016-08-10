#define modulate_enable 3
#define cap_bypass_pin 8
#define led 13

#define TIMER_FREQ 1000 //Hz
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

volatile unsigned long modulate_start_time = 0;



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

  pinMode(cap_bypass_pin, OUTPUT);
  digitalWrite(cap_bypass_pin, LOW);
  //digitalWrite(cap_bypass_pin, HIGH);

  m_seq = new uint8_t[L];
  m_seq_idx = 0;
  m_seq_create(poly, n, 1, m_seq);

  /*for (int i = 0; i < L; i++) {
    Serial.print(m_seq[i]); Serial.print(" ");
  }
  Serial.println();
*/
  modulate_idx = 0;


  // initialize timer1 -
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  timer1_counter =  65536 - (16 * 1000000 / 256 / TIMER_FREQ);

  //Serial.println(TIMER_FREQ);
  //Serial.println(timer1_counter);

 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  disable_timer();
  

        
  modulate_enable_flag = 0;
  modulate_start_time = 0;
  
  attachInterrupt(digitalPinToInterrupt(modulate_enable), isr_change, CHANGE );
  

  interrupts();       

}

ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  //Serial.println(micros());

  
  //Serial.println("a");
  
  if (modulate_idx < 6) {
    
    digitalWrite(led, m_seq[m_seq_idx]);
    
    //Serial.print(m_seq[m_seq_idx]); Serial.print(" " );
    //Serial.println(m_seq[m_seq_idx]);
    
    m_seq_idx++;
    modulate_idx++;

    if (m_seq_idx >= L) {
      m_seq_idx = 0;
      done_modulating_one_seq++;

      if (done_modulating_one_seq >= 20) {
        //detachInterrupt(digitalPinToInterrupt(modulate_enable));
        modulate_enable_flag = 0;
        
        digitalWrite(cap_bypass_pin, LOW);
        done_modulating_one_seq = 0;
        Serial.println("DONE");
      }
      
    }
  } else {
    digitalWrite(led, HIGH);
    disable_timer();
  }
 
}


void isr_change(void) {
  

  if (modulate_enable_flag == 1) {

    //delayMicroseconds(750); // debouncing effect...
  
    int state = digitalRead(modulate_enable);
  
    if (state == 1) {
      //Serial.println("s1");
  
      
      modulate_idx = 0;
      disable_timer();
  
      
      digitalWrite(led, HIGH);
    
      
    } else {
      //delayMicroseconds(1100); // less than 1 ms seems to work because that is the symbol length
      //Serial.println("s0");
      enable_timer();
    }

    
  } else {
    
    /*int state = digitalRead(modulate_enable);
    //delayMicroseconds(1000); // debouncing effect...
    
    if (state == 1) {
      digitalWrite(led, LOW);

    } else {
      digitalWrite(led, HIGH);
    }*/

    digitalWrite(led, HIGH);
  }
}


void loop() {  

  if ((digitalRead(cap_bypass_pin) == 1) && (millis() - modulate_start_time > 60000)) {
    Serial.println("TIME-OUT DETECTED");
    modulate_enable_flag = 0;
    modulate_start_time = 0;
    digitalWrite(cap_bypass_pin, LOW);
    
  }
  
  if (modulate_enable_flag == 0) {
    if (Serial.available() > 0) {
      Serial.print("Received: "); Serial.println(Serial.read());
      
      while (Serial.available() > 0) {
        Serial.println(Serial.read());
      }
  
      noInterrupts();
  
      digitalWrite(cap_bypass_pin, HIGH);
      delay(40);
  
      //attachInterrupt(digitalPinToInterrupt(modulate_enable), isr_change, CHANGE );
      modulate_enable_flag = 1;

      modulate_start_time = millis();
  
      interrupts();
    }
  }

  
}


