#define led 13

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


void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  Serial.begin(250000);


  m_seq = new uint8_t[L];
  m_seq_idx = 0;
  m_seq_create(poly, n, 1, m_seq);
 
}

void loop() {

  for (int i = 0; i < L; i++) {
    digitalWrite(led, m_seq[i]);
    //Serial.println(digitalRead(led));
    delayMicroseconds(1000);
  }

  
}

