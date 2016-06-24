#define modulate_enable 3

#define led 8 

int done_modulating = 0;

#define ADC_BUFFER_SIZE 200
uint16_t *adc_buffer;
uint16_t adc_idx;


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


  Serial.begin(250000);

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  pinMode(modulate_enable, INPUT);

  adc_buffer = new uint16_t[ADC_BUFFER_SIZE];
  adc_idx = 0;

  m_seq = new uint8_t[L];
  m_seq_idx = 0;
  m_seq_create(poly, n, 1, m_seq);

}

void loop() {
  if (!done_modulating && digitalRead(modulate_enable) == 0) {
    
    for (int i = 0; i < 4; i++) {

      delayMicroseconds(1000);
      digitalWrite(led, m_seq[m_seq_idx++]);
      
      
      //Serial.println(analogRead(A0));
      //adc_buffer[adc_idx++] = analogRead(A0);

      if (m_seq_idx >= L) {
        m_seq_idx = 0;
        adc_idx = 0;
        break;
      }
        
      
    }
    done_modulating = 1;
    digitalWrite(led, HIGH);

  }


  float corr_sum = 0;

  /*for (int i = 0; i < L; i++) {
    float r_chip = 1 - 2 * ((int8_t)m_seq[i]);

    corr_sum += adc_buffer[(i+0) % L] * r_chip;

    //Serial.print(adc_buffer[i]);
    //Serial.print(" * ");
    //Serial.println(r_chip);
  }

  Serial.println((-2 * corr_sum - 1) / L / 850);*/

  while (digitalRead(modulate_enable) == 0);
  done_modulating = 0;
}


