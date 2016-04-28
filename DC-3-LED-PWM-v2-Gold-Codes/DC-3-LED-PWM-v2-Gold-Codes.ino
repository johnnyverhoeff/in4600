
#define SENSOR_PIN A0
#define NUM_OF_TX 3

#define AMPLITUDE 200

uint8_t **tx_codes;

int16_t *measured_values;
uint8_t start_ptr;
uint8_t end_ptr;

uint8_t leds[] = {3, 5, 6};

uint8_t *tx_decode_status = new uint8_t[NUM_OF_TX];
uint8_t *tx_status        = new uint8_t[NUM_OF_TX];


//uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1

uint8_t poly[] = {1, 0, 0, 0, 0, 1, 1}; // x^6 + x + 1
uint8_t poly2[] = {1, 1, 0, 0, 1, 1, 1}; // x^6 + x^5 + x^2 + x + 1

//uint8_t poly[] = {1, 0, 0, 0, 1, 0, 0, 1}; // x^7 + x^3 + 1
//uint8_t poly2[] = {1, 0, 0, 0, 1, 1, 1, 1}; // x^7 + x^3 + x^2 + x + 1

//uint8_t poly[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 1}; // x^9 + x^4 + 1
//uint8_t poly2[] = {1, 0, 0, 1, 0, 1, 1, 0, 0, 1}; // x^9 + x^6 + x^4 + x^3 + 1


//uint8_t poly[] = {1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1}; // x^10 + x^2 + 1
//uint8_t poly2[] = {1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1}; // x^10 + x^8 + x^3 + x^2 + 1

uint8_t n = sizeof(poly) / sizeof(uint8_t) - 1;
uint16_t L = (1 << n) - 1;
uint16_t N = (1 << n) + 1;

uint16_t num_of_balanced_gold_seq;
uint16_t *balanced_gold_seq_start_states = new uint16_t[N];



/*
This function takes two polynomials arrays and the shift register length n, 
writes the number of balanced gold seqs it produces 
and writes to an array the start states of the second register to obtain balanced gold codes.\

call like so :

uint8_t n = 5;
uint16_t L = (1 << n) - 1;
uint8_t p1[] = {...};
uint8_t p2[] = {...};

uint8_t Nb;

uint8_t bgs = new uint8_t[L];

calc_balanced_gold_codes_idx(p1, p2, n, Nb, bgs);
*/
void calc_balanced_gold_codes_idx(uint8_t *pref_poly1, uint8_t *pref_poly2, uint8_t n, uint16_t &num_of_balanced_gold_seq, uint16_t *balanced_gold_seq_start_states) {
  uint16_t L = (1 << n) - 1;
  uint16_t N = (1 << n) + 1;

  uint16_t start_state1 = 1;

  num_of_balanced_gold_seq = 0;

  for (uint16_t start_state2 = 1; start_state2 <= L; start_state2++) {

    uint16_t lfsr1 = start_state1;
    uint16_t lfsr2 = start_state2;

    uint8_t out1;
    uint8_t out2;

    uint8_t gold_out;
    uint16_t sum_of_gold_seq = 0;

    do {
      out1 = out2 = 0;

      for (uint8_t i = n; i > 0; i--) {
        if (pref_poly1[i] == 1) {
          out1 ^= (lfsr1 >> (n - i));
        }

        if (pref_poly2[i] == 1) {
          out2 ^= (lfsr2 >> (n - i));
        }
      }

      out1 &= 1;
      out2 &= 1;

      gold_out = out1 ^ out2;
      sum_of_gold_seq += gold_out;

      lfsr1 = (lfsr1 >> 1) | (out1 << (n - 1));
      lfsr2 = (lfsr2 >> 1) | (out2 << (n - 1));

    } while (lfsr1 != start_state1);


    if (sum_of_gold_seq == (1 << (n - 1))) {
      // Half + 1 are ones, so balanced
      // save the start_state2;
      balanced_gold_seq_start_states[num_of_balanced_gold_seq++] = start_state2;
    }


  }

}

/*
This function takes two polynomials arrays and the shift register length n, 
and the start state for the second LFSR
and writes to an other array the gold code.

call like so:

uint8_t n = 5;
uint16_t L = (1 << n) - 1;
uint8_t p1[] = {...};
uint8_t p2[] = {...};

uint8_t *gs = new uint8_t[L];


gold_seq_create(p1, p2, n, 1, gs);

*/
void gold_seq_create(uint8_t *pref_poly1, uint8_t *pref_poly2, uint8_t n, uint16_t start_state2, uint8_t *gold_seq) {
  //uint16_t L = (1 << n) - 1;
  //gold_seq = new uint8_t[L];
  
  uint16_t start_state1 = 1;

  uint16_t lfsr1 = start_state1;
  uint16_t lfsr2 = start_state2;

  uint8_t out1;
  uint8_t out2;

  uint16_t step_pos = 0;
  
  do {
    out1 = out2 = 0;

    for (uint8_t i = n; i > 0; i--) {
      if (pref_poly1[i] == 1) {
        out1 ^= (lfsr1 >> (n - i));
      }

      if (pref_poly2[i] == 1) {
        out2 ^= (lfsr2 >> (n - i));
      }
    }

    out1 &= 1;
    out2 &= 1;

    gold_seq[step_pos++] = out1 ^ out2;

    lfsr1 = (lfsr1 >> 1) | (out1 << (n - 1));
    lfsr2 = (lfsr2 >> 1) | (out2 << (n - 1));

  } while (lfsr1 != start_state1);
}


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


void print_seq(uint8_t *seq, uint16_t L) {
  for (uint16_t i = 0; i < L; i++) {
    Serial.print(seq[i]); Serial.print(" ");

  }
  Serial.println();
}



float correlate_w_gold_seq(uint8_t gold_seq_idx) {
  float corr_sum = 0;
  float signal_sum = 0;

  uint8_t *gold_seq = new uint8_t[L];
  gold_seq_create(poly, poly2, n, balanced_gold_seq_start_states[gold_seq_idx], gold_seq);


  for (uint16_t i = 0; i < L; i++) {
    float r = 1 - 2 * ((int8_t)gold_seq[i]);

    float m = measured_values[i] / AMPLITUDE;

    signal_sum += m;

    corr_sum += (m * r);
  }

  float calc_num_of_tx = (signal_sum / (1 << (n - 1))) / AMPLITUDE;

  float correlation = (-2 * corr_sum) - calc_num_of_tx * (1 + 2 * AMPLITUDE);

  delete gold_seq;

  return correlation / L;
}







void setup() {

  Serial.begin(115200); 

  randomSeed(analogRead(A1));

  
  calc_balanced_gold_codes_idx(poly, poly2, n, num_of_balanced_gold_seq, balanced_gold_seq_start_states);




  tx_codes = new uint8_t*[NUM_OF_TX];
  for (uint8_t i = 0; i < NUM_OF_TX; i++) {
    tx_codes[i] = new uint8_t[L];

    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);

    gold_seq_create(poly, poly2, n, balanced_gold_seq_start_states[i], tx_codes[i]);
  }

  measured_values = new int16_t[L];



  start_ptr = 0;
  end_ptr = 0;
   

}




void loop() {
  

  for (uint8_t i = 0; i < NUM_OF_TX; i++) {
    long p = random(0, 100);

    if (p < 50)   
      tx_status[i] = 1;
    else 
      tx_status[i] = 0;
  }


  for (uint8_t chip = 0; chip < L; chip++) {
    for (uint8_t i = 0; i < ( NUM_OF_TX - 0); i++) {
      if (tx_status[i] == 1) {
        
        digitalWrite(leds[i], tx_codes[i][chip]);

        /*if (tx_codes[i][chip] == 1) {
          PORTD |= (1 << leds[i]);
        } else {
          PORTD &= ~(1 << leds[i]);
        }*/


      } else {
        digitalWrite(leds[i], HIGH);
      }
    }

    measured_values[end_ptr++] = analogRead(SENSOR_PIN);


    //delayMicroseconds(10);
  }

  for (uint8_t i = 0; i < ( NUM_OF_TX - 0); i++) {
    digitalWrite(leds[i], HIGH);
  }


  for (uint8_t i = 0; i < NUM_OF_TX; i++) {
    float corr = correlate_w_gold_seq(i);
    uint8_t status  = corr > ((float)(1 / 2)) ? 1 : 0;

    if (i < NUM_OF_TX) {
      tx_decode_status[i] = status;
    }

    Serial.print( "(" ); 
    Serial.print(tx_status[i]); 
    Serial.print(", ");
    Serial.print(tx_decode_status[i]);
    Serial.print(", ");
    Serial.print(corr * L);
    Serial.print("), ");

    if (tx_status[i] != tx_decode_status[i])
      Serial.println("PANIC");
  }

  Serial.println();


  end_ptr = 0;
  delay(1000);


}
