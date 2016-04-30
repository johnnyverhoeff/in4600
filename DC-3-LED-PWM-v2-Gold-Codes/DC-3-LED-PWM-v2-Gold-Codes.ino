#include <limits.h>


#define SENSOR_PIN A0
#define NUM_OF_TX 3

#define AMPLITUDE 200

uint8_t **tx_codes;

int16_t *measured_values;
uint16_t iptr;

uint8_t leds[] = {3, 5, 6};

uint8_t *tx_decode_status = new uint8_t[NUM_OF_TX];
uint8_t *tx_status        = new uint8_t[NUM_OF_TX];
uint16_t *tx_timestamp     = new uint16_t[NUM_OF_TX];
uint16_t *tx_detected = new uint16_t[NUM_OF_TX];

uint16_t all_detected_timestamp = 0;

uint16_t time;

float p = 0.0058;
//float p = 0.1;


uint16_t fp = 0;
uint16_t fn = 0;
uint16_t tp = 0;
uint16_t tn = 0;

uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1
uint8_t poly2[] = {1, 1, 1, 1, 0, 1}; // x^5 + x^4 + x^3 +x^2 + 1

//uint8_t poly[] = {1, 0, 0, 0, 0, 1, 1}; // x^6 + x + 1
//uint8_t poly2[] = {1, 1, 0, 0, 1, 1, 1}; // x^6 + x^5 + x^2 + x + 1

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



int16_t find_min(int16_t *array, uint16_t length) {
  int16_t min = SHRT_MAX;

  for (uint16_t i = 0; i < length; i++) {
    if (array[i] < min)
      min = array[i];
  }

  return min;
}

int16_t find_max(int16_t *array, uint16_t length) {
  int16_t max = SHRT_MIN;

  for (uint16_t i = 0; i < length; i++) {
    if (array[i] > max)
      max = array[i];
  }

  return max;
}








float correlate_w_gold_seq(uint8_t gold_seq_idx) {
  float corr_sum = 0;
  float signal_sum = 0;

  uint8_t *gold_seq = new uint8_t[L];
  gold_seq_create(poly, poly2, n, balanced_gold_seq_start_states[gold_seq_idx], gold_seq);

  int16_t min_signal = find_min(measured_values, L);
  int16_t max_signal = find_max(measured_values, L);

  uint16_t begin = iptr;
  uint16_t end   = iptr + L;


  uint16_t code_i = 0;

  for (uint16_t i = begin; i < end; i++) {
    uint16_t idx = i % L;
    float r_chip = 1 - 2 * ((int8_t)gold_seq[code_i++]);

    float scaled_measurement = (measured_values[idx] - min_signal) / AMPLITUDE;

    signal_sum += scaled_measurement;

    corr_sum += (scaled_measurement * r_chip);
  }

  float calc_num_of_tx = signal_sum / (1 << (n - 1));

  float correlation = (-2 * corr_sum) - calc_num_of_tx;

  delete gold_seq;

  return correlation / L;
}

 





void setup() {

  Serial.begin(115200); 

  randomSeed(analogRead(A1));

  
  calc_balanced_gold_codes_idx(poly, poly2, n, num_of_balanced_gold_seq, balanced_gold_seq_start_states);


  time = 0;


  tx_codes = new uint8_t*[NUM_OF_TX];
  for (uint8_t i = 0; i < NUM_OF_TX; i++) {

    tx_codes[i] = new uint8_t[L];
    tx_timestamp[i] = 0;
    tx_detected[i] = 0;


    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);

    gold_seq_create(poly, poly2, n, balanced_gold_seq_start_states[i], tx_codes[i]);
  }

  measured_values = new int16_t[L];



  iptr = 0;

  for (uint8_t i = 0; i < NUM_OF_TX; i++) {
    digitalWrite(leds[i], HIGH);
    tx_status[i] = 0;
  }
   


  /*for (int i = 0; i < L; i++) {
    measured_values[i] = (int16_t) AMPLITUDE * tx_codes[0][i];
  }*/

  

  Serial.println("TP TX, TIME: (status, decode_status, corr*L, tx_timestamp)");

}


uint8_t enough_measurements = 0;

void loop() {
  

  for (uint8_t i = 0; i < (NUM_OF_TX - 0); i++) {
    if (tx_timestamp[i] - time >= L) {
      long r = random(0, 100);

      if (r < (p * 100.0)) {   
        tx_status[i] = 1;
        tx_timestamp[i] = time;
      } else { 
        tx_status[i] = 0;
      }
    }
  }


  for (uint8_t chip = 0; chip < L; chip++) {
    for (uint8_t i = 0; i < (NUM_OF_TX - 0); i++) {
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

    measured_values[iptr++] = analogRead(SENSOR_PIN);
    //iptr++;

    if (iptr >= L) {
      enough_measurements = 1;
      iptr = 0;
    }

    if (enough_measurements) {

      uint8_t note_worthy_msg;

      for (uint8_t i = 0; i < NUM_OF_TX; i++) {

        float corr = correlate_w_gold_seq(i);
        uint8_t status  = (corr >= 0.5) ? 1 : 0;

        if (i < NUM_OF_TX) {
          tx_decode_status[i] = status;
        }

        

        note_worthy_msg = 0;

        if (time == (L + tx_timestamp[i] - 1)) { // correct time to decode result
          if (tx_status[i] == 1) {
            if (tx_decode_status[i] == 0) {
              Serial.print("FN");
              fn++;
              note_worthy_msg = 1;
            } else {
              Serial.print("TP");
              tp++;
              note_worthy_msg = 1;
              tx_detected[i] = 1;
            }
          } else {
            if (tx_decode_status[i] == 0) {
              //Serial.print("TN");
              tn++;
            } else {
              Serial.print("FP");
              fp++;
              note_worthy_msg = 1;
            }
          } 
        } else {
          if (tx_status[i] == 1) { // busy modulating
            if (tx_decode_status[i] == 0) {
              // ok still busy modulating
              tn++;
              //Serial.print("TN");
            } else {
              // not ok because still busy modulating so should not be high
              fp++;
              Serial.print("FP");
              note_worthy_msg = 1;
            }
          } else {
            if (tx_decode_status[i] == 0) {
              // ok should be off
              tn++;
              //Serial.print("TN");
            } else {
              // not ok because not modulating so should not be high
              fp++;
              Serial.print("FP");
              note_worthy_msg = 1;
            }
          }
        }

        if (note_worthy_msg == 1) {
          Serial.print(" ");
          Serial.print(i);
          Serial.print(", ");
          Serial.print(time);
          Serial.print(": ");
          Serial.print( "(" ); 
          Serial.print(tx_status[i]); 
          Serial.print(", ");
          Serial.print(tx_decode_status[i]);
          Serial.print(", ");
          Serial.print(corr * L);
          Serial.print(", ");
          Serial.print(tx_timestamp[i]);
          Serial.print(")");

          Serial.println();
        }


      }


      //delay(100);
    }
    


    //delayMicroseconds(10);
    time++;
  }


  uint8_t all_detected = 1;

  for (uint8_t i = 0; i < NUM_OF_TX; i++) {
    digitalWrite(leds[i], HIGH);

    all_detected  &= tx_detected[i];
  }

  if (all_detected == 1) {
    all_detected_timestamp = time - all_detected_timestamp;
    Serial.println("***********************************");
    Serial.print("All tx detected after: "); Serial.println(all_detected_timestamp);
    Serial.println("***********************************");
    Serial.print("F-measure: "); Serial.println(2*tp / (2*tp + fp + fn));
    Serial.println("***********************************");
    for (uint8_t i = 0; i < NUM_OF_TX; i++) {
      tx_detected[i] = 0;
    }
  }



  

  


}
