#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <pigpio.h>

/*#define SENSOR_PIN_0 A0
#define SENSOR_PIN_1 A1
#define RANDOM_SEED A2*/

#define NUM_OF_TX 3
#define NUM_OF_ACTIVE_TX 2

#define LARGE_INT_NUMBER 10000

#define AMPLITUDE 295

int16_t *measured_values;
uint16_t iptr;

uint8_t leds[] = {13, 19, 26, 8, 9, 10}; // other three TBD


uint8_t *tx_decode_status   = new uint8_t[NUM_OF_TX];
uint8_t *tx_status          = new uint8_t[NUM_OF_TX];
uint32_t *tx_timestamp      = new uint32_t[NUM_OF_TX];
uint32_t *tx_detected       = new uint32_t[NUM_OF_TX];
uint32_t *tx_k              = new uint32_t[NUM_OF_TX];

uint8_t **tx_code;

uint32_t all_detected_timestamp = 0;

uint32_t time_time;

float p = 0.01; //0.0289;// 0.05; // Prob. that tx will tx_status.

float epsilon = 0.1; // 1 - Prob. that a tx has tx'ed atleast 1 time.

float k = log(epsilon) / log(1 - p);
uint8_t k_msg = 0;


uint32_t fp = 0;
uint32_t fn = 0;
uint32_t tp = 0;
uint32_t tn = 0;

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
    printf("%d ", seq[i]);
  }
  printf("\n");
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








float correlate_w_gold_seq(uint16_t gold_seq_idx) {
  float corr_sum = 0;
  float signal_sum = 0;

  int16_t min_signal = find_min(measured_values, L);
  int16_t max_signal = find_max(measured_values, L);

  uint16_t beginning = iptr;
  uint16_t ending   = iptr + L;


  uint16_t code_i = 0;

  for (uint16_t i = beginning; i < ending; i++) {
    uint16_t idx = i % L;
    float r_chip = 1 - 2 * ((int8_t)tx_code[gold_seq_idx][code_i++]);

    float scaled_measurement = (measured_values[idx] - min_signal) / AMPLITUDE;

    signal_sum += scaled_measurement;

    corr_sum += (scaled_measurement * r_chip);
  }

  float calc_num_of_tx = signal_sum / (1 << (n - 1));

  float correlation = (-2 * corr_sum) - calc_num_of_tx;

  return correlation / L;
  //delayMicroseconds(0);
  //return 0;
}

 
uint8_t enough_measurements = 0;


int seed;

int main(int argc, char *argv[]) {
  
	if (gpioInitialise() < 0) {
		fprintf(stderr, "pigpio init. failed\0");
		return 1;
	}

	seed = time(NULL);
	srand(seed);



	calc_balanced_gold_codes_idx(poly, poly2, n, num_of_balanced_gold_seq, balanced_gold_seq_start_states);

	tx_code = new uint8_t*[NUM_OF_ACTIVE_TX];

	time_time = 0;

	for (uint8_t i = 0; i < NUM_OF_ACTIVE_TX; i++) {

		tx_code[i] = new uint8_t[L];
		gold_seq_create(poly, poly2, n, balanced_gold_seq_start_states[i], tx_code[i]);

		tx_timestamp[i] = 0;
		tx_detected[i] = 0;
		tx_k[i] = 0;


		gpioSetMode(leds[i], OUTPUT);
		gpioWrite(leds[i], LOW);
	}

	measured_values = new int16_t[L];



	iptr = 0;

	for (uint8_t i = 0; i < NUM_OF_ACTIVE_TX; i++) {
		digitalWrite(leds[i], HIGH);
		tx_status[i] = 0;
	}

	printf("k: %f\n", k);

	printf("TP TX, TIME: (status, decode_status, corr*L, tx_timestamp)\n");

	printf("L: %d\n", L)

	while (1) {

	  //unsigned long start_time_corr = micros();

	  for (uint8_t i = 0; i < NUM_OF_ACTIVE_TX; i++) {
	    //if (tx_timestamp[i] - time_time >= L) {
	    if (time_time - tx_timestamp[i] >= L) {
	      
	      tx_k[i]++;

	      if (tx_k[i] >= (uint16_t)k) {
	        if (!k_msg) {
	          printf("***** K reached *****"\n);
	          printf("***** TX's that tx'ed: ");
	          for (uint8_t j = 0; j < NUM_OF_ACTIVE_TX; j++) {
	            if (tx_detected[j])
	              printf("%d", j);
	            printf(" ");
	          }
	          printf("*****\n");
	          
	          k_msg = 1;
	        }
	      }

	      long r = rand() % 10000;
	      float scaled_r = ((float) r) / 10000.0;

	      if (scaled_r < p) {   
	        tx_status[i] = 1;
	        tx_timestamp[i] = time_time;
	      } else { 
	        tx_status[i] = 0;
	      }
	    }
	  }


	  for (uint16_t chip = 0; chip < L; chip++) {
	    for (uint8_t i = 0; i < NUM_OF_ACTIVE_TX; i++) {
	      if (tx_status[i] == 1) {

	        gpioWrite(leds[i], tx_code[i][chip]);
	        //delay(1);

	        /*Serial.print("chip: "); Serial.print(chip);
	        Serial.print(" ,i: "); Serial.println(i);*/

	      } else {
	        gpioWrite(leds[i], HIGH); 
	      }
	    }

	    measured_values[iptr++] = 0; /*analogRead(SENSOR_PIN_0) + analogRead(SENSOR_PIN_1);*/

	    if (iptr >= L) {
	      enough_measurements = 1;
	      iptr = 0;
	    }

	    if (enough_measurements) {

	      uint8_t note_worthy_msg;

	      
	      
	      
	      for (uint8_t i = 0; i < NUM_OF_ACTIVE_TX; i++) {

	        //unsigned long start_corr_time = micros();

	        float corr = correlate_w_gold_seq(i);

	        /*Serial.println(micros() - start_corr_time);
	        printf("\n");;
	        delay(2000);*/
	        
	        uint8_t status  = (corr >= 0.5) ? 1 : 0;

	        if (i < NUM_OF_ACTIVE_TX) {
	          tx_decode_status[i] = status;
	        }

	        note_worthy_msg = 0;

	        if (time_time == (L + tx_timestamp[i] - 1)) { // correct time to decode result
	          if (tx_status[i] == 1) {
	            if (tx_decode_status[i] == 0) {
	              printf("*FN*");
	              fn++;
	              note_worthy_msg = 1;
	            } else {
	              Serial.print(" TP ");
	              tp++;
	              note_worthy_msg = 1;
	              tx_detected[i] = 1;
	            }
	          } else {
	            if (tx_decode_status[i] == 0) {
	              //printf("TN");
	              tn++;
	            } else {
	              printf("*FP*");
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
	              printf("*FP*");
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
	              printf("*FP*");
	              note_worthy_msg = 1;
	            }
	          }
	        }

	        if (note_worthy_msg == 1) {
	          printf(" %d, %d:  (%d, %d, %f, %d)\n", i, time_time, tx_status[i], tx_decode_status[i], corr * L, tx_timestamp[i]);
	          /*Serial.print(" ");
	          Serial.print(i);
	          Serial.print(", ");
	          Serial.print(time_time);
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

	          printf("\n");*/
	        }


	      }


	      
	      
	    }
	   
	    time_time++;
	  }


	  uint8_t all_detected = 1;

	  for (uint8_t i = 0; i < NUM_OF_ACTIVE_TX; i++) {
	    gpioWrite(leds[i], HIGH);

	    all_detected  &= tx_detected[i];
	  }

	  if (all_detected == 1) {
	    


	    printf("      ***********************************\n");
	    printf("      All tx detected after: %d\n", time_time - all_detected_timestamp)
	    printf("      ***********************************\n");
	    

	    float f_measure = 2 * ((float)tp) / (2 * ((float)tp) + (float)fp + (float)fn);

	    printf("      F-measure: %f\n", f_measure);
	    printf("      ***********************************\n");

	    printf("      ");
	    all_detected_timestamp = time_time;

	    for (uint8_t i = 0; i < NUM_OF_ACTIVE_TX; i++) {
	      tx_detected[i] = 0;
	      printf("%d: %d, ", i, tx_k[i]);
	      tx_k[i] = 0;
	      k_msg = 0;
	    }

	    printf("\n");
	    printf("      ***********************************\n");
	  }


	  //Serial.print("TIME CORRRRR: "); Serial.println((micros() - start_time_corr));
	  //printf("\n");;
	  //delay(1000);
	}

}
