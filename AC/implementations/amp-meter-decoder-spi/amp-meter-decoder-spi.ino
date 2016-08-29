
#include <SPI.h>

#define ADC_CS 38 // arduino PIN number
#define PORTD_ADC_CS 7 // ATMEGA PIN number

#define modulate_enable 3

#define NUM_OF_TIMES_CHECK_TRIGGER_SIGNAL 100

//#define HARDCODE_MODULATE_TIME
// define this for hardcoded 7 ms, else for auto-detection

#define USE_TEST_OUPUT_PINS
// define this for letting the two isr's, timer & trigger, toggle pins for testing purposes.

uint32_t  time_to_modulate_per_period,
          modulate_times_per_period;

const uint32_t timer_freq = 10000; //Hz




uint16_t timer1_counter;

#define ADC_BUFFER_SIZE 1600
uint16_t *adc_buffer;
uint16_t adc_idx;
uint8_t adc_buffer_full;

uint8_t sample_idx = 0;

uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1
//uint8_t poly2[] = {1, 1, 1, 1, 0, 1}; // x^5 + x^4 + x^3 +x^2 + 1

//uint8_t poly[] = {1, 0, 0, 0, 1, 0, 0, 1}; // x^7 + x^3 + 1

uint8_t n = sizeof(poly) / sizeof(uint8_t) - 1;
uint16_t L = (1 << n) - 1;
uint16_t N = (1 << n) + 1;

uint8_t *m_seq;
//uint8_t *m_seq2;
uint16_t m_seq_idx;
//uint16_t m_seq2_idx;




volatile uint8_t timer_enable = 0;

volatile uint32_t loop_counter = 0;

uint32_t start_low_time = 0;

uint16_t avg_adc_value;


/*volatile uint32_t last_time_clear_flag = 0;

void clear_timer_flag(void) {
  last_time_clear_flag = micros();
}*/



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

uint32_t get_avg_trigger_low_time(void) {
  uint32_t  avg_signal_low_time = 0, 
          min_signal_low_time = 10000,
          max_signal_low_time = 0,
          
          avg_signal_high_time = 0,
          min_signal_high_time = 10000,
          max_signal_high_time = 0;
          
  for (int i = 0; i < NUM_OF_TIMES_CHECK_TRIGGER_SIGNAL; i++) {
    
    while (digitalRead(modulate_enable) == 0); //wait while signal is low
    while (digitalRead(modulate_enable) == 1); //wait while signal is high
    
    //signal is low.
  
    uint32_t begin_time_signal_low = micros();
  
    while (digitalRead(modulate_enable) == 0); //wait while signal is low
  
    uint32_t end_time_signal_low = micros();
    uint32_t begin_time_signal_high = end_time_signal_low;
  
    while (digitalRead(modulate_enable) == 1); //wait while signal is high
  
    uint32_t end_time_signal_high = micros();

    uint32_t signal_low_time = end_time_signal_low - begin_time_signal_low;
    uint32_t signal_high_time = end_time_signal_high - begin_time_signal_high;

    min_signal_low_time = min(min_signal_low_time, signal_low_time);
    max_signal_low_time = max(max_signal_low_time, signal_low_time);

    min_signal_high_time = min(min_signal_high_time, signal_high_time);
    max_signal_high_time = max(max_signal_high_time, signal_high_time);

    avg_signal_low_time = (avg_signal_low_time + (min_signal_low_time + max_signal_low_time) / 2) / 2;
    avg_signal_high_time = (avg_signal_high_time + (min_signal_high_time + max_signal_high_time) / 2) / 2;

  }

  uint32_t avg_period_time = avg_signal_low_time + avg_signal_high_time;

  if (avg_period_time > 10100) {
    //Serial.print("Sanity check failed...");
    //Serial.println(avg_period_time);
    return get_avg_trigger_low_time();
  }

  //Serial.println(avg_signal_low_time);

  return avg_signal_low_time;
}



void enable_timer() {
  TCNT1 = 0;
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

void disable_timer() {
  TIMSK1 &= ~(1 << TOIE1); 
}

void init_timer() {
  // initialize timer1 -
  TCCR1A = 0;
  TCCR1B = 0;

  timer1_counter = 65536 - (16 * 1000000 / 256 / timer_freq);

  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
}



void setup() { 
  Serial.begin(250000);

  pinMode(modulate_enable, INPUT);
  
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);

  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);

  digitalWrite(30, LOW);
  digitalWrite(31, LOW);

  randomSeed(analogRead(0));


#ifdef HARDCODE_MODULATE_TIME

  time_to_modulate_per_period = 7000; //us

#else

  uint32_t avg_trigger_low_time = get_avg_trigger_low_time();

  time_to_modulate_per_period = min(7000 /* us */, avg_trigger_low_time);
  /*Serial.print("avg_trigger_low_time: ");*/ Serial.println(avg_trigger_low_time);

#endif

  modulate_times_per_period = time_to_modulate_per_period * timer_freq / 1000000 - 1;


  
  /*Serial.print("time_to_modulate_per_period: "); Serial.println(time_to_modulate_per_period);
  Serial.print("modulate_times_per_period: "); Serial.println(modulate_times_per_period);*/



  

  SPI.begin();
  /*SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV64);*/
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  //Serial.println("SPI started...");


  adc_buffer = new uint16_t[ADC_BUFFER_SIZE];
  adc_idx = 0;
  adc_buffer_full = 0;

  for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
    adc_buffer[i] = 0;
  }

  m_seq = new uint8_t[L];
  //m_seq2 = new uint8_t[L];
  
  m_seq_idx = 0;
  //m_seq2_idx = 0;
  
  m_seq_create(poly, n, 1, m_seq);
  //m_seq_create(poly2, n, 1, m_seq2);

  /*for (int i = 0; i < L; i++) {
    Serial.print(m_seq[i]); Serial.print(" ");
  }
  Serial.println();*/


  sample_idx = 0;

  noInterrupts();
  cli();

  init_timer();
  disable_timer();

  attachInterrupt(digitalPinToInterrupt(modulate_enable), isr_change, CHANGE );

  avg_adc_value = get_ground_adc_readings();

  interrupts();             // enable all interrupts
  sei();
}




uint16_t get_ground_adc_readings(void) {
  #define SAMPLES_FOR_AVG 20000
  
  uint32_t sum = 0;
  uint16_t min_val = 4095;
  uint16_t max_val = 0;
  
  for (int i = 0; i < SAMPLES_FOR_AVG; i++) {
    uint16_t val = readADC(0);
    sum += val;

    if (val < min_val) {
      min_val = val;
    }

    if (val > max_val) {
      max_val = val;
    }
    delayMicroseconds(100);
  }

  uint16_t avg = sum / SAMPLES_FOR_AVG;

  /*Serial.print("sum: "); Serial.println(sum);
  Serial.print("avg: "); Serial.println(avg);
  Serial.print("min: "); Serial.println(min_val);
  Serial.print("max: "); Serial.println(max_val);
  Serial.print("avg_min_max: "); Serial.println( (min_val + max_val) / 2);*/

  return avg;
}




// with DIV8 -> ~44 us
// with DIV8 & direct io -> 20 us 
// with DIV32 -> 60 us -> 16 kHz
// with DIV64 -> 110us -> 
uint16_t readADC(uint8_t channel) {

  //digitalWrite(ADC_CS, LOW);
  PORTD = PORTD & ~(1 << PORTD_ADC_CS);
  
  uint8_t spi_tx = 0x06; // Refer to FIGURE 6-1 in MCP3204 datasheet.
  SPI.transfer(spi_tx);

  spi_tx = channel << 6;
  
  uint8_t rx1 = SPI.transfer(spi_tx);
  uint8_t rx2 = SPI.transfer(0xFF);

  //digitalWrite(ADC_CS, HIGH);
  PORTD = PORTD | (1 << PORTD_ADC_CS);

  return (((rx1 & 0x0F) << 8) | rx2);
}



ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

#if defined(USE_TEST_OUPUT_PINS)
  PORTC ^= (1 << 7); // pin 31, for testing purposes only
#endif
  

    
  if (sample_idx < modulate_times_per_period) {

    uint16_t read_value = readADC(0);
    uint16_t scaled_value = 0;
    
    if (read_value >= avg_adc_value) {
      scaled_value = read_value - avg_adc_value;
    } else {
      scaled_value = avg_adc_value - read_value ;
    }

    // adds a offset to the signal, this should matter, so it is here for test purposes
    //#define OFFSET 1326 

    // this adds an artificial other LED to the read ADC signal
    // this will give the correlatet signal more than 2 levels, i.e., the t(n) plays a role
    // The amplitude should be roughly the same as the real LED (~250) 
    //#define OTHER_LED_AMPLITUDE 250
    //#define OTHER_LED_OFFSET 6

#if defined(OFFSET)

    scaled_value += OFFSET;
    
#endif

#if defined(OTHER_LED_AMPLITUDE) && defined(OTHER_LED_OFFSET)

    scaled_value += (OTHER_LED_OFFSET + OTHER_LED_AMPLITUDE * m_seq2[m_seq2_idx++]);

    if (m_seq2_idx >= L) {
      m_seq2_idx = 0;
    }
    
#endif
    
    adc_buffer[adc_idx++] = scaled_value;

    sample_idx++;
    
    if (adc_idx >= ADC_BUFFER_SIZE) {
      adc_idx = 0;
      adc_buffer_full = 1;
    }
  }
}





void isr_change(void) {

#if defined(USE_TEST_OUPUT_PINS)
  PORTC ^= (1 << 6); // pin 30, for testing purposes only
#endif
 
  if (digitalRead(modulate_enable) == 1) {
    sample_idx = 0;
    disable_timer();
  } else {
    start_low_time = micros();
    enable_timer();
  }
}



#define SLOPE_NOISE 50

void loop() {

  // to make sure the timer wont fail...
  loop_counter++;
  if (loop_counter % 10000 == 0) {
    init_timer();
  }

 
  if (adc_buffer_full == 1) {
    
    noInterrupts();
    cli();

    /* changed from 0 to 1 !! , for differentiating */
    for (uint16_t offset = 1; offset < (ADC_BUFFER_SIZE - 2*L); offset++) {

      uint16_t abs_max_slope = 0;
      
      uint16_t abs_slope_per_led = 4095; // smallest absolute slope, but bigger than (around) 0

      int16_t *diff_signal = new int16_t[L - 1];

      
      for (uint16_t i = 0; i < (L - 1); i++) {
        diff_signal[i] = ((int16_t)adc_buffer[i + offset]) - ((int16_t)adc_buffer[i + offset - 1]);

        uint16_t abs_slope = abs( diff_signal[i] );
        
        if (abs_slope > SLOPE_NOISE) {
          abs_slope_per_led = min(abs_slope_per_led, abs_slope);
        }
        
        abs_max_slope = max(abs_max_slope, abs_slope);
      }

      float normalized_l_corr = 0;

      if (abs_max_slope <= SLOPE_NOISE) {
        // not so much change in signal, so probably no LEDs modulating, setting correlation at 0

        normalized_l_corr = 0;
      } else {
      
        int16_t *integrated_diff_signal = new int16_t[L];

        for (uint16_t i = 0; i < (L - 1); i++) {
          if (diff_signal[i] != 0 && abs(diff_signal[i]) > SLOPE_NOISE) {
            
            if (diff_signal[i] < 0) {
              // first slope is negative, so initial value must be a positive number
              integrated_diff_signal[0] = abs_slope_per_led;
            } else {
              // else (0 or positive), intial value will be set to 0
              integrated_diff_signal[0] = 0;
            }

            break;
          }
        }
  
        for (uint16_t i = 1; i < L; i++) {
          
          if (abs(diff_signal[i - 1]) > SLOPE_NOISE) {
            integrated_diff_signal[i] = integrated_diff_signal[i - 1] + diff_signal[i - 1];
          } else {
            integrated_diff_signal[i] = integrated_diff_signal[i - 1];
          }

          if (integrated_diff_signal[i] < SLOPE_NOISE)
            integrated_diff_signal[i] = 0;
            
        }

        uint32_t signal_sum = 0;
        int32_t corr_sum = 0;
  
        for (uint16_t i = 0; i < L; i++) {
  
          signal_sum += integrated_diff_signal[i];
  
          int8_t r_chip = 1 - 2 * ((int8_t)m_seq[i]);
          corr_sum += (int16_t)integrated_diff_signal[i] * r_chip;
        }
          
        float calc_num_of_tx = (float)signal_sum / (abs_slope_per_led * (1 << (n - 1)));

        if ( calc_num_of_tx < (1 << (n + 1))) {
          // acceptable number...
          //Serial.println(calc_num_of_tx);
          
          normalized_l_corr = ((float)-2 * corr_sum / abs_slope_per_led) - calc_num_of_tx;
          
        } else {
          normalized_l_corr = 0;
        }

        delete integrated_diff_signal;
        
      }

      delete diff_signal;
      
      Serial.println(normalized_l_corr);
      
    }

    /*for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
      Serial.println(adc_buffer[i]);
    }*/

    adc_buffer_full = 0;
    sample_idx = 0;
    
    interrupts();
    sei(); 
  }
}
