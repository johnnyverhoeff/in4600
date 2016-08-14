
#include <SPI.h>

#define ADC_CS 38 // arduino PIN number
#define PORTD_ADC_CS 7 // ATMEGA PIN number

#define modulate_enable 3

#define NUM_OF_TIMES_CHECK_TRIGGER_SIGNAL 100

#define HARDCODE_MODULATE_TIME
// define this for hardcoded 7 ms, else for auto-detection

uint32_t  time_to_modulate_per_period,
          modulate_times_per_period;

const uint32_t timer_freq = 15000; //Hz




uint16_t timer1_counter;

#define ADC_BUFFER_SIZE 1600
uint16_t *adc_buffer;
uint16_t adc_idx;
uint8_t adc_buffer_full;

uint8_t sample_idx = 0;

uint8_t poly[] = {1, 0, 0, 1, 0, 1}; // x^5 + x^2 + 1

uint8_t n = sizeof(poly) / sizeof(uint8_t) - 1;
uint16_t L = (1 << n) - 1;
uint16_t N = (1 << n) + 1;

uint8_t *m_seq;
uint16_t m_seq_idx;




volatile uint8_t timer_enable = 0;




uint16_t avg_adc_value;



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

uint32_t get_min_trigger_low_time(void) {
  uint32_t  avg_signal_low_time, 
          min_signal_low_time = 10000,
          max_signal_low_time = 0,
          
          avg_signal_high_time,
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


    uint32_t max_period_time = min_signal_low_time + max_signal_high_time;

    if (max_period_time > 10000) {
      //Serial.print("Sanity check failed...");
      Serial.println(max_period_time);
      return get_min_trigger_low_time();
    }
  
    //Serial.print("Low time: ");
    //Serial.println(signal_low_time);
  
    //Serial.print("High time: ");
    //Serial.println(signal_high_time);

  }

  return min_signal_low_time;
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
  noInterrupts();           // disable all interrupts
  cli();
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  timer1_counter = 65536 - (16 * 1000000 / 256 / timer_freq);

  //Serial.println(TIMER_FREQ);
  //Serial.println(timer1_counter);

 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  disable_timer();
  
  
}



void setup() { 

  //delay(1000);
  Serial.begin(250000);

  pinMode(modulate_enable, INPUT);
  
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);

  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);

  digitalWrite(30, LOW);
  digitalWrite(31, LOW);


#ifdef HARDCODE_MODULATE_TIME

  time_to_modulate_per_period = 7000; //us

#else

  uint32_t min_trigger_low_time = get_min_trigger_low_time();

  time_to_modulate_per_period = min(7000 /* us */, min_trigger_low_time);
  //Serial.print("min_trigger_low_time: "); Serial.println(min_trigger_low_time);

#endif

  modulate_times_per_period = time_to_modulate_per_period * timer_freq / 1000000 - 1;


  /*
  Serial.print("time_to_modulate_per_period: "); Serial.println(time_to_modulate_per_period);
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
  m_seq_idx = 0;
  m_seq_create(poly, n, 1, m_seq);

  /*for (int i = 0; i < L; i++) {
    Serial.print(m_seq[i]); Serial.print(" ");
  }
  Serial.println();*/


  sample_idx = 0;

  init_timer();

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



unsigned long start_time, stop_time, start_time2, stop_time2 = 0;


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

uint8_t flag = 0;

ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  if (flag == 1) {
    flag = 0;
    PORTC = PORTC | (1 << 7);
    
  } else {
    flag = 1;
    PORTC = PORTC & ~(1 << 7); // pin 30
  }


  if (sample_idx < modulate_times_per_period) {

    uint16_t read_value = readADC(0);
    uint16_t scaled_value = 0;
    if (read_value >= avg_adc_value) {
      scaled_value = read_value - avg_adc_value;
    } else {
      scaled_value = avg_adc_value - read_value ;
    }


    /*if (scaled_value < 50) {
      scaled_value = 0;
    } else 
      scaled_value = 1;*/


    
    adc_buffer[adc_idx++] = scaled_value;


    
    sample_idx++;
    if (adc_idx >= ADC_BUFFER_SIZE) {
      adc_idx = 0;
      adc_buffer_full = 1;
      //Serial.print("T: ");
      //Serial.println(micros() - start_time);
      start_time = micros();
      //noInterrupts();
    }
  }
  
}






void isr_change(void) {
  int state = digitalRead(modulate_enable);

  /*if (state == 1) {
    PORTC = PORTC & ~(1 << 6); // pin 31
  } else {
    PORTC = PORTC | (1 << 6);
  }*/
  

  //Serial.println(state);
  //Serial.println();

  if (state == 1) {
    sample_idx = 0;
    
    //disable_timer();
    TIMSK1 &= ~(1 << TOIE1); 
    PORTC = PORTC & ~(1 << 6); // pin 30
  } else {
    
    //enable_timer();
    TCNT1 = 0;
    TIMSK1 |= (1 << TOIE1);
    PORTC = PORTC | (1 << 6);
  }

}


int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}




uint16_t times_led_detected = 0;

void loop() {

  /*if (millis() % 1000 == 0) {
    Serial.println("OAJNSDOIDAS");
  }*/
  
  
  /*uint16_t read_value = readADC(0);
  uint16_t scaled_value = 0;
  if (read_value >= avg_adc_value) {
    scaled_value = read_value - avg_adc_value;
  } else {
    scaled_value = avg_adc_value - read_value ;
  }

  Serial.println(scaled_value);*/
  


  //start_time = micros();


  /*if (Serial.available() > 0) {

     
    while (Serial.available() > 0) {
      Serial.println(Serial.read());
      //Serial.read();
    }

    noInterrupts();
    cli();

    init_timer();

    interrupts();
    sei();

      
  }*/

  


  
  if (adc_buffer_full == 1) {
    noInterrupts();
    cli();

    
     // pin 30
    //PORTC = PORTC | (1 << 7);

    /*stop_time = micros();
    Serial.print("T1: ");
    Serial.println(stop_time - start_time);*/

    
      
    
    

    uint16_t sample_min = find_min(adc_buffer, ADC_BUFFER_SIZE);
    uint16_t sample_max = find_max(adc_buffer, ADC_BUFFER_SIZE);

    /*Serial.print("sample_min: "); Serial.println(sample_min);
    Serial.print("sample_max: "); Serial.println(sample_max);*/

    /*for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
      float s = (float) ((adc_buffer[i] - sample_min));
      s /= (float) sample_max;
      Serial.println(s);
      
    }*/

    for (int offset = 0; offset < (ADC_BUFFER_SIZE - L); offset++) {

      float corr_sum = 0;
      float signal_sum = 0;


      //start_time2 = micros();

      //PORTC = PORTC | (1 << 6);
        
      for (int i = 0; i < L; i++) {

        float scaled_sample = ((float)(adc_buffer[i + offset] - sample_min)) / ((float)sample_max);
        
        

        //float scaled_sample = adc_buffer[i + offset];


        signal_sum += scaled_sample;
        
        float r_chip = 1 - 2 * ((int8_t)m_seq[i]);
        corr_sum += (scaled_sample) * r_chip;

      }

      float calc_num_of_tx = signal_sum / (1 << (n - 1));
      //Serial.println(calc_num_of_tx);

      float normalized_l_corr = (-2 * corr_sum) - calc_num_of_tx;
      Serial.println(normalized_l_corr);
      
      //PORTC ^= (1 << 6);

      if (normalized_l_corr >= ((float)L/2)) {
        times_led_detected++;
      }

      /*if (normalized_l_corr >= ((float)L/2)) {
        //Serial.println("D");
        PORTC = PORTC | (1 << 5);
      } else {
        
        PORTC = PORTC & ~(1 << 5);
      }*/


      //PORTC = PORTC & ~(1 << 6);
      /*stop_time2 = micros();

      Serial.print("T2: ");
      Serial.println(stop_time2 - start_time2);*/

    }

    //stop_time = micros();

    //Serial.println(times_led_detected);
    times_led_detected = 0;

    //Serial.println(stop_time - start_time);


    
    //PORTC = PORTC & ~(1 << 7);

    
    




    /*uint16_t *y = new uint16_t[ADC_BUFFER_SIZE];
    
    for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
      y[i] = 0;;
    }

    float RC = 1.0/(1*2*3.14);
    float dt = 1.0/TIMER_FREQ;
    float alpha = dt/(RC+dt);

    float beta = 0.0025;

    y[0] = adc_buffer[0] + 0;

    for (uint16_t n = 1; n < ADC_BUFFER_SIZE; n++) {
      //y[n] = y[n - 1] + alpha * (adc_buffer[n] - y[n - 1]);
      y[n] = (1 - beta) * y[n-1] + beta * adc_buffer[n];
    }

    for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
      Serial.println(y[i]);
    }
    //Serial.println("-------------");

    delete y;*/
  
    /*for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
      Serial.println(adc_buffer[i]);
    }*/
    //Serial.println("-------------");

    /*for (int i = 0; i < L; i++) {
      Serial.println(m_seq[i]);
    }
    Serial.println("-------------");*/

    //Serial.println(freeRam());

    adc_buffer_full = 0;
    sample_idx = 0;

    //start_time = micros();
    interrupts();
    sei();
    
  }

  
}

uint16_t find_min(uint16_t *array, uint16_t length) {
  uint16_t min = 1 << 15;

  for (uint16_t i = 0; i < length; i++) {
    if (array[i] < min)
      min = array[i];
  }

  return min;
}

uint16_t find_max(uint16_t *array, uint16_t length) {
  uint16_t max = 0;

  for (uint16_t i = 0; i < length; i++) {
    if (array[i] > max)
      max = array[i];
  }

  return max;
}




