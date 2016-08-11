
#include <SPI.h>

#define ADC_CS 38 // arduino PIN number
#define PORTD_ADC_CS 7 // ATMEGA PIN number

#define modulate_enable 3

#define TIME_TO_MODULATE 8 // in ms
#define TIMER_FREQ 1000 //Hz
#define MODULATE_TIMES (TIME_TO_MODULATE * (TIMER_FREQ / 1000) - 1)

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

void enable_timer() {
  TCNT1 = 0;
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

void disable_timer() {
  TIMSK1 &= ~(1 << TOIE1); 
}



void setup() { 

  delay(1000);
  Serial.begin(250000);

  pinMode(modulate_enable, INPUT);
  
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);

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

    for (int i = 0; i < L; i++) {
    Serial.print(m_seq[i]); Serial.print(" ");
  }
  Serial.println();


  sample_idx = 0;

  // initialize timer1 -
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  timer1_counter = 65536 - (16 * 1000000 / 256 / TIMER_FREQ);

  //Serial.println(TIMER_FREQ);
  //Serial.println(timer1_counter);

 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  disable_timer();
  
  

  attachInterrupt(digitalPinToInterrupt(modulate_enable), isr_change, CHANGE );

  

  avg_adc_value = get_ground_adc_readings();


  interrupts();             // enable all interrupts
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
// with DIV8 & MMIO -> 20 us 
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

  if (sample_idx < MODULATE_TIMES) {

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
      //noInterrupts();
    }
  }
  
}






void isr_change(void) {
  int state = digitalRead(modulate_enable);

  //Serial.println(state);
  //Serial.println();

  if (state == 1) {
    sample_idx = 0;
    disable_timer();
  } else {
    enable_timer();
  }

}





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
  



  
  if (adc_buffer_full == 1) {
    noInterrupts();

    //long start_time = micros();

    

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

    }



    
    //long stop_time = micros();
    //Serial.println(stop_time - start_time);
    //Serial.println((stop_time - start_time) / (ADC_BUFFER_SIZE / L));


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

    

    adc_buffer_full = 0;
    sample_idx = 0;
    interrupts();
    
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




