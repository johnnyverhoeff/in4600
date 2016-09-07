


#include <SPI.h>

#define ADC_CS 38 // arduino PIN number
#define PORTD_ADC_CS 7 // ATMEGA PIN number

#define modulate_enable 3

#define NUM_OF_TIMES_CHECK_TRIGGER_SIGNAL 100

#define HARDCODE_MODULATE_TIME
// define this for hardcoded 7 ms, else for auto-detection

#define USE_TEST_OUPUT_PINS
// define this for letting the two isr's, timer & trigger, toggle pins for testing purposes.

uint32_t  time_to_modulate_per_period,
          modulate_times_per_period;

const uint32_t timer_freq = 10000; //Hz




uint16_t timer1_counter;

#define ADC_BUFFER_SIZE 2500
uint16_t *adc_buffer;
uint16_t adc_idx;
uint8_t adc_buffer_full;

uint8_t sample_idx = 0;


uint8_t *trigger_buffer;






volatile uint8_t timer_enable = 0;

volatile uint32_t loop_counter = 0;

uint32_t start_low_time = 0;

uint16_t avg_adc_value;


/*volatile uint32_t last_time_clear_flag = 0;

void clear_timer_flag(void) {
  last_time_clear_flag = micros();
}*/














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






  

  SPI.begin();
  /*SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV64);*/
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  //Serial.println("SPI started...");


  adc_buffer = new uint16_t[ADC_BUFFER_SIZE];
  trigger_buffer = new uint8_t[ADC_BUFFER_SIZE];
  adc_idx = 0;
  adc_buffer_full = 0;

  for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
    adc_buffer[i] = 0;
    trigger_buffer[i] = 0;
  }


  
 

  
  





  /*for (int i = 0; i < L; i++) {
    Serial.print(m_seq[i]); Serial.print(" ");
  }
  Serial.println();*/


  sample_idx = 0;

  noInterrupts();
  cli();

  init_timer();
  disable_timer();

  enable_timer();
  interrupts();             // enable all interrupts
  sei();
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
  



  uint16_t read_value = readADC(0);
  uint16_t scaled_value = 0;
  
  if (read_value >= avg_adc_value) {
    scaled_value = read_value - avg_adc_value;
  } else {
    scaled_value = avg_adc_value - read_value ;
  }

  
  adc_buffer[adc_idx] = read_value/*scaled_value*/;
  trigger_buffer[adc_idx] = digitalRead(3);
  adc_idx++;

  sample_idx++;
  
  if (adc_idx >= ADC_BUFFER_SIZE) {
    adc_idx = 0;
    adc_buffer_full = 1;
  }

}








void loop() {

  // to make sure the timer wont fail...
  loop_counter++;
  if (loop_counter % 10000 == 0) {
    init_timer();
  }

 
  if (adc_buffer_full == 1) {
    
    noInterrupts();
    cli();

    Serial.println("BEGIN ADC********************");

    for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
      Serial.println(adc_buffer[i]);
    }

    Serial.println("END ADC------------------------");
    Serial.println("BEGIN TRIGGER********************");
    for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++) {
      Serial.println(trigger_buffer[i]);
    }
    Serial.println("END TRIGGER------------------------");
    

    adc_buffer_full = 0;
    sample_idx = 0;
    
    interrupts();
    sei(); 
  }
}
