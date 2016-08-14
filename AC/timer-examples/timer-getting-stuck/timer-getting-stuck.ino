#include <SPI.h>

#define ADC_CS 38 // arduino PIN number
#define PORTD_ADC_CS 7 // ATMEGA PIN number



#define modulate_enable 3

#define TIMER_FREQ 15000 //Hz



uint16_t timer1_counter;






volatile uint8_t timer_enable = 0;

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
  /*SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV64);*/

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));

  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);




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



  interrupts();             // enable all interrupts
}




uint16_t counter = 0;



uint8_t sample_idx = 0;

ISR(TIMER1_OVF_vect) {      // interrupt service routine
  TCNT1 = timer1_counter;   // preload timer

  PORTC ^= (1 << 7); // pin 30




  PORTD = PORTD & ~(1 << PORTD_ADC_CS);
  SPI.transfer(0x06);
  uint8_t rx1 = SPI.transfer(0);
  uint8_t rx2 = SPI.transfer(0xFF);
  PORTD = PORTD | (1 << PORTD_ADC_CS);
  uint16_t val =  (((rx1 & 0x0F) << 8) | rx2);

    
  counter++;

  if (counter >= 1600) {
    noInterrupts();
  }
    



}






void isr_change(void) {
  int state = digitalRead(modulate_enable);

  /*if (state == 1) {
    PORTC = PORTC & ~(1 << 7); // pin 30
    } else {
    PORTC = PORTC | (1 << 7);
    }*/


  //Serial.println(state);
  //Serial.println();

  if (state == 1) {


    //disable_timer();
    TIMSK1 &= ~(1 << TOIE1);
    PORTC = PORTC & ~(1 << 6); // pin 31
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

  if (counter >= 1600) {
    counter = 0;
    cli();
    //noInterrupts();
    //Serial.println(freeRam());
    sei();
    //interrupts();
  }



}




