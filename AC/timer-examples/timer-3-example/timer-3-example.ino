

#define TIMER_FREQ 1000 //Hz
volatile uint16_t timer3_counter;



void enable_timer3() {
  TCNT3 = 0;
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
}

void disable_timer3() {
  TIMSK3 &= ~(1 << TOIE3); 
}

void setup() {
  Serial.begin(250000);


  // initialize timer3 -
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;


  timer3_counter =  65536 - (16 * 1000000 / 256 / TIMER_FREQ);

  Serial.println(TIMER_FREQ);
  Serial.println(timer3_counter);

 
  TCNT3 = timer3_counter;   // preload timer
  TCCR3B |= (1 << CS12);    // 256 prescaler 
  
  disable_timer3();
  enable_timer3();
  
  interrupts();             // enable all interrupts
 
}


ISR(TIMER3_OVF_vect) {      // interrupt service routine 
  TCNT3 = timer3_counter;   // preload timer

  Serial.println(micros());

 
}




void loop() {  

}


