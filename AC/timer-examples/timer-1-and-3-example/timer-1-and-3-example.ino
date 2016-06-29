
#define TIMER_1_FREQ 1000 //Hz


#define TIMER_3_FREQ 500 //Hz

volatile uint16_t timer1_counter;
volatile uint16_t timer3_counter;


void enable_timer1() {
  TCNT1 = 0;
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
}

void disable_timer1() {
  TIMSK1 &= ~(1 << TOIE1); 
}

void init_timer1() {
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  timer1_counter =  65536 - (16 * 1000000 / 256 / TIMER_1_FREQ);

  Serial.print("T1 f: "); Serial.println(TIMER_1_FREQ);

  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  
  disable_timer1();
  enable_timer1();
}



void enable_timer3() {
  TCNT3 = 0;
  TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
}

void disable_timer3() {
  TIMSK3 &= ~(1 << TOIE3); 
}

void init_timer3() {
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;

  timer3_counter =  65536 - (16 * 1000000 / 256 / TIMER_3_FREQ);

  Serial.print("T3 f: "); Serial.println(TIMER_3_FREQ);

  TCNT3 = timer3_counter;   // preload timer
  TCCR3B |= (1 << CS12);    // 256 prescaler 
  
  disable_timer3();
  enable_timer3();
}

void setup() {
  Serial.begin(250000);


  init_timer1();
  init_timer3();
  
  interrupts();             // enable all interrupts
 
}


ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  Serial.print("T1: "); Serial.println(micros());

 
}

ISR(TIMER3_OVF_vect) {      // interrupt service routine 
  TCNT3 = timer3_counter;   // preload timer

  Serial.print("T3: "); Serial.println(micros());

 
}




void loop() {  

}


