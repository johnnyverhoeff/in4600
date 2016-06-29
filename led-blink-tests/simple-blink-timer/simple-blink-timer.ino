#define led 13

#define TIMER_FREQ 200 //Hz

uint16_t timer1_counter;

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  Serial.begin(250000);

  // initialize timer1 -
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  timer1_counter = 65536 - (16 * 1000000 / 256 / TIMER_FREQ);

  //Serial.println(TIMER_FREQ);
  //Serial.println(timer1_counter);

 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer
  digitalWrite(led, digitalRead(led) ^ 1);
}

void loop() {
}

