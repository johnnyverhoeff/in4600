#define TRIGGER_PIN_1 12
#define TRIGGER_PIN_2 13

#define TIMER_FREQ 200 //Hz

volatile uint16_t timer1_counter;

void setup() {
  pinMode(TRIGGER_PIN_1, OUTPUT);
  digitalWrite(TRIGGER_PIN_1, LOW);

  pinMode(TRIGGER_PIN_2, OUTPUT);
  digitalWrite(TRIGGER_PIN_2, LOW);

  Serial.begin(250000);




  // initialize timer1 -
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  timer1_counter = /*(1 << 16)*/ 65536 - (16 * 1000000 / 256 / TIMER_FREQ);

  Serial.println(TIMER_FREQ);
  Serial.println(timer1_counter);

 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts

}

void loop() {
  // put your main code here, to run repeatedly:

}




ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer
  
  //Serial.println("trigger" );
  digitalWrite(TRIGGER_PIN_1, digitalRead(TRIGGER_PIN_1) ^ 1);
  digitalWrite(TRIGGER_PIN_2, digitalRead(TRIGGER_PIN_2) ^ 1);

  
}
