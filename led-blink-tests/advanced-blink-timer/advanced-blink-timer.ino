#define led 13

#define TIMER_FREQ 1000 //Hz -> 1 / 1000 = 1 ms

uint16_t timer1_counter;

uint16_t counter;
uint8_t off_counter;

uint8_t enable = 0;

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

  counter = 0;

  off_counter = 0;
}






ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  counter++;

  if (counter % 5 == 0) {
    // if counter divisible by 5 -> f = 1000 Hz, so this is true after 5 isr's -> 200 Hz

    enable = 1 ^ enable;
      
  }


  if (enable) {
    if (off_counter < 4) {
      
      off_counter++;
      if (off_counter >= 4)
        off_counter = 0;

      digitalWrite(led, 1 ^ digitalRead(led));
    }

    
  } else {
    digitalWrite(led, 1);
  }

  Serial.println(digitalRead(led));

  
  
  
}

void loop() {

}

