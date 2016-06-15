#define led 8
#define modulate_enable_pin 3

#define OFF LOW
#define ON HIGH

#define TIMER_FREQ 2000 //Hz

#define TIMES 4


volatile uint16_t timer1_counter;
volatile uint8_t timer_enable;

volatile uint8_t ctr = 0;

volatile uint8_t idx = 0;

void setup() {
  pinMode(led, OUTPUT);
  pinMode(modulate_enable_pin, INPUT);
  
  digitalWrite(led, OFF);

  Serial.begin(250000);

  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  timer1_counter = /*(1 << 16)*/ 65536 - (16 * 1000000 / 256 / TIMER_FREQ);

  //Serial.println(TIMER_FREQ);
  //Serial.println(timer1_counter);

 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt


  
  attachInterrupt(digitalPinToInterrupt(modulate_enable_pin), isr_change, CHANGE );

  timer_enable = 0;
  
  interrupts();             // enable all interrupts
}

ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  if (timer_enable == 1)
    Serial.println(analogRead(A0));
  
  if (ctr++ % 2 == 0) { 
    if (idx >= TIMES) {
      timer_enable = 0;
    }
  
    if (timer_enable == 1) {
      idx++;
      digitalWrite(led, 1 ^ digitalRead(led));
        
    } else {
      digitalWrite(led, ON);
      
    }
  }

}

void isr_change(void) {
  uint8_t st = digitalRead(modulate_enable_pin);
  timer_enable = st ^ 1;
  
  if (st == 0)
    idx = 0;
}



void loop() {
  
}

