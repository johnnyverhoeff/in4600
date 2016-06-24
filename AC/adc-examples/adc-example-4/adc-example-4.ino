#define led 2
#define modulate_enable_pin 3

#define sensing_pin 7

#define SIZE 800

#define TIMER_FREQ 1000 //Hz

uint16_t *adc_buffer;

volatile uint16_t timer1_counter;


void setup() {
  pinMode(sensing_pin, OUTPUT);
  digitalWrite(sensing_pin, LOW);

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  pinMode(modulate_enable_pin, INPUT);

  Serial.begin(250000);

  adc_buffer = new uint16_t[SIZE];

  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  //timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
  //timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
  //timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz

  timer1_counter = /*(1 << 16)*/ 65536 - (16 * 1000000 / 256 / TIMER_FREQ);


 
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

  interrupts();             // enable all interrupts
}

int i;

int buffer_clear = 0;


ISR(TIMER1_OVF_vect) {      // interrupt service routine 
  TCNT1 = timer1_counter;   // preload timer

  if (digitalRead(modulate_enable_pin) == 0) {
      digitalWrite(led, digitalRead(led) ^ 1);
  }
}

void loop() {


  for (i = 0; ((i < SIZE) && (digitalRead(modulate_enable_pin) == 0)) ; i++) {
    adc_buffer[i] = analogRead(A0);
    buffer_clear = 0;
  }

  if (buffer_clear == 0 && digitalRead(modulate_enable_pin) == 1) {
    for (int j = 0; j < i; j++) {
      Serial.println(adc_buffer[j]);
    }
    buffer_clear = 1;
  }
  
}


