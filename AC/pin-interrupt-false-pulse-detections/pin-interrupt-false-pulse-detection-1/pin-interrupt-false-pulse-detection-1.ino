#define led 13
#define interrupt_pin 3

/*#define NUM_OF_ENTRIES 100

unsigned long time_buffer[NUM_OF_ENTRIES];
uint8_t value_buffer[NUM_OF_ENTRIES];
*/


unsigned long first_high_time = 0;
unsigned long last_high_time = 0;

void setup() {
  pinMode(led, OUTPUT);
  pinMode(interrupt_pin, INPUT);

  Serial.begin(250000);

  digitalWrite(led, LOW);

  first_high_time = 0;
  last_high_time = 0;
}


void loop() {


  /*for (uint16_t i = 0; i < NUM_OF_ENTRIES; i++){
    time_buffer[i] = micros();
    value_buffer[i] = PIND & (1 << interrupt_pin) & 1;
  }

  for (uint16_t i = 0; i < NUM_OF_ENTRIES; i++){
    Serial.print(time_buffer[i]); Serial.print(": "); Serial.println(value_buffer[i]);
  }

  */

  //while (digitalRead(interrupt_pin) == 1);

  while ((PIND & (1 << interrupt_pin)) == 0) ; // wait for int. to go high
  //while (digitalRead(interrupt_pin) == 0);

  // now it is high;
  first_high_time = micros();

  while ((PIND & (1 << interrupt_pin)) != 0) ; // wait for int. to go low
  //while (digitalRead(interrupt_pin) == 1);


  last_high_time = micros();

  unsigned long delta_t = last_high_time - first_high_time;

  if (delta_t < 1500) {
    Serial.print("delta_t: "); Serial.println(delta_t);
  }
    
}






