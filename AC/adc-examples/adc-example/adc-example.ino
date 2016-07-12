#define led 13
#define modulate_enable_pin 3

#define OFF LOW
#define ON HIGH

#define SIZE 200

uint32_t *time_buffer;
uint16_t *adc_buffer;


int modulating_flag = 0;
int dumping_flag = 0;

void setup() {
  pinMode(led, OUTPUT);
  pinMode(modulate_enable_pin, INPUT);

  Serial.begin(250000);

  //time_buffer = new uint32_t[SIZE];
  adc_buffer = new uint16_t[SIZE];

  modulating_flag = 0;
  dumping_flag = 0;


  attachInterrupt(digitalPinToInterrupt(modulate_enable_pin), isr_change, CHANGE );


  digitalWrite(led, ON);

}

void loop() {
  
}

void isr_change() {
  if (dumping_flag == 0 && modulating_flag == 0 && digitalRead(modulate_enable_pin) == 0) {

    modulating_flag = 1;

    for (int i = 0; i < SIZE; i++) {

      digitalWrite(led, 1 ^ digitalRead(led));
   
      //time_buffer[i] = micros();
      adc_buffer[i] = analogRead(A0);

      delayMicroseconds(2000);
    }

    modulating_flag = 0;
    
  } 
  
  if (dumping_flag == 0 && modulating_flag == 0 && digitalRead(modulate_enable_pin) == 1) {

    dumping_flag = 1;

    //Serial.println("**********************************************");
    for (int i = 0; i < SIZE; i++) {
      /*Serial.print(time_buffer[i]); Serial.print(": ");*/ Serial.println(adc_buffer[i]);
    }
    Serial.println("**********************************************");

    //delayMicroseconds(8000);
    dumping_flag = 0;
    
  }
}

