#define led 2
#define modulate_enable_pin 3

#define sensing_pin 7

#define OFF HIGH
#define ON LOW

#define SIZE 30

uint32_t *time_buffer;
uint16_t *adc_buffer;


int modulating_flag = 0;
int dumping_flag = 0;

void setup() {
  pinMode(led, OUTPUT);
  pinMode(modulate_enable_pin, INPUT);

  pinMode(sensing_pin, OUTPUT);

  Serial.begin(250000);

  time_buffer = new uint32_t[SIZE];
  adc_buffer = new uint16_t[SIZE];

  modulating_flag = 0;
  dumping_flag = 0;


  attachInterrupt(digitalPinToInterrupt(modulate_enable_pin), isr_change, CHANGE );


  digitalWrite(led, ON);
  digitalWrite(sensing_pin, LOW);

}

void loop() {
  
}

void isr_change() {
  if (dumping_flag == 0 && modulating_flag == 0 && digitalRead(modulate_enable_pin) == 0) {

    modulating_flag = 1;

    for (int i = 0; i < SIZE; i++) {

      //digitalWrite(led, 1 ^ digitalRead(led));
  

      digitalWrite(sensing_pin, HIGH);
      time_buffer[i] = micros();
      adc_buffer[i] = analogRead(A0);
      digitalWrite(sensing_pin, LOW);
  
    }

    modulating_flag = 0;
    
  } 
  
  if (dumping_flag == 0 && modulating_flag == 0 && digitalRead(modulate_enable_pin) == 1) {

    dumping_flag = 1;

    Serial.println("**********************************************");
    for (int i = 0; i < SIZE; i++) {
      Serial.print(time_buffer[i]); Serial.print(": "); Serial.println(adc_buffer[i]);
    }
    Serial.println("**********************************************");

    delayMicroseconds(8000);
    dumping_flag = 0;
    
  }
}

