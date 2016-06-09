#define modulate_enable_pin 3

#define sensing_pin 7

#define SIZE 800

uint16_t *adc_buffer;


void setup() {
  pinMode(sensing_pin, OUTPUT);
  digitalWrite(sensing_pin, LOW);

  pinMode(modulate_enable_pin, INPUT);

  Serial.begin(250000);

  adc_buffer = new uint16_t[SIZE];

}

int i;

int buffer_clear = 0;

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


