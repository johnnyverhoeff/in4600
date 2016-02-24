uint8_t pn_code[] = {1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0};
uint8_t leds[] = {3, 5, 6};

uint8_t led0_data = 1;
uint8_t led1_data = 1;

uint8_t time_shift_led1 = 0;

#define NUM_OF_LEDS 3
#define CODE_LENGTH 15
#define SENSOR_PIN A0



#define LOWER_THRESHOLD 6
#define UPPER_THRESHOLD 9

uint8_t counter = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Begun.");

  for (uint8_t i = 0; i < NUM_OF_LEDS; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  randomSeed(analogRead(A1));

}

void loop() {
  // put your main code here, to run repeatedly:

  uint8_t num_of_ones_led0 = 0;
  uint8_t num_of_zeroes_led0 = 0;

  uint8_t num_of_ones_led1 = 0;
  uint8_t num_of_zeroes_led1 = 0;

  uint8_t decoded_led0_data = 2;
  uint8_t decoded_led1_data = 2;

  led0_data = random(0, 2);
  led1_data = random(0, 2);

  Serial.println("**********************");
  for (int i = 0; i < CODE_LENGTH; i++) {
    digitalWrite(leds[0], led0_data ^ pn_code[i]);
    digitalWrite(leds[1], led1_data ^ pn_code[(i + time_shift_led1) % CODE_LENGTH]);

    //Serial.println(analogRead(SENSOR_PIN));

    uint16_t read_value = analogRead(SENSOR_PIN);

    if ((read_value > 1) ^ pn_code[i] == 1) 
      num_of_ones_led0++;
    else
      num_of_zeroes_led0++;

    if ((read_value > 1) ^ pn_code[(i + time_shift_led1) % CODE_LENGTH] == 1) 
      num_of_ones_led1++;
    else
      num_of_zeroes_led1++;
    
  }

  Serial.print("Time shift: "); Serial.println(time_shift_led1);

  //Serial.print("Original led0 data: "); Serial.println(led0_data);

  if (num_of_ones_led0 > UPPER_THRESHOLD && num_of_zeroes_led0 < LOWER_THRESHOLD)
    decoded_led0_data = 1;
  else if (num_of_zeroes_led0 > UPPER_THRESHOLD && num_of_ones_led0 < LOWER_THRESHOLD)
    decoded_led0_data = 0;
  else 
    decoded_led0_data = 2;

  //Serial.print("Decoded  led0 data: "); Serial.println(decoded_led0_data);
  
  //Serial.println();
  //Serial.print("Original led1 data: "); Serial.println(led1_data);

  if (num_of_ones_led1 > UPPER_THRESHOLD && num_of_zeroes_led1 < LOWER_THRESHOLD)
    decoded_led1_data = 1;
  else if (num_of_zeroes_led1 > UPPER_THRESHOLD && num_of_ones_led1 < LOWER_THRESHOLD)
    decoded_led1_data = 0;
  else 
    decoded_led1_data = 2;

  //Serial.print("Decoded  led1 data: "); Serial.println(decoded_led1_data);

  if (led0_data != decoded_led0_data) {
    Serial.print("Original led0 data: "); Serial.println(led0_data);
    Serial.print("Decoded  led0 data: "); Serial.println(decoded_led0_data);
  }

  if (led1_data != decoded_led1_data) {
    Serial.print("Original led1 data: "); Serial.println(led1_data);
    Serial.print("Decoded  led1 data: "); Serial.println(decoded_led1_data);
  }
  
  Serial.println("**********************");
  delay(1000);

}
