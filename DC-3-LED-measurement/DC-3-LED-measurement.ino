int sensorPin = A0;

int ledPins[] = {3, 5, 6};

int minVal = 190;
int maxVal = 215;

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  Serial.begin(115200);

  Serial.println("Begun");

}

void detectLEDs() {
  int adc = analogRead(sensorPin);

  if (adc >= 0 && adc <= minVal) {
    Serial.println("0");
  } else if (adc >= minVal && adc <= maxVal) {
    Serial.println("1");
  } else if (adc >= 2 * minVal && adc <= 2 * maxVal) {
    Serial.println("2");
  } else if (adc >= 3 * minVal && adc <= 3 * maxVal) {
    Serial.println("3");
  }
}

void loop() {

  /*
   * Code to run one by one LED and read the ADC and output on serial
   */
  /*
  delay(1000);
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPins[i], LOW);
  }
  Serial.print("A0: ");
  Serial.println(analogRead(sensorPin));
  

  for (int i = 0; i < 3; i++) {
    delay(1000);
    digitalWrite(ledPins[i], HIGH);
    Serial.print("A0: ");
    Serial.println(analogRead(sensorPin));
  }
  */



  /* 
   *  Code to run only one LED segment at a time and output ADC to serial.
   *  Seems between 190 and 215, for one LED segment.
   */
  /*for (int i = 0; i < 3; i++) {
    
    digitalWrite(ledPins[i], HIGH);
    Serial.print("A0: ");
    Serial.println(analogRead(sensorPin));
    delay(1000);
    digitalWrite(ledPins[i], LOW);
    
  }
  */


  /*
   * Code to randomly set the three LED segment on or off
   * And then check how many LED segment are actually one.
   */

  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPins[i], (int)random(0, 2));
  }

  detectLEDs();

  delay(2000);

  for (int i = 0; i < 3; i++)
    digitalWrite(ledPins[i], LOW);
  

}
