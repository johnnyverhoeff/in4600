#define led 13
#define modulate_enable 3

#define NUM_OF_TIMES_CHECK_TRIGGER_SIGNAL 100

void setup() {
  pinMode(led, OUTPUT);
  pinMode(modulate_enable, INPUT);

  Serial.begin(250000);

  digitalWrite(led, LOW);



}





uint32_t get_min_trigger_low_time(void) {
  uint32_t  avg_signal_low_time, 
          min_signal_low_time = 10000,
          max_signal_low_time = 0,
          
          avg_signal_high_time,
          min_signal_high_time = 10000,
          max_signal_high_time = 0;
          
  for (int i = 0; i < NUM_OF_TIMES_CHECK_TRIGGER_SIGNAL; i++) {
    
    while (digitalRead(modulate_enable) == 0); //wait while signal is low
    while (digitalRead(modulate_enable) == 1); //wait while signal is high
    
    //signal is low.
  
    uint32_t begin_time_signal_low = micros();
  
    while (digitalRead(modulate_enable) == 0); //wait while signal is low
  
    uint32_t end_time_signal_low = micros();
    uint32_t begin_time_signal_high = end_time_signal_low;
  
    while (digitalRead(modulate_enable) == 1); //wait while signal is high
  
    uint32_t end_time_signal_high = micros();


    uint32_t signal_low_time = end_time_signal_low - begin_time_signal_low;
    uint32_t signal_high_time = end_time_signal_high - begin_time_signal_high;

    min_signal_low_time = min(min_signal_low_time, signal_low_time);
    max_signal_low_time = max(max_signal_low_time, signal_low_time);

    min_signal_high_time = min(min_signal_high_time, signal_high_time);
    max_signal_high_time = max(max_signal_high_time, signal_high_time);


    uint32_t max_period_time = min_signal_low_time + max_signal_high_time;

    if (max_period_time > 10000) {
      Serial.println("Sanity check failed...");
      return get_min_trigger_low_time();
    }
  
    //Serial.print("Low time: ");
    //Serial.println(signal_low_time);
  
    //Serial.print("High time: ");
    //Serial.println(signal_high_time);

  }

  return min_signal_low_time;
}


void loop() {

  



  /*Serial.print("min_signal_low_time: "); Serial.println(min_signal_low_time);
  Serial.print("max_signal_low_time: "); Serial.println(max_signal_low_time);

  Serial.print("min_signal_high_time: "); Serial.println(min_signal_high_time);
  Serial.print("max_signal_high_time: "); Serial.println(max_signal_high_time);*/


  uint32_t t = get_min_trigger_low_time();


  Serial.println(t);
  /*Serial.print(".9 * t: "); Serial.println(.9 * t);*/
  



  

  
}






