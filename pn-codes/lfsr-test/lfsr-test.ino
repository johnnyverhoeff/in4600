uint8_t start_state = 0x3;
uint8_t lfsr = start_state;
uint8_t bit;

uint16_t period = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Begun.");

  do {
    bit = ((lfsr >> 0) ^ (lfsr >> 1)) & 1;
    lfsr = (lfsr >> 1) | (bit << 1);

    Serial.print("Period: "); Serial.print(period);
    Serial.print(", lfsr: "); Serial.print(lfsr); 
    Serial.print(", bit: "); Serial.println(bit);
    
    ++period;
    
  } while (lfsr != start_state);

}

void loop() {
  // put your main code here, to run repeatedly:

}
