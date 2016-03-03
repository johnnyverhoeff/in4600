
uint8_t code[] = {1,1,1,1,1,0,1,1,1,0,0,0,1,0,1,0,1,1,0,1,0,0,0,0,1,1,0,0,1,0,0};

uint8_t base_shift = 31 - 3;

uint8_t compare_arrays(uint8_t a[], uint8_t b[], uint8_t l, uint8_t shift) {
  for (uint8_t i = 0; i < l; i++) {
    if (a[(i + base_shift) % l] != b[(i + shift) % l]) return 0;
  }
  return 1;
}


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  Serial.println("begun");

  uint8_t code_length = sizeof(code) / sizeof(uint8_t);


  for (uint8_t shift = 0; shift < code_length; shift++) {

    uint8_t *shifted_and_xored_code;
    shifted_and_xored_code = new uint8_t[code_length];

    // saving the shifted version of the code..
    for (uint8_t i = 0; i < code_length; i++) {
      shifted_and_xored_code[i] = code[(i + base_shift) % code_length] ^ code[(i + shift) % code_length];
    }

    /*Serial.print("shift=");Serial.print(shift);Serial.print(": ");
    for (uint8_t i = 0; i < code_length; i++) {
      Serial.print(shifted_and_xored_code[i]);Serial.print(" ");
    }
    Serial.println();*/

    for (uint8_t shift2 = 0; shift2 < code_length; shift2++) {
      if (compare_arrays(code, shifted_and_xored_code, code_length, shift2)) {
        Serial.print("Shift: ");Serial.print(shift); Serial.print(", Shift2: ");Serial.println(code_length - shift2);
      }
    
    }

  }
  

}

void loop() {
  // put your main code here, to run repeatedly:

}
