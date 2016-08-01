
#include <SPI.h>

#define ADC_CS 7

void setup() { 

  Serial.begin(250000);


  
  pinMode(ADC_CS, OUTPUT);
  digitalWrite(ADC_CS, HIGH);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  Serial.println("SPI started...");




  
}

//unsigned long start = 0;
//unsigned long end_t = 0;

uint16_t readADC(uint8_t channel) {

  //SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  
  //digitalWrite(ADC_CS, LOW);
  PORTD = PORTD & ~(1 << ADC_CS);


  uint8_t spi_tx = 0x06; // Refer to FIGURE 6-1 in MCP3204 datasheet.

  //start = micros();
  
  SPI.transfer(spi_tx);

  spi_tx = channel << 6;
  
  uint8_t rx1 = SPI.transfer(spi_tx);
  uint8_t rx2 = SPI.transfer(0xFF);

  //end_t = micros();

  //digitalWrite(ADC_CS, HIGH);
  PORTD = PORTD | (1 << ADC_CS);
  //SPI.endTransaction();

  return (((rx1 & 0x0F) << 8) | rx2);
  
}



void loop() {
  Serial.println(readADC(3));
  //Serial.println(analogRead(A3));
  //delay(1000);

  //readADC(3);
  //Serial.println(1 / ((float)(end_t - start) / 64));

/*
  //PORTD = PORTD & ~(1 << ADC_CS); // turn off
  digitalWrite(ADC_CS, LOW);
  
  //PORTD = PORTD | (1 << ADC_CS); // turn on
  digitalWrite(ADC_CS, HIGH);

  */

  
}




