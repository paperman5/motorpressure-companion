// Testing SPI sketch for Raspberry Pi Pico to Adafruit Feather M0 Adalogger
// BUILD FOR FEATHER M0

#include <SercomSPISlave.h>
Sercom4SPISlave SPISlave; // to use a different SERCOM, change this line and find and replace all SERCOM4 with the SERCOM of your choice

// #define DEBUG // uncomment this line to print debug data to the serial bus
// #define INTERRUPT2BUFFER // uncomment this line to copy the data received in the Data Received Complete interrupt to a buffer to be used in the main loop
// #define INTERRUPT2SERIAL // uncomment this line to print the data to the serial bus whenever the Data Received Complete interrupt is triggered

// Wiring:
// Pico MISO  GP0 <-> Feather PB11 (Labelled SCK)
// Pico CS    GP1 <-> Feather PB10 (Labelled MOSI)
// Pico CK    GP2 <-> Feather PB09 (Labelled A2)
// Pico MOSI  GP3 <-> Feather PA12 (Labelled MISO)

volatile uint8_t txIndex = 0;
volatile uint8_t rxIndex = 0;
volatile uint8_t rxBuf[32];

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("Serial started");

  SPISlave.SercomInit(SPISlave.MOSI_Pins::PA12, SPISlave.SCK_Pins::PB09, SPISlave.SS_Pins::PB10, SPISlave.MISO_Pins::PB11);
  Serial.println("SERCOM4 SPI slave initialized");
  Serial.printf("0x%x\n", SERCOM4->SPI.CTRLA.reg);
  Serial.printf("0x%x\n", SERCOM4->SPI.CTRLB.reg);
}

void loop() {
  if (rxIndex == sizeof(rxBuf)) {
    Serial.println("Message Received");
    for (int i = 0; i < sizeof(rxBuf); i++) {
      Serial.printf("%02X ", rxBuf[i]);
      if (i % 8 == 0 && i > 0) {
        Serial.println("");
      }
    }
    Serial.println("");
    rxIndex = 0;
  }
}

void SERCOM4_Handler()
/*
Reference: Atmel-42181G-SAM-D21_Datasheet section 26.8.6 on page 503
*/
{
  #ifdef DEBUG
    Serial.println("In SPI Interrupt");
  #endif
  // uint8_t data = 0;
  // data = (uint8_t)SERCOM4->SPI.DATA.reg;
  uint8_t interrupts = SERCOM4->SPI.INTFLAG.reg; // Read SPI interrupt register
  #ifdef DEBUG
    Serial.print("Interrupt: "); Serial.println(interrupts);
  #endif
  
  // Slave Select Low interrupt
  if (interrupts & (1 << 3)) // 1000 = bit 3 = SSL // page 503
  {
  #ifdef DEBUG
    Serial.println("SPI Slave Select Low interupt");
  #endif
    SERCOM4->SPI.INTFLAG.bit.SSL = 1; // Clear Slave Select Low interrupt
  }
  
  // Data Received Complete interrupt: this is where the data is received, which is used in the main loop
  if (interrupts & (1 << 2)) // 0100 = bit 2 = RXC // page 503
  {
    #ifdef DEBUG
      Serial.println("SPI Data Received Complete interrupt");
    #endif
    uint8_t data = SERCOM4->SPI.DATA.reg; // Read data register. This clears the RXC interrupt flag!
    // Serial.println(rxIndex);
    rxBuf[rxIndex++] = data;
  }
  
  // Data Transmit Complete interrupt
  if (interrupts & (1 << 1)) // 0010 = bit 1 = TXC // page 503
  {
  #ifdef DEBUG
    Serial.println("SPI Data Transmit Complete interrupt");
  #endif
    SERCOM4->SPI.INTFLAG.bit.TXC = 1; // Clear Transmit Complete interrupt
  }
  
  // Data Register Empty interrupt
  if (interrupts & (1 << 0)) // 0001 = bit 0 = DRE // page 503
  {
    #ifdef DEBUG
      Serial.println("SPI Data Register Empty interrupt");
    #endif
    // Serial.println(txIndex);
    SERCOM4->SPI.DATA.reg = txIndex++; // Writing to the DATA register clears the DRE interrupt flag
  }
  
  #ifdef INTERRUPT2SERIAL
    // Print data received during the Data Receive Complete interrupt
    char _data = data;
    Serial.print("DATA: ");
    Serial.println(_data); // Print received data
  #endif

}