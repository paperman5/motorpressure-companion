// Testing SPI sketch for Raspberry Pi Pico to Adafruit Feather M0 Adalogger
// BUILD FOR PI PICO

#include <SPI.h>

// Wiring:
// Pico MISO  GP0 <-> Feather PB11 (Labelled SCK)
// Pico CS    GP1 <-> Feather PB10 (Labelled MOSI)
// Pico CK    GP2 <-> Feather PB09 (Labelled A2)
// Pico MOSI  GP3 <-> Feather PA12 (Labelled MISO)

SPISettings spisettings(100, MSBFIRST, SPI_MODE0);

// Core 0 will be SPI master
void setup() {
  SPI.setRX(0);
  SPI.setCS(1);
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.begin(true);

  delay(5000);
}

int transmits = 0;
void loop() {
  char msg[32];
  for (char i = 0; i < sizeof(msg); i++) {
    msg[i] = i + 65;
  }
  Serial.printf("\n\nM-SEND: '%s'\n", msg);
  SPI.beginTransaction(spisettings);
  SPI.transfer(msg, sizeof(msg));
  SPI.endTransaction();
  Serial.printf("M-RECV: '%s'\n", msg);
  transmits++;
  delay(5000);
}
