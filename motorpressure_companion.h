#include "hardware/spi.h"

#define SPI_COMPANION spi1
#define SPI_BAUDRATE 200000
#define PIN_SPI_CS 9
#define PIN_SPI_SCK 10
#define PIN_SPI_MISO 11
#define PIN_SPI_MOSI 12


void setup_spi();
void companion_handler();