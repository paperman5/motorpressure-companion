#pragma once
#include "hardware/spi.h"
#include "hardware/pio.h"

#define SPI_COMPANION spi1
#define SPI_BAUDRATE 200000
#define PIN_SPI_CS 9
#define PIN_SPI_MISO 10
#define PIN_SPI_MOSI 11
#define PIN_SPI_SCK 12
#define BUFFER_SIZE 40
#define DMA_RX_CMD_CHAN 0
#define DMA_RX_MSG_CHAN 1
#define DMA_TX_CHAN 2

#define PIO_SPI_COMPANION pio0
#define PIO_SPI_CS_SM 0
#define PIO_SPI_DATA_SM 1

static void setup_pio_spi_sm(PIO pio, uint cs_sm, uint data_sm, int cs_pin, int sck_pin, int miso_pin, int mosi_pin, irq_handler_t pio_irq_handler);
static void reset_data_sm();
static void setup_pio_spi_dma(PIO pio, uint cs_sm, uint data_sm, uint rx_cmd_chan, uint rx_msg_chan, uint tx_chan, altos_header_t *msg, irq_handler_t dma_irq_handler);
static void setup_messages();
static void pio_spi_start();
static void pio_spi_stop();
void setup_spi();
void spi_handler();
void dma_handler();
inline void print_raw(void* obj, size_t size);
inline void print_binary(int num);