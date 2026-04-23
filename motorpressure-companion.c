#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "companion_types.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/structs/spi.h"
#include "pico/binary_info.h"
#include "motorpressure_companion.h"
#include "pio_spi.pio.h"
#include "pico/time.h"
#include "pio_spi.pio.h"
#include "hardware/adc.h"


altos_header_t message;
altos_setup_t setup_reply;
altos_fetch_t fetch_reply;

uint pio_offset_cs = 0;
uint pio_offset_data = 0;

uint16_t adc_buf[128];
volatile uint16_t adc_idx = 0;

int main()
{
    stdio_init_all();
    sleep_ms(1000);
    setup_spi();
    puts("SPI SETUP COMPLETE");
    setup_messages();
    puts("MESSAGE SETUP COMPLETE");

    pio_spi_start();
    puts("SPI STARTED");

    setup_adc();
    puts("ADC STARTED");

    while (true) {
        tight_loop_contents();
    }
}

static void setup_pio_spi_sm(PIO pio, uint cs_sm, uint data_sm, int cs_pin, int sck_pin, int miso_pin, int mosi_pin, irq_handler_t pio_irq_handler) {
    pio_sm_claim(pio, cs_sm);
    pio_sm_claim(pio, data_sm);
    pio_offset_cs = pio_add_program(pio, &spi_cs_program);
    pio_offset_data = pio_add_program(pio, &spi_data_program);
    
    pio_sm_config conf_cs = spi_cs_program_get_default_config(pio_offset_cs);
    pio_sm_config conf_data = spi_data_program_get_default_config(pio_offset_data);

    sm_config_set_in_pins(&conf_cs, cs_pin);
    
    // SCK pin must be directly after MOSI pin, because all input pins must be
    // consecutive and data is read starting from the first pin.
    sm_config_set_in_pins(&conf_data, mosi_pin);
    sm_config_set_out_pins(&conf_data, miso_pin, 1);

    sm_config_set_in_shift(&conf_data, false, false, 8);
    sm_config_set_out_shift(&conf_data, false, false, 8);
    sm_config_set_out_special(&conf_data, true, false, 0);

    pio_sm_set_pindirs_with_mask(pio, cs_sm, 0<<cs_pin, 1<<cs_pin);
    pio_sm_set_pindirs_with_mask(pio, data_sm, (0<<sck_pin)|(0<<mosi_pin)|(1<<miso_pin), 
                                               (1<<sck_pin)|(1<<mosi_pin)|(1<<miso_pin));

    pio_gpio_init(pio, cs_pin);
    pio_gpio_init(pio, sck_pin);
    pio_gpio_init(pio, miso_pin);
    pio_gpio_init(pio, mosi_pin);
    // Give CS & SCK a pullup so they don't cause spurious events while the flight computer is booting.
    gpio_set_pulls(cs_pin, true, false);
    gpio_set_pulls(sck_pin, true, false);
    gpio_set_pulls(miso_pin, false, true);
    gpio_set_pulls(mosi_pin, false, false);
    pio_set_irq0_source_mask_enabled(pio, (1<<pis_interrupt0) | (1<<pis_interrupt1), true);
    irq_set_exclusive_handler(pio_get_irq_num(pio, 0), pio_irq_handler);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);

    pio_sm_init(pio, cs_sm, pio_offset_cs, &conf_cs);
    pio_sm_init(pio, data_sm, pio_offset_data, &conf_data);
}

static inline void reset_data_sm() {
    pio_sm_exec_wait_blocking(PIO_SPI_COMPANION, PIO_SPI_DATA_SM, pio_encode_jmp(pio_offset_data + spi_data_offset_reset));
    // While transmitting a message to the flight computer, the PIO state machine will continue to fill the RX FIFO.
    // When the next message is received this RX FIFO with irrelevant data will be transferred into memory via DMA.
    // Therefore we want to clear the FIFOs before that happens.
    pio_sm_clear_fifos(PIO_SPI_COMPANION, PIO_SPI_DATA_SM);
}

static void setup_pio_spi_dma(PIO pio, uint cs_sm, uint data_sm, uint rx_cmd_chan, uint rx_msg_chan, uint tx_chan, altos_header_t *msg, irq_handler_t dma_irq_handler) {
    // 3 DMA channels:
    // 1-Receives first header byte (command), chains immediately to #2 & configures #3
    // 2-Receives the remaining 15 bytes of the header & chains to #3
    // 3-Transmits to the flight computer with the correct message (or nothing if not needed)
    dma_claim_mask((1<<rx_cmd_chan) | (1<<rx_msg_chan) | (1<<tx_chan));
    
    dma_channel_config_t config = dma_channel_get_default_config(rx_cmd_chan);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_dreq(&config, pio_get_dreq(pio, data_sm, false));
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, false);
    channel_config_set_chain_to(&config, rx_msg_chan);
    dma_channel_configure(rx_cmd_chan, &config, (uint8_t *)msg, 
                          &pio->rxf[data_sm], dma_encode_transfer_count(1), false);

    config = dma_channel_get_default_config(rx_msg_chan);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_dreq(&config, pio_get_dreq(pio, data_sm, false));
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_chain_to(&config, tx_chan);
    dma_channel_configure(rx_msg_chan, &config, ((uint8_t *)msg) + 1, 
                          &pio->rxf[data_sm], dma_encode_transfer_count(sizeof(message)-1), false);

    config = dma_channel_get_default_config(tx_chan);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
    channel_config_set_dreq(&config, pio_get_dreq(pio, data_sm, true));
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    dma_channel_configure(tx_chan, &config, &pio->txf[data_sm], NULL, 0, false);

    dma_set_irq0_channel_mask_enabled((1<<rx_cmd_chan), true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

static void setup_messages() {
    memset(&message, 0, sizeof(message));
    memset(&setup_reply, 0, sizeof(setup_reply));
    memset(&fetch_reply, 0, sizeof(fetch_reply));

    setup_reply.board_id = (uint16_t)7;
    setup_reply.board_id_inverse = (uint16_t)~(setup_reply.board_id);
    setup_reply.channels = count_of(fetch_reply.data);
    setup_reply.update_period = COMPANION_FETCH_RATE;

    for (int i = 0; i < count_of(fetch_reply.data); i++) {
        fetch_reply.data[i] = i;
    }
}

static void pio_spi_start() {
    pio_set_sm_mask_enabled(PIO_SPI_COMPANION, (1<<PIO_SPI_CS_SM)|(1<<PIO_SPI_DATA_SM), true);
}

static void pio_spi_stop() {

}

void setup_spi() {
    setup_pio_spi_sm(PIO_SPI_COMPANION, PIO_SPI_CS_SM, PIO_SPI_DATA_SM, PIN_SPI_CS, PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, spi_handler);
    setup_pio_spi_dma(PIO_SPI_COMPANION, PIO_SPI_CS_SM, PIO_SPI_DATA_SM, DMA_RX_CMD_CHAN, DMA_RX_MSG_CHAN, DMA_TX_CHAN, &message, dma_spi_handler);
    
    bi_decl(bi_4pins_with_func(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_CS, GPIO_FUNC_SPI));
}

void setup_adc() {
    memset(adc_buf, 0, sizeof(adc_buf));
    adc_gpio_init(ADC_BASE_PIN + ADC_CHAN);
    adc_init();
    adc_select_input(ADC_CHAN);
    adc_set_clkdiv(0.0f);
    adc_fifo_setup(true,    // Enable FIFO
                   true,    // Enable DRAM DREQ
                   1,       // DREQ size of 1 (Datasheet 4.9.2.5 pg. 561)
                   false,   // No error flag
                   false);  // No 12b->8b byte shift

    setup_adc_dma();
    dma_channel_start(DMA_ADC_CHAN);
    
    // Let the ADC run!
    adc_run(true);
}

void setup_adc_dma() {
    // DMA to read the samples coming in from the ADC. Max ADC rate is 500ksps,
    // and since we are aiming for 1ksps we can oversample 500 times. (+ ~4.5 ENOB)
    // To achieve this we can (ab)use the DMA sniffer accumulation register usually
    // used for CRC checks to add up the 500 samples, then retrieve the accumulated
    // value from this register. Individual samples are DMA'd to NULL.
    dma_channel_claim(DMA_ADC_CHAN);
    dma_channel_config_t config = dma_channel_get_default_config(DMA_ADC_CHAN);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
    channel_config_set_dreq(&config, DREQ_ADC);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, false);
    channel_config_set_sniff_enable(&config, true);
    dma_sniffer_set_data_accumulator(0);
    // Set the sniffer to add each DMA'd value to the accumulation register
    dma_sniffer_enable(DMA_ADC_CHAN, DMA_SNIFF_CTRL_CALC_VALUE_SUM, true);
    dma_channel_configure(DMA_ADC_CHAN, &config, NULL, &adc_hw->fifo, 
                          dma_encode_transfer_count(ADC_OVERSAMPLE_COUNT), false);
    
    dma_set_irq1_channel_mask_enabled((1<<DMA_ADC_CHAN), true);
    irq_set_exclusive_handler(DMA_IRQ_1, dma_adc_handler);
    irq_set_enabled(DMA_IRQ_1, true);
}

void spi_handler() {
    if (pio_interrupt_get(PIO_SPI_COMPANION, 0)) {
        // CS LOW
        // puts("CS LOW");
        pio_interrupt_clear(PIO_SPI_COMPANION, 0);
        dma_channel_set_write_addr(DMA_RX_MSG_CHAN, ((uint8_t *)&message) + 1, false);
        dma_channel_set_write_addr(DMA_RX_CMD_CHAN, (uint8_t *)&message, true);
    }
    if (pio_interrupt_get(PIO_SPI_COMPANION, 1)) {
        // CS HIGH
        pio_interrupt_clear(PIO_SPI_COMPANION, 1);
        reset_data_sm();
        // print_raw(&message, sizeof(message));
    }
}

void dma_spi_handler() {
    if (dma_channel_get_irq0_status(DMA_RX_CMD_CHAN)) {
        dma_channel_acknowledge_irq0(DMA_RX_CMD_CHAN);
        // If there is an in-progress TX operation, abort it
        if (dma_channel_is_busy(DMA_TX_CHAN)) {
            dma_hw->abort = 1<<DMA_TX_CHAN;
        }
        // Adjust the address/transfers for the TX operation based on the command received
        switch (message.command) {
            case CMD_SETUP:
                dma_channel_set_read_addr(DMA_TX_CHAN, &setup_reply, false);
                dma_channel_set_transfer_count(DMA_TX_CHAN, dma_encode_transfer_count(sizeof(setup_reply)), false);
                break;
            case CMD_FETCH:
                dma_channel_set_read_addr(DMA_TX_CHAN, &(fetch_reply.data), false);
                dma_channel_set_transfer_count(DMA_TX_CHAN, dma_encode_transfer_count(sizeof(fetch_reply.data)), false);
                break;
            default:
                dma_channel_set_read_addr(DMA_TX_CHAN, NULL, false);
                dma_channel_set_transfer_count(DMA_TX_CHAN, 0, false);
                break;
        }
    }
}

void dma_adc_handler() {
    // Retrieve the sniffer value and get DMA going again as quickly as possible
    uint32_t adc_temp = dma_sniffer_get_data_accumulator();
    dma_sniffer_set_data_accumulator(0);
    dma_channel_start(DMA_ADC_CHAN);

    // Scale the oversampled value to fit in a uint16
    adc_buf[adc_idx++] = (uint16_t)(adc_temp*ADC_OVERSAMPLE_SCALE_FACTOR);
    adc_idx %= count_of(adc_buf);
    dma_channel_acknowledge_irq1(DMA_ADC_CHAN);
}

inline void print_raw(void* obj, size_t size) {
    uint8_t* buf = (uint8_t *)(obj);
    for (uint16_t i = 0; i < size; i++) {
        printf("%02X%s", *(buf + i), (i + 1) % 8 == 0 ? "\n" : " ");
    }
    printf("\n");
}

inline void print_binary(int num) {
    for (int i = sizeof(int) * 8 - 1; i >= 0; i--) {
        printf("%d", (num>>i)&1);
    }
    printf("\n");
}