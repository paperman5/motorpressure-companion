#include <stdio.h>
#include <string.h>
#include <bsp/board_api.h>
#include <tusb.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/timer.h"
#include "pio_spi.pio.h"
#include "hardware/regs/usb.h"
#include "hardware/structs/usb.h"

#include "motorpressure_companion.h"
#include "companion_types.h"
#include "companion_flash.h"
#include "companion_disk.h"

volatile altos_header_t message;
altos_setup_t setup_reply;
altos_fetch_t fetch_reply;
save_header_t save_header;

uint pio_offset_cs = 0;
uint pio_offset_data = 0;

data_t sample_buf[2][ADC_BUFFER_COUNT];
volatile uint8_t current_sample_buf = 0;
volatile uint16_t buffer_index = 0;
volatile flash_status_t flash_status = {
    .file_index = 0,
    .file_offset = 0,
    .buffer_save_index = 0,
    .do_flash_saving = false,
    .do_single_buffer_save = false,
    .current_flash_state = FLASH_STATE_READY
};
volatile bool companion_port_initialized = false;
volatile bool reset_pending = false;
volatile bool overrun = false;

int main()
{
    init_fat_filesystem();
    
    // Initialize TinyUSB stack
    board_init();
    tusb_init();
    if (board_init_after_tusb) {
        board_init_after_tusb();
    }
    stdio_init_all();

    // If USB is physically connected (not necessarily enumerated, etc.),
    // wait up to 1s for a serial monitor to connect so we are sure to get messages.
    if (usb_hw->sie_status & USB_SIE_STATUS_VBUS_DETECTED_BITS) {
        uint32_t t1 = board_millis();
        while (!tud_cdc_connected() && (board_millis() - t1) <= 1000) {
            tud_task();
        }
        // At this point it can still miss messages and sleep_ms() doesn't fix it.
        // Only fix seems to be calling tud_task() for a short time instead of sleeping.
        t1 = board_millis();
        while (board_millis() - t1 <= 10) {
            tud_task();
        }
    }

    setup_spi();
    puts("SPI SETUP COMPLETE");
    setup_messages();
    puts("MESSAGE SETUP COMPLETE");

    pio_spi_start();
    puts("SPI STARTED");

    setup_adc();
    puts("ADC STARTED");

#if DEBUG_NUKE_STORAGE
    for (uint8_t i = 0; i < COMPANION_FILE_COUNT; i++) {
        erase_file(i);
    }
#endif

    flash_status.file_index = get_first_unused_file_index();
    if (flash_status.file_index >= COMPANION_FILE_COUNT) {
        puts("Storage full!");
        flash_status.file_index = 0;
    }
    printf("Will save to file %d\n", flash_status.file_index);
    if (file_needs_erased(flash_status.file_index)) {
        printf("Erasing file %d\n", flash_status.file_index);
        erase_file(flash_status.file_index);
        puts("File erased");
    }

    enum FlashState prev_flash_state = FLASH_STATE_READY;
    enum FlightState prev_flight_state = FS_STARTUP;

    while (true) {
        tud_task();
        enum FlashState flash_temp = (enum FlashState)flash_status.current_flash_state;
        enum FlightState flight_temp = (enum FlightState)message.flight_state;

        if (flash_status.do_flash_saving && flash_status.do_single_buffer_save) {
            uint8_t buffer_index = (uint8_t)flash_status.buffer_save_index;
            if (flash_status.file_offset == 0) {
                // Overwrite beginning of buffer with file header
                memcpy((uint8_t *)sample_buf[buffer_index], (uint8_t *)&save_header, sizeof(save_header));
            }
            flash_status.file_offset += save_buffer_to_flash(flash_status.file_index, flash_status.file_offset, buffer_index);
            if (flash_temp == FLASH_STATE_FINALIZING) {
                finalize_file(flash_status.file_index);
                flash_status.current_flash_state = FLASH_STATE_FINALIZED;
            }
            if (flash_temp == FLASH_STATE_RECORDING && flash_status.file_offset >= COMPANION_FILE_SIZE-ADC_BUFFER_SIZE) {
                flash_status.current_flash_state = FLASH_STATE_FINALIZING;
            }
            flash_status.do_single_buffer_save = false;
        }

        if (prev_flight_state != flight_temp) {
            flight_state_change_handler(prev_flight_state, flight_temp);
        }

        if (prev_flash_state != flash_temp) {
            flash_state_change_handler(prev_flash_state, flash_temp);
        }

        if (reset_pending && flash_temp != FLASH_STATE_RECORDING && flash_temp != FLASH_STATE_FINALIZING) {
            software_reset();
        }

        if (overrun) {
            puts("BUFFER OVERRUN");
            overrun = false;
        }

        prev_flash_state = flash_temp;
        prev_flight_state = flight_temp;
    }
}

static void setup_pio_spi_sm(PIO pio, uint cs_sm, uint data_sm, 
                             int cs_pin, int sck_pin, int miso_pin, 
                             int mosi_pin, irq_handler_t pio_irq_handler) 
{
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

    sm_config_set_in_shift(&conf_data, false, false, 16);
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

static void setup_pio_spi_dma(PIO pio, uint cs_sm, uint data_sm, 
                              uint rx_cmd_chan, uint rx_msg_chan, uint tx_chan, 
                              altos_header_t *msg, irq_handler_t dma_irq_handler) 
{
    // 3 DMA channels:
    // 1-Receives first two bytes (command & state), chains immediately to #2 & configures #3
    // 2-Receives the remaining 14 bytes of the header & chains to #3
    // 3-Transmits to the flight computer with the correct message (or nothing if not needed)
    // Note: RX size is set to 16 in order to set the message's uint16s atomically instead of
    //       one byte at a time, so that accessing them is safe at any time.
    dma_claim_mask((1<<rx_cmd_chan) | (1<<rx_msg_chan) | (1<<tx_chan));
    
    dma_channel_config_t config = dma_channel_get_default_config(rx_cmd_chan);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
    channel_config_set_dreq(&config, pio_get_dreq(pio, data_sm, false));
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, false);
    channel_config_set_bswap(&config, true);
    channel_config_set_chain_to(&config, rx_msg_chan);
    dma_channel_configure(rx_cmd_chan, &config, (uint16_t *)msg, 
                          &pio->rxf[data_sm], dma_encode_transfer_count(1), false);

    config = dma_channel_get_default_config(rx_msg_chan);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
    channel_config_set_dreq(&config, pio_get_dreq(pio, data_sm, false));
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_bswap(&config, true);
    channel_config_set_chain_to(&config, tx_chan);
    dma_channel_configure(rx_msg_chan, &config, ((uint16_t *)msg) + 1, 
                          &pio->rxf[data_sm], dma_encode_transfer_count(sizeof(altos_header_t)/sizeof(uint16_t) - 1), false);

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
    memset((altos_header_t *)&message, 0, sizeof(message));
    memset(&setup_reply, 0, sizeof(setup_reply));
    memset(&fetch_reply, 0, sizeof(fetch_reply));

    setup_reply.board_id = (uint16_t)COMPANION_ID;
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
    setup_pio_spi_dma(PIO_SPI_COMPANION, PIO_SPI_CS_SM, PIO_SPI_DATA_SM, DMA_RX_CMD_CHAN, DMA_RX_MSG_CHAN, DMA_TX_CHAN, (altos_header_t *)&message, dma_spi_handler);
    
    bi_decl(bi_4pins_with_func(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_CS, GPIO_FUNC_SPI));
}

void setup_adc() {
    memset(sample_buf, 0, sizeof(sample_buf));

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
        pio_interrupt_clear(PIO_SPI_COMPANION, 0);
        dma_channel_set_write_addr(DMA_RX_MSG_CHAN, ((uint16_t *)&message) + 1, false);
        dma_channel_set_write_addr(DMA_RX_CMD_CHAN, (uint16_t *)&message, true);
    }
    if (pio_interrupt_get(PIO_SPI_COMPANION, 1)) {
        // CS HIGH
        pio_interrupt_clear(PIO_SPI_COMPANION, 1);
        reset_data_sm();
        if (!companion_port_initialized && message.command >= CMD_SETUP && message.command <= CMD_NOTIFY) {
            printf("Companion message received, serial %d flight %d\n", message.serial, message.flight);
            save_header_init();
            companion_port_initialized = true;
        }
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
    // Retrieve the sniffer value and get DMA going again as quickly as possible,
    // since the ADC is free-running and we don't want to miss any samples
    uint32_t adc_temp = dma_sniffer_get_data_accumulator();
    dma_sniffer_set_data_accumulator(0);
    dma_channel_start(DMA_ADC_CHAN);

    // Scale the oversampled value to fit in a uint16
    data_t sample = {
        .tick = message.tick,
        .value = (uint16_t)(adc_temp*ADC_OVERSAMPLE_SCALE_FACTOR)
    };
    sample_buf[current_sample_buf][buffer_index++] = sample;
    buffer_index %= ADC_BUFFER_COUNT;
    if (buffer_index == 0) {
        // Hand off the save routine to the main loop so interrupts can continue
        flash_status.buffer_save_index = current_sample_buf;
        current_sample_buf ^= 1;
        if (flash_status.current_flash_state == FLASH_STATE_RECORDING || flash_status.current_flash_state == FLASH_STATE_FINALIZING) {
            if (flash_status.do_single_buffer_save) {
                overrun = true;
            }
            flash_status.do_single_buffer_save = true;
        }
    }
    dma_channel_acknowledge_irq1(DMA_ADC_CHAN);
}

inline void save_header_init() {
    memset(&save_header, 0, sizeof(save_header));
    save_header.byte1 = COMPANION_FILE_HEADER_MAGIC1;
    save_header.byte2 = COMPANION_FILE_HEADER_MAGIC2;
    save_header.companion_version = COMPANION_FILE_HEADER_COMPANION_VERSION;
    save_header.save_version = COMPANION_FILE_HEADER_SAVE_VERSION;
    save_header.sample_rate = COMPANION_FILE_HEADER_SAMPLE_RATE;
    save_header.sample_count = 0xFFFF;
    save_header.flcomp_serial = message.serial;
    save_header.flcomp_flight = message.flight;
}

uint16_t save_buffer_to_flash(uint8_t file_idx, uint32_t file_offset, uint8_t buffer_idx) {
    if (!companion_port_initialized || file_idx >= COMPANION_FILE_COUNT || file_offset + ADC_BUFFER_SIZE > COMPANION_FILE_SIZE || buffer_idx > 1)
        return 0;
    uint32_t file_addr = (uint32_t)&companion_flash_storage->files[file_idx];
    // printf("%d\n", file_offset);
    flash_range_program(file_addr + file_offset - XIP_BASE, (uint8_t *)sample_buf[buffer_idx], ADC_BUFFER_SIZE);
    return ADC_BUFFER_SIZE;
}

void erase_file(uint8_t index) {
    if (index >= COMPANION_FILE_COUNT)
        return;
    flash_range_erase((uint32_t)&(companion_flash_storage->files[index]) - XIP_BASE, COMPANION_FILE_SIZE);
}

uint8_t get_first_unused_file_index() {
    for (uint8_t i = 0; i < COMPANION_FILE_COUNT; i++) {
        save_file_t *file = &companion_flash_storage->files[i];
        if (file->header.byte1 != COMPANION_FILE_HEADER_MAGIC1 || file->header.byte2 != COMPANION_FILE_HEADER_MAGIC2) {
            return i;
        }
    }
    return COMPANION_FILE_COUNT;
}

bool file_needs_erased(uint8_t file_index) {
    // Check if the first 512 bytes are 0xFF, we will assume it has all been erased if true
    for (uint16_t i = 0; i < 512; i++) {
        if (*((uint8_t *)&companion_flash_storage->files[file_index] + i) != 0xFF)
            return true;
    }
    return false;
}

inline void finalize_file(uint8_t file_index) {
    // Re-save the first 256 bytes of data with the updated sample count in the header
    uint32_t file_addr = (uint32_t)&companion_flash_storage->files[file_index];
    uint8_t temp_buf[FLASH_PAGE_SIZE];
    uint16_t n_samples = (uint16_t)((flash_status.file_offset - sizeof(save_header_t))/sizeof(data_t));
    memcpy(temp_buf, &companion_flash_storage->files[file_index], FLASH_PAGE_SIZE);
    memcpy(&temp_buf[offsetof(save_header_t, sample_count)], &n_samples, sizeof(n_samples));
    flash_range_program(file_addr - XIP_BASE, temp_buf, FLASH_PAGE_SIZE);
}

void flight_state_change_handler(enum FlightState prev_state, enum FlightState new_state) {
    // printf("Flight: %d -> %d\n", prev_state, new_state);
    switch (new_state) {
        case FS_STARTUP:
            // Flight computer has rebooted so we should too
            if (prev_state != FS_STARTUP) {
                reset_pending = true;
            }
            break;
        case FS_IDLE:
            // Flight computer is flat on the desk not on a rocket
        case FS_PAD: // (intentional fallthrough)
            // Flight computer is ready to launch
            flash_status.current_flash_state = FLASH_STATE_READY;
            break;
        case FS_BOOST:
            // Rocket has launched
            if (prev_state == FS_PAD && flash_status.current_flash_state == FLASH_STATE_READY) {
                flash_status.current_flash_state = FLASH_STATE_RECORDING;
            }
            break;
        case FS_FAST:
        case FS_COAST:
        case FS_DROGUE:
        case FS_MAIN:
        case FS_LANDED:
            // Rocket motor has burned out
            if (flash_status.current_flash_state == FLASH_STATE_RECORDING) {
                flash_status.current_flash_state = FLASH_STATE_FINALIZING;
            }
            break;
        default:
            break;
    }
}

void flash_state_change_handler(enum FlashState prev_state, enum FlashState new_state) {
    if (new_state != prev_state + 1) {
        printf("Invalid flash state transition: %d -> %d\n", prev_state, new_state);
        return;
    }
    // printf("Flash: %d -> %d\n", prev_state, new_state);
    switch (new_state) {
        case FLASH_STATE_RECORDING:
            flash_status.do_flash_saving = true;
            break;
        case FLASH_STATE_FINALIZING:
            break;
        case FLASH_STATE_FINALIZED:
            flash_status.do_flash_saving = false;
            break;
        default:
            break;
    }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {

}

// Invoked when device is unmounted
void tud_umount_cb(void) {

}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {

}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {

}


//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
void cdc_task(void) {
    // connected and there are data available
    if (tud_cdc_available()) {
        // read data
        char     buf[CFG_TUD_CDC_RX_BUFSIZE];
        uint32_t count = tud_cdc_read(buf, sizeof(buf));
        (void)count;

        // Echo back
        // Note: Skip echo by commenting out write() and write_flush()
        // for throughput test e.g
        //    $ dd if=/dev/zero of=/dev/ttyACM0 count=10000
        // tud_cdc_write(buf, count);
        // tud_cdc_write_flush();
    }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {

}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {

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

inline void software_reset() {
    // Set the reset bit in the Application Interrupt and Reset Control Register (AIRCR)
    // Datasheet section 2.4
    *((volatile uint32_t*)(PPB_BASE + 0x0ED0C)) = 0x5FA0004;
}