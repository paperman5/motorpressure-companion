#include <stdio.h>
#include "pico/stdlib.h"
#include "companion_types.h"
#include "hardware/spi.h"
#include "hardware/irq.h"
#include "hardware/structs/spi.h"
#include "pico/binary_info.h"
#include "motorpressure_companion.h"
#include "pico/time.h"

enum Command {
    CMD_SETUP = 1,
    CMD_FETCH = 2,
    CMD_NOTIFY = 3
};
enum FlightState { // From firmware kernel source `ao_flight.h`
    FS_STARTUP = 0,
    FS_IDLE = 1,
    FS_PAD = 2,
    // PAD->BOOST: Baro >20m vert OR accel >2g && vel > 5m/s
    FS_BOOST = 3,
    // BOOST->FAST: accel < -1/4g OR boost time > 15s
    FS_FAST = 4,
    FS_COAST = 5,
    FS_DROGUE = 6,
    FS_MAIN = 7,
    FS_LANDED = 8,
    FS_INVALID = 9,
    FS_TEST = 10
};
enum ProgramState {
    PS_DISCONNECTED = 0,
    PS_MONITORING = 1,
    PS_RECORDING = 2,
    PS_DONE = 3,
    PS_ERROR = -1
};



int main()
{
    stdio_init_all();
    sleep_ms(10000);
    setup_spi();

    while (true) {
        // printf("Hello, world!\n");
        // sleep_ms(1000);
    }
}



void setup_spi() {
    spi_init(SPI_COMPANION, SPI_BAUDRATE);
    spi_set_format(SPI_COMPANION, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    spi_set_slave(SPI_COMPANION, true);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_CS, GPIO_FUNC_SPI);
    gpio_set_pulls(PIN_SPI_SCK, false, false);
    gpio_set_pulls(PIN_SPI_MOSI, false, false);
    gpio_set_pulls(PIN_SPI_MISO, false, false);
    gpio_set_pulls(PIN_SPI_CS, false, false);
    gpio_set_slew_rate(PIN_SPI_SCK, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_SPI_MOSI, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_SPI_MISO, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PIN_SPI_CS, GPIO_SLEW_RATE_FAST);
    gpio_set_drive_strength(PIN_SPI_SCK, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_SPI_MOSI, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(PIN_SPI_MISO, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_drive_strength(PIN_SPI_CS, GPIO_DRIVE_STRENGTH_2MA);
    const uint spi_irq = SPI_NUM(SPI_COMPANION) ? SPI1_IRQ : SPI0_IRQ;
    // irq_set_exclusive_handler(spi_irq, companion_handler);
    // spi_get_hw(SPI_COMPANION)->imsc = 1 << 3 | 1 << 2; // TX & RX interrupts enabled
    // irq_set_enabled(spi_irq, true);
    bi_decl(bi_4pins_with_func(PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK, PIN_SPI_CS, GPIO_FUNC_SPI));

    printf("%d\n", gpio_get_drive_strength(PIN_SPI_SCK));
    printf("%d\n", gpio_get_drive_strength(PIN_SPI_MOSI));
    printf("%d\n", gpio_get_drive_strength(PIN_SPI_MISO));
    printf("%d\n", gpio_get_drive_strength(PIN_SPI_CS));
    printf("%d\n", gpio_get_slew_rate(PIN_SPI_SCK));
    printf("%d\n", gpio_get_slew_rate(PIN_SPI_MOSI));
    printf("%d\n", gpio_get_slew_rate(PIN_SPI_MISO));
    printf("%d\n", gpio_get_slew_rate(PIN_SPI_CS));
}

void companion_handler() {
    // printf("%d\n", spi_get_hw(SPI_COMPANION)->dr);
}