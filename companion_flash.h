#pragma once
#include "companion_types.h"

// NOTE: These addresses should be changed if flash storage locations change.
// Best way to get these is from the memory map file.
#define COMPANION_FLASH_CONFIG_BASE _u(0x101fc000)
#define COMPANION_FLASH_CONFIG_LEN _u(0x00004000)
#define COMPANION_FLASH_STORAGE_BASE _u(0x10200000)
#define COMPANION_FLASH_STORAGE_LEN _u(0x00600000)

#define COMPANION_FILE_SIZE _u(0x00020000) // ~131kB, enough to hold ~30s of data
#define COMPANION_FILE_COUNT (COMPANION_FLASH_STORAGE_LEN / COMPANION_FILE_SIZE)

typedef struct __packed {
    uint8_t byte1;
    uint8_t byte2;
    uint8_t companion_version;
    uint8_t save_version;
    uint16_t sample_rate;
    uint16_t sample_count;
    uint8_t unused[8];
} save_header_t;

typedef struct __packed {
    save_header_t header;
    data_t data[(COMPANION_FILE_SIZE - sizeof(save_header_t))/sizeof(data_t)];
} save_file_t;

typedef struct __packed {
    uint16_t adc_lut[4096];
    // The rest of the storage space is unused for now
    uint8_t unused[COMPANION_FLASH_CONFIG_LEN - 4096*2];
} flash_config_t;

typedef struct __packed {
    save_file_t files[COMPANION_FILE_COUNT];
} flash_storage_t;

#define companion_flash_config ((flash_config_t *)COMPANION_FLASH_CONFIG_BASE)
#define companion_flash_storage ((flash_storage_t *)COMPANION_FLASH_STORAGE_BASE)
#define COMPANION_FILE_MAX_SAMPLES (COMPANION_FILE_SIZE - sizeof(save_header_t))/sizeof(data_t)
static_assert(sizeof(flash_config_t) == COMPANION_FLASH_CONFIG_LEN, "");
static_assert(sizeof(flash_storage_t) == COMPANION_FLASH_STORAGE_LEN, "");