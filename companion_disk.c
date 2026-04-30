/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "bsp/board_api.h"
#include "tusb.h"
#include "companion_disk.h"
#include "companion_flash.h"

#include "motorpressure_companion.h"

#if CFG_TUD_MSC

// whether host does safe-eject
static bool ejected = false;

// Some MCU doesn't have enough 8KB SRAM to store the whole disk
// We will use Flash as read-only disk with board that has
// CFG_EXAMPLE_MSC_READONLY defined

#define README_CONTENTS \
"This is tinyusb's MassStorage Class demo.\r\n\r\n\
If you find any bugs or get any questions, feel free to file an\r\n\
issue at github.com/hathach/tinyusb"

#define BPB_JMP 0xEB, 0x3C, 0x90
#define BPB_OEMNAME 'M', 'S', 'D', 'O', 'S', '5', '.', '0'
#define BPB_BYTES_PER_SECTOR 0x00, 0x02             // 512 bytes per sector
#define BPB_SECTORS_PER_CLUSTER 0x40                // 64 sectors per cluster
#define BPB_RESERVED_SECTOR_COUNT 0x01, 0x00        // 1 reserved sector before FAT
#define BPB_FAT_COUNT 0x01                          // 1x FAT
#define BPB_ROOT_ENTRY_COUNT 0x40, 0x00             // Max of 64 root directory entries
#define BPB_TOTAL_SECTORS_16 0x06, 0x30             // 12294 total sectors (6MiB/512Byte + 6 RAM sectors)
#define BPB_MEDIA_TYPE 0xF8                         // Standard removable media type
#define BPB_SECTORS_PER_FAT 0x01, 0x00              // 1 sector per FAT
#define BPB_SECTORS_PER_TRACK 0x01, 0x00            // 1 sector per track (not relevant for RAM disk)
#define BPB_NUM_HEADS 0x01, 0x00                    // 1 head (not relevant for RAM disk)
#define BPB_HIDDEN_SECTORS 0x00, 0x00, 0x00, 0x00   // 0 hidden sectors
#define BPB_TOTAL_SECTORS_32 0x00, 0x00, 0x00, 0x00 // Total sectors determined by BPB_TOTAL_SECTORS_16
#define BPB_DRIVE_NUM 0x80
#define BPB_RESERVED 0x00
#define BPB_BOOT_SIG 0x29
#define BPB_VOL_ID 0x07, 0xD0, 0xA6, 0xF1           // Serial number 0xF1A6D007
#define BPB_VOL_LABEL 'M', 'P', 'C', 'o', 'm', 'p', 'a', 'n', 'i', 'o', 'n'
#define BPB_FS_TYPE 'F', 'A', 'T', '1', '2', ' ', ' ', ' '
#define BPB_MAGIC 0x55, 0xAA
#define FAT_FILE_DATE 0x21, 0x5C // 0x5C21 -> Jan 1st 2026
#define FAT_FILE_TIME 0x00, 0x08 // 01:00:00
#define FAT_FILE_ATTR 0x20       // ATTR_ARCHIVE

#define DISK_SECTOR_SIZE 512
#define DISK_SECTORS_PER_CLUSTER 64
#define DISK_CLUSTER_SIZE (DISK_SECTOR_SIZE * DISK_SECTORS_PER_CLUSTER)
#define DISK_RAM_SECTORS 6 // 1x Boot, 1x FAT, 4x RootDir (64entries*32bytes/512bytes)
#define DISK_SECTOR_NUM (12288 + DISK_RAM_SECTORS)
#define FILE_CLUSTER_COUNT (COMPANION_FILE_SIZE / DISK_CLUSTER_SIZE)

// Sector 0 is boot sector
// Sector 1 is FAT cluster allocation table
// Sectors 2-5 are the root directory (64 entries * 32 bytes / 512 bytes = 4 sectors)
static uint8_t ram_disk[DISK_RAM_SECTORS][DISK_SECTOR_SIZE] = {{0x00}};

void init_fat_filesystem() {
  // Setup boot sector
  memcpy(ram_disk[0], (uint8_t[])
  {
    BPB_JMP, BPB_OEMNAME, BPB_BYTES_PER_SECTOR, BPB_SECTORS_PER_CLUSTER, BPB_RESERVED_SECTOR_COUNT,
    BPB_FAT_COUNT, BPB_ROOT_ENTRY_COUNT, BPB_TOTAL_SECTORS_16, BPB_MEDIA_TYPE, BPB_SECTORS_PER_FAT,
    BPB_SECTORS_PER_TRACK, BPB_NUM_HEADS, BPB_HIDDEN_SECTORS, BPB_TOTAL_SECTORS_32, BPB_DRIVE_NUM,
    BPB_RESERVED, BPB_BOOT_SIG, BPB_VOL_ID, BPB_VOL_LABEL, BPB_FS_TYPE
  }, 62);
  memcpy(&ram_disk[0][510], (uint8_t[])
  {
    BPB_MAGIC
  }, 2);

  // Required first 3 bytes of FAT12 table
  memcpy(ram_disk[1], (uint8_t[])
  {
    BPB_MEDIA_TYPE, 0xFF, 0xFF
  }, 3);
  
  // First entry in root directory is volume label
  memcpy(ram_disk[2], (uint8_t[])
  { // 0x28 is ATTR_VOLUME_ID | ATTR_ARCHIVE
    BPB_VOL_LABEL, 0x08, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, FAT_FILE_TIME, FAT_FILE_DATE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  }, 32);

  // Enumerate the files in flash
  for (uint8_t i = 0; i < COMPANION_FILE_COUNT; i++) {
    save_file_t *file = &(companion_flash_storage->files[i]);
    if (file->header.byte1 != 0xAB || file->header.byte2 != 0xCD)
      break;
    
    uint32_t file_size = file->header.sample_count * sizeof(data_t) + sizeof(save_header_t);
    uint16_t first_cluster = i * FILE_CLUSTER_COUNT + 2;
    uint8_t cluster_count = (file_size + DISK_CLUSTER_SIZE - 1) / DISK_CLUSTER_SIZE;
    
    // Construct the cluster list for this file in the FAT12 table
    for (uint8_t j = 0; j < cluster_count; j++) {
      uint16_t bi = 3 + (i * FILE_CLUSTER_COUNT * 3 / 2) + (j * 3 / 2);
      uint16_t next_cluster = (j == cluster_count - 1) ? 0xFFF : first_cluster + j + 1;
      if (bi % 2) {
        ram_disk[1][bi]   |= next_cluster & 0xFF;
        ram_disk[1][bi+1] |= (next_cluster >> 8) & 0x0F;
      }
      else {
        ram_disk[1][bi]   |= (next_cluster << 4) & 0xF0;
        ram_disk[1][bi+1] |= (next_cluster >> 4) & 0xFF;
      }
    }
    
    // Construct the directory entry for this file in the root directory
    uint8_t dir_entry[32] = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, FAT_FILE_ATTR, 0x00, 0x00, FAT_FILE_TIME,
      FAT_FILE_DATE, FAT_FILE_DATE, 0x00, 0x00, FAT_FILE_TIME, FAT_FILE_DATE, first_cluster&0xFF, (first_cluster>>8)&0xFF, 
      file_size&0xFF, (file_size>>8)&0xFF, (file_size>>16)&0xFF, (file_size>>24)&0xFF
    };
    sprintf(dir_entry, "%05d%03dBIN", file->header.flcomp_serial, file->header.flcomp_flight);
    dir_entry[11] = FAT_FILE_ATTR; //sprintf writes a null terminator here so re-write it
    memcpy(ram_disk[2] + (i+1)*32, dir_entry, 32);
  }
}

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
  (void) lun;
  const char vid[] = "MPComp";
  const char pid[] = "Companion";
  const char rev[] = "2.0";

  memcpy(vendor_id  , vid, strlen(vid));
  memcpy(product_id , pid, strlen(pid));
  memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received SCSI_CMD_INQUIRY, v2 with full inquiry response
// Some inquiry_resp's fields are already filled with default values, application can update them
// Return length of inquiry response, typically sizeof(scsi_inquiry_resp_t) (36 bytes), can be longer if included vendor data.
uint32_t tud_msc_inquiry2_cb(uint8_t lun, scsi_inquiry_resp_t *inquiry_resp, uint32_t bufsize) {
  (void) lun;
  (void) bufsize;
  const char vid[] = "MPComp";
  const char pid[] = "Companion";
  const char rev[] = "2.0";

  (void) strncpy((char*) inquiry_resp->vendor_id, vid, 8);
  (void) strncpy((char*) inquiry_resp->product_id, pid, 16);
  (void) strncpy((char*) inquiry_resp->product_rev, rev, 4);

  return sizeof(scsi_inquiry_resp_t); // 36 bytes
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun) {
  (void) lun;

  // RAM disk is ready until ejected
  if (ejected) {
    // Additional Sense 3A-00 is NOT_FOUND
    return tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0x00);
  }

  return true;
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size) {
  (void) lun;
  *block_count = DISK_SECTOR_NUM;
  *block_size = DISK_SECTOR_SIZE;
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject) {
  (void) lun;
  (void) power_condition;

  if (load_eject) {
    if (start) {
      // load disk storage
    } else {
      // unload disk storage
      ejected = true;
    }
  }

  return true;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) {
  (void) lun;

  // out of ramdisk
  if (lba >= DISK_SECTOR_NUM) {
    return -1;
  }

  // Check for overflow of offset + bufsize
  if (lba * DISK_SECTOR_SIZE + offset + bufsize > DISK_SECTOR_NUM * DISK_SECTOR_SIZE) {
    return -1;
  }

  // printf("%02X %08X %08X %08X\n", lun, lba, offset, bufsize);

  if (lba < DISK_RAM_SECTORS) {
    uint8_t const *addr = ram_disk[lba] + offset;
    // printf("%p\n", addr);
    (void) memcpy(buffer, addr, bufsize);
  }
  else {
    uint8_t const *addr = (uint8_t *)COMPANION_FLASH_STORAGE_BASE 
                          + (lba - DISK_RAM_SECTORS) * DISK_SECTOR_SIZE
                          + offset;
    // printf("%p\n", addr);
    (void) memcpy(buffer, addr, bufsize);
  }


  return (int32_t) bufsize;
}

bool tud_msc_is_writable_cb(uint8_t lun) {
  (void) lun;

  return false;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize) {
  (void) lun;

  // out of ramdisk
  if (lba >= DISK_SECTOR_NUM) {
    return -1;
  }

  (void) lba;
  (void) offset;
  (void) buffer;

  return (int32_t) bufsize;
}

// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize) {
  (void) lun;
  (void) scsi_cmd;
  (void) buffer;
  (void) bufsize;

  // currently no other commands are supported

  // Set Sense = Invalid Command Operation
  (void) tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

  return -1; // stall/failed command request;
}

#endif