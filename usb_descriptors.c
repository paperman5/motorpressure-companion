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

// See https://forums.raspberrypi.com/viewtopic.php?t=394703
#include "pico/usb_reset_interface.h"

#ifndef TUD_RPI_RESET_DESC_LEN
#define TUD_RPI_RESET_DESC_LEN  9
#endif

#ifndef TUD_RPI_RESET_DESCRIPTOR
#define TUD_RPI_RESET_DESCRIPTOR(_itfnum, _stridx) \
  9, TUSB_DESC_INTERFACE, _itfnum, 0, 0, TUSB_CLASS_VENDOR_SPECIFIC, RESET_INTERFACE_SUBCLASS, RESET_INTERFACE_PROTOCOL, _stridx
#endif

// Raspberry Pi Pico info - as far as I can tell this is required for reset functionality
// (If changing VID&PID, resetting via `picotool load -fx` won't work).
// Can use `picotool load --vid VID --pid PID -fx` but the pico will default back to original
// VID/PID when in BOOTSEL mode which picotool then can't see.
#define USB_VID   0x2E8A
#define USB_PID   0x000A
#define USB_BCD   0x0200
#define DEV_MANUFACTURER "Raspberry Pi"
#define DEV_PRODUCT "Pico"

// #define USB_VID   0xD007
// #define USB_PID   0x4003
// #define USB_BCD   0x0200
// #define DEV_MANUFACTURER "DOOT"
// #define DEV_PRODUCT "Pressure Companion"

// String Descriptor Index
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_RPI_RESET,
  STRID_CDC,
  STRID_MSC,
};

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,

    // // Use Interface Association Descriptor (IAD) for CDC
    // // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    // .bDeviceClass       = TUSB_CLASS_MISC,
    // .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    // .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bDeviceClass       = TUSB_CLASS_UNSPECIFIED, // Let the individual classes be set in the interface descriptors

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum {
  ITF_NUM_RPI_RESET,
  ITF_NUM_CDC,
  ITF_NUM_CDC_DATA,
  ITF_NUM_MSC,
  ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define EPNUM_MSC_OUT     0x03
#define EPNUM_MSC_IN      0x83

#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_MSC_DESC_LEN + TUD_RPI_RESET_DESC_LEN)

// full speed configuration
static uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // RPi Reset Interface
    TUD_RPI_RESET_DESCRIPTOR(ITF_NUM_RPI_RESET, STRID_RPI_RESET),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 16, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, STRID_MSC, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),

};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void) index; // for multiple configurations

  return desc_configuration;
};

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 }, // 0: is supported language is English (0x0409)
    DEV_MANUFACTURER,              // 1: Manufacturer
    DEV_PRODUCT,                   // 2: Product
    NULL,                          // 3: Serials will use unique ID if possible
    "RP2040 RESET",                // 4: RPi Reset Interface
    "Board CDC",                   // 5: CDC Interface
    "Board MSC",                   // 6: MSC Interface
};

static_assert(ITF_NUM_RPI_RESET == PICO_STDIO_USB_RESET_INTERFACE_MS_OS_20_DESCRIPTOR_ITF, "ITF_NUM_RPI_RESET must be equal to the PICO_STDIO_USB_RESET_INTERFACE_MS_OS_20_DESCRIPTOR_ITF set in CMakeLists.txt");

static uint16_t _desc_str[32 + 1];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void) langid;
  size_t chr_count;

  switch ( index ) {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      chr_count = board_usb_get_serial(_desc_str + 1, 32);
      break;

    default:
      // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
      // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

      if ( !(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) ) { return NULL; }

      const char *str = string_desc_arr[index];

      // Cap at max char
      chr_count = strlen(str);
      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1; // -1 for string type
      if ( chr_count > max_count ) { chr_count = max_count; }

      // Convert ASCII string into UTF-16
      for ( size_t i = 0; i < chr_count; i++ ) {
        _desc_str[1 + i] = str[i];
      }
      break;
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}