# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD adafruit_feather_rp2040_adalogger CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(motorpressure-companion C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(motorpressure-companion motorpressure-companion.c 
                                       usb_descriptors.c 
                                       companion_disk.c)

pico_generate_pio_header(motorpressure-companion ${CMAKE_CURRENT_LIST_DIR}/pio_spi.pio)

pico_set_program_name(motorpressure-companion "motorpressure-companion")
pico_set_program_version(motorpressure-companion "0.1")

# Want the binary to be stored in flash and loaded into RAM on boot.
# However breakpoints may not work this way. For debugging use `no_flash`
pico_set_binary_type(motorpressure-companion copy_to_ram)
# pico_set_binary_type(motorpressure-companion no_flash)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(motorpressure-companion 0)
pico_enable_stdio_usb(motorpressure-companion 1)

# Add the standard library to the build
target_link_libraries(motorpressure-companion
        pico_stdlib
        pico_unique_id
        hardware_pio
        hardware_dma
        hardware_adc
        tinyusb_device
        tinyusb_board)

# Add the standard include files to the build
target_include_directories(motorpressure-companion PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(motorpressure-companion)

# In order to define the memory map, the toolchain uses a linker script located at
# ~/.pico-sdk/sdk/2.2.0/src/rp2_common/pico_crt0/rp2040. In order to allow individual boards
# to specify flash size, another separate linker script template is INCLUDE-d from within this script.
# We can hijack this to specify our own flash regions if we overwrite it prior to linking. However
# this does not give symbols that are accessible from C so the memory locations must be hardcoded.
# See https://community.element14.com/products/raspberry-pi/b/blog/posts/raspberry-pico-c-sdk-reserve-a-flash-memory-block-for-persistent-storage
configure_file(${CMAKE_CURRENT_LIST_DIR}/companion_flash_layout.ld ${CMAKE_BINARY_DIR}/pico_flash_region.ld)

# Enable RESET via RPi method & Baud1200 method
target_compile_definitions(motorpressure-companion PRIVATE
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=1
    PICO_STDIO_USB_RESET_MAGIC_BAUD_RATE=1200
    PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK=0
)
target_compile_definitions(motorpressure-companion PUBLIC
   PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1
   PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_MS_OS_20_DESCRIPTOR=1
   PICO_STDIO_USB_RESET_INTERFACE_MS_OS_20_DESCRIPTOR_ITF=0
   PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_BOOTSEL=1
   PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_RESET_TO_FLASH_BOOT=1
   PICO_STDIO_USB_RESET_BOOTSEL_INTERFACE_DISABLE_MASK=0
   PICO_STDIO_USB_RESET_RESET_TO_FLASH_DELAY_MS=100
)

# Adjust the system clock speed
# set(SYS_CLK_HZ 48000000)