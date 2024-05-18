cmake_minimum_required(VERSION 3.13)

# PICO_BOARD should be set to 'pico_w', or not set at all.
if(DEFINED PICO_BOARD AND NOT ${PICO_BOARD} STREQUAL "pico_w")
    message(FATAL_ERROR "PICO_BOARD must be set to 'pico_w' or not set at all")
else()
    set(PICO_BOARD "pico_w")
endif()

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

set(BLUEPAD32_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/bluepad32)

# To use BTstack from Pico SDK do
#set(BTSTACK_ROOT ${PICO_SDK_PATH}/lib/btstack)
# To use it from Bluepad32 (up-to-date, with custom patches for controllers):
set(BTSTACK_ROOT ${BLUEPAD32_ROOT}/external/btstack)
set(PICO_BTSTACK_PATH ${BTSTACK_ROOT})

project(picoblue C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(picoblue
    src/main.c
    src/my_platform.c
    src/gamecube.c
    src/intercore.c
)

pico_generate_pio_header(${PROJECT_NAME}   
        ${CMAKE_CURRENT_LIST_DIR}/src/picoboot.pio
        
)

pico_generate_pio_header(${PROJECT_NAME}   
        ${CMAKE_CURRENT_LIST_DIR}/src/joybus.pio
)

target_include_directories(picoblue PRIVATE
    src
    ${BLUEPAD32_ROOT}/src/components/bluepad32/include)

# Needed for btstack_config.h / sdkconfig.h
# so that libbluepad32 can include them
include_directories(picoblue src)

# Needed when using BTstack from our branch
include_directories(${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/include)
include_directories(${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/include)

target_link_libraries(picoblue
    pico_stdlib
    hardware_pio 
    hardware_dma
    pico_cyw43_arch_none
    pico_btstack_classic
    pico_btstack_cyw43
    bluepad32
    pico_multicore
)

add_subdirectory(${BLUEPAD32_ROOT}/src/components/bluepad32 libbluepad32)

pico_enable_stdio_usb(picoblue 1)
pico_enable_stdio_uart(picoblue 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(picoblue)
