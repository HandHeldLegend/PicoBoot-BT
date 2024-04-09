cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD "pico_w")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(picobootbt C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()



add_executable(${PROJECT_NAME} 
src/picoboot.c
src/gamecube.c
src/pico_gamecube_platform.c
)

pico_generate_pio_header(${PROJECT_NAME}   
        ${CMAKE_CURRENT_LIST_DIR}/src/picoboot.pio
        
)

pico_generate_pio_header(${PROJECT_NAME}   
        ${CMAKE_CURRENT_LIST_DIR}/src/joybus.pio
)



target_include_directories(${PROJECT_NAME} PRIVATE
    src
    bluepad32/src/components/bluepad32/include)
include_directories(
        src
)

pico_set_program_name(${PROJECT_NAME}  "picobootBT")
pico_set_program_version(${PROJECT_NAME}  "0.1")
#pico_set_binary_type(${PROJECT_NAME} copy_to_ram)

pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

target_link_libraries(${PROJECT_NAME} PRIVATE 
        pico_stdlib 
        hardware_pio 
        hardware_dma
        
        pico_cyw43_arch_none
        pico_btstack_ble
        pico_btstack_classic
        pico_btstack_cyw43
        bluepad32
        pico_multicore
)

add_subdirectory(bluepad32/src/components/bluepad32 libbluepad32)

pico_add_extra_outputs(${PROJECT_NAME})