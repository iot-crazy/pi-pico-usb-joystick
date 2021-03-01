cmake_minimum_required(VERSION 3.12)

#set (PICO_TINYUSB_PATH "C:\\IOT\\raspipico\\repos\\tinyusb")

include(pico_sdk_import.cmake)

set(projname "usbjoystick")
project($(projname) C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CSS_STANDARD 17)

pico_sdk_init()

#include(example_auto_set_url.cmake)
add_executable(${projname}
            usbjoystick.c
            usb_descriptors.c
            )

target_include_directories(usbjoystick PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${projname} pico_stdlib tinyusb_device tinyusb_board hardware_adc)

pico_add_extra_outputs(${projname})

