# The Arduino Nano ESP32: the shared application (ports/esp_idf/app), with
# the static buffers of TinyDesk and TinyDesk Shell in PSRAM (extram.lf).
#
# Two desktop links are built from this directory:
#
#   link_usj.c  the desktop as terminal output on the USB-C port (default)
#   link_lcd.c  the desktop drawn on an ILI9340 panel, input still over USB
#
# The panel link is chosen with -DTD_LINK=lcd. Both provide the same four
# functions from link.h, so nothing outside this directory changes.
include("${CMAKE_CURRENT_LIST_DIR}/../../esp_idf/app/app.cmake")

set(TD_LINK "usb" CACHE STRING "desktop link: usb (terminal on USB) or lcd")

if(TD_LINK STREQUAL "lcd")
    set(TD_LINK_SRCS "link_lcd.c" "ili9340.c" "vt.c")
    set(TD_LINK_REQ esp_driver_spi esp_driver_gpio)
    message(STATUS "tinydesk link: ILI9340 panel (input over USB)")
else()
    set(TD_LINK_SRCS "link_usj.c")
    set(TD_LINK_REQ "")
    message(STATUS "tinydesk link: terminal on USB Serial/JTAG")
endif()

idf_component_register(
    SRCS ${TD_LINK_SRCS} ${TD_ESP_APP_SRCS}
    EMBED_TXTFILES "${TD_BOARD_BUILTIN_CONF}"
    INCLUDE_DIRS "." "${TD_ESP_APP_DIR}"
    LDFRAGMENTS "extram.lf"
    REQUIRES ${TD_ESP_APP_REQUIRES} esp_driver_usb_serial_jtag ${TD_LINK_REQ}
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

# The chip is the same ESP32-S3 as ports/esp32s3, but that image serves the
# desktop on UART0 through a bridge chip this board does not have: taking it
# from Software Update would leave a working firmware with no way to show
# it. Name this port's own board, which has no update feed yet.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_UPDATE_BOARD="nanoesp32")
