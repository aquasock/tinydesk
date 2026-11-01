# The Arduino Nano ESP32: the shared application (ports/esp_idf/app), with
# the desktop drawn on the ILI9340 panel and the static buffers of TinyDesk and
# TinyDesk Shell in PSRAM (extram.lf).
#
# This board is remote-first: the panel is the display and USB is not a
# tether, so only link_lcd.c is built. link_usj.c (the desktop as terminal
# output on USB) is kept in the tree for bench work; build it by adding it to
# TD_LINK_SRCS below.
include("${CMAKE_CURRENT_LIST_DIR}/../../esp_idf/app/app.cmake")

idf_component_register(
    SRCS "link_lcd.c" "ili9340.c" "vt.c" ${TD_ESP_APP_SRCS}
    EMBED_TXTFILES "${TD_BOARD_BUILTIN_CONF}"
    INCLUDE_DIRS "." "${TD_ESP_APP_DIR}"
    LDFRAGMENTS "extram.lf"
    REQUIRES ${TD_ESP_APP_REQUIRES} esp_driver_usb_serial_jtag esp_driver_spi esp_driver_gpio
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

# Buffer sizes. The library defaults are TD_MAX_COLS 400 / TD_MAX_ROWS 150 with
# 200 lines of terminal scrollback, sized for a full desktop monitor: two
# 480 KB screen buffers and 300 KB of terminal. This panel is 40x15 cells, so
# the maxima are set to what the display can actually show and the scrollback
# to a useful-but-cheap ring. Costs about 13 KB instead of about 680 KB.
#
# Keep TD_MAX_COLS/ROWS in step with LCD_W/LCD_H in ili9340.h: the grid is
# LCD_W / FONT_W x LCD_H / FONT_H, which is 40 x 15.
# TD_MAX_COLS / TD_MAX_ROWS / TD_VT_SCROLLBACK are set in ../CMakeLists.txt,
# before project(), so the tinydesk component sees them when it configures.

# The chip is the same ESP32-S3 as ports/esp32s3, but that image serves the
# desktop on UART0 through a bridge chip this board does not have: taking it
# from Software Update would leave a working firmware with no way to show
# it. Name this port's own board, which has no update feed yet.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_UPDATE_BOARD="nanoesp32")
