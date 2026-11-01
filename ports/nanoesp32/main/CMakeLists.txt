# The Arduino Nano ESP32: the shared application (ports/esp_idf/app), with the
# static buffers of TinyDesk and TinyDesk Shell in PSRAM (extram.lf).
#
# The desktop has two sinks and picks between them at runtime: USB
# Serial/JTAG when a host terminal is attached, the ILI9340 panel otherwise.
# link_nano.c is both. It supersedes link_lcd.c (panel output with USB input)
# and link_usj.c (USB only, as ports/esp32c6 ships it); those files remain in
# the tree because they document the two halves, but neither is built.
include("${CMAKE_CURRENT_LIST_DIR}/../../esp_idf/app/app.cmake")

idf_component_register(
    SRCS "link_nano.c" "ili9340.c" "vt.c" "node.cpp" "sensors.cpp" ${TD_ESP_APP_SRCS}
    EMBED_TXTFILES "${TD_BOARD_BUILTIN_CONF}"
    INCLUDE_DIRS "." "${TD_ESP_APP_DIR}"
    LDFRAGMENTS "extram.lf"
    REQUIRES ${TD_ESP_APP_REQUIRES} esp_driver_usb_serial_jtag esp_driver_spi esp_driver_gpio
             esp_wifi esp_netif esp_event nvs_flash driver esp_driver_i2c rns lxm
             joltwallet__littlefs heap
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
# microReticulum is C++ with exceptions; node.cpp includes its headers.
target_compile_options(${COMPONENT_LIB} PRIVATE -fexceptions)
# node.cpp uses microStore's POSIX (stdio) filesystem adapter, whose header is
# guarded by this define.
target_compile_definitions(${COMPONENT_LIB} PRIVATE USTORE_USE_STDIOFS)
# node.cpp includes microStore's adapter header, which brace-initialises struct
# stat; -Werror turns that third-party warning into a build failure. Narrow the
# suppression to that one warning so first-party code stays strict.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)

# Buffer sizes. The library defaults are TD_MAX_COLS 400 / TD_MAX_ROWS 150 with
# 200 lines of terminal scrollback, sized for a full desktop monitor: two
# 480 KB screen buffers and 300 KB of terminal. This panel is 40x15 cells, so
# the maxima are set to what the display can actually show and the scrollback
# to a useful-but-cheap ring. Costs about 13 KB instead of about 680 KB.
#
# The maxima are no longer the panel's size. The panel's grid is 40 x 15
# (LCD_W / FONT_W by LCD_H / FONT_H in ili9340.h) and is pinned in link_nano.c;
# the maxima now bound what a host terminal may ask for over USB.
# TD_MAX_COLS / TD_MAX_ROWS / TD_VT_SCROLLBACK are set in ../CMakeLists.txt,
# before project(), so the tinydesk component sees them when it configures.

# Optional build-time WiFi credentials, for a bench image with no way to write
# /fs/etc/wifi.conf yet. Absent in a normal build, where that file is the only
# source. Pass with -DTD_WIFI_SSID=... -DTD_WIFI_PASSWORD=...
if(TD_NODE_WIFI_ONLY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_NODE_WIFI_ONLY)
endif()

if(TD_WIFI_SSID)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_WIFI_SSID="${TD_WIFI_SSID}")
    if(TD_WIFI_PASSWORD)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_WIFI_PASSWORD="${TD_WIFI_PASSWORD}")
    endif()
endif()

# The chip is the same ESP32-S3 as ports/esp32s3, but that image serves the
# desktop on UART0 through a bridge chip this board does not have: taking it
# from Software Update would leave a working firmware with no way to show
# it. Name this port's own board, which has no update feed yet.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_UPDATE_BOARD="nanoesp32")
