# The Arduino Nano ESP32: a console, not a desktop.
#
# TinyDesk Shell runs on its own over USB Serial/JTAG when a host terminal is
# attached, and on the ILI9340 panel when one is not (link_nano.c). The window
# manager, the app set and the desktop's screen buffers are not built into
# this image: that is what makes room for WiFi, which the previous desktop
# build could not bring up.
#
# tinydesk stays a REQUIRED component for its headers only (td_modbus.h, which
# link.h needs for board_rtu_lines). Nothing here calls into it, so its static
# buffers are not referenced and the linker drops them.
include("${CMAKE_CURRENT_LIST_DIR}/../../esp_idf/app/app.cmake")

idf_component_register(
    SRCS "main_console.c" "link_nano.c" "ili9340.c" "vt.c" "node.cpp" "sensors.cpp"
         "lora.cpp" "lora_frame.cpp" "tang_proto.cpp" "tang_link.cpp"
         "${TD_ESP_APP_DIR}/rs485.c"
    EMBED_TXTFILES "${TD_BOARD_BUILTIN_CONF}"
    INCLUDE_DIRS "." "${TD_ESP_APP_DIR}"
    LDFRAGMENTS "extram.lf"
    REQUIRES tinydesk tdsh nvs_flash app_update esp_driver_uart esp_timer
             esp_netif esp_event lwip mbedtls json esp_http_client
             esp_driver_usb_serial_jtag esp_driver_spi esp_driver_gpio driver
             esp_driver_i2c rns lxm joltwallet__littlefs heap
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
# desktop on UART0 through a bridge chip this board does not have. Name this
# port's own board, which has no update feed yet.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_UPDATE_BOARD="nanoesp32")
