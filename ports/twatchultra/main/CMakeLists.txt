# The LilyGo T-Watch Ultra: the shared application (ports/esp_idf/app) with
# the desktop on the AMOLED and touch panel and the keyboard on the ESP32-S3's
# built-in USB Serial/JTAG port (link_twatch.c), and the static buffers in
# PSRAM (extram.lf).
include("${CMAKE_CURRENT_LIST_DIR}/../../esp_idf/app/app.cmake")

idf_component_register(
    SRCS "link_twatch.c" "display.c" "touch.c" "font8x16.c" ${TD_ESP_APP_SRCS}
    EMBED_TXTFILES "${TD_BOARD_BUILTIN_CONF}"
    INCLUDE_DIRS "." "${TD_ESP_APP_DIR}"
    LDFRAGMENTS "extram.lf"
    REQUIRES ${TD_ESP_APP_REQUIRES} esp_driver_spi esp_driver_i2c esp_driver_gpio
             esp_driver_usb_serial_jtag
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

# Its own update feed: this image draws to a QSPI AMOLED, and the esp32s3
# image would replace it with one that talks to a UART.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_UPDATE_BOARD="twatchultra")
