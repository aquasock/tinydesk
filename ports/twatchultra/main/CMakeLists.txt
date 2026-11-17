# The LilyGo T-Watch Ultra: the shared application (ports/esp_idf/app) with
# the desktop on the AMOLED and touch panel and the keyboard on the ESP32-S3's
# built-in USB Serial/JTAG port (link_twatch.c), and the static buffers in
# PSRAM (extram.lf).
include("${CMAKE_CURRENT_LIST_DIR}/../../esp_idf/app/app.cmake")

idf_component_register(
    SRCS "link_twatch.c" "display.c" "touch.c" "pusher.c" "pwrkey.c" "charge.c" "nfc.c" "nfc_scan.cpp" "gps.c" "nmea.c" "ts_convert.c" "power.c" "rtc.c" "haptic.c" "sound.c" "wav.c" "font8x16.c" "sdpaths.c" "gamepad.c" "statusbar.c" "usbdiag.c" "bleuart.c" "uac2.c" "usbhost.c"  "modload.c" "modhost.c" "music_player.c" "music_ui.c" "tama.c" "tama_ui.c" "nes.c" "pmbench.c" "gbcbench.c" "gb.c" "pm.c" "memdiag.c" "imu.c" ${TD_ESP_APP_SRCS}
    EMBED_TXTFILES "${TD_BOARD_BUILTIN_CONF}"
    INCLUDE_DIRS "." "${TD_ESP_APP_DIR}"
    LDFRAGMENTS "extram.lf"
    REQUIRES ${TD_ESP_APP_REQUIRES} st25r3916 tamacore nofrendo pokemini gnuboy bhy2 esp_driver_spi esp_driver_i2c esp_driver_gpio esp_timer
    PRIV_REQUIRES usb bt esp_ringbuf
             esp_driver_i2s esp_driver_uart esp_driver_usb_serial_jtag
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

# DIAGNOSTIC BUILD ONLY. Defines a Music-window self-test that runs at boot and
# leaves its answer in /fs/root/musicboot.txt, because the display mirror and a
# focused window cannot be trusted to report one. Uncomment to run it, and
# comment it out again once the question is answered -- it delays boot by six
# seconds and plays audio nobody asked for.
# target_compile_definitions(${COMPONENT_LIB} PRIVATE MUSIC_UI_SELFTEST=1)


# Its own update feed: this image draws to a QSPI AMOLED, and the esp32s3
# image would replace it with one that talks to a UART.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_UPDATE_BOARD="twatchultra")

# No terminal-size polling. The desktop asks the terminal for its size once a
# second because a serial terminal can never report a resize -- but this
# panel's geometry is fixed at build time, so there is nothing to discover and
# the question is not free: the query moves the cursor to the bottom-right
# corner to measure it, and every cell that touches repaints. On a 40x25 grid
# that showed up as a full-screen flash once a second, on both the panel and
# the USB mirror.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_SIZE_POLL_MS=0)
