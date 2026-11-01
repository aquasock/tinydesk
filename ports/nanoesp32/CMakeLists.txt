# tinydesk firmware for the Arduino Nano ESP32 (ESP32-S3, NORA-W106,
# 16 MB flash, 8 MB Octal PSRAM), ESP-IDF 5.3.1.
#
#   idf.py set-target esp32s3
#   idf.py build
#   idf.py -p COMx flash                  (no baud argument: the link is USB)
#
# The board has no USB-UART bridge chip: its USB-C connector is wired to the
# ESP32-S3's own USB Serial/JTAG peripheral (GPIO19/20). The desktop is
# therefore served on that port, as on the ESP32-C6 (main/link_usj.c), not on
# UART0 as in ports/esp32s3. The ESP-IDF console is off (logs go to the Log
# Viewer). The application is shared with the other ESP-IDF projects
# (../esp_idf/app); this project brings the USB link, the PSRAM placement
# (main/extram.lf), the partitions and the sdkconfig.
cmake_minimum_required(VERSION 3.16)

get_filename_component(TD_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(TDSH_SDK_ROOT "${TD_ROOT}/third_party/tdsh")

set(IDF_TARGET "esp32s3" CACHE STRING "tinydesk target")

# Screen and terminal buffer limits, in cells.
#
# These are an upper bound, not the layout: the desktop asks the active sink
# how big the terminal is and sizes itself to the answer, so a small window
# uses a small grid whatever these say. They must be cache variables set
# BEFORE project(), because the tinydesk component is configured before main/
# and reads them then.
# 160 x 50 is the largest grid a host terminal may fill over USB: 160*50*8 =
# 64 KB per screen, two screens = 128 KB, all of it in PSRAM through
# extram.lf. The panel does not use these; its 40 x 15 grid is pinned in
# link_nano.c from LCD_W/LCD_H, and the desktop re-lays-out on the size the
# active sink reports. Scrollback is cheap and useful now that there is a real
# terminal to read it in.
set(TD_MAX_COLS     160 CACHE STRING "screen cells across, upper bound for a USB terminal")
set(TD_MAX_ROWS     50 CACHE STRING "screen cells down, upper bound for a USB terminal")
set(TD_VT_SCROLLBACK 200 CACHE STRING "terminal scrollback lines")
list(APPEND EXTRA_COMPONENT_DIRS
    "${TDSH_SDK_ROOT}/ports/esp_idf/components"
    "${TD_ROOT}/ports/esp_idf/components"
    "${CMAKE_CURRENT_LIST_DIR}/components")

# The firmware version shown by Software Update and `ota status`.
# Override for a test build: set TD_VERSION=0.1.6 before idf.py build.
if(DEFINED ENV{TD_VERSION})
    set(PROJECT_VER "$ENV{TD_VERSION}")
else()
    set(PROJECT_VER "0.2.0")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tinydesk)

# Fresh build date and time in every firmware image.
include("${TD_ROOT}/ports/common/app_desc_stamp.cmake")

# Compatibility settings for TinyDesk Shell's pinned SMB / wolfSSL dependencies.
include("${TDSH_SDK_ROOT}/ports/esp_idf/tdsh_espidf_project_setup.cmake")
