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
# This image runs a console, not a desktop, so these are only an upper bound
# that nothing large uses: the shell's line editor asks the active sink for its
# width (40 columns on the panel, the host terminal's own width over USB) and
# the desktop screen buffers are never allocated because nothing references
# them. Kept at the panel's size so an accidental reference stays cheap.
set(TD_MAX_COLS     40 CACHE STRING "screen cells across (panel grid)")
set(TD_MAX_ROWS     15 CACHE STRING "screen cells down (panel grid)")
set(TD_VT_SCROLLBACK 50 CACHE STRING "terminal scrollback lines")
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
