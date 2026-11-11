# tinydesk firmware for the LilyGo T-Watch Ultra (ESP32-S3, 16 MB flash,
# 8 MB QUAD PSRAM; 410x502 AMOLED, CO5300 controller on QSPI; CST9217 touch),
# ESP-IDF 5.3.1.
#
#   idf.py set-target esp32s3
#   idf.py build
#
# The desktop is drawn on the AMOLED (main/display.c) and the touch panel is
# its mouse (main/touch.c). The keyboard arrives on the ESP32-S3's own USB
# Serial/JTAG peripheral, which is also the mirror of the picture: this board
# has no USB-UART bridge chip. The application is shared with the other
# ESP-IDF projects (../esp_idf/app); this project brings the panel, the touch
# controller, the USB link, the PSRAM placement (main/extram.lf), the
# partitions and the sdkconfig.
cmake_minimum_required(VERSION 3.16)

get_filename_component(TD_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(TDSH_SDK_ROOT "${TD_ROOT}/third_party/tdsh")

set(IDF_TARGET "esp32s3" CACHE STRING "tinydesk target")

# Screen limits, in cells. The panel is 320x400 after its own bezel is taken
# off; at the 8x16 font that is 40x25 cells, and the desktop follows whatever
# the link reports, so these are an upper bound rather than a target.
set(TD_MAX_COLS     40 CACHE STRING "screen cells across (panel grid)")
set(TD_MAX_ROWS     30 CACHE STRING "screen cells down (panel grid)")
set(TD_VT_SCROLLBACK 50 CACHE STRING "terminal scrollback lines")

list(APPEND EXTRA_COMPONENT_DIRS
    "${TDSH_SDK_ROOT}/ports/esp_idf/components"
    "${TD_ROOT}/ports/esp_idf/components"
    "${CMAKE_CURRENT_LIST_DIR}/components")

if(DEFINED ENV{TD_VERSION})
    set(PROJECT_VER "$ENV{TD_VERSION}")
else()
    set(PROJECT_VER "0.4.0")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tinydesk)

# Fresh build date and time in every firmware image.
include("${TD_ROOT}/ports/common/app_desc_stamp.cmake")

include("${TDSH_SDK_ROOT}/ports/esp_idf/tdsh_espidf_project_setup.cmake")
