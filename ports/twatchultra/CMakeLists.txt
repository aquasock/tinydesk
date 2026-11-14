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

# THE GRID, IN CELLS. This board's screen is a soldered panel, not a terminal
# whose size is discovered at run time, so the grid is fixed here and the
# desktop is clamped to it.
#
# The framebuffer is 320x416 (main/display.h, DISPLAY_W x DISPLAY_H) at the
# 8x16 font, which is 40 columns by 26 rows. Keep these three in step: this is
# what the window manager believes, and DISPLAY_W/H are what the panel
# actually shows. They disagreed before -- 30 rows here, 26 rendered -- and the
# failure is invisible until an app puts something on the rows that do not
# exist.
#
# Setting these makes the `tinydesk` component use them instead of its own
# PSRAM-sized defaults, so a host terminal attached over USB cannot resize the
# desktop to its own window: both views show the same 40x26.
#
# PLAIN set(), NOT CACHE. A cached value survives in the build directory, so
# changing the number here while a build directory already exists changes
# nothing -- idf.py keeps reporting the old grid and the old cache entry has to
# be deleted by hand. This is a property of the soldered panel, not a choice the
# builder makes, so it should not be cached: -DTD_MAX_ROWS= on the command line
# is deliberately ignored, because a grid larger than the glass would only clip.
set(TD_MAX_COLS     40)
set(TD_MAX_ROWS     26)
set(TD_VT_SCROLLBACK 50 CACHE STRING "terminal scrollback lines")

list(APPEND EXTRA_COMPONENT_DIRS
    "${TDSH_SDK_ROOT}/ports/esp_idf/components"
    "${TD_ROOT}/ports/esp_idf/components")

if(DEFINED ENV{TD_VERSION})
    set(PROJECT_VER "$ENV{TD_VERSION}")
else()
    set(PROJECT_VER "0.4.0")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

#
# ONE DIAGNOSTIC IS DEMOTED FOR THE WHOLE BUILD, AND ONLY BECAUSE OF -O2.
#
# The firmware is built at -O2 (CONFIG_COMPILER_OPTIMIZATION_PERF). Above -Og,
# GCC inlines far enough to see the destination size of two snprintf() calls in
# tdsh's prompt builders, and -Werror=all then turns -Wformat-truncation into a
# build failure. Neither site is a memory-safety fault: snprintf() bounds the
# write by construction, so the worst case is a truncated prompt, which is
# cosmetic and visible on the glass.
#
# IT IS SET HERE RATHER THAN IN THE COMPONENT because tdsh is a submodule pinned
# to upstream (tinydesk-project/tinydesk-shell), and this fork publishes to
# aquasock only. Editing the submodule would either make the build depend on a
# commit nobody else can fetch, or require pushing to a repository this project
# must not push to. The parent is also where the -O2 decision lives, so the
# exception belongs beside it.
#
# SCOPE AND COST: this is build-wide, so a genuine truncation anywhere would be
# demoted to a warning rather than an error. It still prints -- it is invisible
# in the build log only if nobody reads it -- and the alternative, raising those
# prompt buffers to a theoretical 603 bytes each, spends internal RAM that this
# board does not have.
#
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=format-truncation" APPEND)

project(tinydesk)

# No terminal-size polling (TD_SIZE_POLL_MS=0, see main/project_include.cmake).
#
# It is NOT set here. td.c lives in the shared `tinydesk` component, and
# `add_compile_definitions` at this point in the file comes after ESP-IDF has
# registered the components, so the definition only reaches the components
# added afterwards -- this port's `main`, and not `tinydesk`. That was the
# first attempt and it did not work; the define reached 19 translation units
# and not the one that reads it. main/project_include.cmake is the place the
# build system guarantees is evaluated first.

# Fresh build date and time in every firmware image.
include("${TD_ROOT}/ports/common/app_desc_stamp.cmake")

include("${TDSH_SDK_ROOT}/ports/esp_idf/tdsh_espidf_project_setup.cmake")
