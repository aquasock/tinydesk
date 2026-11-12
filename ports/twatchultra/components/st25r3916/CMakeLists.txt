# ST's ST25R3916 driver, RFAL abstraction layer and NDEF stack, compiled for
# ESP-IDF behind the three headers in include/.
#
# The sources under rfal/ are byte-for-byte the vendor's, from
#   https://github.com/lewisxhe/ST25R3916-NFC-RFAL  tag v1.0.1
# which merges ST's ST25R3916 fork and ST's NFC-RFAL fork. LilyGo's own
# firmware for this watch drives this reader through this exact code.
#
# NOTHING IN rfal/ IS EDITED. Everything this port needed was expressible in
# include/Arduino.h, include/SPI.h and include/Wire.h, so re-vendoring a newer
# tag is a straight copy. demo_ce.cpp -- the card-emulation demo wrapper, not
# part of the stack -- is the one file left out.

file(GLOB rfal_srcs "${CMAKE_CURRENT_LIST_DIR}/rfal/*.cpp")

idf_component_register(
    SRCS "shim/arduino_shim.cpp" ${rfal_srcs}
    INCLUDE_DIRS "include" "rfal"
    PRIV_REQUIRES esp_driver_spi esp_driver_gpio
)

# ST's library is not ours to lint: 40 000 lines of vendor code kept unmodified
# on purpose, so this project's -Wall -Wextra -Werror is turned off for those
# files alone. The shim still gets them.
#
# THE GUARD IS LOAD-BEARING. ESP-IDF expands every component's CMakeLists twice
# -- once in a scripting-mode pass that works out the dependency graph, and once
# for the real build. set_source_files_properties() is not available in the
# first of those and fails there with "command is not scriptable", which reads
# as a build-system fault rather than as a line to move.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set_source_files_properties(${rfal_srcs} PROPERTIES COMPILE_OPTIONS "-w")
endif()
