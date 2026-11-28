# The LilyGo T-Watch Ultra: the shared application (ports/esp_idf/app) with
# the desktop on the AMOLED and touch panel and the keyboard on the ESP32-S3's
# built-in USB Serial/JTAG port (link_twatch.c), and the static buffers in
# PSRAM (extram.lf).
include("${CMAKE_CURRENT_LIST_DIR}/../../esp_idf/app/app.cmake")

#
# WHICH PERSONALITY THIS BUILD IS. The two OTA slots hold DIFFERENT images, so
# the same source tree is built twice:
#
#   idf.py -B build-desktop -DTD_PERSONALITY=desktop build   -> flash 0x20000
#   idf.py -B build-slotb   -DTD_PERSONALITY=slotb   build   -> flash 0x400000
#
# ONE sdkconfig drives both. ESP-IDF keeps it at the PROJECT root rather than
# per build directory, so the two personalities read the same generated file and
# cannot disagree. A second sdkconfig would be the TOL-008 trap: settings that
# live only in a generated file vanish when it is regenerated, silently.
#
# What CAN drift is which sources each build compiles, so check_personality.py
# reads the two linked ELFs and fails if the desktop regains an emulator entry
# point. Run it after touching this file.
#
# WHY SPLIT AT ALL. Until slot B was proven on the glass, both slots held one
# image so that a bad slot B still left a working desktop. The cost was that the
# desktop linked four emulator cores to run menu items that only set a boot
# partition and restarted -- about 35 KB of internal DRAM in `.bss` that the
# desktop never touches, in the memory this board has least of, plus the whole
# image duplicated into both slots.
#
# WHAT THE DESKTOP LOSES: the in-process emulator commands (`gb`, `nes`, `pm`,
# `tama`, `maps`, `pmbench`, `gbcbench`). They are slot B apps now. See slotb.h.
#
set(TD_PERSONALITY "slotb" CACHE STRING "Which image to build: desktop or slotb")
set_property(CACHE TD_PERSONALITY PROPERTY STRINGS desktop slotb)

if(TD_PERSONALITY STREQUAL "slotb")
    set(TD_IS_SLOTB TRUE)
elseif(TD_PERSONALITY STREQUAL "desktop")
    set(TD_IS_SLOTB FALSE)
else()
    message(FATAL_ERROR
        "TD_PERSONALITY must be 'desktop' or 'slotb', not '${TD_PERSONALITY}'")
endif()

#
# THE PLAIN SOURCE LIST, shared by both personalities. Everything here is either
# driver code both images need or the shared desktop application.
#
set(TD_SRCS_COMMON
    "link_twatch.c" "display.c" "touch.c" "pusher.c" "pwrkey.c" "charge.c"
    "nfc.c" "nfc_scan.cpp" "gps.c" "nmea.c" "ts_convert.c" "power.c"
    "rtc.c" "haptic.c" "sound.c" "wav.c" "font8x16.c" "sdpaths.c"
    "gamepad.c" "statusbar.c" "usbdiag.c" "bleuart.c" "uac2.c" "usbhost.c"
    "modload.c" "modhost.c" "music_player.c" "music_ui.c"
    "memdiag.c" "imu.c" "imu_posture.c" "klio.c" "klio_store.c")

#
# THE EMULATOR PERSONALITY'S SOURCES. Four apps and the cartridge picker, plus
# the two benchmark commands that only exist to measure those emulators.
#
set(TD_SRCS_EMULATORS
    "gb.c" "gbpick.c" "pm.c" "nes.c" "tama.c" "tama_age.c" "tama_ui.c"
    "maps.c" "pmbench.c" "gbcbench.c")

#
# THE LAUNCHER is the desktop's half of the slot B subsystem; slotb.c is slot
# B's half. Exactly one of them is compiled into each image, and slotb.h explains
# why they are two files.
#
if(TD_IS_SLOTB)
    set(TD_SRCS_PERSONALITY ${TD_SRCS_EMULATORS} "slotb.c" "slotbl.c")
    set(TD_REQUIRES_PERSONALITY tamacore nofrendo pokemini gnuboy)
else()
    set(TD_SRCS_PERSONALITY "slotbl.c")
    set(TD_REQUIRES_PERSONALITY "")
endif()

idf_component_register(
    SRCS ${TD_SRCS_COMMON} ${TD_SRCS_PERSONALITY} ${TD_ESP_APP_SRCS}
    EMBED_TXTFILES "${TD_BOARD_BUILTIN_CONF}"
    INCLUDE_DIRS "." "${TD_ESP_APP_DIR}"
    LDFRAGMENTS "extram.lf"
    REQUIRES ${TD_ESP_APP_REQUIRES} st25r3916 ${TD_REQUIRES_PERSONALITY} bhy2 esp_driver_spi esp_driver_i2c esp_driver_gpio esp_timer
    PRIV_REQUIRES usb bt esp_ringbuf
             esp_driver_i2s esp_driver_uart esp_driver_usb_serial_jtag
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

if(TD_IS_SLOTB)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_PERSONALITY_SLOTB=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_PERSONALITY_DESKTOP=1)
endif()

# Pull slotb.o out of libmain.a even though nothing apparently needs it. main.c's
# board_startup_override() is WEAK, so without this the linker is satisfied by
# the weak no-op and never fetches the strong definition -- a silent no-op that
# looks like slot B running. See slotb.c.
#
# SLOT B ONLY. The desktop defines nothing here: its weak no-op IS the correct
# behaviour, and asking for a symbol the desktop does not compile would fail the
# link.
if(TD_IS_SLOTB)
    target_link_options(${COMPONENT_LIB} PUBLIC "-Wl,--undefined=slotb_force_link")
endif()

# DIAGNOSTIC BUILD ONLY. Defines a Music-window self-test that runs at boot and
# leaves its answer in /fs/root/musicboot.txt, because the display mirror and a
# focused window cannot be trusted to report one. Uncomment to run it, and
# comment it out again once the question is answered -- it delays boot by six
# seconds and plays audio nobody asked for.
# target_compile_definitions(${COMPONENT_LIB} PRIVATE MUSIC_UI_SELFTEST=1)


# Its own update feed: this image draws to a QSPI AMOLED, and the esp32s3
# image would replace it with one that talks to a UART.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_UPDATE_BOARD="twatchultra")

# This board has an IMU, so ports/esp_idf/app fills td_sysinfo()->imu.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_BOARD_HAS_IMU=1)

# No terminal-size polling. The desktop asks the terminal for its size once a
# second because a serial terminal can never report a resize -- but this
# panel's geometry is fixed at build time, so there is nothing to discover and
# the question is not free: the query moves the cursor to the bottom-right
# corner to measure it, and every cell that touches repaints. On a 40x25 grid
# that showed up as a full-screen flash once a second, on both the panel and
# the USB mirror.
target_compile_definitions(${COMPONENT_LIB} PRIVATE TD_SIZE_POLL_MS=0)
