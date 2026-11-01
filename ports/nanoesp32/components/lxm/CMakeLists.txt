# microLXMF as an ESP-IDF component. Depends on the `rns` component for
# microReticulum itself; see components/rns/CMakeLists.txt for why the shim
# and the -DARDUINO rule exist.
#
# Point TD_LXMF_DIR at the microLXMF checkout.

# The dependency paths are owned by the rns component, which this one requires.
set(TD_RNS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../../../TinyNode/microReticulum")
get_filename_component(TD_RNS_DIR "${TD_RNS_DIR}" ABSOLUTE)
set(TD_RNS_DEPS "${CMAKE_CURRENT_LIST_DIR}/../../../../../../TinyNode/nano-node/.pio/libdeps/nano-esp32")
get_filename_component(TD_RNS_DEPS "${TD_RNS_DEPS}" ABSOLUTE)

set(TD_LXMF_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../../../TinyNode/microLXMF"
    CACHE PATH "microLXMF checkout")
get_filename_component(TD_LXMF_DIR "${TD_LXMF_DIR}" ABSOLUTE)
if(NOT EXISTS "${TD_LXMF_DIR}/src/LXMF/LXMRouter.h")
    message(FATAL_ERROR "microLXMF not found at TD_LXMF_DIR='${TD_LXMF_DIR}'")
endif()
file(GLOB_RECURSE TD_LXMF_SRC "${TD_LXMF_DIR}/src/LXMF/*.cpp")
# ArduinoJson is header-only for the parts microReticulum uses; the Crypto
# library is not -- its .cpp files have to be compiled in.
# The UDP interface lives in the microReticulum examples, not src/. Compiled
# here with ARDUINO undefined so it takes its POSIX socket path (lwIP serves
# it) and leaves WiFi to main/node.cpp.
# The ESP32 AES is selected at runtime from the ROM; the portable one is used.

idf_component_register(
    SRCS ${TD_LXMF_SRC}
    INCLUDE_DIRS
        "../rns/shim"
        "${TD_LXMF_DIR}/src"
        "${TD_RNS_DEPS}/ArduinoJson/src"
        "${TD_RNS_DIR}/../MsgPack-vendored"
    REQUIRES esp_hw_support esp_timer esp_rom rns
)

# third-party code: keep the warnings out of the build log, but keep the
# exception support microReticulum needs.
target_compile_options(${COMPONENT_LIB} PRIVATE -w -std=gnu++17 -fexceptions)
# microLXMF is Arduino code too, so it gets the same shim as microReticulum.
set_source_files_properties(${TD_LXMF_SRC} PROPERTIES
    COMPILE_OPTIONS "-include;${CMAKE_CURRENT_LIST_DIR}/../rns/shim/arduino_shim.h")

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    RNS_LOG_LEVEL=RNS_LOG_LEVEL_WARNING
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    MSGPACK_USE_BOOST=OFF
    RNS_USE_FS
    USTORE_USE_POSIXFS
)

# microReticulum includes MsgPack.h and reaches for its own dependencies by
# bare name, so the include paths above have to be transitive for anything
# that includes Reticulum.h from outside this component.
target_include_directories(${COMPONENT_LIB} PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/../rns/shim"
    "${TD_LXMF_DIR}/src"
)
