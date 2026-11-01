# microReticulum as an ESP-IDF component.
#
# microReticulum is an Arduino library, so this wraps it with the small shim in
# shim/ (see the comment at the top of arduino_shim.h) rather than requiring
# the Arduino framework. It was proven to compile and link against ESP-IDF
# 5.3.1 with no Arduino present before this file existed.
#
# The library and its dependencies are NOT vendored here yet. Point these at
# the trees they live in:
#
#   TD_RNS_DIR    the microReticulum checkout
#   TD_RNS_DEPS   a directory holding its Arduino dependencies, which is what
#                 PlatformIO resolves into nano-node/.pio/libdeps/<env>/
#                 (ArduinoJson, microStore, DebugLog, ArxContainer,
#                 ArxTypeTraits, Crypto)
#
# Vendoring them alongside microReticulum is the next step: a build that
# depends on another project's .pio directory is not reproducible.

set(TD_RNS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../../../TinyNode/microReticulum"
    CACHE PATH "microReticulum checkout")
set(TD_RNS_DEPS "${CMAKE_CURRENT_LIST_DIR}/../../../../../../TinyNode/nano-node/.pio/libdeps/nano-esp32"
    CACHE PATH "microReticulum Arduino dependencies (ArduinoJson, microStore, ...)")

get_filename_component(TD_RNS_DIR "${TD_RNS_DIR}" ABSOLUTE)
get_filename_component(TD_RNS_DEPS "${TD_RNS_DEPS}" ABSOLUTE)

if(NOT EXISTS "${TD_RNS_DIR}/src/microReticulum/Reticulum.h")
    message(FATAL_ERROR
        "microReticulum not found at TD_RNS_DIR='${TD_RNS_DIR}'. "
        "Pass -DTD_RNS_DIR=/path/to/microReticulum.")
endif()
if(NOT EXISTS "${TD_RNS_DEPS}/ArduinoJson/src/ArduinoJson.h")
    message(FATAL_ERROR
        "microReticulum dependencies not found at TD_RNS_DEPS='${TD_RNS_DEPS}'. "
        "Build the Arduino project once so PlatformIO resolves them, or pass "
        "-DTD_RNS_DEPS=/path/to/libdeps.")
endif()

file(GLOB_RECURSE TD_RNS_SRC "${TD_RNS_DIR}/src/microReticulum/*.cpp")
# ArduinoJson is header-only for the parts microReticulum uses; the Crypto
# library is not -- its .cpp files have to be compiled in.
file(GLOB TD_CRYPTO_SRC "${TD_RNS_DEPS}/Crypto/*.cpp")
# AESEsp32.cpp is Arduino-specific hardware glue; the portable AES is used.
list(FILTER TD_CRYPTO_SRC EXCLUDE REGEX "AESEsp32")
# The UDP interface lives in the microReticulum examples, not src/. Compiled
# here with ARDUINO undefined so it takes its POSIX socket path (lwIP serves
# it) and leaves WiFi to main/node.cpp.
set(TD_UDP_DIR "${TD_RNS_DIR}/examples/common/udp_interface")
# The ESP32 AES is selected at runtime from the ROM; the portable one is used.
list(FILTER TD_JSON_SRC EXCLUDE REGEX ".*")

idf_component_register(
    SRCS ${TD_RNS_SRC} ${TD_CRYPTO_SRC} "nanosleep.c" "${TD_UDP_DIR}/UDPInterface.cpp"
    INCLUDE_DIRS
        "shim"
        "include"
        "${TD_UDP_DIR}"
        "${TD_RNS_DIR}/src"
        "${TD_RNS_DIR}/src/microReticulum"
        "${TD_RNS_DEPS}/ArduinoJson/src"
        "${TD_RNS_DEPS}/microStore/include"
        "${TD_RNS_DEPS}/microStore/src"
        "${TD_RNS_DEPS}/DebugLog"
        "${TD_RNS_DEPS}/ArxContainer"
        "${TD_RNS_DEPS}/ArxTypeTraits"
        "${TD_RNS_DEPS}/ArxTypeTraits/ArxTypeTraits"
        "${TD_RNS_DEPS}/Crypto"
        "${TD_RNS_DEPS}/Crypto/utility"
        "${TD_RNS_DIR}/../MsgPack-vendored"
    REQUIRES esp_hw_support esp_timer esp_rom
)

# third-party code: keep the warnings out of the build log, but keep the
# exception support microReticulum needs.
target_compile_options(${COMPONENT_LIB} PRIVATE -w -std=gnu++17 -fexceptions)

# The shim is C++ (it declares a class), so it is force-included into the C++
# sources only. nanosleep.c is plain C and must not see it.
set(TD_RNS_CPP_SRC ${TD_RNS_SRC} ${TD_CRYPTO_SRC} "${TD_UDP_DIR}/UDPInterface.cpp")
set_source_files_properties(${TD_RNS_CPP_SRC} PROPERTIES
    COMPILE_OPTIONS "-include;${CMAKE_CURRENT_LIST_DIR}/shim/arduino_shim.h")

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    RNS_LOG_LEVEL=RNS_LOG_LEVEL_WARNING
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    MSGPACK_USE_BOOST=OFF
    RNS_USE_FS
    USTORE_USE_POSIXFS
    USTORE_USE_STDIOFS
    # The file-backed known-destination store does not retain here: announces
    # validate and enter the path table, but Identity::recall() never returns
    # the key again, so every reply fails with "identity not known". With this
    # at 0 the library picks its BasicHeapStore instead and recall works. The
    # table is a few dozen small entries, so RAM costs little and nothing is
    # written to flash on every announce. Path persistence is unaffected.
    RNS_PERSIST_KNOWN_DESTINATIONS=0
)

# microReticulum includes MsgPack.h and reaches for its own dependencies by
# bare name, so the include paths above have to be transitive for anything
# that includes Reticulum.h from outside this component.
target_include_directories(${COMPONENT_LIB} PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}/shim"
    "${TD_RNS_DIR}/src"
    "${TD_RNS_DIR}/src/microReticulum"
)
