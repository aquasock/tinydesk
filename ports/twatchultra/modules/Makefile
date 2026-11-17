# Build a module blob for the `modules` partition.
#
#   make            build modtest.bin and report its layout
#   make flash      write it to 0xE00000 and verify
#
# The address must match what `modtest` reports on the watch; see link.ld.

MODULES := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
MAIN    := $(abspath $(MODULES)/../main)

CROSS   ?= xtensa-esp32s3-elf-
CC      := $(CROSS)gcc
CXX     := $(CROSS)g++
OBJCOPY := $(CROSS)objcopy
NM      := $(CROSS)nm
SIZE    := $(CROSS)size

BUILD   ?= $(MODULES)/build
PORT    := $(abspath $(MODULES)/..)
APP_ELF := $(PORT)/build/tinydesk.elf

# Addresses come from a GENERATED HEADER, not -D flags.
#
# A -D does not make an object depend on its VALUE: when the app was rebuilt and
# the table moved 0x3c27b8fc -> 0x3c27ba74, make reused the stale flacmod.o, the
# blob linked against the old address, and the module's host() read garbage --
# surfacing only as a verdict of -1. modaddr.h is a prerequisite of every module
# object, so a moved address forces a rebuild.
MODADDR_H := $(BUILD)/modaddr.h

# For `make imu`, which flashes with esptool: the interpreter that has it.
IDF_PY ?= /home/vash/esp/idf-tools/python_env/idf5.3_py3.14_env/bin/python
PORT   ?= /dev/ttyACM1

# The linker needs these as --defsym symbols too. They are read from nm as well
# as the header: the header is what makes OBJECTS rebuild when an address moves,
# and these are what the LINK consumes.
MODBSS_AT      := $(shell $(NM) $(APP_ELF) 2>/dev/null | awk '$$3=="g_modbss"{print "0x"$$1}')

# WHERE THE MMU PUT THE TWO PARTITIONS. Both are runtime-determined, both move
# when the app image changes size, and moddata.at is the single place they are
# recorded -- copied from the watch's /root/modaddr.txt, which modload_prepare()
# rewrites at every boot.
#
# Measured drift, and the reason these are not constants: adding the inline FLAC
# decoder gave 0x42ae0000/0x3cbe0000, and removing it gave
# 0x42ad0000/0x3cbd0000. A blob linked for the first pair still EXECUTED against
# the second and read its .rodata 64 KB off, which reached the user as "bad
# magic number" on two files that were perfectly good.
MODTEXT_AT     := $(shell sed -n 's/^inst=//p' $(MODULES)/moddata.at 2>/dev/null)
MODDATA_AT     := $(shell sed -n 's/^data=//p' $(MODULES)/moddata.at 2>/dev/null)
# The API table's address, which is the third thing that moves when the app is
# rebuilt. Recorded in the blob header so the loader can refuse a stale one.


MF      := $(abspath $(MODULES)/../components/micro-flac)
XTINC   := $(IDF_PATH)/components/xtensa/esp32s3/include

# micro-flac's decoder, WITHOUT the encoder (which is 76 KB of the archive and
# nobody calls it) and without the Ogg container path. These five sources plus
# the Xtensa LPC assembly measured 26,257 bytes of text and 20 bytes of bss.
FLAC_SRC := $(MF)/src/crc.cpp $(MF)/src/frame_header.cpp \
            $(MF)/src/pcm_packing.cpp $(MF)/src/flac_decoder.cpp \
            $(MF)/src/decorrelation.cpp $(MF)/src/lpc.cpp
FLAC_ASM := $(MF)/src/xtensa/lpc_32_xtensa.S $(MF)/src/xtensa/lpc_64_xtensa.S

# FLAC_MALLOC/FLAC_FREE are micro-flac's documented override point. Inside a
# module the default (heap_caps_malloc_prefer) is an external symbol the blob
# cannot resolve, so both are redirected to the host API table via the shim.
FLAC_DEFS := -DFLAC_MALLOC=modhost_malloc -DFLAC_FREE=modhost_free \
             -DMICRO_FLAC_DISABLE_OGG=1

SRC     := $(MODULES)/src/modtest.c
ELF     := $(BUILD)/modtest.elf
BIN     := $(BUILD)/modtest.bin

# -mlongcalls is the ESP32-S3 default and the blob NEEDS it for the same reason
# the app does: a direct call cannot reach 0x42ae0000 from everywhere.
# -fno-pic keeps the literals absolute, which is correct here because the load
# address is fixed. -nostdlib and -ffreestanding keep libc out, because a blob
# cannot resolve an external symbol.
# HOSTED, not -ffreestanding: micro-flac's decoder includes <vector>, and the
# libstdc++ headers refuse freestanding mode outright. Freestanding-ness is
# enforced at LINK time (-nostdlib) and by the shim, which is where it belongs.
# Flags common to C and C++, and the C++-only ones kept apart: gcc rejects
# -std=gnu++17, -fno-rtti and -fno-threadsafe-statics on C input.
CFLAGS := -mlongcalls -mtext-section-literals -fno-builtin -fno-pic \
          -Os -Wall -Wextra -Werror \
          -include $(MODADDR_H) \
          -I$(MAIN)

CXXFLAGS := $(CFLAGS) -fno-exceptions -fno-rtti -fno-threadsafe-statics -std=gnu++17

FLAC_CFLAGS  := $(CFLAGS) -I$(MF)/include -I$(MF)/src -I$(XTINC)
FLAC_CXXFLAGS := $(FLAC_CFLAGS) -fno-exceptions -fno-rtti -fno-threadsafe-statics -std=gnu++17 $(FLAC_DEFS) \
                 -include $(MODULES)/src/flacmalloc.h

LDFLAGS := -nostdlib -T $(MODULES)/link.ld -Wl,--build-id=none \
           -Wl,--defsym=MODTEXT_AT=$(MODTEXT_AT) -Wl,--defsym=MODBSS_AT=$(MODBSS_AT) \
           -Wl,--defsym=MODDATA_AT=$(MODDATA_AT) \
           -Wl,-Map=$(BUILD)/mod.map

.PHONY: all clean flash size check flac modaddr

all: $(BIN)

# The FLAC decoder as a module.
flac: $(BUILD)/flac.bin

$(BUILD)/flacmod.o: $(MODULES)/src/flacmod.cpp $(MODADDR_H) | $(BUILD)
	$(CXX) $(FLAC_CXXFLAGS) -I$(MAIN) -c $< -o $@

$(BUILD)/flac_%.o: $(MF)/src/%.cpp $(MODADDR_H) | $(BUILD)
	$(CXX) $(FLAC_CXXFLAGS) -c $< -o $@

# Both LPC widths: the decoder references the 64-bit path whether or not a
# given file needs it, because 32-bit joint stereo can widen to int64.
$(BUILD)/lpc32.o: $(MF)/src/xtensa/lpc_32_xtensa.S | $(BUILD)
	$(CC) $(FLAC_CFLAGS) -c $< -o $@

$(BUILD)/lpc64.o: $(MF)/src/xtensa/lpc_64_xtensa.S | $(BUILD)
	$(CC) $(FLAC_CFLAGS) -c $< -o $@

# moddata.at IS A PREREQUISITE, and it was missing. The two addresses are passed
# to the link as --defsym values read at Makefile-parse time, so a changed file
# only relinks if make knows the blob depends on it -- without this, `make modaddr
# && make flac` updates the file, prints "gen_addr: unchanged", and silently
# ships the blob linked for the OLD address. That is the same class of stale-
# address failure the generated header was built to prevent, one layer down.
$(BUILD)/flac.bin: $(BUILD)/flacmod.o $(BUILD)/modshim.o \
                   $(patsubst $(MF)/src/%.cpp,$(BUILD)/flac_%.o,$(FLAC_SRC)) \
                   $(BUILD)/lpc32.o $(BUILD)/lpc64.o $(MODULES)/link.ld \
                   $(MODULES)/moddata.at
	$(CXX) $(LDFLAGS) -o $(BUILD)/flac.elf $(filter %.o,$^) -lgcc
	# TWO BINARIES, because .text and .rodata live in different windows and
	# therefore different partitions: the code goes to `modules`, the constants
	# to `moddata`. --only-section keeps them from bleeding into each other.
	$(OBJCOPY) -O binary --only-section=.modheader --only-section=.text $(BUILD)/flac.elf $@
	$(OBJCOPY) -O binary --only-section=.rodata $(BUILD)/flac.elf $(BUILD)/flac.data.bin
	@$(NM) $(BUILD)/flac.elf > $(BUILD)/flac.nm
	@python3 $(MODULES)/patch_header.py $@ "$$(cat $(BUILD)/flac.nm)" $(MODTEXT_AT) $(MODDATA_AT) $(MODADDR_H)
	@printf 'blob: %s (%s bytes) + data %s (%s bytes)\n' "$@" "$$(stat -c %s $@)" \
		"$(BUILD)/flac.data.bin" "$$(stat -c %s $(BUILD)/flac.data.bin)"

# Re-read the two mapping addresses off the board and rewrite moddata.at.
#
# Needed after ANY change that alters the app image size: both mmap addresses move
# when it does, and a blob linked for the old pair still executes while reading
# its constants from the wrong place. A couple of minutes, and it cannot be
# silently wrong the way a typed address can.
modaddr:
	@python3 $(MODULES)/read_modaddr.py -o $(MODULES)/moddata.at

# Fail before compiling anything if the app has not been built, or if its
# table symbol moved out of the image.
check:
	@test -f $(APP_ELF) || { echo "modules: build the app first ($(APP_ELF) missing)"; exit 1; }
	@test -n "$(MODDATA_AT)" && test -n "$(MODTEXT_AT)" || { echo "modules: moddata.at missing -- it needs 'inst=..' and 'data=..' copied from the watch's /root/modaddr.txt"; exit 1; }
	@test -f $(MODULES)/moddata.at || { echo "modules: moddata.at gone"; exit 1; }

$(BUILD):
	mkdir -p $(BUILD)

# The shim is C++ (operator new/delete), the entry is C.
$(MODADDR_H): $(APP_ELF) $(MODULES)/gen_addr.py
	@mkdir -p $(BUILD)
	@python3 $(MODULES)/gen_addr.py $(APP_ELF) $@

$(BUILD)/modtest.o: $(SRC) $(MODADDR_H) | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/modshim.o: $(MODULES)/src/modshim.c $(MODADDR_H) | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(ELF): $(BUILD)/modtest.o $(BUILD)/modshim.o $(MODULES)/link.ld
	$(CXX) $(LDFLAGS) -o $@ $(BUILD)/modtest.o $(BUILD)/modshim.o -lgcc

# --only-section, for the same reason the FLAC blob needs it: .rodata is linked
# into the DATA window at a LOWER address than .text, so a plain `objcopy -O
# binary` starts the file at .rodata and the header is not at offset 0. The
# symptom was "patch_header: bad magic 0x75646f6d" -- the bytes of the word
# "module" from a string literal.
$(BIN): $(ELF)
	$(OBJCOPY) -O binary --only-section=.modheader --only-section=.text $< $@
	$(OBJCOPY) -O binary --only-section=.rodata $< $(BUILD)/modtest.data.bin
	@$(NM) $< > $(BUILD)/modtest.nm
	@python3 $(MODULES)/patch_header.py $@ "$$(cat $(BUILD)/modtest.nm)" $(MODTEXT_AT) $(MODDATA_AT) $(MODADDR_H)
	@printf 'blob: %s (%s bytes)\n' "$@" "$$(stat -c %s $@)"
	@$(NM) $< | grep -E ' (mod_header|mod_entry)$$' || true

size: $(ELF)
	$(SIZE) $(ELF)

clean:
	rm -rf $(BUILD)

# --------------------------------------------------------------- the IMU
#
# The BHI260AP's firmware is not in the app image: it is a blob in `moddata` at
# 0x10000, streamed into the part on first use. Flashing it is a separate step
# from `make flac`, and deliberately so -- they share that partition without
# overlapping (the codec's .rodata is at offset 0, 1,976 bytes; this is 64 KB in).
#
# WHY 0x10000 AND NOT 0x2000: the codec blob is written by objcopy at the start
# of the partition. Leaving 64 KB of gap means `make flac` and `make imu` can be
# flashed in either order and neither clobbers the other. Do not move either
# without checking the other.
IMU_IMG   := $(MODULES)/../components/imu/bhi260_klio.img
IMU_AT    := 0xE10000

.PHONY: imu imu-image
imu-image:
	@ls -la $(IMU_IMG)

# Read the address out of the partition table rather than trusting this file:
#   esptool ... read_flash 0x8000 0x1000 pt.bin && gen_esp32part.py pt.bin
imu: $(IMU_IMG)
	@printf 'IMU firmware %s -> 0x%X (moddata + 0x10000)\n' "$(IMU_IMG)" $(IMU_AT)
	$(IDF_PY) -m esptool --chip esp32s3 -p $(PORT) -b 921600 \
	    --before default_reset --after hard_reset write_flash $(IMU_AT) $(IMU_IMG)
