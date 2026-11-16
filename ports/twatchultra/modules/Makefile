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

# The address of the firmware's published API-table pointer, read out of the
# app's own ELF. DERIVED, never typed: a stale address here links a blob that
# dereferences garbage the first time it allocates.
MODHOST_PTR_AT := $(shell $(NM) $(APP_ELF) 2>/dev/null | awk '$$3=="g_modhost_ptr"{print "0x"$$1}')

# Where the firmware lends modules their .bss. Derived the same way, and for the
# same reason: a stale address here would have the module zeroing the wrong RAM.
MODBSS_AT      := $(shell $(NM) $(APP_ELF) 2>/dev/null | awk '$$3=="g_modbss"{print "0x"$$1}')

# Where the MMU mapped the `moddata` partition, as the firmware published it in
# /root/modaddr.txt at boot. Absolute and runtime-determined, so it is read from
# that file rather than calculated. Re-run `make modaddr` after any change that
# moves a mapping.
MODDATA_AT     := $(shell sed -n 's/^data=//p' $(MODULES)/moddata.at 2>/dev/null)

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
          -DMODHOST_PTR_AT=$(MODHOST_PTR_AT) \
          -I$(MAIN)

CXXFLAGS := $(CFLAGS) -fno-exceptions -fno-rtti -fno-threadsafe-statics -std=gnu++17

FLAC_CFLAGS  := $(CFLAGS) -I$(MF)/include -I$(MF)/src -I$(XTINC)
FLAC_CXXFLAGS := $(FLAC_CFLAGS) -fno-exceptions -fno-rtti -fno-threadsafe-statics -std=gnu++17 $(FLAC_DEFS) \
                 -include $(MODULES)/src/flacmalloc.h

LDFLAGS := -nostdlib -T $(MODULES)/link.ld -Wl,--build-id=none \
           -Wl,--defsym=MODBSS_AT=$(MODBSS_AT) -Wl,--defsym=MODDATA_AT=$(MODDATA_AT) \
           -Wl,-Map=$(BUILD)/mod.map

.PHONY: all clean flash size check flac

all: $(BIN)

# The FLAC decoder as a module.
flac: $(BUILD)/flac.bin

$(BUILD)/flacmod.o: $(MODULES)/src/flacmod.cpp | $(BUILD) check
	$(CXX) $(FLAC_CXXFLAGS) -I$(MAIN) -c $< -o $@

$(BUILD)/flac_%.o: $(MF)/src/%.cpp | $(BUILD)
	$(CXX) $(FLAC_CXXFLAGS) -c $< -o $@

# Both LPC widths: the decoder references the 64-bit path whether or not a
# given file needs it, because 32-bit joint stereo can widen to int64.
$(BUILD)/lpc32.o: $(MF)/src/xtensa/lpc_32_xtensa.S | $(BUILD)
	$(CC) $(FLAC_CFLAGS) -c $< -o $@

$(BUILD)/lpc64.o: $(MF)/src/xtensa/lpc_64_xtensa.S | $(BUILD)
	$(CC) $(FLAC_CFLAGS) -c $< -o $@

$(BUILD)/flac.bin: $(BUILD)/flacmod.o $(BUILD)/modshim.o \
                   $(patsubst $(MF)/src/%.cpp,$(BUILD)/flac_%.o,$(FLAC_SRC)) \
                   $(BUILD)/lpc32.o $(BUILD)/lpc64.o $(MODULES)/link.ld
	$(CXX) $(LDFLAGS) -o $(BUILD)/flac.elf $(filter %.o,$^) -lgcc
	# TWO BINARIES, because .text and .rodata live in different windows and
	# therefore different partitions: the code goes to `modules`, the constants
	# to `moddata`. --only-section keeps them from bleeding into each other.
	$(OBJCOPY) -O binary --only-section=.modheader --only-section=.text $(BUILD)/flac.elf $@
	$(OBJCOPY) -O binary --only-section=.rodata $(BUILD)/flac.elf $(BUILD)/flac.data.bin
	@$(NM) $(BUILD)/flac.elf > $(BUILD)/flac.nm
	@python3 $(MODULES)/patch_header.py $@ "$$(cat $(BUILD)/flac.nm)"
	@printf 'blob: %s (%s bytes) + data %s (%s bytes)\n' "$@" "$$(stat -c %s $@)" \
		"$(BUILD)/flac.data.bin" "$$(stat -c %s $(BUILD)/flac.data.bin)"

# Fail before compiling anything if the app has not been built, or if its
# table symbol moved out of the image.
check:
	@test -f $(APP_ELF) || { echo "modules: build the app first ($(APP_ELF) missing)"; exit 1; }
	@test -n "$(MODHOST_PTR_AT)" || { echo "modules: g_modhost_ptr not in $(APP_ELF)"; exit 1; }
	@test -n "$(MODBSS_AT)" || { echo "modules: g_modbss not in $(APP_ELF)"; exit 1; }
	@test -n "$(MODDATA_AT)" || { echo "modules: moddata.at missing -- create it with 'inst=..' and 'data=..' lines from the watch's /root/modaddr.txt"; exit 1; }
	@echo "modules: table $(MODHOST_PTR_AT), bss $(MODBSS_AT), rodata $(MODDATA_AT)"

$(BUILD):
	mkdir -p $(BUILD)

# The shim is C++ (operator new/delete), the entry is C.
$(BUILD)/modtest.o: $(SRC) | $(BUILD) check
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/modshim.o: $(MODULES)/src/modshim.c | $(BUILD) check
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(ELF): $(BUILD)/modtest.o $(BUILD)/modshim.o $(MODULES)/link.ld
	$(CXX) $(LDFLAGS) -o $@ $(BUILD)/modtest.o $(BUILD)/modshim.o -lgcc

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@
	@$(NM) $< > $(BUILD)/modtest.nm
	@python3 $(MODULES)/patch_header.py $@ "$$(cat $(BUILD)/modtest.nm)"
	@printf 'blob: %s (%s bytes)\n' "$@" "$$(stat -c %s $@)"
	@$(NM) $< | grep -E ' (mod_header|mod_entry)$$' || true

size: $(ELF)
	$(SIZE) $(ELF)

clean:
	rm -rf $(BUILD)
