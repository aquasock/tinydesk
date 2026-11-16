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

LDFLAGS := -nostdlib -T $(MODULES)/link.ld -Wl,--build-id=none -Wl,-Map=$(BUILD)/modtest.map

.PHONY: all clean flash size check

all: $(BIN)

# Fail before compiling anything if the app has not been built, or if its
# table symbol moved out of the image.
check:
	@test -f $(APP_ELF) || { echo "modules: build the app first ($(APP_ELF) missing)"; exit 1; }
	@test -n "$(MODHOST_PTR_AT)" || { echo "modules: g_modhost_ptr not in $(APP_ELF)"; exit 1; }
	@echo "modules: host API table at $(MODHOST_PTR_AT)"

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
