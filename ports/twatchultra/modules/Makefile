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
OBJCOPY := $(CROSS)objcopy
NM      := $(CROSS)nm
SIZE    := $(CROSS)size

BUILD   ?= $(MODULES)/build
SRC     := $(MODULES)/src/modtest.c
ELF     := $(BUILD)/modtest.elf
BIN     := $(BUILD)/modtest.bin

# -mlongcalls is the ESP32-S3 default and the blob NEEDS it for the same reason
# the app does: a direct call cannot reach 0x42ae0000 from everywhere.
# -fno-pic keeps the literals absolute, which is correct here because the load
# address is fixed. -nostdlib and -ffreestanding keep libc out, because a blob
# cannot resolve an external symbol.
CFLAGS := -mlongcalls -mtext-section-literals -ffreestanding -fno-builtin \
          -fno-pic -mno-target-align -Os -Wall -Wextra -Werror \
          -I$(MAIN)

LDFLAGS := -nostdlib -T $(MODULES)/link.ld -Wl,--build-id=none -Wl,-Map=$(BUILD)/modtest.map

.PHONY: all clean flash size

all: $(BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(ELF): $(SRC) $(MODULES)/link.ld | $(BUILD)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SRC)

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
