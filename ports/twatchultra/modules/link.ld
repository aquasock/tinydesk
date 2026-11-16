/*
 * Linker script for a module: everything at the address esp_partition_mmap()
 * hands back for the `modules` partition.
 *
 * 0x42ae0000 is MEASURED, not assumed -- `modtest` on the watch reports it, and
 * it must be re-measured if the partition table or the flash layout changes.
 * It is in the ESP32-S3's instruction address window (0x42000000..0x43ffffff),
 * which is what makes the blob fetchable rather than merely readable.
 *
 * The header goes first so a loader can read the magic at offset 0 without
 * knowing anything about the blob's layout.
 */
ENTRY(mod_entry)

SECTIONS
{
    . = 0x42ae0000;

    .modheader : ALIGN(4)
    {
        KEEP(*(.modheader))
    }

    .text : ALIGN(4)
    {
        *(.literal .literal.* .text .text.*)
    }

    .rodata : ALIGN(4)
    {
        *(.rodata .rodata.*)
    }

    /*
     * .DATA IS DISCARDED; .BSS GOES TO THE FIRMWARE.
     *
     * A blob executes from read-only flash, so there is nowhere for initialised
     * data to live and no loader to copy it -- .data is discarded, and a module
     * that needs initialised state must construct it at run time.
     *
     * .bss is different: real library code carries function-local statics
     * (micro-flac's get_metadata_blocks() holds a static empty vector, 20 bytes
     * plus a guard), and those are zero-initialised by definition. The firmware
     * lends a small region for them -- `g_modbss`, published at a build-derived
     * address like the API table -- and modload_zero_bss() clears it before the
     * first call. That is the loader step a blob cannot perform for itself.
     */
    .bss MODBSS_AT (NOLOAD) : ALIGN(16)
    {
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(COMMON)
    }

    /DISCARD/ :
    {
        *(.data .data.*)
        *(.comment)
        *(.xt.lit .xt.prop .xtensa.info)
    }
}
