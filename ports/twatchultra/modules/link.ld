/*
 * Linker script for a module: everything at the address esp_partition_mmap()
 * hands back for the `modules` partition.
 *
 * MODTEXT_AT AND MODDATA_AT ARE BOTH MEASURED, never assumed. The firmware
 * maps both partitions at boot and writes what the MMU gave it to
 * /root/modaddr.txt; modules/moddata.at holds that answer and the Makefile
 * passes both here as --defsym. They are NOT stable across app rebuilds: the
 * pairing 0x42ae0000/0x3cbe0000 became 0x42ad0000/0x3cbd0000 when the app image
 * changed size, and a blob linked for the old pair read its .rodata 64 KB off,
 * which surfaced only as "bad magic number" from the decoder.
 * It is in the ESP32-S3's instruction address window (0x42000000..0x43ffffff),
 * which is what makes the blob fetchable rather than merely readable.
 *
 * The header goes first so a loader can read the magic at offset 0 without
 * knowing anything about the blob's layout.
 */
ENTRY(mod_entry)

SECTIONS
{
    . = MODTEXT_AT;

    .modheader : ALIGN(4)
    {
        KEEP(*(.modheader))
    }

    .text : ALIGN(4)
    {
        *(.text .text.*)
    }

    /*
     * .RODATA GOES TO A DIFFERENT WINDOW.
     *
     * The instruction window serves fetches and nothing else -- a data load from
     * it faults with exccause 3 (measured at 0x42ae6ff0, inside this very
     * section, when the decoder read its CRC table). So constant data is linked
     * into the `moddata` partition, which is mapped as DATA, and MODDATA_AT is
     * whatever address the MMU handed back for it -- published by the firmware in
     * /root/modaddr.txt and read by the build.
     */
    .rodata MODDATA_AT : ALIGN(4)
    {
        *(.rodata .rodata.*)
        *(.literal .literal.*)
    }

    /* .text stays in the instruction window, which is what `modules` maps. */
    /DISCARD/ : { *(.comment) *(.xt.lit .xt.prop .xtensa.info) }

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
    }
}
