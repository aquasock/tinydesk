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

    /* A blob has no data or bss: those would need a loader to copy and zero
     * them, and the whole point is that loading costs nothing. A blob that
     * needs writable state must ask its caller for a buffer. */
    /DISCARD/ :
    {
        *(.data .data.*)
        *(.bss .bss.*)
        *(.comment)
        *(.xt.lit .xt.prop .xtensa.info)
    }
}
