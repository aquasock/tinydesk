/*
 * modload.h - load and call code from the `modules` flash partition.
 *
 * THE IDEA. Xtensa executes from flash natively: every instruction ESP-IDF runs
 * is already XIP-mapped through the MMU. So a blob in a raw flash partition can
 * be mapped as INSTRUCTIONS with esp_partition_mmap() and called directly, and
 * loading it costs no internal SRAM -- which is the pool this board is short of
 * (about 18 KB free with the desktop running).
 *
 * WHY IT MATTERS. The app image has ~27 KB of headroom in ota_1 and the FLAC
 * decoder already cost 31.7 KB of it, so a second codec cannot be added inline
 * at all. A 2 MB `modules` partition exists at 0xE00000 for exactly this.
 *
 * WHAT IS UNPROVEN, AND WHAT THIS FILE IS FOR. Two things have to be true
 * before a codec can live there:
 *
 *   1. esp_partition_mmap() must hand back an address this CPU can execute from.
 *   2. Code linked for that address must be callable across the Xtensa windowed
 *      ABI -- the caller's window rotation, the blob's own literals, and any
 *      call the blob makes must all survive crossing a partition boundary.
 *
 * Both are answered by one small experiment, and it must be cheap: if either is
 * false the whole design is dead and the partition should be given back to
 * `storage`. So this header starts with a self-test that maps the partition,
 * reports the address, and calls a blob's entry point if one is present.
 *
 * THE ADDRESS PROBLEM. A blob linked with absolute literals (which is what
 * -mlongcalls produces, and what this toolchain does by default) needs a KNOWN
 * load address. The mmap address is not documented as fixed, so it is MEASURED
 * first and the blob is linked for what was measured. That makes the address a
 * build-time constant that must be re-measured whenever the partition table
 * moves -- recorded here rather than assumed.
 */
#ifndef TWATCH_MODLOAD_H
#define TWATCH_MODLOAD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Blob header, at offset 0 of the partition. A blob that does not start with
 * this magic is not one, so the self-test can tell "empty partition" from
 * "something is here and it will not load".
 */
#define MODLOAD_MAGIC   0x4D4F444CU     /* "MODL", little-endian */

typedef struct
{
    uint32_t magic;
    uint32_t abi_version;       /* must equal MODLOAD_ABI_VERSION */
    uint32_t size;              /* bytes the blob occupies, for bounds checks */
    uint32_t entry_offset;      /* from the start of the mapped region */
    char name[24];
} modload_header_t;

/* Bump when the calling convention or the header changes. A blob built for a
 * different version is refused rather than called. */
#define MODLOAD_ABI_VERSION 1

/*
 * The blob's entry point. Plain int32_t in and out on purpose: a struct or a
 * float would make the test partly a test of argument passing, and the first
 * question is only whether control transfers at all.
 *
 * `heap_free`/`heap_largest` are passed in so the blob can be checked for
 * actually seeing this board's memory rather than a stale or bogus view.
 */
typedef int32_t (*modload_entry_t)(int32_t a, int32_t b, uint32_t heap_free,
                                   uint32_t heap_largest);

/*
 * A RESULT RECORD, written to a file rather than to the partition.
 *
 * The display mirror cannot be trusted to report a one-line answer (it sends
 * only CHANGED cells, and a restored window holds focus at boot). Writing into
 * `modules` itself worked but consumed the last 4 KB sector of the partition
 * and needed an unmap first, because esp_partition_write() refuses a mapped
 * region. A file on littlefs has neither problem, and the PC can extract the
 * partition and mount it with littlefs-python.
 *
 * NOTE the path: littlefs is mounted at "/fs" in the POSIX VFS
 * (TDSH_MOUNT_POINT) and the SHELL presents that as "/". Ordinary C sees
 * /fs/root/... . A path of "/root/..." silently fails.
 */
#define MODLOAD_RESULT_MAGIC    0x524C5453U             /* "STLR" */

typedef struct
{
    uint32_t magic;
    uint32_t data_base;         /* where the bytes are readable */
    uint32_t inst_base;         /* where the bytes are executable */
    uint32_t entry;             /* inst_base + entry_offset */
    uint32_t ret;               /* what the blob returned */
    uint32_t heap_free;
    uint32_t heap_largest;
    uint32_t called;            /* 1 when the entry point was invoked */
} modload_result_t;

/* Map the modules partition as instructions. Returns false with a reason in
 * `why` when the partition is missing or mmap fails. */
bool modload_map(const void **base, size_t *size, char *why, size_t why_len);

/* Undo modload_map(). */
void modload_unmap(void);

/*
 * Read the blob header through the DATA path.
 *
 * This is not a convenience: the mapped pointer is in the INSTRUCTION window,
 * where a data load faults (coredump, LoadStoreErrorCause at 0x42ae0014). The
 * header must come from esp_partition_read().
 */
bool modload_read_header(modload_header_t *h);

/*
 * Map the partition and read the blob header ONCE, from a task whose stack is in
 * INTERNAL RAM.
 *
 * THIS IS NOT AN OPTIMISATION, IT IS A HARD REQUIREMENT, and the coredump that
 * taught it is worth keeping: esp_partition_mmap() calls
 * spi_flash_mmap(), which disables the flash cache, and PSRAM is reached THROUGH
 * that cache. A task running on a PSRAM stack therefore cannot map flash at all:
 *
 *   assert failed: spi_flash_disable_interrupts_caches_and_other_cpu
 *   (esp_task_stack_is_sane_cache_disabled())
 *     spi_flash_mmap <- esp_partition_mmap <- modload_map (task 'modflac')
 *
 * Every task in this port that touches USB or audio runs on a PSRAM stack to
 * save internal RAM, so none of them may call modload_map() first. Call this at
 * boot instead; afterwards modload_inst_base() and modload_header() answer from
 * what was cached here.
 */
bool modload_prepare(char *why, size_t why_len);

/*
 * True when modload_prepare() succeeded. For tasks OTHER than the caller of
 * modload_prepare(): they must never map, only ask.
 *
 * This exists because the distinction was not structural and cost a coredump:
 * modflac called modload_prepare(), the boot-time call had failed (no module
 * flashed yet), so it retried the mapping from a PSRAM-stack task and hit
 * "assert failed: esp_task_stack_is_sane_cache_disabled()". A retry that can
 * panic is not a retry.
 */
bool modload_ready(void);

/* The label and subtype of the data-side partition. A module's `.rodata` is
 * linked at whatever address this partition maps to; see modules/link.ld. */
#define MODLOAD_DATA_LABEL   "moddata"
#define MODLOAD_DATA_SUBTYPE 0x41

/* Where a module's constant data lives once mapped, or NULL. */
const void *modload_data_base(void);

/* The header read during modload_prepare(). NULL before it succeeds. */
const modload_header_t *modload_header(void);

/* The blob's entry point, computed from the instruction mapping. NULL when no
 * module is present or the header is unusable. */
void *modload_entry(void);

/* Zero the writable region lent to modules (`g_modbss`), which is where a
 * module's .bss is linked. Must run before the first call into a module. */
void modload_zero_bss(void);

/* The address the bytes are EXECUTABLE from, which is not the address they are
 * readable from. Measured: the ESP32-S3 maps flash instructions on the
 * instruction bus, and a data load through that address faults (coredump,
 * LoadStoreErrorCause). Entry offsets are relative to this base. */
const void *modload_inst_base(void);

/*
 * The self-test, as a shell command:
 *
 *   modtest          map, report the address, describe what is there
 *   modtest call     additionally call the blob's entry point
 *
 * Reports the measured address, which is what a blob must be linked for.
 */
bool modload_register_command(void);

#endif
