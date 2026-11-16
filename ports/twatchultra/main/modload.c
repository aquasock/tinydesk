/*
 * modload.c - map the `modules` partition as instructions and call into it.
 *
 * See modload.h for why this exists. This file is deliberately small: it is an
 * experiment, and the whole point is that a negative result is cheap.
 */
#include "modload.h"

#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "tdsh.h"

static const char *TAG = "modload";

/* The partition as declared in partitions.csv: data, subtype 0x40, label
 * "modules". Found by label AND subtype so a table edit that renames or
 * retypes it fails loudly here instead of mapping something else. */
#define MODLOAD_PART_LABEL   "modules"
#define MODLOAD_PART_SUBTYPE 0x40

/*
 * ONE MAPPING, IN THE INSTRUCTION WINDOW, AND THE HEADER READ BY
 * esp_partition_read().
 *
 * TWO HARDWARE FACTS, EACH FROM A COREDUMP, AND THEY CONSTRAIN EACH OTHER:
 *
 *   1. Mapping as INSTRUCTIONS and then reading the header THROUGH THAT POINTER
 *      faults: `exccause 3 (LoadStoreErrorCause), excvaddr 0x42ae0014`, in
 *      memcpy reached from printf formatting the header's name string. The
 *      magic word at +0 read fine and the string at +0x10 did not. The
 *      ESP32-S3 puts flash instructions on the instruction bus at
 *      0x42000000..0x43ffffff and a data load there is not a supported access.
 *
 *   2. Mapping the SAME partition TWICE (once DATA, once INST) does not give
 *      two windows. The second mapping came back at `0x3cae0000`, which is
 *      PSRAM address space, and calling into it faulted with `exccause 2
 *      (InstructionFetchErrorCause), excvaddr 0x3cae0028`.
 *
 * So: exactly ONE mapping, as INSTRUCTIONS, for execution only. The header is
 * read with esp_partition_read() into a stack struct, which is a normal flash
 * read through the data path and cannot fault on the instruction bus. The blob
 * carries no strings the host needs at run time -- the name is for logs, and a
 * log line is not worth a second mapping.
 */
static const void *s_inst;
static esp_partition_mmap_handle_t s_handle_inst;
static size_t s_size;

bool modload_map(const void **base, size_t *size, char *why, size_t why_len)
{
    if (s_inst)
    {
        *base = s_inst;
        *size = s_size;
        return true;
    }

    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                        MODLOAD_PART_SUBTYPE,
                                                        MODLOAD_PART_LABEL);

    if (!p)
    {
        snprintf(why, why_len, "no '%s' partition (data/0x%02X) in the table",
                 MODLOAD_PART_LABEL, MODLOAD_PART_SUBTYPE);
        return false;
    }

    /* EXACTLY ONE MAPPING. See the note above on why a second one is fatal. */
    const esp_err_t err = esp_partition_mmap(p, 0, p->size, ESP_PARTITION_MMAP_INST,
                                             &s_inst, &s_handle_inst);
    if (err != ESP_OK)
    {
        snprintf(why, why_len, "mmap(INST, %u B) failed: %s",
                 (unsigned)p->size, esp_err_to_name(err));
        return false;
    }

    s_size = p->size;
    ESP_LOGI(TAG, "'%s' %u B mapped for execution at %p", MODLOAD_PART_LABEL,
             (unsigned)s_size, s_inst);

    *base = s_inst;
    *size = s_size;
    return true;
}

/* Read the blob header into `h` through the DATA path, which is the only safe
 * way to touch these bytes without executing them. */
bool modload_read_header(modload_header_t *h)
{
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                        MODLOAD_PART_SUBTYPE,
                                                        MODLOAD_PART_LABEL);

    if (!p || !h)
        return false;

    return esp_partition_read(p, 0, h, sizeof(*h)) == ESP_OK;
}

void modload_unmap(void)
{
    if (s_inst)
        esp_partition_munmap(s_handle_inst);
    s_inst = NULL;
    s_size = 0;
}

const void *modload_inst_base(void)
{
    return s_inst;
}

/* Is this address inside the region the CPU fetches from? A blob that maps at
 * a DATA address would be readable and not executable, and the difference is
 * worth reporting before anyone tries to call it. */
static bool in_irom(const void *p)
{
    const uintptr_t a = (uintptr_t)p;

    return a >= 0x42000000u && a < 0x44000000u;
}

/*
 * Persist the verdict where the PC can read it.
 *
 * TWO CHANNELS, because the first two attempts at this failed for different
 * reasons and both are worth knowing:
 *
 *   - the display mirror only sends CHANGED cells, so a one-line result is
 *     unreadable without a full repaint, and a focused window eats keystrokes;
 *   - esp_partition_write() refuses a partition that is currently mapped, so
 *     writing the result into `modules` itself needs the unmap and is still
 *     fighting the region the test just used.
 *
 * A plain file on the littlefs partition has neither problem: it is a normal
 * write, and the PC can extract the whole partition from flash with esptool and
 * mount it with littlefs-python. Slow, but it cannot lie.
 */
static void write_result_file(const modload_result_t *r)
{
    static const char *paths[] = { "/root/modresult.txt", "/modresult.txt" };

    for (size_t i = 0; i < sizeof(paths) / sizeof(paths[0]); i++)
    {
        FILE *f = fopen(paths[i], "w");

        if (!f)
            continue;
        fprintf(f, "magic=%08x\ndata=%08x\ninst=%08x\nentry=%08x\n"
                   "called=%u\nheap_free=%u\nheap_largest=%u\nret=%d\n",
                (unsigned)r->magic, (unsigned)r->data_base, (unsigned)r->inst_base,
                (unsigned)r->entry, (unsigned)r->called, (unsigned)r->heap_free,
                (unsigned)r->heap_largest, (int)r->ret);
        fclose(f);
        ESP_LOGI(TAG, "result -> %s (ret %ld)", paths[i], (long)(int)r->ret);
        return;
    }
    ESP_LOGE(TAG, "could not write the result file");
}

/* Persist the verdict where the PC can read it. See the header for why. */
static void write_result(const modload_result_t *r)
{
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                        MODLOAD_PART_SUBTYPE,
                                                        MODLOAD_PART_LABEL);

    if (!p)
        return;

    esp_err_t err = esp_partition_erase_range(p, MODLOAD_RESULT_OFFSET, 0x1000);

    if (err == ESP_OK)
        err = esp_partition_write(p, MODLOAD_RESULT_OFFSET, r, sizeof(*r));

    if (err != ESP_OK)
        ESP_LOGE(TAG, "result write failed: %s", esp_err_to_name(err));
}

void modload_selftest_call(void)
{
    char why[96] = "";
    const void *base = NULL;
    size_t size = 0;
    modload_result_t res = { .magic = MODLOAD_RESULT_MAGIC };

    if (!modload_map(&base, &size, why, sizeof(why)))
    {
        ESP_LOGE(TAG, "%s", why);
        res.ret = 0xFFFFFFFFu;
        write_result_file(&res);
        return;
    }

    res.data_base = (uint32_t)(uintptr_t)base;
    res.inst_base = (uint32_t)(uintptr_t)modload_inst_base();
    (void)size;
    res.heap_free = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    res.heap_largest = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    /* The header comes through esp_partition_read(), NOT through the mapped
     * pointer: see the note at the top of this file. */
    modload_header_t hdr = {0};
    const modload_header_t *h = &hdr;

    modload_read_header(&hdr);
    ESP_LOGI(TAG, "blob magic 0x%08lx abi %lu size %lu entry +0x%lx",
             (unsigned long)h->magic, (unsigned long)h->abi_version,
             (unsigned long)h->size, (unsigned long)h->entry_offset);

    if (h->magic == MODLOAD_MAGIC && h->abi_version == MODLOAD_ABI_VERSION &&
        h->entry_offset < size)
    {
        const modload_entry_t fn =
            (modload_entry_t)((const uint8_t *)res.inst_base + h->entry_offset);

        res.entry = (uint32_t)(uintptr_t)fn;
        res.called = 1;
        res.ret = (uint32_t)fn(6, 7, res.heap_free, res.heap_largest);
        ESP_LOGI(TAG, "entry %p returned %ld", (void *)fn, (long)res.ret);
    }
    else
    {
        res.ret = 0xFFFFFFFEu;      /* no usable blob */
    }

    /* UNMAP BEFORE WRITING. esp_partition_write() refuses a region that is
     * currently mapped, which is why the first attempt at this left the result
     * sector erased and looked like the test had never run. */
    modload_unmap();
    write_result(&res);
    write_result_file(&res);
}

static int cmd_modtest(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    const bool do_call = argc >= 2 && strcmp(argv[1], "call") == 0;
    char why[96] = "";
    const void *base = NULL;
    size_t size = 0;

    modload_result_t res = { .magic = MODLOAD_RESULT_MAGIC };

    if (!modload_map(&base, &size, why, sizeof(why)))
    {
        printf("modtest: %s\n", why);
        res.ret = 0xFFFFFFFFu;              /* mmap refused */
        write_result(&res);
        return 1;
    }

    res.data_base = (uint32_t)(uintptr_t)base;
    res.inst_base = (uint32_t)(uintptr_t)modload_inst_base();
    (void)size;

    printf("modules: %u bytes\n", (unsigned)size);
    printf("  data  %p %s\n", base,
           in_irom(base) ? "(instruction space!)" : "(data space)");
    printf("  exec  %p %s\n", modload_inst_base(),
           in_irom(modload_inst_base()) ? "(instruction space)" : "(NOT exec!)");

    /* Free heap is passed to the blob so it can prove it sees this board's
     * memory rather than a stale view. */
    const uint32_t hf = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const uint32_t hl = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    printf("heap: free %lu largest %lu\n", (unsigned long)hf, (unsigned long)hl);

    modload_header_t hdr = {0};
    const modload_header_t *h = &hdr;

    modload_read_header(&hdr);

    if (h->magic != MODLOAD_MAGIC)
    {
        printf("blob: none (magic 0x%08lx, want 0x%08lx)\n",
               (unsigned long)h->magic, (unsigned long)MODLOAD_MAGIC);
        return 0;
    }

    printf("blob: '%s' abi %lu size %lu entry +0x%lx\n", h->name,
           (unsigned long)h->abi_version, (unsigned long)h->size,
           (unsigned long)h->entry_offset);

    if (h->abi_version != MODLOAD_ABI_VERSION)
    {
        printf("modtest: abi %lu != %d, refusing to call\n",
               (unsigned long)h->abi_version, MODLOAD_ABI_VERSION);
        return 1;
    }

    if (h->size > size || h->entry_offset >= size)
    {
        printf("modtest: blob header points outside the partition, refusing\n");
        return 1;
    }

    if (!do_call)
    {
        printf("modtest: 'modtest call' to call the entry point\n");
        return 0;
    }

    /* The entry is an offset into the BLOB, so it is added to the INSTRUCTION
     * mapping's base, not the data one. */
    const modload_entry_t fn =
        (modload_entry_t)((const uint8_t *)modload_inst_base() + h->entry_offset);

    /* THE ACTUAL EXPERIMENT. A function pointer call across a partition
     * boundary: the compiler emits a CALL8 with its window rotation, and the
     * blob's own literals must be correct for the address it was mapped at. */
    printf("modtest: calling %p(6, 7, %lu, %lu)\n", (void *)fn,
           (unsigned long)hf, (unsigned long)hl);

    res.entry = (uint32_t)(uintptr_t)fn;
    res.heap_free = hf;
    res.heap_largest = hl;
    res.called = 1;

    const int32_t r = fn(6, 7, hf, hl);

    res.ret = (uint32_t)r;

    /* The result sector lives inside the mapped partition, and
     * esp_partition_write() refuses a mapped region. */
    modload_unmap();
    write_result(&res);

    /* 6*7 = 42 is the expected answer and proves the arguments arrived. */
    printf("modtest: returned %ld (record written to modules+0x%X)\n", (long)r,
           MODLOAD_RESULT_OFFSET);
    return r == 42 ? 0 : 1;
}

static const tdsh_command_t s_modtest_cmd = {
    "modtest",
    "modtest [call]",
    "Map the modules partition as instructions and call a blob",
    cmd_modtest,
    TDSH_CMD_INTERACTIVE,
};

bool modload_register_command(void)
{
    return tdsh_register_command(&s_modtest_cmd) == 0;
}
