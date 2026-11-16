/*
 * modload.c - map the `modules` partition as instructions and call into it.
 *
 * See modload.h for why this exists. This file is deliberately small: it is an
 * experiment, and the whole point is that a negative result is cheap.
 */
#include "modhost.h"
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

/*
 * THE DATA SIDE. A module's .text is fetched from the instruction window and its
 * .rodata is read from this one -- they are different partitions because the two
 * windows cannot serve the same bytes (see partitions.csv and modload.h).
 */
static const void *s_data_base;
static esp_partition_mmap_handle_t s_handle_data;

const void *modload_data_base(void)
{
    return s_data_base;
}

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

/* Cached by modload_prepare(), which must run on an internal-stack task. */
static modload_header_t s_hdr;
static bool s_prepared;
static void *s_entry;

/*
 * WHY A PREPARE FAILED, RETAINED.
 *
 * The caller passes a buffer, but the failure is usually wanted much later -- by
 * `music` when the user finally asks for a track, not by the boot task that knew.
 * Without keeping it the only report was "no module mapped at boot", which is
 * true and useless: it does not distinguish an empty partition from a blob
 * linked for a different mapping, and those need different fixes.
 */
static char s_why[128];

const char *modload_why(void)
{
    return s_why[0] ? s_why : "module not prepared";
}

bool modload_prepare(char *why, size_t why_len)
{
    if (s_prepared)
        return true;

    /* Every exit below that fails leaves the reason in s_why. */
    s_why[0] = '\0';

    const void *base = NULL;
    size_t size = 0;

    if (!modload_map(&base, &size, why, why_len))
    {
        snprintf(s_why, sizeof(s_why), "%s", why);
        return false;
    }

    /* Map the data side too. It has to happen HERE, on the boot task: mapping
     * disables the flash cache, so a PSRAM-stack task cannot do it. */
    const esp_partition_t *dp = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                         MODLOAD_DATA_SUBTYPE,
                                                         MODLOAD_DATA_LABEL);

    if (dp)
    {
        const esp_err_t derr = esp_partition_mmap(dp, 0, dp->size, ESP_PARTITION_MMAP_DATA,
                                                  &s_data_base, &s_handle_data);

        if (derr == ESP_OK)
            ESP_LOGI(TAG, "'%s' %u B mapped for data at %p", MODLOAD_DATA_LABEL,
                     (unsigned)dp->size, s_data_base);
        else
            ESP_LOGW(TAG, "mmap(DATA) failed: %s", esp_err_to_name(derr));
    }
    else
    {
        ESP_LOGW(TAG, "no '%s' partition; a module with .rodata cannot run",
                 MODLOAD_DATA_LABEL);
    }

    modload_read_header(&s_hdr);

    if (s_hdr.magic != MODLOAD_MAGIC || s_hdr.abi_version != MODLOAD_ABI_VERSION ||
        s_hdr.entry_offset >= size)
    {
        snprintf(why, why_len, "no usable module (magic 0x%08lx abi %lu, want %u)",
                 (unsigned long)s_hdr.magic, (unsigned long)s_hdr.abi_version,
                 (unsigned)MODLOAD_ABI_VERSION);
        snprintf(s_why, sizeof(s_why), "%s", why);
        s_prepared = false;
        s_entry = NULL;
        return false;
    }

    /*
     * REFUSE A BLOB LINKED FOR A DIFFERENT MAPPING.
     *
     * Both windows are runtime-chosen and both move when the app image changes
     * size, so a module built against the previous pair is not merely stale, it
     * is wrong in a way that does not crash: .text still runs from the entry
     * offset, and the damage appears only when a constant is read -- 64 KB off,
     * from the wrong place in the data window.
     *
     * Measured cost of not checking: the first real track played nothing and the
     * decoder reported "bad magic number" on two files that were fine. One line
     * here replaces that hunt.
     */
    {
        const uint32_t live_inst = (uint32_t)(uintptr_t)modload_inst_base();
        const uint32_t live_data = (uint32_t)(uintptr_t)s_data_base;

        if (s_hdr.linked_inst != live_inst || s_hdr.linked_data != live_data)
        {
            snprintf(why, why_len,
                     "module linked for inst %08lx data %08lx, mapped at %08lx %08lx"
                     " -- rebuild it: make -C modules flac",
                     (unsigned long)s_hdr.linked_inst,
                     (unsigned long)s_hdr.linked_data, (unsigned long)live_inst,
                     (unsigned long)live_data);
            snprintf(s_why, sizeof(s_why), "%s", why);
            ESP_LOGE(TAG, "%s", why);
            s_prepared = false;
            s_entry = NULL;
            return false;
        }
    }

    s_entry = (void *)((const uint8_t *)modload_inst_base() + s_hdr.entry_offset);
    s_prepared = true;

    /*
     * PUBLISH THE ADDRESSES FOR THE MODULE BUILD.
     *
     * A module's .rodata is linked at an absolute address, and that address is
     * whatever the MMU hands back for the data partition -- knowable only at
     * runtime. So it is written where the build can read it, and modules/Makefile
     * picks it up. Typing it by hand would work exactly once.
     */
    {
        FILE *f = fopen("/fs/root/modaddr.txt", "w");

        if (f)
        {
            fprintf(f, "inst=%p\ndata=%p\n", modload_inst_base(), s_data_base);
            fclose(f);
        }
    }

    return true;
}

bool modload_ready(void)
{
    return s_prepared;
}

const modload_header_t *modload_header(void)
{
    return s_prepared ? &s_hdr : NULL;
}

void *modload_entry(void)
{
    return s_entry;
}

void modload_unmap(void)
{
    if (s_inst)
        esp_partition_munmap(s_handle_inst);
    s_inst = NULL;
    s_size = 0;
}

/*
 * Zero the writable region the firmware lends to modules.
 *
 * A blob cannot zero its own .bss -- it executes from read-only flash and no
 * loader runs -- so its linker script places .bss in `g_modbss`, and this is the
 * loader step that would normally do the zeroing. Derived from the symbol, not
 * from a constant, so a change of size cannot be half-applied.
 */
extern uint8_t g_modbss[];
extern uint32_t g_modbss_size;

void modload_zero_bss(void)
{
    memset(g_modbss, 0, g_modbss_size);
    ESP_LOGI(TAG, "module bss zeroed: %lu bytes at %p", (unsigned long)g_modbss_size,
             (void *)g_modbss);
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
    /* TDSH_MOUNT_POINT is "/fs": littlefs lives at /fs in the POSIX VFS and the
     * SHELL presents that as "/". A path of "/root/..." therefore fails from
     * ordinary C -- the process sees /fs/root/... . This cost a wrong handoff
     * note claiming the mount point was "/" and not /fs. */
    static const char *paths[] = { "/fs/root/modresult.txt", "/fs/modresult.txt" };

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
    const mod_entry_fn fn =
        (mod_entry_fn)((const uint8_t *)modload_inst_base() + h->entry_offset);

    /* A REAL CALL WITH REAL BUFFERS. The blob is asked to allocate through the
     * API table, copy through it, and write into a caller-owned PCM buffer --
     * which is exactly the shape a decoder will be given. */
    int32_t pcm[8] = { 0 };
    static uint8_t modstate[2048] __attribute__((aligned(8)));   /* caller-owned */
    modcall_t call = {
        .in = NULL,
        .in_len = 0,
        .consumed = NULL,
        .pcm = pcm,
        .pcm_capacity = 8,
        .pcm_samples = NULL,
        .state = modstate,
        .state_size = sizeof(modstate),
    };
    size_t used = 0, samples = 0;

    call.consumed = &used;
    call.pcm_samples = &samples;

    /* Modules have no writable bss of their own; the firmware lends them one,
     * and it must be cleared before the first call, exactly as a loader would. */
    modload_zero_bss();

    printf("modtest: calling %p(&call)\n", (void *)fn);

    const int32_t r = fn(&call);

    res.entry = (uint32_t)(uintptr_t)fn;
    res.heap_free = hf;
    res.heap_largest = hl;
    res.called = 1;
    res.ret = (uint32_t)r;

    modload_unmap();

    /* 42 means every step of the API-table round trip worked. The pcm[] values
     * it left behind are the evidence: a pattern byte, the ABI it saw, and a
     * pointer that must lie inside this firmware's RAM. */
    printf("modtest: returned %ld\n", (long)r);
    if (r == 42)
        printf("  api ok: pcm[2]=%ld (abi) pcm[3]=0x%lx\n", (long)pcm[2],
               (unsigned long)pcm[3]);

    write_result_file(&res);

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
