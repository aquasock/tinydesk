/*
 * memdiag.c - `mem`, the internal-RAM accounting command.
 *
 * WHY THIS EXISTS. The board runs with a few tens of KB of internal SRAM free
 * and no single number says what took the rest. Static analysis of the ELF only
 * gets part of the way: it accounts for .dram0.data and .dram0.bss (about 107 KB
 * here) and for IRAM, which shares the same physical SRAM, but the rest is heap
 * -- per-task stacks, DMA buffers, driver state, and whatever the components
 * allocate at start-up -- and only the board can say how that divides.
 *
 * WHAT IT REPORTS, and why each part matters:
 *
 *   heap lines    esp_heap_caps_get_info() per capability. `min free` is the
 *                 LOWEST free figure since boot, so it is the real headroom:
 *                 a transient during boot can starve an allocation long before
 *                 the steady state looks tight. `largest` says whether a big
 *                 allocation could still succeed at all, which is a different
 *                 question from how much is free.
 *
 *   task lines    every task with its stack's high-water mark, which is how
 *                 much of the stack was NEVER touched at the deepest point
 *                 reached. A task with 6 KB of 8 KB unused is over-provisioned
 *                 by 6 KB, and that is memory the desktop could have.
 *
 * STACK SIZE IS NOT AVAILABLE FROM FreeRTOS, so it is not printed. The API
 * exposes the watermark (unused bytes) but not the allocation, and guessing a
 * size from the task's name would be an invented number. What is printed is
 * therefore exactly what is known, and `mem` says so rather than filling the
 * column with something plausible.
 *
 * NOTHING HERE ALLOCATES. It runs on the shell's task, which may be a PSRAM
 * stack, so it uses a bring-your-own-task-table buffer sized for the worst case
 * and touches no file system.
 */
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh.h"

#define MAX_TASKS 32

static const char *state_name(eTaskState s)
{
    switch (s)
    {
    case eRunning:   return "run";
    case eReady:     return "rdy";
    case eBlocked:   return "blk";
    case eSuspended: return "sus";
    case eDeleted:   return "del";
    default:         return "?";
    }
}

/* One capability set, in the words the board's own tools use.
 *
 * multi_heap_info_t reports free and allocated separately rather than a total,
 * so the total is their sum. `allocated` is what the heap actually handed out,
 * which is NOT the same as what the code asked for: every block carries its own
 * overhead, and that gap is part of the answer when a few KB matter. */
static void heap_line(const char *label, uint32_t caps)
{
    multi_heap_info_t info;
    memset(&info, 0, sizeof(info));
    heap_caps_get_info(&info, caps);

    const uint32_t total = (uint32_t)(info.total_free_bytes + info.total_allocated_bytes);
    if (total == 0)
    {
        printf("  %-8s (none)\n", label);
        return;
    }

    printf("  %-8s total %6u  used %6u  free %6u  largest %6u  min-free %6u  blocks %u/%u\n",
           label, (unsigned)total, (unsigned)info.total_allocated_bytes,
           (unsigned)info.total_free_bytes, (unsigned)info.largest_free_block,
           (unsigned)info.minimum_free_bytes, (unsigned)info.allocated_blocks,
           (unsigned)info.total_blocks);
}

/* ---------------------------------------------------------- by size ---- */

/* A size histogram of the ALLOCATED blocks in one pool.
 *
 * This is what answers "what is using it". A mean of 590 bytes over 400 blocks
 * says the memory is spread thin, but it does not say whether the mass is forty
 * 4 KB buffers or four hundred 600-byte structs -- and those have completely
 * different remedies. The buckets are powers of two because allocation sizes
 * cluster there: a 4 KB DMA buffer and a 100-byte struct are the two shapes
 * that matter, and anything in between is usually a multiple of something. */
#define NBUCKET 12

#define NTOP 6

typedef struct
{
    uint32_t count[NBUCKET];
    uint32_t bytes[NBUCKET];
    uint32_t used_total, free_total;
    uint32_t used_blocks, free_blocks;
    uint32_t largest_used, largest_addr;
    /* The biggest blocks BY SIZE, kept as they are seen. Addresses are printed
     * so a block can be matched against a task's stack base or a driver's known
     * buffer; a size alone cannot be attributed to anything. */
    uint32_t top_size[NTOP];
    uint32_t top_addr[NTOP];
} hist_t;

static int bucket_of(size_t n)
{
    int b = 0;
    size_t lim = 64;
    while (b < NBUCKET - 1 && n > lim)
    {
        lim <<= 1;
        b++;
    }
    return b;
}

static bool walk_cb(walker_heap_into_t hi, walker_block_info_t bi, void *ud)
{
    (void)hi;
    hist_t *h = (hist_t *)ud;
    if (bi.used)
    {
        const int b = bucket_of(bi.size);
        h->count[b]++;
        h->bytes[b] += (uint32_t)bi.size;
        h->used_total += (uint32_t)bi.size;
        h->used_blocks++;
        if (bi.size > h->largest_used)
        {
            h->largest_used = (uint32_t)bi.size;
            h->largest_addr = (uint32_t)(uintptr_t)bi.ptr;
        }
        for (int k = 0; k < NTOP; k++)
        {
            if (bi.size > h->top_size[k])
            {
                for (int j = NTOP - 1; j > k; j--)
                {
                    h->top_size[j] = h->top_size[j - 1];
                    h->top_addr[j] = h->top_addr[j - 1];
                }
                h->top_size[k] = (uint32_t)bi.size;
                h->top_addr[k] = (uint32_t)(uintptr_t)bi.ptr;
                break;
            }
        }
    }
    else
    {
        h->free_total += (uint32_t)bi.size;
        h->free_blocks++;
    }
    return true;
}

static void show_hist(const char *label, uint32_t caps)
{
    hist_t h;
    memset(&h, 0, sizeof(h));
    heap_caps_walk(caps, walk_cb, &h);

    if (h.used_blocks == 0 && h.free_blocks == 0)
    {
        printf("  %s: no blocks\n", label);
        return;
    }

    printf("%s: %u used blocks, %u bytes; %u free blocks, %u bytes\n", label,
           (unsigned)h.used_blocks, (unsigned)h.used_total,
           (unsigned)h.free_blocks, (unsigned)h.free_total);
    printf("  %-11s %5s %9s %7s\n", "size", "count", "bytes", "share");
    for (int b = 0; b < NBUCKET; b++)
    {
        if (h.count[b] == 0)
            continue;
        /* Bucket b holds sizes (64 << (b-1)) + 1 .. 64 << b, with the first one
         * starting at 0 and the last open-ended. Spelled out rather than
         * derived inline so the printed bounds cannot drift from the ones
         * bucket_of() actually uses. */
        char range[20];
        const size_t lo = b == 0 ? 0 : ((size_t)64 << (b - 1)) + 1;
        const size_t hi = (size_t)64 << b;
        if (b == 0)
            snprintf(range, sizeof(range), "<=64B");
        else if (b == NBUCKET - 1)
            snprintf(range, sizeof(range), ">%uB", (unsigned)hi);
        else if (hi >= 1024)
        {
            const unsigned lok = (unsigned)((lo + 1023) / 1024);
            const unsigned hik = (unsigned)(hi / 1024);
            if (lok >= hik)   /* a bucket thinner than 1K, e.g. 513..1024 */
                snprintf(range, sizeof(range), "%u-%uB", (unsigned)lo, (unsigned)hi);
            else
                snprintf(range, sizeof(range), "%uK-%uK", lok, hik);
        }
        else
            snprintf(range, sizeof(range), "%u-%uB", (unsigned)lo, (unsigned)hi);
        printf("  %-11s %5u %9u %6u%%\n", range, (unsigned)h.count[b],
               (unsigned)h.bytes[b],
               (unsigned)(h.used_total ? (100ull * h.bytes[b]) / h.used_total : 0));
    }
    printf("  largest blocks, by size:\n");
    for (int k = 0; k < NTOP && h.top_size[k]; k++)
        printf("    %7u B @ 0x%08x\n", (unsigned)h.top_size[k], (unsigned)h.top_addr[k]);
}

static int cmd_mem(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    /* SPLIT, because the desktop's terminal is 24 usable rows and the mirror
     * carries only what is ON it: output longer than the screen scrolls away
     * and cannot be recovered from a capture. `mem heap` and `mem tasks` each
     * fit; a bare `mem` prints both and is expected to scroll. */
    const bool heap_only = argc >= 2 && argv[1][0] == 'h';
    const bool tasks_only = argc >= 2 && argv[1][0] == 't';
    const bool blocks = argc >= 2 && argv[1][0] == 'b';

    if (blocks)
    {
        show_hist("internal", MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        return 0;
    }

    if (argc >= 2 && argv[1][0] == 'r')
    {
        /* IDF's own per-region view. heap_caps_get_info() SUMS every region
         * that matches a capability, which hides how many physical regions
         * there are and how the total divides between them -- and a "pool"
         * that is really three pools fragments differently from one. */
        printf("internal:\n");
        heap_caps_print_heap_info(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        printf("exec (IRAM):\n");
        heap_caps_print_heap_info(MALLOC_CAP_EXEC);
        printf("spiram:\n");
        heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
        return 0;
    }

    if (!tasks_only)
    {
        printf("heap, by capability (bytes; min-free is the low-water mark since boot):\n");

        /* INTERNAL first: it is the pool everything competes for. */
        heap_line("ALL", MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        printf("  %-8s total %6u  used %6u  free %6u  largest %6u\n", "DMA",
               (unsigned)heap_caps_get_total_size(MALLOC_CAP_DMA),
               (unsigned)(heap_caps_get_total_size(MALLOC_CAP_DMA) -
                          heap_caps_get_free_size(MALLOC_CAP_DMA)),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        printf("  %-8s total %6u  used %6u  free %6u  largest %6u\n", "IRAM-32",
               (unsigned)heap_caps_get_total_size(MALLOC_CAP_32BIT),
               (unsigned)(heap_caps_get_total_size(MALLOC_CAP_32BIT) -
                          heap_caps_get_free_size(MALLOC_CAP_32BIT)),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_32BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
        heap_line("PSRAM", MALLOC_CAP_SPIRAM);
        heap_line("EXEC", MALLOC_CAP_EXEC | MALLOC_CAP_8BIT);
        printf("  %-8s %u bytes (esp_get_free_heap_size)\n", "any-free",
               (unsigned)esp_get_free_heap_size());
        printf("\n");
        if (heap_only)
            return 0;
    }

    static TaskStatus_t s_tasks[MAX_TASKS];
    const UBaseType_t n = uxTaskGetSystemState(s_tasks, MAX_TASKS, NULL);

    printf("tasks (%u visible, %d is this command's limit):\n", (unsigned)n, MAX_TASKS);
    printf("  %-16s %-3s %-3s %6s %-8s %8s\n", "name", "pri", "st", "unused",
           "stack in", "base");

    /* Two passes so the report is ordered by what is UNUSED: the biggest
     * opportunities to reclaim come first rather than the biggest priority.
     *
     * usStackHighWaterMark is already in BYTES here: this port's StackType_t is
     * uint8_t, so the usual "multiply by sizeof(StackType_t)" would be a no-op
     * that implies a conversion which is not happening.
     *
     * The stack's SIZE is not in any FreeRTOS API, so it is not printed. What
     * IS printed is where the stack lives, from pxStackBase: internal SRAM or
     * PSRAM. That is the column that matters, because a task on a PSRAM stack
     * costs the scarce pool nothing. */
    bool shown[MAX_TASKS];
    memset(shown, 0, sizeof(shown));
    for (int pass = 0; pass < 2; pass++)
    {
        for (UBaseType_t i = 0; i < n; i++)
        {
            if (pass == 0 && s_tasks[i].usStackHighWaterMark < 1024)
                continue;
            if (shown[i])
                continue;
            shown[i] = true;
            const uintptr_t base = (uintptr_t)s_tasks[i].pxStackBase;
            const char *where = base >= 0x3C000000 && base < 0x3E000000 ? "PSRAM"
                              : base >= 0x3FC00000 && base < 0x3FE00000 ? "internal"
                                                                        : "?";
            printf("  %-16s %-3u %-3s %6u %-8s %8x\n", s_tasks[i].pcTaskName,
                   (unsigned)s_tasks[i].uxCurrentPriority,
                   state_name(s_tasks[i].eCurrentState),
                   (unsigned)s_tasks[i].usStackHighWaterMark, where,
                   (unsigned)(uintptr_t)s_tasks[i].pxStackBase);
        }
    }

    if (heap_only || tasks_only)
        printf("\nunused = never-touched bytes at the deepest point. Size is not\n"
               "exposed by FreeRTOS, so only the stack's HOME is shown.\n");
    return 0;
}

static const tdsh_command_t s_mem_cmd = {
    "mem",
    "mem [heap|tasks|blocks|regions]",
    "Internal RAM: heap by capability, task stacks, and used-block sizes",
    cmd_mem,
    0,
};

bool memdiag_register_command(void)
{
    return tdsh_register_command(&s_mem_cmd) == 0;
}
