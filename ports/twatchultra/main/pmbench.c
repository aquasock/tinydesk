/*
 * pmbench.c - the Pokemon Mini core benchmark, on the watch, as a shell command.
 *
 * THE ONE QUESTION THIS ANSWERS. Every other platform decision in this project
 * was settled by a host measurement or by arithmetic on one. Pokemon Mini could
 * not be: the requirement is 0.22-0.36 MIPS of S1C88 instructions, derived from
 * twelve retail cartridges at 3,091-5,053 instructions per frame, and what the
 * ESP32-S3's LX7 actually retires while emulating cannot be inferred from an
 * x86 figure. QEMU cannot help -- it has no LX7 core, and a hello-world built by
 * this toolchain fails identically on every Xtensa core it offers. So the core
 * runs here.
 *
 * WHY THE CARTRIDGE IS SYNTHETIC. No commercial ROM may be distributed and none
 * is on this board. `--synthetic` fills a 32 KB image from a deterministic
 * xorshift32, which is the LARGEST a PM cartridge gets and therefore the worst
 * case for the ROM read path. The interpreter cannot tell what the opcodes mean,
 * and random 12-bit words hit disproportionately many multi-cycle paths, so this
 * is the pessimistic figure -- the one a go/no-go should be decided on. A real
 * cartridge usually comes out faster.
 *
 * TWO MEMORY CASES, BECAUSE THE ROM'S HOME DECIDES THE ANSWER. PokeMini does not
 * copy the cartridge: PokeMini_SetMINMem() stores the caller's pointer, and S1C88
 * instruction fetch comes out of the cartridge. So `--internal` puts 32 KB of ROM
 * in internal SRAM and `--psram` puts the same bytes in PSRAM, and the gap
 * between them is the cost of the choice a real port has to make. The default
 * runs both, internal first, so the pair is always measured on one boot rather
 * than compared across two.
 *
 * WHAT TO READ. `realtime` is the gate: 1.00x means the watch emulates a PM
 * cycle for cycle in wall time, and since the workload is the pessimistic one,
 * clearing realtime on a synthetic cartridge clears it for real games. The
 * instruction rate is printed against the 0.22-0.36 MIPS band for context, but
 * the synthetic mix averages far more emulated cycles per instruction than real
 * code (random opcodes take slow paths), so read it as a rate on THIS workload
 * rather than as "MIPS of real game code".
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tdsh.h"

/*
 * THE CORE'S HEADERS AND THE XTENSA SPECIAL-REGISTER NAMESPACE COLLIDE, and
 * this one line is the whole fix. `MinxCPU.h` declares the S1C88's
 * shift-arithmetic-right helper as `static inline uint8_t SAR(uint8_t A)`, while
 * `<xtensa/config/specreg.h>` -- reached through FreeRTOS.h -- defines `SAR` as
 * the special register number 3. Including the IDF headers first makes the
 * declaration expand to `uint8_t 3(uint8_t A)`, which is a parse error pointing
 * at a line in an ESP-IDF header rather than at anything that could be edited.
 *
 * `#undef` rather than reordering the includes, because reordering only works
 * while nothing earlier transitively includes FreeRTOS.h, and tdsh.h does. The
 * register number is not used anywhere in this file; the function is, through
 * the core.
 */
#undef SAR

/* The vendored core. Included through the component's exported include path,
 * which is why components/pokemini lists "." in INCLUDE_DIRS. */
#include "PokeMini.h"

static const char *TAG = "pmbench";

/* --- the emulated machine's constants ------------------------------------- */

#define PM_CLOCK_HZ       4000000
#define PM_FRAME_HZ       72
#define CYCLES_PER_FRAME  (PM_CLOCK_HZ / PM_FRAME_HZ)   /* 55,555 */

/* The requirement this project derived from twelve retail cartridges, in
 * instructions per second. See Escapement docs/pokemini-scoping.md. */
#define REQUIRED_MIPS_LO  220000.0
#define REQUIRED_MIPS_HI  360000.0

/* 32 KB is the largest a PM cartridge gets, so this is the worst case for the
 * ROM read path rather than a convenient size. */
#define SYNTH_BYTES       32768

#define PMBENCH_STACK     8192
#define PMBENCH_PRIO      5

/*
 * Core 0, explicitly, because that is where the NES port runs its emulator
 * (main/nes.c pins `nes` to core 0 and the blit to core 1). Measuring the PM
 * core anywhere else would compare it against a different machine. The radio is
 * off when this is expected to run; with WiFi or BLE up, core 0 loses cycles to
 * the stack and the number is a measurement of that instead.
 */
#define PMBENCH_CORE      0

extern int PokeMini_EmulateCycles(int cycles);
extern unsigned long g_exec_count;   /* added by patches/0001-exec-counter.patch */

/* --- the synthetic cartridge ---------------------------------------------- */

/* Deterministic, so a run on this board can be compared with a run on any other
 * and with the host harness in Escapement. Never allocated in the same place
 * twice: `--internal` and `--psram` generate identical bytes into different
 * homes, which is the whole point of having two cases. */
static uint32_t g_rng = 0x12345678u;

static uint32_t rnd(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}

static void fill_synthetic(unsigned char *buf, size_t bytes)
{
    g_rng = 0x12345678u;             /* reset per case, so both see the same image */
    for (size_t i = 0; i < bytes; i++)
        buf[i] = (unsigned char)(rnd() & 0xFFu);
}

/* --- one measured run ------------------------------------------------------ */

typedef struct
{
    const char *label;
    bool ok;
    int frames;
    double wall_s;
    double instr_per_s;
    double per_frame;
    double realtime;
    /* Why this case did not run, or NULL. A reason rather than a printf so that
     * all output leaves this file from the command's own task -- see the note
     * on pmbench_task(). */
    const char *skipped;
} pm_result_t;

/*
 * Run `frames` frames of 55,555 emulated cycles against `rom` and fill `out`.
 *
 * Prints nothing. Frames are emulated as fixed cycle slices rather than through
 * PokeMini_EmulateFrame(), which waits on a vertical blank the free BIOS never
 * produces. Slicing by cycles is also what a real port wants, since there is no
 * vsync to wait for on this board either.
 */
static bool bench_case(const char *label, unsigned char *rom, int frames,
                       pm_result_t *out)
{
    memset(out, 0, sizeof(*out));
    out->label = label;
    out->frames = frames;

    if (!PokeMini_Create(0, 0))
    {
        out->skipped = "PokeMini_Create failed";
        return false;
    }

    /* SetMINMem only points the core at the image; it does not copy, so `rom`
     * stays where the caller put it. */
    if (!PokeMini_SetMINMem(rom, SYNTH_BYTES))
    {
        PokeMini_Destroy();
        out->skipped = "SetMINMem refused";
        return false;
    }

    /* Without a reset the CPU is still wherever PokeMini_Create left it and the
     * BIOS never runs -- which reads as a working emulator measuring the BIOS
     * idle loop. Every front end calls Reset after loading for this reason. */
    PokeMini_Reset(0);

    /* Let the BIOS hand over to the cartridge before measuring, so the reset
     * sequence is not counted as per-frame cost. */
    PokeMini_EmulateCycles(CYCLES_PER_FRAME * 8);

    const unsigned long steps0 = g_exec_count;
    const int64_t t0 = esp_timer_get_time();

    for (int i = 0; i < frames; i++)
        PokeMini_EmulateCycles(CYCLES_PER_FRAME);

    const int64_t t1 = esp_timer_get_time();
    const unsigned long steps = g_exec_count - steps0;

    PokeMini_Destroy();

    out->wall_s = (double)(t1 - t0) / 1e6;
    if (out->wall_s <= 0.0)
    {
        out->skipped = "clock did not advance";
        return false;
    }

    out->instr_per_s = (double)steps / out->wall_s;
    out->per_frame   = (double)steps / (double)frames;
    /* The frames emulated are exactly 1/72 s of PM time each, so this is the
     * ratio of emulated time to wall time -- independent of how many
     * instructions the workload happened to execute. */
    out->realtime    = ((double)frames / out->wall_s) / (double)PM_FRAME_HZ;
    out->ok = true;
    return true;
}

/* --- the whole benchmark --------------------------------------------------- */

typedef struct
{
    int frames;
    int core;
    pm_result_t internal;
    pm_result_t psram;
    bool have_internal, have_psram;
    const char *internal_skip;
    const char *psram_skip;
} pm_report_t;

/* Measurement only. No printf: see pmbench_task(). */
static void run_bench(int frames, int core, bool do_internal, bool do_psram,
                      pm_report_t *rep)
{
    memset(rep, 0, sizeof(*rep));
    rep->frames = frames;
    rep->core = core;

    if (do_internal)
    {
        /* Internal SRAM: the fastest home the cartridge can have, and the
         * figure that flatters the platform. It is the ceiling, not the plan. */
        unsigned char *rom = heap_caps_malloc(SYNTH_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!rom)
        {
            rep->internal_skip = "no internal SRAM free for a 32 KB cartridge";
        }
        else
        {
            fill_synthetic(rom, SYNTH_BYTES);
            rep->have_internal = bench_case("internal", rom, frames, &rep->internal);
            if (!rep->have_internal) rep->internal_skip = rep->internal.skipped;
            heap_caps_free(rom);
        }
    }

    if (do_psram)
    {
        /* PSRAM: what a real port will do with a 32 KB-2 MB cartridge, because
         * 32 KB will not come out of internal DRAM once the emulator's own
         * state has taken its share. This is the number to decide on. */
        unsigned char *rom = heap_caps_malloc(SYNTH_BYTES, MALLOC_CAP_SPIRAM);
        if (!rom)
        {
            rep->psram_skip = "no PSRAM free for a 32 KB cartridge";
        }
        else
        {
            fill_synthetic(rom, SYNTH_BYTES);
            rep->have_psram = bench_case("psram", rom, frames, &rep->psram);
            if (!rep->have_psram) rep->psram_skip = rep->psram.skipped;
            heap_caps_free(rom);
        }
    }
}

static void print_result(const pm_result_t *r)
{
    printf("  %-9s %7.2f s   %10.0f instruct/s   %6.0f/frame   %6.2fx realtime\n",
           r->label, r->wall_s, r->instr_per_s, r->per_frame, r->realtime);
}

static void print_report(const pm_report_t *rep)
{
    printf("pmbench: %d frames of %d cycles (%d emulated seconds), core %d\n",
           rep->frames, CYCLES_PER_FRAME,
           (int)((long)rep->frames * CYCLES_PER_FRAME / PM_CLOCK_HZ), rep->core);

    if (rep->internal_skip) printf("  %-9s skipped: %s\n", "internal", rep->internal_skip);
    if (rep->psram_skip)    printf("  %-9s skipped: %s\n", "psram", rep->psram_skip);

    if (rep->have_internal) print_result(&rep->internal);
    if (rep->have_psram)    print_result(&rep->psram);

    if (rep->have_internal && rep->have_psram && rep->internal.realtime > 0.0)
        printf("  psram costs %.2fx realtime against internal SRAM\n",
               rep->psram.realtime / rep->internal.realtime);

    /*
     * THE VERDICT IS ON THE WORST CASE THAT IS STILL REALISTIC, which is PSRAM
     * when it was measured and internal SRAM otherwise. Deciding on the flattering
     * case would be how a platform gets refused after a week of porting instead of
     * before it.
     */
    const pm_result_t *verdict = rep->have_psram ? &rep->psram
                               : (rep->have_internal ? &rep->internal : NULL);
    if (!verdict)
    {
        printf("  verdict: NO MEASUREMENT\n");
        return;
    }

    printf("  instructions/sec against a %.0f-%.0f requirement: ",
           REQUIRED_MIPS_LO, REQUIRED_MIPS_HI);
    if (verdict->instr_per_s >= REQUIRED_MIPS_HI)
        printf("clears the upper bound (%.2fx)\n", verdict->instr_per_s / REQUIRED_MIPS_HI);
    else if (verdict->instr_per_s >= REQUIRED_MIPS_LO)
        printf("inside the band (%.2fx of the low bound, %.2fx of the high)\n",
               verdict->instr_per_s / REQUIRED_MIPS_LO,
               verdict->instr_per_s / REQUIRED_MIPS_HI);
    else
        printf("BELOW the lower bound (%.2fx)\n", verdict->instr_per_s / REQUIRED_MIPS_LO);

    printf("  realtime on a worst-case synthetic cartridge: %.2fx -- %s\n",
           verdict->realtime,
           verdict->realtime >= 1.0
               ? "keeps up; real cartridges are cheaper per instruction"
               : "drops frames; a real cartridge may still be cheaper per instruction");
    printf("  measured on %s\n", verdict->label);
}

/* --- the shell command ----------------------------------------------------- */

typedef struct
{
    int frames;
    int core;
    bool internal, psram;
} pm_job_t;

static pm_job_t s_job;
static pm_report_t s_report;
static SemaphoreHandle_t s_done;

/*
 * The bench runs on its own pinned task rather than inline, because the shell
 * task is created unpinned: running there would measure a different core
 * depending on what the scheduler felt like, and this number is a decision.
 *
 * BUT NOTHING IS PRINTED FROM HERE, and that is a measured requirement rather
 * than a style choice. The first version of this file printed the results from
 * this task; a run drew `pmbench: running, 2000 frames per case` and then a
 * fresh prompt with the entire measurement missing -- not even the header line
 * that is the first statement of the measuring function. Timing and the
 * semaphore were fine, so the work happened and its output did not arrive.
 *
 * The mechanism was NOT established, and the obvious theory is wrong: tdsh's
 * stdout is a single global newlib stream over the USB Serial/JTAG VFS, not a
 * per-task one (tdsh_console_espidf.c, `usb_serial_jtag_vfs_use_driver()`), so
 * a second task has no reason to be locked out of it. Between a task and the
 * glass sit the desktop's terminal capture and its incremental repaint, and
 * that is where the bytes are lost -- but which part drops them was not
 * determined. What is established is the rule that works: a command prints from
 * the task that invoked it. So this task measures and fills s_report, and
 * cmd_pmbench prints.
 */
static void pmbench_task(void *arg)
{
    (void)arg;
    run_bench(s_job.frames, s_job.core, s_job.internal, s_job.psram, &s_report);
    xSemaphoreGive(s_done);
    vTaskDelete(NULL);
}

static int parse_count(const char *s, int fallback)
{
    if (!s || !*s) return fallback;
    for (const char *p = s; *p; p++)
        if (*p < '0' || *p > '9') return fallback;
    int v = atoi(s);
    return v > 0 ? v : fallback;
}

static int cmd_pmbench(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    s_job.frames = 2000;
    s_job.core = PMBENCH_CORE;
    s_job.internal = true;
    s_job.psram = true;

    bool explicit_case = false;

    for (int i = 1; i < argc; i++)
    {
        const char *a = argv[i];
        if (strcmp(a, "--internal") == 0)
        {
            if (!explicit_case) { s_job.psram = false; explicit_case = true; }
            s_job.internal = true;
        }
        else if (strcmp(a, "--psram") == 0)
        {
            if (!explicit_case) { s_job.internal = false; explicit_case = true; }
            s_job.psram = true;
        }
        else if (strcmp(a, "--both") == 0)
        {
            s_job.internal = s_job.psram = true;
            explicit_case = true;
        }
        else if (strcmp(a, "--core") == 0 && i + 1 < argc)
        {
            int c = parse_count(argv[++i], -1);
            if (c != 0 && c != 1)
            {
                printf("--core takes 0 or 1\n");
                return 2;
            }
            s_job.core = c;
        }
        else if (a[0] == '-')
        {
            printf("usage: pmbench [frames] [--both|--internal|--psram] [--core 0|1]\n");
            return 2;
        }
        else
        {
            s_job.frames = parse_count(a, 2000);
        }
    }

    /* Two cases at the default frame count is 4,000 frames, a few seconds. A
     * long run is much better evidence than a short one: the host harness
     * reports 107 M instructions/sec at 200 frames and 194 M at 20,000, so a
     * short run measures warm-up rather than throughput. */
    printf("pmbench: running, %d frames per case\n", s_job.frames);

    s_done = xSemaphoreCreateBinary();
    if (!s_done)
    {
        printf("pmbench: no semaphore\n");
        return 1;
    }

    TaskHandle_t task = NULL;
    if (xTaskCreatePinnedToCore(pmbench_task, "pmbench", PMBENCH_STACK, NULL,
                                PMBENCH_PRIO, &task, s_job.core) != pdPASS)
    {
        vSemaphoreDelete(s_done);
        s_done = NULL;
        printf("pmbench: no task\n");
        return 1;
    }

    xSemaphoreTake(s_done, portMAX_DELAY);
    vSemaphoreDelete(s_done);
    s_done = NULL;

    print_report(&s_report);
    return 0;
}

static const tdsh_command_t s_pmbench_cmd = {
    "pmbench",
    "pmbench [frames] [--both|--internal|--psram] [--core 0|1]",
    "Measure Pokemon Mini instructions/sec on this board (synthetic cartridge)",
    cmd_pmbench,
    TDSH_CMD_INTERACTIVE,
};

bool pmbench_register_command(void)
{
    ESP_LOGI(TAG, "registering pmbench");
    return tdsh_register_command(&s_pmbench_cmd) == 0;
}
