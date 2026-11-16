/*
 * modflac.c - play a FLAC file using the decoder in the `modules` partition.
 *
 * WHAT THIS PROVES AND WHY IT EXISTS. micro-flac costs 31.7 KB inline in the app,
 * where only ~24 KB of headroom remained. As a module it costs the app image
 * nothing: the code lives in the 2 MB `modules` partition and executes from
 * there by XIP, so loading it spends no internal SRAM either. This command is
 * the integration point -- it decodes real audio through the module and plays it
 * through the same I2S path `music` uses.
 *
 * THE LOOP IS `music`'s WITH A FUNCTION POINTER IN THE MIDDLE:
 *
 *   read a window -> call the module's entry -> write the PCM -> repeat
 *
 * and the module gets its buffers from here, never from itself, which is what
 * keeps the memory cost visible to the firmware that has to live within it.
 *
 * THE TWO THINGS A MODULE NEEDS THAT THE APP MUST DO FOR IT:
 *   1. zero its .bss -- the firmware lends `g_modbss` and the module's linker
 *      script places .bss there, but nothing zeroes it unless the loader does.
 *   2. give it a state block -- the decoder object has no home in read-only
 *      flash, so it is placement-new'd inside a caller-provided buffer.
 */
#include "modflac.h"

#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modhost.h"
#include "modload.h"
#include "sound.h"
#include "tdsh.h"

__attribute__((unused)) static const char *TAG = "modflac";

#define MODFLAC_IN_BYTES    (16 * 1024)
/* A FLAC block is at most 4608 frames in this decoder's configuration, and the
 * module is handed this many samples at a time. Generous so an 8192-block
 * stream still fits. */
#define MODFLAC_OUT_FRAMES  8192
/* The decoder object lives in here. It is a few hundred bytes; the buffers it
 * allocates for its own work come through the host table and go to PSRAM. */
#define MODFLAC_STATE_BYTES 4096
#define MODFLAC_WRITE_MS    1000
#define MODFLAC_STACK       8192
#define MODFLAC_PRIO        3

static char s_path[128];
static TaskHandle_t s_task;
static volatile bool s_stop;
static volatile int s_state;
static volatile uint32_t s_frames;
static volatile uint32_t s_calls;
static volatile int32_t s_verdict;

/* The module handle's first words, copied out before the block is freed so the
 * status command can explain a failure without a diagnostic file. The console
 * works now, which is cheaper than any file. */
static volatile uint32_t s_h_magic, s_h_ready;
static volatile int32_t s_h_rate, s_h_ch, s_h_bits, s_h_last;
static volatile uint32_t s_h_consumed, s_h_calls;
static char s_msg[64];

static void set_msg(const char *m) { snprintf(s_msg, sizeof(s_msg), "%s", m); }

/* A diagnostic file, for the same reason modtest has one: the display mirror
 * cannot be trusted for a multi-field answer, and this task's status line is cut
 * at 40 columns. */
static void dump_state(const void *state, size_t size)
{
    /*
     * ON THE SD CARD, NOT ON LITTLEFS, and that is not a preference.
     *
     * littlefs lives in the SPI flash partition, so opening a file there calls
     * esp_flash_read(), which DISABLES THE FLASH CACHE -- and this task's stack
     * is in PSRAM, which is reached through that cache. The coredump:
     *
     *   assert failed: esp_task_stack_is_sane_cache_disabled()
     *     esp_flash_read(address=0x620F40) <- littlefs <- fopen
     *
     * So every task in this port that runs on a PSRAM stack must also stay off
     * /fs. /sd is the SD card over its own bus and is fine -- `music` reads
     * audio from there on a PSRAM stack all day.
     */
    FILE *f = fopen("/sd/modflac.txt", "w");

    if (!f)
        return;
    fprintf(f, "frames=%lu\ncalls=%lu\nverdict=%ld\nmsg=%s\n",
            (unsigned long)s_frames, (unsigned long)s_calls, (long)s_verdict, s_msg);

    /* The first two words of the module's handle: its magic and ready flag. */
    if (state && size >= 8)
    {
        const uint32_t *w = (const uint32_t *)state;

        fprintf(f, "handle_magic=%08lx ready=%lu\n", (unsigned long)w[0],
                (unsigned long)w[1]);
        fprintf(f, "rate=%ld ch=%ld bits=%ld last=%ld consumed=%lu n=%lu\n",
                (long)(int32_t)w[2], (long)(int32_t)w[3], (long)(int32_t)w[4],
                (long)(int32_t)w[5], (unsigned long)w[6], (unsigned long)w[7]);
    }
    fclose(f);
}

static void play_task(void *arg)
{
    (void)arg;

    /*
     * NO MAPPING HERE. This task runs on a PSRAM stack, and esp_partition_mmap()
     * disables the flash cache -- which PSRAM is reached through -- so mapping
     * from here panics with "esp_task_stack_is_sane_cache_disabled()". The boot
     * task mapped the partition and read the header already; this takes those.
     */
    /*
     * ASK, DO NOT MAP. This task runs on a PSRAM stack, so calling
     * modload_prepare() here would retry a mapping that disables the flash
     * cache -- the panic is quoted in modload.h. Only the boot task maps.
     */
    if (!modload_ready())
    {
        set_msg("no module mapped at boot");
        s_state = 3;
        goto done;
    }

    const modload_header_t *h = modload_header();

    if (!h || strcmp(h->name, "flac") != 0)
    {
        set_msg("no flac module in the partition");
        s_state = 3;
        goto done;
    }

    const mod_entry_fn fn = (mod_entry_fn)modload_entry();

    FILE *f = fopen(s_path, "rb");

    if (!f)
    {
        set_msg("cannot open file");
        s_state = 3;
        goto done;
    }

    uint8_t *in = (uint8_t *)heap_caps_malloc(MODFLAC_IN_BYTES, MALLOC_CAP_SPIRAM);
    int16_t *out = (int16_t *)heap_caps_malloc(MODFLAC_OUT_FRAMES * 2 * sizeof(int16_t),
                                               MALLOC_CAP_SPIRAM);
    /* int32 samples in, as the module produces them left-justified */
    int32_t *pcm = (int32_t *)heap_caps_malloc(MODFLAC_OUT_FRAMES * 2 * sizeof(int32_t),
                                               MALLOC_CAP_SPIRAM);
    void *state = heap_caps_malloc(MODFLAC_STATE_BYTES,
                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!in || !out || !pcm || !state)
    {
        set_msg("no memory");
        s_state = 3;
        goto cleanup;
    }

    size_t have = 0;
    bool started = false;
    int consecutive_errors = 0;

    while (!s_stop)
    {
        if (have < MODFLAC_IN_BYTES)
        {
            const size_t n = fread(in + have, 1, MODFLAC_IN_BYTES - have, f);

            have += n;
            if (n == 0 && have == 0)
                break;
        }

        size_t consumed = 0, samples = 0;
        modcall_t call = {
            .in = in,
            .in_len = have,
            .consumed = &consumed,
            .pcm = pcm,
            .pcm_capacity = MODFLAC_OUT_FRAMES * 2,
            .pcm_samples = &samples,
            .state = state,
            .state_size = MODFLAC_STATE_BYTES,
        };

        const int32_t r = fn(&call);

        s_calls++;
        s_verdict = r;

        if (consumed > 0 && consumed <= have)
        {
            memmove(in, in + consumed, have - consumed);
            have -= consumed;
        }

        if (r == MODCALL_OK)
        {
            consecutive_errors = 0;

            if (!started)
            {
                /* The module reports the rate once STREAMINFO is parsed, which
                 * is signalled by a first OK with no samples. */
                if (samples == 0)
                {
                    if (feof(f) && have == 0)
                        break;
                    continue;
                }

                /* We do not have the rate through modcall_t yet -- use the file's
                 * own STREAMINFO via the decoder is not exposed, so start at the
                 * common case and let a wrong rate be audible rather than
                 * silent. */
                if (!sound_stream_open(44100))
                {
                    set_msg("no amplifier");
                    s_state = 3;
                    break;
                }
                started = true;
            }

            if (samples > 0)
            {
                const size_t frames = samples / 2;

                for (size_t i = 0; i < samples; i++)
                    out[i] = (int16_t)(pcm[i] >> 16);

                const size_t got = sound_stream_write_stereo_wait(out, (int)frames,
                                                                  MODFLAC_WRITE_MS);

                s_frames += (uint32_t)got;
            }
        }
        else if (r == MODCALL_NEED_INPUT)
        {
            if (consumed == 0 && have == MODFLAC_IN_BYTES)
            {
                have = 0;
                if (++consecutive_errors > 8)
                {
                    set_msg("decoder stuck");
                    break;
                }
            }
            if (feof(f) && have == 0)
                break;
        }
        else if (r == MODCALL_END)
        {
            set_msg("end of track");
            break;
        }
        else
        {
            if (++consecutive_errors > 8)
            {
                set_msg("decode error");
                break;
            }
            have = 0;
        }
    }

    if (s_state == 1)
        s_state = s_stop ? 0 : 2;
    if (started)
        sound_stream_close();
    /* Copy the handle out before it is freed: the status command reports these,
     * and they are what explain a decode that returned an error. */
    {
        const uint32_t *w = (const uint32_t *)state;

        s_h_magic = w[0];
        s_h_ready = w[1];
        s_h_rate = (int32_t)w[2];
        s_h_ch = (int32_t)w[3];
        s_h_bits = (int32_t)w[4];
        s_h_last = (int32_t)w[5];
        s_h_consumed = w[6];
        s_h_calls = w[7];
    }
    dump_state(state, MODFLAC_STATE_BYTES);

cleanup:
    if (in) heap_caps_free(in);
    if (out) heap_caps_free(out);
    if (pcm) heap_caps_free(pcm);
    if (state) heap_caps_free(state);
    fclose(f);

done:
    s_task = NULL;
    vTaskDelete(NULL);
}

static void task_entry(void *arg) { play_task(arg); }

static bool start(const char *path)
{
    if (s_task || strlen(path) >= sizeof(s_path))
        return false;

    snprintf(s_path, sizeof(s_path), "%s", path);
    s_stop = false;
    s_state = 1;
    s_frames = 0;
    s_calls = 0;
    s_verdict = 0;
    set_msg("");

    if (xTaskCreateWithCaps(task_entry, "modflac", MODFLAC_STACK, NULL, MODFLAC_PRIO, &s_task,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS)
    {
        s_task = NULL;
        s_state = 0;
        return false;
    }
    return true;
}

static int cmd_modflac(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    if (argc >= 2 && strcmp(argv[1], "stop") == 0)
    {
        s_stop = true;
        printf("modflac: stopping\n");
        return 0;
    }

    if (argc < 2)
    {
        printf("modflac <file.flac> | modflac stop\n");
        if (s_task || s_state >= 2)
        {
            printf("state %s, %lu frames, %lu calls, verdict %ld%s%s\n",
                   s_state == 1 ? "playing" : (s_state == 2 ? "done" : "error"),
                   (unsigned long)s_frames, (unsigned long)s_calls, (long)s_verdict,
                   s_msg[0] ? ": " : "", s_msg);
            /* The module handle, which is what says WHY a decode failed. */
            printf("handle: magic %08lx ready %lu rate %ld ch %ld bits %ld\n",
                   (unsigned long)s_h_magic, (unsigned long)s_h_ready,
                   (long)s_h_rate, (long)s_h_ch, (long)s_h_bits);
            printf("        last %ld consumed %lu in-calls %lu\n",
                   (long)s_h_last, (unsigned long)s_h_consumed,
                   (unsigned long)s_h_calls);
        }
        return 0;
    }

    if (!start(argv[1]))
    {
        printf("modflac: cannot start\n");
        return 1;
    }
    printf("modflac: decoding %s through the module\n", argv[1]);
    return 0;
}

static const tdsh_command_t s_modflac_cmd = {
    "modflac",
    "modflac <file.flac> | modflac stop",
    "Play a FLAC using the decoder in the modules partition",
    cmd_modflac,
    TDSH_CMD_INTERACTIVE,
};

bool modflac_register_command(void)
{
    return tdsh_register_command(&s_modflac_cmd) == 0;
}
