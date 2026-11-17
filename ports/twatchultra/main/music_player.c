/*
 * music_player.c - the FLAC player: a playback task, a playlist, and the two
 * shell commands that drive it (`music` and its alias `modflac`).
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
#include "music_player.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "modhost.h"
#include "modload.h"
#include "sound.h"
#include "tdsh.h"

__attribute__((unused)) static const char *TAG = "music";

#define MODFLAC_IN_BYTES    (16 * 1024)
/* A FLAC block is at most 4608 frames in this decoder's configuration, and the
 * module is handed this many samples at a time. Generous so an 8192-block
 * stream still fits. */
#define MODFLAC_OUT_FRAMES  8192
/* The decoder object lives in here. It is a few hundred bytes; the buffers it
 * allocates for its own work come through the host table and go to PSRAM. */
#define MODFLAC_STATE_BYTES 4096

/*
 * THE MODULE'S HANDLE, AS WORD OFFSETS.
 *
 * `flac_handle_t` in modules/src/flacmod.cpp is the authority on this layout --
 * it is the module's own state and the module is what writes it. These offsets
 * exist because the firmware needs to read the stream rate before it opens the
 * amplifier, and because the status command reports the rest. Named once here
 * so the two places that decode the handle cannot drift apart.
 */
#define MODH_WORD_MAGIC   0
#define MODH_WORD_READY   1
#define MODH_WORD_RATE    2
#define MODH_WORD_CH      3
#define MODH_WORD_BITS    4
#define MODH_WORD_LAST    5
#define MODH_WORD_CONSUMED 6
#define MODH_WORD_CALLS   7
#define MODFLAC_WRITE_MS    1000
#define MODFLAC_STACK       8192
#define MODFLAC_PRIO        3

static char s_path[128];
static TaskHandle_t s_task;
static volatile bool s_stop;
static volatile int s_state;
static volatile uint32_t s_frames;
static volatile uint32_t s_calls;
/* The playlist entry being played, or -1 when a bare path was given. */
static volatile int s_index = -1;
static volatile int32_t s_verdict;
/* The two figures `music` used to report from its own loop, kept because they
 * are the ones that diagnose a bad output path: a decode ERROR means the module
 * rejected the stream, a SHORT WRITE means the I2S DMA stopped taking samples
 * and audio was lost. Both look like "no sound" from the outside and neither
 * does from these counters. */
static volatile uint32_t s_errors;
static volatile uint32_t s_short;

/* The module handle's first words, copied out before the block is freed so the
 * status command can explain a failure without a diagnostic file. The console
 * works now, which is cheaper than any file. */
static volatile uint32_t s_h_magic, s_h_ready;
static volatile int32_t s_h_rate, s_h_ch, s_h_bits, s_h_last;
static volatile uint32_t s_h_consumed, s_h_calls;

/*
 * THE STATE BLOCK, WHILE A TRACK IS PLAYING.
 *
 * The handle is copied into the s_h_* fields when the task ENDS, because it is
 * freed then. That left the status command reporting magic 00000000, rate 0,
 * ch 0 for the whole duration of a track that was playing perfectly -- a status
 * line that reads like a failure. Keeping the pointer lets a running track
 * report what the decoder actually parsed; it is cleared before the block is
 * freed, so a finished track reads the copy as before.
 */
static const void *s_state_live;
static char s_msg[160];   /* long enough for the loader's mismatch explanation */

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

    /* The module's handle, at the offsets flacmod.cpp writes it. */
    if (state && size >= 8 * sizeof(uint32_t))
    {
        const uint32_t *w = (const uint32_t *)state;

        fprintf(f, "handle_magic=%08lx ready=%lu\n",
                (unsigned long)w[MODH_WORD_MAGIC], (unsigned long)w[MODH_WORD_READY]);
        fprintf(f, "rate=%ld ch=%ld bits=%ld last=%ld consumed=%lu n=%lu\n",
                (long)(int32_t)w[MODH_WORD_RATE], (long)(int32_t)w[MODH_WORD_CH],
                (long)(int32_t)w[MODH_WORD_BITS], (long)(int32_t)w[MODH_WORD_LAST],
                (unsigned long)w[MODH_WORD_CONSUMED], (unsigned long)w[MODH_WORD_CALLS]);
    }
    fclose(f);
}

static void play_task(void *arg)
{
    (void)arg;

    /* Declared up here, not beside the loop: several of the early failure
     * paths jump to cleanup, and one of them is above the loop. Teardown is
     * only meaningful after a decoder could have been constructed. */
    bool teardown_ok = false;

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
        /* The loader knows exactly why -- an empty partition and a blob built
         * for a different mapping are different problems, so report its reason
         * rather than a generic one. */
        set_msg(modload_why());
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

    /* Readable by the status command from here on; cleared before the free. */
    s_state_live = state;

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

        teardown_ok = true;
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

                /*
                 * THE RATE COMES FROM THE FILE, NOT FROM A CONSTANT. The module
                 * wrote it into its handle when it parsed STREAMINFO, which is
                 * the OK-with-no-samples call just above -- so by the time a
                 * call returns samples, the true rate is here. Opening at a
                 * hard-coded 44100 made every 48 kHz file play sharp by a
                 * semitone and a half, which sounds like a decoder bug and is
                 * not one.
                 */
                const int32_t rate = ((const int32_t *)state)[MODH_WORD_RATE];

                /* A rate outside what the I2S clock can divide sensibly means
                 * the handle was not what we think it was; the common case is a
                 * better guess than a clock that will not lock. */
                if (rate < 8000 || rate > 96000)
                {
                    set_msg("odd stream rate, assuming 44100");
                    if (!sound_stream_open(44100))
                    {
                        set_msg(sound_stream_error());
                        s_state = 3;
                        break;
                    }
                }
                else if (!sound_stream_open(rate))
                {
                    /* The specific call and its error, not a generic message:
                     * see sound_stream_error(). */
                    set_msg(sound_stream_error());
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
                if (got < frames)
                    s_short++;
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
            s_errors++;
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

        s_h_magic = w[MODH_WORD_MAGIC];
        s_h_ready = w[MODH_WORD_READY];
        s_h_rate = (int32_t)w[MODH_WORD_RATE];
        s_h_ch = (int32_t)w[MODH_WORD_CH];
        s_h_bits = (int32_t)w[MODH_WORD_BITS];
        s_h_last = (int32_t)w[MODH_WORD_LAST];
        s_h_consumed = w[MODH_WORD_CONSUMED];
        s_h_calls = w[MODH_WORD_CALLS];
    }
    dump_state(state, MODFLAC_STATE_BYTES);

cleanup:
    s_state_live = NULL;

    /*
     * TELL THE MODULE TO DESTROY THE DECODER. Its working buffer lives until
     * ~FLACDecoder() and is allocated through the host table, so freeing only
     * the state BLOCK here leaks it -- measured at 41 KB of PSRAM per track,
     * which is what made the player fail after a few tracks. The entry point
     * with no input is the teardown (MODLOAD_ABI_VERSION 3).
     */
    if (state && teardown_ok)
    {
        modcall_t bye = {.in = NULL, .in_len = 0, .state = state,
                         .state_size = MODFLAC_STATE_BYTES};

        (void)fn(&bye);
    }

    if (in) heap_caps_free(in);
    if (out) heap_caps_free(out);
    if (pcm) heap_caps_free(pcm);
    if (state) heap_caps_free(state);
    fclose(f);

done:
    s_task = NULL;
    /*
     * vTaskDeleteWithCaps, NOT vTaskDelete. The contract in idf_additions.h is
     * explicit: a task created with xTaskCreateWithCaps() "must only be deleted
     * using vTaskDeleteWithCaps()". Plain vTaskDelete puts the task on the
     * termination list and the idle task frees the TCB it knows about -- but not
     * the PSRAM stack this API allocated, and not the caps-aware TCB.
     *
     * Measured cost of getting this wrong: 8,196 bytes of PSRAM and 356 bytes
     * of internal RAM leaked PER TRACK, which is what stopped playback after
     * four or five songs with the misleading message "no amplifier". The leak
     * was invisible until the module's allocations were counted separately and
     * came back clean -- the remaining 8,196/356 was exactly this.
     */
    vTaskDeleteWithCaps(NULL);
}

static void task_entry(void *arg) { play_task(arg); }

/*
 * ONE TRACK AT A TIME, AND STOP MEANS STOP.
 *
 * Starting while a track is playing is refused rather than queued: the caller
 * is either a person pressing Play on a row or the shell, and in both cases the
 * honest answer to "play that now" is to stop this one first. The UI does
 * exactly that, so the refusal is a guard against a double press rather than a
 * policy the caller has to work around.
 */
static bool start(const char *path, int index)
{
    if (s_task || strlen(path) >= sizeof(s_path))
        return false;

    snprintf(s_path, sizeof(s_path), "%s", path);
    s_index = index;
    s_stop = false;
    s_state = 1;
    s_frames = 0;
    s_calls = 0;
    s_verdict = 0;
    s_errors = 0;
    s_short = 0;
    set_msg("");

    if (xTaskCreateWithCaps(task_entry, "musicplay", MODFLAC_STACK, NULL, MODFLAC_PRIO, &s_task,
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
        printf("music: stopping\n");
        return 0;
    }

    if (argc < 2)
    {
        printf("music <file.flac> | music stop\n");
        if (s_task || s_state >= 2)
        {
            printf("state %s, %lu frames, %lu errors, %lu short writes"
                   "%s%s\n",
                   s_state == 1 ? "playing" : (s_state == 2 ? "done" : "error"),
                   (unsigned long)s_frames, (unsigned long)s_errors,
                   (unsigned long)s_short, s_msg[0] ? ": " : "", s_msg);
            /*
             * The module handle, which is what says WHY a decode failed -- and
             * what it actually parsed when one is working. While a track runs it
             * is read from the block itself; the s_h_* copy is only filled in
             * when the task ends and the block is freed.
             */
            uint32_t w_magic = s_h_magic, w_ready = s_h_ready, w_calls = s_h_calls;
            int32_t w_rate = s_h_rate, w_ch = s_h_ch, w_bits = s_h_bits;
            int32_t w_last = s_h_last;
            uint32_t w_consumed = s_h_consumed;
            const void *live = s_state_live;

            if (live)
            {
                const uint32_t *w = (const uint32_t *)live;

                w_magic = w[MODH_WORD_MAGIC];
                w_ready = w[MODH_WORD_READY];
                w_rate = (int32_t)w[MODH_WORD_RATE];
                w_ch = (int32_t)w[MODH_WORD_CH];
                w_bits = (int32_t)w[MODH_WORD_BITS];
                w_last = (int32_t)w[MODH_WORD_LAST];
                w_consumed = w[MODH_WORD_CONSUMED];
                w_calls = w[MODH_WORD_CALLS];
            }

            printf("handle: magic %08lx ready %lu rate %ld ch %ld bits %ld\n",
                   (unsigned long)w_magic, (unsigned long)w_ready, (long)w_rate,
                   (long)w_ch, (long)w_bits);
            printf("        last %ld consumed %lu in-calls %lu, %lu decode calls\n",
                   (long)w_last, (unsigned long)w_consumed, (unsigned long)w_calls,
                   (unsigned long)s_calls);
        }
        return 0;
    }

    if (!start(argv[1], -1))
    {
        printf("music: cannot start\n");
        return 1;
    }
    printf("music: playing %s\n", argv[1]);
    return 0;
}

/*
 * TWO NAMES, ONE PLAYER. `music` is the user-facing command and `modflac` is
 * kept as its alias because every log entry, script and habit in this project
 * refers to it, and because when something goes wrong it is useful to have a
 * name that says "this is the path that goes through the partition". Before
 * this they were two separate implementations of the same loop -- music.c with
 * the decoder linked in, modflac.c with it in flash -- which is why the app
 * carried both copies.
 */
static const tdsh_command_t s_modflac_cmd = {
    "music",
    "music <file.flac> | music stop",
    "Play a FLAC through the decoder in the modules partition",
    cmd_modflac,
    TDSH_CMD_INTERACTIVE,
};

static const tdsh_command_t s_modflac_alias_cmd = {
    "modflac",
    "modflac <file.flac> | modflac stop",
    "Alias of `music`, naming the modules partition it decodes from",
    cmd_modflac,
    TDSH_CMD_INTERACTIVE,
};

/* Stops the task if one is running, then rewrites s_path. Used by both the
 * playlist and a bare path so there is one place that decides what plays. */
static bool play(const char *path, int index)
{
    if (s_task)
    {
        s_stop = true;
        /* The task clears s_task itself; wait briefly so the next open does not
         * race the amplifier close. Two frames of audio is enough. */
        for (int i = 0; i < 200 && s_task; i++)
            vTaskDelay(pdMS_TO_TICKS(10));
    }
    return start(path, index);
}

/* ------------------------------------------------------------- playlist */

/*
 * THE NAMES LIVE IN PSRAM, AND THEY HAVE TO.
 *
 * 256 entries of 72 bytes is 18 KB, which as a static array would be internal
 * DRAM -- the pool this board is short of, and the same mistake that once
 * pushed `ssh start` under its floor when the Tamagotchi's dirty-row tables
 * were .bss. A static array here would also be paid whether or not anyone opens
 * the Music window, so the pool is heap-allocated on the first scan.
 *
 * The names are what the list shows and what a path is built from: the
 * directory is stored once, so a track is dir + "/" + name at play time. That
 * keeps the pool small and means a card whose directory is renamed needs only a
 * rescan, not 256 paths rewritten.
 */
static char *s_names;
static int s_name_count;
static char s_dir[128];

/* One entry from readdir: kept when it is a .flac and the pool has room. */
static bool track_add(const char *name)
{
    /* .flac only, case-insensitively: the card has been written from a PC and
     * from this watch, and a .FLAC that the decoder would play must be listed. */
    const size_t n = strlen(name);
    static const char ext[] = ".flac";

    if (n <= sizeof(ext) - 1)
        return false;
    if (strcasecmp(name + n - (sizeof(ext) - 1), ext) != 0)
        return false;
    if (s_name_count >= MUSIC_TRACK_MAX)
        return false;

    char *slot = s_names + (size_t)s_name_count * MUSIC_NAME_MAX;

    snprintf(slot, MUSIC_NAME_MAX, "%s", name);
    s_name_count++;
    return true;
}

int music_tracks_scan(const char *dir)
{
    if (!dir || !dir[0])
        return -1;

    if (!s_names)
    {
        s_names = heap_caps_malloc((size_t)MUSIC_TRACK_MAX * MUSIC_NAME_MAX,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_names)
            return -1;
    }

    /* A scan invalidates the index, so a playing track is stopped first. */
    if (s_task)
    {
        s_stop = true;
        for (int i = 0; i < 200 && s_task; i++)
            vTaskDelay(pdMS_TO_TICKS(10));
        s_index = -1;
    }

    s_name_count = 0;
    snprintf(s_dir, sizeof(s_dir), "%s", dir);

    DIR *d = opendir(dir);

    if (!d)
        return 0;      /* no card, no directory: an empty list, not an error */

    struct dirent *e;

    while ((e = readdir(d)) != NULL)
        track_add(e->d_name);
    closedir(d);

    /*
     * SORTED, because readdir order on FAT is the directory's on-disk order --
     * which is the order the files were copied, so a card written by an album
     * ripper lists 01, 10, 11, 02. A playlist is read by a person.
     *
     * Insertion sort: 256 entries at most, run once per scan, and it costs no
     * stack worth measuring.
     */
    for (int i = 1; i < s_name_count; i++)
    {
        char tmp[MUSIC_NAME_MAX];

        snprintf(tmp, sizeof(tmp), "%s", s_names + (size_t)i * MUSIC_NAME_MAX);
        int j = i - 1;

        while (j >= 0 && strcasecmp(s_names + (size_t)j * MUSIC_NAME_MAX, tmp) > 0)
        {
            memcpy(s_names + (size_t)(j + 1) * MUSIC_NAME_MAX,
                   s_names + (size_t)j * MUSIC_NAME_MAX, MUSIC_NAME_MAX);
            j--;
        }
        memcpy(s_names + (size_t)(j + 1) * MUSIC_NAME_MAX, tmp, MUSIC_NAME_MAX);
    }

    return s_name_count;
}

int music_track_count(void)
{
    return s_name_count;
}

const char *music_track_name(int index)
{
    if (!s_names || index < 0 || index >= s_name_count)
        return "";
    return s_names + (size_t)index * MUSIC_NAME_MAX;
}

const char *music_tracks_dir(void)
{
    return s_dir;
}

const char *music_track_path(int index)
{
    static char path[sizeof(s_dir) + MUSIC_NAME_MAX + 2];

    if (!s_dir[0] || index < 0 || index >= s_name_count)
        return NULL;

    snprintf(path, sizeof(path), "%s/%s", s_dir, music_track_name(index));
    return path;
}

/* --------------------------------------------------------------- player */

bool music_play_index(int index)
{
    const char *path = music_track_path(index);

    if (!path)
        return false;
    return play(path, index);
}

bool music_play_path(const char *path)
{
    if (!path || !path[0])
        return false;
    return play(path, -1);
}

void music_stop(void)
{
    s_stop = true;
}

void music_status_get(music_status_t *out)
{
    if (!out)
        return;

    out->running = s_task != NULL;
    out->state = s_state;
    out->index = s_index;
    out->frames = s_frames;

    /*
     * THE RATE HAS TO COME FROM THE LIVE HANDLE WHILE A TRACK PLAYS.
     *
     * s_h_rate is only copied out of the caller-owned state block when the task
     * ENDS, because that is when the block is freed. Reading it here during
     * playback gave 0 -- which made the elapsed time 0 as well, so the UI's
     * clock sat at 00:00 for a whole track. The self-test caught it: frames
     * 253440 with rate 0 is impossible, and only one of those figures was wrong.
     */
    out->rate = (uint32_t)s_h_rate;

    if (s_state_live)
    {
        const uint32_t *w = (const uint32_t *)s_state_live;

        /* 0x464C4143 is "FLAC", the magic flacmod.cpp writes into the handle. A
         * block that is being constructed has not got it yet, and its other
         * words are garbage. */
        if (w[MODH_WORD_MAGIC] == 0x464C4143U)
            out->rate = (uint32_t)w[MODH_WORD_RATE];
    }
    /* Frames over rate is the elapsed time, and it is the only clock the
     * decoder gives: FLAC header durations are not always present and a
     * progress bar that freezes is worse than no bar. */
    out->seconds = out->rate ? out->frames / out->rate : 0;
    out->errors = s_errors;
    out->short_writes = s_short;
    out->message = s_msg;
}

bool music_player_register_commands(void)
{
    return tdsh_register_command(&s_modflac_cmd) == 0 &&
           tdsh_register_command(&s_modflac_alias_cmd) == 0;
}
