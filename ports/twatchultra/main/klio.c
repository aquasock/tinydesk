/*
 * klio.c - KLIO, the learning gesture engine inside the BHI260AP.
 *
 * WHAT KLIO IS, because it is not what the other sensors are. Everything else
 * this port reads from the part is a MEASUREMENT -- acceleration, orientation, a
 * step count. KLIO is a CLASSIFIER that learns from the wearer:
 *
 *   1. learning is enabled, and the wearer performs a gesture repeatedly
 *   2. the part reports progress 0..100 as it accumulates repetitions
 *   3. at 100 it announces a new pattern and hands back an index
 *   4. with recognition enabled it then reports "pattern N, repetition count X"
 *
 * There is no training data, no framework and no network. The classifier is a
 * state machine on the Fuser core, and the only thing it costs the ESP32 is the
 * bytes that describe what it concluded.
 *
 * THIS FILE IS AN INSTRUMENT, NOT A FEATURE. It exists so the engine can be
 * exercised with a human hand before anything is built on top of it -- the same
 * way acceleration was proven on the glass before a window was drawn around it.
 * The subcommands are read-mostly; the only destructive ones are `learn` and
 * `reset`, and both say what they do before they do it.
 *
 * THE TWO UNDOCUMENTED THINGS, stated up front because they will be the first
 * things to check when something does not work:
 *   - the payload format of a KLIO output frame is taken from the struct
 *     definitions in bhy2_klio_defs.h, not from Bosch's prose
 *   - KLIO_PARAM_INPUT_SENSORS says which sensors feed the algorithm; this port
 *     reads it but does not set it, so it reports what the firmware chose
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh.h"

#include "esp_heap_caps.h"

#include "bhy2.h"
#include "bhy2_defs.h"
#include "bhy2_klio.h"
#include "bhy2_klio_defs.h"
#include "imu.h"
#include "klio_store.h"

static const char *TAG = "klio";

/* ------------------------------------------------------------ the state */

static bhy2_klio_sensor_state_t s_state;
static bool s_have_state;

/* The last frame the algorithm sent, decoded by the callback below. */
typedef struct
{
    volatile bool valid;
    int8_t learn_index;        /* -1: nothing new learnt */
    uint8_t learn_progress;    /* 0..100 while learning */
    uint8_t learn_reason;      /* 0 ok, 1 interrupted, 2 no movement */
    uint8_t recog_index;       /* 255: nothing recognised */
    float recog_count;
    uint32_t frames;
} klio_frame_t;

static klio_frame_t s_last;

/* Per-frame byte trace, off by default. See the note in on_klio(). */
static bool s_trace = false;

/* THE PATTERN BANK IS PSRAM, AND THE FILE IS THE DURABLE COPY OF IT.
 *
 * `klio learn` sets learning_reset = 1, which clears the engine's own pattern,
 * so a bank has to exist for a saved pattern to survive a second learning pass.
 * It also has to survive a REBOOT, which the part cannot do for us: the BHI260AP
 * runs firmware the host uploads into its RAM, so its pattern table starts empty
 * every boot (IMU-001). The bank is therefore mirrored to a file in littlefs,
 * and `klio_after_boot()` puts it back into the engine once the firmware is up.
 *
 * PSRAM RATHER THAN .bss, which is the whole reason this is a heap allocation.
 * The store is 32 slots of 264 bytes -- 8,448 -- and EMU-018 records that 4,148
 * bytes of static .bss once starved the SSH server of its internal-RAM floor.
 * The previous bank was 2 KB of .bss; growing it in place would have been the
 * same mistake with a bigger number. Allocated once, on first use.
 *
 * The slot count is the PART's (25 here, read from the part rather than
 * assumed), capped at the store format's limit. */
static klio_store_t *s_store;

#define KLIO_STORE_PATH "/fs/root/klio_patterns.bin"

/* Room for the largest file the format can describe, plus the frame, and a
 * little slack so a future format change fails on version rather than space. */
#define KLIO_STORE_FILE_MAX \
    (KLIO_STORE_MAX * (1 + 1 + 2 + KLIO_STORE_NAME_MAX + KLIO_STORE_BLOB_MAX) + 16)

/* Carries the per-cycle cross-axis amount from the generator to the frame. */
static volatile float s_tilt = 0.1f;

/* The index the engine assigned when it last learnt a pattern, so `detect` can
 * enable exactly that slot rather than guessing. -1 until something is learnt. */
static int s_learnt = -1;

/* ---------------------------------------------------------- the callback */

/* A KLIO frame carries BOTH halves every time: the learning half says how far
 * training has got, the recognition half says what it currently thinks you are
 * doing. Decoding only the half you expect is how "it does nothing" happens. */
static void on_klio(const struct bhy2_fifo_parse_data_info *info, void *priv)
{
    (void)priv;

    if (info->data_size < 1)
        return;

    const uint8_t *d = info->data_ptr;
    const size_t n = info->data_size;

    /* The real frame is bhy2_klio_sensor_frame_t: learning {instance, index,
     * progress, change_reason} then recognition {instance, index, count}. Sizes
     * are checked before every read because a firmware variant that sends a
     * shorter frame must not be read past its end. */
    if (n >= 4)
    {
        s_last.learn_index = (int8_t)d[1];
        s_last.learn_progress = d[2];
        s_last.learn_reason = d[3];
    }
    if (n >= 4 + 6)
    {
        /* recognition { uint8 instance; uint8 index; float count; } */
        s_last.recog_index = d[5];
        memcpy(&s_last.recog_count, d + 6, sizeof(float));
    }
    s_last.frames++;
    s_last.valid = true;

    /* A COMPACT TRACE OF EVERYTHING THAT ARRIVES. Without this, "no frames"
     * and "frames we failed to decode" look identical from the command line,
     * and the first `klio detect` could not be diagnosed for that reason. */
    /* ALL ELEVEN BYTES. The first version printed six and that was not enough:
     * the frame is longer than the two packed structs in bhy2_klio_defs.h
     * account for (4 + 6 = 10), so one byte is somewhere unexpected and the
     * recognition index may not be where the struct layout suggests. Printing
     * the whole frame is the only way to place it with evidence. */
    /* AT DEBUG, NOT INFO. At INFO this printed one line per frame -- 50 a
     * second while learning -- which scrolled the progress lines and the final
     * verdict off a 26-row terminal. An instrument whose output cannot be read
     * is not an instrument. `klio trace on` turns it back up when the bytes are
     * what is in question. */
    if (s_trace)
    {
        char hex[3 * 24 + 1];
        size_t pos = 0;
        for (size_t i = 0; i < n && pos + 3 < sizeof(hex); i++)
            pos += (size_t)snprintf(hex + pos, sizeof(hex) - pos, "%02x ", d[i]);
        ESP_LOGI(TAG, "id %u len %u: %s", (unsigned)info->sensor_id, (unsigned)n, hex);
    }
}

/* ------------------------------------------------------------- helpers */

/* Log a return code with a name, because "8" is not an answer. */
static void say_rc(const char *what, int8_t rc)
{
    if (rc == BHY2_OK)
        printf("  %s: ok\n", what);
    else
        printf("  %s: error %d\n", what, rc);
}

/* A return code as a word, for the messages that print rather than log. */
static const char *rc_name(int8_t rc)
{
    switch (rc)
    {
    case BHY2_OK:                 return "ok";
    case BHY2_E_NULL_PTR:         return "null pointer";
    case BHY2_E_INVALID_PARAM:    return "invalid param";
    case BHY2_E_IO:               return "io";
    case BHY2_E_MAGIC:            return "bad magic";
    case BHY2_E_TIMEOUT:          return "timeout";
    case BHY2_E_BUFFER:           return "buffer";
    case BHY2_E_INVALID_FIFO_TYPE:return "bad fifo type";
    case BHY2_E_INVALID_EVENT_SIZE:return "bad event size";
    case BHY2_E_PARAM_NOT_SET:    return "param not set";
    default:                      return "error";
    }
}

/* Turn learning and recognition on or off in one call, with the reset flags
 * spelled out. Every caller here needs a different combination and the struct
 * zero-initialises to "everything off, everything reset", which is a trap that
 * has already cost one cycle -- a stray reset deletes the pattern table. */
static int8_t change_state(int learn, int learn_reset, int recog, int recog_reset)
{
    bhy2_klio_sensor_state_t st = {
        .learning_enabled = (uint8_t)(learn ? 1 : 0),
        .learning_reset = (uint8_t)(learn_reset ? 1 : 0),
        .recognition_enabled = (uint8_t)(recog ? 1 : 0),
        .recognition_reset = (uint8_t)(recog_reset ? 1 : 0),
    };
    return bhy2_klio_set_state(&st, imu_dev());
}

/* Read a KLIO parameter and report it. Each one has its own size, documented in
 * bhy2_klio_defs.h, so the buffer is sized for the largest (244). */
static bool read_param(const char *label, bhy2_klio_parameter_t id, uint16_t want)
{
    uint8_t buf[260];
    uint16_t len = want;

    memset(buf, 0, sizeof(buf));
    const int8_t rc = bhy2_klio_get_parameter(id, buf, &len, imu_dev());

    if (rc != BHY2_OK)
    {
        printf("  %-24s error %d\n", label, rc);
        return false;
    }

    if (want == 1)
        printf("  %-24s %u\n", label, (unsigned)buf[0]);
    else if (want == 2)
    {
        uint16_t v;
        memcpy(&v, buf, 2);
        printf("  %-24s %u\n", label, (unsigned)v);
    }
    else if (want == 4)
    {
        float f;
        memcpy(&f, buf, 4);
        printf("  %-24s %.3f\n", label, (double)f);
    }
    else
    {
        /* The version string is 244 bytes of text. */
        printf("  %-24s %.40s\n", label, (const char *)buf);
    }
    return true;
}

/* ------------------------------------------------------- the pattern bank */

/* The bank, allocated on first use in PSRAM. See the note at s_store for why
 * this is not a static array. */
static klio_store_t *store(void)
{
    if (!s_store)
    {
        s_store = heap_caps_calloc(1, sizeof(*s_store), MALLOC_CAP_SPIRAM);
        if (!s_store)
        {
            ESP_LOGE(TAG, "no PSRAM for the pattern bank");
            return NULL;
        }
        klio_store_init(s_store);
    }
    return s_store;
}

/* How many slots the part offers. Read from the part, because the number is a
 * property of the firmware image it is running rather than of this build, and
 * fall back to the 25 the part reports here only when the read fails. */
int klio_max_patterns(void)
{
    if (!imu_dev())
        return 0;

    uint8_t buf[2];
    uint16_t len = sizeof(buf);
    memset(buf, 0, sizeof(buf));

    int n = 25;
    if (bhy2_klio_get_parameter(KLIO_PARAM_RECOGNITION_MAX_PATTERNS, buf, &len,
                                imu_dev()) == BHY2_OK)
    {
        uint16_t v;
        memcpy(&v, buf, 2);
        if (v > 0)
            n = (int)v;
    }
    if (n > KLIO_STORE_MAX)
        n = KLIO_STORE_MAX;
    return n;
}

/* Write the bank to flash, atomically.
 *
 * TEMP FILE THEN RENAME, because the alternative is a power cut during a write
 * leaving a half-written file that decodes to nothing -- the very state
 * persistence exists to prevent. littlefs makes the rename atomic, so a reader
 * sees either the old file or the new one and never a mixture. */
static bool store_save_file(void)
{
    const klio_store_t *bank = store();
    if (!bank)
        return false;

    uint8_t *buf = heap_caps_malloc(KLIO_STORE_FILE_MAX, MALLOC_CAP_SPIRAM);
    if (!buf)
    {
        ESP_LOGE(TAG, "no PSRAM to encode the pattern store");
        return false;
    }

    size_t len = 0;
    const int rc = klio_store_encode(bank, buf, KLIO_STORE_FILE_MAX, &len);
    if (rc != KLIO_STORE_OK)
    {
        ESP_LOGE(TAG, "cannot encode the pattern store: %s", klio_store_strerror(rc));
        free(buf);
        return false;
    }

    static const char *tmp = KLIO_STORE_PATH ".tmp";
    FILE *f = fopen(tmp, "wb");
    if (!f)
    {
        ESP_LOGE(TAG, "cannot write %s", tmp);
        free(buf);
        return false;
    }
    const size_t wrote = fwrite(buf, 1, len, f);
    const bool flushed = fflush(f) == 0;
    fclose(f);
    free(buf);

    if (wrote != len || !flushed)
    {
        ESP_LOGE(TAG, "short write to %s (%u of %u bytes)", tmp,
                 (unsigned)wrote, (unsigned)len);
        remove(tmp);
        return false;
    }
    if (rename(tmp, KLIO_STORE_PATH) != 0)
    {
        ESP_LOGE(TAG, "cannot rename %s into place", tmp);
        remove(tmp);
        return false;
    }
    return true;
}

/* Read the file into the bank. 1 when patterns were loaded, 0 when there is no
 * file, -1 when the file exists and was refused -- the distinction matters
 * because "none stored" and "corrupt" deserve different words. */
static int store_load_file(void)
{
    klio_store_t *bank = store();
    if (!bank)
        return -1;

    FILE *f = fopen(KLIO_STORE_PATH, "rb");
    if (!f)
        return 0;

    uint8_t *buf = heap_caps_malloc(KLIO_STORE_FILE_MAX, MALLOC_CAP_SPIRAM);
    if (!buf)
    {
        fclose(f);
        ESP_LOGE(TAG, "no PSRAM to read the pattern store");
        return -1;
    }

    const size_t got = fread(buf, 1, KLIO_STORE_FILE_MAX, f);
    /* A file longer than the format can describe cannot be a file this build
     * wrote; refuse it rather than decode the prefix of it. */
    const bool overlong = (got == KLIO_STORE_FILE_MAX) && (fgetc(f) != EOF);
    fclose(f);

    if (overlong)
    {
        free(buf);
        ESP_LOGW(TAG, "pattern store is larger than the format allows");
        return -1;
    }

    const int rc = klio_store_decode(bank, buf, got);
    free(buf);
    if (rc != KLIO_STORE_OK)
    {
        ESP_LOGW(TAG, "pattern store refused: %s (%d)", klio_store_strerror(rc), rc);
        return -1;
    }
    if (bank->count == 0)
        return 0;
    ESP_LOGI(TAG, "pattern store: %d pattern(s)", bank->count);
    return 1;
}

/* Put every banked pattern back into the engine and enable it.
 *
 * QUIESCE FIRST. write_pattern and set_pattern_states are several blocking I2C
 * transactions each, and a restore can run while the sensors stream; a FIFO
 * overflow here drops packets mid-transfer and is how a pattern once came back
 * scoring -8.837. */
static bool store_restore_into_part(bool verbose)
{
    const klio_store_t *bank = store();
    if (!bank || !imu_dev())
        return false;

    imu_quiesce();
    change_state(0, 0, 0, 0);
    imu_drain_hard();

    int ok = 0;
    for (int i = 0; i < bank->count; i++)
    {
        const klio_store_rec_t *r = &bank->rec[i];
        const int8_t w = bhy2_klio_write_pattern(r->slot, r->blob, r->len, imu_dev());
        imu_drain_hard();
        const uint8_t id = r->slot;
        const int8_t e = bhy2_klio_set_pattern_states(KLIO_PATTERN_STATE_ENABLE,
                                                      &id, 1, imu_dev());
        imu_drain_hard();
        if (w == BHY2_OK && e == BHY2_OK)
            ok++;
        if (verbose)
            printf("  slot %u (%u B): write %s, enable %s\n", (unsigned)r->slot,
                   (unsigned)r->len, rc_name(w), rc_name(e));
        else
            ESP_LOGI(TAG, "slot %u (%u B): write %s, enable %s",
                     (unsigned)r->slot, (unsigned)r->len, rc_name(w), rc_name(e));
    }

    if (verbose)
        printf("%d of %d stored pattern(s) restored\n", ok, bank->count);
    else
        ESP_LOGI(TAG, "restored %d of %d pattern(s) after boot", ok, bank->count);

    return ok == bank->count;
}

/* Called by imu_start() through a weak symbol, once the part's firmware is up.
 *
 * EVERY BOOT STARTS WITH AN EMPTY ENGINE. The BHI260AP's firmware is uploaded
 * into its RAM by this host, so nothing in the pattern table survives a reset --
 * which is why every entry before this one recorded "patterns do not survive a
 * reboot" as a standing limitation. This is the fix: the 157-byte blobs are in
 * littlefs, and they go back in here.
 *
 * It must never be able to stop the sensor starting, so every failure path below
 * logs and returns. */
void klio_after_boot(void)
{
    if (!imu_dev())
        return;

    if (store_load_file() <= 0)
        return;                 /* nothing stored, or already logged and refused */

    (void)store_restore_into_part(false);
}

/* ------------------------------------------------------------ the command */

static int cmd_klio(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;
    const char *sub = argc >= 2 ? argv[1] : "";

    if (!imu_start())
    {
        printf("klio: %s\n", imu_why());
        return 1;
    }

    /* ---------------------------------------------------------- status */
    if (strcmp(sub, "status") == 0 || sub[0] == '\0')
    {
        s_have_state = bhy2_klio_get_state(&s_state, imu_dev()) == BHY2_OK;

        printf("KLIO learning engine\n");
        read_param("algorithm version", KLIO_PARAM_ALGORITHM_VERSION, 244);

        if (s_have_state)
            printf("  %-24s learning %s, recognition %s\n", "state",
                   s_state.learning_enabled ? "ON" : "off",
                   s_state.recognition_enabled ? "ON" : "off");
        else
            printf("  state: unreadable\n");

        read_param("max patterns", KLIO_PARAM_RECOGNITION_MAX_PATTERNS, 2);
        read_param("pattern blob size", KLIO_PARAM_PATTERN_BLOB_SIZE, 2);
        read_param("input sensors", KLIO_PARAM_INPUT_SENSORS, 1);
        printf("  frames received          %lu\n", (unsigned long)s_last.frames);
        printf("\n  klio learn    train a gesture with your hand\n");
        printf("  klio detect   learn-then-recognise in one step\n");
        printf("  klio recognise test a SAVED pattern (no learn slot)\n");
        printf("  klio stop     both off\n");
        printf("  klio save <n> keep the learnt pattern in slot n, in flash\n");
        printf("  klio stored   what is kept in flash\n");
        printf("  klio restore  put the kept patterns back into the engine\n");
        printf("  klio forget <n>  drop a kept pattern\n");

        /* What is on flash, reported with the state, because "is my gesture
         * still there" is the question a person asks after a restart and the
         * engine's own table cannot answer it. */
        {
            const int have = store_load_file();
            klio_store_t *bank = store();
            if (bank && have > 0)
                printf("  kept in flash            %d pattern(s)\n", bank->count);
            else if (have == 0)
                printf("  kept in flash            none\n");
            else
                printf("  kept in flash            unreadable (see log)\n");
        }
        return 0;
    }

    /* ----------------------------------------------------------- learn */
    if (strcmp(sub, "learn") == 0)
    {
        bhy2_klio_sensor_state_t st = {
            .learning_enabled = 1,
            .learning_reset = 1,      /* forget any half-trained pattern first */
            .recognition_enabled = 0,
            .recognition_reset = 0,
        };

        /* KLIO IS AN ALGORITHM, NOT A SENSOR -- IT CONSUMES THE PHYSICAL ONES.
         *
         * THE FIRST VERSION OF THIS PROBE ENABLED KLIO AND NOTHING ELSE, and the
         * engine then reported zero frames for a whole minute of a human waving
         * at it. The reason is obvious once stated: a classifier that learns
         * from motion has no motion to look at unless the accelerometer and
         * gyroscope are running and feeding it. Enabling them is not a
         * workaround for a bug, it is the prerequisite.
         *
         * 100 Hz on both, which is the rate these algorithms expect; report
         * latency 20 ms so samples arrive in a steady stream rather than in
         * FIFO-sized lumps. */
        (void)imu_stream_on(100.0f);
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, 100.0f, 20, imu_dev());

        /* Subscribe BEFORE enabling, or the first frames are missed. */
        (void)bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_KLIO, on_klio, NULL,
                                                imu_dev());
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_KLIO, 50.0f, 20, imu_dev());

        /* CLEAR THE LAST FRAME BEFORE STARTING, and this is not tidiness. The
         * struct zero-initialises, and learn_index defaults to 0 -- which the
         * success test `>= 0` reads as "pattern 0 was learnt". The first run of
         * this probe therefore announced a learnt pattern in under a second,
         * with no gesture at all, which is exactly the sort of confident false
         * positive a diagnostic must never produce. -1 means "nothing new". */
        memset((void *)&s_last, 0, sizeof(s_last));
        s_last.learn_index = -1;
        s_last.recog_index = 255;
        const uint32_t frames_before = s_last.frames;

        say_rc("reset + learning on", bhy2_klio_set_state(&st, imu_dev()));

        printf("\nperform one gesture over and over -- the same motion each time.\n");
        printf("watch `progress` climb to 100, then press Ctrl+C.\n\n");

        int last_pct = -1;
        int8_t learnt = -1;
        for (int i = 0; i < 600; i++)      /* 600 x 100 ms = up to a minute */
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            (void)imu_poll();

            /* ONLY A REAL FRAME MAY PRINT PROGRESS. s_last is zeroed before the
             * loop, so an unconditional first print announces 0% from a struct
             * that no packet has touched -- the same false-confidence failure
             * the learnt-index check had. */
            if (s_last.frames != frames_before && s_last.learn_progress != last_pct)
            {
                last_pct = s_last.learn_progress;
                printf("  progress %3d%%%s\n", last_pct,
                       s_last.learn_reason ? "  (interrupted)" : "");
            }
            /* Only a FRAME THAT ACTUALLY ARRIVED may report a new pattern, and
             * only once progress has been seen to reach 100. Both conditions,
             * because either one alone can be satisfied by a stale field. */
            if (s_last.frames != frames_before && s_last.learn_index >= 0 &&
                s_last.learn_progress >= 100)
            {
                learnt = s_last.learn_index;
                s_learnt = learnt;
                break;
            }
        }

        bhy2_klio_sensor_state_t off = {0};
        off.learning_reset = 0;
        (void)bhy2_klio_set_state(&off, imu_dev());

        printf("\nframes from the engine: %lu\n",
               (unsigned long)(s_last.frames - frames_before));
        if (learnt >= 0)
            printf("LEARNT pattern %d -- `klio detect` will recognise it now\n", learnt);
        else if (s_last.frames == frames_before)
            printf("no frames at all: the engine is not reporting. It is configured\n"
                   "and subscribed, so the next thing to check is whether this\n"
                   "firmware routes KLIO output to the FIFO without an explicit\n"
                   "input-sensor setup (that parameter returned -2).\n");
        else
            printf("no pattern learnt. progress reached %u%% and stopped, so the\n"
                   "part did not see the motion as repetitive enough.\n",
                   (unsigned)s_last.learn_progress);
        return 0;
    }

    /* ------------------------------------------------------------ save */
    /* PULL THE JUST-LEARNT PATTERN OUT AND PARK IT IN A SLOT.
     *
     * The whole certainty experiment rests on this: learning leaves a 157-byte
     * fingerprint inside the algorithm, and until it is read out and written
     * into the recognition table it exists nowhere the host can reach. Park one
     * per variant and they can all be compared afterwards, in one call, without
     * having to redo the motions.
     *
     *   klio save 0     after learning the reference
     *   klio save 1     after learning a 70% amplitude variant
     */
    /* ------------------------------------------------------------ save */
    /* Read the learnt pattern out of the engine, write it to a slot, and keep a
     * DURABLE copy -- which is the part that did not exist before: slot 0 of the
     * engine lives in the part's RAM and dies with the power, while the file
     * this writes is in littlefs and is reloaded by klio_after_boot() at the next
     * start. */
    if (strcmp(sub, "save") == 0)
    {
        const int max_slots = klio_max_patterns();
        const int slot = argc >= 3 ? atoi(argv[2]) : 0;

        if (max_slots <= 0)
        {
            printf("klio save: the part has not reported its slot count\n");
            return 1;
        }
        if (slot < 0 || slot >= max_slots)
        {
            printf("klio save <0..%d>\n", max_slots - 1);
            return 1;
        }

        /* QUIESCE FIRST. read_pattern, write_pattern and set_pattern_states are
         * several blocking I2C calls each, and the learn pass leaves accel and
         * gyro streaming at 100 Hz. Without this the FIFO overflows mid-save
         * (`meta 12 fifo overflow`) and what lands in the slot is whatever
         * survived -- which is how a pattern came back scoring -8.837. */
        imu_quiesce();
        change_state(0, 0, 0, 0);
        imu_drain_hard();

        uint8_t pattern[244];
        uint16_t plen = sizeof(pattern);
        int8_t rc = bhy2_klio_read_pattern(0, pattern, &plen, imu_dev());
        imu_drain_hard();
        /* The blob size the part itself reports, read rather than assumed: a
         * short read means the pattern is not complete and must not be saved. */
        printf("  read %u bytes\n", (unsigned)plen);
        if (rc == BHY2_OK && plen != 157)
            printf("  WARNING: expected 157, got %u -- saving anyway\n",
                   (unsigned)plen);
        if (rc != BHY2_OK || plen == 0 || plen > sizeof(pattern))
        {
            printf("read_pattern failed: %s (%d), %u bytes\n", rc_name(rc), rc,
                   (unsigned)plen);
            printf("learn a pattern first -- `klio learn` -- then save it.\n");
            return 1;
        }

        imu_drain_hard();
        rc = bhy2_klio_write_pattern((uint8_t)slot, pattern, plen, imu_dev());
        imu_drain_hard();
        printf("slot %d <- %u bytes: %s\n", slot, (unsigned)plen, rc_name(rc));
        if (rc != BHY2_OK)
            return 1;

        /* Written patterns are DISABLED until enabled, and a similarity
         * calculation reads the table rather than the engine, so the slot has
         * to be enabled for the comparison to have anything to chew on. */
        const uint8_t id = (uint8_t)slot;
        rc = bhy2_klio_set_pattern_states(KLIO_PATTERN_STATE_ENABLE, &id, 1,
                                          imu_dev());
        imu_drain_hard();
        printf("slot %d enabled: %s\n", slot, rc_name(rc));

        /* NOW THE FILE, and only after the engine accepted the pattern: storing
         * a blob the part refused would fill the file with something that can
         * never match, and the failure would only show up at the next boot. */
        klio_store_t *bank = store();
        if (!bank)
            return 1;

        /* LOAD THE FILE FIRST IF THE BANK IS EMPTY, because the alternative is
         * clobbering patterns that are on flash. The restore at imu_start()
         * normally leaves the bank populated, so this is a guard rather than the
         * usual path -- but a save that silently reduced a four-pattern file to
         * one because this session happened never to restore first would be
         * exactly the kind of loss persistence exists to prevent. */
        if (bank->count == 0)
            (void)store_load_file();

        klio_store_rec_t r;
        memset(&r, 0, sizeof(r));
        r.slot = (uint8_t)slot;
        r.flags = KLIO_STORE_FLAG_ENABLED;
        r.len = plen;
        memcpy(r.blob, pattern, plen);

        if (klio_store_put(bank, &r) < 0)
        {
            printf("  the pattern bank is full -- `klio forget <slot>` first\n");
            return 1;
        }
        if (store_save_file())
            printf("  kept in %s (%d pattern(s))\n", KLIO_STORE_PATH, bank->count);
        else
            printf("  WARNING: the part has it, but %s could not be written\n",
                   KLIO_STORE_PATH);

        return rc == BHY2_OK ? 0 : 1;
    }

    /* --------------------------------------------------------- restore */
    /* Reload the file and put every pattern back into the engine.
     *
     * This is also what happens automatically at boot (klio_after_boot), so the
     * command exists for the case where the table was cleared underneath a
     * running session -- `klio learn` resets the engine's state, and this is how
     * a stored pattern gets back in without a reboot. */
    if (strcmp(sub, "restore") == 0)
    {
        const int have = store_load_file();
        if (have == 0)
        {
            printf("no stored patterns at %s\n", KLIO_STORE_PATH);
            printf("`klio save <slot>` after a learn writes one there.\n");
            return 1;
        }
        if (have < 0)
        {
            printf("%s exists but was refused -- see the log\n", KLIO_STORE_PATH);
            return 1;
        }
        return store_restore_into_part(true) ? 0 : 1;
    }

    /* ---------------------------------------------------------- forget */
    /* Keep the gesture out of the file AND out of the engine.
     *
     * Both halves matter: dropping only the file would leave the pattern
     * matching for as long as the watch stayed on, which reads as "forget does
     * not work"; dropping only the engine would bring it back at the next boot,
     * which reads as "it came back". */
    if (strcmp(sub, "forget") == 0)
    {
        const int slot = argc >= 3 ? atoi(argv[2]) : -1;
        if (slot < 0 || slot > 255)
        {
            printf("klio forget <slot>\n");
            return 1;
        }

        (void)store_load_file();
        klio_store_t *bank = store();
        if (!bank)
            return 1;

        if (!klio_store_drop(bank, slot))
        {
            printf("slot %d is not stored\n", slot);
            return 1;
        }

        imu_quiesce();
        const uint8_t id = (uint8_t)slot;
        (void)bhy2_klio_set_pattern_states(KLIO_PATTERN_STATE_DISABLE, &id, 1,
                                           imu_dev());
        imu_drain_hard();

        const bool kept = store_save_file();
        printf("slot %d forgotten; %d pattern(s) still stored%s\n", slot,
               bank->count, kept ? "" : "  (WARNING: the file was not rewritten)");
        return kept ? 0 : 1;
    }

    /* --------------------------------------------------------- stored */
    if (strcmp(sub, "stored") == 0 || strcmp(sub, "list") == 0)
    {
        const int have = store_load_file();
        klio_store_t *bank = store();
        if (!bank)
            return 1;

        if (have < 0)
        {
            printf("%s exists but was refused -- see the log\n", KLIO_STORE_PATH);
            return 1;
        }
        if (bank->count == 0)
        {
            printf("no stored patterns at %s\n", KLIO_STORE_PATH);
            return 0;
        }

        for (int i = 0; i < bank->count; i++)
        {
            const klio_store_rec_t *r = &bank->rec[i];
            printf("  slot %2u  %3u bytes  %s%s%s\n", (unsigned)r->slot,
                   (unsigned)r->len,
                   (r->flags & KLIO_STORE_FLAG_ENABLED) ? "enabled" : "disabled",
                   r->name[0] ? "  " : "", r->name);
        }
        printf("%d pattern(s) stored in %s\n", bank->count, KLIO_STORE_PATH);
        return 0;
    }

    /* --------------------------------------------------------- compare */
    /* HOW SIMILAR IS EACH STORED VARIANT TO THE REFERENCE?
     *
     * bhy2_klio.h: "If the two patterns represent the same excercise the
     * similarity will be close to 1.0, and if they are for very different
     * excercises, the similarity will be negative."
     *
     * That is the number this whole exercise is for -- a graded answer, not a
     * yes/no. One call, several slots, so a modulation sweep costs one command.
     *
     *   klio compare 0 1 2 3
     */
    if (strcmp(sub, "compare") == 0)
    {
        if (argc < 4)
        {
            printf("klio compare <ref slot> <slot> [slot...]\n");
            printf("  prints similarity to the reference, as a percentage\n");
            return 1;
        }

        const uint8_t ref = (uint8_t)atoi(argv[2]);
        uint8_t ids[24];
        int n = 0;
        for (int i = 3; i < argc && n < 24; i++)
        {
            const int v = atoi(argv[i]);
            if (v >= 0 && v <= 24)
                ids[n++] = (uint8_t)v;
        }
        if (n == 0)
        {
            printf("klio compare: no valid slots\n");
            return 1;
        }

        /* QUIESCE, AND THIS IS THE FIX FOR A SCORE OF -8.837. The similarity
         * call is a parameter write followed by a parameter read, with the part
         * doing the arithmetic in between -- several blocking I2C transactions.
         * With accel and gyro still streaming from the learn pass the FIFO
         * overflows during it (`meta 12 fifo overflow` in every line of the run
         * that produced -8.837) and the returned buffer is not the answer.
         *
         * The SDK does not bound-check it either: score_multiple() calls
         * bhy2_get_parameter() and copies whatever comes back into the caller's
         * float array without comparing ret_len, so a short or failed read
         * arrives as a plausible-looking negative number. */
        imu_quiesce();
        change_state(0, 0, 0, 0);
        imu_drain_hard();

        /* Enable every slot in the comparison. A pattern is disabled when
         * written and a disabled one is not loaded for scoring. */
        (void)bhy2_klio_set_pattern_states(KLIO_PATTERN_STATE_ENABLE, ids,
                                           (uint16_t)n, imu_dev());
        imu_drain_hard();

        float sim[24] = {0};
        for (int i = 0; i < 24; i++)
            sim[i] = -99.0f;      /* a value no real score can take, so a
                                   * slot the call failed to fill is visible */

        const int8_t rc = bhy2_klio_similarity_score_multiple(ref, ids, (uint8_t)n,
                                                              sim, imu_dev());
        imu_drain_hard();
        if (rc != BHY2_OK)
        {
            printf("similarity failed: %s (%d)\n", rc_name(rc), rc);
            printf("driver status:\n");
            uint32_t drv = 0;
            if (bhy2_klio_read_reset_driver_status(&drv, imu_dev()) == BHY2_OK)
                printf("  %lu\n", (unsigned long)drv);
            return 1;
        }

        printf("similarity to slot %d\n\n", ref);
        printf("  slot   score   percent  bar\n");
        int out_of_range = 0;
        for (int i = 0; i < n; i++)
        {
            /* A tenth of a point of bar per percent, so a full match fills the
             * row and a negative score shows nothing -- which is what it means:
             * less alike than two unrelated motions. */
            char bar[24];
            int w = (int)(sim[i] * 20.0f);   /* only meaningful for 0..1 */
            if (w < 0) w = 0;
            if (w > 20) w = 20;
            memset(bar, '#', (size_t)w);
            bar[w] = '\0';
            const double v = (double)sim[i];
            if (v < -99.5)
            {
                printf("  %4u   ----        --  (not filled by the call)\n",
                       ids[i]);
                continue;
            }
            /* THE SCALE IS NOT -1..+1 and assuming it was is a defect this
             * once had. bhy2_klio.h says only that two similar patterns score
             * "close to 1.0" and two very different ones "negative", with NO
             * stated floor. Measured on this board, an orthogonal motion against
             * a forward-back reference scores -4.486, reproducibly to four
             * decimal places across runs -- which makes it a real score, not a
             * corrupt read. The earlier -8.837 that started this was also
             * probably genuine, from a run whose FIFO was overflowing.
             *
             * So: print what the part says. Flag only a value that is not finite,
             * because that cannot be an answer. */
            if (!isfinite(v))
            {
                printf("  %4u   ----        --  (not a number)\n", ids[i]);
                out_of_range++;
                continue;
            }
            if (v > 1.05)
                printf("  %4u  %+6.3f  %5.1f%%  %s  (above 1.0)\n",
                       ids[i], v, v * 100.0, bar);
            else if (v < -0.001)
                printf("  %4u  %+6.3f        --  %s  (negative = dissimilar)\n",
                       ids[i], v,
                       v < -1.0 ? "well below" : "slightly");
            else
                printf("  %4u  %+6.3f  %5.1f%%  %s\n", ids[i], v, v * 100.0, bar);
        }
        if (out_of_range)
            printf("\n%d slot(s) returned a non-numeric value. Re-run `klio restore`\n"
                   "and try again.\n", out_of_range);

        return out_of_range ? 1 : 0;
    }

    /* ----------------------------------------------------------- trace */
    if (strcmp(sub, "trace") == 0)
    {
        s_trace = !(argc >= 3 && strcmp(argv[2], "off") == 0);
        printf("frame trace %s\n", s_trace ? "ON (one line per frame)" : "off");
        return 0;
    }

    /* -------------------------------------------------- learn from a file */
    /* CAN THE ENGINE LEARN FROM SAMPLES THE HOST MAKES UP?
     *
     * This is the question the operator's whole idea rests on: treat the
     * BHI260AP as a general-purpose accelerator -- stream x,y,z in, get a
     * yes/no out -- rather than a gesture toy that only works when a hand moves.
     *
     * Entry 54 established the command is present and accepts a payload. It did
     * NOT establish that the payload reaches the algorithms. This measures the
     * effect, which is the only test that distinguishes "accepted and used" from
     * "accepted and dropped": run KLIO's learning mode, inject a repeating
     * synthetic motion at a steady rate, and watch whether `progress` climbs
     * with nobody touching the watch.
     *
     * THE WATCH MUST BE STILL, and the control run exists because of that. Run
     * `klio inject 0` first: same loop, same timing, injection off. If progress
     * moves in the control, the motion is coming from the wearer, not the host,
     * and nothing below means anything.
     *
     * THREE FRAMINGS, because Bosch documents none of them. Each is plausible
     * and they are cheap to distinguish:
     *   1  28 bytes of float: accel xyz, gyro xyz, 4-byte timestamp
     *   2  FIFO format: two 7-byte packets per sample, one per sensor id
     *   3  id-prefixed floats: the sensor id, a pad byte, then six floats
     *
     * WHAT EACH OUTCOME MEANS is printed at the end rather than left to the
     * reader, because these lines will be read weeks later by someone who was
     * not here. */
    if (strcmp(sub, "inject") == 0)
    {
        const int shape = argc >= 3 ? atoi(argv[2]) : 0;
        int seconds = argc >= 4 ? atoi(argv[3]) : 30;

        /* THE MOTION IS ARGUMENTS, NOT CONSTANTS, because the first result was
         * 20% at one setting and 0% at the next and one sample of that is not
         * enough to tell a real effect from noise. Sweeping means reflashing
         * each time otherwise, and a reflash is two minutes of not learning
         * anything.
         *
         *   klio inject 4 30 10 6 50     shape, seconds, 1.0 Hz, 0.6 rad, 50/s
         *
         * Frequency and amplitude are in tenths, because the shell's atoi has no
         * float parser and a tenth of a hertz is finer than anyone needs. */
        const float hz = argc >= 5 ? (float)atoi(argv[4]) / 10.0f : 1.0f;
        const float amp = argc >= 6 ? (float)atoi(argv[5]) / 10.0f : 0.6f;
        int rate = argc >= 7 ? atoi(argv[6]) : 50;
        const int wave = argc >= 8 ? atoi(argv[7]) : 2;
        if (hz <= 0.0f || hz > 5.0f || amp <= 0.0f || amp > 2.0f ||
            rate < 10 || rate > 200 || wave < 1 || wave > 2)
        {
            printf("motion: hz 1..50 (tenths), amp 1..20 (tenths), rate 10..200,\n"
                   "        wave 1 = sine, 2 = gesture (default)\n");
            return 1;
        }

        /* A DETERMINISTIC PRNG, because a run has to be repeatable to be
         * compared with another. Numerical Recipes' LCG: three lines, no
         * library, same sequence on every build. */
        uint32_t rng = 0x12345678u + (uint32_t)shape;
        #define RND() (rng = rng * 1664525u + 1013904223u, \
                       (float)((rng >> 8) & 0xFFFFFF) / 16777216.0f)
        if (shape < 0 || shape > 4)
        {
            printf("klio inject <shape> [seconds]\n");
            printf("  0 control, no injection\n");
            printf("  1 floats + timestamp (28 B)\n");
            printf("  2 FIFO events, inject ids 55/58 -- Bosch's dinject framing\n");
            printf("  3 id-prefixed floats\n");
            printf("  4 FIFO events, corrected driver ids 76/79\n");
            return 1;
        }
        if (seconds < 5 || seconds > 120)
            seconds = 30;

        bhy2_klio_sensor_state_t st = {
            .learning_enabled = 1,
            .learning_reset = 1,
            .recognition_enabled = 0,
            .recognition_reset = 0,
        };

        (void)imu_stream_on(100.0f);
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, 100.0f, 20, imu_dev());
        (void)bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_KLIO, on_klio, NULL,
                                                imu_dev());
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_KLIO, 50.0f, 20, imu_dev());

        memset((void *)&s_last, 0, sizeof(s_last));
        s_last.learn_index = -1;
        s_last.recog_index = 255;
        const uint32_t frames_before = s_last.frames;

        /* STEP-BY-STEP FOR THE FIFO FRAMINGS, REAL-TIME FOR THE OTHERS.
         *
         * Bosch's own tool replays a recorded log in STEP-BY-STEP mode and never
         * uses real-time for a file -- dmode s, dinject, dmode n. Real-time is
         * for a host that can keep up with the sensor's configured rate. Both
         * are tried because which one the algorithms accept is exactly what is
         * unknown. */
        if (shape)
        {
            const int mode = (shape == 2 || shape == 4) ? BHY2_STEP_BY_STEP_INJECTION
                                                        : BHY2_REAL_TIME_INJECTION;
            if (!imu_inject_begin(mode))
            {
                printf("klio: the part refused injection mode\n");
                return 1;
            }
        }

        say_rc("learning on", bhy2_klio_set_state(&st, imu_dev()));

        printf("\n%s, %d s", shape ? "INJECTING" : "CONTROL (no injection)", seconds);
        if (shape)
            printf(": %.1f Hz, %.1f rad, %d samples/s", (double)hz, (double)amp, rate);
        printf("\n");
        printf("the watch must be STILL.\n\n");

        /* The motion: theta = 0.6 sin(2 pi f t) about the Y axis, so
         * acceleration is gravity rotated by theta and the gyro reads dtheta/dt.
         * The shape matters more than the scale -- KLIO matches envelopes. */
        /* TUNED ONCE, FROM THE FIRST RESULT. At 1 Hz and 0.6 rad the run reached
         * 20% and fell back to 0 -- progress moved, so injected data does reach
         * the algorithm, but the motion was not repeatable enough for the engine
         * to lock on. Slower and larger is what a human gesture looks like: half
         * a hertz at a radian of swing is a deliberate wrist rock rather than a
         * wobble.
         *
         * 100 samples a second, matching the sensors' configured rate, because
         * in step-by-step mode the part processes what it is given and a rate
         * mismatch is a rhythm mismatch. */
        const int step_ms = 1000 / rate;
        const int steps = seconds * 1000 / step_ms;
        uint32_t sent = 0;
        uint32_t ts_us = 0;

        int last_pct = -1;
        int peak_pct = 0;
        int8_t learnt = -1;

        FILE *trace = fopen("/fs/root/klio.txt", "w");
        if (trace)
            fprintf(trace, "klio inject shape %d, %d s, %.1f Hz, %.1f rad,"
                           " %d/s, wave %d%s\n",
                    shape, seconds, (double)hz, (double)amp, rate, wave,
                    shape ? "" : "  (CONTROL)");

        for (int i = 0; i < steps; i++)
        {
            if (shape)
            {
                const float t = (float)i * step_ms / 1000.0f;
                float theta = 0.0f, dtheta = 0.0f;

                if (wave == 1)
                {
                    /* A PURE SINE, kept only as the control it turned out to
                     * be. It has no repetition boundary and no variance, so a
                     * classifier that segments motion into cycles has nothing
                     * to lock on: peak progress reached 20% once and 0% at every
                     * setting after that. */
                    const float w = 2.0f * (float)M_PI * hz;
                    theta = amp * sinf(w * t);
                    dtheta = amp * w * cosf(w * t);
                }
                else
                {
                    /* A GESTURE, which is what this was always supposed to be:
                     * discrete REPETITIONS separated by rest, not an endless
                     * wave.
                     *
                     * Every part of this exists because a sine lacks it:
                     *
                     *   REST. 40% of each cycle is deliberately still. A human
                     *   gesture ends and the arm comes back down; "one
                     *   repetition finished" is a fact the engine can only learn
                     *   if it is in the data.
                     *
                     *   AN ATTACK AND A DECAY, not a cosine. The swing out is
                     *   30% of the cycle and the return is 70%, with a
                     *   raised-cosine ease on each -- a flick and a settle. A
                     *   symmetric sine spends equal time in every phase and so
                     *   has no landmark.
                     *
                     *   JITTER. Amplitude +/-8% and period +/-10% per cycle, and
                     *   noise on every sample. Two identical cycles teach a
                     *   template that only two identical cycles match; a real
                     *   hand never repeats exactly, and KLIO's own
                     *   LEARNING_SIMILARITY_THRESHOLD exists because of that.
                     *
                     *   A CROSS-AXIS COMPONENT, because a wrist rock about Y is
                     *   never purely about Y. */
                    const float period = 1.0f / hz;
                    const float cycle = floorf(t / period);
                    const float phase = (t - cycle * period) / period;   /* 0..1 */

                    /* Per-cycle variation, held constant within a cycle so the
                     * envelope stays smooth. */
                    uint32_t save = rng;
                    rng = 0x12345678u + (uint32_t)(cycle * 2654435761u) + (uint32_t)shape;
                    /* TIGHT VARIATION, AND THAT IS A MEASUREMENT, NOT A
                     * PREFERENCE. With +/-10% on the period the run stopped at
                     * exactly 20% every time -- one repetition of the five the
                     * engine wants (entry 50 saw progress move 0/20/40/60/80/100).
                     * A matcher that compares repetitions to a template cannot
                     * match two cycles that take different amounts of time, so
                     * the tempo is held nearly exact here and only the amplitude
                     * and the shape of the swing vary. A person is also more
                     * repeatable in tempo than in effort. */
                    const float amp_j = amp * (0.96f + 0.08f * RND());
                    const float period_j = period * (0.98f + 0.04f * RND());
                    const float attack = 0.30f + 0.05f * RND();
                    const float tilt = 0.10f + 0.10f * RND();
                    rng = save;

                    const float p = (t - cycle * period) / period_j;
                    const float active = 0.60f;      /* 60% gesture, 40% rest */

                    if (p > 0.0f && p < active)
                    {
                        const float u = p / active;   /* 0..1 through the gesture */

                        /* Raised-cosine up to u = attack, then back down. */
                        float shape_u;
                        if (u < attack)
                        {
                            const float x = u / attack;
                            shape_u = 0.5f - 0.5f * cosf((float)M_PI * x);
                        }
                        else
                        {
                            const float x = (u - attack) / (1.0f - attack);
                            shape_u = 0.5f + 0.5f * cosf((float)M_PI * x);
                        }
                        theta = amp_j * shape_u;

                        /* d/dt of that, in rad/s: the derivative of the raised
                         * cosine, chained through the phase scaling. */
                        if (u < attack)
                        {
                            const float x = u / attack;
                            dtheta = amp_j * (0.5f * (float)M_PI / attack)
                                     * sinf((float)M_PI * x) / (active * period_j);
                        }
                        else
                        {
                            const float x = (u - attack) / (1.0f - attack);
                            dtheta = -amp_j * (0.5f * (float)M_PI / (1.0f - attack))
                                     * sinf((float)M_PI * x) / (active * period_j);
                        }
                    }
                    (void)phase;

                    /* Noise, on the signal and not just the reading. A real
                     * accelerometer is never still, and an engine that expects
                     * stillness to look like exactly 1.000 g will not recognise
                     * a human arm. */
                    theta += (RND() - 0.5f) * 0.01f;
                    dtheta += (RND() - 0.5f) * 3.0f;      /* deg/s */

                    /* Remembered for the accel below. */
                    s_tilt = tilt;
                }

                const float tilt = (wave == 1) ? 0.0f : s_tilt;
                const float ax = sinf(theta) + tilt * 0.3f;
                const float az = cosf(theta);
                const float ay = tilt * sinf(theta) * 0.5f;
                const float gy = dtheta * 180.0f / (float)M_PI;  /* deg/s */

                uint8_t pkt[56];
                uint16_t plen = 0;

                if (shape == 1)
                {
                    float f[7] = {ax, ay, az, 0.0f, gy, 0.0f, t};
                    memcpy(pkt, f, sizeof(f));
                    plen = (uint16_t)sizeof(f);
                }
                else if (shape == 2 || shape == 4)
                {
                    /* BOSCH'S OWN FRAMING, from BHy2CLI's dinject.c.
                     *
                     * A stream of ordinary FIFO events -- a ONE-BYTE event id
                     * then that event's data -- concatenated into one buffer and
                     * handed over in a single inject call. Accel and gyro are
                     * two events, so one sample is 14 bytes.
                     *
                     * THE IDS ARE THE POINT. dinject_defs.h names inject ids 55
                     * (accel) and 58 (gyro), plus driver ids 76 (corrected
                     * accel) and 79 (corrected gyro). The earlier attempts used
                     * the ORDINARY sensor ids 4 and 13, which the injection
                     * handler does not know -- that is why the command was
                     * accepted and the data went nowhere.
                     *
                     * Accel is 1/4096 g per count. Gyro is taken as 1/16 deg/s,
                     * the usual BHI260 scaling and a guess; KLIO matches motion
                     * envelopes, so the rhythm carries more than the units. */
                    const uint8_t acc_id = (shape == 4) ? 76 : 55;
                    const uint8_t gyr_id = (shape == 4) ? 79 : 58;

                    int16_t a[3] = {(int16_t)(ax * 4096.0f), (int16_t)(ay * 4096.0f),
                                    (int16_t)(az * 4096.0f)};
                    int16_t g[3] = {0, (int16_t)(gy * 16.0f), 0};

                    /* A FULL TIMESTAMP GOES FIRST, AS ITS OWN INJECTION.
                     *
                     * Bosch's swim_data_injection.c does exactly this and it is
                     * the one step this command was missing: the part will not
                     * place an event in time without it, so an event injected
                     * without a preceding timestamp has nowhere to sit. 0xFD is
                     * TS_FULL on the non-wakeup channel; five bytes carry the
                     * 40-bit microsecond value. */
                    uint8_t stamp[6];
                    stamp[0] = BHY2_SYS_ID_TS_FULL;
                    stamp[1] = (uint8_t)(ts_us & 0xFF);
                    stamp[2] = (uint8_t)((ts_us >> 8) & 0xFF);
                    stamp[3] = (uint8_t)((ts_us >> 16) & 0xFF);
                    stamp[4] = (uint8_t)((ts_us >> 24) & 0xFF);
                    stamp[5] = 0;
                    ts_us += (uint32_t)(step_ms * 1000);
                    (void)imu_inject_frame(stamp, sizeof(stamp));

                    pkt[0] = acc_id;
                    memcpy(pkt + 1, a, 6);
                    pkt[7] = gyr_id;
                    memcpy(pkt + 8, g, 6);
                    plen = 14;
                }
                else
                {
                    float f[6] = {ax, ay, az, 0.0f, gy, 0.0f};
                    pkt[0] = BHY2_SENSOR_ID_ACC;
                    pkt[1] = 0;
                    memcpy(pkt + 2, f, sizeof(f));
                    plen = (uint16_t)(2 + sizeof(f));
                }

                if (imu_inject_frame(pkt, plen))
                    sent++;


            }

            vTaskDelay(pdMS_TO_TICKS(step_ms));
            (void)imu_poll();

            /* Every five seconds, not every second: a 20-second run otherwise
             * prints 20 progress lines and pushes the verdict off a 26-row
             * terminal. Four lines and the summary is what a reader gets. */
            if ((i % 500) == 499)
            {
                float ax = 0, ay = 0, az = 0;
                uint32_t n = 0;
                (void)imu_accel(&ax, &ay, &az, &n);
                /* TO A FILE, NOT THE SCREEN. The display mirror sends only
                 * changed cells and this console capture applies them to an
                 * empty grid, so a scrolling transcript comes back interleaved
                 * and unreadable -- measured: "progress I20%APframesH626accel".
                 * The numbers were real and the screen could not show them.
                 * `cat /root/klio.txt` afterwards reads a clean file. */
                if (trace)
                    fprintf(trace, "  %2d s  progress %3u%%%s  frames %lu"
                                   "  accel %+.2f %+.2f %+.2f\n",
                            (i + 1) * step_ms / 1000, (unsigned)s_last.learn_progress,
                            s_last.learn_reason == 1 ? " (interrupted)"
                            : s_last.learn_reason == 2 ? " (no movement)" : "",
                            (unsigned long)(s_last.frames - frames_before),
                            (double)ax, (double)ay, (double)az);
                if (s_last.learn_progress > peak_pct)
                    peak_pct = s_last.learn_progress;
                last_pct = s_last.learn_progress;
            }

            if (s_last.frames != frames_before && s_last.learn_index >= 0 &&
                s_last.learn_progress >= 100)
            {
                learnt = s_last.learn_index;
                s_learnt = learnt;
                break;
            }
        }

        bhy2_klio_sensor_state_t off = {0};
        (void)bhy2_klio_set_state(&off, imu_dev());
        if (shape)
            imu_inject_end();

        const uint32_t frames = s_last.frames - frames_before;
        if (s_last.learn_progress > peak_pct)
            peak_pct = s_last.learn_progress;
        printf("\n%lu frames, %lu injected, peak progress %d%% (ended %d%%)\n",
               (unsigned long)frames, (unsigned long)sent, peak_pct, last_pct);
        if (trace)
        {
            fprintf(trace, "\n%lu frames, %lu injected, peak %d%%, ended %d%%\n",
                    (unsigned long)frames, (unsigned long)sent, peak_pct, last_pct);
        }

        if (learnt >= 0)
        {
            printf("\nLEARNT from injected data. The classifier saw the host's\n"
                   "samples and not the accelerometer's. This is the black box\n"
                   "working.\n");
        }
        else if (shape == 0)
        {
            printf("\nCONTROL: nothing fed in, and progress did not reach 100.\n"
                   "That is the baseline the injected runs are measured against.\n");
        }
        else if (frames == 0)
        {
            printf("\nNo KLIO frames either. Real-time injection appears to have\n"
                   "taken the algorithm's input away without supplying new data,\n"
                   "so the framing above is wrong or the mode is not what it says.\n");
        }
        else
        {
            printf("\nFrames kept coming and progress did not move: the part is\n"
                   "still running the algorithms on the PHYSICAL sensors, so the\n"
                   "injected bytes did not reach them in this framing.\n");
        }

        if (trace)
        {
            fprintf(trace, "%s\n", learnt >= 0 ? "LEARNT" : "NOT LEARNT");
            fclose(trace);
            printf("\nfull trace: cat /root/klio.txt\n");
        }

        (void)imu_accel_enable(0.0f);
        return learnt >= 0 ? 0 : 1;
    }

    /* ---------------------------------------------------------- detect */
    /* ---------------------------------------------------------- detect */
    if (strcmp(sub, "detect") == 0)
    {
        /* THE STEP THAT WAS MISSING, AND WHY NOTHING EVER FIRED.
         *
         * bhy2_klio.h, on read_pattern: "After learning of a new pattern has
         * been signaled by a sensor data event, CALL THIS FUNCTION TO RETRIEVE
         * THE LEARNT PATTERN." Learning leaves the pattern inside the algorithm;
         * it is not in the host's pattern table, and `set_pattern_states(ENABLE)`
         * was therefore enabling an empty slot. The full lifecycle is:
         *
         *   learn  -> the engine signals a new pattern
         *   read_pattern(0)      -> pull it out into host memory
         *   write_pattern(0)     -> put it in the RECOGNITION table
         *   set_pattern_states(ENABLE, {0}) -> and turn it on
         *   recognition
         *
         * "only 0 supported now", says the header, which is why the pattern id
         * here is 0 and not the index the engine reported while learning -- that
         * index numbers the engine's internal slot, not the recognition table.
         *
         * Every step is reported, because each one can refuse and a silent
         * refusal is what made this look like "recognition just does nothing". */
        uint32_t drv = 0;
        if (bhy2_klio_read_reset_driver_status(&drv, imu_dev()) == BHY2_OK)
            printf("driver status: %lu\n", (unsigned long)drv);

        /* THE RATE IS DELIBERATELY LOWER THAN LEARNING USED, and the reason is
         * the FIFO. Accel and gyro at 100 Hz each is 26 bytes of packet per
         * 10 ms -- 2.6 KB a second -- and every setup call below (read_pattern,
         * write_pattern, set_pattern_states) is several I2C transactions during
         * which nothing drains. That produced `meta 12 fifo overflow`, which
         * DROPS DATA, and a dropped KLIO frame during recognition is
         * indistinguishable from "no match".
         *
         * 50 Hz on both halves the byte rate, which is plenty: KLIO works on
         * motion envelopes, not sample-accurate waveforms. */
        (void)imu_stream_on(50.0f);
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, 50.0f, 20, imu_dev());
        /* Early interrupts so a drain has something to do rather than waiting
         * for a batch that the setup calls would overflow. */
        (void)bhy2_set_fifo_wmark_wkup(32, imu_dev());
        imu_poll();

        /* 1. Read the learnt pattern out of the engine. The buffer must hold
         *    the maximum pattern_data (244), not the blob size (157). */
        static uint8_t pattern[244];
        uint16_t plen = sizeof(pattern);
        int8_t rc = bhy2_klio_read_pattern(0, pattern, &plen, imu_dev());
        imu_poll();
        printf("read_pattern:  %s (%d), %u bytes\n",
               rc == BHY2_OK ? "ok" : "FAILED", rc, (unsigned)plen);
        if (rc != BHY2_OK)
        {
            printf("\nNo learnt pattern could be read. Run `klio learn` first --\n"
                   "and in THIS boot: the firmware is uploaded to RAM at start,\n"
                   "so the pattern table does not survive a reboot.\n");
            return 1;
        }

        /* 2. Write it into the recognition table at index 0. */
        imu_poll();
        rc = bhy2_klio_write_pattern(0, pattern, plen, imu_dev());
        printf("write_pattern: %s (%d)\n", rc == BHY2_OK ? "ok" : "FAILED", rc);
        if (rc != BHY2_OK)
            return 1;

        /* 3. Enable it -- a written pattern is disabled by default. */
        {
            uint8_t id = 0;
            imu_poll();
            rc = bhy2_klio_set_pattern_states(KLIO_PATTERN_STATE_ENABLE, &id, 1,
                                              imu_dev());
            printf("enable pattern: %s (%d)\n", rc == BHY2_OK ? "ok" : "FAILED", rc);
        }

        /* 4. Recognition on, learning OFF (learning would keep re-training and
         *    never set the recognition half), and NO reset -- the reset fields
         *    wipe the table that was just written. */
        {
            bhy2_klio_sensor_state_t st = {
                .learning_enabled = 0,
                .learning_reset = 0,
                .recognition_enabled = 1,
                .recognition_reset = 0,
            };
            rc = bhy2_klio_set_state(&st, imu_dev());
            printf("recognition on: %s (%d)\n", rc == BHY2_OK ? "ok" : "FAILED", rc);
        }

        (void)bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_KLIO, on_klio, NULL,
                                                imu_dev());
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_KLIO, 50.0f, 20, imu_dev());

        printf("\nperform the trained gesture, repeatedly. Ctrl+C to stop.\n\n");

        const uint32_t frames0 = s_last.frames;
        int last_idx = -1;
        float last_count = -1.0f;
        for (int i = 0; i < 12000; i++)         /* 12000 x 5 ms = 60 s */
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            (void)imu_poll();
            (void)imu_poll();                   /* twice: cheap, and one call
                                                 * per pass can lag the fill */

            if (s_last.recog_index != 0xFF &&
                ((int)s_last.recog_index != last_idx ||
                 s_last.recog_count > last_count + 0.4f))
            {
                printf("  RECOGNISED pattern %u, count %.1f\n",
                       (unsigned)s_last.recog_index, (double)s_last.recog_count);
                last_idx = (int)s_last.recog_index;
                last_count = s_last.recog_count;
            }
        }

        printf("\nframes from the engine: %lu\n",
               (unsigned long)(s_last.frames - frames0));
        if (last_idx < 0)
            printf("no recognition event. The pattern IS loaded (read/write/enable\n"
                   "all returned 0), so either the gesture did not match at the\n"
                   "configured sensitivity, or this firmware only reports a match\n"
                   "and stays silent otherwise.\n");
        (void)bhy2_klio_set_state(&(bhy2_klio_sensor_state_t){0}, imu_dev());
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_KLIO, 0.0f, 0, imu_dev());
        (void)imu_accel_enable(0.0f);
        return 0;
    }

    /* ------------------------------------------------------------ stop */
    if (strcmp(sub, "stop") == 0)
    {
        bhy2_klio_sensor_state_t off = {0};
        say_rc("both off", bhy2_klio_set_state(&off, imu_dev()));
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_KLIO, 0.0f, 0, imu_dev());
        return 0;
    }

    /* ------------------------------------------------------- recognise */
    /* RECOGNITION ONLY -- no read_pattern, no write_pattern.
     *
     * This exists because `klio detect` cannot test a SAVED pattern. detect's
     * first step is read_pattern(0), which reads the engine's LEARN slot; once a
     * pattern has been written into the recognition table that slot is consumed
     * and read_pattern returns -2 with driver status 3 (invalid pattern
     * operation). So detect reports "no learnt pattern" against a pattern that
     * is loaded, enabled and matching perfectly well.
     *
     * The distinction is the whole point of persistence: a restored pattern
     * never passes through the learn slot at all. It is written straight into
     * the recognition table by klio_after_boot() -- or by `klio restore` -- so
     * the only correct test is to turn recognition on and listen. */
    if (strcmp(sub, "recognise") == 0 || strcmp(sub, "recognize") == 0)
    {
        const int secs = argc >= 3 ? atoi(argv[2]) : 30;
        if (secs < 1 || secs > 120)
        {
            printf("klio recognise [1..120]\n");
            return 1;
        }

        klio_store_t *bank = store();
        const int have = store_load_file();
        if (have <= 0 || !bank || bank->count == 0)
        {
            printf("nothing is loaded to recognise\n");
            printf("`klio save <slot>` keeps a learnt pattern, and `klio restore`\n"
                   "puts stored ones back into the engine.\n");
            return 1;
        }

        /* ENABLE EVERY STORED SLOT. A pattern in the recognition table that is
         * not enabled is never used, and this is where that would show up as
         * "recognition does nothing". */
        for (int i = 0; i < bank->count; i++)
        {
            const uint8_t id = bank->rec[i].slot;
            const int8_t rc = bhy2_klio_set_pattern_states(KLIO_PATTERN_STATE_ENABLE,
                                                           &id, 1, imu_dev());
            printf("slot %u enable: %s (%d)\n", (unsigned)id,
                   rc == BHY2_OK ? "ok" : "FAILED", rc);
            imu_drain_hard();
        }

        /* 50 Hz, for the FIFO reason detect documents: the setup calls below are
         * several I2C transactions during which nothing drains. */
        (void)imu_stream_on(50.0f);
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GYRO, 50.0f, 20, imu_dev());
        (void)bhy2_set_fifo_wmark_wkup(32, imu_dev());
        imu_poll();

        /* LEARNING OFF AND NO RESET -- either field being 1 would wipe the table
         * that is about to be tested. */
        {
            bhy2_klio_sensor_state_t st = {
                .learning_enabled = 0,
                .learning_reset = 0,
                .recognition_enabled = 1,
                .recognition_reset = 0,
            };
            const int8_t rc = bhy2_klio_set_state(&st, imu_dev());
            printf("recognition on: %s (%d)\n", rc == BHY2_OK ? "ok" : "FAILED", rc);
            if (rc != BHY2_OK)
                return 1;
        }

        (void)bhy2_register_fifo_parse_callback(BHY2_SENSOR_ID_KLIO, on_klio, NULL,
                                                imu_dev());
        (void)bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_KLIO, 50.0f, 20, imu_dev());

        printf("\nperform the trained gesture, repeatedly, for %d s.\n\n", secs);

        const uint32_t frames0 = s_last.frames;
        s_last.recog_index = 0xFF;
        int last_idx = -1;
        float last_count = -1.0f;
        const int64_t t0 = esp_timer_get_time();

        while (esp_timer_get_time() - t0 < (int64_t)secs * 1000000)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            (void)imu_poll();
            (void)imu_poll();       /* twice, as detect does: cheap, and one
                                     * call per pass can lag the fill */

            if (s_last.recog_index != 0xFF &&
                ((int)s_last.recog_index != last_idx ||
                 s_last.recog_count > last_count + 0.4f))
            {
                printf("  RECOGNISED pattern %u, count %.1f\n",
                       (unsigned)s_last.recog_index, (double)s_last.recog_count);
                last_idx = (int)s_last.recog_index;
                last_count = s_last.recog_count;
            }
        }

        printf("\nframes from the engine: %lu\n",
               (unsigned long)(s_last.frames - frames0));
        printf("%s\n", last_idx >= 0
                       ? "MATCH: the stored pattern is live in the engine"
                       : "NO MATCH: the pattern is loaded and enabled, so either the "
                         "gesture differed or the blob is not what was learnt");

        (void)bhy2_klio_set_state(&(bhy2_klio_sensor_state_t){0}, imu_dev());
        return last_idx >= 0 ? 0 : 1;
    }

    if (strcmp(sub, "reset") == 0)
    {
        bhy2_klio_sensor_state_t st = {0};
        st.learning_reset = 1;
        st.recognition_reset = 1;
        say_rc("patterns cleared", bhy2_klio_set_state(&st, imu_dev()));
        return 0;
    }

    printf("usage: klio [status|learn|detect|stop|reset]\n"
           "       klio [save <slot>|stored|restore|forget <slot>]\n"
           "       klio [recognise [seconds]]\n");
    return 2;
}

static const tdsh_command_t s_klio_cmd = {
    "klio",
    "klio [status|learn|detect|recognise|stop|reset|save|stored|restore|forget]",
    "The BHI260AP's self-learning gesture engine",
    cmd_klio,
    TDSH_CMD_INTERACTIVE,
};

bool klio_register_command(void)
{
    return tdsh_register_command(&s_klio_cmd) == 0;
}
