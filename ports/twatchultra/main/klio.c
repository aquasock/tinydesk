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
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh.h"

#include "bhy2.h"
#include "bhy2_defs.h"
#include "bhy2_klio.h"
#include "bhy2_klio_defs.h"
#include "imu.h"

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
    char hex[3 * 24 + 1];
    size_t pos = 0;
    for (size_t i = 0; i < n && pos + 3 < sizeof(hex); i++)
        pos += (size_t)snprintf(hex + pos, sizeof(hex) - pos, "%02x ", d[i]);
    ESP_LOGI(TAG, "id %u len %u: %s", (unsigned)info->sensor_id, (unsigned)n, hex);
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
        printf("  klio detect   recognise what you do\n");
        printf("  klio stop     both off\n");
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

    if (strcmp(sub, "reset") == 0)
    {
        bhy2_klio_sensor_state_t st = {0};
        st.learning_reset = 1;
        st.recognition_reset = 1;
        say_rc("patterns cleared", bhy2_klio_set_state(&st, imu_dev()));
        return 0;
    }

    printf("usage: klio [status|learn|detect|stop|reset]\n");
    return 2;
}

static const tdsh_command_t s_klio_cmd = {
    "klio",
    "klio [status|learn|detect|stop|reset]",
    "The BHI260AP's self-learning gesture engine",
    cmd_klio,
    TDSH_CMD_INTERACTIVE,
};

bool klio_register_command(void)
{
    return tdsh_register_command(&s_klio_cmd) == 0;
}
