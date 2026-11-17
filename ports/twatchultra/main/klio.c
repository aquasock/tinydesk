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

/* Per-frame byte trace, off by default. See the note in on_klio(). */
static bool s_trace = false;

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
