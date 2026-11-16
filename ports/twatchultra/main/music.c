/*
 * music.c - play a FLAC file through the built-in speaker.
 *
 * THE SHAPE IS sound_play_wav()'s, WITH A DECODER IN FRONT. Its own task, the
 * file streamed a chunk at a time (a track is tens of megabytes against 8 MiB
 * of PSRAM), and the I2S rate set from the FILE rather than from a constant.
 * What is new is that the bytes off the card are compressed, so the loop is
 *
 *     read a window -> decode what it yields -> write the PCM to I2S -> repeat
 *
 * and the write BLOCKS (sound_stream_write_stereo_wait). That is deliberate and
 * is the opposite of the emulator path's choice: an emulator drops a tick
 * rather than stall a frame, while a player has its own task and a dropped
 * sample is an audible click. Blocking also paces the decoder to the output
 * rate for free -- the I2S DMA ring absorbs SD stalls, and the write waits when
 * it is full, so nothing needs a clock of its own.
 *
 * WHY A WINDOW AND NOT A WHOLE FILE. Two reasons, both measured elsewhere in
 * this port: a FLAC track is tens of megabytes and PSRAM is shared, and the SD
 * card stalls for 100+ ms at a time (core-log entry 7). Decoding from a small
 * window means a stall costs a wait, not a re-read, and the I2S ring rides it
 * out.
 *
 * WHY THIS LIVES IN THE SHELL. Same reasoning as `play`: it is the smallest
 * thing that answers "does the decoder reach the speaker?", and a Start-menu
 * app would need a picker, a progress bar and a stop button before it was worth
 * testing whether the audio path works at all. The app comes after the plumbing
 * is proven.
 */
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "musicdec.h"
#include "sound.h"
#include "tdsh.h"

static const char *TAG = "music";

/* The compressed window: one SD read, and enough for several FLAC blocks so the
 * decoder is not called per frame. */
#define MUSIC_IN_BYTES      (16 * 1024)

/* PCM out: one FLAC block is at most 4608 frames in this decoder's default
 * configuration, and the buffer is sized generously so a 8192-block stream
 * still fits without reallocating mid-track. */
#define MUSIC_OUT_FRAMES    8192

/* How long to wait for the I2S DMA to take a chunk before calling the output
 * dead. Generous: a 100 ms SD stall happens between writes, not inside one. */
#define MUSIC_WRITE_MS      1000

/* The player keeps its own stack and runs below the UI, like `play`. */
#define MUSIC_STACK         8192
#define MUSIC_PRIO          3

static char s_path[128];
static TaskHandle_t s_task;
static volatile bool s_stop;
static volatile int s_state;             /* 0 idle, 1 playing, 2 done, 3 error */
static volatile uint32_t s_frames;
static volatile uint32_t s_errors;
static volatile uint32_t s_underruns;    /* writes that were short */
static char s_msg[64];

static void set_msg(const char *m)
{
    snprintf(s_msg, sizeof(s_msg), "%s", m);
}

static void play_task(void *arg)
{
    (void)arg;

    music_dec_t *dec = music_dec_open();

    if (!dec)
    {
        set_msg("no decoder");
        s_state = 3;
        goto done;
    }

    FILE *f = fopen(s_path, "rb");

    if (!f)
    {
        set_msg("cannot open file");
        s_state = 3;
        music_dec_close(dec);
        goto done;
    }

    uint8_t *in = (uint8_t *)heap_caps_malloc(MUSIC_IN_BYTES, MALLOC_CAP_SPIRAM);
    int16_t *out = (int16_t *)heap_caps_malloc_prefer(MUSIC_OUT_FRAMES * 2 * sizeof(int16_t), 2,
                                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                                      MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!in || !out)
    {
        set_msg("no memory");
        s_state = 3;
        if (in)
            heap_caps_free(in);
        if (out)
            heap_caps_free(out);
        music_dec_close(dec);
        fclose(f);
        goto done;
    }

    /* A window holds a partial frame at its end after the decoder consumes
     * what it can, so the leftovers are carried to the front of the next
     * read rather than discarded. Without this every window boundary would
     * cost a corrupt frame. */
    size_t have = 0;         /* bytes carried in `in` */
    bool started = false;
    int consecutive_errors = 0;

    while (!s_stop)
    {
        if (have < MUSIC_IN_BYTES)
        {
            const size_t n = fread(in + have, 1, MUSIC_IN_BYTES - have, f);

            have += n;

            if (n == 0 && have == 0)
                break;              /* end of file with nothing left */
        }

        size_t consumed = 0, samples = 0;
        const int r = music_dec_read(dec, in, have, &consumed, out,
                                     started ? MUSIC_OUT_FRAMES : 0, &samples);

        /* Whatever the decoder did not use is carried forward. A decoder that
         * consumes nothing and produces nothing on a full window would spin,
         * so both are guarded below. */
        if (consumed > 0 && consumed <= have)
        {
            memmove(in, in + consumed, have - consumed);
            have -= consumed;
        }

        if (r == 1)
        {
            consecutive_errors = 0;

            if (!started && music_dec_ready(dec))
            {
                const int rate = music_dec_rate(dec);

                if (!sound_stream_open(rate))
                {
                    set_msg("no amplifier");
                    s_state = 3;
                    break;
                }
                ESP_LOGI(TAG, "%d Hz, %d ch, %d bit", rate, music_dec_channels(dec),
                         music_dec_bits(dec));
                started = true;
            }

            if (started && samples > 0)
            {
                const size_t got = sound_stream_write_stereo_wait(out, (int)samples,
                                                                  MUSIC_WRITE_MS);

                s_frames += (uint32_t)got;
                if (got < samples)
                    s_underruns++;
            }
        }
        else if (r == 0)
        {
            /* Needs more input. If the window is full and nothing was consumed
             * the decoder is stuck: count it and drop the window rather than
             * loop forever on the same bytes. */
            if (consumed == 0 && have == MUSIC_IN_BYTES)
            {
                have = 0;
                if (++consecutive_errors > 8)
                {
                    set_msg("decoder stuck");
                    s_state = 3;
                    break;
                }
            }
            if (feof(f) && have == 0)
                break;
        }
        else if (r == -1)
        {
            set_msg("end of track");
            s_state = 2;
            break;
        }
        else
        {
            s_errors++;

            /* The decoder resets its frame state on a bad frame, so drop the
             * window and carry on. Repeats mean the stream is not FLAC. */
            have = 0;
            if (++consecutive_errors > 8)
            {
                set_msg("decode error");
                s_state = 3;
                break;
            }
        }
    }

    if (s_state == 1)
        s_state = s_stop ? 0 : 2;

    if (started)
        sound_stream_close();
    heap_caps_free(in);
    heap_caps_free(out);
    music_dec_close(dec);
    fclose(f);

done:
    s_task = NULL;
    vTaskDelete(NULL);
}

static void music_task_entry(void *arg)
{
    play_task(arg);
}

/* Run the decode on a PSRAM stack: the same 8 KB taken from internal SRAM
 * otherwise, and this task never touches flash. See components/micro-flac's
 * CMakeLists for why that matters on this board. */
static bool start(const char *path)
{
    if (s_task)
        return false;
    if (strlen(path) >= sizeof(s_path))
        return false;

    snprintf(s_path, sizeof(s_path), "%s", path);

    s_stop = false;
    s_state = 1;
    s_frames = 0;
    s_errors = 0;
    s_underruns = 0;
    set_msg("");

    if (xTaskCreateWithCaps(music_task_entry, "music", MUSIC_STACK, NULL, MUSIC_PRIO, &s_task,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS)
    {
        s_task = NULL;
        s_state = 0;
        return false;
    }
    return true;
}

static int cmd_music(tdsh_session_t *session, int argc, char **argv)
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
        if (s_task || s_state == 2 || s_state == 3)
            printf("state %s, %lu frames, %lu errors, %lu short writes%s%s\n",
                   s_state == 1 ? "playing" : (s_state == 2 ? "done" : "error"),
                   (unsigned long)s_frames, (unsigned long)s_errors,
                   (unsigned long)s_underruns, s_msg[0] ? ": " : "", s_msg);
        return 0;
    }

    if (!start(argv[1]))
    {
        printf("music: cannot start\n");
        return 1;
    }
    printf("music: playing %s\n", argv[1]);
    return 0;
}

static const tdsh_command_t s_music_cmd = {
    "music",
    "music <file.flac> | music stop",
    "Play a FLAC file through the speaker",
    cmd_music,
    TDSH_CMD_INTERACTIVE,
};

bool music_register_command(void)
{
    return tdsh_register_command(&s_music_cmd) == 0;
}
