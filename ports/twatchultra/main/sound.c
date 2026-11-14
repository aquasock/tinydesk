/*
 * sound.c - a beep from the MAX98357A, over I2S.
 *
 * WRITTEN FROM LILYGO'S OWN CODE, because that is the only thing on this
 * machine that has ever driven this amplifier. LilyGoLib's
 * src/audio/AudioDevice.cpp, AudioOutputDev::begin():
 *
 *   mode               I2S_MODE_MASTER | I2S_MODE_TX
 *   sample_rate        44100
 *   bits_per_sample    16
 *   channel_format     I2S_CHANNEL_FMT_ALL_RIGHT
 *   dma_buf_count      4
 *   dma_buf_len        1024
 *   tx_desc_auto_clear true
 *   pins               bclk 9, ws 10, dout 11
 *
 * and LilyGoWatchUltra.cpp raises the supply first, in its PMU bring-up:
 * `setVoltage(CH_BLDO2, 3300); enable(CH_BLDO2, true)`, right beside the
 * display's ALDO2 and the card's ALDO1. BLDO2 is recorded as the speaker's rail
 * in BRD-003 as well, so two independent sources agree.
 *
 * WHAT LILYGO DOES NOT HAVE is a tone generator: their audio path plays WAV
 * files. A beep therefore needs one, and a square wave is the cheapest honest
 * answer -- it is what an IBM PC speaker was, it needs no filter or DAC table,
 * and at 16 kHz a 1 kHz tone is sixteen samples per period, so the arithmetic
 * is a compare and a negate.
 *
 * THE API IS THE MODERN ONE. LilyGo uses i2s_driver_install(), which still
 * exists in ESP-IDF 5.3.1 but only under components/driver/deprecated. New code
 * should not start there, so this uses i2s_std: i2s_new_channel(),
 * i2s_channel_init_std_mode(), i2s_channel_enable(). The settings above carry
 * over unchanged except the channel format, where a mono slot is what a single
 * speaker wants -- ALL_RIGHT would work too, but it is describing a stereo
 * pair's right half and this board has one channel.
 *
 * THE RAIL IS THE SILENT FAILURE, again. A class-D amplifier with its supply
 * switched off takes every I2S clock, accepts every sample and outputs nothing,
 * so a wrong I2S configuration and an unpowered amplifier look identical from
 * here. board_speaker_rail_on() runs first and its result is checked.
 */

#include "sound.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <errno.h>

#include "wav.h"

#include <string.h>

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "power.h"

#define TAG "twatch-sound"

/* Pins, from arduino-esp32 variants/lilygo_twatch_ultra/pins_arduino.h and
 * LilyGoLib's own I2S_BCLK/I2S_WCLK/I2S_DOUT. */
#define PIN_I2S_BCLK 9
#define PIN_I2S_WS   10
#define PIN_I2S_DOUT 11

/* 16 kHz is plenty for a beep and keeps the sample count small: a 1 kHz tone is
 * sixteen samples per period. LilyGo plays files at 44.1 kHz, which a tone does
 * not need. */
#define SOUND_RATE_HZ  16000

/* The beep: a 1 kHz square for 80 ms. Long enough to be heard as a deliberate
 * sound rather than a click, short enough not to be in the way of whatever the
 * program is doing. 80 ms at 16 kHz is 1280 samples, which fits one DMA buffer
 * and does not need the caller to wait for more. */
/* 200 ms rather than 80. The first attempt was 80 ms at 18 per cent and could
 * have been missed by a listener who was not sure what to expect; this is long
 * enough to be unmistakable. A beep that might be inaudible is not a test. */
#define BEEP_HZ        1000
#define BEEP_MS        250

#define AMPLITUDE      26000       /* of 32767: 80 per cent, as loud as a
                                    * watch speaker is likely to get */

static i2s_chan_handle_t s_tx;
static bool s_tried;

/* One period of the tone, played out as square samples.
 *
 * The period is computed rather than hardcoded so the frequency stays the
 * frequency if SOUND_RATE_HZ changes; at 16 kHz and 1 kHz it is 16, and the
 * division is exact. */
static void beep_samples(int16_t *out, int frames)
{
    const int period = SOUND_RATE_HZ / BEEP_HZ;
    const int half = period / 2;
    int i;

    /* One square period per `period` frames, written to both slots. The frame
     * index drives the phase, not the sample index, so the left and right
     * halves of a frame stay in step. */
    for (i = 0; i < frames; i++)
    {
        int16_t v = ((i % period) < half) ? AMPLITUDE : -AMPLITUDE;

        out[2 * i] = v;
        out[2 * i + 1] = v;
    }
}

bool sound_init(void)
{
    i2s_chan_config_t chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_std_config_t std = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SOUND_RATE_HZ),
         /* PHILIPS, not MSB, and STEREO, not mono -- both to match LilyGo.
         *
         * The first was a real bug: I2S_STD_MSB_SLOT_DEFAULT_CONFIG differs from
         * the Philips form only in `bit_shift`, and that flag is the one-clock
         * delay between the WS transition and the first data bit. The MAX98357A
         * expects standard I2S, so MSB framing latches every sample one bit
         * early, which decodes to rubbish rather than to an obvious error --
         * the tone came out inaudible rather than wrong-sounding.
         *
         * The second is quieter but worth having: LilyGo sends stereo with both
         * slots carrying the same sample (I2S_CHANNEL_FMT_ALL_RIGHT). In its
         * default SD-mode configuration the amplifier outputs (L+R)/2, so a
         * mono-left stream is half amplitude for free. Stereo with both slots
         * filled is the full signal into that average. */
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws = PIN_I2S_WS,
            .dout = PIN_I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    s_tried = true;

    /* The amplifier's supply before its clocks. A class-D with the rail down
     * accepts every sample and outputs silence, so getting this order wrong
     * presents as a broken I2S setup. */
    if (!board_speaker_rail_on())
    {
        ESP_LOGE(TAG, "speaker rail failed");
        return false;
    }

    /* Four buffers of 1024 samples, as LilyGo configures it, so a beep fits in
     * one buffer and the DMA never underruns mid-tone. */
    chan.dma_desc_num = 4;
    chan.dma_frame_num = 1024;
    chan.auto_clear_after_cb = true;

    if (i2s_new_channel(&chan, &s_tx, NULL) != ESP_OK)
    {
        ESP_LOGE(TAG, "no i2s tx channel");
        s_tx = NULL;
        return false;
    }
    if (i2s_channel_init_std_mode(s_tx, &std) != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s std mode failed");
        i2s_del_channel(s_tx);
        s_tx = NULL;
        return false;
    }
    if (i2s_channel_enable(s_tx) != ESP_OK)
    {
        ESP_LOGE(TAG, "i2s enable failed");
        i2s_del_channel(s_tx);
        s_tx = NULL;
        return false;
    }

    /* Short enough for the Log Viewer's 40 columns. */
    ESP_LOGI(TAG, "i2s %dHz philips st bclk%d ws%d dout%d", SOUND_RATE_HZ,
             PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT);
    return true;
}

void sound_beep(void)
{
    /*
     * PSRAM, NOT .bss, AND THAT IS A CORRECTION RATHER THAN A PREFERENCE.
     *
     * This was a 16,000-byte static array in internal DRAM, held for the whole
     * life of the firmware to be used for 250 ms at a time. Internal DRAM is
     * this board's scarcest resource -- tdsh's SSH server refuses to start below
     * 65,536 bytes free -- and adding the NES component's 4 KB of .bss pushed it
     * under that floor: `ssh start` began reporting "insufficient internal RAM
     * (60143 bytes free)". That is EMU-018 happening a second time, in a
     * peripheral with nothing to do with the change that caused it.
     *
     * So the buffer moves to the heap, where 16 KB lands in PSRAM and is released
     * on the way out. PSRAM latency does not matter here: the buffer is filled
     * once and handed to I2S, which DMAs it and buffers it -- nobody reads it back
     * per-sample from the CPU.
     *
     * Allocated per call rather than kept, because a beep is a rare event on a
     * watch and holding 16 KB of PSRAM permanently to avoid a malloc on each one
     * is the wrong trade. A failed allocation falls back to the internal-DRAM
     * behaviour of returning quietly, which is what a beep already does when
     * there is no speaker.
     */
    const int frames = BEEP_MS * SOUND_RATE_HZ / 1000;
    const size_t want = (size_t)frames * 2 * sizeof(int16_t);
    size_t written = 0;
    esp_err_t err;
    int16_t *buf = (int16_t *)heap_caps_malloc(want, MALLOC_CAP_SPIRAM);

    if (!s_tx)
    {
        ESP_LOGW(TAG, "beep: no channel");
        return;
    }

    if (!buf)
    {
        ESP_LOGW(TAG, "beep: no psram for %u bytes", (unsigned)want);
        return;
    }

    beep_samples(buf, frames);

    /* The timeout is well beyond the tone's own length rather than equal to it.
     * 200 ms of stereo samples at 16 kHz is 25,600 bytes, which takes 200 ms to
     * clock out; with four DMA buffers only the first 1024 frames queue
     * immediately, so a 200 ms timeout sat exactly on the boundary and would
     * have failed or truncated intermittently. */
    err = i2s_channel_write(s_tx, buf, want, &written, pdMS_TO_TICKS(1000));

    /* REPORT WHAT ACTUALLY HAPPENED. This line is the difference between "the
     * amp is silent" and "we never handed it any samples", and without it the
     * two are indistinguishable from the outside -- which is where the last two
     * attempts ended up. Short, for the Log Viewer's 40 columns. */
    ESP_LOGI(TAG, "beep %u/%u %s", (unsigned)written, (unsigned)want,
             err == ESP_OK ? "ok" : esp_err_to_name(err));

    heap_caps_free(buf);
}

/* ------------------------------------------------------------ wav playback */

/*
 * Streaming playback, so a tens-of-megabytes file costs a few kilobytes of RAM.
 *
 * The size of the read is chosen against the DMA's own depth: four buffers of
 * 1024 frames is 4096 frames, which at 44.1 kHz is 93 ms of audio already
 * queued. A 4 KiB read at the 172 KiB/s the stream needs is under 25 ms of
 * file I/O, so the queue has roughly a 4x margin and a single slow read cannot
 * empty it. That margin is the whole design; without it a hiccup on a shared
 * SPI bus becomes an audible gap.
 */
#define WAV_CHUNK       4096
#define WAV_STACK       4096
#define WAV_TASK_PRIO   4

static char s_path[192];            /* the caller's string does not outlive the call */
static volatile bool s_playing;
static volatile bool s_stop;
static TaskHandle_t s_player;

/* The rate and slot count to restore after a file, so the beep stays a beep. */
static uint32_t s_home_rate = SOUND_RATE_HZ;

bool sound_is_playing(void)
{
    return s_playing;
}

void sound_stop(void)
{
    s_stop = true;
}

static bool set_std(uint32_t rate, bool stereo)
{
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    i2s_std_slot_config_t slot = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                                   stereo ? I2S_SLOT_MODE_STEREO
                                                                          : I2S_SLOT_MODE_MONO);

    if (!s_tx)
        return false;

    /* The channel must be stopped before either is changed, or the change is
     * ignored -- the API says so and a silent no-op here would play the whole
     * file at the wrong rate. */
    if (i2s_channel_disable(s_tx) != ESP_OK)
        return false;
    if (i2s_channel_reconfig_std_clock(s_tx, &clk) != ESP_OK)
        return false;
    if (i2s_channel_reconfig_std_slot(s_tx, &slot) != ESP_OK)
        return false;
    return i2s_channel_enable(s_tx) == ESP_OK;
}

/* Write one buffer, retrying on timeout rather than dropping it: a dropped
 * buffer is a click, and this is the inner loop of the whole feature. */
static bool write_all(const uint8_t *p, size_t n)
{
    while (n > 0)
    {
        size_t wrote = 0;
        esp_err_t err = i2s_channel_write(s_tx, p, n, &wrote, pdMS_TO_TICKS(1000));

        if (err != ESP_OK && err != ESP_ERR_TIMEOUT)
            return false;
        if (wrote == 0)
        {
            if (s_stop)
                return false;
            continue;                       /* timed out with nothing taken */
        }
        p += wrote;
        n -= wrote;
    }
    return true;
}

static void play_task(void *arg)
{
    const char *path = (const char *)arg;   /* s_path, copied by the caller */
    FILE *f = NULL;
    uint8_t *hdr = NULL;
    uint8_t *buf = NULL;
    int16_t *dup = NULL;
    wav_info_t info;
    size_t got;
    uint32_t left;
    uint32_t rate_before = s_home_rate;
    bool mono = false;
    bool reconfigured = false;

    f = fopen(path, "rb");
    if (!f)
    {
        /* WHY it failed, not just that it did. "no file" cost a cycle of
         * guessing at paths: a wrong path and an unreadable one and a
         * filesystem that is not mounted all look identical without errno, and
         * this watch gives no shell for finding out by hand. The errno name is
         * short enough for the 40-column ring. */
        ESP_LOGE(TAG, "open: %s", strerror(errno));
        goto done;
    }

    /* A header is small, but chunks can push `data` a long way in and a file
     * carrying cover art can carry it megabytes in. 64 KiB covers anything
     * sane; past that the search gives up with an explanation rather than
     * reading a whole file looking for it. */
    hdr = malloc(65536);
    buf = malloc(WAV_CHUNK);
    if (!hdr || !buf)
    {
        ESP_LOGE(TAG, "no ram");
        goto done;
    }

    got = fread(hdr, 1, 65536, f);
    {
        wav_status_t st = wav_parse(hdr, got, &info);

        if (st != WAV_OK)
        {
            ESP_LOGE(TAG, "wav: %s", wav_status_str(st));
            goto done;
        }
    }

    if (info.format != 1 || info.bits != 16 || info.channels < 1 || info.channels > 2 ||
        info.rate < 8000 || info.rate > 96000)
    {
        /* Short, because these are the numbers that say why. */
        ESP_LOGE(TAG, "fmt%d %db %dch %lu", info.format, info.bits, info.channels,
                 (unsigned long)info.rate);
        goto done;
    }

    mono = info.channels == 1;

    /* How much is really there: a file whose header was written before its data
     * can claim more than the filesystem holds, and reading past the end would
     * play whatever the allocator had. */
    if (fseek(f, 0, SEEK_END) == 0)
    {
        long real = ftell(f);
        long avail = real > 0 ? (long)(real - (long)info.data_offset) : 0;

        if (avail <= 0)
        {
            ESP_LOGE(TAG, "no data");
            goto done;
        }
        if ((uint32_t)avail < info.data_bytes)
            info.data_bytes = (uint32_t)avail;
    }
    if (fseek(f, (long)info.data_offset, SEEK_SET) != 0)
    {
        ESP_LOGE(TAG, "seek failed");
        goto done;
    }

    if (!set_std(info.rate, !mono))
    {
        ESP_LOGE(TAG, "i2s reconfig");
        goto done;
    }
    reconfigured = true;

    s_playing = true;
    left = info.data_bytes;

    /* One short line, then silence: a per-chunk log would bury the ring, which
     * is the mistake this port has already recorded twice. */
    ESP_LOGI(TAG, "play %luHz %s %lus", (unsigned long)info.rate, mono ? "mono" : "st",
             (unsigned long)(info.data_bytes / (info.rate * (mono ? 2 : 4))));

    while (left > 0 && !s_stop)
    {
        size_t want = left < WAV_CHUNK ? left : WAV_CHUNK;
        size_t n = fread(buf, 1, want, f);

        if (n == 0)
            break;
        left -= (uint32_t)n;

        if (mono)
        {
            /* Duplicate into both slots. The amplifier averages (L+R)/2 in its
             * default SD mode, so a mono file sent to one slot plays at half
             * amplitude -- the same trap sound_init() records for the beep. */
            size_t frames = n / 2, i;

            /* Allocated with the read buffer: a static array here would be 8 KiB
             * of permanent internal RAM for a path most files never take. */
            if (!dup)
            {
                dup = malloc(WAV_CHUNK * 2);
                if (!dup)
                {
                    ESP_LOGE(TAG, "no ram");
                    break;
                }
            }
            for (i = 0; i < frames && i < WAV_CHUNK / 2; i++)
            {
                int16_t v = (int16_t)((uint16_t)buf[2 * i] | ((uint16_t)buf[2 * i + 1] << 8));

                dup[2 * i] = v;
                dup[2 * i + 1] = v;
            }
            if (!write_all((const uint8_t *)dup, frames * 4))
                break;
        }
        else if (!write_all(buf, n))
            break;
    }

    ESP_LOGI(TAG, "stop");

done:
    if (reconfigured)
        set_std(rate_before, true);      /* the beep's own configuration */
    s_playing = false;
    s_stop = false;
    free(hdr);
    free(buf);
    free(dup);
    if (f)
        fclose(f);
    s_player = NULL;
    vTaskDelete(NULL);
}

bool sound_play_wav(const char *path)
{
    if (!path || !path[0])
        return false;
    if (!s_tx)
    {
        ESP_LOGW(TAG, "no channel");
        return false;
    }
    if (s_playing)
    {
        ESP_LOGW(TAG, "already playing");
        return false;
    }

    /* COPY THE PATH BEFORE STARTING THE TASK. A caller's string is very often a
     * stack buffer -- a command handler's argv, an app's local -- and it is gone
     * by the time this task runs, so handing the pointer over would read freed
     * stack and open whatever path happened to be there. */
    if (strlen(path) >= sizeof(s_path))
    {
        ESP_LOGE(TAG, "path too long");
        return false;
    }
    snprintf(s_path, sizeof(s_path), "%s", path);

    s_stop = false;
    if (xTaskCreatePinnedToCore(play_task, "tdwav", WAV_STACK, (void *)s_path, WAV_TASK_PRIO,
                                &s_player, 0) != pdPASS)
    {
        ESP_LOGE(TAG, "no task");
        s_player = NULL;
        return false;
    }
    return true;
}

/* ------------------------------------------------------- live sample stream */

/*
 * A continuous PCM source, for an app that generates audio as it runs rather
 * than playing a file.
 *
 * WHY THIS IS NOT sound_play_wav(). That one owns a task, reads from the
 * filesystem and may block for the length of a track. This one never blocks the
 * caller and never reads a file: it exists for an emulator frame loop that has
 * about a millisecond of slack and cannot afford to wait for anything.
 *
 * THE WHOLE DESIGN IS THE ZERO TIMEOUT. i2s_channel_write() blocks until the
 * DMA takes the samples, and the DMA only drains at the output rate: 613
 * samples at 44.1 kHz is 13.9 ms of real time. Called from the frame loop that
 * would be a stall longer than the entire frame budget, every frame, which is
 * the single worst thing audio could do to this port. So the write timeout is 0
 * and a full ring means the samples are DROPPED. A dropped sample is a tick; a
 * stalled frame is a visible stutter, and the trade is not close.
 *
 * The ring is why dropping is rare rather than constant: sound_init() already
 * builds four DMA buffers of 1024 frames, which at 44.1 kHz is 93 ms of audio
 * queued ahead. A frame loop producing 13.9 ms per frame fills that in about
 * six frames, so in steady state each write finds room.
 *
 * MONO IN, STEREO OUT. The generator produces one channel and the amplifier
 * averages (L+R)/2, so a sample written to one slot is half amplitude. The
 * caller's buffer is duplicated into both slots here rather than in the
 * emulator, because the emulator's per-frame budget is the scarce thing.
 */

#define STREAM_FRAMES 1024        /* > one frame's 613, so a chunk always fits */

static int16_t *s_stream;         /* PSRAM: touched once per frame, not per sample */
static bool s_stream_open;
static volatile uint32_t s_stream_drops;

bool sound_stream_open(int rate)
{
    if (!s_tried && !sound_init())
        return false;
    if (!s_tx)
        return false;

    /* The rate comes from the generator, not from a constant: MinxAudio.h fixes
     * its output at MINX_AUDIOFREQ 44100 and a mismatch here is a pitch shift of
     * however wrong the ratio is, which sounds like a tuning fault rather than a
     * configuration one. */
    if (!set_std((uint32_t)rate, true))
        return false;

    if (!s_stream)
    {
        s_stream = (int16_t *)heap_caps_malloc(STREAM_FRAMES * 2 * sizeof(int16_t),
                                               MALLOC_CAP_SPIRAM);
        if (!s_stream)
            return false;
    }

    s_stream_drops = 0;
    s_stream_open = true;
    return true;
}

/* Write `frames` mono samples. Returns how many were accepted; the rest were
 * dropped because the ring was full. Never blocks. */
size_t sound_stream_write(const int16_t *mono, int frames)
{
    if (!s_stream_open || !s_tx || !s_stream || frames <= 0)
        return 0;

    if (frames > STREAM_FRAMES)
        frames = STREAM_FRAMES;

    for (int i = 0; i < frames; i++)
    {
        s_stream[2 * i] = mono[i];
        s_stream[2 * i + 1] = mono[i];
    }

    const size_t want = (size_t)frames * 2 * sizeof(int16_t);
    size_t wrote = 0;

    /* Timeout 0 by design -- see the note above. ESP_ERR_TIMEOUT here is the
     * normal full-ring case and not a failure. */
    i2s_channel_write(s_tx, s_stream, want, &wrote, 0);

    const size_t got = wrote / (2 * sizeof(int16_t));

    if (got < (size_t)frames)
        s_stream_drops += (uint32_t)((size_t)frames - got);

    return got;
}

void sound_stream_close(void)
{
    if (!s_stream_open)
        return;

    s_stream_open = false;

    /* Back to the beep's own rate, or the next BEL is pitched wrong. */
    set_std(SOUND_RATE_HZ, false);

    if (s_stream)
    {
        heap_caps_free(s_stream);
        s_stream = NULL;
    }
}

uint32_t sound_stream_dropped(void)
{
    return s_stream_drops;
}

/* The terminal's BEL. The user pointer exists because the vterm's callbacks all
 * carry one; the beep does not need it. */
void sound_bell(void *user)
{
    (void)user;
    sound_beep();
}

/* The port's BEL hook, called through td_sysinfo_t by whichever app's terminal
 * emulator saw the 0x07. This is the path that actually works: the Terminal app
 * owns its own td_vterm_t, so a callback installed on the renderer's copy never
 * fires for terminal output. */
void board_bell(void)
{
    sound_beep();
}
