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
    /* Stereo frames: two samples each, so twice the buffer the mono version
     * needed for the same tone length. */
    static int16_t buf[2 * BEEP_MS * SOUND_RATE_HZ / 1000];
    const int frames = BEEP_MS * SOUND_RATE_HZ / 1000;
    const size_t want = sizeof(buf);
    size_t written = 0;
    esp_err_t err;

    if (!s_tx)
    {
        ESP_LOGW(TAG, "beep: no channel");
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
