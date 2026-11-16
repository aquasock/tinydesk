/*
 * musicdec.cpp - the whole of micro-flac's C++ surface, behind musicdec.h.
 *
 * micro-flac is Apache-2.0 and vendored unmodified in
 * components/micro-flac/. Nothing here reaches into its internals: the decoder
 * is used only through its documented streaming API, so updating the component
 * is a copy and a rebuild.
 *
 * THE 32-BIT OVERLOAD IS USED ON PURPOSE. micro-flac can emit the stream's
 * native packing (2 bytes for 16-bit, 3 for 24) or everything as int32
 * left-justified. The second is chosen because it makes the conversion to the
 * amplifier's 16-bit input uniform and exact: a right shift by (32 - 16),
 * regardless of whether the file was 16- or 24-bit. With the native path this
 * file would need a different case per depth, and getting the 24-bit case
 * subtly wrong would sound like distortion rather than failing.
 *
 * DOWNMIX IS NOT DONE HERE. The MAX98357A averages (L+R)/2 itself, so stereo
 * reaches it correctly as interleaved pairs; folding to mono in software would
 * only lose the separation the amplifier can still carry.
 */
#include "musicdec.h"

#include "esp_heap_caps.h"

#include "micro_flac/flac_decoder.h"

#include <cstring>
#include <new>

struct music_dec
{
    micro_flac::FLACDecoder decoder;
    int32_t *pcm;               /* PSRAM: one block, sized from STREAMINFO */
    size_t pcm_samples;         /* capacity, in samples (frames x channels) */
    bool ready;
    int rate, channels, bits;
    uint64_t total_frames;
    bool ended;
};

music_dec_t *music_dec_open(void)
{
    music_dec_t *d = new (std::nothrow) music_dec_t();
    if (!d)
        return nullptr;
    d->pcm = nullptr;
    d->pcm_samples = 0;
    d->ready = false;
    d->rate = d->channels = d->bits = 0;
    d->total_frames = 0;
    d->ended = false;
    return d;
}

void music_dec_close(music_dec_t *d)
{
    if (!d)
        return;
    if (d->pcm)
        heap_caps_free(d->pcm);
    delete d;
}

bool music_dec_ready(const music_dec_t *d) { return d && d->ready; }
int music_dec_rate(const music_dec_t *d) { return d ? d->rate : 0; }
int music_dec_channels(const music_dec_t *d) { return d ? d->channels : 0; }
int music_dec_bits(const music_dec_t *d) { return d ? d->bits : 0; }
uint64_t music_dec_total_frames(const music_dec_t *d) {
    return d ? d->total_frames : 0;
}

int music_dec_read(music_dec_t *d, const uint8_t *in, size_t in_len,
                              size_t *consumed, int16_t *out, size_t out_frames,
                              size_t *samples)
{
    if (!d || !consumed || !samples)
        return -2;

    *consumed = 0;
    *samples = 0;

    if (d->ended)
        return -1;

    /* Before the header is parsed there is no output buffer to size, so the
     * first call passes no PCM at all; that is the documented shape. */
    const size_t out_samples = d->ready ? d->pcm_samples : 0;

    const micro_flac::FLACDecoderResult r =
        d->decoder.decode(in, in_len, d->ready ? d->pcm : nullptr, out_samples,
                          *consumed, *samples);

    if (r == micro_flac::FLAC_DECODER_HEADER_READY)
    {
        const micro_flac::FLACStreamInfo &si = d->decoder.get_stream_info();

        d->channels = (int)si.num_channels();
        d->bits = (int)si.bits_per_sample();
        d->rate = (int)si.sample_rate();
        d->total_frames = si.total_samples_per_channel();
        d->ready = true;

        /* One block, sized from the file rather than from a worst case: this
         * is the allocation that made micro-flac viable where rockbox's static
         * 7-channel buffers were not. PSRAM first, internal as fallback. */
        d->pcm_samples = d->decoder.get_output_buffer_size_samples();
        d->pcm = (int32_t *)heap_caps_malloc_prefer(d->pcm_samples * sizeof(int32_t), 2,
                                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!d->pcm)
            return -2;

        return 1;
    }

    if (r == micro_flac::FLAC_DECODER_SUCCESS)
    {
        /* Convert what was just decoded: `*samples` is interleaved across all
         * channels, so frames = samples / channels. */
        if (!d->ready || !out || d->channels < 1)
            return 1;

        const size_t frames = *samples / (size_t)d->channels;

        if (frames > out_frames)
            return -2;          /* cannot happen: the buffer is sized for a block */

        const int shift = 32 - 16;      /* left-justified int32 -> top 16 bits */

        for (size_t i = 0; i < frames; i++)
        {
            if (d->channels == 1)
            {
                const int16_t v = (int16_t)(d->pcm[i] >> shift);

                out[i * 2] = v;
                out[i * 2 + 1] = v;     /* mono source to both slots */
            }
            else
            {
                out[i * 2] = (int16_t)(d->pcm[i * 2] >> shift);
                out[i * 2 + 1] = (int16_t)(d->pcm[i * 2 + 1] >> shift);
            }
        }

        *samples = frames;
        return 1;
    }

    if (r == micro_flac::FLAC_DECODER_NEED_MORE_DATA)
        return 0;

    if (r == micro_flac::FLAC_DECODER_END_OF_STREAM)
    {
        d->ended = true;
        return -1;
    }

    return -2;              /* a negative FLAC_DECODER_ERROR_* */
}
