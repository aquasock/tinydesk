/*
 * flacmod.cpp - micro-flac as a loadable module.
 *
 * THE POINT. micro-flac costs 31.7 KB inline in the app, where 24 KB of headroom
 * remained. As a module it runs from the `modules` flash partition, costs the
 * app image nothing, and the code is 26 KB of text with 20 bytes of bss --
 * measured before this file existed.
 *
 * WHAT A MODULE MUST NOT DO, all of it learned the hard way this cycle:
 *   - no libc (it cannot resolve an external symbol)
 *   - no initialised globals or .bss (the partition is read-only flash and no
 *     loader zeroes anything)
 *   - no data reads through its own mapped address (instruction bus)
 *
 * The decoder's own object is heap-allocated, so its state is fine; what it must
 * not do is call heap_caps_malloc_prefer DIRECTLY, which is why FLAC_MALLOC is
 * redirected to the host table for the module build.
 *
 * THE ENTRY TAKES ITS BUFFERS FROM THE CALLER. modcall_t carries the compressed
 * window and the PCM destination, so the firmware keeps control of every
 * allocation it has to live within -- which is the whole reason a module is
 * worth having on a board this short of RAM.
 */

#include "modhost.h"
#include "modload.h"

#include "micro_flac/flac_decoder.h"

#include <new>

__attribute__((section(".modheader"), used))
const modload_header_t mod_header = {
    .magic = MODLOAD_MAGIC,
    .abi_version = MODLOAD_ABI_VERSION,
    .size = 0,                  /* patched after linking */
    .entry_offset = 0,          /* patched after linking */
    .name = "flac",
    .linked_inst = 0,           /* patched after linking */
    .linked_data = 0,           /* patched after linking */
    .linked_host = 0,                              /* patched after linking */
};

static const modhost_t *host(void)
{
    return *(const modhost_t *const volatile *)(uintptr_t)MODHOST_PTR_AT;
}

/*
 * A decoder handle the CALLER owns.
 *
 * The decoder object cannot live in the module's .bss, and having the module
 * hold a static pointer would be writable state it does not have. So the caller
 * allocates this struct and passes it in, exactly as it passes the PCM buffer.
 * The FLACDecoder itself is placed in the tail of that block with placement new.
 */
/*
 * PADDED TO 32 BYTES ON PURPOSE. The FLACDecoder is placement-new'd immediately
 * after this struct, so wherever this ends the decoder begins: at 20 bytes it
 * would start at a 4-byte offset inside an 8-byte-aligned block, and the decoder
 * holds pointers and 64-bit state. 32 keeps the boundary aligned.
 */
typedef struct
{
    uint32_t magic;
    uint32_t ready;
    int32_t rate;
    int32_t channels;
    int32_t bits;
    int32_t last_result;        /* what the decoder returned, for diagnosis */
    size_t last_consumed;
    uint32_t calls;
} flac_handle_t;

_Static_assert(sizeof(flac_handle_t) == 32, "handle must keep 8-byte alignment for the decoder");

#define FLAC_HANDLE_MAGIC   0x464C4143U     /* "FLAC" */

/*
 * The module's one entry point.
 *
 * call->in / in_len       compressed bytes
 * call->consumed          out: how many the decoder used
 * call->pcm / capacity    out: interleaved samples
 * call->pcm_samples       out: how many written
 * call->state             caller-allocated block for the decoder object
 * call->state_size        its size
 *
 * Returns MODCALL_OK / NEED_INPUT / END / ERROR.
 */
extern "C" int32_t mod_entry(modcall_t *call);

extern "C" int32_t mod_entry(modcall_t *call)
{
    const modhost_t *h = host();

    if (!h || h->abi_version != MODHOST_ABI_VERSION)
        return MODCALL_ERROR;
    if (!call || !call->state || call->state_size < sizeof(flac_handle_t))
        return MODCALL_ERROR;

    flac_handle_t *fh = (flac_handle_t *)call->state;
    micro_flac::FLACDecoder *dec;

    /*
     * TEARDOWN. A call with no input asks us to destroy the decoder and release
     * what it allocated through the host table.
     *
     * WITHOUT THIS THE BUFFERS LEAK ON EVERY TRACK. micro-flac keeps its
     * working buffers until ~FLACDecoder(), and the caller only ever freed the
     * state BLOCK it owns, not what lived inside it -- so each track leaked its
     * buffer, measured at 41 KB of PSRAM and 356 bytes of internal RAM. Four or
     * five tracks of that, and a later allocation fails.
     */
    if (!call->in && call->in_len == 0)
    {
        if (fh->magic == FLAC_HANDLE_MAGIC)
        {
            dec = (micro_flac::FLACDecoder *)((uint8_t *)fh + sizeof(flac_handle_t));
            dec->~FLACDecoder();
            fh->magic = 0;
            fh->ready = 0;
        }
        return MODCALL_OK;
    }

    fh->calls++;

    if (fh->magic != FLAC_HANDLE_MAGIC)
    {
        /* First call: construct the decoder inside the caller's block. */
        dec = new ((uint8_t *)fh + sizeof(flac_handle_t)) micro_flac::FLACDecoder();
        fh->magic = FLAC_HANDLE_MAGIC;
        fh->ready = 0;
    }
    else
    {
        dec = (micro_flac::FLACDecoder *)((uint8_t *)fh + sizeof(flac_handle_t));
        /* The object already exists; re-deriving the pointer from the block is
         * what lets this survive being called repeatedly without any static. */
        if (!dec)
            return MODCALL_ERROR;
    }

    size_t out_samples = fh->ready ? dec->get_output_buffer_size_samples() : 0;

    fh->rate = fh->rate;        /* keep the members live for the debugger */
    size_t consumed = 0, samples = 0;
    const size_t pcm_cap = call->pcm_capacity;

    if (out_samples > pcm_cap)
        out_samples = pcm_cap;

    const micro_flac::FLACDecoderResult r =
        dec->decode(call->in, call->in_len, fh->ready ? call->pcm : nullptr,
                    out_samples, consumed, samples);

    fh->last_result = (int32_t)r;
    fh->last_consumed = consumed;

    if (call->consumed)
        *call->consumed = consumed;

    if (r == micro_flac::FLAC_DECODER_HEADER_READY)
    {
        const micro_flac::FLACStreamInfo &si = dec->get_stream_info();

        fh->ready = 1;
        fh->rate = (int32_t)si.sample_rate();
        fh->channels = (int32_t)si.num_channels();
        fh->bits = (int32_t)si.bits_per_sample();
        if (call->pcm_samples)
            *call->pcm_samples = 0;
        return MODCALL_OK;
    }

    if (r == micro_flac::FLAC_DECODER_SUCCESS)
    {
        if (call->pcm_samples)
            *call->pcm_samples = samples;
        return MODCALL_OK;
    }

    if (r == micro_flac::FLAC_DECODER_NEED_MORE_DATA)
    {
        if (call->pcm_samples)
            *call->pcm_samples = 0;
        return MODCALL_NEED_INPUT;
    }

    if (r == micro_flac::FLAC_DECODER_END_OF_STREAM)
        return MODCALL_END;

    return MODCALL_ERROR;
}
