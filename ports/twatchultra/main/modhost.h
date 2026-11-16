/*
 * modhost.h - the API a module uses to reach the firmware, and the blob side of
 * the contract.
 *
 * OPTION (a): A TABLE, NOT A SET OF BOUND SYMBOLS.
 *
 * A blob in the `modules` partition is linked at a fixed flash address and has
 * no relocations it can resolve -- it cannot call libc, and it cannot call the
 * firmware by name. Two ways to give it a way back:
 *
 *   (b) bind each function with --defsym malloc=<address>. Simple, but every
 *       blob then breaks the moment the app is rebuilt, because the addresses
 *       move. That defeats the point of a separate, independently updatable
 *       codec partition.
 *
 *   (a) pass ONE table of function pointers. This file is that table.
 *
 * HOW THE BLOB FINDS THE TABLE, given that it executes from READ-ONLY flash and
 * therefore cannot keep a pointer in its own .bss:
 *
 *   1. The firmware declares `const modhost_t *const g_modhost_ptr` in its own
 *      RAM, and fills it at boot.
 *   2. The blob build reads that symbol's ADDRESS out of the app's ELF with nm
 *      and passes it as -DMODHOST_PTR_AT=<addr> (see modules/Makefile).
 *   3. The blob's shim does `*(const modhost_t *const *)MODHOST_PTR_AT`.
 *
 * So the address is DERIVED AT BUILD TIME from the app it will run against,
 * not typed in. It is still an absolute address, and it still moves when the
 * app is rebuilt -- but the blob is rebuilt from the same app in the same
 * breath, which is the honest constraint: A BLOB BELONGS TO THE FIRMWARE BUILD
 * THAT PRODUCED IT. What the table buys is that only ONE address has to be
 * right, not one per library function, and that the module's own code is free
 * of frozen function addresses.
 *
 * A blob may not use libc, initialised globals, or any external symbol other
 * than the ones routed here. Everything else it needs must resolve inside
 * itself.
 */
#ifndef TWATCH_MODHOST_H
#define TWATCH_MODHOST_H

#include <stddef.h>
#include <stdint.h>

/*
 * Bump when anything in this struct or the entry signature changes. A blob
 * built against a different version is refused rather than called, because a
 * table whose layout has shifted turns a call into a jump to garbage.
 */
#define MODHOST_ABI_VERSION 1

typedef struct
{
    uint32_t abi_version;       /* must equal MODHOST_ABI_VERSION */

    /* Memory. The module decodes audio, so these are the four the decoder
     * actually reaches: malloc/free and C++'s operator new/delete both end up
     * here, and the buffers want PSRAM, which is why they are not libc. */
    void *(*malloc)(size_t bytes);
    void (*free)(void *p);
    void *(*malloc_prefer_psram)(size_t bytes);

    /* Strings and buffers. */
    void *(*memcpy)(void *dst, const void *src, size_t n);
    void *(*memset)(void *dst, int c, size_t n);

    /* Logging, so a module can report a problem instead of failing silently. */
    void (*log)(int level, const char *msg);

    /* The table stays 4-byte aligned and pointer-sized throughout; a tail of
     * reserved slots means adding a function later does not move any existing
     * one. */
    void *reserved[4];
} modhost_t;

/*
 * The entry point of a module. Deliberately narrow: a blob that decodes audio
 * is asked to do one thing, and a wider signature would make the first
 * integration a test of argument marshalling rather than of the decoder.
 *
 * `out` and `in` are caller-owned; the module never allocates the caller's
 * buffers, which is what keeps a blob's memory use visible to the firmware
 * that has to live within the budget.
 */
typedef struct
{
    const uint8_t *in;
    size_t in_len;
    size_t *consumed;           /* out: bytes the module used */

    int32_t *pcm;               /* caller-allocated, interleaved */
    size_t pcm_capacity;        /* samples available in pcm */
    size_t *pcm_samples;        /* out: samples written */

    /*
     * THE MODULE'S OWN STATE, and the caller allocates it.
     *
     * A blob executes from read-only flash and has no .bss, so it cannot keep a
     * decoder object or even a pointer to one. Rather than pin a fixed address
     * in the firmware's RAM -- which would be another build-time constant to
     * keep in step -- the caller hands the module a block and the module places
     * whatever it needs inside it. That also keeps the cost visible to the
     * firmware that has to live within the budget.
     *
     * `state_size` must be at least what the module asks for; the module
     * returns MODCALL_ERROR rather than overrunning.
     */
    void *state;
    size_t state_size;
} modcall_t;

/* Return codes a module entry may give. */
#define MODCALL_OK           0
#define MODCALL_NEED_INPUT   1
#define MODCALL_END          2
#define MODCALL_ERROR      (-1)

typedef int32_t (*mod_entry_fn)(modcall_t *call);

/* Firmware side only: filled at boot and pointed at by g_modhost_ptr. */
const modhost_t *modhost_table(void);

#endif
