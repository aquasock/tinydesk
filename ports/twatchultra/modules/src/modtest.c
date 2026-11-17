/*
 * The module-API test: prove a blob can reach the firmware THROUGH A TABLE.
 *
 * The earlier test proved a blob could be mapped and called; it passed only
 * integers, because the question then was whether control transfers at all.
 * This one asks the harder question that decides whether a real codec can live
 * here: can a blob ALLOCATE MEMORY and COPY through it, from read-only flash,
 * with no libc and no bound symbols?
 *
 * Each step is separable, so a failure names itself:
 *
 *   ret  1  the host table could not be read (bad MODHOST_PTR_AT, or the app
 *           that built this blob is not the app running it)
 *   ret  2  the table is present but a function pointer is null
 *   ret  3  malloc through the table returned nothing
 *   ret  4  memcpy through the table copied the wrong bytes
 *   ret  5  the caller's buffer could not be reached
 *   ret 42  every step worked
 *
 * NOTHING HERE TOUCHES LIBC, an initialised global, or a symbol outside itself.
 * The blob has no writable .bss -- link.ld discards it, because the partition is
 * read-only flash and there is no loader to zero it -- so there is no static
 * state anywhere in this file.
 */

#include "modhost.h"

/* The header magic and layout are the loader's contract; modload.h owns them. */
#include "modload.h"

/* Placed at offset 0 by link.ld so a loader can read the magic first. */
__attribute__((section(".modheader"), used))
const modload_header_t mod_header = {
    .magic = MODLOAD_MAGIC,
    .abi_version = MODLOAD_ABI_VERSION,
    .size = 0,                      /* patched after linking */
    .entry_offset = 0,              /* patched after linking */
    .name = "modapi",
    .linked_inst = 0,               /* patched after linking */
    .linked_data = 0,               /* patched after linking */
    .linked_host = 0,                                  /* patched after linking */
};

/* The table is fetched by the shim at a build-time-derived address; this is the
 * same one dereference, repeated here so a failure is a return code, not a
 * crash. */
static const modhost_t *host(void)
{
    return *(const modhost_t *const volatile *)(uintptr_t)MODHOST_PTR_AT;
}

int32_t mod_entry(modcall_t *call);

int32_t mod_entry(modcall_t *call)
{
    const modhost_t *h = host();

    if (!h || h->abi_version != MODHOST_ABI_VERSION)
        return 1;

    if (!h->malloc || !h->free || !h->memcpy || !h->memset)
        return 2;

    /* ALLOCATE through the firmware, which knows this board's memory map and
     * can reach PSRAM. A blob cannot call malloc directly and must not try. */
    unsigned char *buf = (unsigned char *)h->malloc(64);

    if (!buf)
        return 3;

    h->memset(buf, 0, 64);

    for (int i = 0; i < 32; i++)
        buf[i] = (unsigned char)(i * 3 + 7);

    unsigned char check[32];
    h->memcpy(check, buf, 32);

    for (int i = 0; i < 32; i++)
    {
        if (check[i] != (unsigned char)(i * 3 + 7))
        {
            h->free(buf);
            return 4;
        }
    }

    /* The caller's own buffers must be reachable: this is what a decoder will
     * actually be handed. */
    if (!call || !call->pcm || call->pcm_capacity < 4)
    {
        h->free(buf);
        return 5;
    }

    call->pcm[0] = (int32_t)buf[0];
    call->pcm[1] = (int32_t)buf[1];
    call->pcm[2] = (int32_t)h->abi_version;
    call->pcm[3] = (int32_t)(uintptr_t)h;

    if (call->pcm_samples)
        *call->pcm_samples = 4;
    if (call->consumed)
        *call->consumed = 0;

    h->free(buf);
    return 42;
}
