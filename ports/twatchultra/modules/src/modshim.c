/*
 * modshim.c - the blob side: route every external symbol through the host table.
 *
 * This file is compiled INTO the module, not into the firmware. It is what the
 * decoder's `malloc`, `operator new`, `memcpy` and friends resolve to, so the
 * decoder itself needs no knowledge that it is running from a flash partition
 * behind an API table.
 *
 * HOW IT FINDS THE TABLE. `MODHOST_PTR_AT` is the address of the firmware's
 * `g_modhost_ptr`, injected by modules/Makefile from the app's ELF (`nm`), so
 * it is derived from the firmware this blob is built against rather than typed
 * in by hand. See modhost.h for why the blob cannot simply keep a pointer: it
 * executes from read-only flash.
 *
 * WHAT IS NOT SHIMMED. `__ashldi3` and `__ashrdi3` come from libgcc, which is
 * self-contained and linked in (-lgcc). `__assert_func` and `atexit` are
 * one-line stubs below, because the firmware cannot usefully service them and a
 * decoder that asserts should instead report an error code.
 *
 * `-fno-threadsafe-statics` is what keeps `__cxa_guard_acquire` out of the
 * object list; without it every function-local static in the decoder would pull
 * in the C++ guard machinery, which a blob cannot satisfy.
 */
#include "modhost.h"

#define MODHOST_PTR_AT_DEFAULT 0u

#ifndef MODHOST_PTR_AT
#error "MODHOST_PTR_AT must be defined: see modules/Makefile, which derives it from the app ELF"
#endif

static const modhost_t *host(void)
{
    /* One dereference of a known address. The value is installed by the
     * firmware before any module is called. */
    return *(const modhost_t *const volatile *)(uintptr_t)MODHOST_PTR_AT;
}

/* ------------------------------------------------------------------ memory */

void *malloc(size_t n)
{
    const modhost_t *h = host();

    return (h && h->malloc) ? h->malloc(n) : 0;
}

void free(void *p)
{
    const modhost_t *h = host();

    if (h && h->free)
        h->free(p);
}

/* C++ allocation. The decoder's containers use these, so they must NOT be
 * left to the firmware's libstdc++ -- a blob has no dynamic linker to find it. */
void *operator new(size_t n) { return malloc(n); }
void *operator new[](size_t n) { return malloc(n); }
void operator delete(void *p) noexcept { free(p); }
void operator delete[](void *p) noexcept { free(p); }
void operator delete(void *p, size_t) noexcept { free(p); }
void operator delete[](void *p, size_t) noexcept { free(p); }

/* ------------------------------------------------------------------ memory */

void *memcpy(void *dst, const void *src, size_t n)
{
    const modhost_t *h = host();

    if (h && h->memcpy)
        return h->memcpy(dst, src, n);

    /* No host: a byte loop still works and keeps the decoder from crashing
     * silently if the table were ever missing. */
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;

    while (n--)
        *d++ = *s++;
    return dst;
}

void *memset(void *dst, int c, size_t n)
{
    const modhost_t *h = host();

    if (h && h->memset)
        return h->memset(dst, c, n);

    unsigned char *d = (unsigned char *)dst;

    while (n--)
        *d++ = (unsigned char)c;
    return dst;
}

/* --------------------------------------------------------------- reporting */

/* A decoder that hits an internal assertion has no business halting the watch.
 * The stub reports and returns; the decoder then returns an error code up the
 * stack, which the caller handles. */
void __assert_func(const char *file, int line, const char *fn, const char *expr)
{
    const modhost_t *h = host();

    (void)file;
    (void)line;
    (void)fn;

    /* NO STATIC BUFFER HERE. A blob has no writable .bss -- link.ld discards it,
     * because there is no loader to zero it and the partition is read-only
     * flash. An earlier version built a message in `static char msg[120]` and
     * failed to link with "`.bss` referenced in section `.text`". The
     * expression string is already in the module's own .rodata, so it is
     * passed through unchanged. */
    if (h && h->log)
        h->log(0, expr ? expr : "module assertion");
}

/* Nothing to flush at exit; a module is never "exited". */
int atexit(void (*fn)(void))
{
    (void)fn;
    return 0;
}

/* Containers throw on OOM, and the module is built -fno-exceptions, so this
 * symbol exists but is never unwound through. Reporting and returning keeps the
 * process alive; the caller sees a decode error instead of a reset. */
void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) { }

void _ZSt20__throw_length_errorPKc(const char *what)
{
    const modhost_t *h = host();

    (void)what;
    if (h && h->log)
        h->log(0, "module: allocation grew past its limit");
}

void _ZSt19__throw_logic_errorPKc(const char *what)
{
    const modhost_t *h = host();

    (void)what;
    if (h && h->log)
        h->log(0, "module: logic error");
}

void _ZSt17__throw_bad_allocv(void)
{
    const modhost_t *h = host();

    if (h && h->log)
        h->log(0, "module: out of memory");
}
