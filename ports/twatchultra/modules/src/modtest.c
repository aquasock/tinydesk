/*
 * The smallest possible module: prove that code in the `modules` partition can
 * be mapped as instructions and CALLED across a partition boundary.
 *
 * Everything in this file exists to answer a question, so each part is
 * separable when something fails:
 *
 *   - arguments arrive        -> return a * b
 *   - a loop runs             -> sum 1..10 must be 55
 *   - an internal call works  -> mul() is a separate function
 *   - the caller's own state  -> heap_free/heap_largest must be non-zero and
 *                                sane, which they would not be if the blob were
 *                                reading from a stale or bogus view
 *
 * Nothing here touches libc, a global with an initialiser, or any symbol
 * outside this file. Every one of those would drag in a relocation the blob
 * cannot resolve, and the point of the test is to fail for the INTERESTING
 * reason if it fails at all.
 */

#include "modload.h"

/* Placed at offset 0 of the partition by the linker script, so a loader finds
 * it before it knows anything else about the blob. */
__attribute__((section(".modheader"), used))
const modload_header_t mod_header = {
    .magic = MODLOAD_MAGIC,
    .abi_version = MODLOAD_ABI_VERSION,
    .size = 0,                  /* patched by the build, see Makefile */
    .entry_offset = 0,          /* patched by the build */
    .name = "modtest",
};

static int mul(int a, int b)
{
    return a * b;
}

int32_t mod_entry(int32_t a, int32_t b, uint32_t heap_free, uint32_t heap_largest);

int32_t mod_entry(int32_t a, int32_t b, uint32_t heap_free, uint32_t heap_largest)
{
    int32_t sum = 0;

    for (int i = 1; i <= 10; i++)
        sum += i;

    if (sum != 55)
        return -1;              /* the loop, or the stack, is not working */
    if (heap_free == 0 || heap_largest == 0)
        return -2;              /* the arguments did not arrive */
    if (heap_largest > heap_free)
        return -3;              /* the figures are nonsense */

    return mul(a, b);           /* 6 x 7 = 42, through an internal call */
}
