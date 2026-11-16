/*
 * flacmalloc.h - point micro-flac's allocator at the host API table.
 *
 * micro-flac calls FLAC_MALLOC / FLAC_FREE, which by default are
 * heap_caps_malloc_prefer and heap_caps_free. Inside a module those are external
 * symbols the blob cannot resolve, so they are names of shim functions instead.
 *
 * The override is done with -DFLAC_MALLOC=modhost_malloc, which means the macro
 * expands to a NAME. Every translation unit that includes alloc.h therefore has
 * to have that name declared, which is what -include of this file does -- a
 * plain -D would leave the decoder's own .cpp files with an undeclared
 * identifier.
 *
 * The shim routes both through modhost_t, so the module never reaches the
 * firmware except through the table.
 */
#ifndef MODULES_FLACMALLOC_H
#define MODULES_FLACMALLOC_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void *modhost_malloc(size_t bytes);
void modhost_free(void *p);

#ifdef __cplusplus
}
#endif

#endif
