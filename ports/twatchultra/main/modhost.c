/*
 * modhost.c - the firmware side of the module API table.
 *
 * See modhost.h for why a table exists and how a blob finds it. The short
 * version: the blob executes from read-only flash and can hold no pointer, so
 * the firmware publishes ONE pointer in its own RAM and the blob's build learns
 * that pointer's address from this app's ELF.
 *
 * `g_modhost_ptr` must stay a global symbol: modules/Makefile reads its address
 * with `nm` and passes it to the blob compiler. Renaming it, or making it
 * static, breaks every module build with a clear "symbol not found" rather than
 * a wrong call.
 */
#include "modhost.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "modhost";

static void *h_malloc(size_t n) { return malloc(n); }
static void h_free(void *p) { free(p); }

static void *h_malloc_prefer_psram(size_t n)
{
    return heap_caps_malloc_prefer(n, 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

static void *h_memcpy(void *d, const void *s, size_t n) { return memcpy(d, s, n); }
static void *h_memset(void *d, int c, size_t n) { return memset(d, c, n); }

static void h_log(int level, const char *msg)
{
    /* Modules get one call, not printf: a blob cannot format. */
    if (!msg)
        return;
    if (level <= 0)
        ESP_LOGE(TAG, "%s", msg);
    else if (level == 1)
        ESP_LOGW(TAG, "%s", msg);
    else
        ESP_LOGI(TAG, "%s", msg);
}

static const modhost_t s_table = {
    .abi_version = MODHOST_ABI_VERSION,
    .malloc = h_malloc,
    .free = h_free,
    .malloc_prefer_psram = h_malloc_prefer_psram,
    .memcpy = h_memcpy,
    .memset = h_memset,
    .log = h_log,
    .reserved = { 0 },
};

/*
 * PUBLISHED, AND ITS ADDRESS IS PART OF THE BUILD CONTRACT.
 *
 * `volatile` and non-static on purpose: the address is read out of the ELF by
 * the module build, and a compiler that folded it away would make every blob
 * link against a symbol that no longer exists.
 */
/* __attribute__((used)) matters: nothing in the firmware references this
 * symbol, because its VALUE is never read -- only its ADDRESS, by the blob
 * build. Without it the linker garbage-collects the symbol and every module
 * build fails to find it. */
__attribute__((used))
const modhost_t *const volatile g_modhost_ptr = &s_table;

const modhost_t *modhost_table(void)
{
    /* Reading the published pointer here is what keeps the symbol alive under
     * --gc-sections: its ADDRESS is the modules' contract, but a section that
     * nothing references is still collected. This is the one reference. */
    return g_modhost_ptr;
}
