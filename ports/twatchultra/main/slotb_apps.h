/*
 * slotb_apps.h - the apps slot B offers, and the NVS key that remembers which
 * one was asked for. ONE DEFINITION, USED BY BOTH PERSONALITIES.
 *
 * WHY THIS FILE EXISTS AT ALL. The two OTA slots now hold DIFFERENT images: the
 * desktop in ota_0 and the emulator personality in ota_1. The desktop has to
 * know which app keys slot B accepts BEFORE it switches the boot partition,
 * because a key slot B does not have reboots the wearer into a personality that
 * answers "did not start" with no console and no log -- slot B has neither.
 *
 * That check used to be free: both slots held the same image, so
 * `slotb_app_index()` and the table it searched were the same code. With the
 * images split, a list copied into the launcher would be a second description of
 * slot B's contents, kept in step by hand, which is the drift this project has
 * paid for twice (a second cartridge picker, and a control whose label and duty
 * lived apart). So the list lives here and both builds compile it.
 *
 * WHAT IS NOT HERE: the functions. A row of `slotb_app_descs[]` is a name and a
 * label; the start/stop/running/why pointers belong to slot B and stay in
 * slotb.c, indexed by the same position. `s_apps[]` carries a _Static_assert
 * that it is the same length, so adding an app without its row here -- or the
 * reverse -- is a compile error rather than an app that cannot be launched.
 *
 * THE NVS HELPERS ARE INLINE FOR THE SAME REASON: the launcher writes the key and
 * slot B reads it, and two hand-written nvs_open/nvs_get_str pairs in two images
 * is exactly how the namespace or the key name drifts apart.
 */
#ifndef TWATCH_SLOTB_APPS_H
#define TWATCH_SLOTB_APPS_H

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "esp_err.h"
#include "nvs.h"

/*
 * Order is the menu order in slot B and the order `slotb play` lists them in.
 * The labels are what the glass and the shell print; the keys are what travels
 * through NVS and what the desktop passes to slotb_launch().
 */
typedef struct
{
    const char *key;
    const char *label;
} slotb_app_desc_t;

static const slotb_app_desc_t slotb_app_descs[] = {
    { "gb",   "Game Boy Color" },
    { "pm",   "Pokemon Mini"   },
    { "tama", "Tamagotchi V3"  },
    { "map",  "Maps"           },
};

#define SLOTB_APP_N ((int)(sizeof(slotb_app_descs) / sizeof(slotb_app_descs[0])))

/*
 * The choice has to survive a reboot, so it goes in NVS: nvs_flash_init() has
 * already run by the time either personality asks, and the value is a dozen
 * bytes in a wear-levelled partition.
 */
#define SLOTB_NVS_NS   "slotb"
#define SLOTB_NVS_KEY  "app"

/* Index of `key` in the table, or -1. Empty and NULL name nothing. */
static inline int slotb_app_index(const char *key)
{
    if (!key || !*key)
        return -1;

    for (int i = 0; i < SLOTB_APP_N; i++)
        if (!strcmp(slotb_app_descs[i].key, key))
            return i;

    return -1;
}

/* Store the key, or clear it when `key` is NULL/empty. False if NVS refused. */
static inline bool slotb_app_store(const char *key)
{
    nvs_handle_t h;

    if (nvs_open(SLOTB_NVS_NS, NVS_READWRITE, &h) != ESP_OK)
        return false;

    esp_err_t err = (key && *key) ? nvs_set_str(h, SLOTB_NVS_KEY, key)
                                  : nvs_erase_key(h, SLOTB_NVS_KEY);

    if (err == ESP_OK)
        err = nvs_commit(h);

    nvs_close(h);
    return err == ESP_OK;
}

/* Read the stored key into `out`; an empty string when there is none. */
static inline void slotb_app_load(char *out, size_t cap)
{
    nvs_handle_t h;
    size_t len = cap;

    if (!out || cap == 0)
        return;

    out[0] = '\0';

    if (nvs_open(SLOTB_NVS_NS, NVS_READONLY, &h) != ESP_OK)
        return;

    if (nvs_get_str(h, SLOTB_NVS_KEY, out, &len) != ESP_OK)
        out[0] = '\0';

    nvs_close(h);
}

#endif /* TWATCH_SLOTB_APPS_H */
