/*
 * slotb.c - a bare Game Boy firmware, chosen by which OTA slot booted.
 *
 * The header explains the architecture. This file is the boot path itself.
 *
 * THIS FIRST INCREMENT DOES NOT RUN THE EMULATOR YET, AND THAT IS DELIBERATE.
 * The emulator is a lot of new code sitting on top of a switching mechanism
 * nobody has proved on this board. So the mechanism is proved first, on its
 * own, because a fault here is indistinguishable from a fault in the emulator
 * and would cost a session to separate. What this runs:
 *
 *   - takes over the boot when, and only when, the running slot is ota_1,
 *   - brings up the panel through the port's own display_init() -- the same
 *     driver the desktop uses, not a copy,
 *   - claims the framebuffer exactly as the emulators do, so the desktop
 *     painter (which display_init starts) idles instead of drawing,
 *   - pushes a picture through display_panel_push(), which is the exact call
 *     the emulator's blitter will make from core 0,
 *   - holds it long enough to be unmistakable on the glass, then
 *   - sets ota_0 as the boot partition and restarts, returning to the desktop.
 *
 * If the panel shows the pattern and the desktop comes back, every part of the
 * switch works: boot from ota_1, the display driver, pixel ownership, the
 * picture path, and the return. The emulator then replaces the hold-and-return
 * without touching any of it.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_heap_caps.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "slotb.h"

static const char *TAG = "slotb";

/*
 * A STAGE MARKER IN THE UNUSED FLASH PADDING, BECAUSE SLOT B HAS NO OTHER RECORD.
 *
 * The console is compiled out (CONFIG_ESP_CONSOLE_NONE), a picture owns the
 * panel whenever slot B is drawing so the USB mirror is dead (DSP-003), and a
 * hard hang writes no coredump -- measured on the first run of this module,
 * where the coredump partition read back empty while the desktop returned.
 * The card is not mounted here either, because the shell that mounts it does
 * not exist in slot B.
 *
 * So the marker goes to flash at 0x1A000: the 24 KB of deliberate padding that
 * partitions.csv keeps between ota_0's 64 KB alignment and the start of the
 * app, and that nothing reads. One sector, one struct, rewritten at each stage.
 * A host reads it with a four-second `esptool read_flash`, which is the only
 * instrument that survives a reset.
 */
#define SLOTB_MARK_ADDR  0x1A000u   /* slot B's stage trail */
#define SLOTB_BOOT_ADDR  0x1B000u   /* which slot this boot came from */
#define SLOTB_MARK_MAGIC 0x53424C53u   /* "SLBS" */

typedef struct
{
    uint32_t magic;
    uint32_t stage;
    uint32_t uptime_ms;
} slotb_mark_t;

void board_power_rails_preinit(void);   /* defined in the port's power.c */

/*
 * FORCE THIS OBJECT OUT OF libmain.a, OR NONE OF IT RUNS.
 *
 * `board_startup_override()` in main.c is WEAK, so that every other board in
 * this tree links unchanged. But a weak definition that already exists in an
 * object the linker has included (main.o, which app_main lives in) SATISFIES
 * the only reference -- so the linker never looks in the archive for a strong
 * one, and this file is never pulled in at all. The result is silent: the weak
 * no-op returns false, app_main carries on, and the desktop boots from the
 * other slot. That is exactly what the first build of this module did, and it
 * was caught by `nm` showing board_startup_override as `W` rather than `T`.
 *
 * CMakeLists.txt adds `-Wl,--undefined=slotb_force_link`, which puts an
 * undefined reference in the link; the linker then has to fetch the archive
 * member that defines it, and that member also carries the strong override.
 */
const int slotb_force_link = 1;

/* Stages, in order. A reading of N means stage N was reached and N+1 was not. */
#define SLOTB_STAGE_ENTER   1
#define SLOTB_STAGE_RAILS   2
#define SLOTB_STAGE_DISPLAY 3
#define SLOTB_STAGE_PAINT   4
#define SLOTB_STAGE_HOLD    5
#define SLOTB_STAGE_RETURN  6
#define SLOTB_STAGE_SETBOOT_FAILED 7
#define SLOTB_STAGE_FAILED  99

/* DIAGNOSTIC: stage 200 + the running partition's subtype, recorded on EVERY
 * boot including the desktop's, so a host read names which slot booted and
 * whether esp_ota_set_boot_partition() actually wrote otadata. Subtypes are
 * 0x00 factory, 0x10 ota_0, 0x11 ota_1. Remove once the switch is proven. */
#define SLOTB_STAGE_SLOT_BASE 200

static void slotb_mark_at(uint32_t addr, uint32_t stage)
{
    slotb_mark_t m = {SLOTB_MARK_MAGIC, stage, (uint32_t)(esp_timer_get_time() / 1000)};

    if (esp_flash_erase_region(esp_flash_default_chip, addr, 0x1000) != ESP_OK)
        return;
    esp_flash_write(esp_flash_default_chip, &m, addr, sizeof(m));
}

/* Slot B's own trail, in its own sector so nothing that boots later can erase it. */
static void slotb_mark(uint32_t stage) { slotb_mark_at(SLOTB_MARK_ADDR, stage); }

/*
 * WHICH SLOT THIS BOOT CAME FROM, IN A SECOND SECTOR.
 *
 * These were one sector at first, and that was wrong in a way worth recording:
 * whichever OS boots AFTER slot B writes the sector too, so the desktop's own
 * boot mark overwrote slot B's trail and the evidence of how far slot B got
 * was gone by the time a host could read it. Two sectors, one writer each.
 */
static void slotb_boot_mark(const esp_partition_t *run)
{
    slotb_mark_at(SLOTB_BOOT_ADDR,
                  SLOTB_STAGE_SLOT_BASE + (run ? (uint32_t)run->subtype : 0xFFu));
}

/* How long the proving picture is held before handing back. Long enough that a
 * sighted operator cannot miss it, short enough not to be a nuisance. */
#define SLOTB_HOLD_MS 4000

void slotb_confirm(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;

    if (run && esp_ota_get_state_partition(run, &st) == ESP_OK &&
        st == ESP_OTA_IMG_PENDING_VERIFY)
    {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "slot B confirmed: %s", run->label);
    }
}

void slotb_return_to_desktop(void)
{
    const esp_partition_t *desk = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);

    if (!desk)
    {
        /* Nothing to return to. Rebooting would simply boot slot B again, so
         * say so and stay put rather than reset-loop. */
        ESP_LOGE(TAG, "no ota_0 partition to return to");
        return;
    }

    ESP_LOGI(TAG, "returning the boot to %s", desk->label);
    slotb_mark(SLOTB_STAGE_RETURN);

    if (esp_ota_set_boot_partition(desk) != ESP_OK)
    {
        ESP_LOGE(TAG, "could not set the boot partition; staying in slot B");
        slotb_mark(SLOTB_STAGE_SETBOOT_FAILED);
        return;
    }

    esp_restart();
}

/*
 * Push a solid rectangle through the picture path.
 *
 * display_panel_push() is the call the emulator's blitter makes, so exercising
 * it here proves the same thing the emulator will depend on. It refuses unless
 * the framebuffer has been claimed, which is why the claim precedes it.
 */
static void push_block(int cx, int cy, int w, int h, uint16_t colour)
{
    const size_t n = (size_t)w * (size_t)h;
    uint16_t *px = heap_caps_malloc(n * sizeof(*px), MALLOC_CAP_SPIRAM);

    if (!px)
    {
        ESP_LOGW(TAG, "no PSRAM for a %dx%d block", w, h);
        return;
    }

    for (size_t i = 0; i < n; i++)
        px[i] = colour;

    display_panel_push(cx, cy, w, h, px);
    heap_caps_free(px);
}

/*
 * The proving sequence. Returns only if the panel never came up, in which case
 * there is nothing to wait for and the caller hands back to the desktop at once
 * -- a slot B with no display must not become a black watch.
 */
static void slotb_prove(void)
{
    slotb_mark(SLOTB_STAGE_ENTER);

    /* THE PANEL NEEDS ITS POWER RAIL, AND slot B RUNS BEFORE app_main REACHES
     * THE CALL THAT ENABLES IT.
     *
     * board_startup_override() is called before board_power_rails_preinit(),
     * because the override has to happen before the shell and the desktop
     * exist. That means slot B must bring the rails up itself -- on this board
     * the AMOLED sits behind a switched regulator, so display_init() against a
     * dead rail is not a slow boot, it is no boot. */
    board_power_rails_preinit();
    slotb_mark(SLOTB_STAGE_RAILS);

    /* NULL reply hook: display_init() guards it, and slot B has no desktop to
     * answer a terminal query for. */
    if (!display_init(NULL))
    {
        ESP_LOGE(TAG, "the panel did not come up; returning to the desktop");
        slotb_mark(SLOTB_STAGE_FAILED);
        return;
    }
    slotb_mark(SLOTB_STAGE_DISPLAY);

    /* Stop the painter display_init() just started. Claiming twice is
     * idempotent, and releasing it is what the emulators do on exit. */
    display_pixels_claim(true);
    slotb_mark(SLOTB_STAGE_PAINT);

    /* Green, so the whole glass is unmistakably not the desktop. */
    display_fill(0x07E0);

    /* Then a white block through the picture path, centred. If the panel is
     * green but the block is missing or misplaced, the fault is in the picture
     * transfer specifically, which is a much smaller thing to chase. */
    push_block((DISPLAY_W - 80) / 2, (DISPLAY_H - 80) / 2, 80, 80, 0xFFFF);

    vTaskDelay(pdMS_TO_TICKS(SLOTB_HOLD_MS));
    slotb_mark(SLOTB_STAGE_HOLD);

    /* Blue for a second phase: a panel stuck on the first colour reads as a
     * frozen driver, while a clean change proves the path is live. */
    display_fill(0x001F);
    vTaskDelay(pdMS_TO_TICKS(SLOTB_HOLD_MS));

    display_pixels_claim(false);
}

bool board_startup_override(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();

    /* Record which slot booted, on every boot, before deciding anything. */
    slotb_boot_mark(run);

    /* Not slot B: the desktop boots exactly as it always has. */
    if (!run || run->subtype != ESP_PARTITION_SUBTYPE_APP_OTA_1)
        return false;

    ESP_LOGI(TAG, "booting slot B from %s (%u bytes)", run->label,
             (unsigned)run->size);

    slotb_prove();

    /* The switch is proved once the picture has been held; if the panel failed
     * we are here immediately, which is the correct time to hand back. */
    slotb_return_to_desktop();

    /* Unreachable: slotb_return_to_desktop() restarts. Returning true anyway
     * keeps the contract honest if a future change makes it return. */
    return true;
}
