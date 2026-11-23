/*
 * slotb.c - a bare Game Boy firmware, chosen by which OTA slot booted.
 *
 * The header explains the architecture. This file is the boot path itself.
 *
 * WHAT RUNS HERE: the port's own Game Boy emulator and nothing else. No desktop,
 * no shell, no status strip, no WiFi, no SSH -- so the only tasks on either core
 * are the ones gb_start() creates. The panel, the SD card and the input hooks are
 * the port's existing drivers, called directly, not copies of them.
 *
 * THE EMULATOR IS `gb.c`, NOT A SECOND VERSION OF IT. gb.c is 2100 lines of
 * frame pool, blitter, audio, save RAM and diagnostics with several hard-won
 * defects fixed inside it, and a slot-B-local copy would be the duplication this
 * project has already paid for elsewhere. Its only contact with the shell is
 * gb_register_command(), which slot B does not call: it installs the two input
 * hooks itself and never registers a command, because there is no shell here to
 * register into.
 *
 * LEAVING IS PUSHER A. slot B installs its own key hook rather than gb.c's,
 * because gb.c's handler calls gb_stop() from the key task and then a slot B
 * that returned to the desktop would be racing the save it had just started.
 * Here the press only raises a flag; the boot task calls gb_stop() itself, so the
 * save finishes before the reboot, in one ordered sequence.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "charge.h"
#include "gb.h"
#include "haptic.h"
#include "power.h"
#include "pwrkey.h"
#include "slotb.h"
#include "tdsh_board.h"
#include "tdsh_espidf.h"
#include "touch.h"

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

/* Stages, in order. A reading of N means stage N was reached and N+1 was not.
 * The FAIL_* codes are separate so a read names WHICH step failed rather than
 * "somewhere in slot B". */
#define SLOTB_STAGE_ENTER      1
#define SLOTB_STAGE_RAILS      2
#define SLOTB_STAGE_DISPLAY    3
#define SLOTB_STAGE_BOARDCFG   4
#define SLOTB_STAGE_SD         5
#define SLOTB_STAGE_EMU_START  6
#define SLOTB_STAGE_EMU_STOP   7
#define SLOTB_STAGE_RETURN     8
#define SLOTB_STAGE_SETBOOT_FAILED 9

#define SLOTB_FAIL_DISPLAY  90
#define SLOTB_FAIL_BOARDCFG 91
#define SLOTB_FAIL_SD       92
#define SLOTB_FAIL_NO_ROM   93

/* DIAGNOSTIC: stage 200 + the running partition's subtype, recorded on EVERY
 * boot including the desktop's, so a host read names which slot booted and
 * whether esp_ota_set_boot_partition() actually wrote otadata. Subtypes are
 * 0x00 factory, 0x10 ota_0, 0x11 ota_1. Remove once the switch is proved. */
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

/* How long an error screen is held before handing the watch back. Long enough
 * to be read, short enough that a missing ROM does not look like a hang. */
#define SLOTB_ERR_HOLD_MS 4000

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
 * Where a key press would go if anything wanted it.
 *
 * touch_init() and pwrkey_init() take a callback that types into the desktop's
 * terminal, and slot B has no terminal. The on-glass d-pad reaches the emulator
 * through the app-touch hook instead (gb_app_touch), and Pusher A through the
 * app-key hook, so nothing a real user can press depends on this sink. It exists
 * because the drivers require a callback to start at all.
 */
static void slotb_sink(const char *data, int len)
{
    (void)data;
    (void)len;
}

/*
 * Bring up the four peripherals the emulator needs, in the desktop's order.
 *
 * EACH OF THESE WAS MISSING FROM THE FIRST RUN, AND THE OPERATOR FOUND ALL
 * THREE: haptics, touch and Pusher A were dead while picture, audio and sound
 * were perfect. The cause was one mistake repeated: slot B registered HOOKS into
 * drivers whose TASKS HAD NEVER BEEN STARTED. touch_add_app_touch() and
 * pwrkey_set_app_key() install a consumer into an existing poll loop; the loop
 * itself is created by touch_init() and pwrkey_init(), which only the desktop's
 * link_init() was calling. Audio escaped because sound_stream_open() calls
 * sound_init() itself.
 *
 * THE ORDER HERE IS THE DESKTOP'S AND IT MATTERS:
 *
 *   - the panel's rail and its expander switch before display_init() touches the
 *     bus, or the driver drives a dark panel;
 *   - the touch controller's reset released before touch_init(), because the
 *     expander holds it in reset until something drives the pin;
 *   - the haptic rail raised before haptic_init(), because the DRV2605 answers
 *     I2C without its supply and probes as healthy while never moving -- the
 *     failure BRD-005 records. This one had a visible cause on the first run:
 *     the XL9555 LATCH SURVIVES AN MCU RESET, so P07 was still high from the
 *     desktop's last boot and the panel lit up by luck, while P06 had never been
 *     raised by anyone and the motor was silent. On a cold boot slot B would
 *     have been dark as well as silent;
 *   - the charger's interrupts armed before pwrkey_init(), which clears the PMU
 *     status and refuses to arm on a line that is still low.
 */
static void slotb_peripherals(void)
{
    /* The expander's latches outlive a reset, so this is usually a no-op -- but
     * "usually" is not a reason to skip the step that makes a cold boot work. */
    if (!board_display_power_on())
        ESP_LOGW(TAG, "panel power did not come up");

    board_touch_reset_release();

    if (touch_init(slotb_sink))
        ESP_LOGI(TAG, "touch polling");
    else
        ESP_LOGW(TAG, "no touch controller");

    if (!board_haptic_power_on())
        ESP_LOGW(TAG, "haptic rail failed");
    else if (!haptic_init())
        ESP_LOGW(TAG, "no DRV2605");

    charge_watch_enable();
}

/*
 * PUSHER A LEAVES SLOT B.
 *
 * A flag and nothing else, because the caller has to be the task that then calls
 * gb_stop() and the restart: gb.c's own handler calls gb_stop() from the key
 * task, and a slot B that returned on top of that would reboot while the save it
 * had just started was still running. Claiming the press also stops an Enter
 * reaching anything, which is what makes the desktop's behaviour unchanged.
 */
static volatile bool s_exit_req;

static bool slotb_key(void *user)
{
    (void)user;
    s_exit_req = true;
    return true;
}

/*
 * Say what failed, on the glass, then hand the watch back.
 *
 * A slot B that failed to find a cartridge must not become a black watch that
 * only a reflash recovers: the colour and the marker both name the step, and the
 * boot returns to the desktop by itself a few seconds later.
 */
static void slotb_fail(uint32_t stage, uint16_t colour)
{
    slotb_mark(stage);
    display_fill(colour);
    vTaskDelay(pdMS_TO_TICKS(SLOTB_ERR_HOLD_MS));
}

/*
 * The whole of slot B.
 *
 * Order matters here and each step depends on the one before it: the rails
 * before the panel, the panel before the card is worth trying, the board config
 * before the card's pins are known, and the cartridges before the emulator.
 */
static void slotb_run(const char *board_conf)
{
    slotb_mark(SLOTB_STAGE_ENTER);

    /* THE PANEL AND THE CARD BOTH NEED RAILS, AND slot B RUNS BEFORE app_main
     * REACHES THE CALL THAT RAISES THEM.
     *
     * board_power_rails_preinit() is the port's own hook: it raises ALDO1 (the SD
     * card's supply), raises the IMU rail, and parks the NFC and LoRa chip selects
     * high, which BRD-004 records as the difference between a card that answers
     * and one that does not, because MISO is shared. It runs from app_main after
     * this override returns, so slot B calls it here. */
    board_power_rails_preinit();
    slotb_mark(SLOTB_STAGE_RAILS);

    /* NULL reply hook: display_init() guards it, and slot B has no desktop to
     * answer a terminal query for. */
    if (!display_init(NULL))
    {
        ESP_LOGE(TAG, "the panel did not come up");
        slotb_mark(SLOTB_FAIL_DISPLAY);
        return;
    }
    slotb_mark(SLOTB_STAGE_DISPLAY);

    /* Take the glass away from the painter display_init() just started, so the
     * emulator's frames are not drawn over by a terminal nobody is reading. */
    display_pixels_claim(true);

    /* Touch, haptics and Pusher A. Without this the emulator is a picture with
     * no way to play it -- measured, not assumed: see the note on
     * slotb_peripherals(). */
    slotb_peripherals();

    /* The SD pins live in board.conf, and the shell that normally loads it does
     * not exist here. main.c passes the built-in text down for exactly this.
     * NULL file path: the device copy at /etc/board.conf is on the littlefs the
     * shell would have mounted, and BRD-004 confirms the built-in pin set is the
     * correct one for this board (the card is on SPI3, host value 2). */
    if (tdsh_board_load(board_conf, NULL) != 0)
    {
        ESP_LOGW(TAG, "board.conf had a bad line; using what parsed");
    }
    slotb_mark(SLOTB_STAGE_BOARDCFG);

    if (tdsh_sdcard_mount() != ESP_OK)
    {
        ESP_LOGE(TAG, "the card did not mount");
        slotb_fail(SLOTB_FAIL_SD, 0xF800);        /* red */
        return;
    }
    slotb_mark(SLOTB_STAGE_SD);

    /* The emulator's own input hook, minus its shell command: gb.c installs the
     * on-glass d-pad and A/B through this. The key hook is slot B's own, below,
     * because gb.c's handler stops the emulator on the key task and slot B has to
     * save before it reboots. */
    gb_install_touch_hook();

    /* Pusher A last, once the poll task that will service it exists. */
    if (!pwrkey_init(slotb_sink))
        ESP_LOGW(TAG, "Pusher A did not arm; the only way out is a power cycle");
    pwrkey_set_app_key(slotb_key, NULL);

    /* No argument: gb_start() falls back to GB_ROM_DEFAULT, then GB_ROM_FLAT. */
    if (!gb_start(NULL))
    {
        ESP_LOGE(TAG, "the emulator did not start");
        slotb_fail(SLOTB_FAIL_NO_ROM, 0xF800);
        return;
    }
    slotb_mark(SLOTB_STAGE_EMU_START);

    /* The firmware is demonstrably working, so take it off the bootloader's
     * trial list. Until this the next reset would roll back to the desktop;
     * after it a power cycle comes back here, which is what a game console
     * should do. */
    slotb_confirm();

    while (gb_running() && !s_exit_req)
        vTaskDelay(pdMS_TO_TICKS(100));

    /* THE STOP HAPPENS HERE, ON THIS TASK, NOT ON THE KEY TASK.
     * gb_stop() writes SRAM before it releases anything and then waits for its
     * own tasks to leave; doing it here means the save has finished before the
     * restart below, in one ordered sequence. */
    gb_stop();
    slotb_mark(SLOTB_STAGE_EMU_STOP);

    display_pixels_claim(false);
}

bool board_startup_override(const char *board_conf)
{
    const esp_partition_t *run = esp_ota_get_running_partition();

    /* Record which slot booted, on every boot, before deciding anything. */
    slotb_boot_mark(run);

    /* Not slot B: the desktop boots exactly as it always has. */
    if (!run || run->subtype != ESP_PARTITION_SUBTYPE_APP_OTA_1)
        return false;

    ESP_LOGI(TAG, "booting slot B from %s (%u bytes)", run->label,
             (unsigned)run->size);

    s_exit_req = false;
    slotb_run(board_conf);

    /* Every path above ends here: the emulator was left by Pusher A, or a step
     * failed and said so on the glass. Either way the next boot is the desktop. */
    slotb_return_to_desktop();

    /* Unreachable: slotb_return_to_desktop() restarts. Returning true anyway
     * keeps the contract honest if a future change makes it return. */
    return true;
}
