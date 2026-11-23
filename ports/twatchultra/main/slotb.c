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
#include <string.h>

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
#include "gbpick.h"
#include "haptic.h"
#include "power.h"
#include "pwrkey.h"
#include "slotb.h"
#include "tdsh.h"
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

/* DIAGNOSTIC: the boot-slot mark that used to live in a second sector is gone.
 * It is kept here only as a note so a reader who finds 0x1B000 stale on a board
 * knows what wrote it: a sector per desktop boot, removed in core-log entry 93. */

static void slotb_mark_at(uint32_t addr, uint32_t stage)
{
    slotb_mark_t m = {SLOTB_MARK_MAGIC, stage, (uint32_t)(esp_timer_get_time() / 1000)};

    if (esp_flash_erase_region(esp_flash_default_chip, addr, 0x1000) != ESP_OK)
        return;
    esp_flash_write(esp_flash_default_chip, &m, addr, sizeof(m));
}

/*
 * Slot B's own trail, in its own sector so nothing that boots later can erase it.
 *
 * THE DESKTOP'S BOOT MARK IS GONE. It used to write a sector on EVERY boot, slot
 * A's included, purely to record which slot had booted -- a flash erase-and-write
 * on every power-up of a watch, for a number that only mattered while the switch
 * was unproven. It is removed, and the picker's text on the glass replaces it
 * for the failures a person needs to see.
 *
 * The trail below survives for slot B sessions only, and its remaining use is the
 * case the glass cannot show: a hang. A stage value with no later stage after it
 * is the one thing that distinguishes "stopped here" from "never started" when a
 * picture has killed the mirror. Remove it when slot B has been through a few
 * cycles without one.
 */
static void slotb_mark(uint32_t stage) { slotb_mark_at(SLOTB_MARK_ADDR, stage); }

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

/*
 * Hand the boot back to the desktop: forget the slot switch and restart.
 *
 * WHY ERASE RATHER THAN esp_ota_set_boot_partition(ota_0).
 *
 * Setting the boot partition writes an otadata entry for ota_0 in state NEW, and
 * with CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE the bootloader turns that into
 * PENDING_VERIFY on the next boot. That is normally fine, because the app then
 * confirms itself -- and ON THIS BUILD THE DESKTOP'S CONFIRM DOES NOT FIRE.
 * Measured 2026-11-23: with the desktop confirmed drawing on the glass and left
 * running 45 s, otadata still read PENDING_VERIFY. OTA-001 records the same
 * confirm working on 2026-11-12 in the pre-repartition layout, so this is a
 * regression of unestablished cause, not a design property.
 *
 * Erasing instead produces a different and better-known end state, measured the
 * same day by the contrast between two reads: straight after `otatool
 * erase_otadata` both otadata entries read 0xffffffff, and after ONE BOOT they
 * read a single entry, seq 1, PENDING_VERIFY, naming ota_0. So the bootloader
 * itself records the app it booted when otadata is empty. Erasing therefore does
 * NOT leave "nothing to roll back from" -- an earlier version of this comment
 * said that and was wrong -- but it does leave the boot target chosen by the
 * bootloader from the partition table rather than by a sequence number written
 * on top of another slot's pending entry.
 *
 * What that buys is empirical and was confirmed by the operator across several
 * sessions: exit slot B, and the next boot is the desktop, every time. The
 * mechanism of the ONE observed failure of the previous approach (leave the
 * emulator, reset, land in the emulator again) is NOT established; what is
 * established is that it has not recurred with this one.
 *
 * The cost is that the desktop's `ota status` shows no sequence number after an
 * exit. That is a report, not a behaviour, and a reliable boot wins.
 */
void slotb_return_to_desktop(void)
{
    ESP_LOGI(TAG, "returning to the desktop");
    slotb_mark(SLOTB_STAGE_RETURN);

    const esp_partition_t *ota = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);

    if (!ota)
    {
        ESP_LOGE(TAG, "no otadata partition; staying in slot B");
        slotb_mark(SLOTB_STAGE_SETBOOT_FAILED);
        return;
    }

    if (esp_partition_erase_range(ota, 0, ota->size) != ESP_OK)
    {
        ESP_LOGE(TAG, "could not erase otadata; staying in slot B");
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
 *     failure BRD-005 records. The rail is the smaller half of it, though, and
 *     the first run showed why: haptic_init() is what selects a library and
 *     writes a wave sequence, and without it the chip sits in internal-trigger
 *     mode replaying an empty sequence, so it answers I2C and never moves. On
 *     that run the rail happened to be up already -- the XL9555's CONFIG and
 *     LATCH registers survive an MCU reset, so a warm reboot leaves the pins
 *     driven at their previous levels -- and the motor was still silent. On a
 *     COLD boot every expander pin is an input (power.c says so at
 *     board_touch_reset_release), so the panel would have been dark too; the
 *     panel working was therefore luck as well as the rail being set.
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
        gbpick_message("SD card did not mount", "check the card is seated");
        slotb_fail(SLOTB_FAIL_SD, 0xF800);        /* red */
        return;
    }
    slotb_mark(SLOTB_STAGE_SD);

    /* The emulator's own input hook, minus its shell command: gb.c installs the
     * on-glass d-pad and A/B through this. The key hook is slot B's own, below,
     * because gb.c's handler stops the emulator on the key task and slot B has to
     * save before it reboots. */
    gb_install_touch_hook();

    /* Pusher A last, once the poll task that will service it exists. Armed
     * BEFORE the picker so the press can also abandon the picker. */
    if (!pwrkey_init(slotb_sink))
        ESP_LOGW(TAG, "Pusher A did not arm; the only way out is a power cycle");
    pwrkey_set_app_key(slotb_key, NULL);

    /* No argument: the operator chose a cartridge on the glass. Falls back to
     * gb_start()'s own default only if the picker could not run at all. */
    {
        static char s_rom[96];

        if (!gbpick_run(s_rom, sizeof(s_rom), &s_exit_req))
        {
            ESP_LOGE(TAG, "no cartridge chosen");
            slotb_mark(SLOTB_FAIL_NO_ROM);
            return;
        }

        ESP_LOGI(TAG, "playing %s", s_rom);

        if (!gb_start(s_rom))
        {
            ESP_LOGE(TAG, "the emulator did not start");
            gbpick_message("The emulator did not start", s_rom);
            slotb_fail(SLOTB_FAIL_NO_ROM, 0xF800);
            return;
        }
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

/* ------------------------------------------------------------ shell command */

/*
 * `slotb` - enter the emulator-only personality.
 *
 * This is what the Start menu's launcher runs. It is deliberately not
 * `ota rollback`: that command means "the other slot", which is true only by
 * coincidence, and it marks the running image invalid when that image is
 * pending verification -- so running it from the desktop within 30 s of boot
 * would condemn the desktop it is about to leave.
 *
 * The check that matters is that ota_1 actually HOLDS an image. Switching to an
 * empty slot would reboot into nothing, which from the outside is a brick.
 */
static int cmd_slotb(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_partition_t *sb = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);

    const char *sub = argc >= 2 ? argv[1] : NULL;

    if (sub && !strcmp(sub, "status"))
    {
        esp_app_desc_t d;

        printf("running: %s\n", run ? run->label : "unknown");
        if (sb && esp_ota_get_partition_description(sb, &d) == ESP_OK)
            printf("slot B:  %s, version %s\n", sb->label, d.version);
        else
            printf("slot B:  empty -- no image to boot\n");
        return 0;
    }

    if (sub && strcmp(sub, "start") != 0)
    {
        printf("slotb [start | status]\n");
        printf("  start   reboot into slot B: the Game Boy with no desktop\n");
        return 1;
    }

    if (!sb)
    {
        printf("slotb: no ota_1 partition in this table\n");
        return 1;
    }

    esp_app_desc_t d;
    if (esp_ota_get_partition_description(sb, &d) != ESP_OK)
    {
        printf("slotb: slot B is empty -- flash an image to %s first\n", sb->label);
        return 1;
    }

    if (run && run->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)
    {
        printf("slotb: already running from slot B\n");
        return 0;
    }

    printf("slotb: rebooting into %s (%s)\n", sb->label, d.version);
    fflush(stdout);

    if (esp_ota_set_boot_partition(sb) != ESP_OK)
    {
        printf("slotb: could not set the boot partition\n");
        return 1;
    }

    /* Long enough for the line above to reach the terminal, which is the only
     * report the operator gets -- the screen is about to become a reboot. */
    vTaskDelay(pdMS_TO_TICKS(400));
    esp_restart();
    return 0;
}

static const tdsh_command_t s_slotb_cmd = {
    "slotb",
    "slotb [start | status]",
    "Slot B: the bare Game Boy personality",
    cmd_slotb,
    0,
};

void slotb_register_command(void)
{
    if (tdsh_register_command(&s_slotb_cmd) != 0)
        ESP_LOGW(TAG, "slotb command did not register");
}
