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
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "nvs.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "charge.h"
#include "gb.h"
#include "gbpick.h"
#include "nes.h"
#include "pm.h"
#include "tama.h"
#include "haptic.h"
#include "power.h"
#include "pwrkey.h"
#include "slotb.h"
#include "statusbar.h"
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

/* THE APP MENU WAS DRAWN. Not part of the linear trail -- it can repeat, and it
 * is written every time the menu appears. It answers the one question the trail
 * could not: when slot B is found sitting on the app menu, did the boot arrive
 * there (no launch key) or did something inside the loop fall back to it? */
#define SLOTB_STAGE_MENU       10

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

/*
 * WHAT THE PICKER CHOSE, AND WHAT THE EMULATOR FAILED TO OPEN.
 *
 * A cartridge that will not open is the one failure whose message on the glass
 * cannot carry enough: the reason is short but the PATH is what decides the
 * answer, and a path is 40 to 130 characters against a 38-character row. The
 * wearer can read "cannot open the cartridge" and still not know which of 1,143
 * it meant, and a picture kills the mirror so nothing else can be asked.
 *
 * So it goes to flash, in the sector the old boot mark used to occupy, and a
 * four-second `esptool read_flash` answers it exactly. Written once per launch
 * (the chosen path) and once per failure (the path and the reason).
 */
#define SLOTB_REC_ADDR  0x1B000u
#define SLOTB_REC_MAGIC 0x53425243u   /* "SBRC" */

typedef struct
{
    uint32_t magic;
    uint32_t uptime_ms;
    char     why[48];
    char     path[256];
} slotb_rec_t;

void slotb_record(const char *why, const char *path)
{
    slotb_rec_t r;

    memset(&r, 0, sizeof(r));
    r.magic = SLOTB_REC_MAGIC;
    r.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    snprintf(r.why, sizeof(r.why), "%s", why ? why : "");
    snprintf(r.path, sizeof(r.path), "%s", path ? path : "");

    if (esp_flash_erase_region(esp_flash_default_chip, SLOTB_REC_ADDR, 0x1000) != ESP_OK)
        return;
    esp_flash_write(esp_flash_default_chip, &r, SLOTB_REC_ADDR, sizeof(r));
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

/*
 * Hand the boot back to the desktop: forget the slot switch and restart.
 *
 * WHY ERASE RATHER THAN esp_ota_set_boot_partition(ota_0).
 *
 * With rollback protection enabled, setting the boot partition writes a NEW
 * otadata entry for ota_0, and the bootloader turns that into PENDING_VERIFY on
 * the next boot. The app then confirms itself and the entry becomes VALID.
 *
 * THAT CONFIRM WORKS, and an earlier version of this comment said it did not --
 * that was a misread, not a defect. The reader had the state enum shifted by one
 * (ESP_OTA_IMG_VALID is 0x2, not PENDING_VERIFY), so a confirmed desktop was
 * printed as pending. Measured correctly on 2026-11-23: a desktop left running
 * 45 s reads back VALID.
 *
 * The real behaviour is rollback protection doing its job, and it is what the
 * operator hit on 2026-11-23: exit the emulator with Pusher A, reset within the
 * 30-second trial, and the bootloader sees PENDING_VERIFY on the second boot,
 * marks it ABORTED, and falls back to the OTHER valid image -- which after a game
 * session is slot B. Landing in the emulator when you meant to be at the desktop
 * is correct firmware behaviour and bad user experience.
 *
 * Erasing otadata removes the alternative rather than the trial. The bootloader
 * then records the first app partition itself, and with the other entry UNDEFINED
 * there is no second valid image for a rollback to choose. Measured: after an
 * erase, a reset at t=12 s -- inside the trial -- still came up on the desktop and
 * the entry read VALID afterwards. That is the property this exit needs, because
 * a wearer putting the watch down after a game is exactly the person who power
 * cycles within 30 s.
 *
 * The cost is that `ota status` reports no sequence number after an exit. That is
 * a report, not a behaviour, and a boot that cannot land in the wrong personality
 * wins.
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

/*
 * THE APPS SLOT B CAN RUN.
 *
 * Slot B was written around one emulator. This table is the whole of what makes
 * it a "performance slot" with a menu: an entry per app, and the run loop below
 * needs nothing else. Adding a fourth app is a row here plus its own touch-hook
 * split (nes_install_touch_hook() and pm_install_touch_hook() exist for exactly
 * that reason -- see gb_install_touch_hook()).
 *
 * WHY THIS AND NOT AN IMAGE PER APP. The three emulators are already in this
 * binary and share the display, touch, audio and card drivers. A separate 3 MB
 * image per app would store the same code four times to change a core assignment,
 * and every launch would rewrite a flash slot that has to survive power loss.
 * The desktop, Python, Bluetooth, TLS and TCP/IP are most of the image and slot B
 * never calls any of them, so the dead weight is flash rather than CPU -- and
 * there are 900-odd KB free in the slot.
 *
 * Each app's touch consumer is installed ONCE, below, because they are additive
 * and each declines unless it is the one running.
 */
typedef struct
{
    const char *label;
    const char *key;                     /* the name `slotb play <key>` takes */
    bool (*start)(const char *rom);      /* NULL rom means the app's default */
    void (*stop)(void);
    bool (*running)(void);
    const char *(*why)(void);            /* why the last start refused, or NULL */
} slotb_app_t;

/*
 * Game Boy is the one app with a cartridge chooser, so its start is a wrapper:
 * pick a cartridge, then start. Abandoning the picker returns false with the exit
 * flag set, and where that leads is slotb_app_go()'s decision -- desktop or menu
 * -- because it depends on how slot B was entered, not on how the app ended.
 */
/* Big enough for a full card path. A bucket plus a 110-character cartridge
 * name is over 130 bytes, and the default cartridge's own path is 120, so
 * the 96 that used to be here could not name the cartridge the firmware
 * itself prefers. */
static char s_gb_rom[256];

static bool slotb_start_gb(const char *rom)
{
    (void)rom;

    /* THE BLITTER STAYS ON THE GUEST'S CORE; gb_set_blit_core() carries the
     * measurement that decided it. */
    gb_set_blit_core(1);

    if (!gbpick_run(s_gb_rom, sizeof(s_gb_rom), &s_exit_req))
        return false;

    /* WHICH CARTRIDGE THE PICKER HANDED OVER, recorded before the emulator gets
     * a chance to refuse it. The pair of records -- this and the failure below
     * -- says whether a "not found" was a bad path or a bad file. */
    slotb_record("pick", s_gb_rom);

    const bool ok = gb_start(s_gb_rom);

    if (!ok)
        slotb_record(gb_last_error(), gb_last_path());

    return ok;
}

/*
 * POKEMON MINI GETS THE SAME CHOOSER, through the same navigator with its own
 * table: one root, `.min` files. There is no second picker here -- pk_pm is a
 * config, not a copy -- so the paging, the margin handling and the Up bar cannot
 * drift from Game Boy's behaviour.
 *
 * THE PATH BUFFER IS THE SAME SIZE FOR THE SAME REASON as Game Boy's: a bucket
 * directory plus a long cartridge name. Pokemon Mini's retail set is small and
 * flat today, but the card is the user's to organise and a 96-byte buffer is
 * exactly the bug the Game Boy picker already paid for.
 */
static char s_pm_rom[256];

static bool slotb_start_pm(const char *rom)
{
    (void)rom;

    if (!gbpick_run_cfg(&pk_pm, s_pm_rom, sizeof(s_pm_rom), &s_exit_req))
        return false;

    /* THE SAME RECORD GAME BOY MAKES, and for the same reason: it names the path
     * the picker handed over, so a later "did not start" can be told apart from a
     * picker that built the wrong path. */
    slotb_record("pick", s_pm_rom);

    return pmini_start(s_pm_rom);
}

/* THE PATH BUFFER IS 256 FOR THE REASON Game Boy's and Pokemon Mini's are: a
 * bucket directory plus a long ROM name. The Tamagotchi's own default is a short
 * literal (`/sd/media/tama/rom.bin`), which is exactly the case that HIDES the
 * problem -- the picker hands over whatever the card holds, and tama.c's internal
 * copy of this path was 64 bytes until this cycle. */
static char s_tama_rom[256];

static bool slotb_start_tama(const char *rom)
{
    (void)rom;

    /* THE SAME CONFIGURED NAVIGATOR, not a copy of it. The Tamagotchi has one
     * image on the card today and the tree is offered anyway, so a second one
     * shows up in the picker without touching this function. */
    if (!gbpick_run_cfg(&pk_tama, s_tama_rom, sizeof(s_tama_rom), &s_exit_req))
        return false;

    slotb_record("pick", s_tama_rom);

    return tama_start(s_tama_rom);
}

/*
 * THE APPS THE MENU OFFERS, AND THE REASON THERE ARE THREE.
 *
 * The NES WAS HERE AND IS NOT, because it will not ship: 320x240 console output on
 * a watch is not something a wearer can see, and a menu that offers a console
 * nobody should use is a promise this port should not make. NOTHING WAS DELETED --
 * `nes.c` is untouched and `nes` still runs from a Terminal in the desktop, which is
 * where it belongs while it is on the backburner. What went is the menu row.
 *
 * REMOVING A ROW HERE REMOVES IT FROM TWO PLACES AT ONCE, deliberately: the menu
 * draws this table directly, and `slotb play <key>` resolves through it, so an app
 * cannot be launchable-but-invisible or visible-but-unlaunchable. A stored NVS key
 * that no longer names a row falls back to the menu rather than failing, which is
 * the path a wearer who rebooted mid-update would take.
 */
static const slotb_app_t s_apps[] = {
    { "Game Boy Color", "gb",  slotb_start_gb,   gb_stop,     gb_running,     gb_last_error },
    { "Pokemon Mini",   "pm",  slotb_start_pm,   pmini_stop,  pmini_running,  NULL },
    { "Tamagotchi V3",  "tama", slotb_start_tama, tama_stop,   tama_running,   NULL },
};

#define SLOTB_APP_N ((int)(sizeof(s_apps) / sizeof(s_apps[0])))


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
/* ------------------------------------------------------- the app handoff */

/*
 * WHICH APP TO START, CARRIED ACROSS THE REBOOT IN NVS.
 *
 * The desktop's Start menu is the natural place to launch an app, and a shortcut
 * that says "Game Boy" should boot straight into the Game Boy -- not into slot B
 * and then a menu asking which app, which is two choices where the wearer made
 * one. The choice has to survive a reboot, so it goes in NVS: `nvs_flash_init()`
 * has already run by the time board_startup_override() is called, and the value
 * is a dozen bytes in a wear-levelled partition.
 *
 * `slotb start` with no app CLEARS the key, so entering slot B that way always
 * gives the menu. Otherwise a stale key from an earlier launch would run that app
 * again every time slot B was entered, including after a fresh boot.
 *
 * A key that no longer names an app -- an app removed from the table, or
 * corrupted -- falls back to the menu rather than failing, because a menu is
 * always a usable answer and a refused boot is not.
 */
#define SLOTB_NVS_NS   "slotb"
#define SLOTB_NVS_KEY  "app"

static void slotb_app_set(const char *key)
{
    nvs_handle_t h;

    if (nvs_open(SLOTB_NVS_NS, NVS_READWRITE, &h) != ESP_OK)
    {
        ESP_LOGW(TAG, "nvs: open failed; the next boot will show the menu");
        return;
    }

    if (key && *key)
        nvs_set_str(h, SLOTB_NVS_KEY, key);
    else
        nvs_erase_key(h, SLOTB_NVS_KEY);

    nvs_commit(h);
    nvs_close(h);
}

/* Returns the stored key, or an empty string. */
static void slotb_app_get(char *out, size_t cap)
{
    nvs_handle_t h;
    size_t len = cap;

    out[0] = '\0';

    if (nvs_open(SLOTB_NVS_NS, NVS_READONLY, &h) != ESP_OK)
        return;

    if (nvs_get_str(h, SLOTB_NVS_KEY, out, &len) != ESP_OK)
        out[0] = '\0';

    nvs_close(h);
}

static int slotb_app_index(const char *key)
{
    if (!key || !*key)
        return -1;

    for (int i = 0; i < SLOTB_APP_N; i++)
        if (!strcmp(s_apps[i].key, key))
            return i;

    return -1;
}

/*
 * Run one app from start to stop. Returns true when slot B should leave for the
 * desktop.
 *
 * `direct` says how slot B was entered, and it is the ONLY thing that decides
 * where every exit goes: from a desktop shortcut, every way out is the desktop,
 * because that is where the wearer came from. From the menu, every way out but
 * the deliberate one is the menu again, so another game is a tap away.
 *
 * It is a parameter rather than a flag the caller resets, because the previous
 * version carried a `launched_direct` local inside the loop and a path that lost
 * it -- which is how a wearer who tapped "Game Boy Color" and picked a cartridge
 * ended up looking at the app menu.
 */
static bool slotb_app_go(int sel, bool direct)
{
    const slotb_app_t *app = &s_apps[sel];

    if (!app->start(NULL))
    {
        /*
         * BACKING OUT IS NOT A FAILURE, and the two arrive through the same
         * return value: Game Boy's start is the cartridge picker, so Pusher A in
         * it returns false with the exit flag set. Reporting that as "did not
         * start" would be a lie printed to the wearer's face.
         */
        if (s_exit_req)
        {
            s_exit_req = false;
            return direct;
        }

        /*
         * SAY WHY, IN THE APP'S OWN WORDS WHERE IT HAS THEM. An app that cannot
         * start has only its own printf, and there is no console here. "no ROM?"
         * was a guess, and a bad cartridge, a full heap and a leaked blitter all
         * produced it.
         */
        const char *why = app->why ? app->why() : NULL;
        const bool have_why = why && *why;
        char l1[48];

        snprintf(l1, sizeof(l1), "%s did not start", app->label);
        gbpick_message(l1, have_why ? why : "no ROM? put one in /sd/media");
        ESP_LOGW(TAG, "%s did not start%s%s", app->label,
                 have_why ? ": " : "", have_why ? why : "");
        vTaskDelay(pdMS_TO_TICKS(SLOTB_ERR_HOLD_MS));

        return direct;
    }

    ESP_LOGI(TAG, "running %s", app->label);
    slotb_mark(SLOTB_STAGE_EMU_START);

    /* The firmware is demonstrably working, so take it off the bootloader's
     * trial list. */
    slotb_confirm();

    while (app->running() && !s_exit_req)
        vTaskDelay(pdMS_TO_TICKS(100));

    /* THE STOP HAPPENS HERE, ON THIS TASK, NOT ON THE KEY TASK. An app's stop
     * writes SRAM before it releases anything and then waits for its own tasks to
     * leave; doing it here means the save has finished before the next app claims
     * the amplifier or the glass. */
    app->stop();
    slotb_mark(SLOTB_STAGE_EMU_STOP);

    s_exit_req = false;
    return direct;
}

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

    /*
     * THE BOARD CONFIGURATION IS LOADED HERE, BEFORE THE PANEL, because the panel
     * settings now come from it. The SD card's pins are the reason this file
     * exists, but `lcd.boot_pattern` is read at the same moment and would be
     * useless if the load happened after display_init() as it used to.
     *
     * NULL file path: the device copy at /etc/board.conf is on the littlefs the
     * shell would have mounted, and BRD-004 confirms the built-in pin set is the
     * correct one for this board (the card is on SPI3, host value 2).
     */
    if (tdsh_board_load(board_conf, NULL) != 0)
        ESP_LOGW(TAG, "board.conf had a bad line; using what parsed");
    slotb_mark(SLOTB_STAGE_BOARDCFG);

    /*
     * THE BOOT BANDS FOLLOW THE SAME SETTING THE DESKTOP USES, so a panel fault
     * is diagnosed identically on both personalities. The default is OFF --
     * display_init() pushes the bands and then blacks the panel a moment later,
     * which is a colour flash at every boot -- and `lcd.boot_pattern = 1` in
     * board.conf turns them back on without a rebuild.
     */
    display_boot_pattern(tdsh_board_int("lcd.boot_pattern", 0) != 0);

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

    /*
     * THE TOP STRIP, WHICH THIS PERSONALITY HAS NEVER HAD.
     *
     * `statusbar_start()` lives in `link_init()`, and slot B NEVER REACHES IT:
     * `board_startup_override()` runs in app_main long before `hal_start()` calls
     * link_init(), and this path does not return. So slot B has always run with no
     * strip at all -- and, because `display_init()` above blacks the whole panel
     * including rows 0..43, with those rows simply dark.
     *
     * That is invisible until an app wants to say something there, which is what
     * the Tamagotchi's up-time label does. Started HERE and not earlier because
     * display_init()'s panel_fill() would wipe it: the same ordering rule link_init()
     * documents.
     *
     * NO CLOCK, so the strip is told so: `fill_sysinfo()` -- the source of
     * `td_sysinfo()->time_now` -- runs in app_main after this override, and this
     * path never returns to it. Without that flag the strip would paint `--/--`
     * placeholders here for ever, which reads as damage rather than as absence.
     */
    statusbar_clock_present(false);

    if (!statusbar_start())
        ESP_LOGW(TAG, "status strip did not start");

    /* Touch, haptics and Pusher A. Without this the emulator is a picture with
     * no way to play it -- measured, not assumed: see the note on
     * slotb_peripherals(). */
    slotb_peripherals();

    /* The SD pins live in board.conf, and the shell that normally loads it does
     * not exist here. main.c passes the built-in text down for exactly this.
     * NULL file path: the device copy at /etc/board.conf is on the littlefs the
     * shell would have mounted, and BRD-004 confirms the built-in pin set is the
     * correct one for this board (the card is on SPI3, host value 2). */
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
    /* EVERY APP'S TOUCH CONSUMER, INSTALLED ONCE. They are additive and each
     * declines unless it is the one running, which is what lets the menu hand
     * the glass between apps without touching the drivers again. */
    gb_install_touch_hook();
    nes_install_touch_hook();
    pm_install_touch_hook();
    tama_install_touch_hook();

    /* Pusher A, armed before the menu so the press can leave slot B from the
     * menu itself. */
    if (!pwrkey_init(slotb_sink))
        ESP_LOGW(TAG, "Pusher A did not arm; the only way out is a power cycle");
    pwrkey_set_app_key(slotb_key, NULL);

    /*
     * THE MENU IS ONE WAY INTO AN APP, NOT THE ONLY ONE, and this loop is why it
     * is worth having when it IS the way in: back out of a game and the menu
     * returns instead of the watch rebooting to the desktop, so changing games
     * costs a tap and not a boot.
     *
     * PUSHER A MEANS "BACK" -- in the menu it leaves slot B for the desktop, and
     * inside an app it stops the app and goes back one step: to the menu when the
     * menu is where the wearer came from, and to the desktop when a shortcut is.
     */
    {
        static const char *labels[SLOTB_APP_N];

        for (int i = 0; i < SLOTB_APP_N; i++)
            labels[i] = s_apps[i].label;

        /* A SHORTCUT ON THE DESKTOP ARRIVES HERE AS A NAME, so the wearer who
         * tapped "Game Boy" gets the Game Boy and not a menu asking which app.
         * Cleared as it is read: it is a one-shot instruction, and leaving it set
         * would make every later entry into slot B start that app. */
        char want[16];

        slotb_app_get(want, sizeof(want));
        const int direct = slotb_app_index(want);

        if (direct >= 0)
        {
            slotb_app_set(NULL);
            ESP_LOGI(TAG, "booting straight into %s", s_apps[direct].label);

            /*
             * NO MENU IN THIS PATH, AT ALL -- and the reason has to be structural
             * rather than a promise.
             *
             * The shortcut said "Game Boy Color". A menu listing three apps was
             * never part of that instruction, and it used to be reachable from
             * here: a cartridge that would not load -- a bad dump, a full heap, a
             * leaked blitter -- fell through to the menu, so a wearer who had
             * tapped a game and picked a cartridge could end up looking at a
             * choice they had already made. The menu is not drawn on this path and
             * there is nothing here that can draw it.
             */
            slotb_app_go(direct, true);
        }
        else
        {
            for (;;)
            {
                int sel = -1;

                display_pixels_claim(true);
                slotb_mark(SLOTB_STAGE_MENU);

                if (!gbpick_list_run("Slot B - select an app", labels,
                                     SLOTB_APP_N, false, -1, &sel, NULL,
                                     &s_exit_req))
                    break;                   /* Pusher A in the menu: leave */

                if (sel < 0 || sel >= SLOTB_APP_N)
                    continue;

                if (slotb_app_go(sel, false))
                    break;
            }
        }
    }

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

/* ------------------------------------------------- slot B from the start menu */

/*
 * BOOT SLOT B WITHOUT SHOWING THE MACHINERY.
 *
 * The Start menu entry for a slot B app used to be a `.tdlink` naming a Python
 * script. The desktop opens a script by running it in a TERMINAL, so the wearer
 * saw a terminal window, the echoed command, a two-line script and the line
 * "rebooting into ota_1" before the reboot -- for a menu item whose only visible
 * outcome is a game.
 *
 * Two things are wrong with that beyond the cosmetics. `td_script_run()` is the
 * only path a `.py` target has, and it is a terminal by construction. And the
 * launcher lived on the card's littlefs, so a full-image flash or a factory
 * restore deleted it -- the handover notes had to carry the three `echo` lines
 * that recreate them.
 *
 * A native app has neither problem: it is in the image, and it draws nothing.
 *
 * EVERYTHING THAT CAN FAIL HAPPENS BEFORE THE PANEL IS BLANKED. Writing the NVS
 * key and setting the boot partition are the fallible steps, and doing them
 * first means a failure leaves a desktop the wearer can still use, with a message
 * from the caller, rather than a black screen and nothing to press.
 */
bool slotb_launch(const char *key)
{
    const esp_partition_t *sb = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);

    esp_app_desc_t d;

    if (!sb || esp_ota_get_partition_description(sb, &d) != ESP_OK)
        return false;                   /* slot B is empty: nothing to boot */

    if (key && *key && slotb_app_index(key) < 0)
        return false;                   /* not an app this table knows */

    slotb_app_set(key);

    if (esp_ota_set_boot_partition(sb) != ESP_OK)
    {
        /* Leave the key cleared so a later entry into slot B shows the menu
         * rather than an app nobody asked for. */
        slotb_app_set(NULL);
        return false;
    }

    /*
     * NOW BLANK IT. Claiming the pixels stops the desktop repainting over the
     * black, and `display_fill()` pushes synchronously, so the panel is already
     * black when the restart happens. The short delay is for the last band to
     * leave the SPI engine -- the panel is write-only and cannot be asked.
     */
    display_pixels_claim(true);
    display_fill(0x0000);

    vTaskDelay(pdMS_TO_TICKS(150));
    esp_restart();
    return true;                        /* not reached */
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

    /* `play <key>`: boot slot B straight into one app. `start`: slot B's menu. */
    const char *play = NULL;

    if (sub && !strcmp(sub, "play"))
    {
        play = argc >= 3 ? argv[2] : NULL;

        if (slotb_app_index(play) < 0)
        {
            printf("slotb play <app>   apps:");
            for (int i = 0; i < SLOTB_APP_N; i++)
                printf(" %s", s_apps[i].key);
            printf("\n");
            return 1;
        }
    }
    else if (sub && strcmp(sub, "start") != 0)
    {
        printf("slotb [start | play <app> | status]\n");
        printf("  start       reboot into slot B's app menu\n");
        printf("  play <app>  reboot into one app:");
        for (int i = 0; i < SLOTB_APP_N; i++)
            printf(" %s", s_apps[i].key);
        printf("\n");
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

    /* The key is written BEFORE the boot partition, so a switch that succeeds
     * but whose write failed is still a coherent state -- it lands on the menu. */
    slotb_app_set(play);

    if (play)
    {
        int idx = slotb_app_index(play);
        printf("slotb: rebooting into %s, running %s\n", sb->label, s_apps[idx].label);
    }
    else
    {
        printf("slotb: rebooting into %s (%s)\n", sb->label, d.version);
    }
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
