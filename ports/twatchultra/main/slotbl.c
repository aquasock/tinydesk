/*
 * slotbl.c - the DESKTOP's half of the slot B system: switch the boot partition
 * and get out of the way.
 *
 * WHY THIS FILE IS SEPARATE FROM slotb.c. The two OTA slots no longer hold the
 * same image. ota_0 is the desktop and ota_1 is the emulator personality, so each
 * one compiles only the half of this subsystem it runs:
 *
 *   slotbl.c   compiled into the DESKTOP. Knows how to validate a request and
 *              hand the boot to ota_1. Contains no emulator code at all.
 *   slotb.c    compiled into SLOT B. The apps, their table, the peripheral
 *              bring-up and the way back. Never linked into the desktop.
 *
 * Before the split both lived in one file inside one image, which is why the
 * desktop carried every emulator core and its `.bss` to run a menu item that only
 * ever set a boot partition and restarted.
 *
 * WHAT CROSSES THE BOUNDARY: a string in NVS (`slotb_apps.h` owns the list and
 * the key name) and the boot partition. Nothing else. There is no call from the
 * desktop into slot B, because there cannot be -- after the switch this image is
 * not running.
 */
#include "slotb.h"

#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "display.h"
#include "slotb_apps.h"
#include "tdsh.h"
#include "tdsh_board.h"
#include "tdsh_espidf.h"

static const char *TAG = "slotbl";

/*
 * WHY THE LAST LAUNCH WAS REFUSED, in words the glass can carry.
 *
 * `slotb_launch()` answers true or false and the Start menu needs a reason: "no
 * image", "that build is not the one running" and "no such app" are three
 * different faults with three different fixes, and the menu has one line. This is
 * the same shape as gb_last_error() on the emulator side.
 */
static char s_why[96];

const char *slotb_last_error(void)
{
    return s_why[0] ? s_why : "no reason recorded";
}

static bool fail(const char *why)
{
    snprintf(s_why, sizeof(s_why), "%s", why);
    return false;
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
 *
 * THE VERSION CHECK IS NEW AND IT IS THE POINT OF THE SPLIT. While both slots
 * held one image, an empty ota_1 was the only way this could go wrong. Now ota_1
 * has its own build, and a desktop from one build can sit beside a slot B from
 * another -- after a flash that wrote one slot, or an interrupted install. The two
 * would disagree about which apps exist and where. Comparing the app descriptor's
 * version string costs one call and turns that into a message instead of a
 * reboot into a personality that cannot do what was asked. It is not proof the
 * builds match bit for bit; it is proof they were not built from different
 * sources, which is the failure that actually happens.
 */
bool slotb_launch(const char *key)
{
    s_why[0] = '\0';

    const esp_partition_t *sb = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);

    if (!sb)
        return fail("this partition table has no ota_1 slot");

    esp_app_desc_t d;

    if (esp_ota_get_partition_description(sb, &d) != ESP_OK)
        return fail("slot B holds no image");

    const esp_app_desc_t *self = esp_app_get_description();

    if (self && strncmp(d.version, self->version, sizeof(d.version)) != 0)
    {
        /* Name both, truncated to what fits: the fix is a flash of both slots. */
        snprintf(s_why, sizeof(s_why), "slot B is version %.20s, not %.20s",
                 d.version, self->version);
        return false;
    }

    if (key && *key && slotb_app_index(key) < 0)
        return fail("slot B has no app by that name");

    /* Written BEFORE the boot partition, so a switch whose key write failed is
     * still a coherent state: slot B lands on its menu rather than on an app
     * nobody asked for. */
    if (!slotb_app_store(key))
        return fail("could not record the choice in NVS");

    if (esp_ota_set_boot_partition(sb) != ESP_OK)
    {
        slotb_app_store(NULL);
        return fail("could not set the boot partition");
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
 * `slotb` - enter the emulator personality.
 *
 * This is what the Start menu's launcher runs. It is deliberately not
 * `ota rollback`: that command means "the other slot", which is true only by
 * coincidence now that the slots differ, and it marks the running image invalid
 * when that image is pending verification -- so running it from the desktop
 * within 30 s of boot would condemn the desktop it is about to leave.
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
                printf(" %s", slotb_app_descs[i].key);
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
            printf(" %s", slotb_app_descs[i].key);
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

    const esp_app_desc_t *self = esp_app_get_description();
    if (self && strncmp(d.version, self->version, sizeof(d.version)) != 0)
    {
        printf("slotb: slot B is version %s but this desktop is %s\n",
               d.version, self->version);
        printf("slotb: flash both slots from one build -- refusing to switch\n");
        return 1;
    }

    if (!slotb_app_store(play))
    {
        printf("slotb: could not record the choice in NVS\n");
        return 1;
    }

    if (play)
        printf("slotb: rebooting into %s, running %s\n", sb->label,
               slotb_app_descs[slotb_app_index(play)].label);
    else
        printf("slotb: rebooting into %s (%s)\n", sb->label, d.version);

    fflush(stdout);

    if (esp_ota_set_boot_partition(sb) != ESP_OK)
    {
        slotb_app_store(NULL);
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
    "slotb [start | play <app> | status]",
    "Slot B: the emulator personality in the other OTA slot",
    cmd_slotb,
    0,
};

void slotb_register_command(void)
{
    if (tdsh_register_command(&s_slotb_cmd) != 0)
        ESP_LOGW(TAG, "slotb command did not register");
}
