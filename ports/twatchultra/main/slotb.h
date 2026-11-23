/*
 * slotb.h - the watch's second boot personality.
 *
 * The same app image is flashed into BOTH OTA slots. Which slot the bootloader
 * picked decides which personality runs:
 *
 *   ota_0   the desktop. Unchanged: this module returns false and app_main
 *           carries on exactly as it always has.
 *   ota_1   slot B. A bare Game Boy firmware -- the port's own emulator and the
 *           display, touch, audio and card drivers it needs, and nothing else:
 *           no desktop, no shell, no status strip, no WiFi and no SSH. Both
 *           cores carry only what the emulator creates.
 *
 * WHY ONE IMAGE AND TWO SLOTS, RATHER THAN A SECOND FIRMWARE.
 *
 * The alternative is a separate build, forked from this one, linked against its
 * own copy of the display and touch drivers. That copy is the cost: display.c
 * holds this project's hardest-won panel defects, and a second copy would have
 * to be kept in step with the first by hand -- the fault `scripts/tamacore-sync.sh`
 * exists only to police. It also needs a second sdkconfig, and TOL-008 records
 * what a regenerated sdkconfig silently does to this board.
 *
 * A second BOOT PATH in one binary costs neither. The code that is not called
 * costs flash and nothing else, and this board's slot is 4 MB against a 3.1 MB
 * image. What slot B saves is RUNTIME: no `esp_wifi_init()` (41,500 bytes,
 * SOC-005), no wolfSSH floor, no littlefs, no shell stacks, and no desktop or
 * status task competing with the emulator for a core.
 *
 * HOW IT IS ENTERED AND LEFT. Entry is the desktop's own `ota rollback`, which
 * calls `esp_ota_set_boot_partition()` on the slot that is not running and
 * restarts. Exit is the same call in reverse from this module. Neither needs a
 * new command.
 *
 * HOW A BROKEN SLOT B CANNOT BRICK THE WATCH. `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`
 * is on, so an image that has been switched to but never confirmed is rolled
 * back by the bootloader on the next reset. Slot B confirms itself only once it
 * is actually running (slotb_confirm()), so a slot B that dies before then
 * returns the watch to the desktop on its own.
 */
#ifndef TWATCH_SLOTB_H
#define TWATCH_SLOTB_H

#include <stdbool.h>

/*
 * The board hook main.c calls. Defined here and declared weak there.
 *
 * `board_conf` is the built-in board.conf text, so slot B can mount the SD card
 * without the shell that normally loads that configuration.
 *
 * Returns true when this module has taken over the boot -- that is, when the
 * running slot is ota_1. On ota_0 it returns false without touching anything.
 */
bool board_startup_override(const char *board_conf);

/*
 * Mark the running slot as good, cancelling the bootloader's automatic rollback.
 * Called once slot B is demonstrably running, NOT before: until this runs, a
 * reset sends the watch back to the desktop, which is the safety net.
 */
void slotb_confirm(void);

/*
 * Hand the boot back to the desktop: set ota_0 as the boot partition and
 * restart. Does not return.
 */
void slotb_return_to_desktop(void);

#endif /* TWATCH_SLOTB_H */
