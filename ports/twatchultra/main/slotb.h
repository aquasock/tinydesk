/*
 * slotb.h - the watch's two boot personalities, and the boundary between them.
 *
 * THE TWO OTA SLOTS HOLD DIFFERENT IMAGES:
 *
 *   ota_0   the desktop. tinydesk's window manager, the shell, WiFi, telnet and
 *           SSH, Bluetooth, the music player, and LAUNCHERS for slot B's apps.
 *           It contains no emulator core, no cartridge picker and no map viewer.
 *
 *   ota_1   slot B, the emulator personality. The port's four apps (Game Boy,
 *           Pokemon Mini, Tamagotchi, Maps), the display, touch, audio and card
 *           drivers they need, and nothing else: no desktop, no shell, no status
 *           strip, no WiFi, no SSH. Both cores carry only what the apps create.
 *
 * WHY ASYMMETRIC, AND WHY IT CHANGED. The slots held the SAME image while the
 * slot B design was being proved, so that a bad slot B still left a working
 * desktop and so that one flash covered both.
 *
 * The cost of that symmetry was that the desktop linked every emulator core to
 * run menu items that only set a boot partition and restart -- 35 KB of internal
 * DRAM in `.bss` for code it never calls, in the scarcest memory on the board,
 * plus ~300 KB of image written into BOTH slots. Slot B is now proven on the
 * glass, so the scaffolding is gone.
 *
 * BUILDING THEM. One sdkconfig, one source tree, two build directories:
 *    idf.py -B build-desktop -DTD_PERSONALITY=desktop build
 *    idf.py -B build-slotb   -DTD_PERSONALITY=slotb   build
 * A second sdkconfig would be the TOL-008 trap, so there is only one: ESP-IDF
 * keeps it at the PROJECT root, not per build directory, so both personalities
 * read the same generated file and cannot drift apart. What can silently drift
 * is which SOURCES each build compiles, so check_personality.py reads the two
 * linked ELFs and asserts the desktop has no emulator entry points and the weak
 * board hook, and that slot B has the strong one.
 *
 * sdkconfig.defaults must still carry every setting that matters. That file is
 * tracked and sdkconfig is not, so a value living only in the generated copy is
 * lost on a clean checkout -- the FAT and -O2 blocks in sdkconfig.defaults say
 * so at length because both were nearly lost that way.
 *
 * WHAT SLOT B STILL CANNOT DO, WHICH IS THE PRICE OF THE SPLIT: nothing in the
 * desktop can call an emulator. `gb start`, `nes`, `tama do ...`, `maps` and the
 * two benchmark commands no longer exist in the desktop build; they are slot B
 * apps now, reached through the Start menu or `slotb play <key>`. That is the
 * intent -- everything needed to run an emulator lives in the image that runs
 * one -- but it IS a behaviour change and it is recorded as one.
 *
 * HOW IT IS ENTERED. The desktop calls slotb_launch() (slotbl.c), which writes
 * the chosen app key to NVS and switches the boot partition, then restarts.
 *
 * HOW A BROKEN SLOT B IS HANDLED. `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` is on,
 * so an image switched to but never confirmed is rolled back by the bootloader on
 * the next reset. Slot B confirms itself only once it is actually running
 * (slotb_confirm()), so a slot B that dies before then returns the watch to the
 * desktop on its own. That is now the ONLY automatic way back and it is why the
 * confirmation still sits where it does.
 *
 * THE DANGER THAT IS NEW WITH ASYMMETRIC SLOTS: ota_1 can hold an image from a
 * different build than ota_0 -- after a flash that wrote one slot, or an
 * interrupted install. The two would then disagree about which apps exist. Both
 * the desktop launcher and the `slotb` command compare the two app descriptors'
 * version strings before switching and refuse rather than booting into a
 * personality that cannot do what was asked; see slotb_launch().
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

/*
 * Boot slot B directly into one of its apps, blanking the panel first.
 *
 * THIS IS THE START MENU'S PATH AND IT EXISTS SO THE WEARER SEES A GAME RATHER
 * THAN A REBOOT. The menu entry used to be a `.tdlink` pointing at a Python
 * script, which the desktop opens by running `py "<path>"` IN A TERMINAL -- so
 * launching a game put a terminal window on the glass, echoed the command, ran a
 * two-line script and printed "rebooting into ota_1" before anything happened.
 * A native app calls this instead and nothing is shown but black.
 *
 * `key` names the app (see slotb_apps.h) or is NULL for slot B's menu.
 * Returns false when slot B holds no image, holds an image from a different
 * build, or the key is not an app -- and in every one of those cases NOTHING IS
 * CHANGED, so the caller can still put a message on the glass.
 * On success this does not return.
 *
 * DEFINED IN slotbl.c AND COMPILED INTO THE DESKTOP ONLY.
 */
bool slotb_launch(const char *key);

/*
 * Why the last slotb_launch() refused, as a sentence the glass can carry.
 * "slot B holds no image" and "slot B is version X, not Y" are different faults
 * with different fixes, and the Start menu has one line to say so in.
 */
const char *slotb_last_error(void);

/*
 * The `slotb` shell command: enter slot B on purpose.
 *
 * This is what the Start menu's launcher calls. It is a command rather than
 * the desktop's own `ota rollback` for two reasons: `rollback` means "the other
 * slot", which is true only by accident and would break the day the slots are
 * used for anything else; and `ota rollback` has a branch that MARKS THE
 * RUNNING IMAGE INVALID when it is pending verification, which is not what a
 * person who wants to play a game is asking for.
 */
void slotb_register_command(void);

/*
 * Record what the picker chose, or what an app failed on, into flash.
 *
 * For the one failure the glass cannot carry: a cartridge that will not open.
 * "cannot open the cartridge" is 26 characters and the path that decides the
 * answer is 40 to 130, against a 38-character row and no console. Two strings
 * survive a reset in the sector the old boot mark used to occupy, and
 * `esptool read_flash 0x1B000 4K` reads them back.
 *
 * Both may be NULL. Safe to call at any point after flash init.
 */
void slotb_record(const char *why, const char *path);

#endif /* TWATCH_SLOTB_H */
