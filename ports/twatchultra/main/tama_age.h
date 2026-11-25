/*
 * tama_age.h - how long the pet has been up, in the emulator's own time.
 *
 * THE UNIT IS GUEST SECONDS, NOT WALL-CLOCK SECONDS, and that single choice is
 * what makes three of this feature's four requirements fall out for free rather
 * than being implemented:
 *
 *   - a 4x run advances the guest four times as fast, because the guest task
 *     already runs `TMC_FRAME_CYCLES * s_speed` cycles per frame;
 *   - a paused run advances it not at all, because a paused guest is stepped
 *     zero times;
 *   - a reset from an egg starts it at zero, because the machine is rebuilt and
 *     its cycle count starts there.
 *
 * A wall clock with a speed multiplier bolted on would need a second rule for
 * pause and a third for speed, and it would disagree with the guest the first
 * time the board could not keep up. Cycles are the emulator's own notion of
 * time; there is nothing to reconcile.
 *
 * WHY THIS FILE HAS NO ESP DEPENDENCIES. It is formatting and a 16-byte file
 * layout -- decisions that can be walked exhaustively on a host, which
 * tests/test_tama_age.c does. The task in tama.c is the only thing that touches
 * a card or a machine.
 */
#ifndef TWATCH_TAMA_AGE_H
#define TWATCH_TAMA_AGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 24 guest hours to the day, which is the pet's own day: at 1x the guest runs
 * one second per second, so its day and a wall-clock day are the same length and
 * the number the operator reads is the number they expect. */
#define TAMA_AGE_SECONDS_PER_DAY 86400ull

/*
 * THE SIDECAR'S SIZE AND SHAPE.
 *
 * The age cannot go in the checkpoint, because that file is a byte-exact
 * HT24LC08 EEPROM image -- `tmc_eeprom_save()` memcpys 1024 bytes with no length
 * argument (EMU-021 records the overflow that caused when the two disagreed) --
 * and appending to it would make `tama.sav` stop being a dump of the part it
 * claims to be. So the age is a second, tiny file written and read beside it.
 *
 * The two are written together or not at all: see save_checkpoint() in tama.c.
 * An age that outlived its pet, or a pet that outlived its age, would be a
 * number the operator has no reason to trust.
 */
#define TAMA_AGE_FILE_BYTES 16

/* "10d07h31m" -- days, hours into the day, and minutes into the hour. The hour
 * field keeps a fresh pet from reading "0d" for a whole day; the minute field is
 * what keeps the label from looking frozen for an hour, which on a wrist reads as
 * broken rather than slow. Returns the length written, excluding the NUL, or 0
 * when `cap` is too small for even the shortest form. */
size_t tama_age_format(uint64_t seconds, char *out, size_t cap);

/* Serialise and parse the sidecar's 16 bytes. The encoding is explicit
 * little-endian rather than a memcpy of the struct, so the file means the same
 * thing on any host that reads it. */
void tama_age_pack(uint64_t seconds, uint8_t out[TAMA_AGE_FILE_BYTES]);
bool tama_age_unpack(const uint8_t *in, size_t len, uint64_t *seconds);

#endif /* TWATCH_TAMA_AGE_H */
