/*
 * rtc.h - the PCF85063A real-time clock at I2C 0x51.
 *
 * WHY THIS EXISTS. This board has a battery-backed RTC and TinyDesk has no RTC
 * hook: upstream takes the time from SNTP, which means a watch with no network
 * shows "--:--" and "no clock" while a correct time sits in the chip two
 * millimetres away. This port supplies the time at boot instead, and writes it
 * back when it is set, so a correction survives a power cycle.
 *
 * SNTP STILL WINS. RTC at boot, network time when it arrives: the chip does not
 * know about time zones and can drift, but it is never wrong by more than that,
 * and it is the only source available before Wi-Fi associates.
 *
 * The register map is mainline's (drivers/rtc/rtc-pcf85063.c), not a guess: the
 * time starts at 0x04 and is seven BCD bytes -- second, minute, hour, day,
 * weekday, month, year -- with bit 7 of the seconds register set when the
 * oscillator has stopped, which means the stored time cannot be trusted.
 */
#ifndef TWATCH_RTC_H
#define TWATCH_RTC_H

#include <stdbool.h>
#include <stdint.h>

/* Read the chip's time as seconds since the epoch, UTC. False if the part does
 * not answer or its oscillator-stop flag says the value is not valid. */
bool rtc_read(int64_t *utc);

/* Write seconds since the epoch, UTC, to the chip. */
bool rtc_write(int64_t utc);

/*
 * Seed the system clock from the RTC if it holds a valid time.
 *
 * Called once at start-up, before TinyDesk runs, so the desktop has a clock
 * from its first frame rather than a dash. Does nothing and returns false when
 * the chip is absent, stopped, or holding something absurd -- a watch that
 * boots to 1970 is worse than one that admits it does not know.
 */
bool rtc_seed_system_clock(void);

/* Arm the write-back: when the system clock is set later (Date & time, or
 * SNTP), the new time goes to the chip too. */
void rtc_writeback_enable(bool on);

/*
 * Set the system clock from a source this port owns, and write it through to
 * the chip.
 *
 * The same two steps the shared app takes for SNTP: settimeofday, then
 * board_time_set_post. Existing so a GPS time survives a power cycle the way a
 * network time does; settimeofday alone would leave the chip holding the old
 * value and the next boot would undo the correction.
 *
 * Refuses a time before 2020 and changes nothing in that case.
 */
bool board_time_apply(int64_t utc);

#endif
