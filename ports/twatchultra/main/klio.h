/*
 * klio.h - the BHI260AP's self-learning gesture engine (klio.c).
 */
#ifndef KLIO_H
#define KLIO_H

#include <stdbool.h>

/* Register the `klio` diagnostic command with the shell. */
bool klio_register_command(void);

/* How many pattern slots the part offers for recognition, as the part itself
 * reports it. Falls back to 25 when the parameter cannot be read, and is capped
 * at the store's own limit. 0 before the part has booted. */
int klio_max_patterns(void);

/* Put the stored patterns back into the engine, once, straight after the part's
 * firmware boots.
 *
 * THIS EXISTS BECAUSE THE PATTERN TABLE IS NOT DURABLE. The BHI260AP runs
 * firmware the host uploads into its RAM, so every boot starts with an empty
 * engine and any gesture learnt in the previous session is gone. `klio save`
 * writes the engine's own 157-byte blob to a file; this puts it back.
 *
 * Best-effort by design: a missing, unreadable or corrupt store logs one line
 * and the sensor still comes up. Persistence must never be able to stop the IMU
 * from starting. Called by imu_start() through a weak symbol, so a build without
 * this file has no hook and needs no change. */
void klio_after_boot(void);

#endif
