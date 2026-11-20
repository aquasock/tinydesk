/*
 * klio.h - the BHI260AP's self-learning gesture engine (klio.c).
 */
#ifndef KLIO_H
#define KLIO_H

#include <stdbool.h>
#include <stdint.h>   /* uint32_t in klio_learn_state_t */

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

/* ---- learning a gesture, as a state machine the window can drive -----------
 *
 * A pass lasts as long as the wearer keeps repeating a motion, so it cannot be a
 * blocking call: the window still has to draw and read touch. begin() starts
 * one, tick() advances it and is cheap, cancel() abandons it. `klio learn` uses
 * exactly these three, so the command and the button cannot drift apart.
 */
typedef struct
{
    bool active;          /* a pass is running */
    bool succeeded;       /* the last finished pass produced a pattern */
    bool got_frame;       /* the engine has sent at least one frame this pass */
    int progress;         /* 0..100, and it CAN fall back to 0: see the note below */
    int reason;           /* 0 progressing, 1 non-repetitive, 2 no movement */
    int learnt_index;     /* the engine's index, -1 until a pattern is learnt */
    uint32_t frames;      /* frames received since the pass began */
} klio_learn_state_t;

bool klio_learn_active(void);
bool klio_learn_begin(void);
void klio_learn_cancel(void);
bool klio_learn_tick(klio_learn_state_t *out);

#endif
