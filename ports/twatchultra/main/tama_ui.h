/*
 * tama_ui.h - what the margin buttons mean to the Tamagotchi, as arithmetic.
 *
 * THIS FILE HAS NO ESP DEPENDENCIES ON PURPOSE. It is the decision layer: given
 * which points on the glass are being pressed right now, which action (if any)
 * that means. Keeping it out of tama.c means the whole rule -- including the
 * hold-to-arm interlock -- can be swept on a host over every combination of two
 * fingers, which is what tests/test_tama_ui.c does. A chord rule that has only
 * ever been tried by a human finger on a 41-pixel button is a rule that has
 * been tried once.
 *
 * THE ARM, and why it is not a confirmation dialog. Reset and Load destroy state
 * that cannot be recovered, and a 41 px button in a column of five is easy to
 * hit by accident -- a thumb sliding down the edge finds `Reset` as readily as
 * `Sound`. So the destructive three (Save, Load, Reset) require a SECOND finger
 * to be held on the reserved slot at the top of the right column, in the way a
 * console's home button is held to reach its menu. Two simultaneous, distinct,
 * specific locations is a thing a hand does deliberately and does not do by
 * sliding, and unlike a "press again to confirm" it cannot be satisfied by
 * mashing one button twice.
 *
 * ORDER DOES NOT MATTER, and that is a decision made from measurement rather
 * than convenience. The controller assigns point numbers in arrival order, so a
 * rule of "point 1 must be the arm" would silently invert when the arm finger
 * happened to land second. Instead the rule is a property of the SET: the arm
 * slot must be occupied at the moment the action is pressed, whoever got there
 * first. The touch dumps behind this (see TCH-012) show both points reported
 * on every block while a chord is held, so the set is observable on the same
 * report the action arrives on.
 */
#ifndef TAMA_UI_H
#define TAMA_UI_H

#include <stdbool.h>

#include "edges.h"

/* The controller reports at most two points (MAX_FINGERS in touch.c). */
#define TAMA_UI_POINTS 2

typedef struct
{
    edge_region_t region;   /* EDGE_NONE when this point is not present */
    int index;
} tama_ui_point_t;

/* What is being touched right now. Rebuilt from scratch on every report, so an
 * absent finger is simply absent -- there is no stale state to clear. */
typedef struct
{
    tama_ui_point_t p[TAMA_UI_POINTS];
    int n;
} tama_ui_frame_t;

typedef enum
{
    TAMA_ACT_NONE = 0,
    TAMA_ACT_PAUSE,      /* stop stepping the guest; press again to resume  */
    TAMA_ACT_SAVE,       /* gated: write /sd/tama.sav                      */
    TAMA_ACT_LOAD,       /* gated: read /sd/tama.sav and reset into it     */
    TAMA_ACT_SPEED,      /* cycle 1x -> 2x -> 4x -> 1x                     */
    TAMA_ACT_SOUND,      /* toggle the guest's beeps                       */
    TAMA_ACT_SHOT,       /* write the frame to a .bmp on the card          */
    TAMA_ACT_RESET,      /* gated: fresh machine, guest state discarded    */
    TAMA_ACT_EXIT,       /* gated: leave the emulator, back to the shell   */
} tama_act_t;

/* At most two actions can complete in one report: one from each finger, and at
 * most one of those can be gated (the other is then the arm, which has no
 * action of its own). */
#define TAMA_UI_MAX_ACTIONS 2

typedef struct
{
    tama_ui_frame_t prev;
    bool have_prev;
    unsigned refused;    /* gated button pressed without the arm held */
    unsigned armed_frames;   /* reports seen with the arm down, for the log */
} tama_ui_t;

void tama_ui_init(tama_ui_t *ui);

/*
 * Feed one report and get the actions it completes.
 *
 * Actions fire on PRESS EDGES only: a held finger reports every ~20 ms and a
 * button that fired per report would be a hundred presses a second, which for
 * `Speed` means cycling through every rate faster than the screen can show it.
 *
 * `out` must have room for TAMA_UI_MAX_ACTIONS. Returns how many were written,
 * 0 when nothing completed.
 */
int tama_ui_update(tama_ui_t *ui, const tama_ui_frame_t *now,
                   tama_act_t out[TAMA_UI_MAX_ACTIONS]);

/* Is the arm slot occupied in this frame? Exposed for the caller that wants to
 * paint an indication, and used by the tests. */
bool tama_ui_armed(const tama_ui_frame_t *f);

/* The action a single point means, or TAMA_ACT_NONE. Exposed so the label table
 * in edges.h and this mapping can be checked against each other. */
tama_act_t tama_ui_action_of(edge_region_t region, int index);

/*
 * Should touching this slot give haptic feedback?
 *
 * TWO RULES, and the difference between them is what the operator corrected:
 *
 *   the ARM always buzzes, because it is a control whose only job is to be held
 *   and the player needs to feel that it has taken;
 *   a GATED action buzzes only while the arm is held, because unarmed it does
 *   nothing at all -- and a buzz for a press that provably did nothing is a lie
 *   the finger can detect;
 *   an UNGATED action always buzzes, since it fires on every press.
 *
 * THE FIRST VERSION OF THIS FUNCTION DID NOT TAKE `armed` AND WAS WRONG IN BOTH
 * DIRECTIONS AT ONCE. It returned true for every mapped action whether or not it
 * would fire, so Save and Load buzzed when the arm was down (correct) and equally
 * when it was not (wrong -- the operator heard it and said so). The reason it had
 * no `armed` parameter is that its author reasoned "a physical button clicks
 * whether or not the machine acts on it", which is true of a real button and not
 * of this one: a gated margin slot is not a button that fails, it is a button
 * that is not armed, and the whole point of the interlock is that an unarmed
 * press is not a press.
 *
 * `armed` is whether the arm slot is occupied in the frame being delivered, so
 * the caller passes what the UI layer is about to decide on -- not a cached
 * reading from a previous frame.
 */
bool tama_ui_haptic_of(edge_region_t region, int index, bool armed);

/* True when that action may only fire while armed. */
bool tama_ui_is_gated(tama_act_t act);

/* Short name for the log and the report file. Never NULL. */
const char *tama_ui_action_name(tama_act_t act);

#endif /* TAMA_UI_H */
