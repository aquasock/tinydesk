/*
 * sound.h - a beep, from the MAX98357A I2S amplifier.
 *
 * This exists to answer one question: does the speaker on this board work?
 * Nothing else on the board has ever driven it, it is documented only in
 * LilyGo's pin table and rail table, and the cost of finding out is a tone.
 *
 * It is deliberately not a sound API. TinyDesk has no audio layer at all --
 * td_sysinfo_t has no sound ops and the public header set has nothing -- so
 * anything richer than a beep would be inventing an interface for the desktop
 * to adopt, which is a conversation with its author rather than a port task.
 * What is here is the smallest thing that proves the hardware and gives the
 * terminal's BEL somewhere to go.
 */
#ifndef TWATCH_SOUND_H
#define TWATCH_SOUND_H

#include <stdbool.h>

/*
 * Raise the amplifier's rail and start an I2S output; false when the rail or
 * the channel would not come up.
 *
 * Creates the channel once; calling it again is harmless.
 */
bool sound_init(void);

/*
 * Play a short beep. Safe before sound_init() and safe when there is no
 * speaker: it returns quietly, because a beep is feedback and never a thing
 * whose failure matters.
 */
void sound_beep(void);

/* sound_beep() as a td_vterm bell callback, which carries a user pointer.
 * Trivial, but it keeps the vterm's signature out of sound.c. */
void sound_bell(void *user);

#endif
