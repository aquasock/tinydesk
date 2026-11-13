/*
 * splb32_sound.h - two-channel audio for the SPLB32
 *
 * The V3 makes sound with a PWM channel and a tone channel, driven from the
 * timer overflow path. This model follows the reference emulator exactly: it
 * produces a stream of amplitude samples, it does not synthesise a waveform.
 *
 * Two sources of events:
 *
 *   set_data   the firmware writes an audio data register. In PWM mode the
 *              register holds an amplitude; in tone mode it sets the envelope.
 *   toggle     TM0 or TM1 overflowed while routed to this channel. Flips the
 *              square wave and emits the new level.
 *
 * A front end turns emits into sound; a bench harness turns them into samples
 * for comparison.
 */

#ifndef SPLB32_SOUND_H
#define SPLB32_SOUND_H

#include <stdint.h>

#define AUD_CTRL_PWM_ENBL   0x80
#define AUD_CTRL_TONE       0x40
#define AUD_CTRL_TM0_OVFLW  0x08
#define AUD_CTRL_TM1_OVFLW  0x10

typedef struct {
    double envelope[2];      /* 0..1, set in tone mode */
    int    toggle_state[2];  /* +1 / -1, flips on each overflow */

    /* last emitted sample per channel, and a count of emits, for tests */
    double last[2];
    unsigned long emits;
} splb32_sound;

void splb32_sound_init(splb32_sound *s);

/* Audio data register written. `ctrl` is the channel's control byte. */
void splb32_sound_set_data(splb32_sound *s, int channel, uint8_t ctrl, uint8_t data);

/* A driver timer overflowed and is routed to this channel. */
void splb32_sound_toggle(splb32_sound *s, int channel);

/* Current output level for a channel, -1..+1. */
double splb32_sound_level(const splb32_sound *s, int channel);

#endif
