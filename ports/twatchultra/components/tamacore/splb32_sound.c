/*
 * splb32_sound.c - two-channel audio for the SPLB32
 *
 * Ported from the reference emulator's cores/SPLB32sound.py, including the
 * detail that matters most for matching behaviour: in PWM mode the data
 * register is mapped from 0..255 onto an amplitude of -1..+1 as
 * (data / 128) - 1, so 128 is silence and the range is asymmetric.
 */

#include "splb32_sound.h"

void splb32_sound_init(splb32_sound *s)
{
    s->envelope[0] = s->envelope[1] = 0.0;
    s->toggle_state[0] = s->toggle_state[1] = 1;
    s->last[0] = s->last[1] = 0.0;
    s->emits = 0;
}

void splb32_sound_set_data(splb32_sound *s, int channel, uint8_t ctrl, uint8_t data)
{
    if (channel < 0 || channel > 1)
        return;

    if (ctrl & AUD_CTRL_TONE) {
        /* tone mode: the register sets the envelope, not an instant level.
         * The reference only emits here when the envelope is being cleared. */
        s->envelope[channel] = (double)(data & 0xFF) / 255.0;
        if (data == 0) {
            s->last[channel] = 0.0;
            s->emits++;
        }
        return;
    }

    if (data != 0) {
        double amplitude = ((double)(data & 0xFF) / 128.0) - 1.0;
        s->last[channel] = amplitude;
    } else {
        s->last[channel] = 0.0;
    }
    s->emits++;
}

void splb32_sound_toggle(splb32_sound *s, int channel)
{
    if (channel < 0 || channel > 1)
        return;
    if (s->envelope[channel] == 0.0)
        return;

    s->toggle_state[channel] = -s->toggle_state[channel];
    s->last[channel] = (double)s->toggle_state[channel] * s->envelope[channel];
    s->emits++;
}

double splb32_sound_level(const splb32_sound *s, int channel)
{
    if (channel < 0 || channel > 1)
        return 0.0;
    return s->last[channel];
}
