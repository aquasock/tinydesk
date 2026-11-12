/*
 * pusher.h - the T-Watch Ultra's BOOT pusher (Pusher B) as a key.
 *
 * The watch has two side pushers. Pusher A is the power key and is not a GPIO
 * at all: it reaches the SoC only through the AXP2101's PEK interrupt on GPIO 7,
 * which this port does not install. Pusher B is a plain switch on GPIO 0 with a
 * 10K pull-up on the board, so it idles high and pulls low when pressed.
 *
 * GPIO 0 IS A STRAPPING PIN. Holding this pusher while the watch powers on is
 * how the ESP32-S3 is put into serial download mode -- that is the pin's job and
 * not something this driver can or should prevent. A press after boot is an
 * ordinary button, which is what this file is for.
 */
#ifndef TWATCH_PUSHER_H
#define TWATCH_PUSHER_H

#include <stdbool.h>

/* Called with the bytes the pusher stands for, the same way the touch panel's
 * margin buttons deliver their keys. */
typedef void (*pusher_input_fn)(const char *data, int len);

/*
 * Configure the pusher and start watching it; false only when the pin could
 * not be configured, in which case nothing else in the port is affected.
 *
 * The pin comes from the `pusher.pin` board key and defaults to GPIO 0. -1
 * turns the pusher off entirely, which is the control for deciding whether a
 * press that appeared came from here or from the panel.
 */
bool pusher_init(pusher_input_fn input);

#endif
