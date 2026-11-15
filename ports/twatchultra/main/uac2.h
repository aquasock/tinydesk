/*
 * uac2.h - USB Audio Class 2.0 playback (see uac2.c).
 *
 * The watch's speaker cannot be used in public, so headphones are the only
 * viable audio output. Espressif's usb_host_uac is UAC 1.0 only and this DAC is
 * UAC 2.0, so the playback path is implemented here.
 */
#ifndef TWATCH_UAC2_H
#define TWATCH_UAC2_H

#include <stdbool.h>
#include <stdint.h>

/* Find the DAC and prepare the endpoints. False when it is not attached. */
bool uac2_open(void);

/* Claim the interface, set the rate and bring the isochronous stream up. */
bool uac2_start(void);

/* Stop the stream and release the interface. */
void uac2_stop(void);

/* True while the isochronous endpoint is running. */
bool uac2_ready(void);

/* Feed interleaved S16 stereo at uac2_rate(). Returns bytes accepted; may be
 * short when the ring is full, which is a drop rather than a block. */
int uac2_write(const uint8_t *pcm, int len);

/* The rate this driver asks the device for. */
uint32_t uac2_rate(void);

void uac2_stats(uint32_t *frames, uint32_t *errors, uint32_t *underruns,
                uint32_t *missed);

/* Register the `uac2` shell command: status, open, start, stop, tone. */
bool uac2_register_command(void);

#endif
