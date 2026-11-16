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

/* Automatic mode, for OTG host mode with usb.dac = 1 (usbhost.c calls it at
 * boot): the DAC is opened when it is plugged in and forgotten when it goes. */
bool uac2_auto_start(void);

/* True while a DAC is open and ready to stream. */
bool uac2_attached(void);

/* Its product string, for the USB page ("?" before one has been seen). */
const char *uac2_product(void);

/* Claim the interface, set the rate and bring the isochronous stream up, at
 * 48 kHz. */
bool uac2_start(void);

/* As uac2_start(), at `rate`, which must be in the clock's RANGE list. A stream
 * already running at another rate is stopped and restarted, and whatever was
 * queued for the old rate is discarded. */
bool uac2_start_rate(uint32_t rate);

/* Stop the stream and release the interface. */
void uac2_stop(void);

/* True while the isochronous endpoint is running. */
bool uac2_ready(void);

/* Feed interleaved S16 stereo at uac2_rate(). Returns len when the chunk was
 * queued and 0 when the ring is full: a drop rather than a block, all or
 * nothing. For a frame loop that must never stall. */
int uac2_write(const uint8_t *pcm, int len);

/* As uac2_write(), but waits up to timeout_ms for ring space: a producer paced
 * by the sink, which can never fall behind it. len must not exceed the ring. */
int uac2_write_wait(const uint8_t *pcm, int len, uint32_t timeout_ms);

/* The rate the stream is running at. */
uint32_t uac2_rate(void);

void uac2_stats(uint32_t *frames, uint32_t *errors, uint32_t *underruns,
                uint32_t *missed);

/* Register the `uac2` shell command: status, open, start, stop, tone. */
bool uac2_register_command(void);

#endif
