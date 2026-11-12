/*
 * gps.h - the u-blox MIA-M10Q on this board.
 *
 * A UART receiver, not an I2C part and not SPI: TX 43, RX 44, PPS 13, at
 * 38400 8N1. Its supply is the AXP2101's BLDO1 -- one register below the
 * speaker's BLDO2 in both the enable and voltage registers, which is the kind
 * of adjacency that makes a wrong write silent, so board_gnss_rail_on() reads
 * the rail back rather than trusting the write.
 *
 * The module answers UBX as well as NMEA on the same port. Probing is done with
 * UBX-MON-VER, which returns the software and hardware version strings and a
 * list of extensions, one of which is `OD=MIA-M10Q` -- so one exchange proves
 * the module is alive AND says which module it is, rather than inferring
 * presence from a stream of NMEA that any receiver could have produced.
 */
#ifndef TWATCH_GPS_H
#define TWATCH_GPS_H

#include <stdbool.h>
#include <stdint.h>

#include "nmea.h"

/*
 * Open the rail and the UART, and ask the module who it is.
 *
 * Returns false if the rail does not come up or the UART cannot be installed.
 * A module that stays silent is NOT a failure of this call: the rail and the
 * port can be perfectly good with nothing answering, and the caller is given
 * `heard_any` so it can tell "silent" from "speaking a protocol we did not
 * expect". The first thing to try when it is silent is swapping the two data
 * pins, because a crossed pair looks exactly like a dead module.
 */
typedef struct
{
    bool rail;          /* BLDO1 came up */
    bool port;          /* the UART was installed */
    bool ubx_reply;     /* a well-formed UBX-MON-VER came back */
    bool heard_any;     /* bytes arrived at all, whatever they were */
    int bytes;          /* how many, so "a few" and "a flood" differ */
    /* Sized to the UBX-MON-VER fields they hold: three fixed 30-byte strings,
     * each possibly not NUL-terminated, so each buffer carries 30 bytes of
     * content plus a terminator. Sized narrower, snprintf truncates -- which
     * the compiler refused to let pass, and it was right. */
    char model[31];     /* "MIA-M10Q" when the reply carried OD= */
    char sw[31];        /* software version, for the record */
    char hw[31];        /* hardware version */
} gps_probe_t;

bool gps_open(gps_probe_t *out);

/* Feed whatever has arrived into the parser, without blocking. Returns the
 * number of bytes consumed. */
int gps_pump(nmea_state_t *nmea);

/* Read and parse for this many milliseconds, then return. Used by the shell
 * command so a monitor run always terminates. */
void gps_listen(nmea_state_t *nmea, int ms, void (*tick)(const nmea_state_t *, void *),
                void *ctx, int tick_ms);

void gps_close(void);

/* Relay every raw sentence for this long. Used by `gps raw` to tell a receiver
 * with no antenna from one with no fix -- a GSV listing satellites means the
 * antenna hears them. */
void gps_raw_dump(int ms, void (*sink)(const char *line, void *ctx), void *ctx);

/* The PPS line's level, or -1 when no module was opened. A module with a fix
 * pulses this once a second; one without does not pulse at all. It is the only
 * signal here that a parser bug cannot fabricate. */
int gps_pps_level(void);

/* Register the `gps` shell command. */
bool gps_register_command(void);

#endif
