/*
 * music.h - the `music` shell command: play a FLAC file through the speaker.
 *
 * The decoder itself is behind musicdec.h; this is only the command's
 * registration, declared here so link_twatch.c can call it the same way it
 * calls uac2_register_command() and the rest.
 */
#ifndef TWATCH_MUSIC_H
#define TWATCH_MUSIC_H

#include <stdbool.h>

bool music_register_command(void);

#endif
