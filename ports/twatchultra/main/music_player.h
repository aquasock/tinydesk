/*
 * music_player.h - the FLAC player, and the track list the UI walks.
 *
 * THE DECODER IS NOT IN THE APP IMAGE. It runs out of the `modules` partition,
 * mapped as instructions, so a codec costs the app nothing -- see
 * music_player.c for why that matters and what it took.
 *
 * TWO FRONTS, ONE PLAYER. The shell commands `music` and `modflac` play a path
 * directly, and music_ui.c drives the same task through a playlist. They share
 * every buffer, the decoder and the I2S stream, because two players would mean
 * two amplifier opens and a race over which one owns the speaker.
 *
 * THE SCAN IS THE UI'S JOB TO TRIGGER, not the player's to do at boot: a card
 * that is absent, slow or has 3000 files in it should cost nothing until
 * somebody opens the Music window.
 */
#ifndef TWATCH_MUSIC_PLAYER_H
#define TWATCH_MUSIC_PLAYER_H

#include <stdbool.h>
#include <stdint.h>

/* One line of the playlist. Generous: FAT long names on this card run to 60
 * characters and are truncated rather than rejected, because a track whose name
 * does not fit is still a track. */
#define MUSIC_NAME_MAX   72
#define MUSIC_TRACK_MAX  256

/* Player state, as the UI reads it. A snapshot: everything is a copy, so a
 * caller can print or draw without holding anything. */
typedef struct
{
    bool running;         /* the playback task exists */
    int state;            /* 0 idle, 1 playing, 2 finished, 3 error */
    int index;            /* playlist entry, -1 for a path played directly */
    uint32_t frames;      /* stereo frames written to the amplifier */
    uint32_t rate;        /* Hz, as the file's STREAMINFO said; 0 before then */
    uint32_t seconds;     /* frames / rate, 0 when the rate is not known yet */
    uint32_t errors;      /* decode errors */
    uint32_t short_writes;/* amplifier writes that took fewer frames than given */
    const char *message;  /* why it stopped, or "" -- a static string */
} music_status_t;

/* ------------------------------------------------------------- playlist */

/*
 * Read `dir` for *.flac and fill the playlist. Returns the count (0 for an empty
 * or missing directory -- an empty list is not an error), or -1 if the name pool
 * could not be allocated.
 *
 * REPLACES whatever was there, and stops a track that was playing from it,
 * because the index would otherwise point at a different file.
 */
int music_tracks_scan(const char *dir);

int music_track_count(void);

/* The name as shown and the full path to open. Both are "" / NULL when the
 * index is out of range. */
const char *music_track_name(int index);
const char *music_track_path(int index);

/* The directory the last scan read, or "" if none. */
const char *music_tracks_dir(void);

/* --------------------------------------------------------------- player */

/* Play playlist entry `index`. False when the task could not be created or the
 * index is out of range. Replaces a track already playing. */
bool music_play_index(int index);

/* Play an arbitrary path, as the shell command does. */
bool music_play_path(const char *path);

/* Stop. Returns at once; the task closes the stream and frees its buffers. */
void music_stop(void);

void music_status_get(music_status_t *out);

/* ------------------------------------------------------------- commands */

bool music_player_register_commands(void);

#endif
