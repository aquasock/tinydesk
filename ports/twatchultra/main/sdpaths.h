/*
 * sdpaths.h - where things live on the card.
 *
 * The card was reorganised into the layout the TinyTang release cards use:
 * media by console, diagnostics apart from playable software, saves and
 * screenshots in their own directories. That means every literal "/sd/foo.rom"
 * in this port pointed at a file that had moved.
 *
 * TWO RESOLUTIONS, AND THE SECOND IS NOT NOSTALGIA. `sd_media()` tries the
 * organised path and falls back to the flat one, because a card written before
 * this change still holds a working firmware's worth of files and there is no
 * reason to make an old card stop booting. Upgrading the card is a thing the
 * operator does at a keyboard; upgrading the firmware is a thing that happens
 * over USB. They do not always happen together, and the watch should not care.
 *
 * The fallback costs one stat() per lookup, and every caller here does it once
 * at start or once per command -- not in a frame loop.
 *
 * `sd_out()` is the other direction and has no fallback: diagnostics, logs and
 * screenshots belong in the directories the layout reserved for them, and
 * writing them to the card's root is what made the last card 205 entries of
 * which 30 were content.
 */
#ifndef TWATCH_SDPATHS_H
#define TWATCH_SDPATHS_H

#include <stdbool.h>

/* The organised path if it exists, else the flat one, else `organized` so the
 * caller's open() fails with a path in the error that a human can act on. */
const char *sd_media(const char *organized, const char *flat);

/* The directory a diagnostic file of this name belongs in. Returns a path built
 * from `dir` and `name` in a shared static buffer, so copy it if it must outlive
 * the next call. Two rotating buffers so one call can be nested inside another's
 * use without them trampling each other -- the callers here do exactly that when
 * they print a path and then open it. */
const char *sd_out(const char *dir, const char *name);

#endif
