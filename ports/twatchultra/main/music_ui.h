/*
 * music_ui.h - the Music window: a playlist view onto music_player.
 *
 * Registered as an ordinary TinyDesk app, so it appears in the Start menu and
 * opens like any other window. It does not start or own playback; see
 * music_ui.c for why that separation matters.
 */
#ifndef TWATCH_MUSIC_UI_H
#define TWATCH_MUSIC_UI_H

void music_ui_register(void);

/* Diagnostic builds only (MUSIC_UI_SELFTEST): run the window's scan and play
 * path at boot and write the result to /fs/root/musicboot.txt. Called from the
 * boot task, which is the only place allowed to touch /fs. */
void music_ui_selftest(void);

#endif
