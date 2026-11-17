/*
 * music_ui.c - the Music window: a playlist, and the transport under it.
 *
 * THE LAYOUT IS TINYPHOSPHOR'S, SHRUNK. That player is the reference for how a
 * music app should read on a small screen -- the library fills the window, the
 * track it is on and its time sit just above a row of transport buttons, and
 * the buttons never move -- so this is the same shape at watch size: a folder
 * line, the list with a scrollbar, then now-playing, elapsed time and four
 * buttons. What it does NOT have is the progress bar; see below.
 *
 * IT IS A VIEW ONTO music_player, NEVER A SECOND PLAYER. Playback lives in one
 * task that owns the amplifier; this window calls music_play_index() and reads
 * music_status_get() twice a second. That is why closing the window does not
 * stop the music, and why the shell's `music` command and the Play button can
 * never both hold the speaker.
 *
 * WHY THERE IS NO PROGRESS BAR. TinyPhosphor draws one for the formats whose
 * headers carry a length. A FLAC stream does carry total_samples in STREAMINFO,
 * but this decoder's handle is a fixed 32-byte struct with the decoder object
 * placement-new'd immediately after it, so adding a field is an ABI change that
 * needs the blob rebuilt and reflashed -- not a line of UI. Elapsed time is
 * exact, costs nothing and is what a person checks anyway. When the handle is
 * next touched, total_samples is worth adding and the bar can follow.
 */
#include <stdio.h>
#include <string.h>

#include "music_player.h"

/* td_apps.h brings in the window manager and the widget set. */
#include "td_apps.h"

#define TICK_MS 500
#define WIN_W   38
#define WIN_H   20

/* Where music is looked for, in order. The organised card layout first: that is
 * where the real tracks live on this watch. /sd/media/audio is where the
 * generated test tones were put, so a card with no library still has something
 * to prove the path with. */
static const char *const k_dirs[] = {
    "/sd/ROCKBOX/Audio",
    "/sd/media/audio",
    "/sd/music",
};

static td_window_t *s_win;
static td_widget_t *s_list, *s_folder, *s_now, *s_status, *s_message;

/* Last drawn state, so the window only repaints when something moved: a 500 ms
 * tick that invalidated unconditionally would redraw the whole desktop twice a
 * second for nothing. */
static int s_shown_state = -1;
static int s_shown_index = -2;
static uint32_t s_shown_seconds;
static bool s_shown_running;

/* The directory the current list came from, NULL before the first scan. */
static const char *s_scanned;

static void fmt_time(char *out, size_t cap, uint32_t seconds)
{
    snprintf(out, cap, "%02u:%02u", (unsigned)(seconds / 60u),
             (unsigned)(seconds % 60u));
}

static void refresh_list(void)
{
    td_list_set_count(s_list, music_track_count());
    td_list_select(s_list, 0);
}

/* The list asks for a row's text as it draws it, which is also where a row gets
 * its colour: the track being played is drawn in the accent colour. */
static const char *item_text(td_widget_t *w, int index, int *fg, void *user)
{
    (void)w;
    (void)user;

    music_status_t st;

    music_status_get(&st);
    if (st.running && st.index == index)
        *fg = td_theme()->accent;
    return music_track_name(index);
}

static void show_status(void)
{
    music_status_t st;

    music_status_get(&st);

    if (st.running && st.index >= 0)
    {
        char line[48];
        char t[12];

        fmt_time(t, sizeof(t), st.seconds);
        /*
         * The rate is 0 between pressing Play and the decoder parsing
         * STREAMINFO -- a few hundred milliseconds. Printing "0 Hz" there is a
         * number that is not true, so the field is left out until it is known
         * rather than shown wrong and corrected half a second later.
         */
        if (st.rate)
            snprintf(line, sizeof(line), "%s  %s  %lu Hz", t,
                     st.state == 1 ? "playing" : "stopping", (unsigned long)st.rate);
        else
            snprintf(line, sizeof(line), "%s  %s", t,
                     st.state == 1 ? "playing" : "stopping");
        td_widget_printf(s_now, "%s", music_track_name(st.index));
        td_widget_printf(s_status, "%s", line);
    }
    else if (st.state == 2 || st.state == 3)
    {
        /* Finished or failed: say which, and keep the last track named so the
         * window does not look empty once a track ends. */
        td_widget_printf(s_now, "%s", st.index >= 0 ? music_track_name(st.index)
                                                    : "Nothing");
        td_widget_printf(s_status, "%s%s%s", st.state == 2 ? "finished" : "error",
                         st.message[0] ? ": " : "", st.message);
    }
    else
    {
        td_widget_printf(s_now, "Nothing playing");
        td_widget_printf(s_status, "%d tracks", music_track_count());
    }

    const uint32_t secs = st.seconds;

    if (st.state != s_shown_state || st.index != s_shown_index ||
        st.running != s_shown_running || secs != s_shown_seconds)
    {
        s_shown_state = st.state;
        s_shown_index = st.index;
        s_shown_running = st.running;
        s_shown_seconds = secs;
        if (s_win)
            td_win_invalidate(s_win);
    }
}

static void on_play(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;

    const int sel = td_list_selected(s_list);

    if (sel < 0 || sel >= music_track_count())
    {
        td_widget_printf(s_message, "no track selected");
        return;
    }
    if (!music_play_index(sel))
    {
        td_widget_printf(s_message, "cannot start the player");
        return;
    }
    td_widget_printf(s_message, "");
    show_status();
}

static void on_stop(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;

    music_stop();
    td_widget_printf(s_message, "stopping");
    show_status();
}

/* Prev and Next step the selection AND play it when something is already
 * playing, which is what a transport button means. With nothing playing they
 * only move the highlight, so browsing does not start audio. */
static void step(int direction)
{
    const int count = music_track_count();

    if (count <= 0)
        return;

    music_status_t st;
    int sel = td_list_selected(s_list);

    music_status_get(&st);
    sel += direction;

    if (sel < 0)
        sel = count - 1;
    if (sel >= count)
        sel = 0;

    td_list_select(s_list, sel);

    if (st.running)
        (void)music_play_index(sel);
    show_status();
}

static void on_prev(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    step(-1);
}

static void on_next(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;
    step(1);
}

/* Enter or a double click on a row plays it. */
static void on_activate(td_widget_t *w, void *user)
{
    (void)w;
    (void)user;

    const int sel = td_list_selected(s_list);

    if (sel >= 0 && sel < music_track_count())
    {
        if (music_play_index(sel))
            td_widget_printf(s_message, "");
        else
            td_widget_printf(s_message, "cannot start the player");
        show_status();
    }
}

static void on_tick(td_window_t *win)
{
    (void)win;
    show_status();
}

static void on_close(td_window_t *win)
{
    (void)win;
    /* The track keeps playing: this is a view, and a person who closes the
     * window to look at something else has not asked for the music to stop. */
    s_win = NULL;
}

static void launch(void)
{
    if (td_win_is_open(s_win))
    {
        td_win_focus(s_win);
        return;
    }

    td_window_desc_t d = {
        .title = "Music",
        .rect = td_rect(-1, -1, WIN_W, WIN_H),
        .flags = TD_WIN_MOVABLE | TD_WIN_CLOSABLE | TD_WIN_RESIZABLE,
        .min_w = 24,
        .min_h = 11,
        .on_tick = on_tick,
        .tick_ms = TICK_MS,
        .on_close = on_close,
    };

    s_win = td_win_create(&d);

    if (!s_win)
        return;

    /* Scan before creating widgets: the list needs its count, and an empty
     * library should show as an empty library rather than as an error. */
    if (!s_scanned)
    {
        for (size_t i = 0; i < sizeof(k_dirs) / sizeof(k_dirs[0]); i++)
        {
            if (music_tracks_scan(k_dirs[i]) > 0)
            {
                s_scanned = k_dirs[i];
                break;
            }
        }
        if (!s_scanned)
            s_scanned = "";
    }

    /*
     * THE LAYOUT, top to bottom. The folder line and the list are measured from
     * the top; everything below is anchored to the bottom edge, so a resize
     * grows the list and leaves the transport where it was.
     */
    s_folder = td_label(s_win, 0, 0, 0, "");
    td_widget_set_color(s_folder, td_theme()->dim, TD_COLOR_DEFAULT);
    td_widget_printf(s_folder, "%s", s_scanned);

    s_list = td_list(s_win, td_rect(0, 1, -1, -5), item_text, on_activate, NULL);
    td_scrollbar(s_win, -1, 1, -5, s_list);
    refresh_list();

    s_now = td_label(s_win, 0, -4, 0, "");
    s_status = td_label(s_win, 0, -3, 0, "");
    td_widget_set_color(s_status, td_theme()->dim, TD_COLOR_DEFAULT);

    /* The transport row, captions of four to five characters, each button the
     * width the widget gives a caption (length plus five) -- the same rule
     * TinyPhosphor uses to space its row. */
    td_button(s_win, 0, -2, "Play", on_play, NULL);
    td_button(s_win, 9, -2, "Stop", on_stop, NULL);
    td_button(s_win, 18, -2, "Prev", on_prev, NULL);
    td_button(s_win, 27, -2, "Next", on_next, NULL);

    s_message = td_label(s_win, 0, -1, 0, "");
    td_widget_set_color(s_message, td_theme()->dim, TD_COLOR_DEFAULT);

    /* Force the first paint to compare against nothing. */
    s_shown_index = -2;
    s_shown_state = -1;
    show_status();
}

static const td_app_t s_app = {"Music", launch, ">>"};

void music_ui_register(void)
{
    td_app_register(&s_app);
}

/* ------------------------------------------------------------------------ */
#ifdef MUSIC_UI_SELFTEST

/*
 * DIAGNOSTIC BUILD ONLY, removed with the define in CMakeLists once answered.
 *
 * It plays and stops a run of tracks, recording PSRAM free and the failure
 * message after each one, because the operator's report was "the app stopped
 * playing after 4-5 songs with error: no amplifier". That message came from
 * sound_stream_open() failing, and there are five different ways for it to
 * fail -- so this measures rather than guesses, and PSRAM free per iteration
 * separates a leak (a number falling every time) from a state machine fault
 * (a number flat until the failure).
 *
 * It runs on the BOOT task, which is the only place allowed to touch /fs.
 */
#include "esp_heap_caps.h"
#include "modhost.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void music_ui_selftest(void)
{
    FILE *f = fopen("/fs/root/musicboot.txt", "w");

    if (!f)
        return;

    /* Where Music sits in the Start menu, and what follows it. Registration
     * order is menu order, so this is the check that the port's app is at the
     * END (next to the user's emulator entries) rather than first. */
    int n_apps = td_app_count();
    int music_at = -1;
    char last[32] = "";

    for (int i = 0; i < n_apps; i++)
    {
        const td_app_t *a = td_app_get(i);

        if (a && strcmp(a->name, "Music") == 0)
            music_at = i;
        if (a && i == n_apps - 1)
            snprintf(last, sizeof(last), "%s", a->name);
    }
    fprintf(f, "apps=%d music_at=%d last_app=[%s]\n", n_apps, music_at, last);

    launch();
    fprintf(f, "window=%d tracks=%d\n", s_win != NULL, music_track_count());
    fflush(f);

    const int k_runs = 8;

    for (int i = 0; i < k_runs && i < music_track_count(); i++)
    {
        const size_t psram0 = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        const size_t int0 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        const size_t mod0_b = modhost_live_blocks();
        const size_t mod0_s = modhost_live_bytes();

        if (!music_play_index(i))
        {
            fprintf(f, "%d REFUSED TO START\n", i);
            fflush(f);
            break;
        }
        /* Long enough for STREAMINFO and a second of audio. */
        for (int t = 0; t < 25; t++)
            vTaskDelay(pdMS_TO_TICKS(100));

        music_status_t st;

        music_status_get(&st);
        fprintf(f, "%d play state=%d frames=%lu err=%lu msg=[%s] mod=%u/%u\n",
                i, st.state, (unsigned long)st.frames, (unsigned long)st.errors,
                st.message, (unsigned)modhost_live_blocks(),
                (unsigned)modhost_live_bytes());

        music_stop();
        for (int t = 0; t < 20; t++)
            vTaskDelay(pdMS_TO_TICKS(100));

        /* After the task has fully exited: this is where a leak shows. */
        fprintf(f, "%d done psram %d int %d mod %d/%d\n", i,
                (int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) - (int)psram0,
                (int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) - (int)int0,
                (int)modhost_live_blocks() - (int)mod0_b,
                (int)modhost_live_bytes() - (int)mod0_s);
        fflush(f);
    }

    fprintf(f, "final psram=%u int=%u\n",
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    fclose(f);
}

#endif /* MUSIC_UI_SELFTEST */
