/*
 * sysmon.c - live heap, tasks, CPU (and its temperature where the chip has
 * a sensor), uptime and tinydesk's own statistics.
 * The bars show memory in use; internal RAM and PSRAM (if any) apart.
 */
#include <stdio.h>

#include "td_apps.h"

static td_window_t *s_win;
static td_widget_t *s_free_bar, *s_free_txt, *s_min_bar, *s_min_txt, *s_ps_bar, *s_ps_txt;
static td_widget_t *s_tasks, *s_uptime, *s_frame, *s_serial, *s_term;
/* The chip's own temperature sensor, on its own row when the board has one.
 * It used to share the Tasks/CPU line and was cut off at the window's edge --
 * "Tasks: 18  CPU: 240 MHz  Chip" with no number after it, which is worse than
 * not showing it, because it looks like a reading of nothing. */
static td_widget_t *s_temp;
/* One line for the battery, or NULL on a board without one. Above the memory
 * bars rather than below the statistics: power left is the thing a watch user
 * looks for first, and on a 26-row screen the top is where the eye lands. */
static td_widget_t *s_batt;

static int percent(uint32_t part, uint32_t total)
{
    return total ? (int)((uint64_t)part * 100u / total) : 0;
}

static void update(void)
{
    const td_sysinfo_t *si = td_sysinfo();
    const td_stats_t *st = td_stats();
    uint32_t total = si->total_heap ? si->total_heap() : 0;

    if (si->free_heap && total)
    {
        uint32_t f = si->free_heap();
        td_progress_set(s_free_bar, percent(total - f, total));
        td_widget_printf(s_free_txt, "%u KB free of %u", (unsigned)(f / 1024u), (unsigned)(total / 1024u));
    }
    else
    {
        td_widget_printf(s_free_txt, "n/a");
    }
    if (si->min_free_heap && total)
    {
        uint32_t m = si->min_free_heap();
        td_progress_set(s_min_bar, percent(total - m, total));
        td_widget_printf(s_min_txt, "%u KB free at least", (unsigned)(m / 1024u));
    }
    else
    {
        td_widget_printf(s_min_txt, "n/a");
    }
    if (s_ps_bar)
    {
        uint32_t pt = si->psram_total(), pf = si->psram_free();
        td_progress_set(s_ps_bar, percent(pt - pf, pt));
        td_widget_printf(s_ps_txt, "%u KB free of %u", (unsigned)(pf / 1024u), (unsigned)(pt / 1024u));
    }

    if (s_batt)
    {
        td_battery_t b[TD_MAX_BATTERIES];

        int n = si->batteries ? si->batteries(b, TD_MAX_BATTERIES) : 0;

        if (n <= 0)
            td_widget_printf(s_batt, "Battery: n/a");
        else
        {
            /* Voltage to two decimals in volts, the way a battery is quoted;
             * a whole-millivolt figure wastes three of 38 columns. */
            char pct[12] = "  --%";
            const char *state = "";

            /* Clamped, not trusted. A driver that hands back 400 would
             * otherwise overflow the buffer, and "400%" is not a percentage
             * either -- so out-of-range is treated as unknown, the same way a
             * gauge that has not settled is. */
            if (b[0].percent >= 0 && b[0].percent <= 100)
                snprintf(pct, sizeof(pct), "%3d%%", b[0].percent);
            switch (b[0].state)
            {
            case TD_BATT_CHARGING: state = " chg"; break;
            case TD_BATT_FULL: state = " full"; break;
            case TD_BATT_DISCHARGING: state = " dis"; break;
            default: break;
            }
            /* The cell's thermistor rides on the battery line because it is the
             * same subject and a 26-row screen cannot spare a row for eight
             * characters. `ntc511` is seven columns; the whole line is 32 of
             * the 38 the window shows at 40x26. MILLIVOLTS, NOT DEGREES -- see
             * td_battery_t and PMU-006. */
            char ntc[16] = "";

            if (b[0].ntc_millivolts > 0)
                snprintf(ntc, sizeof(ntc), " ntc%d", b[0].ntc_millivolts);

            if (b[0].millivolts > 0)
                td_widget_printf(s_batt, "Battery:%s  %d.%02d V%s%s", pct,
                                 b[0].millivolts / 1000, (b[0].millivolts % 1000) / 10, state, ntc);
            else
                td_widget_printf(s_batt, "Battery:%s%s%s", pct, state, ntc);
        }
    }

    char tasks[16] = "n/a", cpu[16] = "n/a";
    if (si->task_count)
        snprintf(tasks, sizeof(tasks), "%d", si->task_count());
    if (si->cpu_mhz)
        snprintf(cpu, sizeof(cpu), "%d MHz", si->cpu_mhz());
    td_widget_printf(s_tasks, "Tasks: %-8s  CPU: %s", tasks, cpu);
    if (s_temp)
    {
        int t;
        if (si->chip_temp && si->chip_temp(&t))
            td_widget_printf(s_temp, "Chip: %s%d.%d °C", t < 0 ? "-" : "",
                             (t < 0 ? -t : t) / 10, (t < 0 ? -t : t) % 10);
        else
            td_widget_printf(s_temp, "Chip: n/a");
    }

    uint32_t s = td_uptime_ms() / 1000u;
    td_widget_printf(s_uptime, "Uptime: %02u:%02u:%02u",
                     (unsigned)(s / 3600u), (unsigned)(s / 60u % 60u), (unsigned)(s % 60u));
    td_widget_printf(s_frame, "Frame: %u ms  Loop: %u ms  Frames: %u",
                     (unsigned)st->frame_ms, (unsigned)st->loop_ms, (unsigned)st->frames);
    td_widget_printf(s_serial, "Serial out: %u B/s  total %u KB",
                     (unsigned)st->bytes_per_sec, (unsigned)(st->bytes_sent / 1024u));
    if (st->term_cols > st->cols || st->term_rows > st->rows)
        td_widget_printf(s_term, "Screen: %dx%d of %dx%d (max)  dropped: %u", st->cols, st->rows, st->term_cols,
                         st->term_rows, (unsigned)st->dropped_frames);
    else
        td_widget_printf(s_term, "Terminal: %dx%d  dropped frames: %u", st->cols, st->rows,
                         (unsigned)st->dropped_frames);
}

static void on_tick(td_window_t *win)
{
    (void)win;
    update();
}

static void on_close(td_window_t *win)
{
    (void)win;
    s_win = NULL;
    /* The widgets go with the window; a stale pointer would be written to on
     * the next tick if one arrived before launch() ran again. */
    s_batt = NULL;
    s_free_bar = s_free_txt = s_min_bar = s_min_txt = NULL;
    s_ps_bar = s_ps_txt = NULL;
    s_tasks = s_uptime = s_frame = s_serial = s_term = NULL;
    s_temp = NULL;
}

static void launch(void)
{
    if (td_win_is_open(s_win))
    {
        td_win_focus(s_win);
        return;
    }
    const td_sysinfo_t *si = td_sysinfo();
    bool psram = si->psram_total && si->psram_total() > 0;
    /* Somewhere above the memory bars if the board reports one. Probing here
     * rather than in update() because the layout, not the reading, decides
     * whether the row exists. */
    td_battery_t probe[TD_MAX_BATTERIES];
    bool batt = si->batteries && si->batteries(probe, TD_MAX_BATTERIES) > 0;
    /* Same question for the temperature: ask once here, because it decides the
     * layout rather than the reading. A sensor that fails to start would
     * otherwise leave an empty row on a 26-row screen. */
    int probe_t = 0;
    bool temp = si->chip_temp && si->chip_temp(&probe_t);
    int top = batt ? 1 : 0;        /* everything shifts down for the battery */
    int y = top + (psram ? 1 : 0); /* rows below the memory bars move down */
    td_window_desc_t d = {
        .title = "System Monitor",
        .rect = td_rect(4, 2, 56, 12 + y + (temp ? 1 : 0)),
        .flags = TD_WIN_MOVABLE | TD_WIN_CLOSABLE,
        .on_close = on_close,
        .on_tick = on_tick,
        .tick_ms = 500,
    };
    s_win = td_win_create(&d);
    if (!s_win)
        return;

    s_batt = batt ? td_label(s_win, 1, 1, 0, "") : NULL;
    td_label(s_win, 1, 1 + top, 13, psram ? "Internal RAM" : "RAM in use");
    s_free_bar = td_progress(s_win, 14, 1 + top, 18);
    s_free_txt = td_label(s_win, 33, 1 + top, 0, "");
    td_label(s_win, 1, 2 + top, 13, "  peak use");
    s_min_bar = td_progress(s_win, 14, 2 + top, 18);
    s_min_txt = td_label(s_win, 33, 2 + top, 0, "");
    s_ps_bar = s_ps_txt = NULL;
    if (psram)
    {
        td_label(s_win, 1, 3 + top, 13, "PSRAM");
        s_ps_bar = td_progress(s_win, 14, 3 + top, 18);
        s_ps_txt = td_label(s_win, 33, 3 + top, 0, "");
    }
    s_tasks = td_label(s_win, 1, 4 + y, 0, "");
    s_temp = temp ? td_label(s_win, 1, 5 + y, 0, "") : NULL;
    int r = 5 + y + (temp ? 1 : 0);
    s_uptime = td_label(s_win, 1, r++, 0, "");
    s_frame = td_label(s_win, 1, r++, 0, "");
    s_serial = td_label(s_win, 1, r++, 0, "");
    s_term = td_label(s_win, 1, r++, 0, "");
    update();
}

static const td_app_t s_app = {"System Monitor", launch, "▄█"};

void td_sysmon_register(void)
{
    td_app_register(&s_app);
}
