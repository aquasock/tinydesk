/*
 * gps.c - the MIA-M10Q: its rail, its UART, and the probe that names it.
 *
 * See gps.h for the wiring. The parser is in nmea.c and owns no stream, so
 * this file is only the transport: open the rail, open the UART, send one UBX
 * poll, drain bytes, and hand complete lines to the parser.
 *
 * WHY UBX-MON-VER AND NOT "WAIT FOR NMEA". Any GNSS receiver produces NMEA, so
 * receiving sentences proves a receiver is wired up but says nothing about
 * which one. MON-VER returns a list of extension strings, and a u-blox module
 * puts its own model there as `OD=MIA-M10Q`. One eight-byte request therefore
 * answers both questions the first bring-up actually has.
 *
 * WHY THE READ IS NON-BLOCKING THROUGHOUT. The reader's interrupt taught this
 * project that a blocking read on hardware that might be absent is how a
 * command hangs the shell; the NFC presence check then proved it again by
 * blocking for ever on a lifted tag. Every read here is a bounded wait on a
 * UART that is polled, never a wait for a specific expected reply.
 */

#include "gps.h"
#include "power.h"
#include "rtc.h"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh_board.h"

#define TAG "twatch-gps"

/* Pins from the board's Arduino variant: GNSS TX 43, RX 44, PPS 13.
 *
 * The names are from the ESP32's point of view, which is the opposite of the
 * module's -- GPS_TX 43 is the pin the SoC transmits on. Getting this backwards
 * produces a port that is silent in both directions, which is why the probe
 * reports how many bytes arrived rather than only whether a reply parsed. */
#define GPS_TX_DEFAULT  43
#define GPS_RX_DEFAULT  44
#define GPS_PPS_DEFAULT 13
#define GPS_BAUD        38400
#define GPS_UART        UART_NUM_1

/* UBX-MON-VER: B5 62, class 0A, id 04, length 0, checksum 0E 34. The sync
 * bytes are not part of the checksum; the class, id and the two length bytes
 * are, which is what the 0E 34 is. */
static const uint8_t MON_VER[] = { 0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34 };

static bool s_open;
static int s_pps = -1;

/* The GPS app's task holds the receiver while its window is open. Declared
 * up here rather than with the rest of the app's state because cmd_gps()
 * below has to refuse while it is set, and this is the one thing the two
 * halves of the file have to agree on. */
static volatile bool s_app_open;

/* --- UBX framing --------------------------------------------------------- */

/* Fletcher checksum over a UBX payload, which is what the module uses. Written
 * out rather than assumed because a wrong checksum makes every reply look
 * corrupt, and the symptom -- "silent module" -- is the same as a dead one. */
static bool ubx_checksum_ok(const uint8_t *frame, int total)
{
    if (total < 8)
        return false;

    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < total - 2; i++)
    {
        ck_a = (uint8_t)(ck_a + frame[i]);
        ck_b = (uint8_t)(ck_b + ck_a);
    }
    return frame[total - 2] == ck_a && frame[total - 1] == ck_b;
}

/* Pull "sw", "hw" and the OD= extension out of a MON-VER payload.
 *
 * The payload is three fixed 30-byte strings -- software, hardware, then zero
 * or more extensions -- each NUL-padded. Reading them as fixed fields rather
 * than by splitting on NUL matters: a string that exactly fills its 30 bytes is
 * NOT NUL-terminated, and a parser that assumes it is walks one byte too far
 * into the next field. */
static void parse_mon_ver(const uint8_t *p, int len, gps_probe_t *out)
{
    if (len < 60)
        return;

    snprintf(out->sw, sizeof(out->sw), "%.*s", 30, (const char *)p);
    snprintf(out->hw, sizeof(out->hw), "%.*s", 30, (const char *)(p + 30));

    for (int at = 60; at + 30 <= len; at += 30)
    {
        const char *e = (const char *)(p + at);

        /* Extension strings are "KEY=VALUE"; the model is under OD=. */
        if (e[0] == 'O' && e[1] == 'D' && e[2] == '=')
        {
            snprintf(out->model, sizeof(out->model), "%.*s", 29, e + 3);
            /* Trailing NUL padding can leave whitespace or an early terminator;
             * trim anything that is not printable. */
            for (char *m = out->model; *m; m++)
                if (*m < 0x20 || *m > 0x7E)
                {
                    *m = '\0';
                    break;
                }
        }
    }
}

/* --- the probe ----------------------------------------------------------- */

bool gps_open(gps_probe_t *out)
{
    memset(out, 0, sizeof(*out));

    /* Pins come from the board configuration, so a crossed harness can be
     * corrected without a rebuild. */
    int tx = tdsh_board_int("gps.tx", GPS_TX_DEFAULT);
    int rx = tdsh_board_int("gps.rx", GPS_RX_DEFAULT);
    s_pps = tdsh_board_int("gps.pps", GPS_PPS_DEFAULT);

    if (tx < 0 || rx < 0)
    {
        ESP_LOGE(TAG, "gps pins are not configured");
        return false;
    }

    if (!board_gnss_rail_on())
    {
        ESP_LOGE(TAG, "BLDO1 did not come up");
        return false;
    }
    out->rail = true;

    /* The module needs time after its rail before it will answer: its own
     * oscillator has to start and its firmware to boot. 300 ms is from the
     * datasheet's typical start-up, and probing earlier returns silence that
     * looks like a wiring fault. */
    vTaskDelay(pdMS_TO_TICKS(300));

    uart_config_t cfg = {
        .baud_rate = GPS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(GPS_UART, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "uart%d install: %s", GPS_UART, esp_err_to_name(err));
        return false;
    }

    err = uart_param_config(GPS_UART, &cfg);
    if (err == ESP_OK)
        err = uart_set_pin(GPS_UART, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart%d config: %s", GPS_UART, esp_err_to_name(err));
        uart_driver_delete(GPS_UART);
        return false;
    }

    out->port = true;
    s_open = true;

    /* PPS as an input. Its level alone is not informative -- it is low between
     * pulses -- but a pin that cannot be read at all is, and the caller can
     * poll it while listening. */
    if (s_pps >= 0)
    {
        gpio_config_t in = {
            .pin_bit_mask = (1ULL << s_pps),
            .mode = GPIO_MODE_INPUT,
        };
        gpio_config(&in);
    }

    uart_flush_input(GPS_UART);
    uart_write_bytes(GPS_UART, (const char *)MON_VER, sizeof(MON_VER));

    /* Drain for up to a second. Anything the module says is collected: a
     * NMEA stream first (many ship with it enabled) and the MON-VER reply
     * somewhere in it. */
    uint8_t frame[128];
    int have = 0;
    int64_t end = esp_timer_get_time() + 1000000;

    while (esp_timer_get_time() < end)
    {
        uint8_t b;
        int n = uart_read_bytes(GPS_UART, &b, 1, pdMS_TO_TICKS(20));

        if (n != 1)
        {
            /* Nothing for 20 ms. Keep waiting inside the window: a module in
             * its first second emits in bursts. */
            continue;
        }

        out->heard_any = true;
        out->bytes++;

        if (have < (int)sizeof(frame))
            frame[have++] = b;

        /* Look for a complete UBX frame once the sync and header are in. */
        if (have >= 8 && frame[0] == 0xB5 && frame[1] == 0x62)
        {
            int len = frame[4] | (frame[5] << 8);
            int total = len + 8;

            if (total <= (int)sizeof(frame) && have >= total)
            {
                if (frame[2] == 0x0A && frame[3] == 0x04 && ubx_checksum_ok(frame, total))
                {
                    parse_mon_ver(frame + 6, len, out);
                    out->ubx_reply = true;
                    break;
                }
                have = 0;   /* not ours: start over */
            }
        }

        /* If the buffer fills without a UBX frame, the port is speaking
         * something else -- NMEA. That is not a failure, and the byte count
         * already records it. */
        if (have >= (int)sizeof(frame))
            have = 0;
    }

    if (out->ubx_reply)
        ESP_LOGI(TAG, "gnss %s %s", out->model[0] ? out->model : "?", out->sw);
    else if (out->heard_any)
        ESP_LOGW(TAG, "gnss spoke %d bytes, no MON-VER", out->bytes);
    else
        ESP_LOGW(TAG, "gnss silent on %d/%d", tx, rx);

    return true;
}

int gps_pump(nmea_state_t *nmea)
{
    if (!s_open)
        return 0;

    /* A line buffer that survives between calls: a sentence can straddle two
     * reads, and dropping the partial one loses whichever sentence was in
     * flight -- about one in seven at 1 Hz, which shows up as a fix that
     * takes longer to appear than it should. */
    static char line[128];
    static int used;

    uint8_t chunk[64];
    int n = uart_read_bytes(GPS_UART, chunk, sizeof(chunk), 0);

    for (int i = 0; i < n; i++)
    {
        char c = (char)chunk[i];

        if (c == '\n')
        {
            line[used] = '\0';
            if (used > 0)
                nmea_line(nmea, line);
            used = 0;
            continue;
        }
        if (c == '\r')
            continue;

        /* A '$' mid-line means the previous line was truncated: start over
         * rather than gluing two sentences together. */
        if (c == '$' && used > 0)
            used = 0;

        if (used < (int)sizeof(line) - 1)
            line[used++] = c;
        else
            used = 0;
    }

    return n;
}

/* Dump raw sentences for a while.
 *
 * This exists because "no fix" and "no antenna" are the same reading on a fix
 * display, and they are not the same problem. The GSV sentences carry the
 * satellites the receiver can HEAR: a module with a working antenna usually
 * shows several satellites at low C/N0 even indoors, while one with a
 * disconnected antenna shows none at all.
 */
void gps_raw_dump(int ms, void (*sink)(const char *line, void *ctx), void *ctx)
{
    if (!s_open)
        return;

    static char line[128];
    static int used;
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;

    while (esp_timer_get_time() < end)
    {
        uint8_t chunk[64];
        int n = uart_read_bytes(GPS_UART, chunk, sizeof(chunk), pdMS_TO_TICKS(20));

        for (int i = 0; i < n; i++)
        {
            char c = (char)chunk[i];

            if (c == '\n')
            {
                line[used] = '\0';
                if (used > 0 && sink)
                    sink(line, ctx);
                used = 0;
                continue;
            }
            if (c == '\r')
                continue;
            if (c == '$' && used > 0)
                used = 0;
            if (used < (int)sizeof(line) - 1)
                line[used++] = c;
            else
                used = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void gps_listen(nmea_state_t *nmea, int ms, void (*tick)(const nmea_state_t *, void *),
                void *ctx, int tick_ms)
{
    if (!s_open)
        return;

    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    int64_t next_tick = 0;

    while (esp_timer_get_time() < end)
    {
        gps_pump(nmea);

        int64_t now = esp_timer_get_time();
        if (tick && now >= next_tick)
        {
            tick(nmea, ctx);
            next_tick = now + (int64_t)tick_ms * 1000;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

int gps_pps_level(void)
{
    if (!s_open || s_pps < 0)
        return -1;
    return gpio_get_level((gpio_num_t)s_pps);
}

void gps_close(void)
{
    if (!s_open)
        return;

    uart_driver_delete(GPS_UART);
    s_open = false;
}

/* --- the shell command ----------------------------------------------------
 *
 * `gps` and `gps monitor <s>`. In this file rather than in link_twatch.c for
 * the same reason the nfc command lives in nfc.c: a driver that can be probed
 * from the shell is a driver that can be probed again without a rebuild, and
 * keeping the two together means the pin names in the report and the pin names
 * in the code cannot drift.
 */

#include <stdlib.h>

#include "tdsh.h"

static void print_probe(const gps_probe_t *p)
{
    printf("rail  %s\n", p->rail ? "up" : "FAILED");
    printf("port  %s\n", p->port ? "38400 8N1" : "FAILED");
    printf("ubx   %s\n", p->ubx_reply ? "MON-VER ok" : "no reply");
    printf("heard %d bytes%s\n", p->bytes,
           p->heard_any ? "" : " (SILENT)");

    if (p->model[0])
        printf("model %s\n", p->model);
    if (p->sw[0])
        printf("sw    %s\n", p->sw);
    if (p->hw[0])
        printf("hw    %s\n", p->hw);
}

/* One line per tick, short enough for forty columns. */
static void report(const nmea_state_t *n, void *ctx)
{
    int *pps_seen = (int *)ctx;

    int pps = gps_pps_level();
    if (pps > 0 && pps_seen)
        (*pps_seen)++;

    printf("%02d:%02d:%02d.%02d", n->have_time ? n->hour : 0,
           n->have_time ? n->minute : 0,
           n->have_time ? n->second_x100 / 100 : 0,
           n->have_time ? n->second_x100 % 100 : 0);

    if (n->fix_quality > 0)
        printf(" FIX%d", n->fix_quality);
    else
        printf(" nofix");

    printf(" s%02d", n->sats_used);

    if (n->hdop_x100)
        printf(" hd%d.%02d", n->hdop_x100 / 100, n->hdop_x100 % 100);

    if (n->have_pos)
        printf(" %.5f %.5f", n->lat, n->lon);

    if (n->have_alt)
        printf(" %ldm", (long)(n->alt_cm / 100));

    printf(" c%d\n", n->sentences);
}

/* Which sentences arrive, by type, plus whatever GSV says about satellites.
 *
 * The satellite number is the one that matters when there is no fix: it is the
 * receiver's own count of what its antenna can hear. */
static void raw_sink(const char *line, void *ctx)
{
    int *counts = (int *)ctx;   /* indexed by a small table below */
    static const char *const NAME[] = {"GGA", "RMC", "GSA", "GSV", "GLL", "VTG", "TXT", "other"};
    static const int NAME_N = 8;

    const char *type = "other";
    int slot = NAME_N - 1;

    if (line[0] == '$' && strlen(line) > 5)
    {
        for (int i = 0; i < NAME_N - 1; i++)
            if (strncmp(line + 3, NAME[i], 3) == 0)
            {
                type = NAME[i];
                slot = i;
                break;
            }
    }
    (void)type;

    counts[slot]++;

    /* GSV lines look like $GNGSA,... for the fix in use and
     * $GPGSV,<n>,<m>,<sats>,... for the sky view. The fourth field is the
     * number of satellites visible to that constellation. */
    if (slot == 3)
    {
        const char *p = line;
        int comma = 0;
        while (*p && comma < 3)
        {
            if (*p == ',')
                comma++;
            p++;
        }
        if (comma == 3)
        {
            int vis = atoi(p);
            if (vis > counts[8])
                counts[8] = vis;   /* slot 8: best visible count seen */
        }
    }
}

static int cmd_gps(tdsh_session_t *session, int argc, char **argv)
{
    (void)session;

    /* The app's window holds the receiver while it is open, and two parsers on
     * one UART each see half the sentences -- a receiver that reads as faulty
     * from both sides at once. Say so instead of half-working. */
    if (s_app_open)
    {
        printf("the GPS app has the receiver open; close its window first\n");
        return 1;
    }

    const bool raw = (argc >= 2 && strcmp(argv[1], "raw") == 0);
    const bool settime = (argc >= 2 && strcmp(argv[1], "time") == 0);

    if (argc >= 2 && !raw && !settime &&
        strcmp(argv[1], "monitor") != 0 && strcmp(argv[1], "mon") != 0)
    {
        printf("usage: gps | gps monitor [s] | gps raw [s] | gps time [s]\n");
        return 2;
    }

    gps_probe_t p;
    if (!gps_open(&p))
    {
        printf("gps rail or port FAILED\n");
        return 1;
    }

    print_probe(&p);

    if (settime)
    {
        /* A receiver indoors acquires the time long before it can fix, so the
         * default window is generous but bounded: thirty seconds covers
         * ephemeris decode at a window, and the command always returns. */
        int secs = argc >= 3 ? atoi(argv[2]) : 30;
        if (secs <= 0)
            secs = 30;

        nmea_state_t n;
        nmea_init(&n);

        printf("waiting for UTC, %d s\n", secs);

        /* THE TIME IS APPLIED THE MOMENT IT IS VALID, not at the end of the
         * window, and that is a correction rather than a nicety.
         *
         * The first version collected the UTC from the first valid sentence and
         * then kept listening until the window expired before calling
         * settimeofday -- so it set the clock to a value that was already up to
         * the whole window old. Measured on this board: correction runs a
         * minute apart reported the clock `was off +26 s` and then `+1 s`,
         * which reads like drift and is really the setting routine handing the
         * clock a stale reading. A time source that is right and is applied
         * late is still a wrong clock.
         *
         * So the loop watches for a valid time and stops the moment it has one.
         * The window is now only a bound on how long to wait, never a delay
         * after the answer has arrived. */
        int64_t utc = 0;
        int64_t end = esp_timer_get_time() + (int64_t)secs * 1000000;

        while (esp_timer_get_time() < end)
        {
            gps_pump(&n);

            int64_t v;
            if (nmea_utc(&n, &v))
            {
                utc = v;
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }

        if (!utc)
        {
            printf("no UTC -- %d sats, %u sentences\n",
                   n.sats_used, n.sentences);
            gps_close();
            return 1;
        }

        /* Read the clock BEFORE changing it: this is the only measurement of
         * how wrong it was, and it is taken as close to the write as possible
         * so it is not contaminated by anything this command does afterwards. */
        int64_t before = 0;
        struct timeval tv0;
        if (gettimeofday(&tv0, NULL) == 0)
            before = (int64_t)tv0.tv_sec;

        if (!board_time_apply(utc))
        {
            printf("clock refused %lld\n", (long long)utc);
            gps_close();
            return 1;
        }

        printf("clock set %lld\n", (long long)utc);
        printf("was off %+lld s, sats %d\n", (long long)(utc - before), n.sats_used);

        gps_close();
        return 0;
    }

    if (raw)
    {
        int secs = argc >= 3 ? atoi(argv[2]) : 5;
        if (secs <= 0)
            secs = 5;

        int counts[9] = {0};

        printf("raw %d s\n", secs);
        gps_raw_dump(secs * 1000, raw_sink, counts);

        printf("GGA %d RMC %d GSA %d GSV %d\n", counts[0], counts[1], counts[2], counts[3]);
        printf("GLL %d VTG %d TXT %d oth %d\n", counts[4], counts[5], counts[6], counts[7]);
        printf("sats visible %d\n", counts[8]);

        gps_close();
        return 0;
    }

    if (argc >= 2)
    {
        int secs = argc >= 3 ? atoi(argv[2]) : 60;
        if (secs <= 0)
            secs = 60;

        printf("listening %d s\n", secs);

        nmea_state_t n;
        nmea_init(&n);
        int pps_seen = 0;

        gps_listen(&n, secs * 1000, report, &pps_seen, 1000);

        printf("%u ok %u bad %u other, %d pps\n",
               n.sentences - n.failed_checksum - n.ignored,
               n.failed_checksum, n.ignored, pps_seen);
    }

    gps_close();
    return 0;
}

static const tdsh_command_t s_gps_cmd = {
    "gps",
    "gps | gps monitor [s] | gps raw [s] | gps time [s]",
    "Probe the MIA-M10Q GNSS, stream its fix, count sentence types, or set the clock",
    cmd_gps,
    TDSH_CMD_INTERACTIVE,
};

bool gps_register_command(void)
{
    return tdsh_register_command(&s_gps_cmd) == 0;
}

/* --- the GPS app ----------------------------------------------------------
 *
 * The app opens the receiver when its window opens and drops it when the window
 * closes. That is a deliberate power decision and it is the reason the ops type
 * has start() and stop() at all: a GNSS module draws tens of milliamps, and a
 * watch has none to spare for a window nobody is looking at. The cost is that
 * the first reading after opening is empty and a cold fix takes minutes, so the
 * window shows what it learns immediately -- satellites in view appear within a
 * second or two, long before a fix is possible.
 *
 * ONE READER, ONE PARSER. The shell's `gps` command parses this same byte stream
 * from its own nmea_state_t, so two readers would split the sentences between
 * them and each would see half of a receiver that looks broken. Whichever side
 * opens first owns the port and the other is told so rather than being allowed
 * to half-work.
 */
#define GPS_APP_POLL_MS 100
#define GPS_APP_STACK   4096

static volatile bool s_app_run;
static TaskHandle_t s_app_task;
static nmea_state_t s_app_nmea;          /* owned by the pump task alone */
static td_gps_t s_app_snapshot;          /* published for the app */
static portMUX_TYPE s_app_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_app_model[24];

/* Double to 1e-7 degrees, rounded away from zero. Written out rather than using
 * lround() so this file needs no <math.h>, and rounded rather than truncated
 * because truncation is a systematic bias of up to 1.1 cm in one direction. */
static int32_t to_e7(double v)
{
    const double s = v * 1e7;

    return (int32_t)(s < 0 ? s - 0.5 : s + 0.5);
}

/* Copy the parser's state into the published snapshot. Called from the pump
 * task; the copy into s_app_snapshot happens under the lock so a reader never
 * sees a half-updated struct. */
static void gps_app_publish(void)
{
    td_gps_t t;

    memset(&t, 0, sizeof(t));

    t.receiver_open = true;
    snprintf(t.model, sizeof(t.model), "%s", s_app_model);

    t.time_valid = s_app_nmea.have_time;
    t.hour = s_app_nmea.hour;
    t.minute = s_app_nmea.minute;
    t.second = s_app_nmea.second_x100 / 100;

    t.date_valid = s_app_nmea.have_date;
    t.year = s_app_nmea.year;
    t.month = s_app_nmea.month;
    t.day = s_app_nmea.day;

    t.fix = s_app_nmea.fix_quality;
    t.sats_used = s_app_nmea.sats_used;
    t.sats_in_view = s_app_nmea.sats_in_view;

    /* 99.99 is the receiver's "no HDOP" sentinel, and a dilution of precision of
     * a hundred is not a precision -- it is an absence. Shown as a number it
     * would read as a terrible measurement rather than as no measurement. */
    if (s_app_nmea.hdop_x100 > 0 && s_app_nmea.hdop_x100 < 9999)
    {
        t.hdop_valid = true;
        t.hdop_x100 = s_app_nmea.hdop_x100;
    }

    if (s_app_nmea.have_pos)
    {
        t.have_pos = true;
        t.lat_e7 = to_e7(s_app_nmea.lat);
        t.lon_e7 = to_e7(s_app_nmea.lon);
    }
    if (s_app_nmea.have_alt)
    {
        t.have_alt = true;
        t.alt_cm = s_app_nmea.alt_cm;
    }
    if (s_app_nmea.have_speed)
    {
        t.have_speed = true;
        t.speed_mm_s = s_app_nmea.speed_mm_s;
    }
    if (s_app_nmea.have_course)
    {
        t.have_course = true;
        t.course_mdeg = s_app_nmea.course_mdeg;
    }

    t.sentences = s_app_nmea.sentences;
    t.checksum_errors = s_app_nmea.failed_checksum;

    taskENTER_CRITICAL(&s_app_lock);
    s_app_snapshot = t;
    taskEXIT_CRITICAL(&s_app_lock);
}

static void gps_app_task(void *arg)
{
    (void)arg;

    while (s_app_run)
    {
        gps_pump(&s_app_nmea);
        gps_app_publish();
        /* 100 ms, which is a twentieth of the sentence rate: the parser is fed
         * faster than the receiver speaks, so no sentence waits, and the task
         * sleeps five sixths of its life. */
        vTaskDelay(pdMS_TO_TICKS(GPS_APP_POLL_MS));
    }

    s_app_task = NULL;
    vTaskDelete(NULL);
}

static bool gps_app_start(const char **why)
{
    if (s_app_open)
        return true;                     /* reopening an open window is not an error */

    if (s_open)
    {
        /* The shell command has it. Two parsers on one UART each see half the
         * sentences and both look faulty. */
        if (why)
            *why = "in use by the gps command";
        return false;
    }

    gps_probe_t p;

    if (!gps_open(&p))
    {
        if (why)
            *why = "rail or UART would not start";
        return false;
    }

    snprintf(s_app_model, sizeof(s_app_model), "%s", p.model);
    nmea_init(&s_app_nmea);

    taskENTER_CRITICAL(&s_app_lock);
    memset(&s_app_snapshot, 0, sizeof(s_app_snapshot));
    taskEXIT_CRITICAL(&s_app_lock);

    s_app_run = true;
    s_app_open = true;

    /* 4096, and the number is reasoned rather than guessed at: this task's
     * deepest path is gps_open() -> uart_driver_install(), and gps_open() has
     * already returned by the time the task runs, so the task itself only holds
     * a td_gps_t and the parser's statics. 4096 is comfortably above that, and
     * the previous cycle's lesson is that a stack sized to "it does almost
     * nothing" is how a board comes to reboot every thirty-three seconds. */
    if (xTaskCreate(gps_app_task, "gps", GPS_APP_STACK, NULL, 2, &s_app_task) != pdPASS)
    {
        s_app_run = false;
        s_app_open = false;
        gps_close();
        if (why)
            *why = "no task";
        return false;
    }

    return true;
}

static void gps_app_stop(void)
{
    if (!s_app_open)
        return;

    s_app_run = false;

    /* Wait for the task to notice and delete itself rather than deleting it from
     * under a uart_read_bytes(). Five hundred milliseconds is fifty poll
     * intervals, so this only bites if the task is wedged -- and if it is, the
     * delete below is the right answer anyway. */
    for (int i = 0; i < 50 && s_app_task; i++)
        vTaskDelay(pdMS_TO_TICKS(10));

    if (s_app_task)
    {
        vTaskDelete(s_app_task);
        s_app_task = NULL;
    }

    s_app_open = false;
    gps_close();
}

static bool gps_app_status(td_gps_t *out)
{
    if (!out || !s_app_open)
        return false;

    taskENTER_CRITICAL(&s_app_lock);
    *out = s_app_snapshot;
    taskEXIT_CRITICAL(&s_app_lock);

    return true;
}

static const td_gps_ops_t s_gps_ops = {
    .start = gps_app_start,
    .stop = gps_app_stop,
    .status = gps_app_status,
};

/* Installed into td_sysinfo_t by main.c through the weak board_gps_ops(). */
const td_gps_ops_t *gps_sysinfo_ops(void)
{
    return &s_gps_ops;
}

const td_gps_ops_t *board_gps_ops(void)
{
    return &s_gps_ops;
}
