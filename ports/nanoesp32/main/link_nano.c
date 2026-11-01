/*
 * link_nano.c - the Arduino Nano ESP32's console link, with two sinks.
 *
 * USB Serial/JTAG is the primary: when a host terminal is attached the
 * console is served there and the panel is blanked. With nothing attached the
 * console renders on the ILI9340 panel through vt.c at its 40x15 grid.
 *
 * The choice is re-checked on every write. A shell prompt writes at least
 * once a second while someone is typing, and the console asks the terminal
 * for its size periodically, so the switch is noticed promptly without a
 * timer task.
 *
 * How the sink is chosen, per side:
 *
 *   write   usb_serial_jtag_is_connected() decides, per call.
 *   read    USB only. The panel HAT on this bench has no touchscreen, keypad
 *           or buttons, so with no host attached there is nothing to type
 *           into: the console keeps rendering and the board keeps working,
 *           but it is read-only until USB comes back.
 *   size    The console asks with ESC[6n. Over USB the host terminal answers
 *           for itself, through link_read. On the panel there is no terminal,
 *           so the query is intercepted here and answered with 40x15, which
 *           the console then uses as its line width.
 *
 * Nothing here knows about the desktop: the window manager, the apps and the
 * screen buffers are not built into this image at all.
 */
#include "link.h"

#include <stdio.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "rs485.h"
#include "font8x16.h"
#include "ili9340.h"
#include "vt.h"

/* Sized for a shell, not for a desktop frame: a prompt, a typed line and a
 * screenful of scrolling text. The old desktop build needed tens of KB here
 * for whole frames, and 64 KB was what pushed WiFi off the board. */
#define RX_BUFFER        2048
#define TX_BUFFER        8192
#define WRITE_CHUNK      256
#define WRITE_TIMEOUT_MS 50
#define REPLY_MAX        32
/* How long one blocking read waits before returning to check for a sink
 * change. Short enough that a switch is quick, long enough not to spin. */
#define READ_WAIT_MS     100

static const char *TAG = "link_nano";

static char s_reply[REPLY_MAX];
static int s_reply_len, s_reply_pos;

/* Keys a long-running command's Ctrl+C check took out of the USB FIFO that
 * were not Ctrl+C. They are handed to the shell on its next read, so typing
 * during a command is not lost. Small on purpose: typing during a *running*
 * command is rare, and a bigger buffer would only hide input the user is
 * waiting to see. */
static char s_typed[64];
static size_t s_typed_len;
static bool s_panel_ready;
static bool s_on_usb;

/* True once output has gone to USB since the host attached. Used to keep boot
 * logging off the console until the console itself starts writing. */
static bool s_usb_has_console;

/* The host terminal's width, learned by asking it once. Zero means not yet
 * known. The shell's line editor cannot ask for itself: it decides whether it
 * may by testing fileno(stdin), and a funopen() stream reports -1, so it falls
 * back to 80 columns and every wrapped or padded line is wrong. It therefore
 * asks us instead, through tdsh_espidf_set_console_columns(). */
static int s_usb_columns;
/* Set once the query has been tried for this attachment, whatever the result:
 * a terminal that cannot answer must cost one wait, not one per prompt. */
static bool s_usb_columns_tried;

bool link_usb_is_console(void);
int link_console_columns(void);

/* Parse one ESC [ <row> ; <col> R report out of buf and return <col>, or 0.
 * The terminal clamps the cursor at its right edge, so after ESC[999C the
 * reported column is the terminal's width. */
static int parse_report(const char *buf, int len)
{
    for (int i = 0; i + 3 < len; ++i)
    {
        if (buf[i] != '\x1b' || buf[i + 1] != '[')
            continue;
        int j = i + 2;
        while (j < len && buf[j] >= '0' && buf[j] <= '9')
            j++;
        if (j >= len || buf[j] != ';')
            continue;
        j++;
        int cols = 0, digits = 0;
        while (j < len && buf[j] >= '0' && buf[j] <= '9')
        {
            cols = cols * 10 + (buf[j++] - '0');
            digits++;
        }
        if (digits > 0 && j < len && buf[j] == 'R')
            return cols;
    }
    return 0;
}

/* Ask the terminal for its width.
 *
 * ESC[999C parks the cursor at the terminal's right edge and ESC[6n reports
 * where it is. That park is destructive: the cursor is left at the right
 * margin, so the next prompt printed after it starts at the end of the line
 * and wraps, which is exactly the ragged layout this is here to fix. The
 * cursor is therefore restored with a CR whatever happens -- reply, no reply,
 * or a reply that arrives too late to read.
 *
 * A late reply is not harmless either: those bytes stay in the USB FIFO and
 * would be read as typed input, and ESC[20;80R typed into a line editor is
 * cursor movement. link_read() strips any report it sees for that reason, and
 * learns the width from it if we still do not know it. */
static void usb_learn_columns(void)
{
    static const uint8_t ask[] = "\x1b[999C\x1b[6n";
    static const uint8_t home[] = "\r";
    if (!usb_serial_jtag_is_connected())
        return;

    if (usb_serial_jtag_write_bytes(ask, sizeof(ask) - 1,
                                    pdMS_TO_TICKS(50)) != (int)(sizeof(ask) - 1))
        return;

    char buf[64];
    int len = 0;
    const uint32_t start = xTaskGetTickCount();
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(300) &&
           len < (int)sizeof(buf))
    {
        int n = usb_serial_jtag_read_bytes((uint8_t *)buf + len,
                                           (uint32_t)(sizeof(buf) - (size_t)len),
                                           pdMS_TO_TICKS(20));
        if (n <= 0)
            continue;
        len += n;
        if (parse_report(buf, len) > 0)
            break;
    }

    /* Undo the park before anything else is printed. */
    usb_serial_jtag_write_bytes(home, 1, pdMS_TO_TICKS(50));

    const int cols = parse_report(buf, len);
    if (cols > 0)
    {
        s_usb_columns = cols;
        ESP_LOGI(TAG, "host terminal reports %d columns", cols);
    }
    else
    {
        ESP_LOGW(TAG, "host terminal did not report a width; assuming 80");
    }
}

int link_console_columns(void)
{
    if (!s_on_usb)
        return s_panel_ready ? vt_cols() : 0;
    if (s_usb_columns <= 0 && !s_usb_columns_tried)
    {
        s_usb_columns_tried = true;
        usb_learn_columns();
    }
    return s_usb_columns > 0 ? s_usb_columns : 0;
}

static void panel_blank(void)
{
    if (s_panel_ready)
        ili9340_fill(0x0000);
}

/* The panel keeps its own copy of the screen, so after a spell on USB it holds
 * stale text. Resetting it means the console's next size query answers 40x15
 * and the shell repaints into a clean grid. */
static void on_sink_changed(bool on_usb)
{
    s_on_usb = on_usb;
    s_reply_len = s_reply_pos = 0;
    s_usb_columns = 0;          /* re-ask the new terminal */
    s_usb_columns_tried = false;
    if (on_usb)
    {
        panel_blank();
        ESP_LOGI(TAG, "host attached: console on USB, panel off");
    }
    else
    {
        if (s_panel_ready)
        {
            vt_init(LCD_W / FONT_W, LCD_H / FONT_H);
            ili9340_fill(0x0000);
        }
        ESP_LOGI(TAG, "host gone: console on the panel");
    }
}

bool link_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = RX_BUFFER,
        .tx_buffer_size = TX_BUFFER,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "usb install: %s", esp_err_to_name(err));
        return false;
    }

    /* The panel is always initialised: it is the fallback sink, and it is what
     * the board shows when a cable is pulled. A missing panel is not fatal
     * because USB may still be the sink. */
    s_panel_ready = ili9340_init();
    if (s_panel_ready)
    {
        ili9340_fill(0x0000);
        vt_init(LCD_W / FONT_W, LCD_H / FONT_H);
        ESP_LOGI(TAG, "panel %dx%d px, %dx%d cells",
                 LCD_W, LCD_H, vt_cols(), vt_rows());
    }
    else
    {
        ESP_LOGE(TAG, "panel init failed; USB only");
    }

    s_on_usb = usb_serial_jtag_is_connected();
    ESP_LOGI(TAG, "console starts on %s", s_on_usb ? "USB" : "the panel");
    if (s_on_usb)
        panel_blank();
    return true;
}

bool link_usb_is_console(void)
{
    return s_on_usb && s_usb_has_console;
}

/* Strip any cursor-position report from input.
 *
 * A terminal answers ESC[6n with ESC[<row>;<col>R. Nothing a person types
 * produces that, so seeing one means it is a reply we asked for and did not
 * hear in time. Reaching the line editor it would be read as cursor movement
 * and scramble the screen, so it is consumed here -- and used, if the width is
 * still unknown, which is better than throwing the answer away. */
static int strip_reports(uint8_t *buf, int len)
{
    int out = 0;
    for (int i = 0; i < len; )
    {
        if (buf[i] == 0x1b && i + 1 < len && buf[i + 1] == '[')
        {
            int j = i + 2;
            while (j < len && buf[j] >= '0' && buf[j] <= '9')
                j++;
            if (j < len && buf[j] == ';')
            {
                j++;
                while (j < len && buf[j] >= '0' && buf[j] <= '9')
                    j++;
                if (j < len && buf[j] == 'R')
                {
                    if (s_usb_columns <= 0)
                    {
                        const int c = parse_report((const char *)buf + i, j - i + 1);
                        if (c > 0)
                        {
                            s_usb_columns = c;
                            ESP_LOGI(TAG, "late width report: %d columns", c);
                        }
                    }
                    i = j + 1;
                    continue;
                }
            }
        }
        buf[out++] = buf[i++];
    }
    return out;
}

int link_read(uint8_t *buf, int cap)
{
    /* A queued panel size reply goes out before real input: the console is
     * waiting on it before it draws. */
    if (s_reply_pos < s_reply_len)
    {
        int n = s_reply_len - s_reply_pos;
        if (n > cap)
            n = cap;
        memcpy(buf, s_reply + s_reply_pos, (size_t)n);
        s_reply_pos += n;
        return n;
    }
    if (s_typed_len > 0)
    {
        size_t n = s_typed_len < (size_t)cap ? s_typed_len : (size_t)cap;
        memcpy(buf, s_typed, n);
        memmove(s_typed, s_typed + n, s_typed_len - n);
        s_typed_len -= n;
        return (int)n;
    }

    /* Block until something arrives, so the shell's stdio sees a normal
     * terminal rather than repeated EOFs. The timeout exists so the sink can
     * be re-checked: with the cable pulled there is no input and this must not
     * sit inside a driver call forever. */
    for (;;)
    {
        int n = usb_serial_jtag_read_bytes(buf, (uint32_t)cap,
                                           pdMS_TO_TICKS(READ_WAIT_MS));
        if (n > 0)
        {
            n = strip_reports(buf, n);
            if (n > 0)
                return n;
            continue;          /* the whole chunk was a report; keep waiting */
        }
        if (!usb_serial_jtag_is_connected() && s_on_usb)
            on_sink_changed(false);
    }
}

static int usb_write(const uint8_t *buf, int len)
{
    if (!usb_serial_jtag_is_connected())
        return 0;
    int done = 0;
    while (done < len)
    {
        int n = len - done > WRITE_CHUNK ? WRITE_CHUNK : len - done;
        if (usb_serial_jtag_write_bytes(buf + done, (size_t)n,
                                        pdMS_TO_TICKS(WRITE_TIMEOUT_MS)) != n)
            break;
        done += n;
    }
    return done;
}

int link_write(const uint8_t *buf, int len)
{
    const bool usb = usb_serial_jtag_is_connected();
    if (usb != s_on_usb)
        on_sink_changed(usb);

    if (usb)
    {
        int n = usb_write(buf, len);
        if (n > 0)
            s_usb_has_console = true;
        return n;
    }

    if (s_panel_ready)
    {
        vt_feed(buf, len);
        if (vt_take_size_query(NULL, NULL))
        {
            s_reply_len = snprintf(s_reply, sizeof(s_reply), "\x1b[%d;%dR",
                                   vt_rows(), vt_cols());
            s_reply_pos = 0;
        }
    }
    return len;
}

/* Non-blocking Ctrl+C check for a command that is running and owns the
 * console. Anything that is not Ctrl+C is stashed for the shell's next read,
 * which is what keeps type-ahead from being eaten while a command runs. */
bool tdsh_bridge_break_requested(void)
{
    bool brk = false;
    char tmp[32];
    for (;;)
    {
        int n = usb_serial_jtag_read_bytes((uint8_t *)tmp, sizeof(tmp), 0);
        if (n <= 0)
            break;
        for (int i = 0; i < n; ++i)
        {
            if (tmp[i] == 0x03)
            {
                brk = true;
                s_typed_len = 0;      /* like a terminal: drop the type-ahead */
            }
            else if (s_typed_len < sizeof(s_typed))
            {
                s_typed[s_typed_len++] = tmp[i];
            }
        }
    }
    return brk;
}

const char *board_platform(void)
{
    return s_on_usb ? "Arduino Nano ESP32 (USB Serial/JTAG)"
                    : "Arduino Nano ESP32 (ILI9340 320x240)";
}
const char *board_hostname(void)
{
    return "nanoesp32";
}
const td_mb_serial_t *board_rtu_lines(void)
{
    return rs485_serial();
}
