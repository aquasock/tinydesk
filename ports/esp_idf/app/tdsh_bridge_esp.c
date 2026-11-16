/*
 * tdsh_bridge_esp.c - runs the TinyDesk Shell console in its own FreeRTOS task
 * and connects it to the tinydesk Terminal app.
 *
 *   tinydesk task                        "tdsh" task
 *   keys  --write()--> [in  stream buffer] --> stdin  (blocking reads)
 *   vterm <--read()--- [out stream buffer] <-- stdout / stderr / ESP_LOG
 *
 * The shell task's stdin/stdout/stderr are a funopen() stream over the two
 * buffers, so every printf in TinyDesk Shell (and the ESP_LOGx calls it makes) lands
 * in the window. The UI side never blocks.
 */
#include "tdsh_bridge_esp.h"

#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "tdsh_espidf.h"

/*
 * THE INPUT BUFFER IS A FILE-TRANSFER SIZE, NOT A KEYBOARD SIZE.
 *
 * It was 256, which holds about six typed command lines. That is ample for a
 * human and far too small for a console being used as a data channel: a client
 * sending a file a line at a time puts 1000 bytes through here in three
 * seconds, and everything past the first 256 was DROPPED by backend_write()
 * below, silently, because xStreamBufferSend() with a zero timeout returns a
 * short count and its caller ignores the count.
 *
 * Measured before the change: a 1000-byte transfer into littlefs arrived as
 * 760, 800 and 920 bytes on three runs, varying only with pacing, while the BLE
 * console's own counters read `in=1853 out=1853 refused=0` -- every byte it took
 * was delivered. The loss was here.
 *
 * 2048 matches the output buffer and holds fifty of those lines. The shell's
 * input stream is heap, not static RAM.
 */
#define IN_BUFFER      2048
#define OUT_BUFFER     2048
#define SHELL_PRIORITY 4        /* below the UI task, so typing stays smooth */

static const char *TAG = "tdsh_bridge";

static StreamBufferHandle_t s_in;
static StreamBufferHandle_t s_out;
static SemaphoreHandle_t s_out_lock;   /* several tasks may print */
static bool s_started;

/* Keys a long-running command's Ctrl+C check took from s_in that were not
 * Ctrl+C; the shell reads them next. Only the shell task touches these. */
static char s_typed[64];              /* typing during a repeat is rare */
static size_t s_typed_len;
static void flush_input(void)
{
    s_typed_len = 0;
    char discard[64];
    while (xStreamBufferReceive(s_in, discard, sizeof(discard), 0) > 0)
    {
    }
}

/* ---------------------------------------------- shell-side stream */

static int stream_read(void *cookie, char *buf, int len)
{
    (void)cookie;
    if (s_typed_len > 0)
    {
        size_t n = s_typed_len < (size_t)len ? s_typed_len : (size_t)len;
        memcpy(buf, s_typed, n);
        memmove(s_typed, s_typed + n, s_typed_len - n);
        s_typed_len -= n;
        return (int)n;
    }
    /* Block until the user types something. */
    size_t n;
    do
    {
        n = xStreamBufferReceive(s_in, buf, (size_t)len, portMAX_DELAY);
    } while (n == 0);
    return (int)n;
}

static int stream_write(void *cookie, const char *buf, int len)
{
    (void)cookie;
    xSemaphoreTake(s_out_lock, portMAX_DELAY);
    int done = 0;
    while (done < len)
    {
        /* Blocks while the window is behind; the UI drains it every tick. */
        done += (int)xStreamBufferSend(s_out, buf + done, (size_t)(len - done), portMAX_DELAY);
    }
    xSemaphoreGive(s_out_lock);
    return len;
}

static void shell_task(void *arg)
{
    (void)arg;
    FILE *io = funopen(NULL, stream_read, stream_write, NULL, NULL);
    if (!io)
    {
        ESP_LOGE(TAG, "funopen failed");
        vTaskDelete(NULL);
        return;
    }
    setvbuf(io, NULL, _IONBF, 0);
    stdin = io;
    stdout = io;
    stderr = io;
    tdsh_espidf_run_console();   /* never returns */
}

bool tdsh_bridge_break_requested(void)
{
    /* Only for a command on the desktop's console: an SSH session's
     * command must not eat the Terminal window's keys. */
    if (!s_started || !tdsh_is_local_console_task())
        return false;
    bool brk = false;
    char tmp[32];
    size_t n;
    while ((n = xStreamBufferReceive(s_in, tmp, sizeof(tmp), 0)) > 0)
    {
        for (size_t i = 0; i < n; i++)
        {
            if (tmp[i] == 0x03)
            {
                brk = true;
                s_typed_len = 0;              /* like a terminal: drop the type-ahead */
            }
            else if (s_typed_len < sizeof(s_typed))
            {
                s_typed[s_typed_len++] = tmp[i];
            }
        }
    }
    return brk;
}

/* ------------------------------------------------------- backend */

/* The Terminal window's width, for the shell's line editor (set by the UI
 * task from start() and resize(), read by the shell task). */
static volatile int s_cols;

static int console_columns(void)
{
    return s_cols;
}

static void backend_resize(void *ctx, int cols, int rows)
{
    (void)ctx;
    (void)rows;
    s_cols = cols;
}

/* Called immediately before the shell's task is created, so a board that had
 * to hold internal RAM for it (one whose boot takes memory before the shell
 * exists) can release it at the last moment, with nothing in between to take
 * it. The default does nothing. */
__attribute__((weak)) void board_shell_task_prepare(void)
{
}

/* Called after the shell's task exists, which is NOT the same moment as the
 * release above: between them xTaskCreate takes the stack. A board that opens a
 * USB device at boot waits for this one (usbhost.c), so the open cannot race
 * the shell for the freed block. The default does nothing. */
__attribute__((weak)) void board_shell_task_ready(void)
{
}

static int backend_start(void *ctx, int cols, int rows)
{
    (void)ctx;
    (void)rows;
    s_cols = cols;
    if (s_started)
        return 0;
    tdsh_espidf_set_console_columns(console_columns);
    tdsh_espidf_set_console_flush(flush_input);
    s_in = xStreamBufferCreate(IN_BUFFER, 1);
    s_out = xStreamBufferCreate(OUT_BUFFER, 1);
    s_out_lock = xSemaphoreCreateMutex();
    if (!s_in || !s_out || !s_out_lock)
        return -1;
    board_shell_task_prepare();
    if (xTaskCreate(shell_task, "tdsh", TDSH_SHELL_TASK_STACK, NULL, SHELL_PRIORITY, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "cannot create the shell task (%u-byte stack; internal largest %u)",
                 (unsigned)TDSH_SHELL_TASK_STACK,
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        return -1;
    }
    board_shell_task_ready();
    s_started = true;
    return 0;
}

static int backend_read(void *ctx, uint8_t *buf, int cap)
{
    (void)ctx;
    return s_started ? (int)xStreamBufferReceive(s_out, buf, (size_t)cap, 0) : 0;
}

/*
 * HOW A KEY REACHES THE SHELL, AND WHY IT MAY NOW WAIT.
 *
 * xStreamBufferSend() with a zero timeout accepts what fits and returns the
 * count; the old code passed the result straight back to a caller that ignores
 * it, so a full buffer meant keys vanished. That is invisible while typing and
 * became data loss the moment the console carried a file.
 *
 * A bounded wait instead gives the whole chain its backpressure: the shell
 * drains, this blocks briefly, the desktop's key handling slows, fewer bytes
 * leave the transport, and the transport's own flow control (XOFF for the BLE
 * console) tells the far end to pause. That is what a serial console does.
 *
 * THE TOTAL WAIT IS BOUNDED and the function returns whichever is smaller of
 * "everything sent" and "budget used up". A wedged shell must not freeze the
 * desktop for ever, so after KEY_SEND_BUDGET_MS the remainder is abandoned --
 * still a possible loss, but now only behind a fault rather than under ordinary
 * load, and the short count is returned so a caller CAN see it.
 */
#define KEY_SEND_SLICE_MS   20
#define KEY_SEND_BUDGET_MS  2000

static int backend_write(void *ctx, const uint8_t *buf, int len)
{
    (void)ctx;

    if (!s_started || !buf || len <= 0)
        return 0;

    int sent = 0;
    int waited = 0;

    while (sent < len && waited < KEY_SEND_BUDGET_MS)
    {
        const size_t n = xStreamBufferSend(s_in, buf + sent, (size_t)(len - sent),
                                           pdMS_TO_TICKS(KEY_SEND_SLICE_MS));

        sent += (int)n;

        if (sent < len)
            waited += KEY_SEND_SLICE_MS;
    }

    if (sent < len)
        ESP_LOGW(TAG, "shell input full: %d of %d bytes after %d ms",
                 sent, len, waited);

    return sent;
}

/* The console session's user (boot user, or whoever logged in). */
static const char *backend_user(void *ctx)
{
    (void)ctx;
    return tdsh_espidf_console_user();
}

/* Continue the console as another user; Ctrl+C wakes the line editor so
 * the switch happens at once. */
static void backend_set_user(void *ctx, const char *user)
{
    (void)ctx;
    tdsh_espidf_console_set_user(user);
    if (s_started)
    {
        const uint8_t ctrl_c = 0x03;
        xStreamBufferSend(s_in, &ctrl_c, 1, 0);
    }
}

static const td_term_backend_t s_backend = {
    .name = "tdsh",
    .start = backend_start,
    .read = backend_read,
    .write = backend_write,
    .resize = backend_resize,
    .user = backend_user,
    .set_user = backend_set_user,
};

const td_term_backend_t *tdsh_bridge_esp_backend(void)
{
    return &s_backend;
}
