/*
 * main_console.c - app_main for the Nano ESP32: a console, not a desktop.
 *
 * This board runs TinyDesk Shell on its own. There is no window manager, no
 * app set and no screen buffers: what the operator gets is a shell, served
 * over USB Serial/JTAG when a host terminal is attached and on the ILI9340
 * panel when one is not (see link_nano.c for the switch).
 *
 * The Reticulum node runs beside it exactly as before. node_start() is called
 * after the shell is initialised, so a node that fails to come up still leaves
 * a usable console to diagnose it from -- which is the whole reason this image
 * exists.
 */
#include <stdio.h>
#include <string.h>

#include "esp_rom_sys.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "link.h"
#include "node.h"
#include "vt.h"

/* The active sink's width: 40 on the panel, the host terminal's own width over
 * USB. The shell's line editor asks for this because it cannot measure a
 * funopen() stream -- fileno(stdin) is -1, so its own query is skipped and it
 * assumes 80 columns, which pads and wraps every line wrongly. */
int link_console_columns(void);
#include "tdsh.h"
#include "tdsh_espidf.h"

static const char *TAG = "console";

/* Built in from board.conf or board.example.conf by the component CMakeLists;
 * /fs/etc/board.conf on the device overrides it at run time. */
extern const char s_board_builtin[] asm("_binary_board_builtin_conf_start");

/* The shell's own task stack size; a smaller one overflows inside the
 * line editor and takes the board down before it prints anything. */
#define CONSOLE_TASK_STACK TDSH_SHELL_TASK_STACK

static int stream_read(void *cookie, char *buf, int len)
{
    (void)cookie;
    return link_read((uint8_t *)buf, len);
}

/* ONLCR, done here because there is no tty to do it.
 *
 * The shell's line editor emits CRLF itself, but command output arrives from
 * printf as a bare LF. With no line discipline a bare LF moves the cursor down
 * a line and leaves it in the same column, so the next prompt starts where the
 * previous output ended -- the prompt appears indented by the length of the
 * line above it. Both sinks make that assumption: vt.c treats '\n' as a line
 * feed and '\r' separately, and a terminal in raw mode does the same.
 *
 * So every LF that is not already preceded by a CR gets one. State is kept
 * across calls because a write can end mid-pair. */
static bool s_prev_cr;

static int stream_write(void *cookie, const char *buf, int len)
{
    (void)cookie;
    uint8_t out[192];
    size_t n = 0;
    for (int i = 0; i < len; ++i)
    {
        const char c = buf[i];
        if (n + 2 >= sizeof out)
        {
            /* A sink that accepted nothing (cable just pulled, panel not
             * ready) must not look like an error to stdio, so the return value
             * below reports the whole chunk regardless; link_write drops what
             * it cannot send. */
            link_write(out, (int)n);
            n = 0;
        }
        if (c == '\n' && !s_prev_cr)
            out[n++] = '\r';
        out[n++] = (uint8_t)c;
        s_prev_cr = (c == '\r');
    }
    if (n > 0)
        link_write(out, (int)n);
    return len;
}

static void console_task(void *arg)
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

void app_main(void)
{
    /* Keep the ROM banner off USB: it belongs to the console from the first
     * byte the shell writes. */
    esp_rom_install_channel_putc(1, NULL);
    esp_rom_install_channel_putc(2, NULL);
    esp_deep_sleep_disable_rom_logging();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* The TCP/IP stack and the default event loop before anything opens a
     * socket: the shell's network commands and the node's UDP interface both
     * assume they exist. WiFi itself is brought up by the node. */
    ESP_ERROR_CHECK(esp_netif_init());
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(err);

    if (!link_init())
    {
        ESP_LOGE(TAG, "link init failed: no console and no way to report it");
        return;
    }

    tdsh_espidf_set_console_columns(link_console_columns);

    tdsh_espidf_config_t cfg = TDSH_ESP_IDF_CONFIG_DEFAULT();
    cfg.hostname = board_hostname();
    cfg.board_config = s_board_builtin;
    cfg.default_user = "root";
    /* The node owns the radio; the shell must not race it for the interface. */
    cfg.init_network_manager = false;
    err = tdsh_espidf_init(&cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "shell init failed: %s", esp_err_to_name(err));

    /* The node registers its `node` and `magcal` commands at start, so it has
     * to run after the shell's command table exists. */
    node_start();

    if (xTaskCreate(console_task, "console", CONSOLE_TASK_STACK, NULL, 4, NULL) != pdPASS)
        ESP_LOGE(TAG, "cannot create the console task");

    ESP_LOGI(TAG, "console %dx%d, %u bytes free",
             vt_cols(), vt_rows(), (unsigned)esp_get_free_heap_size());
}
