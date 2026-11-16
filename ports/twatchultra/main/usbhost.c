/*
 * usbhost.c - the USB-C port's role, and the one owner of the USB host stack.
 *
 * THE PORT IS EITHER THE PC'S DEVICE OR A HOST, NOT BOTH. The S3's USB-C port
 * is the USB-Serial-JTAG console the PC flashes through, and the OTG host that
 * drives a DAC. Once the host stack is installed the PC cannot see the board,
 * and only a restart gives the console back (observed every time on 2026-11-15:
 * after `uac2 open` the watch never re-enumerated on the PC until rebooted). So
 * the role is a SAVED setting applied at boot:
 *
 *   usb.host  1: OTG host (default 0: Normal, the PC's device)
 *   usb.dac   1: in host mode, use a USB-C audio DAC when one is plugged in
 *
 * Both are board keys, device-wide, in the same file as bt.enable, and the USB
 * page in Settings writes them through td_usb_ops_t.
 *
 * ONE OWNER FOR THE HOST STACK. Before this, uac2.c installed the host and ran
 * the library loop itself, and usbdiag.c did the same; two drivers would have
 * installed it twice. usbhost_install() is now the only install, and class
 * drivers register clients against it.
 *
 * Recovery when host mode leaves no way in: the BLE console still works, so
 * `board set usb.host 0` then `reboot` returns the port to Normal.
 */
#include "usbhost.h"

#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "tdsh_board.h"
#include "tinydesk/td_sysinfo.h"
#include "uac2.h"
#include "usb/usb_host.h"

static const char *TAG = "usbhost";

static bool s_installed;
static bool s_host_at_boot;     /* the role this boot applied */

static void lib_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        uint32_t flags = 0;

        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
            usb_host_device_free_all();
    }
}

bool usbhost_install(void)
{
    if (s_installed)
        return true;

    /* LEVEL3: WiFi and BLE hold the level-1 interrupt slots, and the install
     * returns ESP_ERR_NOT_FOUND with LEVEL1 (core-log entry 14). The FIFO bias
     * that lets an isochronous endpoint above 128 bytes open is a Kconfig
     * choice, CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT (reference USB-001). */
    const usb_host_config_t cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL3,
    };
    const esp_err_t err = usb_host_install(&cfg);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(err));
        return false;
    }
    /*
     * THE LIBRARY TASK'S STACK COMES FROM PSRAM, AND THAT IS THE FIX FOR THE
     * INTERNAL-RAM WALL THIS PORT KEPT HITTING.
     *
     * In OTG mode the internal DMA-capable pool is the bottleneck, not a
     * shortage of total RAM: measured on hardware it read dma free=6359 /
     * largest=4096 at boot, and the USB audio claim -- which needs two
     * 512-byte, 512-ALIGNED descriptor lists -- then asserted inside
     * transfer_descriptor_list_alloc (hcd_dwc.c:1059) because no such hole was
     * left (core-log entries 21 and 22). Three 4 KB internal task stacks are
     * 12 KB of that pool, and NONE of them touches flash: usb_lib only services
     * the host library's event queue. CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY
     * is already enabled for this board, so moving them to PSRAM returns 12 KB
     * to the pool that the claim, enumeration and the stream's transfers all
     * share, with no reserve, no hold and no special ordering.
     *
     * SAFE AGAINST FLASH OPERATIONS: a PSRAM stack may not be RUNNING while the
     * flash cache is disabled. OTA runs on the shell's task (an internal stack),
     * and while it disables the cache IDF parks the other core at a safe point
     * and disables interrupts, so a task with a PSRAM stack is never executing
     * in that window. The rule this code must keep: no task created this way may
     * itself call a flash or OTA API.
     */
    if (xTaskCreateWithCaps(lib_task, "usb_lib", 4096, NULL, 4, NULL,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS)
    {
        ESP_LOGE(TAG, "no library task");
        return false;
    }
    s_installed = true;
    return true;
}

bool usbhost_installed(void)
{
    return s_installed;
}

/* ------------------------------------------------------------- board keys */

static bool key_on(const char *key)
{
    const char *v = tdsh_board_get(key);

    return v && (v[0] == '1' || v[0] == 'y' || v[0] == 'Y');
}

static bool key_set(const char *key, bool on)
{
    return tdsh_board_set(key, on ? "1" : "0") == 0;
}

/* Internal RAM at a point in the boot, for the Log Viewer: OTG mode takes its
 * memory before the shell exists, and the shell's 24 KB task is the first
 * thing to notice when there is not enough left.
 *
 * SHORT AND DMA-FIRST ON PURPOSE. The Log Viewer keeps 128 lines and cuts each
 * at 38 columns, so the previous "before USB: internal free N, largest N, DMA
 * largest N" lost the only number that mattered -- entry 20 recorded that the
 * boot's heap figures were lost exactly this way. DMA-capable internal RAM is
 * the scarce pool that enumeration and the claim both draw from, so it leads.
 */
static void heap_note(const char *when)
{
    ESP_LOGI(TAG, "%s dma=%u/%u int=%u/%u", when,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

/*
 * ROOM FOR THE SHELL. The shell's task is created at boot, just after this
 * port's init, with a 24 KB stack in internal RAM (TDSH_SHELL_TASK_STACK). In OTG
 * mode the USB host, its tasks and the stream's buffers are taken here first,
 * and on hardware the shell then failed: "cannot create the shell task" at
 * 2919 ms. A block of the shell's size released at the end of this function did
 * not help -- the telnet server started in between (2910 ms) and took its stack
 * from the hole. So the block is held until board_shell_task_prepare(), which
 * the shell bridge calls immediately before creating the task.
 */
#define USBHOST_SHELL_HOLD  (24576 + 1024)

static void *s_shell_hold;

void board_shell_task_prepare(void)
{
    if (!s_shell_hold)
        return;
    heap_caps_free(s_shell_hold);
    s_shell_hold = NULL;
    heap_note("hole-free");
}

/*
 * THE OTHER HALF, AND THE ONE THE USB AUDIO WORKER GATES ON. Releasing the hold
 * is not the same event as the task existing: between the two, xTaskCreate takes
 * the stack from the hole, and a DAC open running in that gap races the shell for
 * it. So the worker waits for THIS, set after xTaskCreate returned, not for the
 * release above. The weak default in tdsh_bridge_esp.c does nothing.
 */
static volatile bool s_shell_ready;

void board_shell_task_ready(void)
{
    s_shell_ready = true;
}

bool usbhost_shell_ready(void)
{
    return s_shell_ready;
}

void usbhost_boot(void)
{
    s_host_at_boot = key_on("usb.host");
    if (!s_host_at_boot)
        return;

    ESP_LOGI(TAG, "OTG host mode (usb.host = 1): the PC will not see this board");
    heap_note("pre-USB");

    s_shell_hold = heap_caps_malloc(USBHOST_SHELL_HOLD, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_shell_hold)
        ESP_LOGW(TAG, "no %u-byte block to hold for the shell", (unsigned)USBHOST_SHELL_HOLD);

    if (usbhost_install() && key_on("usb.dac") && !uac2_auto_start())
        ESP_LOGW(TAG, "USB-C DAC support did not start");

    heap_note("post-USB");
}

/* ---------------------------------------------------------- td_usb_ops_t */

/* Device classes, in the order the Settings page lists them. A controller would
 * be appended here with its own key. */
static const struct
{
    const char *name;
    const char *key;
} s_classes[] = {
    {"USB-C DAC", "usb.dac"},
};

#define NCLASSES ((int)(sizeof(s_classes) / sizeof(s_classes[0])))

static bool ops_host_saved(void)
{
    return key_on("usb.host");
}

static bool ops_host_running(void)
{
    return s_installed;
}

static bool ops_set_host(bool host)
{
    return key_set("usb.host", host);
}

static int ops_devices(void)
{
    return NCLASSES;
}

static const char *ops_device_name(int i)
{
    return i >= 0 && i < NCLASSES ? s_classes[i].name : "";
}

static bool ops_device_enabled(int i)
{
    return i >= 0 && i < NCLASSES && key_on(s_classes[i].key);
}

static bool ops_set_device_enabled(int i, bool on)
{
    return i >= 0 && i < NCLASSES && key_set(s_classes[i].key, on);
}

static void ops_status(char *out, int cap)
{
    if (!out || cap <= 0)
        return;
    if (!s_installed)
        snprintf(out, (size_t)cap, "host not running");
    else if (uac2_attached())
        snprintf(out, (size_t)cap, "USB-C DAC: %.30s", uac2_product());
    else
        snprintf(out, (size_t)cap, "nothing attached");
}

static void ops_restart(void)
{
    esp_restart();
}

static const td_usb_ops_t s_ops = {
    .host_saved = ops_host_saved,
    .host_running = ops_host_running,
    .set_host = ops_set_host,
    .devices = ops_devices,
    .device_name = ops_device_name,
    .device_enabled = ops_device_enabled,
    .set_device_enabled = ops_set_device_enabled,
    .status = ops_status,
    .restart = ops_restart,
};

/* Installed into td_sysinfo_t by main.c through the weak board_usb_ops(). */
const td_usb_ops_t *board_usb_ops(void)
{
    return &s_ops;
}
