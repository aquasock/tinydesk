/*
 * pusher.c - the T-Watch Ultra's BOOT pusher (Pusher B) as a key.
 *
 * WHY THIS IS SHAPED LIKE touch.c AND NOT LIKE A BUTTON LIBRARY. The port has
 * exactly one mechanism for "a pin tells a task that something happened", and
 * it is the one the touch interrupt uses: configure the pin, install a
 * falling-edge handler, and have the handler notify a task that does the work.
 * Adding a second mechanism for one switch would mean two places to look when
 * an input misbehaves, so this follows that pattern rather than inventing a
 * poll or pulling in a component.
 *
 * THE ISR DOES NOTHING BUT NOTIFY, for the same reasons it does not in
 * touch.c: it must not take a lock, allocate, or call into the desktop, and the
 * notification coalesces harmlessly because a press that is already being
 * handled does not need a second wake-up.
 *
 * WHAT IT SENDS. One key per press, on the transition into the press, as the
 * terminal would send it -- the desktop's input parser turns the bytes back
 * into a key event, so a pusher press is indistinguishable from the same key
 * on a Bluetooth keyboard. That is the same path the margin buttons use, and
 * it is why nothing below the port has to know this pusher exists.
 *
 * ENTER FOR NOW, DELIBERATELY. The operator has not decided what this button
 * should do, so the mapping is one constant and one table, not a policy. Enter
 * is a key that does something in nearly every context -- it activates the
 * focused control, it runs the selected item in the launcher, it submits a form
 * -- so a press is never silently swallowed while the question is open.
 *
 * GPIO 0 IS A STRAPPING PIN. Holding the pusher down while the watch powers on
 * puts the ESP32-S3 into serial download mode; that is the pin's designed
 * function and no firmware can override it. It also means esptool's resets,
 * which toggle IO0, can look like a press -- hence the debounce and the settle
 * delay before the handler is installed.
 */

#include "pusher.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh_board.h"

#include "haptic.h"
#include "power.h"

#define TAG "twatch-pusher"

/* The BOOT pusher. LilyGoLib declares `Button2 bootButton = Button2(0)` and
 * pins it with pinMode(0, INPUT); the schematic shows the switch on IO0 with a
 * 10K pull-up, so it idles high and pulls low. `pusher.pin` overrides it. */
#define PUSHER_PIN      0

/* Twenty-five milliseconds is long enough to swallow switch bounce and short
 * enough that the key lands while the operator still considers it immediate.
 * Measured against nothing yet -- if a press ever reads as two, this is the
 * number, and if a light tap is ever missed, so is this. */
#define DEBOUNCE_MS     25

/* The pin's level is sampled this long after boot before the handler is
 * installed. esptool's reset sequence drives IO0 low, and without the settle an
 * ordinary flash would inject one key into the desktop on the way up. */
#define SETTLE_MS       250

static pusher_input_fn s_input;
static int s_pin = -1;
static TaskHandle_t s_task;
static volatile uint32_t s_presses;

/* Enter, as a terminal sends it. */
static void send_key(void)
{
    static const char ENTER[] = "\r";

    if (s_input)
        s_input(ENTER, (int)(sizeof(ENTER) - 1));
}

/* Record the edge and wake the task. Nothing else happens here. */
static void IRAM_ATTR pusher_isr(void *arg)
{
    (void)arg;
    BaseType_t woken = pdFALSE;
    TaskHandle_t t = s_task;

    if (t)
        vTaskNotifyGiveFromISR(t, &woken);
    if (woken)
        portYIELD_FROM_ISR();
}

static void pusher_task(void *arg)
{
    (void)arg;

    /* Let a reset finish before believing anything on this pin. esptool drives
     * IO0 low to enter download mode, so a flash can leave an edge pending
     * that no finger caused. Doing this here rather than in pusher_init() keeps
     * the delay off the desktop's startup path -- link_init() would otherwise
     * block for a quarter of a second before the first frame -- and any edge
     * that did arrive is handled by the level check below, because by then IO0
     * is high again. */
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
    ulTaskNotifyTake(pdTRUE, 0);        /* drop anything from the settle */

    for (;;)
    {
        /* Wait for an edge, then let the switch settle before believing it.
         * The notification is taken inside the debounce so that a burst of
         * bounce is one press, not several. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));

        /* Still held? A press is the transition into the low state; a release
         * is ignored, because acting on the release would make the button feel
         * late and would fire once per bounce on the way up. */
        if (gpio_get_level((gpio_num_t)s_pin) != 0)
            continue;

        s_presses++;
        send_key();
        haptic_tap();

        /* One short line per press, and short because the Log Viewer is 40
         * columns: the count first, so it survives the clip. */
        ESP_LOGI(TAG, "p%lu", (unsigned long)s_presses);

        /* Swallow the rest of this press. The switch may still be bouncing or
         * still held; either way the next press is a new notification, and
         * without this a held button would repeat. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
    }
}

bool pusher_init(pusher_input_fn input)
{
    s_input = input;

    s_pin = tdsh_board_int("pusher.pin", PUSHER_PIN);
    if (s_pin < 0)
    {
        ESP_LOGI(TAG, "off");
        return true;                /* deliberately disabled, not a failure */
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_pin,
        .mode = GPIO_MODE_INPUT,
        /* The board already has a 10K pull-up on this line, and the internal
         * one is enabled as well rather than instead: a floating GPIO 0 is a
         * strapping pin left to the weather, and the extra pull costs nothing
         * when the external resistor is fitted. */
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    esp_err_t err = gpio_config(&io);
    if (err == ESP_OK && !board_gpio_isr_init())
        err = ESP_FAIL;
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "pin %d: %s", s_pin, esp_err_to_name(err));
        s_pin = -1;
        return false;
    }

    if (xTaskCreatePinnedToCore(pusher_task, "tdpusher", 2560, NULL, 3,
                                &s_task, 0) != pdPASS)
    {
        s_pin = -1;
        return false;
    }

    if (gpio_isr_handler_add((gpio_num_t)s_pin, pusher_isr, NULL) != ESP_OK)
    {
        ESP_LOGW(TAG, "pin %d: no handler", s_pin);
        s_pin = -1;
        return false;
    }

    ESP_LOGI(TAG, "p%d enter", s_pin);
    return true;
}
