/*
 * pwrkey.c - Pusher A, the watch's power key, as a key.
 *
 * THE HARDWARE, AND WHY THIS IS NOT A BUTTON DRIVER. Pusher A is wired to the
 * AXP2101's PEK input, not to an SoC GPIO. The PMU reports it on its own
 * interrupt line, PMU_IRQ on GPIO 7 (confirmed in the schematic net list and
 * used by this project's Linux port as `interrupts = <7 IRQ_TYPE_LEVEL_LOW>`).
 * The line is open drain, so it needs a pull-up, and BRD-002 records what
 * happens without one: the line floats, reads as permanently asserted, and a
 * level-triggered handler on it storms -- which on the Linux side hung the boot
 * and starved the bit-banged I2C bus.
 *
 * THE PIN'S PULL-UP IS THEREFORE PART OF THE CONTRACT. The board has no
 * external pull-up on GPIO 7; Linux supplies the weak internal one with
 * `PIN(7) FUN0_20MA_IE_WPU`. This enables the internal pull-up for the same
 * reason and does not treat it as a nicety.
 *
 * THE LINE STAYS LOW UNTIL THE STATUS IS CLEARED. The AXP2101 asserts IRQ while
 * any enabled source is pending, and the pending bits at 0x48..0x4a are
 * write-1-to-clear. That single fact is the difference between a working button
 * and a board that will not boot: a handler that wakes, reads and returns
 * without clearing is a handler that is called again immediately, for ever. So
 * every wake reads all three status bytes and writes them straight back. The
 * PMU's own driver does the same thing through regmap-irq, whose ack_base for
 * this part is the status register.
 *
 * THE GUARD AT START-UP. Before the interrupt is armed, anything pending is
 * cleared and the line is read. If it is still low, something is holding it and
 * arming a level interrupt would storm, so the driver declines and says so. One
 * lost button beats an unbootable watch.
 *
 * WHY SHORT PRESS AND NOT THE EDGE. The PMU reports both edges of the key and a
 * separate short-press event. Acting on the falling edge would fire at the
 * moment of touch but would also fire for a long hold, which on this part is
 * the hardware power-off gesture. The short-press event is the deliberate press
 * a button should mean: it fires once, on release, with no bounce and no
 * repeat while held. Its cost is that the action lands on release rather than
 * on touch, which for a power key is what every device does anyway.
 *
 * The default enable mask already covers the edges and VBUS. This adds the
 * short-press bit and leaves the rest as it found them.
 */

#include "pwrkey.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tdsh_board.h"

#include "charge.h"
#include "haptic.h"
#include "power.h"

#define TAG "twatch-pwrkey"

/* PMU_IRQ, from the schematic net list (`PMU_IRQ------IO7`) and from this
 * project's Linux port. `pwrkey.pin` overrides it. */
#define PWRKEY_PIN      7

/* The AXP2101's interrupt block: three enable bytes at 0x40 and three status
 * bytes at 0x48, one bit per source. Bit positions from the part's driver,
 * patches/kernel/axp2101/final/axp20x.c axp2101_regmap_irqs[]. */
#define PMU_IRQ0_EN     0x40
#define PMU_IRQ1_EN     0x41
#define PMU_IRQ2_EN     0x42
#define PMU_IRQ0_STATE  0x48
#define PMU_IRQ_NREGS   3

/* Bank 1 (0x49), the byte that carries the key and the charger. */
#define IRQ1_PEK_RIS    0x01
#define IRQ1_PEK_FAL    0x02
#define IRQ1_PEK_LONG   0x04
#define IRQ1_PEK_SHORT  0x08

/* Enter, as a terminal sends it -- the same key the BOOT pusher sends, and the
 * same one-line change to alter. */
static const char KEY_ENTER[] = "\r";

static pwrkey_input_fn s_input;
static int s_pin = -1;
static TaskHandle_t s_task;
static volatile uint32_t s_presses;

static void send_key(void)
{
    if (s_input)
        s_input(KEY_ENTER, (int)(sizeof(KEY_ENTER) - 1));
}

/* Read the three status bytes. The PMU gives them sequentially from 0x48, so
 * this is three single-byte reads rather than a burst: the bus helper this port
 * owns is one byte at a time and adding a burst path for two extra registers
 * would be more code than three reads. */
static bool read_status(uint8_t st[3])
{
    for (int i = 0; i < PMU_IRQ_NREGS; i++)
    {
        if (!board_pmu_read((uint8_t)(PMU_IRQ0_STATE + i), &st[i]))
            return false;
    }
    return true;
}

/* Write the bits back. These registers are write-1-to-clear, so writing what
 * was read clears exactly what was set and leaves everything else alone. */
static bool clear_status(const uint8_t st[3])
{
    bool ok = true;

    for (int i = 0; i < PMU_IRQ_NREGS; i++)
    {
        if (st[i] && !board_pmu_write((uint8_t)(PMU_IRQ0_STATE + i), st[i]))
            ok = false;
    }
    return ok;
}

static void IRAM_ATTR pwrkey_isr(void *arg)
{
    (void)arg;
    BaseType_t woken = pdFALSE;
    TaskHandle_t t = s_task;

    if (t)
        vTaskNotifyGiveFromISR(t, &woken);
    if (woken)
        portYIELD_FROM_ISR();
}

static void pwrkey_task(void *arg)
{
    (void)arg;

    for (;;)
    {
        uint8_t st[3];

        /* The interrupt is the normal wake-up; the timeout is a safety net so
         * that a missed edge does not leave a pending status uncleared for
         * ever, which is the one state that can hold the line low. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        if (!read_status(st))
            continue;

        /* Clear first, then act. If the action somehow failed, a pending bit
         * left set would re-assert the line and the watchdog above would see it
         * a second later -- which is a recoverable nuisance rather than a
         * storm. */
        clear_status(st);

        if (st[1] & IRQ1_PEK_SHORT)
        {
            s_presses++;
            send_key();
            haptic_tap();
            ESP_LOGI(TAG, "k%lu", (unsigned long)s_presses);
        }

        /* The cable and the charger are charge.c's business, not this file's:
         * this driver owns the pin and clears the status, and hands the bits
         * over. That keeps "what a VBUS edge means" in one place instead of
         * split between a button driver and a charge driver. */
        charge_on_irq(st);

        /* A long press is the PMU's own power-off gesture and is not ours to
         * intercept; logged only so the log explains a shutdown. */
        if (st[1] & IRQ1_PEK_LONG)
            ESP_LOGI(TAG, "key long");
    }
}

bool pwrkey_init(pwrkey_input_fn input)
{
    uint8_t st[3], en;

    s_input = input;

    s_pin = tdsh_board_int("pwrkey.pin", PWRKEY_PIN);
    if (s_pin < 0)
    {
        ESP_LOGI(TAG, "off");
        return true;                /* deliberately disabled, not a failure */
    }

    /* The PMU must be on the bus before any of this means anything. */
    if (!board_pmu_read(PMU_IRQ0_EN, &en))
    {
        ESP_LOGW(TAG, "pmu silent");
        return false;
    }

    /* What was pending before we got here. On a cold boot with USB attached
     * this is typically VBUS_PLUGIN, which is exactly the kind of bit that
     * would hold the line low through a naive bring-up. */
    if (read_status(st))
    {
        if (st[0] || st[1] || st[2])
        {
            /* Compact, and the interesting one first: the 40-column ring cuts
             * the tail off anything longer. */
            ESP_LOGI(TAG, "st %02x%02x%02x", st[0], st[1], st[2]);
            /* Let charge.c see the boot state as well, so its idea of whether
             * the cable is in and the cell is charging starts from what the
             * part actually reports rather than from unknown. Without this the
             * first logged transition would be a removal with no matching
             * arrival, which reads like a fault in the log. */
            charge_on_irq(st);
            clear_status(st);
        }
    }

    /* Enable the short press and keep whatever else was already enabled. The
     * measured default is 0xc3 -- both edges plus VBUS -- so this is adding the
     * one bit the part leaves off. */
    if (board_pmu_read(PMU_IRQ1_EN, &en))
    {
        if (!(en & IRQ1_PEK_SHORT))
        {
            if (!board_pmu_write(PMU_IRQ1_EN, (uint8_t)(en | IRQ1_PEK_SHORT)))
                ESP_LOGW(TAG, "could not enable short press");
        }
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_pin,
        .mode = GPIO_MODE_INPUT,
        /* Open drain on the PMU side, no external resistor on the board: the
         * internal pull-up is the only thing holding this line high. Without it
         * the line floats and the handler storms (BRD-002). */
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    if (gpio_config(&io) != ESP_OK || !board_gpio_isr_init())
    {
        ESP_LOGW(TAG, "pin %d unusable", s_pin);
        s_pin = -1;
        return false;
    }

    /* Let the line settle after the clears above before judging it, then judge
     * it. This is the guard: a line still low means something is holding the
     * interrupt that we are not clearing, and arming a falling-edge handler on
     * it would call the ISR without end. */
    vTaskDelay(pdMS_TO_TICKS(5));
    if (gpio_get_level((gpio_num_t)s_pin) == 0)
    {
        ESP_LOGW(TAG, "line low, not armed");
        s_pin = -1;
        return false;
    }

    if (xTaskCreatePinnedToCore(pwrkey_task, "tdpwrkey", 3072, NULL, 3,
                                &s_task, 0) != pdPASS)
    {
        s_pin = -1;
        return false;
    }

    if (gpio_isr_handler_add((gpio_num_t)s_pin, pwrkey_isr, NULL) != ESP_OK)
    {
        ESP_LOGW(TAG, "pin %d: no handler", s_pin);
        s_pin = -1;
        return false;
    }

    ESP_LOGI(TAG, "p%d enter", s_pin);
    return true;
}
