/*
 * nanosleep for ESP-IDF.
 *
 * microReticulum's OS layer takes its POSIX path on this build (ARDUINO is
 * undefined, which is deliberate: it also keeps ArduinoJson out of Arduino
 * mode) and sleeps with nanosleep(). Newlib under ESP-IDF does not provide
 * it, so the link fails without this.
 *
 * It forwards to the RTOS scheduler rather than spinning, so the desktop's UI
 * task keeps drawing while Reticulum waits.
 */
#include <errno.h>
#include <stdint.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

int nanosleep(const struct timespec *req, struct timespec *rem)
{
    (void)rem;
    if (!req || req->tv_sec < 0 || req->tv_nsec < 0)
    {
        errno = EINVAL;
        return -1;
    }
    uint64_t ms = (uint64_t)req->tv_sec * 1000ULL + (uint64_t)req->tv_nsec / 1000000ULL;
    vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
    return 0;
}
