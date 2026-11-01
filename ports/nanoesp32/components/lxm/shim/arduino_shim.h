/*
 * arduino_shim.h - the small part of the Arduino API that microReticulum
 * uses, provided for an ESP-IDF build.
 *
 * microReticulum is an Arduino library. Its OS layer already has an
 * #ifdef ARDUINO branch that falls back to POSIX (gettimeofday, nanosleep)
 * when the macro is absent, and that branch is what an ESP-IDF build takes --
 * newlib provides both. What is left over is a handful of identifiers that
 * are included unconditionally: Log.h and Reticulum.cpp include <Arduino.h>,
 * and Reticulum.cpp prints ESP.getPsramSize() and calls ESP.restart().
 *
 * This is not an Arduino emulation. It is those identifiers, mapped onto the
 * ESP-IDF calls they already wrap, so the library links unchanged and stays
 * updatable from upstream.
 *
 * IMPORTANT: do not define ARDUINO for this build. Its presence makes
 * ArduinoJson compile in its Arduino mode (String, Stream, PROGMEM,
 * FlashStringHelper) and the error count goes from a handful to hundreds.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#include <string>
#endif

#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* millis() and micros() are NOT defined here: Crypto's RNG.cpp already
 * provides them for non-Arduino targets, and defining them twice is a
 * redefinition error. */

static inline void delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
}

static inline void delayMicroseconds(uint32_t us)
{
    esp_rom_delay_us(us);
}

/* Long random, in the shape the Arduino core's random() has. */
static inline long random(long maximum)
{
    return maximum <= 0 ? 0 : (long)(esp_random() % (uint32_t)maximum);
}

/* The ESP singleton: the two members microReticulum touches. */
struct EspClassShim
{
    inline void restart(void) { esp_restart(); }
    inline size_t getPsramSize(void) { return heap_caps_get_total_size(MALLOC_CAP_SPIRAM); }
    inline size_t getFreePsram(void) { return heap_caps_get_free_size(MALLOC_CAP_SPIRAM); }
    inline size_t getFreeHeap(void) { return heap_caps_get_free_size(MALLOC_CAP_8BIT); }
};
static EspClassShim ESP;

#ifdef __cplusplus
/* Arduino String, as far as microReticulum's Log overloads need it: a string
 * literal converts to a StringSumHelper in the Arduino String API, and only
 * c_str() is called on it. A plain std::string would collide with the other
 * log() overloads, so this stays a distinct type. */
class StringSumHelper
{
    std::string s_;

public:
    StringSumHelper(const char *s) : s_(s ? s : "") {}
    const char *c_str() const { return s_.c_str(); }
    size_t length() const { return s_.size(); }
};
#endif /* __cplusplus */
