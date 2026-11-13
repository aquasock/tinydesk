#pragma once
#include <stdint.h>
#include <stddef.h>
typedef void* SemaphoreHandle_t;
typedef void* TaskHandle_t;
typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(x) (x)
#define portMAX_DELAY 0xffffffff
#define portTICK_PERIOD_MS 1

/* Critical sections: the host test is single-threaded, so these are no-ops.
 * That is honest rather than convenient -- the test exercises the PARSER, which
 * does no locking, and the locking in hid_input.c guards a ring buffer the test
 * never touches. */
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define taskENTER_CRITICAL(m) ((void)0)
#define taskEXIT_CRITICAL(m)  ((void)0)
#define xSemaphoreCreateMutex() ((SemaphoreHandle_t)1)
#define xSemaphoreTake(s, t) 1
#define xSemaphoreGive(s) 1
static inline uint32_t xTaskGetTickCount(void) { return 0; }
