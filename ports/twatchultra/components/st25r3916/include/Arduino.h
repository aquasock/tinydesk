/*
 * Arduino.h - the small part of the Arduino core that ST's NFC stack uses.
 *
 * The ST25R3916-NFC-RFAL library is Arduino code: about 40 000 lines of ST's
 * RFAL abstraction layer, the ST25R3916 register driver, and an NDEF stack,
 * and it is the code LilyGo's own firmware drives this exact chip with. Porting
 * it rather than rewriting it is the point -- the analog configuration table
 * and the ISO14443A anticollision sequence in it are the two things that are
 * hard to get right and impossible to guess.
 *
 * So this header exists to be the ONLY thing that changes. The library is
 * dropped in unmodified under ../rfal and these three headers (this, SPI.h,
 * Wire.h) supply what it expects from a core we do not have. The surface is
 * smaller than it looks: types, six GPIO and timing calls, MIN/MAX, and a
 * Serial object that is never asked to print anything in our configuration.
 *
 * WHAT IS DELIBERATELY A NO-OP. attachInterrupt() and detachInterrupt() do
 * nothing, and that is a considered decision rather than an omission. The
 * library's interrupt path sets a flag and then falls back to reading the pin:
 *
 *     bool isISRPending() { return isr_pending || (digitalRead(int_pin) == HIGH); }
 *
 * so a high line is detected on the next poll whether or not an ISR ever ran.
 * A real ISR here would call through a std::function from interrupt context --
 * not guaranteed to be in IRAM, and it would also fire while an SPI transaction
 * is in flight on the same pin set. The worker loop runs continuously and calls
 * this at least once a millisecond, which is a latency the 106 kbit/s NFC-A
 * framing does not notice.
 */
#ifndef ARDUINO_H
#define ARDUINO_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pin levels and directions. */
#define HIGH 0x1
#define LOW  0x0

#define INPUT        0x0
#define OUTPUT       0x1
#define INPUT_PULLUP 0x2

/* Interrupt edges. Only used as an argument this port ignores. */
#define RISING  0x1
#define FALLING 0x2
#define CHANGE  0x3

/* Print bases, for the Serial calls the library keeps behind debug defines. */
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

/* SPI bit order. */
#define LSBFIRST 0
#define MSBFIRST 1

void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
uint32_t millis(void);
uint32_t micros(void);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
void attachInterrupt(uint8_t pin, void (*fn)(void), int mode);
void detachInterrupt(uint8_t pin);

#ifdef __cplusplus
}
#endif

/* MIN/MAX are macros in the Arduino core, and the stack uses them by name.
 * Guarded because ESP-IDF and newlib both have their own spellings and a
 * redefinition here would be a warning in somebody else's header. */
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifdef __cplusplus
/*
 * The Serial object. Every use in the library is inside a debug macro that is
 * compiled out unless RFAL_CE_DEBUG, RFAL_ST25R3916_LISTEN_DEBUG or one of the
 * ISO-DEP equivalents is defined, all of which are off -- but the calls still
 * have to typecheck, and they take a variety of argument types, so the methods
 * are variadic templates rather than an overload set that would have to be
 * extended every time one is added.
 */
struct ArduinoSerial
{
    void begin(unsigned long) {}

    template <typename... A>
    void print(A...) {}

    template <typename... A>
    void println(A...) {}
};

extern ArduinoSerial Serial;
#endif /* __cplusplus */

#endif /* ARDUINO_H */
