#ifndef ST25R3916_ARDUINO_BUS_H
#define ST25R3916_ARDUINO_BUS_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#ifndef ST25R3916_USE_I2C
#define ST25R3916_USE_I2C 0
#endif

#ifndef ST25R3916_PIN_CS
#define ST25R3916_PIN_CS 5
#endif

#ifndef ST25R3916_PIN_IRQ
#define ST25R3916_PIN_IRQ 2
#endif

#ifndef ST25R3916_PIN_SCK
#define ST25R3916_PIN_SCK 18
#endif

#ifndef ST25R3916_PIN_MISO
#define ST25R3916_PIN_MISO 19
#endif

#ifndef ST25R3916_PIN_MOSI
#define ST25R3916_PIN_MOSI 23
#endif

#ifndef ST25R3916_PIN_SDA
#define ST25R3916_PIN_SDA 21
#endif

#ifndef ST25R3916_PIN_SCL
#define ST25R3916_PIN_SCL 22
#endif

#ifndef ST25R3916_I2C_CLOCK
#define ST25R3916_I2C_CLOCK 400000UL
#endif

#if ST25R3916_USE_I2C
#define ST25R3916_RFAL_HOST_INTERFACE &Wire, ST25R3916_PIN_IRQ
#else
#define ST25R3916_RFAL_HOST_INTERFACE &SPI, ST25R3916_PIN_CS, ST25R3916_PIN_IRQ
#endif

static inline void st25r3916BeginArduinoBus()
{
#if ST25R3916_USE_I2C
#if defined(ARDUINO_ARCH_ESP32)
  Wire.begin(ST25R3916_PIN_SDA, ST25R3916_PIN_SCL);
#else
  Wire.begin();
#endif
  Wire.setClock(ST25R3916_I2C_CLOCK);
#else
#if defined(ARDUINO_ARCH_ESP32)
  SPI.begin(ST25R3916_PIN_SCK, ST25R3916_PIN_MISO, ST25R3916_PIN_MOSI, ST25R3916_PIN_CS);
#else
  SPI.begin();
#endif
#endif
}

static inline const char *st25r3916ArduinoBusName()
{
#if ST25R3916_USE_I2C
  return "host interface=I2C";
#else
  return "host interface=SPI";
#endif
}

#endif
