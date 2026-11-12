/*
 * Wire.h - the I2C object the ST25R3916 driver can alternatively use.
 *
 * THIS BOARD DOES NOT USE IT. The reader is on SPI: CS 4, and the SPI bus it
 * shares with the SD card and the LoRa radio. The stack, however, has two
 * constructors and the I2C one is compiled either way, so this class has to
 * exist and has to typecheck.
 *
 * It is a stub ON PURPOSE rather than an I2C implementation that works, for a
 * reason worth writing down: a stub that silently fails is a loud failure at
 * the first transaction, with the pin that does not exist named in the log. A
 * half-working I2C path over pins that are the SPI clock on this board would be
 * a much quieter kind of wrong.
 */
#ifndef WIRE_H
#define WIRE_H

#include <stddef.h>
#include <stdint.h>

class TwoWire
{
  public:
    void begin(int sda = -1, int scl = -1)
    {
        (void)sda;
        (void)scl;
    }

    void setClock(uint32_t hz) { (void)hz; }

    void beginTransmission(uint8_t addr) { (void)addr; }
    size_t write(uint8_t v)
    {
        (void)v;
        return 0;
    }
    uint8_t endTransmission(bool stop = true)
    {
        (void)stop;
        return 4; /* "other error" in the Arduino I2C API: nothing was sent */
    }

    uint8_t requestFrom(uint8_t addr, uint8_t len)
    {
        (void)addr;
        (void)len;
        return 0;
    }

    int read(void) { return -1; }
    int available(void) { return 0; }
};

extern TwoWire Wire;

#endif /* WIRE_H */
