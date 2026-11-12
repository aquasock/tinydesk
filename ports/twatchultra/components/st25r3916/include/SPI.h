/*
 * SPI.h - the SPI object ST's NFC stack talks through.
 *
 * This is a real implementation, not a stub: every register read and write on
 * the ST25R3916 goes through the two transfer() calls below.
 *
 * TWO THINGS ABOUT HOW THE STACK USES IT, because both look wrong until you
 * know the chip. First, the stack drives chip select itself:
 *
 *     beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE1));
 *     digitalWrite(cs_pin, LOW);
 *     transfer(cmd);
 *     transfer(values, length);
 *     digitalWrite(cs_pin, HIGH);
 *     endTransaction();
 *
 * so the SPI device underneath is created with NO chip select and this port's
 * GPIO 4 is toggled by hand. That is the vendor's own shape and it is also the
 * right one on this board, where CS 4 is shared with nothing but is parked high
 * by the power preinit and must not be released between the command byte and
 * its data.
 *
 * Second, transfer(buf, count) is full duplex in both directions and the stack
 * relies on that: the bytes it sends during a read are meaningless and the
 * bytes it receives are the answer. Arduino's SPI.transfer(buf, count) sends
 * buf and overwrites it with what came back, and that is what this does.
 */
#ifndef SPI_H
#define SPI_H

#include <stddef.h>
#include <stdint.h>

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

class SPISettings
{
  public:
    SPISettings() : clock(1000000), order(1), mode(SPI_MODE0) {}
    SPISettings(uint32_t clk, uint8_t bitOrder, uint8_t dataMode)
        : clock(clk), order(bitOrder), mode(dataMode) {}

    uint32_t clock;
    uint8_t order;
    uint8_t mode;
};

class SPIClass
{
  public:
    /* Which host and clock the bus runs at. Called once before begin(); the
     * pins come from the board configuration in the port, not from here. */
    void configure(int host, uint32_t hz);

    void begin(int sck = -1, int miso = -1, int mosi = -1, int ss = -1);
    void end(void);

    void beginTransaction(SPISettings s);
    void endTransaction(void);

    uint8_t transfer(uint8_t v);
    void transfer(void *buf, size_t count);

    bool ready(void) const;

  private:
    bool start_bus(int sck, int miso, int mosi);
};

extern SPIClass SPI;

#endif /* SPI_H */
