/*
 * ht24lc08.h - HT24LC08 1 KB I2C EEPROM model
 *
 * The Tamagotchi Connection V3 stores its persistent state (name, age, hunger,
 * character, generation) in a 1 KB I2C EEPROM on port D. Without it the ROM
 * still runs -- the game state lives in RAM -- but nothing survives a reset and
 * the device behaves like a factory-fresh unit every boot.
 *
 * Timing is bit-banged by the firmware, so this model reacts to SDA/SCL edges
 * observed on the port rather than owning a clock. That is also why it is fast:
 * it only does work when the firmware toggles a line.
 *
 * Wire format, as the reference emulator implements it:
 *   START, address byte (hw_addr | block<<1 | R/W), ACK
 *   write: word address, ACK, data bytes, ACK each
 *   read:  data bytes, ACK each except the last
 */

#ifndef HT24LC08_H
#define HT24LC08_H

#include <stdint.h>
#include <stddef.h>

#define HT24LC08_SIZE       1024
#define HT24LC08_PAGE_SIZE  256

typedef struct {
    uint8_t  mem[HT24LC08_SIZE];
    uint8_t  hw_addr;       /* 0xA0 >> 0, i.e. the masked address */
    uint8_t  sda_mask, scl_mask;

    uint8_t  state;
    uint8_t  bit_count;
    uint16_t curr_addr;
    uint8_t  scl, sda;
    uint8_t  ack_phase;
    uint8_t  rx_byte, tx_byte;

    /* set when the firmware drives SDA; -1 releases it (external pull-up) */
    int      sda_out_valid;
    int      sda_out_level;
} ht24lc08;

enum { HT_IDLE = 0, HT_ADDR_BYTE = 1, HT_WORD_ADDR = 2, HT_WRITE_DATA = 3, HT_READ_DATA = 4 };

void ht24lc08_init(ht24lc08 *e, uint8_t hw_addr, uint8_t sda_mask, uint8_t scl_mask);
void ht24lc08_load(ht24lc08 *e, const uint8_t *data, size_t len);
void ht24lc08_save(const ht24lc08 *e, uint8_t *out);

/*
 * Feed the new SDA/SCL levels. Returns 1 if the model wants to drive SDA and
 * *level holds the level (0 = drive low, -1 = release). Returns 0 otherwise.
 */
int  ht24lc08_edges(ht24lc08 *e, uint8_t sda, uint8_t scl, int *level);

#endif
