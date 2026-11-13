/*
 * ht24lc08.c - HT24LC08 1 KB I2C EEPROM model
 *
 * Ported from the reference emulator's peripherals/HT24LC08.py so that the
 * firmware sees the same behaviour it was written against -- including the
 * quirks. The caller feeds SDA/SCL levels and applies the returned SDA level.
 */

#include "ht24lc08.h"

/* counters for diagnostics */
unsigned long eeprom_edges_total = 0;
unsigned long eeprom_bits_total = 0;
unsigned long eeprom_writes_total = 0;
unsigned long eeprom_addrs_seen[8] = {0};
#include <string.h>

#define HW_ADDR_MASK 0xF8

void ht24lc08_init(ht24lc08 *e, uint8_t hw_addr, uint8_t sda_mask, uint8_t scl_mask)
{
    memset(e, 0, sizeof(*e));
    e->hw_addr = hw_addr;
    e->sda_mask = sda_mask;
    e->scl_mask = scl_mask;
    e->scl = 1;
    e->sda = 1;
    e->sda_out_valid = 0;
    e->sda_out_level = -1;
}

void ht24lc08_load(ht24lc08 *e, const uint8_t *data, size_t len)
{
    memset(e->mem, 0, sizeof(e->mem));
    if (data && len)
        memcpy(e->mem, data, len < HT24LC08_SIZE ? len : HT24LC08_SIZE);
}

void ht24lc08_save(const ht24lc08 *e, uint8_t *out)
{
    memcpy(out, e->mem, HT24LC08_SIZE);
}

int ht24lc08_edges(ht24lc08 *e, uint8_t sda, uint8_t scl, int *level)
{
    uint8_t prev_scl = e->scl;
    uint8_t prev_sda = e->sda;
    e->scl = scl;
    e->sda = sda;

    *level = -1;
    eeprom_edges_total++;
    if (!e->scl)
        return 0;
    if (!(prev_scl != e->scl || prev_sda != e->sda))
        return 0;

    /* START: SDA falls while SCL is high */
    if (prev_sda > e->sda) {
        e->state = HT_ADDR_BYTE;
        e->bit_count = 0;
        e->ack_phase = 0;
        *level = -1;
        return 1;
    }

    /* STOP: SDA rises while SCL is high */
    if (prev_sda < e->sda) {
        e->state = HT_IDLE;
        e->ack_phase = 0;
        return 0;
    }

    /* SCL rising edge */
    if (e->ack_phase) {
        e->ack_phase = 0;
        *level = 0;                     /* drive SDA low = ACK */
        return 1;
    }

    if (e->state == HT_READ_DATA) {
        if (e->bit_count < 8) {
            /* master samples here; release to send 1, drive low to send 0 */
            *level = ((e->tx_byte >> (7 - e->bit_count)) & 1) ? -1 : 0;
            e->bit_count++;
            return 1;
        }
        e->curr_addr = (uint16_t)((e->curr_addr + 1) % HT24LC08_SIZE);
        e->tx_byte = e->mem[e->curr_addr];
        e->bit_count = 0;
        return 0;
    }

    /* receiving a byte */
    *level = -1;
    e->rx_byte = (uint8_t)((e->rx_byte << 1) | (e->sda > 0));
    e->bit_count++;
    eeprom_bits_total++;
    if (e->bit_count == 8) {
        uint8_t b = e->rx_byte;
        e->bit_count = 0;

        if (e->state == HT_ADDR_BYTE) {
            eeprom_addrs_seen[(b & 0x0E) >> 1]++;
            if ((b & HW_ADDR_MASK) == e->hw_addr) {
                e->ack_phase = 1;
                e->curr_addr = (uint16_t)(((b & ~HW_ADDR_MASK) >> 1) * HT24LC08_PAGE_SIZE
                                          + (e->curr_addr % HT24LC08_PAGE_SIZE));
                if (b & 1) {
                    e->state = HT_READ_DATA;
                    e->tx_byte = e->mem[e->curr_addr];
                } else {
                    e->state = HT_WORD_ADDR;
                }
            } else {
                e->state = HT_IDLE;
            }
        } else if (e->state == HT_WORD_ADDR) {
            e->ack_phase = 1;
            e->curr_addr = (uint16_t)(e->curr_addr - (e->curr_addr % HT24LC08_PAGE_SIZE) + b);
            e->state = HT_WRITE_DATA;
        } else if (e->state == HT_WRITE_DATA) {
            e->ack_phase = 1;
            e->mem[e->curr_addr] = b;
            eeprom_writes_total++;
            e->curr_addr = (uint16_t)((e->curr_addr + 1) % HT24LC08_SIZE);
        }
    }
    return 1;
}
