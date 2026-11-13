/*
 * con_v3_ir.h - Tamagotchi Connection V3 infrared link
 *
 * The V3's only data interface. The firmware bit-bangs a 38 kHz-modulated IR
 * link: it drives the emitter on one port pin and watches a receiver on another,
 * and decodes bytes from the burst/space timing.
 *
 * Two directions:
 *
 *   transmit  the firmware raises and lowers PC bit 2. This model measures how
 *             long each burst lasted and reconstructs bytes from the space
 *             lengths, then hands them to a sink.
 *
 *   receive   bytes pushed in through `con_ir_push` are replayed to the firmware
 *             as timed bursts on PB bit 2, so the ROM's own decoder sees them.
 *
 * Timing is in CPU cycles at 4 MHz, taken from the reference implementation so
 * that the ROM, which was written against real hardware, decodes correctly.
 */

#ifndef CON_V3_IR_H
#define CON_V3_IR_H

#include <stdint.h>
#include <stddef.h>

/* Burst/space durations, in 4 MHz cycles. */
#define IR_BURST_AGC_CYCLES     38232
#define IR_SPACE_AGC_CYCLES      9908
#define IR_BURST_BIT_CYCLES      1768
#define IR_SPACE_BIT0_CYCLES     3088
#define IR_SPACE_BIT1_CYCLES     5508
#define IR_BURST_END_CYCLES      4680
#define IR_BURST_PULSE_CYCLES     104

#define IR_SPACE_BIT_THRESHOLD   ((IR_SPACE_BIT1_CYCLES + IR_SPACE_BIT0_CYCLES) / 2)
#define IR_BURST_PULSE_THRESHOLD (IR_BURST_PULSE_CYCLES + 5)

#define IR_RX_QUEUE 64

enum { IR_RX_IDLE = 0, IR_RX_TRANSMIT = 1, IR_RX_STOP = 2 };

typedef struct {
    /* wiring */
    uint8_t rx_mask;            /* PB bit the firmware watches */
    uint8_t tx_mask;            /* PC bit the firmware drives */

    /* receive side */
    uint8_t  rx_step;
    uint8_t  rx_byte_queue[IR_RX_QUEUE];
    int      rx_queue_len;
    uint8_t  rx_byte;
    uint8_t  rx_bit_count;
    int64_t  rx_burst_counter;   /* CYC_ONE units, same scale as the core */
    int64_t  rx_space_counter;

    /* transmit side */
    uint8_t  prev_pin_state;
    uint8_t  tx_byte;
    uint8_t  tx_bit_count;
    int64_t  tx_burst_counter;   /* CYC_ONE units */
    int64_t  tx_space_counter;

    /* the pin level this model wants on rx_mask: -1 releases, 0/1 drives */
    int      rx_level;
    int      rx_level_pending;

    /* completed transmitted bytes, for the host */
    uint8_t  tx_out[64];
    int      tx_out_len;
} con_ir;

void con_ir_init(con_ir *ir, uint8_t rx_mask, uint8_t tx_mask);

/* Advance the model. Returns 1 if `rx_level` changed and should be applied. */
int  con_ir_clock(con_ir *ir, int64_t cycles);

/* The firmware toggled its transmit pin to this level. */
void con_ir_pin(con_ir *ir, uint8_t pin_state);

/* Queue bytes to deliver to the firmware. Returns number queued. */
int  con_ir_push(con_ir *ir, const uint8_t *data, int len);

/* Pop one transmitted byte. Returns -1 when empty. */
int  con_ir_pop_tx(con_ir *ir);

#endif
