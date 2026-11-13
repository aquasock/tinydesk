/*
 * con_v3_ir.c - Tamagotchi Connection V3 infrared link
 *
 * Ported from the reference emulator's peripherals/CON_V3_IR.py, including its
 * quirks, so that the ROM sees the timing it was written for.
 */

#include "con_v3_ir.h"
#include "splb32.h"
#include <string.h>

unsigned long ir_clock_calls = 0;
unsigned long ir_pin_calls = 0;
unsigned long ir_push_calls = 0;

void con_ir_init(con_ir *ir, uint8_t rx_mask, uint8_t tx_mask)
{
    memset(ir, 0, sizeof(*ir));
    ir->rx_mask = rx_mask;
    ir->tx_mask = tx_mask;
    ir->rx_level = 0;
    ir->rx_level_pending = 0;
    ir->prev_pin_state = 0;
}

int con_ir_clock(con_ir *ir, int64_t cycles)
{
    int changed = 0;
    ir_clock_calls++;

    ir->tx_space_counter += cycles;

    if (ir->rx_step != IR_RX_IDLE) {
        ir->rx_burst_counter += cycles;
        if (ir->rx_burst_counter >= 0) {
            /* burst finished: receiver idles high */
            ir->rx_level = 1;
            ir->rx_level_pending = 1;
            changed = 1;

            ir->rx_space_counter += cycles;
            if (ir->rx_step == IR_RX_STOP) {
                ir->rx_step = IR_RX_IDLE;
            } else if (ir->rx_space_counter >= 0) {
                ir->rx_level = 0;
                changed = 1;

                if (ir->rx_bit_count == 8) {
                    ir->rx_bit_count = 0;
                    if (ir->rx_queue_len > 0) {
                        ir->rx_byte = ir->rx_byte_queue[0];
                        memmove(ir->rx_byte_queue, ir->rx_byte_queue + 1,
                                (size_t)(ir->rx_queue_len - 1));
                        ir->rx_queue_len--;
                    } else {
                        ir->rx_burst_counter = -((int64_t)IR_BURST_END_CYCLES * CYC_ONE);
                        ir->rx_step = IR_RX_STOP;
                        return changed;
                    }
                }

                ir->rx_burst_counter = -((int64_t)IR_BURST_BIT_CYCLES * CYC_ONE);
                if (ir->rx_byte & 0x01)
                    ir->rx_space_counter = -((int64_t)IR_SPACE_BIT1_CYCLES * CYC_ONE);
                else
                    ir->rx_space_counter = -((int64_t)IR_SPACE_BIT0_CYCLES * CYC_ONE);

                ir->rx_byte >>= 1;
                ir->rx_bit_count++;
            }
        }
    }
    return changed;
}

void con_ir_pin(con_ir *ir, uint8_t pin_state)
{
    ir_pin_calls++;
    if (ir->prev_pin_state == pin_state)
        return;
    ir->prev_pin_state = pin_state;

    if (!(pin_state & ir->tx_mask))
        return;

    if (ir->tx_space_counter < (int64_t)IR_BURST_PULSE_THRESHOLD * CYC_ONE) {
        /* short gap inside a bit: accumulate, do not complete a byte */
        ir->tx_burst_counter += ir->tx_space_counter;
    } else {
        if (ir->tx_burst_counter < (int64_t)IR_BURST_END_CYCLES * CYC_ONE) {
            ir->tx_byte >>= 1;
            if (ir->tx_space_counter > (int64_t)IR_SPACE_BIT_THRESHOLD * CYC_ONE)
                ir->tx_byte |= 0x80;

            ir->tx_bit_count++;
            if (ir->tx_bit_count == 8) {
                if (ir->tx_out_len < (int)sizeof(ir->tx_out))
                    ir->tx_out[ir->tx_out_len++] = ir->tx_byte & 0xFF;
                ir->tx_bit_count = 0;
            }
        } else {
            ir->tx_bit_count = 0;
        }
        ir->tx_burst_counter = 0;
    }
    ir->tx_space_counter = 0;
}

int con_ir_push(con_ir *ir, const uint8_t *data, int len)
{
    int queued = 0;
    for (int i = 0; i < len; i++) {
        if (ir->rx_queue_len >= IR_RX_QUEUE)
            break;
        ir->rx_byte_queue[ir->rx_queue_len++] = data[i];
        queued++;
    }

    if (ir->rx_step == IR_RX_IDLE && ir->rx_queue_len > 0) {
        ir->rx_level = 0;
        ir->rx_byte = ir->rx_byte_queue[0];
        memmove(ir->rx_byte_queue, ir->rx_byte_queue + 1,
                (size_t)(ir->rx_queue_len - 1));
        ir->rx_queue_len--;

        ir->rx_bit_count = 0;
        ir->rx_burst_counter = -((int64_t)IR_BURST_AGC_CYCLES * CYC_ONE);
        ir->rx_space_counter = -((int64_t)IR_SPACE_AGC_CYCLES * CYC_ONE);
        ir->rx_step = IR_RX_TRANSMIT;
    }
    return queued;
}

int con_ir_pop_tx(con_ir *ir)
{
    if (ir->tx_out_len <= 0)
        return -1;
    int v = ir->tx_out[0];
    memmove(ir->tx_out, ir->tx_out + 1, (size_t)(ir->tx_out_len - 1));
    ir->tx_out_len--;
    return v;
}
