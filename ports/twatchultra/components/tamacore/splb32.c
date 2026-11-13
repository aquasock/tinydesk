/*
 * splb32.c - Sunplus SPLB32 CPU core
 *
 * Semantics follow the BrickEmuPy reference implementation (cores/SPLB32.py)
 * including its quirks, because that core runs the real ROM. Where the Python
 * and a stock 6502 differ, the Python wins.
 *
 * Cycle counts are per instruction and are the raw values the reference
 * returns; the CPU clock divider (SFR $04, default 2) is applied on top, so a
 * "2 cycle" instruction costs 8 emulated cycles at the default divider. That
 * matches the reference and therefore matches the ROM's timing.
 */

#include "splb32.h"
#include "ht24lc08.h"
#include "con_v3_ir.h"
#include "splb32_sound.h"
#include <stdlib.h>
#include <string.h>

#ifdef SPLB32_PROF
uint64_t splb32_prof_steps, splb32_prof_irqs, splb32_prof_sleep_steps;
#define PROF_STEP()      (splb32_prof_steps++)
#define PROF_IRQ()       (splb32_prof_irqs++)
#define PROF_SLEEP()     (splb32_prof_sleep_steps++)
#else
#define PROF_STEP()      do { } while (0)
#define PROF_IRQ()       do { } while (0)
#define PROF_SLEEP()     do { } while (0)
#endif

/* ---- memory map ------------------------------------------------------- */
#define SFR_BASE        0x0000
#define SFR_END         0x003F
#define RAM_BASE        0x0040
#define RAM_END         0x04FF          /* 0x40 + 0x4C0 */
#define DPRAM_BASE      0x3E00
#define DPRAM_END       0x3FFF
#define ROM_BANK_LO     0x4000
#define ROM_BANK_0      0xC000

#define SP_PAGE         0x0100

/* ---- SFR addresses ---------------------------------------------------- */
#define SFR_BANK_SEL    0x00
#define SFR_INT_CTRL    0x01
#define SFR_INT_CLR     0x02
#define SFR_CLK_CPU     0x04
#define SFR_PA_ATTR     0x06
#define SFR_PA_DIR      0x07
#define SFR_PA_DATA     0x08
#define SFR_PD_DATA     0x09
#define SFR_PB_DATA     0x0A
#define SFR_PC_DATA     0x0B
#define SFR_TIMEBASE    0x0C
#define SFR_CLK32768    0x0E
#define SFR_TIMER_CTRL  0x0F
#define SFR_TM0L        0x10
#define SFR_TM0H        0x11
#define SFR_AUD0_CTRL   0x12
#define SFR_AUD0_DATA   0x13
#define SFR_TM1L        0x14
#define SFR_TM1H        0x15
#define SFR_AUD1_CTRL   0x16
#define SFR_AUD1_DATA   0x17
#define SFR_WAKEUP_CTRL 0x18
#define SFR_LCD_CTRL1   0x24
#define SFR_PB_ATTR     0x28
#define SFR_PB_DIR      0x29
#define SFR_PC_ATTR     0x2A
#define SFR_PC_DIR      0x2B
#define SFR_PD_ATTR     0x2C
#define SFR_PD_DIR      0x2D
#define SFR_NMI_CTRL    0x38
#define SFR_BYTE_MIRROR 0x3E
#define SFR_NIBBLE_SWAP 0x3F

/* ---- timer / interrupt bits ------------------------------------------- */
#define TCTRL_EN        0x80
#define TCTRL_TM0_MASK  0x60
#define TCTRL_TM0_STOP  0x00
#define TCTRL_TM0_SYS   0x20
#define TCTRL_TM0_32K   0x40
#define TCTRL_TM1_MASK  0x03
#define TCTRL_TM1_STOP  0x00
#define TCTRL_TM1_SYS   0x01
#define TCTRL_TM1_32K   0x02

#define IRQ_EXT         0x01
#define IRQ_TIMEBASEL   0x04
#define IRQ_TIMEBASEH   0x08
#define IRQ_TIMER0      0x10
#define IRQ_TIMER1      0x20
#define IRQ_UART_TX     0x40
#define IRQ_UART_RX     0x80

#define WK_EXT          0x01
#define WK_TIMEBASEL    0x02
#define WK_TIMER0       0x04
#define WK_TIMEBASEH    0x08
#define WK_TIMER1       0x10
#define WK_UART_RX      0x20

#define NMI_ENABLE      0x80
#define NMI_TIMER1      0x02

#define TB_L_MASK       0x0C
#define TB_L_SHIFT      2
#define TB_H_MASK       0x03
#define TB_H_TBL_INIT   { 0x07, 0x01, 0x03, 0x00 }
#define TB_L_TBL_INIT   { 0x1FF, 0x7F, 0xFF, 0x3F }

#define CLK32768_EN     0x80
#define CPU_CTRL_DEFAULT 0x02

/* ---- cpu -------------------------------------------------------------- */
struct splb32 {
    uint32_t opcode_hits[256];   /* coverage instrumentation, cleared on reset */
    const uint8_t *rom;             /* borrowed, not copied (512 KB) */
    uint8_t  ram[0x4C0];
    uint8_t  dpram[0x200];
    uint8_t  sfr[0x40];

    /* 32-bit, not 16. The reference's _rts does `self._PC += 1` with no mask,
     * so PC legitimately exceeds 0xFFFF after an RTS from a wrecked stack.
     * _get_rom_addr still reports a low address, and the next instruction is
     * fetched from ROM - a reference bug that is observable, so it is
     * reproduced rather than corrected. */
    uint32_t pc;
    uint8_t  a, x, y, sp;

    uint8_t  nf, vf, bf, df, if_, zf, cf;

    uint32_t rom_bank;
    uint8_t  cpu_enbl;
    uint8_t  cpu_div;
    uint8_t  clk32768_en;

    /*
     * Counters are fixed-point, CYC_SHIFT fractional bits.
     *
     * The reference keeps these as doubles because the sleep tick is
     * 122.0703125 cycles and truncating that to an integer drifts by several
     * hundred counts over a few emulated minutes. 122.0703125 is exactly
     * 15625/128, so at CYC_SHIFT=7 bits the same value is representable as an
     * integer and no drift is possible -- this is not an approximation.
     *
     * It matters because this function runs per instruction and the target has
     * no hardware double.
     */
    int64_t  tm0_cnt, tm1_cnt, t1khz_cnt;   /* CYC_ONE units */
    int64_t  tm0, tm1;                      /* CYC_ONE units */
    uint32_t tm0_reload, tm1_reload, t1khz;
    uint8_t  timer_ctrl, time_base;

    uint8_t  wakeup_ctrl, wakeupreq;
    uint8_t  int_ctrl, ireq;
    uint8_t  nmi_ctrl, nmireq;

    uint8_t  lcd_ctrl1;
    uint8_t  aud0_ctrl, aud0_data, aud1_ctrl, aud1_data;
    uint8_t  pdir[4], pattr[4], platch[4];
    uint8_t  pin_in_lo[4], pin_in_hi[4];   /* external drive: [0]=low level seen, [1]=high level seen */
    uint8_t  pullup_ext[4];                /* mask of externally pulled-up pins */

    int64_t  sub_clock_div;    /* one sleep tick, in CYC_ONE units */

    uint16_t addr_reset, addr_irq, addr_nmi;

    ht24lc08 *eeprom;        /* optional; NULL when not attached */
    con_ir   *ir;            /* optional; NULL when not attached */
    splb32_sound sound;      /* always present; silent unless driven */

    uint8_t  page_crossed;   /* set by addressing modes that can cross a page */
    uint8_t  branch_taken;   /* set by real branches */

    uint64_t instrs;
    uint64_t cycles;         /* whole cycles; cycles_units >> CYC_SHIFT */
    int64_t  cycles_units;   /* exact accumulator, CYC_ONE units */
};

static const uint16_t tb_h_tbl[4] = TB_H_TBL_INIT;
static const uint16_t tb_l_tbl[4] = TB_L_TBL_INIT;

/* ---- flags ------------------------------------------------------------ */
static inline uint8_t get_ps(const splb32 *c)
{
    return (uint8_t)((c->nf << 7) | (c->vf << 6) | (c->bf << 4) |
                     (c->df << 3) | (c->if_ << 2) | (c->zf << 1) | c->cf);
}

static inline void set_ps(splb32 *c, uint8_t ps)
{
    c->nf = (ps >> 7) & 1;
    c->vf = (ps >> 6) & 1;
    c->bf = (ps >> 4) & 1;
    c->df = (ps >> 3) & 1;
    c->if_ = (ps >> 2) & 1;
    c->zf = (ps >> 1) & 1;
    c->cf = ps & 1;
}

/* ---- ports -------------------------------------------------------------
 * Faithful port of the reference's _port_read:
 *
 *   (~PDIR & (in_hi | (pullup_ext & ~in_lo))) |
 *   (PDIR  & PLATCH & (pullup_ext | ~ATTR))
 *
 * Note this means a port configured as INPUT reads its external drive state,
 * not the latch you last wrote. Code that writes to a data register and reads
 * it back gets different answers depending on PDIR -- which the fuzzer found.
 * ---------------------------------------------------------------------- */
static uint8_t port_read(const splb32 *c, int p)
{
    uint8_t pdir = c->pdir[p], attr = c->pattr[p], latch = c->platch[p];
    uint8_t in_lo = c->pin_in_lo[p], in_hi = c->pin_in_hi[p], pu = c->pullup_ext[p];
    return (uint8_t)(((~pdir) & (in_hi | (pu & (uint8_t)~in_lo))) |
                     (pdir & latch & (pu | (uint8_t)~attr))) & 0xFF;
}

/* ---- SFR get/set ------------------------------------------------------ */
static uint8_t sfr_get(splb32 *c, uint8_t a)
{
    switch (a) {
    case SFR_BANK_SEL:  return (uint8_t)(c->rom_bank >> 15);
    case SFR_INT_CTRL:  return c->ireq;
    case SFR_CLK_CPU:   return c->cpu_div;
    case SFR_PA_DATA:   return port_read(c, 0);
    case SFR_PB_DATA:   return port_read(c, 1);
    case SFR_PC_DATA:   return port_read(c, 2);
    case SFR_PD_DATA:   return port_read(c, 3);
    case SFR_TIMEBASE:
        /* Reference quirk: the getter for SFR $0C returns the 1 kHz counter
         * (_T1KHZ), while the setter writes the timebase selector
         * (_TIME_BASE). A read/write asymmetry in the reference. Reproduced
         * deliberately: it is observable by any code that reads $0C, and the
         * fuzzer caught the discrepancy. */
        return (uint8_t)c->t1khz;
    case SFR_TIMER_CTRL:return c->timer_ctrl;
    /* The guest sees a 16-bit CYCLE counter, so the fixed-point fraction has
     * to be shifted off here. Without this the firmware reads a value 128x
     * too large -- the trace matched for the first 400k instructions and
     * diverged once the ROM first polled a timer. */
    case SFR_TM0L:      return (uint8_t)((uint32_t)(c->tm0 >> CYC_SHIFT) & 0xFF);
    case SFR_TM0H:      return (uint8_t)(((uint32_t)(c->tm0 >> CYC_SHIFT) >> 8) & 0xFF);
    case SFR_TM1L:      return (uint8_t)((uint32_t)(c->tm1 >> CYC_SHIFT) & 0xFF);
    case SFR_TM1H:      return (uint8_t)(((uint32_t)(c->tm1 >> CYC_SHIFT) >> 8) & 0xFF);
    case SFR_AUD0_CTRL: return c->aud0_ctrl;
    case SFR_AUD0_DATA: return c->aud0_data;
    case SFR_AUD1_CTRL: return c->aud1_ctrl;
    case SFR_AUD1_DATA: return c->aud1_data;
    case SFR_WAKEUP_CTRL: return c->wakeupreq;
    case SFR_LCD_CTRL1: return c->lcd_ctrl1;
    case SFR_NMI_CTRL:
        /* Reference returns a composite: pending requests ORed with the
         * enable bit, not the raw control register. Returning the raw value
         * makes the ROM read the wrong thing and branch differently. */
        return (uint8_t)(c->nmireq | (c->nmi_ctrl & 0x80));
    case SFR_BYTE_MIRROR: return c->sfr[SFR_BYTE_MIRROR];
    case SFR_NIBBLE_SWAP: return c->sfr[SFR_NIBBLE_SWAP];
    default:            return 0;
    }
}

static void sfr_set(splb32 *c, uint8_t a, uint8_t v)
{
    switch (a) {
    case SFR_BANK_SEL:  c->rom_bank = (uint32_t)(v & 0x0F) << 15; break;
    case SFR_INT_CTRL:  c->int_ctrl = v; c->ireq = 0; break;
    case SFR_INT_CLR:   c->ireq &= (uint8_t)~v; break;
    case SFR_CLK_CPU:
        if ((v & 0x07) == 0x07) c->cpu_enbl = 0;
        else c->cpu_div = v & 0x07;
        break;
    case SFR_PA_DIR:    c->pdir[0] = v; break;
    case SFR_PB_DIR:
        /* The reference expands the four written bits into eight direction
         * bits: each written bit sets a pair of adjacent enables. Storing the
         * value raw makes PB reads disagree with the reference and changes the
         * ROM's configuration decisions (NMI_CTRL, timer behaviour). */
        c->pdir[1] = (uint8_t)(((v & 0x1) * 0x3) | ((v & 0x2) * 0x6) |
                               ((v & 0x4) * 0xC) | ((v & 0x8) * 0x18));
        break;
    case SFR_PC_DIR:    c->pdir[2] = v; break;
    case SFR_PD_DIR:    c->pdir[3] = v; break;
    case SFR_PA_ATTR:   c->pattr[0] = v; break;
    case SFR_PB_ATTR:   c->pattr[1] = v; break;
    case SFR_PC_ATTR:   c->pattr[2] = v; break;
    case SFR_PD_ATTR:   c->pattr[3] = v; break;
    case SFR_PA_DATA:   c->platch[0] = v; break;
    case SFR_PB_DATA:   c->platch[1] = v; break;
    case SFR_PC_DATA:
        c->platch[2] = v;
        if (c->ir) con_ir_pin(c->ir, (uint8_t)port_read(c, 2));
        break;
    case SFR_PD_DATA:
        c->platch[3] = v & 0x3F;   /* SFR_IO_PORTD_MASK in the reference */
        if (c->eeprom) {
            int level;
            /* the EEPROM sees the pin values, not the latch */
            uint8_t pins = port_read(c, 3);
            if (ht24lc08_edges(c->eeprom,
                               (pins & c->eeprom->sda_mask) ? 1 : 0,
                               (pins & c->eeprom->scl_mask) ? 1 : 0,
                               &level)) {
                splb32_pin(c, 3, c->eeprom->sda_mask, level);
            }
        }
        break;
    case SFR_TIMEBASE:  c->time_base = v; break;
    case SFR_CLK32768:  c->clk32768_en = v & CLK32768_EN; break;
    case SFR_TIMER_CTRL: c->timer_ctrl = v; break;
    case SFR_TM0L:      c->tm0_reload = (c->tm0_reload & 0xFF00u) | v; break;
    case SFR_TM0H:      c->tm0_reload = (c->tm0_reload & 0x00FFu) | ((uint32_t)v << 8);
                        c->tm0 = (int64_t)c->tm0_reload * CYC_ONE; break;
    case SFR_TM1L:      c->tm1_reload = (c->tm1_reload & 0xFF00u) | v; break;
    case SFR_TM1H:      c->tm1_reload = (c->tm1_reload & 0x00FFu) | ((uint32_t)v << 8);
                        c->tm1 = (int64_t)c->tm1_reload * CYC_ONE; break;
    case SFR_AUD0_CTRL: c->aud0_ctrl = v; break;
    case SFR_AUD0_DATA: c->aud0_data = v;
        splb32_sound_set_data(&c->sound, 0, c->aud0_ctrl, v); break;
    case SFR_AUD1_CTRL: c->aud1_ctrl = v; break;
    case SFR_AUD1_DATA: c->aud1_data = v;
        splb32_sound_set_data(&c->sound, 1, c->aud1_ctrl, v); break;
    case SFR_WAKEUP_CTRL: c->wakeup_ctrl = v; c->wakeupreq &= v; break;
    case SFR_LCD_CTRL1: c->lcd_ctrl1 = v; break;
    case SFR_NMI_CTRL:  c->nmi_ctrl = v; c->nmireq &= v; break;
    case SFR_BYTE_MIRROR: {
        uint8_t b = (uint8_t)(((v & 0xF0) >> 4) | ((v & 0x0F) << 4));
        b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
        c->sfr[SFR_BYTE_MIRROR] = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
        break;
    }
    case SFR_NIBBLE_SWAP:
        c->sfr[SFR_NIBBLE_SWAP] = (uint8_t)(((v & 0x0F) << 4) | ((v & 0xF0) >> 4));
        break;
    default: break;
    }
}

/* ---- bus -------------------------------------------------------------- */
#ifdef SPLB32_PROF
uint64_t splb32_prof_reads, splb32_prof_writes, splb32_prof_rom,
         splb32_prof_ram, splb32_prof_dpram, splb32_prof_sfr;
#endif

static inline uint8_t read8(splb32 *c, uint32_t addr)
{
#ifdef SPLB32_PROF
    splb32_prof_reads++;
#endif
#ifdef SPLB32_PROF
    if (addr >= ROM_BANK_0)       splb32_prof_rom++;
    else if (addr >= ROM_BANK_LO) splb32_prof_rom++;
    else if (addr >= DPRAM_BASE)  splb32_prof_dpram++;
    else if (addr >= RAM_BASE && addr <= RAM_END) splb32_prof_ram++;
    else if (addr <= SFR_END)     splb32_prof_sfr++;
#endif
    if (addr >= ROM_BANK_0)      return c->rom[addr & 0x7FFF];
    if (addr >= ROM_BANK_LO)     return c->rom[(addr & 0x7FFF) | c->rom_bank];
    if (addr >= DPRAM_BASE)      return c->dpram[addr - DPRAM_BASE];
    if (addr >= RAM_BASE && addr <= RAM_END) return c->ram[addr - RAM_BASE];
    if (addr <= SFR_END)         return sfr_get(c, (uint8_t)addr);
    return 0;
}

static inline void write8(splb32 *c, uint32_t addr, uint8_t v)
{
#ifdef SPLB32_PROF
    splb32_prof_writes++;
#endif
    if (addr >= ROM_BANK_0)      return;                       /* ROM */
    if (addr >= ROM_BANK_LO)     return;                       /* ROM */
    if (addr >= DPRAM_BASE) {
        if (addr <= DPRAM_END) c->dpram[addr - DPRAM_BASE] = v;
        return;
    }
    if (addr >= RAM_BASE && addr <= RAM_END) { c->ram[addr - RAM_BASE] = v; return; }
    if (addr <= SFR_END) { sfr_set(c, (uint8_t)addr, v); return; }
}

/* ---- ALU -------------------------------------------------------------- */
static inline void do_ora(splb32 *c, uint8_t v) { c->a |= v; c->nf = (c->a >> 7) & 1; c->zf = (c->a == 0); }
static inline void do_and(splb32 *c, uint8_t v) { c->a &= v; c->nf = (c->a >> 7) & 1; c->zf = (c->a == 0); }
static inline void do_eor(splb32 *c, uint8_t v) { c->a ^= v; c->nf = (c->a >> 7) & 1; c->zf = (c->a == 0); }

static inline void do_adc(splb32 *c, uint8_t operand)
{
    uint8_t A = c->a;
    int32_t nv = (int32_t)A + operand + c->cf;

    if (c->df && (((A & 0x0F) + (operand & 0x0F) + c->cf) > 9)) nv += 6;

    c->vf = (uint8_t)(((~(A ^ operand) & (A ^ (uint8_t)nv)) >> 7) & 1);
    c->nf = (uint8_t)(((uint8_t)nv >> 7) & 1);

    if (c->df && (nv > 0x99)) nv += 0x60;

    c->zf = ((nv & 0xFF) == 0);
    c->cf = (nv > 255);
    c->a = (uint8_t)(nv & 0xFF);
}

static inline void do_sbc(splb32 *c, uint8_t operand)
{
    uint8_t A = c->a;
    int32_t b = c->cf ? 0 : 1;
    int32_t nv = (int32_t)A - operand - b;

    if (c->df) {
        if (((A & 0x0F) - (operand & 0x0F) - b) < 0) nv -= 6;
        if (nv < 0) nv -= 0x60;
    }

    c->vf = (uint8_t)((((A ^ operand) & (A ^ (uint8_t)nv)) >> 7) & 1);
    c->nf = (uint8_t)(((uint8_t)nv >> 7) & 1);
    c->zf = ((nv & 0xFF) == 0);
    c->cf = (nv >= 0);
    c->a = (uint8_t)(nv & 0xFF);
}

static inline void do_cmp(splb32 *c, uint8_t reg, uint8_t v)
{
    int32_t r = (int32_t)reg - v;
    c->cf = (r >= 0);
    c->zf = ((r & 0xFF) == 0);
    c->nf = (uint8_t)(((uint8_t)r >> 7) & 1);
}

static inline uint8_t do_asl(splb32 *c, uint8_t v) { c->cf = (v >> 7) & 1; v = (uint8_t)(v << 1); c->nf = (v >> 7) & 1; c->zf = (v == 0); return v; }
static inline uint8_t do_lsr(splb32 *c, uint8_t v) { c->cf = v & 1; v = (uint8_t)(v >> 1); c->nf = 0; c->zf = (v == 0); return v; }

static inline uint8_t do_rol(splb32 *c, uint8_t v)
{
    uint8_t carry_in = c->cf;
    c->cf = (v >> 7) & 1;
    v = (uint8_t)((v << 1) | carry_in);
    c->nf = (v >> 7) & 1; c->zf = (v == 0);
    return v;
}

/* ROR, matching the reference exactly.
 *
 * The reference computes the zero flag as `not(prev & 0xFE | CF)` -- the OLD
 * value's upper bits, not the rotated result. That is a bug, but it is what the
 * ROM was timed against, so it is reproduced. The negative flag is set from the
 * old carry, which is genuinely correct (it becomes bit 7 of the result).
 */
static inline uint8_t do_ror(splb32 *c, uint8_t v)
{
    uint8_t old_cf = c->cf;
    uint8_t result = (uint8_t)((v >> 1) | (old_cf << 7));
    c->nf = old_cf;
    c->zf = (((v & 0xFE) | old_cf) == 0);
    c->cf = v & 1;
    return result;
}

/* ---- stack / interrupts ----------------------------------------------- */
static inline void push(splb32 *c, uint8_t v)
{
    write8(c, (uint32_t)(SP_PAGE | c->sp), v);
    c->sp = (uint8_t)((c->sp - 1) & 0xFF);
}
static inline uint8_t pull(splb32 *c)
{
    c->sp = (uint8_t)((c->sp + 1) & 0xFF);
    return read8(c, (uint32_t)(SP_PAGE | c->sp));
}

static void do_irq(splb32 *c)
{
    PROF_IRQ();
    push(c, (uint8_t)(c->pc >> 8));
    push(c, (uint8_t)(c->pc & 0xFF));
    push(c, get_ps(c));
    c->if_ = 1;
    c->pc = c->addr_irq;
}

static void do_nmi(splb32 *c)
{
    if (!(c->nmi_ctrl & NMI_ENABLE)) return;
    if (!c->cpu_enbl) return;
    push(c, (uint8_t)(c->pc >> 8));
    push(c, (uint8_t)(c->pc & 0xFF));
    push(c, get_ps(c));
    c->pc = c->addr_nmi;
}

/* ---- timers ----------------------------------------------------------- */
static void timers_clock(splb32 *c, int64_t exec_units)
{
    if (c->timer_ctrl & TCTRL_TM0_MASK) {
        c->tm0_cnt -= exec_units;
        while (c->tm0_cnt <= 0) {
            if ((c->timer_ctrl & TCTRL_TM0_MASK) == TCTRL_TM0_SYS) {
                c->tm0_cnt = CYC_ONE; c->tm0 += exec_units;
            } else if ((c->timer_ctrl & TCTRL_TM0_MASK) == TCTRL_TM0_32K) {
                c->tm0_cnt += c->sub_clock_div; c->tm0 += CYC_ONE;
            } else break;
            while (c->tm0 > (int64_t)0xFFFF * CYC_ONE) {
                c->tm0 -= ((int64_t)0x10000u - (int64_t)c->tm0_reload) * CYC_ONE;
                c->ireq |= IRQ_TIMER0;
                c->wakeupreq |= WK_TIMER0;

                if ((c->aud0_ctrl & AUD_CTRL_TM0_OVFLW) && (c->aud0_ctrl & AUD_CTRL_TONE))
                    splb32_sound_toggle(&c->sound, 0);
                if ((c->aud1_ctrl & AUD_CTRL_TM0_OVFLW) && (c->aud1_ctrl & AUD_CTRL_TONE))
                    splb32_sound_toggle(&c->sound, 1);
            }
        }
    }

    if (c->timer_ctrl & TCTRL_TM1_MASK) {
        c->tm1_cnt -= exec_units;
        while (c->tm1_cnt <= 0) {
            if ((c->timer_ctrl & TCTRL_TM1_MASK) == TCTRL_TM1_SYS) {
                c->tm1_cnt = CYC_ONE; c->tm1 += exec_units;
            } else if ((c->timer_ctrl & TCTRL_TM1_MASK) == TCTRL_TM1_32K) {
                c->tm1_cnt += c->sub_clock_div; c->tm1 += CYC_ONE;
            } else break;
            while (c->tm1 > (int64_t)0xFFFF * CYC_ONE) {
                c->tm1 -= ((int64_t)0x10000u - (int64_t)c->tm1_reload) * CYC_ONE;
                c->ireq |= IRQ_TIMER1;
                c->wakeupreq |= WK_TIMER1;
                c->nmireq |= NMI_TIMER1;
                if (c->nmi_ctrl & NMI_TIMER1) do_nmi(c);

                if ((c->aud0_ctrl & AUD_CTRL_TM1_OVFLW) && (c->aud0_ctrl & AUD_CTRL_TONE))
                    splb32_sound_toggle(&c->sound, 0);
                if ((c->aud1_ctrl & AUD_CTRL_TM1_OVFLW) && (c->aud1_ctrl & AUD_CTRL_TONE))
                    splb32_sound_toggle(&c->sound, 1);
            }
        }
    }

    c->t1khz_cnt -= exec_units;
    while (c->t1khz_cnt <= 0) {
        c->t1khz_cnt += c->sub_clock_div * (SPLB32_SUB_CLOCK / 1024);
        c->t1khz++;

        uint8_t h = c->time_base & TB_H_MASK;
        if (!(c->t1khz & tb_h_tbl[h])) {
            c->ireq |= IRQ_TIMEBASEH;
            c->wakeupreq |= WK_TIMEBASEH;
        }
        uint8_t l = (c->time_base & TB_L_MASK) >> TB_L_SHIFT;
        if (!(c->t1khz & tb_l_tbl[l])) {
            c->ireq |= IRQ_TIMEBASEL;
            c->wakeupreq |= WK_TIMEBASEL;
        }
    }
}

/* ---- cycle tables ------------------------------------------------------
 * Generated from the BrickEmuPy reference core, which is what the ROM was
 * tuned against. Several entries are irregular (AND zp,X costs 6 where
 * ORA zp,X costs 3); they are reproduced deliberately so that timing matches
 * the reference exactly rather than being arch-correct.
 * ---------------------------------------------------------------------- */
static const uint8_t cyc_base[256] = {
   7, 6, 2, 2, 2, 3, 5, 2, 3, 2, 2, 2, 2, 4, 6, 2,
   2, 5, 2, 2, 2, 3, 6, 2, 2, 4, 2, 2, 2, 4, 6, 2,
   6, 6, 2, 2, 3, 3, 5, 2, 4, 2, 2, 2, 4, 4, 5, 2,
   2, 5, 2, 2, 2, 6, 6, 2, 2, 4, 2, 2, 2, 4, 6, 2,
   6, 4, 2, 2, 2, 3, 5, 2, 3, 2, 2, 2, 3, 4, 6, 2,
   2, 5, 2, 2, 2, 4, 6, 2, 2, 4, 2, 2, 2, 4, 6, 2,
   6, 6, 2, 2, 2, 3, 5, 2, 4, 2, 2, 2, 6, 4, 6, 2,
   2, 5, 2, 2, 2, 5, 6, 2, 2, 4, 2, 2, 2, 4, 6, 2,
   2, 6, 2, 2, 3, 3, 3, 2, 2, 2, 2, 2, 4, 4, 4, 2,
   2, 6, 2, 2, 4, 4, 4, 2, 2, 5, 2, 2, 2, 5, 2, 2,
   2, 6, 2, 2, 3, 3, 3, 2, 2, 2, 2, 2, 4, 4, 4, 2,
   2, 5, 2, 2, 4, 4, 4, 2, 2, 4, 2, 2, 4, 4, 4, 2,
   2, 6, 2, 2, 3, 3, 5, 2, 2, 2, 2, 2, 3, 4, 5, 2,
   2, 5, 2, 2, 2, 4, 6, 2, 2, 4, 2, 2, 2, 4, 7, 2,
   2, 6, 2, 2, 3, 3, 5, 2, 2, 2, 2, 2, 4, 4, 6, 2,
   2, 4, 2, 2, 2, 4, 6, 2, 2, 4, 2, 2, 2, 4, 7, 2,
};
static const uint8_t cyc_page[256] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,1,0,0,0,0,0,0,0,1,0,0,0,1,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,1,0,0,0,0,0,0,0,1,0,0,0,1,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,1,0,0,0,0,0,0,0,1,0,0,0,1,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,1,0,0,0,0,0,0,0,1,0,0,1,1,1,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0,
};

#define PAGE_PENALTY 1

/* ---- addressing modes -------------------------------------------------- */
static inline uint32_t am_zp(splb32 *c)   { return read8(c, (uint32_t)c->pc++); }
static inline uint32_t am_zpx(splb32 *c)  { return (uint32_t)((read8(c, (uint32_t)c->pc++) + c->x) & 0xFF); }
static inline uint32_t am_zpy(splb32 *c)  { return (uint32_t)((read8(c, (uint32_t)c->pc++) + c->y) & 0xFF); }

static inline uint32_t am_abs(splb32 *c)
{
    uint32_t a = read8(c, c->pc) | ((uint32_t)read8(c, (uint16_t)((c->pc + 1) & 0xFFFF)) << 8);
    c->pc = (uint16_t)((c->pc + 2) & 0xFFFF);
    return a;
}
static inline uint32_t am_absx(splb32 *c)
{
    uint32_t base = am_abs(c);
    uint32_t addr = (base + c->x) & 0xFFFF;
    c->page_crossed = ((base ^ addr) > 255u);
    return addr;
}
static inline uint32_t am_absy(splb32 *c)
{
    uint32_t base = am_abs(c);
    uint32_t addr = (base + c->y) & 0xFFFF;
    c->page_crossed = ((base ^ addr) > 255u);
    return addr;
}

static inline uint32_t am_indx(splb32 *c)
{
    uint32_t z = (uint32_t)((read8(c, (uint32_t)c->pc++) + c->x) & 0xFF);
    return read8(c, z) | ((uint32_t)read8(c, (z + 1) & 0xFF) << 8);
}
static inline uint32_t am_indy(splb32 *c)
{
    uint32_t z = read8(c, (uint32_t)c->pc++);
    uint32_t base = read8(c, z) | ((uint32_t)read8(c, (z + 1) & 0xFF) << 8);
    uint32_t addr = (base + c->y) & 0xFFFF;
    c->page_crossed = ((base ^ addr) > 255u);
    return addr;
}

static inline uint32_t branch(splb32 *c, int taken)
{
    c->branch_taken = (uint8_t)taken;
    if (taken) {
        int8_t rel = (int8_t)read8(c, c->pc);
        uint16_t prev = (uint16_t)((c->pc + 1) & 0xFFFF);
        c->pc = (uint16_t)((prev + (int16_t)rel) & 0xFFFF);
        c->page_crossed = ((c->pc ^ prev) > 255u);
        return 3u + (c->page_crossed ? 1u : 0u);
    }
    c->pc = (uint16_t)((c->pc + 1) & 0xFFFF);
    return 2u;
}

/* ---- opcode dispatch ---------------------------------------------------
 * Standard 6502 layout plus the Sunplus SFR/bank behaviour supplied by the
 * bus. Cycle counts are the reference core's raw values; the clock divider is
 * applied by the caller.
 * ---------------------------------------------------------------------- */
static void exec_op_sem(splb32 *c, uint8_t op)
{
    uint32_t ad;
    c->page_crossed = 0;
    c->branch_taken = 0;

    switch (op) {
    /* ---- BRK / stack ---- */
    case 0x00: c->bf = 1; do_irq(c); return;
    case 0x40: c->sp = (uint8_t)((c->sp + 1) & 0xFF); set_ps(c, read8(c, SP_PAGE | c->sp));
               c->sp = (uint8_t)((c->sp + 1) & 0xFF); c->pc = read8(c, SP_PAGE | c->sp);
               c->sp = (uint8_t)((c->sp + 1) & 0xFF);
               c->pc |= (uint16_t)(read8(c, SP_PAGE | c->sp) << 8); return;
    case 0x60: c->sp = (uint8_t)((c->sp + 1) & 0xFF); c->pc = read8(c, SP_PAGE | c->sp);
               c->sp = (uint8_t)((c->sp + 1) & 0xFF);
               c->pc |= (uint32_t)(read8(c, SP_PAGE | c->sp) << 8);
               c->pc = c->pc + 1;          /* deliberately unmasked, as reference */
               return;
    case 0x08: push(c, get_ps(c)); return;
    case 0x28: set_ps(c, pull(c)); return;
    case 0x48: push(c, c->a); return;
    case 0x68: c->a = pull(c); c->nf = (c->a >> 7) & 1; c->zf = (c->a == 0); return;

    /* ---- ORA ---- */
    case 0x09: do_ora(c, read8(c, (uint32_t)c->pc++)); return;
    case 0x05: do_ora(c, read8(c, am_zp(c))); return;
    case 0x15: do_ora(c, read8(c, am_zpx(c))); return;
    case 0x0D: do_ora(c, read8(c, am_abs(c))); return;
    case 0x1D: do_ora(c, read8(c, am_absx(c))); return;
    case 0x19: do_ora(c, read8(c, am_absy(c))); return;
    case 0x01: do_ora(c, read8(c, am_indx(c))); return;
    case 0x11: do_ora(c, read8(c, am_indy(c))); return;

    /* ---- AND ---- */
    case 0x29: do_and(c, read8(c, (uint32_t)c->pc++)); return;
    case 0x25: do_and(c, read8(c, am_zp(c))); return;
    case 0x35: do_and(c, read8(c, am_zpx(c))); return;
    case 0x2D: do_and(c, read8(c, am_abs(c))); return;
    case 0x3D: do_and(c, read8(c, am_absx(c))); return;
    case 0x39: do_and(c, read8(c, am_absy(c))); return;
    case 0x21: do_and(c, read8(c, am_indx(c))); return;
    case 0x31: do_and(c, read8(c, am_indy(c))); return;

    /* ---- EOR ---- */
    case 0x49: do_eor(c, read8(c, (uint32_t)c->pc++)); return;
    case 0x45: do_eor(c, read8(c, am_zp(c))); return;
    case 0x55: do_eor(c, read8(c, am_zpx(c))); return;
    case 0x4D: do_eor(c, read8(c, am_abs(c))); return;
    case 0x5D: do_eor(c, read8(c, am_absx(c))); return;
    case 0x59: do_eor(c, read8(c, am_absy(c))); return;
    case 0x41: do_eor(c, read8(c, am_indx(c))); return;
    case 0x51: do_eor(c, read8(c, am_indy(c))); return;

    /* ---- ADC ---- */
    case 0x69: do_adc(c, read8(c, (uint32_t)c->pc++)); return;
    case 0x65: do_adc(c, read8(c, am_zp(c))); return;
    case 0x75: do_adc(c, read8(c, am_zpx(c))); return;
    case 0x6D: do_adc(c, read8(c, am_abs(c))); return;
    case 0x7D: do_adc(c, read8(c, am_absx(c))); return;
    case 0x79: do_adc(c, read8(c, am_absy(c))); return;
    case 0x61: do_adc(c, read8(c, am_indx(c))); return;
    case 0x71: do_adc(c, read8(c, am_indy(c))); return;

    /* ---- SBC ---- */
    case 0xE9: do_sbc(c, read8(c, (uint32_t)c->pc++)); return;
    case 0xE5: do_sbc(c, read8(c, am_zp(c))); return;
    case 0xF5: do_sbc(c, read8(c, am_zpx(c))); return;
    case 0xED: do_sbc(c, read8(c, am_abs(c))); return;
    case 0xFD: do_sbc(c, read8(c, am_absx(c))); return;
    case 0xF9: do_sbc(c, read8(c, am_absy(c))); return;
    case 0xE1: do_sbc(c, read8(c, am_indx(c))); return;
    case 0xF1: do_sbc(c, read8(c, am_indy(c))); return;

    /* ---- CMP / CPX / CPY ---- */
    case 0xC9: do_cmp(c, c->a, read8(c, (uint32_t)c->pc++)); return;
    case 0xC5: do_cmp(c, c->a, read8(c, am_zp(c))); return;
    case 0xD5: do_cmp(c, c->a, read8(c, am_zpx(c))); return;
    case 0xCD: do_cmp(c, c->a, read8(c, am_abs(c))); return;
    case 0xDD: do_cmp(c, c->a, read8(c, am_absx(c))); return;
    case 0xD9: do_cmp(c, c->a, read8(c, am_absy(c))); return;
    case 0xC1: do_cmp(c, c->a, read8(c, am_indx(c))); return;
    case 0xD1: do_cmp(c, c->a, read8(c, am_indy(c))); return;
    case 0xE0: do_cmp(c, c->x, read8(c, (uint32_t)c->pc++)); return;
    case 0xE4: do_cmp(c, c->x, read8(c, am_zp(c))); return;
    case 0xEC: do_cmp(c, c->x, read8(c, am_abs(c))); return;
    case 0xC0: do_cmp(c, c->y, read8(c, (uint32_t)c->pc++)); return;
    case 0xC4: do_cmp(c, c->y, read8(c, am_zp(c))); return;
    case 0xCC: do_cmp(c, c->y, read8(c, am_abs(c))); return;

    /* ---- LDA ---- */
    case 0xA9: c->a = read8(c, (uint32_t)c->pc++); goto load_tail;
    case 0xA5: c->a = read8(c, am_zp(c));         goto load_tail;
    case 0xB5: c->a = read8(c, am_zpx(c));        goto load_tail;
    case 0xAD: c->a = read8(c, am_abs(c));        goto load_tail;
    case 0xBD: c->a = read8(c, am_absx(c));       goto load_tail;
    case 0xB9: c->a = read8(c, am_absy(c));       goto load_tail;
    case 0xA1: c->a = read8(c, am_indx(c));       goto load_tail;
    case 0xB1: c->a = read8(c, am_indy(c));       goto load_tail;

    /* ---- LDX ---- */
    case 0xA2: c->x = read8(c, (uint32_t)c->pc++); goto load_tail;
    case 0xA6: c->x = read8(c, am_zp(c));          goto load_tail;
    case 0xB6: c->x = read8(c, am_zpy(c));         goto load_tail;
    case 0xAE: c->x = read8(c, am_abs(c));         goto load_tail;
    case 0xBE: c->x = read8(c, am_absy(c));        goto load_tail;

    /* ---- LDY ---- */
    case 0xA0: c->y = read8(c, (uint32_t)c->pc++); goto load_tail;
    case 0xA4: c->y = read8(c, am_zp(c));          goto load_tail;
    case 0xB4: c->y = read8(c, am_zpx(c));         goto load_tail;
    case 0xAC: c->y = read8(c, am_abs(c));         goto load_tail;
    case 0xBC: c->y = read8(c, am_absx(c));        goto load_tail;

load_tail:
    /* which register was loaded is determined by the opcode */
    switch (op) {
    case 0xA9: case 0xA5: case 0xB5: case 0xAD: case 0xBD:
    case 0xB9: case 0xA1: case 0xB1:
        c->nf = (c->a >> 7) & 1; c->zf = (c->a == 0); break;
    case 0xA2: case 0xA6: case 0xB6: case 0xAE: case 0xBE:
        c->nf = (c->x >> 7) & 1; c->zf = (c->x == 0); break;
    default:
        c->nf = (c->y >> 7) & 1; c->zf = (c->y == 0); break;
    }
    return;

    /* ---- STA / STX / STY ---- */
    case 0x85: write8(c, am_zp(c), c->a); return;
    case 0x95: write8(c, am_zpx(c), c->a); return;
    case 0x8D: write8(c, am_abs(c), c->a); return;
    case 0x9D: write8(c, am_absx(c), c->a); return;
    case 0x99: write8(c, am_absy(c), c->a); return;
    case 0x81: write8(c, am_indx(c), c->a); return;
    case 0x91: write8(c, am_indy(c), c->a); return;
    case 0x86: write8(c, am_zp(c), c->x); return;
    case 0x96: write8(c, am_zpy(c), c->x); return;
    case 0x8E: write8(c, am_abs(c), c->x); return;
    case 0x84: write8(c, am_zp(c), c->y); return;
    case 0x94: write8(c, am_zpx(c), c->y); return;
    case 0x8C: write8(c, am_abs(c), c->y); return;

    /* ---- BIT ---- */
    case 0x24: ad = am_zp(c);
               { uint8_t v = read8(c, ad); c->zf = ((c->a & v) == 0);
                 c->nf = (v >> 7) & 1; c->vf = (v >> 6) & 1; } return;
    case 0x2C: ad = am_abs(c);
               { uint8_t v = read8(c, ad); c->zf = ((c->a & v) == 0);
                 c->nf = (v >> 7) & 1; c->vf = (v >> 6) & 1; } return;

    /* ---- shifts: ASL/LSR/ROL/ROR ---- */
    case 0x0A: c->a = do_asl(c, c->a); return;
    case 0x06: ad = am_zp(c); write8(c, ad, do_asl(c, read8(c, ad))); return;
    case 0x16: ad = am_zpx(c); write8(c, ad, do_asl(c, read8(c, ad))); return;
    case 0x0E: ad = am_abs(c); write8(c, ad, do_asl(c, read8(c, ad))); return;
    case 0x1E: ad = am_absx(c); write8(c, ad, do_asl(c, read8(c, ad))); return;

    case 0x4A: c->a = do_lsr(c, c->a); return;
    case 0x46: ad = am_zp(c); write8(c, ad, do_lsr(c, read8(c, ad))); return;
    case 0x56: ad = am_zpx(c); write8(c, ad, do_lsr(c, read8(c, ad))); return;
    case 0x4E: ad = am_abs(c); write8(c, ad, do_lsr(c, read8(c, ad))); return;
    case 0x5E: ad = am_absx(c); write8(c, ad, do_lsr(c, read8(c, ad))); return;

    case 0x2A: c->a = do_rol(c, c->a); return;
    case 0x26: ad = am_zp(c); write8(c, ad, do_rol(c, read8(c, ad))); return;
    case 0x36: ad = am_zpx(c); write8(c, ad, do_rol(c, read8(c, ad))); return;
    case 0x2E: ad = am_abs(c); write8(c, ad, do_rol(c, read8(c, ad))); return;
    case 0x3E: ad = am_absx(c); write8(c, ad, do_rol(c, read8(c, ad))); return;

    case 0x6A: c->a = do_ror(c, c->a); return;
    case 0x66: ad = am_zp(c); write8(c, ad, do_ror(c, read8(c, ad))); return;
    case 0x76:
        /* Reference quirk: _ror_zp_x advances PC by 2, not 1. Every other
         * zp,X instruction advances by 1. Reproduced deliberately so that
         * the core stays bit-exact with the implementation the ROM was
         * timed against. See VALIDATION.md. */
        ad = am_zpx(c);
        write8(c, ad, do_ror(c, read8(c, ad)));
        c->pc = (uint16_t)((c->pc + 1) & 0xFFFF);
        return;
    case 0x6E: ad = am_abs(c); write8(c, ad, do_ror(c, read8(c, ad))); return;
    case 0x7E: ad = am_absx(c); write8(c, ad, do_ror(c, read8(c, ad))); return;

    /* ---- INC / DEC ---- */
    case 0xE6: ad = am_zp(c);  { uint8_t v = (uint8_t)(read8(c, ad) + 1);
                 write8(c, ad, v); c->nf = (v >> 7) & 1; c->zf = (v == 0); } return;
    case 0xF6: ad = am_zpx(c); { uint8_t v = (uint8_t)(read8(c, ad) + 1);
                 write8(c, ad, v); c->nf = (v >> 7) & 1; c->zf = (v == 0); } return;
    case 0xEE: ad = am_abs(c); { uint8_t v = (uint8_t)(read8(c, ad) + 1);
                 write8(c, ad, v); c->nf = (v >> 7) & 1; c->zf = (v == 0); } return;
    case 0xFE: ad = am_absx(c);{ uint8_t v = (uint8_t)(read8(c, ad) + 1);
                 write8(c, ad, v); c->nf = (v >> 7) & 1; c->zf = (v == 0); } return;
    case 0xC6: ad = am_zp(c);  { uint8_t v = (uint8_t)(read8(c, ad) - 1);
                 write8(c, ad, v); c->nf = (v >> 7) & 1; c->zf = (v == 0); } return;
    case 0xD6: ad = am_zpx(c); { uint8_t v = (uint8_t)(read8(c, ad) - 1);
                 write8(c, ad, v); c->nf = (v >> 7) & 1; c->zf = (v == 0); } return;
    case 0xCE: ad = am_abs(c); { uint8_t v = (uint8_t)(read8(c, ad) - 1);
                 write8(c, ad, v); c->nf = (v >> 7) & 1; c->zf = (v == 0); } return;
    case 0xDE: ad = am_absx(c);{ uint8_t v = (uint8_t)(read8(c, ad) - 1);
                 write8(c, ad, v); c->nf = (v >> 7) & 1; c->zf = (v == 0); } return;

    /* ---- register ops ---- */
    case 0xE8: c->x++; c->nf = (c->x >> 7) & 1; c->zf = (c->x == 0); return;
    case 0xC8: c->y++; c->nf = (c->y >> 7) & 1; c->zf = (c->y == 0); return;
    case 0xCA: c->x--; c->nf = (c->x >> 7) & 1; c->zf = (c->x == 0); return;
    case 0x88: c->y--; c->nf = (c->y >> 7) & 1; c->zf = (c->y == 0); return;
    case 0xAA: c->x = c->a; c->nf = (c->x >> 7) & 1; c->zf = (c->x == 0); return;
    case 0xA8: c->y = c->a; c->nf = (c->y >> 7) & 1; c->zf = (c->y == 0); return;
    case 0x8A: c->a = c->x; c->nf = (c->a >> 7) & 1; c->zf = (c->a == 0); return;
    case 0x98: c->a = c->y; c->nf = (c->a >> 7) & 1; c->zf = (c->a == 0); return;
    case 0xBA: c->x = c->sp; c->nf = (c->x >> 7) & 1; c->zf = (c->x == 0); return;
    case 0x9A: c->sp = c->x; return;
    case 0xEA: return;

    /* ---- jumps ---- */
    case 0x4C: c->pc = (uint16_t)(read8(c, c->pc) | (read8(c, (uint16_t)((c->pc + 1) & 0xFFFF)) << 8)); return;
    case 0x6C: { uint16_t ptr = (uint16_t)(read8(c, c->pc) | (read8(c, (uint16_t)((c->pc + 1) & 0xFFFF)) << 8));
                 uint16_t lo = read8(c, ptr);
                 /* reference reads the high byte at ptr+1 with no 6502 page wrap */
                 uint16_t hi = read8(c, (uint16_t)((ptr + 1) & 0xFFFF));
                 c->pc = (uint16_t)(lo | (hi << 8)); } return;
    case 0x20: { uint16_t pc = (uint16_t)((c->pc + 1) & 0xFFFF);
                 push(c, (uint8_t)(pc >> 8));
                 push(c, (uint8_t)(pc & 0xFF));
                 c->pc = (uint16_t)(read8(c, c->pc) | (read8(c, pc) << 8)); } return;

    /* ---- branches ---- */
    case 0x10: branch(c, !c->nf); return;
    case 0x30: branch(c, c->nf); return;
    case 0x50: branch(c, !c->vf); return;
    case 0x70: branch(c, c->vf); return;
    case 0x90: branch(c, !c->cf); return;
    case 0xB0: branch(c, c->cf); return;
    case 0xD0: branch(c, !c->zf); return;
    case 0xF0: branch(c, c->zf); return;

    /* ---- flag ops ---- */
    case 0x18: c->cf = 0; return;
    case 0x38: c->cf = 1; return;
    case 0x58: c->if_ = 0; return;
    case 0x78: c->if_ = 1; return;
    case 0xB8: c->vf = 0; return;
    case 0xD8: c->df = 0; return;
    case 0xF8: c->df = 1; return;

    default: return;   /* illegal: reference returns its table entry */
    }
}

/* ---- core step -------------------------------------------------------- */
static int64_t sleep_cycles(const splb32 *c)
{
    /* clock / 32768 = 122.0703125 at 4 MHz, which is exactly 15625/128 and so
     * an exact integer in CYC_ONE units. Keeping the fraction is what stops
     * the cycle count drifting; keeping it integral is what stops the
     * interpreter calling soft-float once per instruction. */
    return c->sub_clock_div;
}


/* ---- core step -------------------------------------------------------- */
/*
 * One emulated instruction, WITHOUT touching the cycle accumulator.
 *
 * Splitting this out is the single largest optimisation available on this port,
 * and the reason is memory rather than arithmetic. PSRAM is the only memory
 * Linux can see (EMU-003) and a random dependent load there costs about 70 ns,
 * roughly 17 CPU cycles. `splb32_step` was called once per emulated instruction
 * from a loop that read `c->cycles` in its own condition, so every instruction
 * paid for at least three accumulator round trips plus a function prologue and
 * epilogue, all of which land in PSRAM.
 *
 * Returning the elapsed units lets the run loops keep the accumulator in a
 * register for the whole run and write it back once. `c->cycles` is derived
 * from it, and nothing inside the step reads either, so this is a pure
 * accounting change.
 */
static inline int64_t step_core(splb32 *c)
{
    uint32_t raw;
    int64_t raw_units;

    PROF_STEP();
    if (c->cpu_enbl) {
        uint8_t op = read8(c, c->pc);
        /*
         * opcode_hits is coverage instrumentation and nothing in this build
         * reads it: splb32_opcode_count() is declared in the header and called
         * from nowhere. It was a read-modify-write into a 256-entry array on
         * every emulated instruction, in a struct that lives entirely in PSRAM.
         *
         * Kept honest about what this bought: removing it measured 516887 ->
         * 521199 instr/s, about 3.8 native cycles per instruction, or 1.9
         * cycles per access. A DEPENDENT PSRAM load costs 12.4 cycles (memlat,
         * 51.8 ns at 240 MHz), but these two were not on the critical path and
         * the bus pipelines back-to-back transactions, so the saving is small.
         * Worth taking because it is free, not because it is a lever.
         *
         * Behind a flag rather than deleted, because the coverage tool is
         * still useful when deliberately wanted:
         *     -DSPLB32_COVERAGE
         */
#ifdef SPLB32_COVERAGE
        c->opcode_hits[op]++;
#endif
        c->pc = (uint16_t)((c->pc + 1) & 0xFFFF);
        c->instrs++;

        exec_op_sem(c, op);

        /* cycles come from the reference-derived tables */
        if (cyc_page[op] == 2) {
            /* branch opcode: 2 not taken, 3 + page penalty when taken */
            raw = c->branch_taken ? (3u + c->page_crossed) : 2u;
        } else {
            raw = cyc_base[op];
            if (cyc_page[op] && c->page_crossed) raw += PAGE_PENALTY;
        }

        raw <<= c->cpu_div;
        raw_units = (int64_t)raw << CYC_SHIFT;
        timers_clock(c, raw_units);

        if (!c->if_ && (c->ireq & c->int_ctrl)) do_irq(c);
    } else {
        PROF_SLEEP();
        raw_units = sleep_cycles(c);
        raw = (uint32_t)(raw_units >> CYC_SHIFT);
        if (c->wakeup_ctrl & c->wakeupreq) {
            c->cpu_enbl = 1;
            c->cpu_div = CPU_CTRL_DEFAULT;
        }
        if (c->clk32768_en) timers_clock(c, raw_units);
    }

    if (c->ir) {
        if (con_ir_clock(c->ir, raw_units))
            splb32_pin(c, 1, c->ir->rx_mask, c->ir->rx_level);
    }

    return raw_units;
}

uint64_t splb32_run_cycles(splb32 *c, uint64_t cycles)
{
    uint64_t target = c->cycles + cycles;
    int64_t units = c->cycles_units;

    while ((uint64_t)(units >> CYC_SHIFT) < target)
        units += step_core(c);

    c->cycles_units = units;
    c->cycles = (uint64_t)(units >> CYC_SHIFT);
    return c->cycles;
}

uint64_t splb32_run_instrs(splb32 *c, uint64_t instrs)
{
    uint64_t target = c->instrs + instrs;
    int64_t units = c->cycles_units;

    while (c->instrs < target)
        units += step_core(c);

    c->cycles_units = units;
    c->cycles = (uint64_t)(units >> CYC_SHIFT);
    return c->instrs;
}

/*
 * Public single step: keeps the old contract, accumulator included, for callers
 * that interleave stepping with observation.
 */
uint32_t splb32_step(splb32 *c)
{
    int64_t units = step_core(c);

    c->cycles_units += units;
    c->cycles = (uint64_t)(c->cycles_units >> CYC_SHIFT);
    return (uint32_t)(units >> CYC_SHIFT);
}

/* ---- machine save state ------------------------------------------------ */

/* Packed field by field rather than memcpy of the struct, so that padding and
 * pointer values never leak into the file and a snapshot from one build can be
 * checked against another. */
typedef struct {
    uint32_t magic, version, size;

    uint32_t pc;
    uint8_t  a, x, y, sp, p;
    uint8_t  rom_bank_idx;
    uint8_t  cpu_enbl, cpu_div, clk32768_en;

    uint8_t  ram[0x4C0];
    uint8_t  dpram[0x200];
    uint8_t  sfr[0x40];
    uint8_t  pdir[4], pattr[4], platch[4];
    uint8_t  pin_in_lo[4], pin_in_hi[4], pullup_ext[4];

    int64_t  tm0_cnt, tm1_cnt, t1khz_cnt;   /* CYC_ONE units */
    int64_t  tm0, tm1;                      /* CYC_ONE units */
    uint32_t tm0_reload, tm1_reload, t1khz;
    uint8_t  timer_ctrl, time_base;
    uint8_t  wakeup_ctrl, wakeupreq, int_ctrl, ireq, nmi_ctrl, nmireq;
    uint8_t  lcd_ctrl1, aud0_ctrl, aud0_data, aud1_ctrl, aud1_data;

    uint64_t instrs;
    int64_t  cycles_units;

    uint8_t  has_eeprom;
    uint8_t  eeprom[HT24LC08_SIZE];
} state_blob;

int splb32_state_size(void) { return (int)sizeof(state_blob); }

uint64_t splb32_state_cycles(const splb32 *c) { return c->cycles; }

int splb32_state_save(const splb32 *c, uint8_t *out, int len)
{
    if (!out || len < (int)sizeof(state_blob)) return -1;
    state_blob b;
    memset(&b, 0, sizeof(b));
    b.magic = SPLB32_STATE_MAGIC;
    b.version = SPLB32_STATE_VERSION;
    b.size = (uint32_t)sizeof(state_blob);

    b.pc = c->pc;
    b.a = c->a; b.x = c->x; b.y = c->y; b.sp = c->sp; b.p = get_ps(c);
    b.rom_bank_idx = (uint8_t)(c->rom_bank >> 15);
    b.cpu_enbl = c->cpu_enbl; b.cpu_div = c->cpu_div; b.clk32768_en = c->clk32768_en;

    memcpy(b.ram, c->ram, sizeof(b.ram));
    memcpy(b.dpram, c->dpram, sizeof(b.dpram));
    memcpy(b.sfr, c->sfr, sizeof(b.sfr));
    memcpy(b.pdir, c->pdir, sizeof(b.pdir));
    memcpy(b.pattr, c->pattr, sizeof(b.pattr));
    memcpy(b.platch, c->platch, sizeof(b.platch));
    memcpy(b.pin_in_lo, c->pin_in_lo, sizeof(b.pin_in_lo));
    memcpy(b.pin_in_hi, c->pin_in_hi, sizeof(b.pin_in_hi));
    memcpy(b.pullup_ext, c->pullup_ext, sizeof(b.pullup_ext));

    b.tm0_cnt = c->tm0_cnt; b.tm1_cnt = c->tm1_cnt; b.t1khz_cnt = c->t1khz_cnt;
    b.tm0 = c->tm0; b.tm0_reload = c->tm0_reload;
    b.tm1 = c->tm1; b.tm1_reload = c->tm1_reload; b.t1khz = c->t1khz;
    b.timer_ctrl = c->timer_ctrl; b.time_base = c->time_base;
    b.wakeup_ctrl = c->wakeup_ctrl; b.wakeupreq = c->wakeupreq;
    b.int_ctrl = c->int_ctrl; b.ireq = c->ireq;
    b.nmi_ctrl = c->nmi_ctrl; b.nmireq = c->nmireq;
    b.lcd_ctrl1 = c->lcd_ctrl1;
    b.aud0_ctrl = c->aud0_ctrl; b.aud0_data = c->aud0_data;
    b.aud1_ctrl = c->aud1_ctrl; b.aud1_data = c->aud1_data;

    b.instrs = c->instrs;
    b.cycles_units = c->cycles_units;

    if (c->eeprom) {
        b.has_eeprom = 1;
        ht24lc08_save(c->eeprom, b.eeprom);
    }

    memcpy(out, &b, sizeof(b));
    return (int)sizeof(b);
}

int splb32_state_load(splb32 *c, const uint8_t *in, int len)
{
    if (!in || len < (int)sizeof(state_blob)) return -1;
    state_blob b;
    memcpy(&b, in, sizeof(b));

    if (b.magic != SPLB32_STATE_MAGIC) return -2;
    if (b.version != SPLB32_STATE_VERSION) return -3;
    if (b.size != (uint32_t)sizeof(state_blob)) return -4;

    c->pc = b.pc;
    c->a = b.a; c->x = b.x; c->y = b.y; c->sp = b.sp;
    set_ps(c, b.p);
    c->rom_bank = (uint32_t)b.rom_bank_idx << 15;
    c->cpu_enbl = b.cpu_enbl; c->cpu_div = b.cpu_div;
    c->clk32768_en = b.clk32768_en;

    memcpy(c->ram, b.ram, sizeof(c->ram));
    memcpy(c->dpram, b.dpram, sizeof(c->dpram));
    memcpy(c->sfr, b.sfr, sizeof(c->sfr));
    memcpy(c->pdir, b.pdir, sizeof(c->pdir));
    memcpy(c->pattr, b.pattr, sizeof(c->pattr));
    memcpy(c->platch, b.platch, sizeof(c->platch));
    memcpy(c->pin_in_lo, b.pin_in_lo, sizeof(c->pin_in_lo));
    memcpy(c->pin_in_hi, b.pin_in_hi, sizeof(c->pin_in_hi));
    memcpy(c->pullup_ext, b.pullup_ext, sizeof(c->pullup_ext));

    c->tm0_cnt = b.tm0_cnt; c->tm1_cnt = b.tm1_cnt; c->t1khz_cnt = b.t1khz_cnt;
    c->tm0 = b.tm0; c->tm0_reload = b.tm0_reload;
    c->tm1 = b.tm1; c->tm1_reload = b.tm1_reload; c->t1khz = b.t1khz;
    c->timer_ctrl = b.timer_ctrl; c->time_base = b.time_base;
    c->wakeup_ctrl = b.wakeup_ctrl; c->wakeupreq = b.wakeupreq;
    c->int_ctrl = b.int_ctrl; c->ireq = b.ireq;
    c->nmi_ctrl = b.nmi_ctrl; c->nmireq = b.nmireq;
    c->lcd_ctrl1 = b.lcd_ctrl1;
    c->aud0_ctrl = b.aud0_ctrl; c->aud0_data = b.aud0_data;
    c->aud1_ctrl = b.aud1_ctrl; c->aud1_data = b.aud1_data;

    c->instrs = b.instrs;
    c->cycles_units = b.cycles_units;
    c->cycles = (uint64_t)(c->cycles_units >> CYC_SHIFT);

    if (b.has_eeprom) {
        if (!c->eeprom) c->eeprom = (ht24lc08 *)calloc(1, sizeof(ht24lc08));
        if (c->eeprom) {
            ht24lc08_init(c->eeprom, 0xA0, 2, 1);
            ht24lc08_load(c->eeprom, b.eeprom, HT24LC08_SIZE);
        }
    }
    return 0;
}

/* ---- lifecycle -------------------------------------------------------- */
splb32 *splb32_new(const uint8_t *rom, size_t len)
{
    if (!rom || len < SPLB32_ROM_BYTES) return NULL;
    splb32 *c = (splb32 *)calloc(1, sizeof(*c));
    if (!c) return NULL;
    c->rom = rom;
    /* Exact: (4000000 << 7) / 32768 = 15625, i.e. 122.0703125 cycles. */
    c->sub_clock_div = ((int64_t)SPLB32_CLOCK_HZ << CYC_SHIFT) / SPLB32_SUB_CLOCK;
    splb32_reset(c);
    return c;
}

void splb32_free(splb32 *c) { if (c) { free(c->eeprom); free(c->ir); free(c); } }

void splb32_set_pullup(splb32 *c, int port, uint8_t mask)
{
    if (port < 0 || port > 3) return;
    c->pullup_ext[port] = mask;
}

void splb32_reset(splb32 *c)
{
    memset(c->ram, 0, sizeof(c->ram));
    memset(c->dpram, 0, sizeof(c->dpram));
    memset(c->sfr, 0, sizeof(c->sfr));
    memset(c->pdir, 0, sizeof(c->pdir));
    memset(c->pattr, 0, sizeof(c->pattr));
    memset(c->platch, 0, sizeof(c->platch));
    memset(c->pin_in_lo, 0, sizeof(c->pin_in_lo));
    memset(c->pin_in_hi, 0, sizeof(c->pin_in_hi));
    memset(c->pullup_ext, 0, sizeof(c->pullup_ext));

    c->tm0_cnt = c->tm1_cnt = c->t1khz_cnt = 0;
    c->tm0 = c->tm1 = 0;
    c->tm0_reload = c->tm1_reload = 0;
    c->t1khz = 0;
    c->timer_ctrl = 0;
    c->time_base = 0;

    c->pc = 0; c->a = c->x = c->y = 0; c->sp = 0;
    set_ps(c, 0x04);

    c->cpu_enbl = 1;
    c->cpu_div = CPU_CTRL_DEFAULT;
    c->clk32768_en = CLK32768_EN;

    c->rom_bank = 0;
    c->wakeup_ctrl = c->wakeupreq = 0;
    c->int_ctrl = c->ireq = 0;
    c->nmi_ctrl = c->nmireq = 0;
    c->lcd_ctrl1 = 0;
    c->aud0_ctrl = c->aud0_data = c->aud1_ctrl = c->aud1_data = 0;
    splb32_sound_init(&c->sound);

    c->page_crossed = 0;
    c->branch_taken = 0;

    c->instrs = 0;
    c->cycles = 0;
    c->cycles_units = 0;

    /* vectors are little-endian in the fixed bank */
    c->addr_reset = (uint16_t)(read8(c, 0x7FFC) | (read8(c, 0x7FFD) << 8));
    c->addr_irq   = (uint16_t)(read8(c, 0x7FFE) | (read8(c, 0x7FFF) << 8));
    c->addr_nmi   = (uint16_t)(read8(c, 0x7FFA) | (read8(c, 0x7FFB) << 8));
    c->pc = c->addr_reset;
}

static uint32_t flat_pc(const splb32 *c)
{
    /* Matches the reference's _get_rom_addr exactly: the fixed bank reports
     * PC & 0x7FFF, and EVERYTHING else - including low addresses that are
     * really SFR or RAM space - reports (PC & 0x7FFF) | bank. The reference
     * does not special-case low addresses, so neither do we. */
    if (c->pc >= ROM_BANK_0) return c->pc & 0x7FFF;
    return (c->pc & 0x7FFF) | c->rom_bank;
}

void splb32_snapshot(const splb32 *c, splb32_state *s)
{
    s->pc = flat_pc(c);
    s->a = c->a; s->x = c->x; s->y = c->y; s->sp = c->sp;
    s->p = get_ps(c);
    s->instrs = c->instrs;
    s->cycles = c->cycles;
    memcpy(s->ram, c->ram, sizeof(s->ram));
    memcpy(s->dpram, c->dpram, sizeof(s->dpram));
}

const uint8_t *splb32_dpram(const splb32 *c) { return c->dpram; }

void splb32_pin(splb32 *c, int port, uint8_t mask, int level)
{
    if (port < 0 || port > 3) return;

    uint8_t prev = port_read(c, port);
    c->pin_in_lo[port] &= (uint8_t)~mask;
    c->pin_in_hi[port] &= (uint8_t)~mask;
    if (level >= 0)
        (level ? c->pin_in_hi : c->pin_in_lo)[port] |= mask;

    /* PC bit 1 is the IR receiver: a high->low transition raises the
     * external interrupt, per the reference's pin_handler. */
    if (port == 2 && (mask & 0x02)) {
        if ((prev & 0x10) > (port_read(c, 2) & 0x10))
            c->ireq |= IRQ_EXT;
    }
    /* Any PA pin that reads high raises the wakeup request. */
    if (port == 0 && (port_read(c, 0) & mask))
        c->wakeupreq |= WK_EXT;
}

void splb32_attach_ir(splb32 *c, uint8_t rx_mask, uint8_t tx_mask)
{
    if (!c->ir) c->ir = (con_ir *)calloc(1, sizeof(con_ir));
    if (!c->ir) return;
    con_ir_init(c->ir, rx_mask, tx_mask);
    /* the reference holds the receiver low at construction */
    splb32_pin(c, 1, c->ir->rx_mask, 0);
}

int splb32_ir_push(splb32 *c, const uint8_t *data, int len)
{
    if (!c->ir) return 0;
    return con_ir_push(c->ir, data, len);
}

int splb32_ir_pop_tx(splb32 *c)
{
    if (!c->ir) return -1;
    return con_ir_pop_tx(c->ir);
}

void splb32_attach_eeprom(splb32 *c, uint8_t hw_addr, uint8_t sda_mask, uint8_t scl_mask)
{
    if (!c->eeprom)
        c->eeprom = (ht24lc08 *)calloc(1, sizeof(ht24lc08));
    if (!c->eeprom)
        return;
    ht24lc08_init(c->eeprom, hw_addr, sda_mask, scl_mask);
}

int splb32_eeprom_save(const splb32 *c, uint8_t *out)
{
    if (!c->eeprom) return -1;
    ht24lc08_save(c->eeprom, out);
    return 0;
}

void splb32_eeprom_load(splb32 *c, const uint8_t *data)
{
    if (c->eeprom) ht24lc08_load(c->eeprom, data, HT24LC08_SIZE);
}

uint8_t splb32_peek(splb32 *c, uint32_t addr) { return read8(c, addr); }

double splb32_audio_level(const splb32 *c, int channel)
{
    return splb32_sound_level(&c->sound, channel);
}

unsigned long splb32_audio_emits(const splb32 *c) { return c->sound.emits; }

void splb32_dbg_ports(splb32 *c, uint8_t *pdir, uint8_t *pattr, uint8_t *platch)
{
    for (int i = 0; i < 4; i++) { pdir[i]=c->pdir[i]; pattr[i]=c->pattr[i]; platch[i]=c->platch[i]; }
}
uint32_t splb32_bank(const splb32 *c) { return c->rom_bank; }

uint32_t splb32_opcode_count(const splb32 *c, int op)
{
    if (op < 0 || op > 255) return 0;
    return c->opcode_hits[op];
}

void splb32_clear_coverage(splb32 *c)
{
    for (int i = 0; i < 256; i++) c->opcode_hits[i] = 0;
}
