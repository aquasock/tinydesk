/*
 * splb32.h - Sunplus SPLB32 (Tamagotchi Connection V3) CPU core
 *
 * A cycle-counted 6502-family core with the Sunplus extensions:
 *   - SFR bank at $0000-$003F
 *   - CPU RAM at $0040-$04FF (0x4C0)
 *   - display RAM at $3E00-$3FFF (0x200)
 *   - banked ROM at $4000-$BFFF (16 banks x 32K, selected by SFR $00)
 *   - fixed ROM at $C000-$FFFF
 *
 * Sleep cycles are batched: when the CPU is disabled (SFR $04 = 7) one call
 * returns the full 32768 Hz tick (clock/32768 cycles) instead of stepping a
 * single cycle. Without that, an emulator needs 4M iterations/second just to
 * idle, on any host.
 *
 * No peripherals, no I/O routing, no display. CPU, timers and memory only.
 * That is deliberate: this exists to answer "how fast is it", not to be a
 * front end.
 */

#ifndef SPLB32_H
#define SPLB32_H

#include <stdint.h>
#include <stddef.h>

#define SPLB32_CLOCK_HZ     4000000u
#define SPLB32_ROM_BYTES    524288u     /* 512 KB */
#define SPLB32_SUB_CLOCK    32768u

/*
 * Fixed-point scale for cycle accounting.
 *
 * The reference implementation keeps its cycle and timer accumulators as
 * doubles, and the code below used to copy that. On the Xtensa in this watch a
 * double add is a soft-float library call -- measured at 711 ns per iteration
 * against 121 ns for the integer equivalent. timers_clock() runs once per
 * emulated instruction and touches three such counters, so the interpreter
 * paid several hundred nanoseconds of library code for every instruction the
 * guest executed.
 *
 * The scale is 7 fractional bits because the sleep tick has to stay exact:
 * CLOCK_HZ / SUB_CLOCK = 4000000 / 32768 = 122.0703125 = 15625/128. One cycle
 * is 128 units, so that value is the integer 15625 and the fractional drift the
 * original comments warn about cannot occur -- nothing is truncated.
 */
#define CYC_SHIFT   7
#define CYC_ONE     (1 << CYC_SHIFT)

typedef struct splb32 splb32;

/* Load a 512 KB ROM image. Returns NULL on failure. */
splb32 *splb32_new(const uint8_t *rom, size_t len);
void    splb32_free(splb32 *c);

/* Reset to the ROM's reset vector with power-on register state. */
void    splb32_reset(splb32 *c);

/* External pull-up mask per port: 0=PA 1=PB 2=PC 3=PD.
 * The reference's .brick config uses {PD: 2}; set before the first step. */
void    splb32_set_pullup(splb32 *c, int port, uint8_t mask);

/* Execute one opcode, or one sleep tick. Returns emulated cycles consumed. */
uint32_t splb32_step(splb32 *c);

/* Run until at least `cycles` emulated cycles have elapsed. Returns cycles. */
uint64_t splb32_run_cycles(splb32 *c, uint64_t cycles);

#ifdef SPLB32_PROF
extern uint64_t splb32_prof_steps, splb32_prof_irqs, splb32_prof_sleep_steps;
extern uint64_t splb32_prof_reads, splb32_prof_writes, splb32_prof_rom,
                splb32_prof_ram, splb32_prof_dpram, splb32_prof_sfr;
#endif

/* Run until at least `instrs` opcodes have retired (sleep ticks not counted). */
uint64_t splb32_run_instrs(splb32 *c, uint64_t instrs);

/* Introspection for verification and validation. */
typedef struct {
    uint32_t pc;
    uint16_t a, x, y, sp;
    uint8_t  p;              /* status: N V - B D I Z C */
    uint32_t instrs;         /* opcodes retired */
    uint64_t cycles;         /* emulated cycles consumed */
    uint8_t  ram[0x4C0];
    uint8_t  dpram[0x200];
} splb32_state;

void splb32_snapshot(const splb32 *c, splb32_state *s);

/* Direct display-RAM view, for a front end that wants to render. */
const uint8_t *splb32_dpram(const splb32 *c);

/* Coverage instrumentation: how many times each opcode has executed. */
uint32_t splb32_opcode_count(const splb32 *c, int op);

/* ---- machine save state -------------------------------------------------
 * A full snapshot of the emulated machine: registers, RAM, display RAM, SFRs,
 * timers, bank, port state and the EEPROM contents. Enough to resume exactly
 * where you left off, which the ROM itself does not do on this hardware
 * revision (it reads the EEPROM but does not write it in normal play).
 *
 * Format is versioned and size-checked; loading a snapshot from a different
 * build is refused rather than silently corrupting the machine.
 * ---------------------------------------------------------------------- */
#define SPLB32_STATE_MAGIC   0x54534C42u   /* "BLST" */
#define SPLB32_STATE_VERSION 3u   /* 3: timer/cycle accumulators are fixed-point */

int  splb32_state_size(void);
uint64_t splb32_state_cycles(const splb32 *c);
int  splb32_state_save(const splb32 *c, uint8_t *out, int len);
int  splb32_state_load(splb32 *c, const uint8_t *in, int len);

/* Attach the infrared link. rx_mask is the PB bit the firmware watches,
 * tx_mask the PC bit it drives (0x04 for both on the V3). */
void splb32_attach_ir(splb32 *c, uint8_t rx_mask, uint8_t tx_mask);

/* Deliver bytes to the emulated IR receiver. Returns how many were queued. */
int  splb32_ir_push(splb32 *c, const uint8_t *data, int len);

/* Pop a byte the firmware transmitted over IR. -1 when empty. */
int  splb32_ir_pop_tx(splb32 *c);

/* Attach the HT24LC08 EEPROM on a port. hw_addr is the masked I2C address
 * (0xA0 for this board), sda_mask/scl_mask are the bit masks within the port
 * (2 and 1 for PD on the V3). Call before the first step. */
void splb32_attach_eeprom(splb32 *c, uint8_t hw_addr, uint8_t sda_mask, uint8_t scl_mask);

/* Save/load the EEPROM contents (1024 bytes). Returns 0 on success. */
int  splb32_eeprom_save(const splb32 *c, uint8_t *out);
void splb32_eeprom_load(splb32 *c, const uint8_t *data);

/* Drive an external pin.
 *
 * port: 0=PA 1=PB 2=PC 3=PD
 * level: 1 = driven/seen high, 0 = driven/seen low, -1 = released (floating)
 *
 * Button inputs on this board are active-high on PA bits 0/1/2 with release
 * reported as -1, matching the reference's direct_input peripheral.
 * PC bit 1 is the IR receiver and generates an external interrupt on a
 * falling edge. PA applies wakeup.
 */
void splb32_pin(splb32 *c, int port, uint8_t mask, int level);

/* Debug: read an address through the normal bus mapping. */
uint8_t  splb32_peek(splb32 *c, uint32_t addr);
uint32_t splb32_bank(const splb32 *c);

/* Audio: current output level per channel, -1..+1, and an emit counter. */
double splb32_audio_level(const splb32 *c, int channel);
unsigned long splb32_audio_emits(const splb32 *c);

/*
 * Is either channel making a sound right now?
 *
 * THE QUESTION A FRONT END ACTUALLY ASKS, and the reason it is not "did the
 * emit counter change". That counter increments on every square-wave toggle --
 * `toggle()` runs on each driver timer overflow while the envelope is non-zero --
 * so a single held tone raises it thousands of times a second. A front end that
 * treats a rising counter as "a sound happened" therefore plays a sound
 * continuously for as long as one tone lasts, which is exactly what the watch
 * did: a BEL that never stopped and only went quiet while the emulator was
 * paused.
 *
 * The waveform is a clock, not a signal. What a caller wants to know is the
 * envelope: whether the machine is currently trying to make noise. This answers
 * that, and a caller detects the rising EDGE of it -- silence to not-silence --
 * to get one event per sound rather than one per cycle.
 */
int splb32_audio_active(const splb32 *c);
void splb32_dbg_ports(splb32 *c, uint8_t *pdir, uint8_t *pattr, uint8_t *platch);
void     splb32_clear_coverage(splb32 *c);

#endif /* SPLB32_H */
