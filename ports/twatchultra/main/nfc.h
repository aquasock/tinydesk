/*
 * nfc.h - the ST25R3916 NFC reader on the T-Watch Ultra.
 *
 * The reader is not an I2C part and not a PN532: it is an ST25R3916/3916B RF
 * front end on the SECOND SPI bus (SCK 35, MOSI 34, MISO 33), chip select 4,
 * interrupt 5. Its supply is the AXP2101's DLDO1 (BRD-003), so nothing answers
 * until that rail is up -- the same silent shape as the haptic driver, which
 * replies to I2C with its supply switched off.
 *
 * TWO THINGS ABOUT THIS BUS ARE ALREADY KNOWN AND BOTH MATTER HERE:
 *
 *   - MISO is shared with the SD card and the LoRa radio. Every other chip
 *     select has to be parked high or the part that is still selected drives
 *     the line and corrupts the transfer. board_power_rails_preinit() does
 *     that at boot, and this module keeps CS 4 as a plain GPIO driven by hand
 *     rather than handing it to the SPI peripheral -- see nfc.c for why that
 *     distinction is not cosmetic.
 *   - The bus may or may not be up when this runs, because the SD watcher only
 *     initializes it when a card is present. This module initializes it too
 *     and treats "already initialized" as success, so the two can race in
 *     either order.
 */
#ifndef TWATCH_NFC_H
#define TWATCH_NFC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Raise the AXP2101's DLDO1 to 3.3 V and enable it: the reader's supply.
 *
 * Read-modify-write on both registers, because 0x90 also carries ALDO1..4 and
 * BLDO2 -- clobbering it would take the SD card, the panel and the speaker
 * down to power an NFC chip.
 */
bool board_nfc_rail_on(void);

/*
 * Bring the reader up far enough to read its identity register, and log what
 * it found.
 *
 * This is the whole of the first step on this part: it proves the rail, the
 * SPI mode, the shared-bus coexistence and which silicon is fitted, before any
 * protocol code depends on those being right. The identity register is the one
 * answer the chip gives that cannot be produced by a broken bus -- a floating
 * MISO reads 0x00 or 0xFF, and neither is 0x28 or 0x30.
 *
 * Returns true only when a recognised ST25R3916 family identity came back.
 */
bool nfc_probe(void);

/*
 * Register the `nfc` shell command.
 *
 * The Log Viewer is forty columns wide and pins itself to the newest line, so
 * a diagnostic read there is clipped and has to be scrolled to -- and this
 * part's interesting output is a hex identity, a revision and an interrupt
 * level, which is exactly the sort of thing that gets truncated. A shell
 * command prints at full width to the terminal and can be run again without a
 * reboot, which is the difference between an instrument and a post mortem.
 *
 *   nfc               probe the reader and print what answered
 *   nfc reg <hex>     read one register
 */
bool nfc_register_command(void);

/*
 * Register access, in space A, on the bus nfc_probe() opened.
 *
 * These stay exported because the protocol layer above them needs the same
 * two primitive shapes and there is no reason to write them twice. They
 * return false when no device was added, which is also what a failed probe
 * leaves behind.
 */
bool nfc_read_reg(uint8_t reg, uint8_t *v);
bool nfc_write_reg(uint8_t reg, uint8_t v);

/*
 * What the reader found, in plain C so the port does not have to know about
 * RFAL's types. Filled by nfc_scan_step().
 */
typedef struct
{
    char tech[4];    /* "A", "B", "F", "V", "TB", "P2P" */
    char uid[33];    /* lowercase hex, no separators */
    int uid_len;     /* bytes, not characters */
    int sak;         /* NFC-A only; -1 for the technologies that have none */
    char tag[4];     /* "T1T".."T5T", "P2P", or empty when the type is unclassified */
    int ndef;        /* 1 = NDEF present, 0 = none, -1 = not attempted */
    int ndef_len;    /* message length in bytes when ndef is 1 */
    uint8_t ndef_raw[48]; /* the message itself, truncated to fit */
    int ndef_raw_len;
} nfc_card_t;

/*
 * The scanning half, in nfc_scan.cpp, which is C++ because ST's stack is.
 *
 * Split from the register access above on purpose: this is the vendor's
 * protocol code behind a struct, and everything it needs to know about this
 * board is the six numbers passed to nfc_scan_start().
 *
 * nfc_scan_step() is not blocking. Call it in a loop -- it returns 1 when a
 * card was just read, 0 when nothing happened, and -1 when the scanner is not
 * running. It is deliberately not a background task: keeping the field on for
 * ever is tens of milliamps on a watch, so the field is on only while somebody
 * is asking.
 */
bool nfc_scan_start(int host, int sck, int miso, int mosi, int cs, int irq);
int nfc_scan_step(nfc_card_t *out);
void nfc_scan_stop(void);

#ifdef __cplusplus
}
#endif

#endif
