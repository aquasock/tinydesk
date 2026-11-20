/*
 * klio_store.h - the on-flash form of KLIO's learnt patterns (klio_store.c).
 *
 * WHY A PATTERN NEEDS A STORE AT ALL. KLIO's pattern table lives in the
 * BHI260AP's RAM, because the firmware the part runs is uploaded by the host on
 * first use (IMU-001). Nothing about that table is in mask ROM, so every boot
 * starts with an empty engine and every pattern learnt in a session is gone at
 * the next reset. The only thing that can outlive the part is the 157-byte blob
 * the engine hands back -- and `klio save` already reads exactly that out of it.
 * This file is where those bytes are kept between boots.
 *
 * PURE C, NO ESP-IDF AND NO BOSCH. The encoder and decoder are a header, a few
 * records and a CRC, so they can be walked exhaustively on a host
 * (tests/test_klio_store.c) rather than discovered on the watch. A corrupt
 * pattern is not a harmless failure here: this project has already produced one
 * that scored -8.837 because it was read from a part whose FIFO had overflowed
 * mid-save, and a blob written back into the engine from bad bytes would be
 * that failure again, silently. Everything is therefore length-checked and
 * checksummed before a single byte is trusted.
 *
 * THE FORMAT, little-endian throughout:
 *
 *   header   4  magic "KGST"
 *            2  version
 *            2  record count
 *   records  one or more, in the order they were put
 *   trailer  4  CRC-32 (IEEE 802.3, reflected) over everything above
 *
 * Each record is:
 *
 *            1  slot            0..255 (the engine's own pattern index)
 *            1  flags           bit 0 armed, bit 1 enabled
 *            2  length          1..244, the blob's real size
 *           24  name            NUL-padded; the app a gesture opens
 *            n  blob            `length` bytes
 *
 * THE NAME FIELD NAMES AN APP. It was reserved as 16 bytes in version 1 and this
 * is what it was reserved for -- and 16 turned out to be ONE BYTE TOO SMALL: the
 * longest registered app name on this board is "On-screen keyboard" at 18
 * characters, against 15 usable bytes. So version 2 widens it to 24 (23 usable).
 *
 * WHY BOTH "ARMED" AND "ENABLED" FLAGS. They answer different questions and only
 * one of them is ours to answer. `enabled` is the ENGINE's state -- whether the
 * recognition table has the pattern switched on -- and the port sets it when it
 * writes the pattern in. `armed` is the WEARER's switch: whether this gesture
 * should be watched for at all. A pattern can be enabled in the engine while the
 * wearer has switched detection off, and conflating the two would mean a
 * checkbox that fights the boot-time restore.
 *
 * VERSION 1 FILES STILL LOAD. A v1 record is 16 bytes of name and its flag bit 0
 * meant "enabled", so the decoder reads a v1 file into the same struct -- name
 * padded, armed left clear, which is the honest reading: a gesture stored before
 * there was anything to map it to has no app and should not silently start
 * firing. The next save rewrites the file as v2.
 */
#ifndef KLIO_STORE_H
#define KLIO_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define KLIO_STORE_MAGIC_0 'K'
#define KLIO_STORE_MAGIC_1 'G'
#define KLIO_STORE_MAGIC_2 'S'
#define KLIO_STORE_MAGIC_3 'T'

#define KLIO_STORE_VERSION 2

/* The only earlier version this build can still read. */
#define KLIO_STORE_VERSION_MIN 1

/* The engine's own ceiling, from bhy2_klio_defs.h: pattern_data is 244 bytes and
 * a learnt pattern is 157 of them. Kept at the struct's size rather than at 157
 * so a firmware that hands back a different length is stored and reported rather
 * than refused. */
#define KLIO_STORE_BLOB_MAX 244

/* Spare slots for a part that reports more than the eight the old host bank
 * mirrored. A real part reports 25; this leaves room to store the whole table
 * even if a future firmware offers more, and refuses beyond it rather than
 * growing without bound. Worst case in flash is 32 * 263 + 12 = 8,428 bytes. */
#define KLIO_STORE_MAX 32

/* 23 usable characters plus the terminator. Sized for "On-screen keyboard" (18)
 * with room to spare, because app names are upstream's to add. */
#define KLIO_STORE_NAME_MAX 24

/* The ENGINE's state: this pattern is switched on in the recognition table. */
#define KLIO_STORE_FLAG_ENABLED 0x01u

/* THE WEARER'S switch: watch for this gesture at all. Independent of the flag
 * above -- see the note in the header comment. */
#define KLIO_STORE_FLAG_ARMED 0x02u

typedef struct
{
    uint8_t slot;
    uint8_t flags;
    uint16_t len;
    char name[KLIO_STORE_NAME_MAX];
    uint8_t blob[KLIO_STORE_BLOB_MAX];
} klio_store_rec_t;

typedef struct
{
    int count;
    klio_store_rec_t rec[KLIO_STORE_MAX];
} klio_store_t;

/* Decode results. Every failure is distinguishable so the boot log can say what
 * was wrong with the file instead of "restore failed". */
enum
{
    KLIO_STORE_OK = 0,
    KLIO_STORE_E_SHORT = -1,    /* fewer bytes than a header and a trailer */
    KLIO_STORE_E_MAGIC = -2,    /* not one of ours */
    KLIO_STORE_E_VERSION = -3,  /* written by a format this build does not know */
    KLIO_STORE_E_CRC = -4,      /* bytes are not the bytes that were written */
    KLIO_STORE_E_RECORD = -5,   /* a record runs past the buffer or is malformed */
    KLIO_STORE_E_FULL = -6,     /* more records than KLIO_STORE_MAX */
    KLIO_STORE_E_SPACE = -7     /* the caller's buffer is too small */
};

/* Empty the store. */
void klio_store_init(klio_store_t *s);

/* Add or replace the record for this slot. Returns the record's index, or -1
 * when the store is full or the record is invalid (length 0 or over the blob
 * maximum). A slot can only appear once: putting it again replaces it in place,
 * so a re-learnt pattern does not shadow the old one on restore.
 *
 * PUT KEEPS THE MAPPING. When the slot already holds a record, an empty name
 * keeps the app that is there and the armed bit is carried over. That is the
 * correct behaviour for the LEARN path, which has a new blob and no opinion
 * about what the gesture does.
 *
 * PUT MUST NOT BE USED TO CHANGE THE MAPPING. It cannot: the armed bit it
 * carries over is the one it was asked to write. Use replace(). */
int klio_store_put(klio_store_t *s, const klio_store_rec_t *r);

/* Add or replace the record for this slot, taking it EXACTLY as given -- app name
 * and armed bit included. This is the operation for a settings change, where the
 * caller is deliberately saying what the record should now be. */
int klio_store_replace(klio_store_t *s, const klio_store_rec_t *r);

/* The record for this slot, or NULL. */
const klio_store_rec_t *klio_store_find(const klio_store_t *s, int slot);

/* Remove the record for this slot. 1 when something was removed. */
int klio_store_drop(klio_store_t *s, int slot);

/* Bytes encode() will produce, or 0 when the store does not fit the format. */
size_t klio_store_encoded_size(const klio_store_t *s);

/* Serialise into `out`. Returns KLIO_STORE_OK, or E_SPACE when it will not fit
 * and `cap` is too small. `*len` is written only on success. */
int klio_store_encode(const klio_store_t *s, uint8_t *out, size_t cap, size_t *len);

/* Parse and validate. On success the store holds exactly what was written and
 * any previous contents are gone; on failure the store is emptied, so a caller
 * can never restore from a half-parsed file. */
int klio_store_decode(klio_store_t *s, const uint8_t *in, size_t len);

/* A word for a decode result, for a log line. Never NULL. */
const char *klio_store_strerror(int rc);

/* The CRC-32 this format uses. Exposed because the test pins it against known
 * vectors rather than against itself. */
uint32_t klio_store_crc32(const uint8_t *data, size_t len);

#endif /* KLIO_STORE_H */
