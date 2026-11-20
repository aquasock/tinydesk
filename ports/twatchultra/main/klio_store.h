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
 *            1  flags           bit 0: enabled
 *            2  length          1..244, the blob's real size
 *           16  name            NUL-padded, reserved for the UI
 *            n  blob            `length` bytes
 *
 * THE NAME FIELD IS RESERVED, NOT USED. Nothing writes a label into it yet and
 * the restore path ignores it. It is here because adding it later would mean a
 * format bump and a migration for a field the app is certain to want; sixteen
 * bytes per pattern is 400 bytes at the part's maximum.
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

#define KLIO_STORE_VERSION 1

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

#define KLIO_STORE_NAME_MAX 16

#define KLIO_STORE_FLAG_ENABLED 0x01u

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
 * so a re-learnt pattern does not shadow the old one on restore. */
int klio_store_put(klio_store_t *s, const klio_store_rec_t *r);

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
