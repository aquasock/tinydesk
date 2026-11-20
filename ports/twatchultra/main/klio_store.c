/*
 * klio_store.c - encode, decode and validate the on-flash pattern store.
 *
 * See klio_store.h for why this exists and what the bytes mean. Nothing here
 * touches the part, the filesystem or ESP-IDF: it is a serialiser, and it is
 * kept that way so it can be tested off the board.
 */
#include "klio_store.h"

#include <string.h>

/* --------------------------------------------------------------- the CRC */

/* CRC-32, IEEE 802.3: poly 0xEDB88320 reflected, init 0xFFFFFFFF, final xor
 * 0xFFFFFFFF. Computed bitwise rather than from a 1 KB table, because this runs
 * once per save and once per boot -- a table would be a kilobyte of .rodata to
 * make two calls faster, which on this port is the wrong trade (EMU-018: a few
 * hundred bytes of static data has cost a peripheral its memory before now). */
uint32_t klio_store_crc32(const uint8_t *data, size_t len)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++)
    {
        c ^= data[i];
        for (int b = 0; b < 8; b++)
            c = (c & 1u) ? ((c >> 1) ^ 0xEDB88320u) : (c >> 1);
    }
    return c ^ 0xFFFFFFFFu;
}

/* ---------------------------------------------------------- little-endian */

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

/* --------------------------------------------------------------- the API */

#define HEADER_BYTES 8
#define TRAILER_BYTES 4
#define RECORD_FIXED_BYTES (1 + 1 + 2 + KLIO_STORE_NAME_MAX)

void klio_store_init(klio_store_t *s)
{
    if (!s)
        return;
    memset(s, 0, sizeof(*s));
}

static bool rec_valid(const klio_store_rec_t *r)
{
    return r && r->len > 0 && r->len <= KLIO_STORE_BLOB_MAX;
}

/* THE TWO WAYS TO WRITE A SLOT, and the difference is a real distinction rather
 * than spelling.
 *
 *   put()      for a caller that is updating the BLOB and nothing else. The learn
 *              path is the case: it has a fresh 157 bytes and no opinion about
 *              what the gesture opens or whether the wearer armed it, so those
 *              survive.
 *   replace()  for a caller that is changing the RECORD, including its mapping.
 *              The settings path is the case, and it must win -- a switch that
 *              the store overwrites with the old value is a switch that does
 *              not work.
 *
 * THE FIRST VERSION HAD ONLY put(), with the preservation unconditional, and
 * that is exactly how the checkbox came to flip straight back: arming a pattern
 * wrote the record through put(), put() dutifully restored the armed bit from
 * the record it was replacing, and the next tick read back "not armed". The host
 * tests missed it because the app test used a fake port that never reached the
 * store, and the store test only ever exercised put(). */
static int store_write(klio_store_t *s, const klio_store_rec_t *r, bool keep_mapping)
{
    if (!s || !rec_valid(r))
        return -1;

    /* REPLACE IN PLACE, because a slot is one pattern. Appending a second record
     * for the same slot would leave the decoder to decide which one wins, and a
     * restore that took the older of the two would silently undo a re-learn. */
    for (int i = 0; i < s->count; i++)
    {
        if (s->rec[i].slot == r->slot)
        {
            /* A CALLER THAT KNOWS NOTHING ABOUT THE MAPPING MUST NOT DESTROY IT.
             * The learn path re-saves slot 0 with a fresh blob and no idea what
             * the gesture is for; if that wiped the name the wearer's mapping
             * would vanish every time they re-trained the motion, which is the
             * worst possible moment for it to vanish. So an empty name means
             * "keep the one that is here", and the flag bit that says whether
             * detection is armed is carried over the same way -- the blob is
             * new, the decision about it is not.
             *
             * The old values are captured BEFORE the record is overwritten, or
             * there is nothing left to keep. */
            const bool keep_name = keep_mapping && (r->name[0] == '\0');
            char old_name[KLIO_STORE_NAME_MAX];
            memcpy(old_name, s->rec[i].name, KLIO_STORE_NAME_MAX);
            const uint8_t old_flags = s->rec[i].flags;

            s->rec[i] = *r;
            if (keep_name)
                memcpy(s->rec[i].name, old_name, KLIO_STORE_NAME_MAX);
            if (keep_mapping)
                s->rec[i].flags = (uint8_t)((s->rec[i].flags & ~KLIO_STORE_FLAG_ARMED) |
                                            (old_flags & KLIO_STORE_FLAG_ARMED));
            return i;
        }
    }

    if (s->count >= KLIO_STORE_MAX)
        return -1;

    s->rec[s->count] = *r;
    return s->count++;
}

int klio_store_put(klio_store_t *s, const klio_store_rec_t *r)
{
    return store_write(s, r, true);
}

int klio_store_replace(klio_store_t *s, const klio_store_rec_t *r)
{
    return store_write(s, r, false);
}

const klio_store_rec_t *klio_store_find(const klio_store_t *s, int slot)
{
    if (!s || slot < 0 || slot > 255)
        return NULL;
    for (int i = 0; i < s->count; i++)
    {
        if (s->rec[i].slot == (uint8_t)slot)
            return &s->rec[i];
    }
    return NULL;
}

int klio_store_drop(klio_store_t *s, int slot)
{
    if (!s)
        return 0;
    for (int i = 0; i < s->count; i++)
    {
        if (s->rec[i].slot == (uint8_t)slot)
        {
            /* Close the gap by shifting the tail down, so encoding never has to
             * skip a hole and the count stays the number of live records. */
            for (int k = i; k + 1 < s->count; k++)
                s->rec[k] = s->rec[k + 1];
            s->count--;
            memset(&s->rec[s->count], 0, sizeof(s->rec[0]));
            return 1;
        }
    }
    return 0;
}

size_t klio_store_encoded_size(const klio_store_t *s)
{
    if (!s || s->count < 0 || s->count > KLIO_STORE_MAX)
        return 0;
    size_t n = HEADER_BYTES + TRAILER_BYTES;
    for (int i = 0; i < s->count; i++)
    {
        if (!rec_valid(&s->rec[i]))
            return 0;
        n += RECORD_FIXED_BYTES + s->rec[i].len;
    }
    return n;
}

int klio_store_encode(const klio_store_t *s, uint8_t *out, size_t cap, size_t *len)
{
    if (!s || !out || !len)
        return KLIO_STORE_E_SPACE;

    const size_t need = klio_store_encoded_size(s);
    if (need == 0)
        return KLIO_STORE_E_RECORD;      /* empty is legal; invalid is not */
    if (cap < need)
        return KLIO_STORE_E_SPACE;

    size_t p = 0;
    out[p++] = KLIO_STORE_MAGIC_0;
    out[p++] = KLIO_STORE_MAGIC_1;
    out[p++] = KLIO_STORE_MAGIC_2;
    out[p++] = KLIO_STORE_MAGIC_3;
    put16(out + p, KLIO_STORE_VERSION);
    p += 2;
    put16(out + p, (uint16_t)s->count);
    p += 2;

    for (int i = 0; i < s->count; i++)
    {
        const klio_store_rec_t *r = &s->rec[i];
        out[p++] = r->slot;
        out[p++] = r->flags;
        put16(out + p, r->len);
        p += 2;
        memcpy(out + p, r->name, KLIO_STORE_NAME_MAX);
        p += KLIO_STORE_NAME_MAX;
        memcpy(out + p, r->blob, r->len);
        p += r->len;
    }

    /* The CRC covers the header and every record and is written last, so a file
     * truncated anywhere in a record fails the check rather than losing its tail
     * quietly. */
    put32(out + p, klio_store_crc32(out, p));
    p += 4;

    *len = p;
    return KLIO_STORE_OK;
}

int klio_store_decode(klio_store_t *s, const uint8_t *in, size_t len)
{
    if (!s)
        return KLIO_STORE_E_SHORT;

    /* EMPTY THE STORE FIRST, AND AGAIN ON EVERY FAILURE, so that nothing
     * partial is ever left for a caller to restore from. The first version of
     * this function emptied once at the top and then returned from inside the
     * record loop -- so a file whose first record parsed and whose second did
     * not left ONE pattern loaded while reporting failure. The test caught it,
     * and the rule it enforces is the one that matters: a failed decode must
     * leave the engine's table untouched, because the caller will fall back to
     * whatever the part already holds. */
    klio_store_init(s);

    int rc;

    if (!in || len < HEADER_BYTES + TRAILER_BYTES)
    {
        rc = KLIO_STORE_E_SHORT;
        goto fail;
    }

    if (in[0] != KLIO_STORE_MAGIC_0 || in[1] != KLIO_STORE_MAGIC_1 ||
        in[2] != KLIO_STORE_MAGIC_2 || in[3] != KLIO_STORE_MAGIC_3)
    {
        rc = KLIO_STORE_E_MAGIC;
        goto fail;
    }

    /* TWO VERSIONS ARE READABLE, AND THAT IS DELIBERATE. Version 1 is what this
     * project wrote before a pattern could name an app. Refusing it would mean a
     * wearer's existing gesture disappearing on the firmware that was supposed
     * to keep it, which is exactly the failure persistence exists to prevent. A
     * v1 file is read into the same struct with an empty name and armed clear --
     * there was nothing to map it to, and a gesture should not start firing
     * because the format grew. */
    const uint16_t version = get16(in + 4);
    if (version < KLIO_STORE_VERSION_MIN || version > KLIO_STORE_VERSION)
    {
        rc = KLIO_STORE_E_VERSION;
        goto fail;
    }
    const size_t name_bytes = (version == 1) ? 16u : (size_t)KLIO_STORE_NAME_MAX;
    const size_t record_fixed = 1 + 1 + 2 + name_bytes;

    /* CHECK THE CRC BEFORE TRUSTING ANY FIELD. The count and the lengths come
     * out of the same bytes the checksum covers, so a corrupted length cannot
     * walk the parser off the end of the buffer. */
    if (get32(in + len - TRAILER_BYTES) != klio_store_crc32(in, len - TRAILER_BYTES))
    {
        rc = KLIO_STORE_E_CRC;
        goto fail;
    }

    /* get16() cannot be negative, so the only question is whether the file
     * claims more records than this build can hold. */
    const int count = (int)get16(in + 6);
    if (count > KLIO_STORE_MAX)
    {
        rc = KLIO_STORE_E_FULL;
        goto fail;
    }

    size_t p = HEADER_BYTES;
    const size_t end = len - TRAILER_BYTES;

    for (int i = 0; i < count; i++)
    {
        if (p + record_fixed > end)
        {
            rc = KLIO_STORE_E_RECORD;
            goto fail;
        }

        klio_store_rec_t r;
        memset(&r, 0, sizeof(r));
        r.slot = in[p++];
        r.flags = in[p++];
        r.len = get16(in + p);
        p += 2;
        if (!rec_valid(&r))
        {
            rc = KLIO_STORE_E_RECORD;
            goto fail;
        }
        /* THE BOUNDS CHECK MUST COME BEFORE THE COPY. Removing it does not
         * change the answer on a wide buffer -- the over-read lands inside the
         * array and the trailing-byte check below rejects the file anyway -- but
         * it reads past a caller's exact-size allocation, which is undefined
         * behaviour and is what the sanitised test target exists to catch. */
        if (p + name_bytes + r.len > end)
        {
            rc = KLIO_STORE_E_RECORD;
            goto fail;
        }

        /* THE NAME IS COPIED AT THE FILE'S OWN WIDTH and the rest of the field
         * left as the memset put it, so a v1 file's 16 bytes land in the first
         * 16 of a 24-byte field rather than being misread as 24. */
        memcpy(r.name, in + p, name_bytes);
        /* A name is an app name, not data: make sure it is terminated whatever
         * the file says, so nothing downstream can walk past it. */
        r.name[KLIO_STORE_NAME_MAX - 1] = '\0';
        p += name_bytes;

        memcpy(r.blob, in + p, r.len);
        p += r.len;

        /* A duplicate slot in a file this build did not write is malformed
         * rather than a last-one-wins choice. */
        if (klio_store_find(s, r.slot))
        {
            rc = KLIO_STORE_E_RECORD;
            goto fail;
        }

        if (klio_store_put(s, &r) < 0)
        {
            rc = KLIO_STORE_E_FULL;
            goto fail;
        }
    }

    /* Trailing bytes between the last record and the CRC mean the count does
     * not describe the file, which is exactly the class of corruption the CRC
     * cannot see (a stale tail with a recomputed checksum). */
    if (p != end)
    {
        rc = KLIO_STORE_E_RECORD;
        goto fail;
    }

    return KLIO_STORE_OK;

fail:
    klio_store_init(s);
    return rc;
}

const char *klio_store_strerror(int rc)
{
    switch (rc)
    {
    case KLIO_STORE_OK:         return "ok";
    case KLIO_STORE_E_SHORT:    return "file too short";
    case KLIO_STORE_E_MAGIC:    return "not a pattern store";
    case KLIO_STORE_E_VERSION:  return "unknown format version";
    case KLIO_STORE_E_CRC:      return "checksum mismatch";
    case KLIO_STORE_E_RECORD:   return "malformed record";
    case KLIO_STORE_E_FULL:     return "too many patterns";
    case KLIO_STORE_E_SPACE:    return "buffer too small";
    default:                    return "error";
    }
}
