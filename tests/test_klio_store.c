/*
 * test_klio_store.c - the pattern store's format, swept on a host.
 *
 * WHY THIS EXISTS. The store is the only thing standing between a learnt
 * gesture and the next boot, and its failure mode is silent: a pattern that
 * decodes to the wrong 157 bytes is written back into the engine and simply
 * never matches, which the wearer experiences as "it forgot". This project has
 * already shipped a pattern read from a part whose FIFO overflowed mid-save and
 * seen it score -8.837, so the decoder must refuse bad bytes rather than
 * interpret them.
 *
 * The file being tested is pure C with no ESP-IDF and no Bosch dependency, so
 * every branch below runs at a desk. Nothing here needs the watch.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "td_test.h"
#include "klio_store.h"

/* A realistic pattern: the engine reports 157 bytes. */
static void make_blob(uint8_t *b, int n, uint8_t seed)
{
    for (int i = 0; i < n; i++)
        b[i] = (uint8_t)(seed + i * 7);
}

static void rec(klio_store_rec_t *r, int slot, int len, uint8_t seed,
                const char *name, uint8_t flags)
{
    memset(r, 0, sizeof(*r));
    r->slot = (uint8_t)slot;
    r->flags = flags;
    r->len = (uint16_t)len;
    make_blob(r->blob, len, seed);
    if (name)
    {
        strncpy(r->name, name, KLIO_STORE_NAME_MAX - 1);
        r->name[KLIO_STORE_NAME_MAX - 1] = '\0';
    }
}

/* Recompute the trailer so a deliberately mutated payload is checksum-valid.
 * Used only to reach the checks that sit BEHIND the CRC check. */
static void fix_crc(uint8_t *buf, size_t len)
{
    uint32_t c = klio_store_crc32(buf, len - 4);
    buf[len - 4] = (uint8_t)(c & 0xFF);
    buf[len - 3] = (uint8_t)((c >> 8) & 0xFF);
    buf[len - 2] = (uint8_t)((c >> 16) & 0xFF);
    buf[len - 1] = (uint8_t)((c >> 24) & 0xFF);
}

static void test_crc_vectors(void)
{
    /* The canonical check value for CRC-32 (IEEE 802.3): the nine ASCII bytes
     * "123456789" give 0xCBF43926. Pinned against the standard rather than
     * against this implementation's own output, so a broken CRC cannot agree
     * with itself and pass. */
    CHECK_EQ(klio_store_crc32((const uint8_t *)"123456789", 9), 0xCBF43926u);
    CHECK_EQ(klio_store_crc32((const uint8_t *)"", 0), 0x00000000u);

    /* A single flipped bit must change it: this is the property the whole
     * corruption check rests on. */
    const uint8_t a[] = {0x00, 0x01, 0x02, 0x03};
    uint8_t b[sizeof(a)];
    memcpy(b, a, sizeof(a));
    b[2] ^= 0x01;
    CHECK(klio_store_crc32(a, sizeof(a)) != klio_store_crc32(b, sizeof(b)));
}

static void test_empty_round_trip(void)
{
    static klio_store_t s;
    static uint8_t buf[512];
    size_t len = 0;

    klio_store_init(&s);
    CHECK_EQ(s.count, 0);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);
    CHECK_EQ(len, (size_t)(8 + 4));

    static klio_store_t back;
    CHECK_EQ(klio_store_decode(&back, buf, len), KLIO_STORE_OK);
    CHECK_EQ(back.count, 0);
}

static void test_single_round_trip(void)
{
    static klio_store_t s;
    static uint8_t buf[1024];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r;
    rec(&r, 0, 157, 0x11, "wave", KLIO_STORE_FLAG_ENABLED);
    CHECK_EQ(klio_store_put(&s, &r), 0);
    CHECK_EQ(s.count, 1);

    const size_t want = 8 + 4 + (1 + 1 + 2 + KLIO_STORE_NAME_MAX) + 157;
    CHECK_EQ(klio_store_encoded_size(&s), want);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);
    CHECK_EQ(len, want);

    static klio_store_t back;
    CHECK_EQ(klio_store_decode(&back, buf, len), KLIO_STORE_OK);
    CHECK_EQ(back.count, 1);

    const klio_store_rec_t *g = klio_store_find(&back, 0);
    CHECK(g != NULL);
    if (g)
    {
        CHECK_EQ(g->slot, 0);
        CHECK_EQ(g->flags, KLIO_STORE_FLAG_ENABLED);
        CHECK_EQ(g->len, 157);
        CHECK(strcmp(g->name, "wave") == 0);
        CHECK(memcmp(g->blob, r.blob, 157) == 0);
    }
}

static void test_many_and_order(void)
{
    static klio_store_t s;
    static uint8_t buf[4096];
    size_t len = 0;

    klio_store_init(&s);
    for (int i = 0; i < 5; i++)
    {
        klio_store_rec_t r;
        rec(&r, i * 3, 157, (uint8_t)(0x20 + i), NULL, KLIO_STORE_FLAG_ENABLED);
        CHECK_EQ(klio_store_put(&s, &r), i);
    }
    CHECK_EQ(s.count, 5);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);

    static klio_store_t back;
    CHECK_EQ(klio_store_decode(&back, buf, len), KLIO_STORE_OK);
    CHECK_EQ(back.count, 5);
    /* Order is preserved, and every slot is independently reachable. */
    for (int i = 0; i < 5; i++)
    {
        CHECK_EQ(back.rec[i].slot, i * 3);
        const klio_store_rec_t *g = klio_store_find(&back, i * 3);
        CHECK(g != NULL);
        if (g)
            CHECK_EQ(g->blob[0], (uint8_t)(0x20 + i));
    }
}

static void test_replace_keeps_one_and_keeps_name(void)
{
    static klio_store_t s;
    klio_store_init(&s);

    klio_store_rec_t r;
    rec(&r, 4, 157, 0x01, "twist", KLIO_STORE_FLAG_ENABLED);
    CHECK_EQ(klio_store_put(&s, &r), 0);

    /* Same slot, new bytes, no name -- the re-learn case. The record must be
     * replaced rather than duplicated, and the label must survive. */
    rec(&r, 4, 157, 0x02, NULL, KLIO_STORE_FLAG_ENABLED);
    CHECK_EQ(klio_store_put(&s, &r), 0);
    CHECK_EQ(s.count, 1);

    const klio_store_rec_t *g = klio_store_find(&s, 4);
    CHECK(g != NULL);
    if (g)
    {
        CHECK_EQ(g->blob[0], 0x02);              /* the new bytes */
        CHECK(strcmp(g->name, "twist") == 0);    /* the old label */
    }
}

static void test_relearn_keeps_the_mapping_and_the_switch(void)
{
    /* RE-TRAINING A GESTURE MUST NOT UNDO WHAT IT DOES. The learn path re-saves
     * slot 0 with a fresh blob and knows nothing about the app it opens or
     * whether the wearer has detection switched on. If those were reset every
     * time the motion was re-trained, the mapping would vanish at exactly the
     * moment the wearer was least expecting it -- and the checkbox would appear
     * to switch itself off. */
    static klio_store_t s;
    klio_store_init(&s);

    klio_store_rec_t r;
    rec(&r, 0, 157, 0x01, "Files", KLIO_STORE_FLAG_ENABLED | KLIO_STORE_FLAG_ARMED);
    CHECK_EQ(klio_store_put(&s, &r), 0);

    /* The re-learn: a new blob, no name, and a flags byte from a caller that
     * only knows the engine enabled it. */
    rec(&r, 0, 157, 0x02, NULL, KLIO_STORE_FLAG_ENABLED);
    CHECK_EQ(klio_store_put(&s, &r), 0);
    CHECK_EQ(s.count, 1);

    const klio_store_rec_t *g = klio_store_find(&s, 0);
    CHECK(g != NULL);
    if (g)
    {
        CHECK_EQ(g->blob[0], 0x02);                    /* the new motion */
        CHECK(strcmp(g->name, "Files") == 0);          /* the same app */
        CHECK(g->flags & KLIO_STORE_FLAG_ARMED);       /* still switched on */
        CHECK(g->flags & KLIO_STORE_FLAG_ENABLED);     /* and still enabled */
    }

    /* And a re-learn is allowed to change the engine's enable bit without
     * touching the wearer's switch: the two are independent by design. */
    rec(&r, 0, 157, 0x03, NULL, 0);
    klio_store_put(&s, &r);
    g = klio_store_find(&s, 0);
    CHECK(g != NULL);
    if (g)
    {
        CHECK(!(g->flags & KLIO_STORE_FLAG_ENABLED));
        CHECK(g->flags & KLIO_STORE_FLAG_ARMED);
    }
}

static void test_replace_is_the_one_that_can_change_the_mapping(void)
{
    /* THE DEFECT THIS EXISTS FOR. put() carries the armed bit over, which is what
     * keeps a mapping alive across a re-learn -- but the settings path also went
     * through put(), so arming a pattern wrote the record, had the armed bit
     * restored from the old record, and then read back "not armed". On the watch
     * that presented as a checkbox that unchecked itself the instant it was
     * ticked, because the window redraws it from the store every tick.
     *
     * So the two operations are pinned apart here: replace() must win. */
    static klio_store_t s;
    klio_store_init(&s);

    klio_store_rec_t r;
    rec(&r, 0, 157, 0x01, "Files", KLIO_STORE_FLAG_ENABLED);
    CHECK_EQ(klio_store_put(&s, &r), 0);
    CHECK(!(klio_store_find(&s, 0)->flags & KLIO_STORE_FLAG_ARMED));

    /* ARM IT, the way the settings path does. */
    klio_store_rec_t w = *klio_store_find(&s, 0);
    w.flags |= KLIO_STORE_FLAG_ARMED;
    CHECK_EQ(klio_store_replace(&s, &w), 0);
    CHECK(klio_store_find(&s, 0)->flags & KLIO_STORE_FLAG_ARMED);

    /* AND DISARM IT the same way -- the other half of the same switch. */
    w = *klio_store_find(&s, 0);
    w.flags &= (uint8_t)~KLIO_STORE_FLAG_ARMED;
    CHECK_EQ(klio_store_replace(&s, &w), 0);
    CHECK(!(klio_store_find(&s, 0)->flags & KLIO_STORE_FLAG_ARMED));

    /* put(), by contrast, CANNOT arm: that is its documented job, and if this
     * ever starts passing the learn path will have stopped protecting mappings
     * and the two operations will be one again. */
    w = *klio_store_find(&s, 0);
    w.flags |= KLIO_STORE_FLAG_ARMED;
    w.name[0] = '\0';                       /* what a re-learn supplies */
    CHECK_EQ(klio_store_put(&s, &w), 0);
    CHECK(!(klio_store_find(&s, 0)->flags & KLIO_STORE_FLAG_ARMED));

    /* replace() can also change the app, including clearing it. */
    w = *klio_store_find(&s, 0);
    snprintf(w.name, sizeof(w.name), "Terminal");
    klio_store_replace(&s, &w);
    CHECK(strcmp(klio_store_find(&s, 0)->name, "Terminal") == 0);

    w = *klio_store_find(&s, 0);
    w.name[0] = '\0';
    klio_store_replace(&s, &w);
    CHECK(klio_store_find(&s, 0)->name[0] == '\0');
}

static void test_drop_and_find(void)
{
    static klio_store_t s;
    static uint8_t buf[2048];
    size_t len = 0;

    klio_store_init(&s);
    for (int i = 0; i < 3; i++)
    {
        klio_store_rec_t r;
        rec(&r, i, 157, (uint8_t)(0x30 + i), NULL, KLIO_STORE_FLAG_ENABLED);
        klio_store_put(&s, &r);
    }

    CHECK_EQ(klio_store_drop(&s, 1), 1);
    CHECK_EQ(s.count, 2);
    CHECK(klio_store_find(&s, 1) == NULL);
    /* The gap closes: slot 2 is now the second record, not the third. */
    CHECK_EQ(s.rec[1].slot, 2);

    CHECK_EQ(klio_store_drop(&s, 9), 0);   /* absent */

    /* And the shortened store still round-trips. */
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);
    static klio_store_t back;
    CHECK_EQ(klio_store_decode(&back, buf, len), KLIO_STORE_OK);
    CHECK_EQ(back.count, 2);
    CHECK(klio_store_find(&back, 0) != NULL);
    CHECK(klio_store_find(&back, 2) != NULL);
}

static void test_rejects_bad_records(void)
{
    static klio_store_t s;
    /* 32 records of 8-byte blobs with 24-byte names is 1,164 bytes, so this
     * buffer is sized from the format rather than guessed -- the name field
     * widening in version 2 is exactly what a 1 KB buffer would have hidden. */
    static uint8_t buf[4096];
    size_t len = 0;
    klio_store_rec_t r;

    /* Zero length is not a pattern. */
    klio_store_init(&s);
    rec(&r, 0, 0, 0, NULL, 0);
    CHECK_EQ(klio_store_put(&s, &r), -1);
    CHECK_EQ(s.count, 0);

    /* Longer than the engine's own blob field. */
    klio_store_init(&s);
    rec(&r, 0, 1, 0, NULL, 0);
    r.len = (uint16_t)(KLIO_STORE_BLOB_MAX + 1);
    CHECK_EQ(klio_store_put(&s, &r), -1);

    /* A full store refuses the next record rather than overwriting one. */
    klio_store_init(&s);
    for (int i = 0; i < KLIO_STORE_MAX; i++)
    {
        rec(&r, i, 8, (uint8_t)i, NULL, 0);
        CHECK_EQ(klio_store_put(&s, &r), i);
    }
    rec(&r, 200, 8, 0, NULL, 0);
    CHECK_EQ(klio_store_put(&s, &r), -1);
    CHECK_EQ(s.count, KLIO_STORE_MAX);
    /* A full table still encodes -- 32 records of 8 bytes is under 1 KB -- and
     * the size is exactly what the records plus the frame cost. */
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);
    CHECK_EQ(len, (size_t)(8 + 4 + KLIO_STORE_MAX * (1 + 1 + 2 + KLIO_STORE_NAME_MAX + 8)));
}

static void test_buffer_too_small(void)
{
    static klio_store_t s;
    static uint8_t buf[1024];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r;
    rec(&r, 0, 157, 0x40, NULL, 0);
    klio_store_put(&s, &r);

    const size_t need = klio_store_encoded_size(&s);
    CHECK_EQ(klio_store_encode(&s, buf, need - 1, &len), KLIO_STORE_E_SPACE);
}

static void test_corruption_is_refused(void)
{
    static klio_store_t s, back;
    static uint8_t buf[1024];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r;
    rec(&r, 0, 157, 0x50, "g", KLIO_STORE_FLAG_ENABLED);
    klio_store_put(&s, &r);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);

    /* One flipped bit anywhere in the payload. */
    for (int i = 0; i < 3; i++)
    {
        static uint8_t copy[1024];
        memcpy(copy, buf, len);
        const size_t at = 9 + (size_t)i * 40;   /* inside the records */
        copy[at] ^= 0x01;
        CHECK_EQ(klio_store_decode(&back, copy, len), KLIO_STORE_E_CRC);
        CHECK_EQ(back.count, 0);                /* nothing to restore from */
    }

    /* A truncated file: the trailer is no longer where the format says. */
    CHECK_EQ(klio_store_decode(&back, buf, len - 1), KLIO_STORE_E_CRC);

    /* A file too short to hold a header and a trailer at all. */
    CHECK_EQ(klio_store_decode(&back, buf, 4), KLIO_STORE_E_SHORT);

    /* Not ours. */
    static uint8_t other[64];
    memset(other, 0xAA, sizeof(other));
    CHECK_EQ(klio_store_decode(&back, other, sizeof(other)), KLIO_STORE_E_MAGIC);

    /* NULL input. */
    CHECK_EQ(klio_store_decode(&back, NULL, 0), KLIO_STORE_E_SHORT);
}

static void test_version_is_checked_before_crc(void)
{
    static klio_store_t s, back;
    static uint8_t buf[1024];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r;
    rec(&r, 0, 157, 0x60, NULL, 0);
    klio_store_put(&s, &r);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);

    /* A newer format must be refused by version, not misread as this one --
     * and the refusal must come before the CRC, so the message says "unknown
     * format" rather than "checksum". */
    buf[4] = 0x63;
    buf[5] = 0x00;
    fix_crc(buf, len);
    CHECK_EQ(klio_store_decode(&back, buf, len), KLIO_STORE_E_VERSION);
}

static void test_structural_lies_behind_a_valid_crc(void)
{
    static klio_store_t s, back;
    uint8_t buf[1024];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r;
    rec(&r, 0, 157, 0x70, NULL, 0);
    klio_store_put(&s, &r);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);

    /* EVERY VARIANT BELOW IS DECODED FROM A HEAP ALLOCATION OF EXACTLY THE
     * FILE'S LENGTH, and that is the point of the test rather than an
     * implementation detail. A record length the file cannot back is the case a
     * decoder must catch before it copies, and a wide static buffer hides the
     * mistake: the over-read lands in the same array and produces the right
     * answer by luck. The first version of this test did exactly that and let a
     * removed bounds check pass a full sanitised run. */
    for (int variant = 0; variant < 4; variant++)
    {
        uint8_t *copy = (uint8_t *)malloc(len);
        CHECK(copy != NULL);
        if (!copy)
            return;
        memcpy(copy, buf, len);

        int want;
        switch (variant)
        {
        case 0:
            /* The count claims one record more than the file holds. With the
             * CRC recomputed this is exactly the corruption a checksum cannot
             * catch, which is why every read is bounded against the buffer. */
            copy[6] = 2;
            copy[7] = 0;
            want = KLIO_STORE_E_RECORD;
            break;
        case 1:
            /* A record length that runs past the end of the file. */
            copy[8 + 2] = 244;
            copy[8 + 3] = 0;
            want = KLIO_STORE_E_RECORD;
            break;
        case 2:
            /* Zero length: no pattern at all, and not the empty store either. */
            copy[8 + 2] = 0;
            copy[8 + 3] = 0;
            want = KLIO_STORE_E_RECORD;
            break;
        default:
            /* More records claimed than the format allows. */
            copy[6] = (uint8_t)(KLIO_STORE_MAX + 1);
            copy[7] = 0;
            want = KLIO_STORE_E_FULL;
            break;
        }

        fix_crc(copy, len);
        CHECK_EQ(klio_store_decode(&back, copy, len), want);
        CHECK_EQ(back.count, 0);
        free(copy);
    }
}

static void test_duplicate_slot_in_a_file_is_refused(void)
{
    /* TWO RECORDS FOR ONE SLOT is not a file this build writes -- put()
     * replaces in place -- so in a file it is corruption that happens to leave
     * the checksum valid. A decoder that accepted it would let the last record
     * silently win, which is how a re-learned pattern could come back as the
     * old one. */
    static klio_store_t s, back;
    uint8_t buf[1024];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r;
    rec(&r, 0, 157, 0x10, NULL, KLIO_STORE_FLAG_ENABLED);
    klio_store_put(&s, &r);
    rec(&r, 1, 157, 0x20, NULL, KLIO_STORE_FLAG_ENABLED);
    klio_store_put(&s, &r);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);

    /* Record 1's slot byte, made to name slot 0. */
    const size_t rec0 = 8;
    const size_t rec1 = rec0 + (1 + 1 + 2 + KLIO_STORE_NAME_MAX) + 157;
    CHECK(rec1 < len - 4);

    uint8_t *copy = (uint8_t *)malloc(len);
    CHECK(copy != NULL);
    if (!copy)
        return;
    memcpy(copy, buf, len);
    copy[rec1] = 0;
    fix_crc(copy, len);
    CHECK_EQ(klio_store_decode(&back, copy, len), KLIO_STORE_E_RECORD);
    CHECK_EQ(back.count, 0);
    free(copy);
}

/* Build a version 1 file by hand: 16-byte names and the old flag meaning.
 * This is the file a watch running the previous firmware writes, and the
 * firmware that adds app mapping has to keep reading it. */
static size_t make_v1_file(uint8_t *out, size_t cap, uint8_t slot, int blob_len,
                           uint8_t flags, const char *name)
{
    size_t p = 0;
    out[p++] = 'K'; out[p++] = 'G'; out[p++] = 'S'; out[p++] = 'T';
    out[p++] = 1;   out[p++] = 0;          /* version 1 */
    out[p++] = 1;   out[p++] = 0;          /* one record */
    out[p++] = slot;
    out[p++] = flags;
    out[p++] = (uint8_t)(blob_len & 0xFF);
    out[p++] = (uint8_t)((blob_len >> 8) & 0xFF);
    /* Bounded by the literal's own length, not by the field: indexing a shorter
     * string up to 16 reads past its terminator, which is what the sanitised
     * target caught here. */
    const size_t nlen = name ? strlen(name) : 0;
    for (int i = 0; i < 16; i++)           /* v1 name field is 16 bytes */
        out[p++] = (uint8_t)((size_t)i < nlen ? name[i] : 0);
    for (int i = 0; i < blob_len; i++)
        out[p++] = (uint8_t)(0x40 + i);

    const uint32_t crc = klio_store_crc32(out, p);
    out[p] = (uint8_t)(crc & 0xFF);
    out[p + 1] = (uint8_t)((crc >> 8) & 0xFF);
    out[p + 2] = (uint8_t)((crc >> 16) & 0xFF);
    out[p + 3] = (uint8_t)((crc >> 24) & 0xFF);
    p += 4;
    (void)cap;
    return p;
}

static void test_version_1_files_still_load(void)
{
    /* A GESTURE LEARNT BEFORE THE FORMAT GREW MUST NOT DISAPPEAR on the firmware
     * that adds app mapping. It reads, it keeps its blob, and it is NOT armed --
     * there was no app to map it to, so silently watching for it would be a
     * change of behaviour nobody asked for. */
    static klio_store_t back;
    static uint8_t buf[1024];

    const size_t len = make_v1_file(buf, sizeof(buf), 0, 157, 1, "files");
    CHECK_EQ(klio_store_decode(&back, buf, len), KLIO_STORE_OK);
    CHECK_EQ(back.count, 1);

    const klio_store_rec_t *g = klio_store_find(&back, 0);
    CHECK(g != NULL);
    if (g)
    {
        CHECK_EQ(g->len, 157);
        CHECK_EQ(g->blob[0], 0x40);
        CHECK_EQ(g->blob[156], (uint8_t)(0x40 + 156));
        /* The v1 flag meant "enabled in the engine"; v1 had no armed concept. */
        CHECK(g->flags & KLIO_STORE_FLAG_ENABLED);
        CHECK(!(g->flags & KLIO_STORE_FLAG_ARMED));
        CHECK(strcmp(g->name, "files") == 0);
    }

    /* And re-encoding it produces a v2 file, so the next save migrates it. */
    static uint8_t out[2048];
    size_t out_len = 0;
    CHECK_EQ(klio_store_encode(&back, out, sizeof(out), &out_len), KLIO_STORE_OK);
    CHECK_EQ(out[4], 2);                   /* version 2 */
    CHECK_EQ(out[5], 0);

    static klio_store_t again;
    CHECK_EQ(klio_store_decode(&again, out, out_len), KLIO_STORE_OK);
    CHECK_EQ(again.count, 1);
    const klio_store_rec_t *h = klio_store_find(&again, 0);
    CHECK(h != NULL);
    if (h)
    {
        CHECK_EQ(h->len, 157);
        CHECK(strcmp(h->name, "files") == 0);
        CHECK(!(h->flags & KLIO_STORE_FLAG_ARMED));
    }
}

static void test_version_outside_the_range_is_refused(void)
{
    static klio_store_t s, back;
    static uint8_t buf[1024];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r = {0};
    r.slot = 0;
    r.len = 4;
    CHECK(klio_store_put(&s, &r) == 0);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);

    /* Zero and one-past-this-build are both refused, so the readable range is
     * pinned rather than implied. */
    for (int v = 0; v <= 3; v += 3)
    {
        uint8_t *copy = (uint8_t *)malloc(len);
        CHECK(copy != NULL);
        if (!copy)
            return;
        memcpy(copy, buf, len);
        copy[4] = (uint8_t)v;
        fix_crc(copy, len);
        CHECK_EQ(klio_store_decode(&back, copy, len), KLIO_STORE_E_VERSION);
        free(copy);
    }
}

static void test_every_single_bit_flip_is_refused(void)
{
    /* THE STRONGEST PROPERTY THE FORMAT CAN OFFER, checked exhaustively rather
     * than by sampling: flip one bit at EVERY byte position of a valid file and
     * the decoder must refuse all of them. A CRC that silently ignores one
     * region -- a name field excluded by accident, say -- fails here and nowhere
     * else.
     *
     * Each copy is allocated at EXACTLY the file's length, so the same loop is
     * also an over-read check when the suite is run under a sanitiser: a decoder
     * that walked past the end would fault instead of quietly reading padding.
     * That matters because the first version of this test used a static buffer,
     * which hid a removed bounds check -- the array was bigger than the file and
     * the over-read landed in the same object. */
    static klio_store_t s, back;
    uint8_t buf[1024];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r;
    rec(&r, 0, 157, 0xA0, "wave", KLIO_STORE_FLAG_ENABLED);
    klio_store_put(&s, &r);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);

    int refused = 0;
    for (size_t i = 0; i < len; i++)
    {
        uint8_t *p = (uint8_t *)malloc(len);
        if (!p)
        {
            CHECK(0);
            return;
        }
        memcpy(p, buf, len);
        p[i] ^= 0x01;
        const int rc = klio_store_decode(&back, p, len);
        CHECK(rc != KLIO_STORE_OK);
        CHECK_EQ(back.count, 0);
        if (rc != KLIO_STORE_OK)
            refused++;
        free(p);
    }
    CHECK_EQ(refused, (int)len);

    /* And the unmodified file, from an exact-size allocation, must still load. */
    uint8_t *ok = (uint8_t *)malloc(len);
    CHECK(ok != NULL);
    if (ok)
    {
        memcpy(ok, buf, len);
        CHECK_EQ(klio_store_decode(&back, ok, len), KLIO_STORE_OK);
        CHECK_EQ(back.count, 1);
        free(ok);
    }
}

static void test_trailing_bytes_are_refused(void)
{
    static klio_store_t s, back;
    static uint8_t buf[1200];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r;
    rec(&r, 0, 157, 0x80, NULL, 0);
    klio_store_put(&s, &r);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);

    /* Append a byte before the trailer and re-checksum: the count no longer
     * describes the file, so it is refused rather than silently ignored. */
    memmove(buf + len - 4 + 1, buf + len - 4, 4);
    buf[len - 4] = 0x00;
    len += 1;
    fix_crc(buf, len);
    CHECK_EQ(klio_store_decode(&back, buf, len), KLIO_STORE_E_RECORD);
}

static void test_names_are_always_terminated(void)
{
    static klio_store_t s, back;
    static uint8_t buf[1024];
    size_t len = 0;

    klio_store_init(&s);
    klio_store_rec_t r;
    rec(&r, 0, 157, 0x90, NULL, 0);
    /* Sixteen non-NUL bytes: a name field that fills the field completely. */
    memset(r.name, 'A', KLIO_STORE_NAME_MAX);
    klio_store_put(&s, &r);
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);
    CHECK_EQ(klio_store_decode(&back, buf, len), KLIO_STORE_OK);

    const klio_store_rec_t *g = klio_store_find(&back, 0);
    CHECK(g != NULL);
    if (g)
    {
        CHECK_EQ(g->name[KLIO_STORE_NAME_MAX - 1], '\0');
        CHECK_EQ(strlen(g->name), (size_t)(KLIO_STORE_NAME_MAX - 1));
    }
}

static void test_whole_part_table(void)
{
    /* The part reports 25 usable slots; the format carries them all in one
     * file and gives every one back. */
    static klio_store_t s, back;
    static uint8_t buf[8192];
    size_t len = 0;

    klio_store_init(&s);
    for (int i = 0; i < 25; i++)
    {
        klio_store_rec_t r;
        rec(&r, i, 157, (uint8_t)i, NULL, KLIO_STORE_FLAG_ENABLED);
        CHECK(klio_store_put(&s, &r) >= 0);
    }
    CHECK_EQ(s.count, 25);
    CHECK(klio_store_encoded_size(&s) <= sizeof(buf));
    CHECK_EQ(klio_store_encode(&s, buf, sizeof(buf), &len), KLIO_STORE_OK);
    CHECK_EQ(klio_store_decode(&back, buf, len), KLIO_STORE_OK);
    CHECK_EQ(back.count, 25);
    for (int i = 0; i < 25; i++)
    {
        const klio_store_rec_t *g = klio_store_find(&back, i);
        CHECK(g != NULL);
        if (g)
        {
            CHECK_EQ(g->len, 157);
            CHECK_EQ(g->blob[0], (uint8_t)i);
            CHECK_EQ(g->blob[156], (uint8_t)((uint8_t)i + 156 * 7));
        }
    }
}

int main(void)
{
    test_crc_vectors();
    test_empty_round_trip();
    test_single_round_trip();
    test_many_and_order();
    test_replace_keeps_one_and_keeps_name();
    test_relearn_keeps_the_mapping_and_the_switch();
    test_replace_is_the_one_that_can_change_the_mapping();
    test_drop_and_find();
    test_rejects_bad_records();
    test_buffer_too_small();
    test_corruption_is_refused();
    test_version_is_checked_before_crc();
    test_structural_lies_behind_a_valid_crc();
    test_duplicate_slot_in_a_file_is_refused();
    test_version_1_files_still_load();
    test_version_outside_the_range_is_refused();
    test_every_single_bit_flip_is_refused();
    test_trailing_bytes_are_refused();
    test_names_are_always_terminated();
    test_whole_part_table();
    return TD_TEST_RESULT();
}
