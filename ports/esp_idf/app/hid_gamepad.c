/*
 * hid_gamepad.c - HID gamepad descriptors and reports, with no platform in them.
 *
 * A GAMEPAD IS NOT A MOUSE, and the difference is not the axes. A gamepad's
 * report descriptor declares Generic Desktop X and Y for its left stick, which is
 * exactly what hid_mouse_parse() looks for -- so an Xbox controller connects,
 * delivers reports, and is claimed as a mouse. On this board that showed up as
 * the desktop cursor jittering whenever the stick moved.
 *
 * What distinguishes them is the TOP-LEVEL COLLECTION: a gamepad declares
 * Generic Desktop / Game Pad (0x05), a mouse declares / Mouse (0x02) and a
 * keyboard / Keyboard (0x06).
 *
 * WHY THIS IS ITS OWN FILE. Parsing a descriptor is bit arithmetic over a byte
 * array and has no business depending on a window manager. Keeping it separate
 * is what lets tests/test_gamepad.c compile it on a host against the real
 * descriptor captured from the operator's controller -- the same rule the
 * Tamagotchi's tmc_core follows, and for the same reason: a rule that has only
 * ever been exercised by a human pressing buttons has been exercised once.
 *
 * THE LAYOUT WAS MEASURED, NOT ASSUMED. See tests/xbox_btle.h for the descriptor
 * and the reports captured from an Xbox Wireless Controller over Bluetooth LE.
 */
#include "hid_input.h"

#include <stdint.h>
#include <string.h>

/* ------------------------------------------------------------- gamepads */

/*
 * The same descriptor walk as hid_mouse_parse(), and deliberately a separate
 * function rather than a flag on it: the two answer different questions, and
 * sharing the parser meant a gamepad being accepted as a mouse.
 *
 * `collection_usage` is the discriminator. Generic Desktop 0x05 is Game Pad,
 * 0x04 is Joystick; a mouse is 0x02 and a keyboard 0x06. Measured on an Xbox
 * Wireless Controller over BLE: 0x05, report ID 1, 16 bytes, hat at bit 96,
 * fifteen buttons at bit 104.
 */
bool hid_gamepad_parse(hid_gamepad_layout_t *g, const uint8_t *d, size_t len)
{
    if (!g || !d || !len)
        return false;

    memset(g, 0, sizeof(*g));

    uint32_t page = 0, usz = 0, ucnt = 0, usage[16], umin = 0;
    int32_t lmin = 0;
    int nusage = 0;
    bool range = false;
    uint8_t id = 0;
    int depth = 0;
    uint16_t bits = 0;          /* bit offset within the report, after the ID */
    int button_at = 0;          /* bit offset of the first button, 0 when none */
    uint8_t button_id = 0;      /* THE ID AT THE MOMENT THE BUTTONS WERE DECLARED.
                                 * Not the last id in the descriptor: this one has
                                 * two (1 for the gamepad, 3 for a PID collection
                                 * at the end), and reading the last made the
                                 * button id 3 so every report was rejected as
                                 * "another report of this interface". */
    int collection_usage = -1;
    int coll_pending = -1;

    for (size_t i = 0; i < len;)
    {
        uint8_t b = d[i++];

        if (b == 0xFE)          /* a long item: skip it */
        {
            if (i >= len)
                break;
            i += 2u + d[i];
            continue;
        }

        int n = (b & 3) == 3 ? 4 : (b & 3);

        if (i + (size_t)n > len)
            break;

        uint32_t v = 0;

        for (int k = 0; k < n; k++)
            v |= (uint32_t)d[i + k] << (8 * k);

        int32_t sv = n == 1 ? (int8_t)v : n == 2 ? (int16_t)v : (int32_t)v;

        i += (size_t)n;

        int type = (b >> 2) & 3, tag = b >> 4;

        if (type == 1)          /* global */
        {
            if (tag == 0)
                page = v;
            else if (tag == 1)
                lmin = sv;
            else if (tag == 7)
                usz = v;
            else if (tag == 8)
            {
                id = (uint8_t)v;
                g->ids = true;
            }
            else if (tag == 9)
                ucnt = v;
            continue;
        }

        if (type == 2)          /* local */
        {
            if (tag == 0 && nusage < 16)
            {
                uint32_t u = n == 4 ? v : (page << 16 | v);

                usage[nusage++] = u;

                /* The first Usage after a Collection's Usage Page is the
                 * collection's own type. Captured here because it arrives as a
                 * local item before the Collection main item. */
                if (depth == 0 && coll_pending < 0)
                    coll_pending = (int)u;
            }
            else if (tag == 1)
            {
                umin = n == 4 ? v : (page << 16 | v);
                range = true;
            }
            continue;
        }

        /*
         * A COLLECTION IS A MAIN ITEM, so it CLEARS the local items -- and this
         * was the bug that made the whole parser useless on the first attempt.
         *
         * The collection's own Usage (Generic Desktop / Game Pad) is a LOCAL item
         * that arrives just before the Collection item. Resetting the usage list
         * only on Input/Output/Feature left those collection usages in the list,
         * so the next Input's `count` consumed them instead of the real field
         * usages: the first Input here declares two 16-bit fields (X and Y) with
         * usages [0x10005, 0x10001, 0x10030, 0x10031] still queued, which meant k=0
         * read 0x10005 and k=1 read 0x10001. Neither is X or Y, so the sticks were
         * never found while the hat and the buttons were -- a partial parse that
         * looked plausible.
         */
        if (type == 0 && tag == 10)         /* Collection */
        {
            if (depth == 0 && collection_usage < 0)
                collection_usage = coll_pending;

            depth++;
            coll_pending = -1;
            nusage = 0;
            range = false;
            continue;
        }

        if (type == 0 && tag == 12)         /* End Collection */
        {
            if (depth > 0)
                depth--;

            nusage = 0;
            range = false;
            continue;
        }

        if (type == 0 && tag == 8)          /* Input */
        {
            if (!(v & 1) && (v & 2) && usz <= 32)   /* data, variable */
            {
                for (uint32_t k = 0; k < ucnt; k++)
                {
                    uint32_t u = range ? umin + k
                                       : nusage ? usage[k < (uint32_t)nusage ? k : (uint32_t)nusage - 1]
                                                : 0;
                    hid_field_t f = { .pos = (uint16_t)(bits + k * usz),
                                      .size = (uint8_t)usz,
                                      .id = id,
                                      .sign = lmin < 0 };

                    if (u == 0x10039 && !g->hat.size)
                        g->hat = f;                     /* Generic Desktop Hat */
                    else if (u == 0x10030 && !g->x.size)
                        g->x = f;
                    else if (u == 0x10031 && !g->y.size)
                        g->y = f;
                    else if ((page == 0x09) && !button_at)
                    {
                        /* Button page: the first button sets where they all
                         * start, and the count of buttons this field declares
                         * is the total. Bits are numbered from the descriptor,
                         * so a controller with a gap in its numbering keeps it. */
                        button_at = bits + 1;
                        button_id = id;
                        g->count = (uint8_t)ucnt;
                    }
                }
            }

            bits = (uint16_t)(bits + usz * ucnt);
        }
        else if (type == 0 && tag == 9)     /* Output */
            bits = (uint16_t)(bits + usz * ucnt);
        else if (type == 0 && tag == 11)    /* Feature */
            bits = (uint16_t)(bits + usz * ucnt);

        if (type == 0)                      /* any main item ends local ones */
        {
            nusage = 0;
            range = false;
            coll_pending = -1;
        }
    }

    /* THE TEST THAT STOPS A MOUSE BEING TAKEN FOR A GAMEPAD AND VICE VERSA:
     * the top-level collection must be Game Pad (0x10005) or Joystick (0x10004). */
    if (collection_usage != 0x10005 && collection_usage != 0x10004)
        return false;

    if (button_at > 0)
    {
        g->button.pos = (uint16_t)(button_at - 1);
        g->button.size = 1;
        g->button.id = button_id;
        g->id = button_id;
        g->ok = true;
    }

    return g->ok;
}

/* The last decoded state. Written from the Bluetooth task, read from whoever
 * wants it; aligned ints, so atomic on this core. */
static volatile hid_gamepad_mask_t s_gp_mask;
static volatile uint8_t s_gp_hat = 0xFF;

hid_gamepad_mask_t hid_gamepad_state(void)
{
    return s_gp_mask;
}

uint8_t hid_gamepad_hat(void)
{
    return s_gp_hat;
}

hid_gamepad_mask_t hid_gamepad_report(const hid_gamepad_layout_t *g,
                                      const uint8_t *r, size_t len)
{
    if (!g || !g->ok || !r)
        return 0;

    if (g->ids && (len < 1 || r[0] != g->button.id))
        return 0;                       /* another report of this interface */

    hid_gamepad_mask_t mask = 0;

    for (int i = 0; i < g->count && i < 15; i++)
    {
        hid_field_t f = g->button;

        f.pos = (uint16_t)(g->button.pos + i);

        /* `field()` takes the mouse layout type; the extraction is the same
         * bit read, so it is done here rather than duplicated. */
        if (f.size == 0)
            continue;

        size_t skip = 0;

        if (g->ids)
        {
            if (len < 1 || r[0] != f.id)
                continue;
            skip = 1;
        }

        if (skip >= len)
            continue;

        /* Bit offset within the report body, after any report ID. An input
         * field that straddles a byte boundary is read bit by bit: HID packs
         * them without regard to bytes and the hat switch here is 4 bits into
         * the middle of byte 12. */
        uint32_t bit = f.pos;
        uint32_t val = 0;

        for (int k = 0; k < f.size; k++)
        {
            size_t byte = skip + (bit + (uint32_t)k) / 8;
            int off = (int)((bit + (uint32_t)k) % 8);

            if (byte >= len)
                break;

            val |= (uint32_t)((r[byte] >> off) & 1u) << k;
        }

        if (val)
            mask |= (hid_gamepad_mask_t)(1u << i);
    }

    uint8_t hat = 0xFF;

    if (g->hat.size)
    {
        size_t skip = g->ids ? 1 : 0;
        uint32_t val = 0;

        for (int k = 0; k < g->hat.size; k++)
        {
            size_t byte = skip + (g->hat.pos + (uint32_t)k) / 8;
            int off = (int)((g->hat.pos + (uint32_t)k) % 8);

            if (byte >= len)
                break;

            val |= (uint32_t)((r[byte] >> off) & 1u) << k;
        }

        /* The controller reports 1..8 clockwise from up, with 0 (or 15) for
         * centred -- MEASURED, and not the 0-based range a desk mouse-style
         * descriptor would use. Subtracting one here gives 0..7 clockwise with
         * 0xFF for centred, which is what a consumer wants. */
        if (val >= 1 && val <= 8)
            hat = (uint8_t)((val - 1) % 8);
    }

    s_gp_mask = mask;
    s_gp_hat = hat;
    return mask;
}

