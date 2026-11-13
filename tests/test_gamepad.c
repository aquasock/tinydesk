/*
 * test_gamepad.c - the gamepad decoder, against a REAL controller's descriptor.
 *
 * WHY THIS EXISTS. The descriptor in `xbox_btle.h` was captured from the
 * operator's Xbox Wireless Controller over Bluetooth LE on 2026-11-12, together
 * with the raw input reports produced by pressing each button in turn. Those two
 * artefacts are the whole test input: the descriptor says where the fields are,
 * the reports say what a press actually looks like, and the decoder has to agree
 * with both.
 *
 * WHY IT MATTERS THAT IT IS NOT SYNTHETIC. This port already had one confident
 * wrong answer about this controller -- that it was Bluetooth Classic and could
 * not connect -- which source-reading produced and measurement overturned. So
 * the decoder is checked against the bytes the device really sent.
 *
 * WHAT IT DOES NOT CHECK: that the labels are right. The report capture has one
 * line per button press in a known order, and the assertions below encode that
 * ORDER, not any claim about which physical button is which. If the operator
 * pressed A, B, X, Y in that order then bit 0 is A by definition of the test.
 */
#include "td_test.h"

#include "hid_input.h"
#include "xbox_btle.h"

#include <string.h>

/* The descriptor must parse, and must be recognised as a GAMEPAD rather than
 * something else -- that recognition is what stops it being claimed as a mouse. */
static void test_descriptor_is_a_gamepad(void)
{
    hid_gamepad_layout_t g;

    CHECK(hid_gamepad_parse(&g, xbox_btle_desc, sizeof(xbox_btle_desc)));
    CHECK(g.ok);
    CHECK(g.ids);                 /* report ID 1, measured */
    CHECK_EQ(g.id, 1);
    CHECK_EQ(g.count, 15);        /* fifteen buttons, measured */
    CHECK_EQ(g.hat.size, 4);      /* a 4-bit hat switch */

    /*
     * THE FIELDS' OFFSETS, not just that they were found. These pin the bug that
     * the first version of this parser had and this test did not catch: local
     * items were cleared only on Input/Output/Feature, so the COLLECTION's own
     * usages (Generic Desktop / Game Pad, then / Pointer) stayed queued and the
     * first Input consumed them instead of X and Y. The sticks were then never
     * found -- and a test that only checked "the descriptor parses" passed anyway,
     * because the hat and the buttons still resolved.
     *
     * Measured offsets from the real descriptor: left stick at bit 0 and 16, the
     * hat at 96, the buttons at 104.
     */
    CHECK_EQ(g.x.pos, 0);
    CHECK_EQ(g.x.size, 16);
    CHECK_EQ(g.y.pos, 16);
    CHECK_EQ(g.y.size, 16);
    CHECK_EQ(g.hat.pos, 96);
    CHECK_EQ(g.button.pos, 104);
}

/*
 * A NOTE ON THE TRAP THIS FILE CANNOT TEST.
 *
 * `hid_mouse_parse()` ACCEPTS this exact descriptor, because a gamepad declares
 * Generic Desktop X and Y for its left stick and that is all the mouse parser
 * looks for. That is why the cursor jittered. The assertion would be worth
 * having, but it lives in hid_input.c, which needs a window manager and a board
 * config to compile -- so it is recorded here in prose rather than tested here.
 *
 * The consequence is a rule for bt_hid.c: TRY THE GAMEPAD PARSER FIRST. If the
 * mouse parser is asked first it claims the controller and the gamepad path is
 * never reached, which is exactly the behaviour being fixed.
 */

/* Feed a captured report the way bt_hid.c does: with the report ID PREPENDED.
 *
 * The dump captured the characteristic's payload, which does NOT begin with the
 * report ID -- byte 0 of a captured report is 0xeb, not 0x01. BLE carries the
 * report ID in the handle rather than in the data, and bt_hid.c reconstructs it
 * (`buf[0] = r->id`) before handing the report to a decoder. So the test has to
 * do the same or it is testing a different byte string from the one the firmware
 * will see -- which is exactly what made every button read as 0 on the first
 * attempt. */
static hid_gamepad_mask_t feed(const hid_gamepad_layout_t *g, const uint8_t *payload, int n)
{
    uint8_t buf[18];

    buf[0] = g->id;
    memcpy(buf + 1, payload, (size_t)n);

    return hid_gamepad_report(g, buf, (size_t)n + 1);
}

/* The button bits, from the CAPTURED reports rather than from invented bytes. */
static void test_each_button_bit(void)
{
    hid_gamepad_layout_t g;

    CHECK(hid_gamepad_parse(&g, xbox_btle_desc, sizeof(xbox_btle_desc)));

    /*
     * The presses, in the order the operator made them, and the bit each one
     * must set. Bytes 13 and 14 of the report carry buttons 1-8 and 9-15; the
     * numbers here are what the descriptor's layout implies for those bytes, and
     * the test fails if the decoder disagrees.
     *
     * NOTE THE GAP AT BIT 2. The four face buttons landed on bits 0, 1, 3 and 4,
     * so button 3 sets nothing on this controller. That is recorded rather than
     * "corrected", because shifting them to 0..3 would be inventing a layout to
     * match a guess -- and the descriptor, decoded independently, says the button
     * field is fifteen contiguous bits with no gap, so the gap is real and lives
     * in which bits the firmware sets, not in the descriptor.
     */
    static const struct { int idx; hid_gamepad_mask_t want; } cases[] = {
        { 0, 1u << 0  },      /* 1st press: byte13 0x01 */
        { 1, 1u << 1  },      /* 2nd press: byte13 0x02 */
        { 2, 1u << 3  },      /* 3rd press: byte13 0x08 */
        { 3, 1u << 4  },      /* 4th press: byte13 0x10 */
        { 8, 1u << 11 },      /* 9th press: byte14 0x08 */
        { 9, 1u << 10 },      /* 10th press: byte14 0x04 */
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        hid_gamepad_mask_t mask = feed(&g, xbox_btle_presses[cases[i].idx].rep,
                                       sizeof(xbox_btle_presses[cases[i].idx].rep));
        if (mask != cases[i].want)
            printf("  press %d: got 0x%04x want 0x%04x\n",
                   cases[i].idx + 1, (unsigned)mask, (unsigned)cases[i].want);

        CHECK_EQ(mask, cases[i].want);
    }

    /* Nothing pressed is nothing set. */
    {
        uint8_t payload[16] = { 0 };

        CHECK_EQ(feed(&g, payload, sizeof(payload)), 0);
    }
}

/* The hat, from the captured D-pad presses: 1 up, 5 down, 7 left, 3 right. */
static void test_hat(void)
{
    hid_gamepad_layout_t g;

    CHECK(hid_gamepad_parse(&g, xbox_btle_desc, sizeof(xbox_btle_desc)));

    static const struct { int idx; uint8_t want; const char *what; } cases[] = {
        { 4, 0, "up"    },     /* 5th press: byte12 0x01 */
        { 5, 4, "down"  },     /* 6th press: byte12 0x05 */
        { 6, 6, "left"  },     /* 7th press: byte12 0x07 */
        { 7, 2, "right" },     /* 8th press: byte12 0x03 */
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
    {
        (void)feed(&g, xbox_btle_presses[cases[i].idx].rep,
                   sizeof(xbox_btle_presses[cases[i].idx].rep));

        if (hid_gamepad_hat() != cases[i].want)
            printf("  hat %s: got %u want %u\n", cases[i].what,
                   (unsigned)hid_gamepad_hat(), (unsigned)cases[i].want);

        CHECK_EQ(hid_gamepad_hat(), cases[i].want);
    }

    /* A centred hat reports 0xFF, and the idle capture has the hat at 0. */
    {
        uint8_t payload[16] = { 0 };

        (void)feed(&g, payload, sizeof(payload));
        CHECK_EQ(hid_gamepad_hat(), 0xFF);
    }
}

/* A mouse descriptor must NOT be taken for a gamepad. */
static void test_mouse_is_not_a_gamepad(void)
{
    /* A three-button mouse with X/Y/wheel, Generic Desktop / Mouse (0x02). */
    static const uint8_t mouse[] = {
        0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,       /* Generic Desktop, Mouse, App */
        0x09, 0x01, 0xA1, 0x00,                   /* Pointer, Physical */
        0x05, 0x09, 0x19, 0x01, 0x29, 0x03,       /* Button 1..3 */
        0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02,
        0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
        0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38,
        0x15, 0x81, 0x25, 0x7F, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06,
        0xC0, 0xC0,
    };
    hid_gamepad_layout_t g;

    CHECK(!hid_gamepad_parse(&g, mouse, sizeof(mouse)));
}

int main(void)
{
    test_descriptor_is_a_gamepad();
    test_each_button_bit();
    test_hat();
    test_mouse_is_not_a_gamepad();

    return TD_TEST_RESULT();
}
