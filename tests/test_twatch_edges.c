/*
 * test_twatch_edges.c - the T-Watch Ultra's glass margins: where the buttons
 * are, which one a finger lands on, and the two coordinate spaces they meet in.
 *
 * WHY THIS EXISTS.
 *
 * Three faults in this subject matter reached the glass before anything caught
 * them at a desk, and every one was pure arithmetic:
 *
 *   1. The hit test compared panel coordinates against controller coordinates,
 *      so the left column was dead over its outer 22 pixels -- 51 per cent of
 *      its face -- while the bottom strip was merely shifted 22 pixels and
 *      looked fine. Reported as "the left panel buttons dont seem to register".
 *
 *   2. The inverse of the slot formula divided by the span instead of inverting
 *      the floor, which put a press on the wrong side of the gap at two of the
 *      four separators.
 *
 *   3. The two columns were compared as an offset instead of a mirror, which
 *      left their rounded corners facing the same way.
 *
 * None of those needs hardware to find. What they need is a sweep: walk every
 * point of the glass and check the classifier against facts stated
 * independently of it. That is what this test does, and it is why it exists as
 * a committed test rather than a program in a temporary directory.
 *
 * WHAT IT COVERS AND WHAT IT DOES NOT.
 *
 * It covers `main/edges.h`, which is the shared contract: the geometry
 * constants, the slot boundaries, and the hit test. It does not cover
 * `main/display.c`, which paints the same shapes, because that file needs
 * ESP-IDF's SPI headers and cannot be compiled on a host. The painter and the
 * hit test still share `edges.h` for the numbers, so the fault class this test
 * is aimed at -- two copies of the geometry drifting apart -- is covered. A
 * fault inside the painter's own shape arithmetic is not.
 *
 * THE FACTS ARE RESTATED, DELIBERATELY.
 *
 * The reference below is not built from edges.h's macros, because a test that
 * recomputes the code's own formula proves only that the code equals itself.
 * The numbers here are the measured ones: a 410x502 glass, a 320x416
 * framebuffer inset 45 pixels on each side, and panel RAM 22 columns wider than
 * the visible glass. Everything else is derived from those, and the code is
 * checked against the result.
 */
#include "td_test.h"

#include "edges.h"
#include "font8x16.h"

/* ---- the measured facts, as numbers rather than as the port's macros ------ */
#define GLASS_W     410
#define GLASS_H     502
#define FB_INSET    45                  /* on every side */
#define FB_W        320
#define FB_H        416
#define RAM_OFFSET  22                  /* controller RAM columns left of glass */

/* Framebuffer in controller coordinates. */
#define CTL_FB_X0   (FB_INSET + RAM_OFFSET)         /* 67  */
#define CTL_FB_X1   (CTL_FB_X0 + FB_W - 1)          /* 386 */
#define CTL_FB_Y0   FB_INSET                        /* 45  */
#define CTL_FB_Y1   (CTL_FB_Y0 + FB_H - 1)          /* 460 */

/* The two columns and the strip, in controller coordinates. These bounds are
 * the design: the columns run the framebuffer's full height so that the three
 * margins form a frame around it, and the strip is exactly the framebuffer's
 * width so that it stops where the desktop stops. */
#define CTL_LEFT_X0   RAM_OFFSET                    /* 22  */
#define CTL_LEFT_X1   (CTL_FB_X0 - 1)               /* 66  */
#define CTL_RIGHT_X0  (CTL_FB_X1 + 1)               /* 387 */
#define CTL_RIGHT_X1  (RAM_OFFSET + GLASS_W - 1)    /* 431 */
#define CTL_BTN_Y0    (CTL_FB_Y1 + 1)               /* 461 */
#define CTL_BTN_Y1    (GLASS_H - 1)                 /* 501 */

#define N_COL 5
#define N_BTN 3

/* The slot boundaries, stated as the layout they are rather than recomputed.
 *
 * 416 rows over five buttons cannot divide evenly: four slots of 83 and one of
 * 84, with the extra row in the last. 320 columns over three gives two of 105
 * and one of 106, with the extra in the first. Both distributions are visible
 * on the glass and both are load-bearing for the hit test, so they are written
 * here as a table that the code must reproduce. */
static const int k_left_y[6] = { 45, 128, 211, 294, 377, 461 };
static const int k_btn_x[4] = { 67, 173, 280, 387 };

/* Panel coordinates are what the touch controller reports: the visible glass,
 * zero at its own top-left. */
static int to_panel(int ctl_x)
{
    return ctl_x - RAM_OFFSET;
}

/* ---- the reference classifier --------------------------------------------
 *
 * Written in controller coordinates from the tables above, so that it cannot
 * inherit a mistake from edges_hit()'s conversion or from its slot arithmetic.
 * A point is in a margin by bounds alone: the gaps between buttons are not
 * holes in the hit test, they belong to a neighbouring button, which is what
 * makes a press on a one-pixel seam do something rather than nothing.
 */
static int ref_region(int ctl_x, int ctl_y, int *index)
{
    *index = -1;

    if (ctl_y >= CTL_FB_Y0 && ctl_y <= CTL_FB_Y1)
    {
        if (ctl_x >= CTL_LEFT_X0 && ctl_x <= CTL_LEFT_X1)
        {
            for (int i = N_COL - 1; i >= 0; i--)
                if (ctl_y >= k_left_y[i])
                {
                    *index = i;
                    break;
                }
            return EDGE_LEFT;
        }
        if (ctl_x >= CTL_RIGHT_X0 && ctl_x <= CTL_RIGHT_X1)
        {
            for (int i = N_COL - 1; i >= 0; i--)
                if (ctl_y >= k_left_y[i])
                {
                    *index = i;
                    break;
                }
            return EDGE_RIGHT;
        }
        return EDGE_NONE;               /* the framebuffer: not a touch target */
    }

    if (ctl_y >= CTL_BTN_Y0 && ctl_y <= CTL_BTN_Y1 &&
        ctl_x >= CTL_FB_X0 && ctl_x <= CTL_FB_X1)
    {
        for (int i = N_BTN - 1; i >= 0; i--)
            if (ctl_x >= k_btn_x[i])
            {
                *index = i;
                break;
            }
        return EDGE_BOTTOM;
    }

    if (ctl_y < CTL_FB_Y0 && ctl_x >= CTL_LEFT_X0 && ctl_x <= CTL_RIGHT_X1)
        return EDGE_TOP;                /* reserved for status icons, inert */

    return EDGE_NONE;
}

/* ---- the geometry the port publishes against the facts ------------------- */

static void test_geometry_matches_the_glass(void)
{
    CHECK_EQ(DISPLAY_W, FB_W);
    CHECK_EQ(DISPLAY_H, FB_H);
    CHECK_EQ(DISPLAY_ORIGIN_X, FB_INSET);
    CHECK_EQ(DISPLAY_ORIGIN_Y, FB_INSET);
    CHECK_EQ(DISPLAY_PANEL_W, GLASS_W);
    CHECK_EQ(DISPLAY_PANEL_H, GLASS_H);
    CHECK_EQ(PANEL_COL_OFFSET, RAM_OFFSET);

    /* The framebuffer, in controller coordinates. */
    CHECK_EQ(FB_CX0, CTL_FB_X0);
    CHECK_EQ(FB_CX1, CTL_FB_X1);
    CHECK_EQ(FB_CY0, CTL_FB_Y0);
    CHECK_EQ(FB_CY1, CTL_FB_Y1);

    /* The three margins. Their x bounds are what makes them a frame rather
     * than three unrelated strips: each column ends exactly where the
     * framebuffer begins, and the strip spans exactly the framebuffer. */
    CHECK_EQ(LEFT_X0, CTL_LEFT_X0);
    CHECK_EQ(LEFT_X1, CTL_LEFT_X1);
    CHECK_EQ(RIGHT_X0, CTL_RIGHT_X0);
    CHECK_EQ(RIGHT_X1, CTL_RIGHT_X1);
    CHECK_EQ(BTN_X0, CTL_FB_X0);
    CHECK_EQ(BTN_X1, CTL_FB_X1);
    CHECK_EQ(BTN_Y0, CTL_BTN_Y0);
    CHECK_EQ(BTN_Y1, CTL_BTN_Y1);
    CHECK_EQ(COL_Y0, CTL_FB_Y0);
    CHECK_EQ(COL_Y1, CTL_FB_Y1);

    /* The top margin is everything above the framebuffer, and nothing claims
     * it. */
    CHECK_EQ(TOP_Y1, CTL_FB_Y0 - 1);

    CHECK_EQ(COL_N, N_COL);
    CHECK_EQ(BTN_N, N_BTN);
}

/* The slot boundaries must be the table, both forwards and backwards. */
static void test_slot_boundaries(void)
{
    int i;

    for (i = 0; i <= N_COL; i++)
        CHECK_EQ(col_slot_y(i), k_left_y[i]);
    for (i = 0; i <= N_BTN; i++)
        CHECK_EQ(btn_slot_x(i), k_btn_x[i]);

    /* Each boundary maps back to its own slot. This is the property that the
     * obvious division-by-the-span inverse failed at two of the four
     * separators: floor(i*n/d) and its inverse are not floor(y*n/d). */
    for (i = 0; i < N_COL; i++)
        CHECK_EQ(col_slot_at(k_left_y[i]), i);
    for (i = 0; i < N_BTN; i++)
        CHECK_EQ(btn_slot_at(k_btn_x[i]), i);

    /* Every row and column of a slot resolves to that slot, not just its first
     * one, and the index never jumps by more than one as the finger moves --
     * a jump would be a dead band with no button owning it. */
    for (i = 0; i < N_COL; i++)
    {
        for (int y = k_left_y[i]; y < k_left_y[i + 1]; y++)
            CHECK_EQ(col_slot_at(y), i);
    }
    for (i = 0; i < N_BTN; i++)
    {
        for (int x = k_btn_x[i]; x < k_btn_x[i + 1]; x++)
            CHECK_EQ(btn_slot_at(x), i);
    }

    for (int y = CTL_FB_Y0 + 1; y <= CTL_FB_Y1; y++)
        CHECK(col_slot_at(y) - col_slot_at(y - 1) <= 1);
    for (int x = CTL_FB_X0 + 1; x <= CTL_FB_X1; x++)
        CHECK(btn_slot_at(x) - btn_slot_at(x - 1) <= 1);

    /* And the ends clamp rather than running off the array. */
    CHECK_EQ(col_slot_at(CTL_FB_Y0 - 100), 0);
    CHECK_EQ(col_slot_at(CTL_FB_Y1 + 100), N_COL - 1);
    CHECK_EQ(btn_slot_at(CTL_FB_X0 - 100), 0);
    CHECK_EQ(btn_slot_at(CTL_FB_X1 + 100), N_BTN - 1);
}

/*
 * THE SWEEP.
 *
 * Every point of the visible glass, in panel coordinates -- the numbers the
 * touch controller actually reports -- compared against the reference
 * classifier. This is the test that would have caught fault 1 outright: with
 * the coordinate conversion missing, 30,250 points disagree and each column
 * keeps a claim on only 9,568 of its 18,720 -- 51 per cent dead. On the glass
 * that presented as the left column not registering at all while the bottom
 * strip merely felt slightly off, because the strip survived a 22-pixel shift
 * and a column does not survive losing its outer half.
 *
 * Counts are checked as well as regions, because a classifier that puts
 * everything in the right region but assigns the wrong button would otherwise
 * pass on a sweep that only compared region names.
 */
static void test_every_point_of_the_glass(void)
{
    int disagree_region = 0, disagree_index = 0;
    int n_left = 0, n_bottom = 0, n_right = 0, n_top = 0, n_none = 0;
    int first_bad_x = -1, first_bad_y = -1;

    for (int y = 0; y < GLASS_H; y++)
    {
        for (int x = 0; x < GLASS_W; x++)
        {
            edge_hit_t hit;
            int ref_index = -1;
            int ref = ref_region(x + RAM_OFFSET, y, &ref_index);

            edges_hit(x, y, &hit);

            if ((int)hit.region != ref)
            {
                if (disagree_region == 0)
                {
                    first_bad_x = x;
                    first_bad_y = y;
                }
                disagree_region++;
            }
            else if (hit.index != ref_index)
            {
                if (disagree_index == 0)
                {
                    first_bad_x = x;
                    first_bad_y = y;
                }
                disagree_index++;
            }

            switch (hit.region)
            {
            case EDGE_LEFT: n_left++; break;
            case EDGE_BOTTOM: n_bottom++; break;
            case EDGE_RIGHT: n_right++; break;
            case EDGE_TOP: n_top++; break;
            default: n_none++; break;
            }
        }
    }

    if (disagree_region || disagree_index)
        printf("  first disagreement at panel (%d,%d)\n", first_bad_x, first_bad_y);

    CHECK_EQ(disagree_region, 0);
    CHECK_EQ(disagree_index, 0);

    /* The counts are the painted areas, derived from the facts rather than
     * from the code: a 45-wide column over 416 rows is 18,720 points, a
     * 320x41 strip is 13,120, and the top margin is everything above the
     * framebuffer across the full 410. */
    CHECK_EQ(n_left, 45 * FB_H);
    CHECK_EQ(n_right, 45 * FB_H);
    CHECK_EQ(n_bottom, FB_W * (CTL_BTN_Y1 - CTL_BTN_Y0 + 1));
    CHECK_EQ(n_top, FB_INSET * GLASS_W);
    CHECK_EQ(n_left + n_right + n_bottom + n_top + n_none, GLASS_W * GLASS_H);
}

/*
 * THE COORDINATE SPACE, stated as its own test.
 *
 * This is fault 1 in miniature and it is worth a named check even though the
 * sweep covers it, because the failure is silent and reads as a hardware
 * problem: with edges_hit() given controller coordinates instead of panel ones,
 * the left column's outer half does not register. Panel x 0 is the glass's left
 * edge and must land on the left column; panel x 409 is its right edge and must
 * land on the right column.
 */
static void test_panel_coordinates(void)
{
    edge_hit_t hit;

    edges_hit(0, CTL_FB_Y0, &hit);
    CHECK_EQ(hit.region, EDGE_LEFT);
    CHECK_EQ(hit.index, 0);

    edges_hit(0, CTL_FB_Y1, &hit);
    CHECK_EQ(hit.region, EDGE_LEFT);
    CHECK_EQ(hit.index, N_COL - 1);

    edges_hit(GLASS_W - 1, CTL_FB_Y0, &hit);
    CHECK_EQ(hit.region, EDGE_RIGHT);
    CHECK_EQ(hit.index, 0);

    edges_hit(GLASS_W - 1, CTL_FB_Y1, &hit);
    CHECK_EQ(hit.region, EDGE_RIGHT);
    CHECK_EQ(hit.index, N_COL - 1);

    /* The framebuffer is not a touch target, and neither is the gap between
     * the strip and the columns. */
    edges_hit(FB_INSET + FB_W / 2, CTL_FB_Y0 + FB_H / 2, &hit);
    CHECK_EQ(hit.region, EDGE_NONE);

    edges_hit(0, CTL_BTN_Y0, &hit);
    CHECK_EQ(hit.region, EDGE_NONE);        /* below the left column */

    edges_hit(GLASS_W - 1, CTL_BTN_Y0, &hit);
    CHECK_EQ(hit.region, EDGE_NONE);        /* below the right column */
}

/*
 * THE MIRROR, stated as a 180-degree rotation rather than as an offset.
 *
 * This is fault 3: comparing the columns as one shifted by a constant lines
 * their surfaces up but does not mirror them, so the rounded corners face the
 * same way and the columns stop looking like a matched pair. A true mirror
 * about the centre of the glass is x_right = (CTL_LEFT_X0 + CTL_RIGHT_X1) - x_left.
 */
static void test_the_columns_mirror(void)
{
    const int axis_sum = CTL_LEFT_X0 + CTL_RIGHT_X1;   /* 453 */
    int mismatches = 0;

    CHECK_EQ((CTL_LEFT_X0 + CTL_RIGHT_X1), 22 + 431);

    for (int y = CTL_FB_Y0; y <= CTL_FB_Y1; y++)
    {
        for (int x = CTL_LEFT_X0; x <= CTL_LEFT_X1; x++)
        {
            edge_hit_t l, r;

            edges_hit(to_panel(x), y, &l);
            edges_hit(to_panel(axis_sum - x), y, &r);

            if (l.region != EDGE_LEFT || r.region != EDGE_RIGHT || l.index != r.index)
                mismatches++;
        }
    }
    CHECK_EQ(mismatches, 0);

    /* What this can and cannot see: the index depends only on y, so a shifted
     * comparison agrees index-wise and the mirror fault was really in the
     * painter's arcs, which this test does not reach. What it does establish is
     * that neither column is offset, clipped or short of the other -- the
     * shape of the frame -- which is the part the hit test owns. */
}

/*
 * THE LABELS THE MARGINS CARRY, in both modes.
 *
 * This is here because the APP-mode labels are the one part of the margin
 * change that has logic rather than geometry: the degree circle is a UTF-8
 * sequence written by hand ("\xc2\xb0"), and a mistyped escape fails in the
 * quietest possible way -- utf8_cells() decodes it to some other code point,
 * label_bit() finds no glyph, and the button is simply blank on the glass with
 * nothing reporting an error. The glass is the only other place that would show
 * it, and this test exists so that it does not have to.
 *
 * The decoder below is written from the UTF-8 definition rather than borrowed
 * from display.c's, because a test that reuses the implementation it is testing
 * checks nothing. Two independent decodings agreeing on 0x00B0 is the evidence;
 * one decoding agreeing with itself is not.
 */
static int utf8_decode_first(const char *s, unsigned *cp)
{
    const unsigned char *p = (const unsigned char *)s;

    if (!p[0])
        return 0;
    if (p[0] < 0x80)
    {
        *cp = p[0];
        return 1;
    }
    if ((p[0] & 0xE0) == 0xC0)
    {
        if ((p[1] & 0xC0) != 0x80)
            return -1;
        *cp = ((unsigned)(p[0] & 0x1Fu) << 6) | (p[1] & 0x3Fu);
        return 2;
    }
    return -1;
}

static void test_margin_labels(void)
{
    static const char *const desk_strip[BTN_N] = { "Tab", "Enter", "Ctrl+Q" };
    static const char *const desk_left[COL_N] = { "F10", "Esc", "F6", "Super+D", "F11" };
    unsigned cp = 0;
    int i;

    /* The desktop labels are unchanged, so restoring them on stop needs no new
     * table and cannot drift from what shipped. */
    for (i = 0; i < BTN_N; i++)
        CHECK(strcmp(btn_label(i), desk_strip[i]) == 0);
    for (i = 0; i < COL_N; i++)
        CHECK(strcmp(left_label(i), desk_left[i]) == 0);

    /* The right column is a reserved blank slot and then four arrows. Slot 0
     * being empty is not a defect -- it is the reservation -- so the check is
     * that 1..4 carry a label and 0 does not. Written this way after the first
     * version asserted a non-empty label for every slot and failed on the
     * reserved one, which is the test being wrong rather than the code. */
    CHECK(right_label(0)[0] == '\0');
    for (i = 1; i < COL_N; i++)
        CHECK(right_label(i)[0] != '\0');

    /* APP mode: the columns carry the app's own control names, and the strip
     * still shows the degree circle placeholder. The label/action pairing is
     * checked in test_tama_ui; what is checked here is that every app label fits
     * the row a column button offers, because a label that overruns its border
     * is a rendering fault this file is the right place to catch. */
    for (i = 0; i < COL_N; i++)
    {
        CHECK(left_label_app(i) != NULL);
        CHECK(right_label_app(i) != NULL);
        /* Seven characters is the ceiling: a slot is 81-83 rows and a glyph is
         * 8 px wide stood on end, inside the corner radius. */
        CHECK(strlen(left_label_app(i)) <= 7);
        CHECK(strlen(right_label_app(i)) <= 7);
    }
    for (i = 0; i < BTN_N; i++)
    {
        const char *lab = btn_label_app(i);

        CHECK(utf8_decode_first(lab, &cp) == 2);   /* two bytes, one char */
        CHECK_EQ(cp, 0x00B0);
        CHECK(lab[2] == '\0');                    /* and nothing after it */
    }

    /* The font must actually have it, or the button paints blank and says
     * nothing. This is the check that would have caught a wrong code point. */
    {
        const uint8_t *g = font8x16_glyph(0x00B0);
        int ink = 0;

        CHECK(g != NULL);
        if (g)
        {
            for (int r = 0; r < 16; r++)
                for (int b = 0; b < 8; b++)
                    if (g[r] & (0x80 >> b))
                        ink++;
            CHECK(ink > 0);        /* a glyph that draws nothing is no glyph */
        }
    }

    /* And it must fit the bar it is centred in. Upright, a label is one cell
     * tall; the strip's usable height is its face less the corner radius at
     * each end. A degree sign taller than that would be clipped by the very
     * border the label painter draws under it. */
    CHECK(16 <= (BTN_Y1 - BTN_Y0 + 1) - 2 * BTN_RADIUS);
    CHECK(8 <= (BTN_X1 - BTN_X0 + 1) / BTN_N - BTN_SEP - 2 * BTN_RADIUS);
}

int main(void)
{
    test_geometry_matches_the_glass();
    test_slot_boundaries();
    test_panel_coordinates();
    test_every_point_of_the_glass();
    test_the_columns_mirror();
    test_margin_labels();

    return TD_TEST_RESULT();
}
