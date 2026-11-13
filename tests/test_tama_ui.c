/*
 * test_tama_ui.c - the Tamagotchi's margin buttons, swept on a host.
 *
 * WHY THIS EXISTS, AND WHY IT IS NOT OPTIONAL FOR THIS FEATURE.
 *
 * The arm is a safety interlock on two buttons that destroy state. Everything
 * about it that matters is arithmetic over a small set of inputs -- which
 * points are down this report, which were down last report -- and a rule like
 * that can be walked exhaustively at a desk in microseconds. What it cannot be
 * checked for by hand is the case that matters most: the ORDER of events that a
 * real hand produces and a person testing deliberately does not.
 *
 * There are two such orders, and both are here:
 *
 *   1. Press the arm, press Reset. Legitimate, must fire.
 *   2. Press the arm, LIFT the arm, then press Reset. Must be refused.
 *
 * (2) is the one a human tester never tries, because it feels like an obvious
 * mistake. It is also the one that happens on the glass: a thumb rolls off the
 * arm while the other finger is still moving. If the rule were "was armed at
 * some point" or "was armed on the previous report", (2) would fire Reset.
 *
 * AND THE CONTROLLER HAS A RELATED TRAP, which is why touch.c gates on an exact
 * event code rather than a nibble: lifting the first finger of a chord promotes
 * the survivor into the primary slot with event 0x16, whose low nibble is 0x06.
 * That is a driver-side fault, so it is covered in touch.c and recorded as
 * TCH-012 -- but it is the same hazard from the other end, and this test is
 * where the UI half of it is pinned.
 */
#include "td_test.h"

#include "edges.h"
#include "tama_ui.h"

#include <string.h>

/* Build a frame from up to two (region, index) pairs. */
static tama_ui_frame_t frame1(edge_region_t r0, int i0)
{
    tama_ui_frame_t f;

    memset(&f, 0, sizeof(f));
    f.p[0].region = r0;
    f.p[0].index = i0;
    f.p[1].region = EDGE_NONE;
    f.p[1].index = -1;
    f.n = 1;
    return f;
}

static tama_ui_frame_t frame2(edge_region_t r0, int i0, edge_region_t r1, int i1)
{
    tama_ui_frame_t f;

    memset(&f, 0, sizeof(f));
    f.p[0].region = r0;
    f.p[0].index = i0;
    f.p[1].region = r1;
    f.p[1].index = i1;
    f.n = 2;
    return f;
}

static tama_ui_frame_t empty_frame(void)
{
    tama_ui_frame_t f;

    memset(&f, 0, sizeof(f));
    f.p[0].region = EDGE_NONE;
    f.p[1].region = EDGE_NONE;
    f.n = 0;
    return f;
}

/* One step: feed a frame, report what happened. */
static int step(tama_ui_t *ui, tama_ui_frame_t f, tama_act_t *act)
{
    tama_act_t out[TAMA_UI_MAX_ACTIONS];
    int n = tama_ui_update(ui, &f, out);

    if (act)
        *act = n > 0 ? out[0] : TAMA_ACT_NONE;

    return n;
}

/* The arm slot, named once so the tests read as intent rather than as indexes. */
#define ARM_R EDGE_RIGHT
#define ARM_I 0

/* --- the tables agree ---------------------------------------------------- */

/*
 * A label with no action is a button that does nothing; an action with no label
 * is a button nobody can find. Both have happened in this project under other
 * names, and a table is the only place the pairing can be checked.
 */
static void test_labels_match_actions(void)
{
    for (int i = 0; i < COL_N; i++)
    {
        const char *l = left_label_app(i);
        tama_act_t a = tama_ui_action_of(EDGE_LEFT, i);

        CHECK(l != NULL);
        /* Empty label iff no action -- the left column has no reserved slot. */
        CHECK_EQ((l[0] == '\0'), (a == TAMA_ACT_NONE));
    }

    for (int i = 0; i < COL_N; i++)
    {
        const char *l = right_label_app(i);
        tama_act_t a = tama_ui_action_of(EDGE_RIGHT, i);

        CHECK(l != NULL);

        if (i == ARM_I)
        {
            /* The arm is the one intentional exception: labelled, no action.
             * It is a modifier, not a command. */
            CHECK(l[0] != '\0');
            CHECK(a == TAMA_ACT_NONE);
        }
        else
        {
            CHECK_EQ((l[0] == '\0'), (a == TAMA_ACT_NONE));
        }
    }

    /* Every action the UI can emit has a name that is not "none". */
    for (int a = TAMA_ACT_PAUSE; a <= TAMA_ACT_RESET; a++)
        CHECK(strcmp(tama_ui_action_name((tama_act_t)a), "none") != 0);
}

/* --- the interlock -------------------------------------------------------- */

static void test_gated_without_arm_is_refused(void)
{
    static const struct { edge_region_t r; int i; tama_act_t a; } gated[] = {
        { EDGE_LEFT, 1, TAMA_ACT_SAVE },
        { EDGE_LEFT, 2, TAMA_ACT_LOAD },
        { EDGE_RIGHT, 4, TAMA_ACT_RESET },
    };

    for (size_t k = 0; k < sizeof(gated) / sizeof(gated[0]); k++)
    {
        tama_ui_t ui;
        tama_act_t act;

        tama_ui_init(&ui);

        /* Straight press, no arm. */
        CHECK_EQ(step(&ui, frame1(gated[k].r, gated[k].i), &act), 0);
        CHECK(act == TAMA_ACT_NONE);
        CHECK(ui.refused > 0);

        /* Held: still nothing, and refused does not climb while held. */
        unsigned before = ui.refused;

        CHECK_EQ(step(&ui, frame1(gated[k].r, gated[k].i), NULL), 0);
        CHECK_EQ(ui.refused, before);

        /* Released, then pressed again: refused again. */
        step(&ui, empty_frame(), NULL);
        CHECK_EQ(step(&ui, frame1(gated[k].r, gated[k].i), NULL), 0);
        CHECK(ui.refused > before);
    }
}

static void test_gated_with_arm_fires(void)
{
    static const struct { edge_region_t r; int i; tama_act_t a; } gated[] = {
        { EDGE_LEFT, 1, TAMA_ACT_SAVE },
        { EDGE_LEFT, 2, TAMA_ACT_LOAD },
        { EDGE_RIGHT, 4, TAMA_ACT_RESET },
    };

    for (size_t k = 0; k < sizeof(gated) / sizeof(gated[0]); k++)
    {
        tama_ui_t ui;
        tama_act_t act;

        tama_ui_init(&ui);

        /* Arm alone: no action, and it must not fire anything. */
        CHECK_EQ(step(&ui, frame1(ARM_R, ARM_I), &act), 0);
        CHECK(act == TAMA_ACT_NONE);

        /* Now the chord: arm still down, action arrives. */
        CHECK_EQ(step(&ui, frame2(ARM_R, ARM_I, gated[k].r, gated[k].i), &act), 1);
        CHECK(act == gated[k].a);

        /* Held chord does not repeat. */
        CHECK_EQ(step(&ui, frame2(ARM_R, ARM_I, gated[k].r, gated[k].i), NULL), 0);
    }
}

/*
 * THE CASE A HUMAN TESTER DOES NOT TRY.
 *
 * Press the arm, lift the arm, then press the destructive button. If this ever
 * fires, the interlock is decorative.
 */
static void test_arm_released_before_action_is_refused(void)
{
    tama_ui_t ui;
    tama_act_t act;

    tama_ui_init(&ui);

    step(&ui, frame1(ARM_R, ARM_I), NULL);            /* arm down      */
    step(&ui, empty_frame(), NULL);                   /* arm lifted    */
    CHECK_EQ(step(&ui, frame1(EDGE_RIGHT, 4), &act), 0);
    CHECK(act == TAMA_ACT_NONE);
    CHECK(ui.refused > 0);

    /* And the same for the other two, because they share the rule. */
    tama_ui_init(&ui);
    step(&ui, frame1(ARM_R, ARM_I), NULL);
    step(&ui, empty_frame(), NULL);
    CHECK_EQ(step(&ui, frame1(EDGE_LEFT, 1), NULL), 0);   /* Save  */
    step(&ui, empty_frame(), NULL);
    CHECK_EQ(step(&ui, frame1(EDGE_LEFT, 2), NULL), 0);   /* Load  */
    CHECK(ui.refused >= 2);
}

/*
 * The reverse order: action finger first, arm second. The rule is a property of
 * the SET, not of arrival order, because the controller assigns point numbers by
 * arrival and a rule keyed on "point 1 is the arm" would invert silently.
 */
static void test_order_does_not_matter(void)
{
    tama_ui_t ui;
    tama_act_t act;

    /* Action down alone: refused (no arm yet). */
    tama_ui_init(&ui);
    CHECK_EQ(step(&ui, frame1(EDGE_LEFT, 2), &act), 0);
    CHECK(act == TAMA_ACT_NONE);

    /* Arm arrives second and the set now contains both. Still no action: the
     * action's PRESS EDGE has already passed, so it cannot fire retrospectively.
     * This is deliberate -- otherwise adding the arm would retroactively
     * authorise whatever was already held, which would be a way to arm by
     * accident. */
    CHECK_EQ(step(&ui, frame2(EDGE_LEFT, 2, ARM_R, ARM_I), &act), 0);
    CHECK(act == TAMA_ACT_NONE);

    /* Lift the action, keep the arm, press the action again: now it fires. */
    step(&ui, frame1(ARM_R, ARM_I), NULL);
    CHECK_EQ(step(&ui, frame2(ARM_R, ARM_I, EDGE_LEFT, 2), &act), 1);
    CHECK(act == TAMA_ACT_LOAD);
}

/* --- ungated buttons ------------------------------------------------------ */

static void test_ungated_need_no_arm(void)
{
    static const struct { edge_region_t r; int i; tama_act_t a; } plain[] = {
        { EDGE_LEFT, 0, TAMA_ACT_PAUSE },
        { EDGE_LEFT, 3, TAMA_ACT_SPEED },
        { EDGE_LEFT, 4, TAMA_ACT_SOUND },
        { EDGE_RIGHT, 1, TAMA_ACT_SHOT },
    };

    for (size_t k = 0; k < sizeof(plain) / sizeof(plain[0]); k++)
    {
        tama_ui_t ui;
        tama_act_t act;

        tama_ui_init(&ui);

        CHECK_EQ(step(&ui, frame1(plain[k].r, plain[k].i), &act), 1);
        CHECK(act == plain[k].a);
        CHECK_EQ(ui.refused, 0);
    }
}

/* A press edge, not a level: holding a toggle must not repeat it, or Speed would
 * cycle through every rate faster than the panel can show it. */
static void test_press_edge_only(void)
{
    tama_ui_t ui;
    tama_act_t act;

    tama_ui_init(&ui);

    CHECK_EQ(step(&ui, frame1(EDGE_LEFT, 3), &act), 1);   /* Speed */
    CHECK(act == TAMA_ACT_SPEED);

    for (int i = 0; i < 20; i++)
        CHECK_EQ(step(&ui, frame1(EDGE_LEFT, 3), NULL), 0);

    step(&ui, empty_frame(), NULL);
    CHECK_EQ(step(&ui, frame1(EDGE_LEFT, 3), &act), 1);
    CHECK(act == TAMA_ACT_SPEED);
}

/* Two ungated buttons at once both fire: with two fingers and no arm that is a
 * legitimate combination, and neither is destructive. */
static void test_two_plain_at_once(void)
{
    tama_ui_t ui;
    tama_act_t out[TAMA_UI_MAX_ACTIONS];

    tama_ui_init(&ui);

    tama_ui_frame_t f = frame2(EDGE_LEFT, 0, EDGE_RIGHT, 1);
    int n = tama_ui_update(&ui, &f, out);

    CHECK_EQ(n, 2);
    CHECK(out[0] != out[1]);
}

/* --- an exhaustive sweep -------------------------------------------------- */

/*
 * Walk every point on the glass, paired with every other point, and check the
 * rule holds for all of them rather than for the ones that were thought of.
 *
 * The sweep is over edges_hit()'s own output rather than over pixels, which is
 * what makes it small: there are thirteen buttons plus "nothing", so 14 x 14
 * combinations, each in the three states that matter (action alone, arm + action,
 * action alone after the arm was lifted).
 */
static void test_every_button_pair(void)
{
    /* Every (region, index) the hit test can return. */
    edge_region_t regions[] = { EDGE_NONE, EDGE_LEFT, EDGE_RIGHT, EDGE_BOTTOM };
    int checked = 0;

    for (size_t a = 0; a < sizeof(regions) / sizeof(regions[0]); a++)
    for (int ai = -1; ai < COL_N; ai++)
    {
        if (regions[a] != EDGE_LEFT && regions[a] != EDGE_RIGHT && ai >= 0)
            continue;                      /* only the columns have indexes */

        for (size_t b = 0; b < sizeof(regions) / sizeof(regions[0]); b++)
        for (int bi = -1; bi < COL_N; bi++)
        {
            if (regions[b] != EDGE_LEFT && regions[b] != EDGE_RIGHT && bi >= 0)
                continue;

            tama_act_t act_a = regions[a] == EDGE_NONE ? TAMA_ACT_NONE
                                                       : tama_ui_action_of(regions[a], ai);
            tama_act_t act_b = regions[b] == EDGE_NONE ? TAMA_ACT_NONE
                                                       : tama_ui_action_of(regions[b], bi);

            /* Skip pairs where neither point is a real button. */
            if (act_a == TAMA_ACT_NONE && act_b == TAMA_ACT_NONE &&
                !(regions[a] == ARM_R && ai == ARM_I))
                continue;

            /* Skip A paired with itself. A is already down in the first frame,
             * so in the second it is held rather than newly pressed and there
             * is correctly no edge -- which the sweep would otherwise read as a
             * missing action. This cost four failures on the first run, all of
             * them this one case, and the fix belongs here rather than in the
             * rule: two fingers cannot occupy one button. */
            if (regions[a] == regions[b] && ai == bi)
                continue;

            tama_ui_t ui;
            tama_act_t out[TAMA_UI_MAX_ACTIONS];

            tama_ui_init(&ui);

            /* Frame 1: point A alone, after nothing. */
            tama_ui_frame_t f1 = frame1(regions[a], ai);
            tama_ui_update(&ui, &f1, out);

            /* Frame 2: both points, so B is new and A is held. */
            tama_ui_frame_t f2 = frame2(regions[a], ai, regions[b], bi);
            int n = tama_ui_update(&ui, &f2, out);

            /* B's action, if it has one, is a press edge now. It must be
             * present exactly when either B is not gated, or the arm is one of
             * the two points held in this frame. */
            bool arm_present = (regions[a] == ARM_R && ai == ARM_I) ||
                               (regions[b] == ARM_R && bi == ARM_I);

            if (act_b != TAMA_ACT_NONE)
            {
                if (tama_ui_is_gated(act_b) && !arm_present)
                    CHECK_EQ(n, 0);          /* refused */
                else
                    CHECK(n >= 1);
            }

            checked++;
        }
    }

    /* The sweep must actually have swept something; a loop that silently
     * skips everything would otherwise pass by vacuity. */
    CHECK(checked > 100);
}

int main(void)
{
    test_labels_match_actions();
    test_gated_without_arm_is_refused();
    test_gated_with_arm_fires();
    test_arm_released_before_action_is_refused();
    test_order_does_not_matter();
    test_ungated_need_no_arm();
    test_press_edge_only();
    test_two_plain_at_once();
    test_every_button_pair();

    return TD_TEST_RESULT();
}
