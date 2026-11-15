/*
 * tama_ui.c - the margin button rules, with no hardware in them.
 *
 * See tama_ui.h for what this is and why the arm exists. The whole file is a
 * table and two set tests.
 */
#include "tama_ui.h"

#include <string.h>

/*
 * THE BUTTON MAP, in one table.
 *
 * It is a table rather than a switch because tests/test_tama_ui.c walks it: the
 * test asserts that every entry here has a matching label in edges.h, so a
 * button cannot be given an action without being given a name, or a name
 * without an action. That pairing is the fault this project has already paid
 * for twice -- a label and its duty drifting apart -- and a single table is
 * what makes it checkable rather than merely intended.
 */
typedef struct
{
    edge_region_t region;
    int index;
    tama_act_t act;
} ui_entry_t;

static const ui_entry_t s_map[] = {
    /* Left column, top to bottom: the things a player reaches for mid-game. */
    { EDGE_LEFT, 0, TAMA_ACT_PAUSE },
    { EDGE_LEFT, 1, TAMA_ACT_SAVE },
    { EDGE_LEFT, 2, TAMA_ACT_LOAD },
    { EDGE_LEFT, 3, TAMA_ACT_SPEED },

    /* Right column: the arm at the top where a thumb naturally rests, the
     * screenshot next to it because it is harmless, Sound directly under it,
     * and Reset at the BOTTOM -- the last place a finger sliding down the glass
     * arrives. Distance is not the guarantee (the arm is); it is the second line
     * of defence.
     *
     * RESET MOVED UP ONE SLOT to fill the gap Sound left behind when it moved,
     * at the operator's request, so the column reads Hold / Shot / Sound / Reset
     * with no hole and nothing at the bottom. The second-line-of-defence argument
     * is weaker for it -- Reset is now two slots from the arm rather than four --
     * and that is a deliberate trade for a tidier column, recorded so it is not
     * mistaken for an oversight.
     *
     * SOUND MOVED HERE FROM THE LEFT COLUMN at the operator's request, so it sits
     * under Shot rather than at the bottom of the left column. It is a MOVE and
     * not a second button: two slots driving one action would both want to show
     * the state, and the left column's last slot is now genuinely empty. */
    { EDGE_RIGHT, 1, TAMA_ACT_SHOT },
    { EDGE_RIGHT, 2, TAMA_ACT_SOUND },
    { EDGE_RIGHT, 3, TAMA_ACT_RESET },
    { EDGE_RIGHT, 4, TAMA_ACT_EXIT },
};

#define MAP_N ((int)(sizeof(s_map) / sizeof(s_map[0])))

/* The arm slot: reserved in the desktop's label table, which is exactly why it
 * was available. EDGE_RIGHT index 0. */
#define ARM_REGION EDGE_RIGHT
#define ARM_INDEX  0

void tama_ui_init(tama_ui_t *ui)
{
    if (!ui)
        return;
    memset(ui, 0, sizeof(*ui));
}

static bool point_matches(const tama_ui_point_t *p, edge_region_t region, int index)
{
    return p->region == region && p->index == index;
}

bool tama_ui_armed(const tama_ui_frame_t *f)
{
    if (!f)
        return false;

    for (int i = 0; i < f->n && i < TAMA_UI_POINTS; i++)
        if (point_matches(&f->p[i], ARM_REGION, ARM_INDEX))
            return true;

    return false;
}

static bool frame_has(const tama_ui_frame_t *f, edge_region_t region, int index)
{
    if (!f)
        return false;

    for (int i = 0; i < f->n && i < TAMA_UI_POINTS; i++)
        if (point_matches(&f->p[i], region, index))
            return true;

    return false;
}

tama_act_t tama_ui_action_of(edge_region_t region, int index)
{
    for (int i = 0; i < MAP_N; i++)
        if (s_map[i].region == region && s_map[i].index == index)
            return s_map[i].act;

    return TAMA_ACT_NONE;
}

bool tama_ui_is_gated(tama_act_t act)
{
    return act == TAMA_ACT_SAVE || act == TAMA_ACT_LOAD ||
           act == TAMA_ACT_RESET || act == TAMA_ACT_EXIT;
}

bool tama_ui_haptic_of(edge_region_t region, int index, bool armed)
{
    /* The arm is a control even though it has no action: it is held, not
     * pressed, and feeling it take is the point. */
    if (region == ARM_REGION && index == ARM_INDEX)
        return true;

    tama_act_t act = tama_ui_action_of(region, index);

    if (act == TAMA_ACT_NONE)
        return false;               /* blank slot, or bare glass */

    /* A gated button unarmed is not a press. Buzzing for it would tell the finger
     * something happened, when the interlock exists precisely so that it did not. */
    if (tama_ui_is_gated(act) && !armed)
        return false;

    return true;
}

const char *tama_ui_action_name(tama_act_t act)
{
    switch (act)
    {
    case TAMA_ACT_PAUSE: return "pause";
    case TAMA_ACT_SAVE:  return "save";
    case TAMA_ACT_LOAD:  return "load";
    case TAMA_ACT_SPEED: return "speed";
    case TAMA_ACT_SOUND: return "sound";
    case TAMA_ACT_SHOT:  return "shot";
    case TAMA_ACT_RESET: return "reset";
    case TAMA_ACT_EXIT:  return "exit";
    default:             return "none";
    }
}

int tama_ui_update(tama_ui_t *ui, const tama_ui_frame_t *now,
                   tama_act_t out[TAMA_UI_MAX_ACTIONS])
{
    int n = 0;
    bool gated_done = false;

    if (!ui || !now || !out)
        return 0;

    if (tama_ui_armed(now))
        ui->armed_frames++;

    for (int i = 0; i < MAP_N; i++)
    {
        tama_act_t act = s_map[i].act;

        /* Press edge: down now, not down on the previous report. */
        if (!frame_has(now, s_map[i].region, s_map[i].index))
            continue;
        if (ui->have_prev && frame_has(&ui->prev, s_map[i].region, s_map[i].index))
            continue;

        if (tama_ui_is_gated(act))
        {
            /* THE INTERLOCK. Not "was armed at some point" and not "armed in
             * the previous report": armed in THIS report, on the same block
             * that carries the action. Anything looser would let a chord
             * release and a stray press afterwards through. */
            if (!tama_ui_armed(now))
            {
                ui->refused++;
                continue;
            }

            /* Two gated actions in one report cannot happen with two fingers
             * and one of them on the arm, but if it ever did the outcome would
             * depend on table order, which is not something to leave implicit.
             * The first one in table order wins and the second is counted as
             * refused. */
            if (gated_done)
            {
                ui->refused++;
                continue;
            }
            gated_done = true;
        }

        if (n < TAMA_UI_MAX_ACTIONS)
            out[n++] = act;
    }

    ui->prev = *now;
    ui->have_prev = true;

    return n;
}
