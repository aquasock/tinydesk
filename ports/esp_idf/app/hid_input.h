/*
 * hid_input.h - keyboards and mice the board reads itself, from any
 * transport: USB on the ESP32-S3 (usb_hid.c) and Bluetooth LE on every
 * board (bt_hid.c). Their HID reports become the bytes a terminal sends
 * (PuTTY's keys, SGR mouse reports), which the desktop reads with the
 * board link's own input (hal_mux.c); while a mouse is attached the
 * desktop draws the pointer itself (td_wm_set_pointer).
 */
#ifndef HID_INPUT_H
#define HID_INPUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tinydesk/td_sysinfo.h"

/* Read the usb.mouse_* and usb.kbd_* settings and start key repeat. Safe to
 * call more than once. */
void hid_input_init(void);

/* Bytes from the keyboards and mice (0 if none; never blocks). */
int hid_input_read(uint8_t *buf, int cap);

/* A keyboard came or went; its boot report (modifiers, a reserved byte,
 * six key usages). name: what the Keyboard app shows ("USB keyboard"). */
void hid_kbd_attach(const char *name);
void hid_kbd_detach(void);
void hid_kbd_report(const uint8_t *r, size_t len);

/* Where a mouse puts its buttons, movement and wheel in its reports, read
 * from its report descriptor; one per mouse. */
typedef struct
{
    uint16_t pos;   /* bit offset after the report ID */
    uint8_t size;   /* bits */
    uint8_t id;     /* report ID (0: none) */
    bool sign;
} hid_field_t;

typedef struct
{
    bool ok, ids;   /* fields found; reports start with an ID */
    hid_field_t button[3], x, y, wheel;
} hid_mouse_layout_t;

/* Fill layout from a report descriptor; false when it has no X and Y (then
 * the mouse's reports are read as boot reports: buttons, X, Y). */
bool hid_mouse_parse(hid_mouse_layout_t *layout, const uint8_t *desc, size_t len);

/* A mouse came or went (name: "USB mouse, with its wheel"); one of its
 * input reports, with its report ID first when the descriptor has IDs.
 * layout NULL or not ok: a boot report. */
void hid_mouse_attach(const char *name);
void hid_mouse_detach(void);
void hid_mouse_report(const hid_mouse_layout_t *layout, const uint8_t *r, size_t len);

/* ------------------------------------------------------------- gamepads */

/*
 * A GAMEPAD IS NOT A MOUSE, and the difference is not the axes.
 *
 * A gamepad's report descriptor declares Generic Desktop X and Y for its left
 * stick, which is exactly what hid_mouse_parse() looks for -- so an Xbox
 * controller connects, delivers reports, and is claimed as a mouse. On this
 * board that showed up as the desktop cursor jittering whenever the stick moved.
 *
 * What distinguishes them is the TOP-LEVEL COLLECTION: a gamepad declares
 * Generic Desktop / Game Pad (0x05), a mouse declares / Mouse (0x02) and a
 * keyboard / Keyboard (0x06). Measured on an Xbox Wireless Controller over BLE
 * (2026-11-12): its collection is 0x05, and its report is ID 1, 16 bytes, with
 * the hat switch at bit 96 and fifteen buttons at bit 104.
 *
 * THE BUTTON BITS ARE READ FROM THE DESCRIPTOR, not assumed: field offsets are
 * accumulated exactly as the descriptor lays them out, so a controller that puts
 * its buttons somewhere else still works, and the bit-to-button numbering stays
 * whatever that controller declared.
 */
typedef struct
{
    bool ok, ids;
    uint8_t id;            /* report ID (0: none)                      */
    hid_field_t hat;       /* Generic Desktop Hat switch (0x39)        */
    hid_field_t x, y;      /* left stick, for menus and pointer control */
    hid_field_t button;    /* first button; `count` buttons from here  */
    uint8_t count;
} hid_gamepad_layout_t;

/* Which buttons are down. Bits 0..14 are the descriptor's buttons 1..15. */
typedef uint16_t hid_gamepad_mask_t;

/* Fill `layout` from a report descriptor. False when the descriptor is not a
 * gamepad -- which is the test a caller uses to stop one being taken for a
 * mouse. */
bool hid_gamepad_parse(hid_gamepad_layout_t *layout, const uint8_t *desc, size_t len);

/* One input report, with its report ID first when the descriptor has IDs.
 * Returns the button mask; 0 for a report that is not the gamepad's own. */
hid_gamepad_mask_t hid_gamepad_report(const hid_gamepad_layout_t *layout,
                                      const uint8_t *r, size_t len);

/* The most recent mask, and the hat's direction (0..7 clockwise from up, or
 * 0xFF for centred), for a caller that polls rather than being called back.
 * Both are plain integers: they are written from the Bluetooth task and read
 * from an emulator task, and an aligned int is atomic on this core. */
hid_gamepad_mask_t hid_gamepad_state(void);
uint8_t hid_gamepad_hat(void);

/*
 * WHAT THE LAST REPORT ACTUALLY CONTAINED, for a fault that every other view
 * hides.
 *
 * Measured on the watch: the d-pad decodes correctly while every face button
 * reads zero -- `gamepad=0x0000` on a mask taken straight from the decoder while
 * the hat cycles 0,2,4,6. The parser passes 27 checks on the host against the
 * captured descriptor, so the descriptor and the decode are not in question; what
 * is left is the report itself, and its LENGTH is the prime suspect. The hat sits
 * at bit 96 and the buttons at bit 104, which with a prepended report ID means
 * byte 13 and byte 14: a report one byte shorter than the descriptor expects
 * would decode the hat perfectly and every button as zero, which is exactly the
 * symptom.
 *
 * So this records the length, the report ID, the parsed field positions and the
 * first bytes verbatim. Guessing between a short report, a different controller
 * revision and a field-id mismatch is what the file is for.
 */
typedef struct
{
    uint16_t len;        /* bytes handed to the decoder, report ID included */
    uint8_t  id;         /* r[0] as received */
    uint8_t  want_id;    /* the layout's own report ID */
    uint8_t  count;      /* buttons the descriptor declared */
    uint16_t btn_pos;    /* bit offset of the first button */
    uint8_t  btn_size;   /* bits per button */
    uint16_t hat_pos;
    uint8_t  hat_size;
    bool     ids;        /* the layout expects a report ID */
    uint8_t  bytes[16];  /* the report as received */
    uint32_t reports;    /* how many reports have been decoded */
} hid_gamepad_last_t;

hid_gamepad_last_t hid_gamepad_last(void);

/* The Mouse and Keyboard apps' settings (td_sysinfo_t.mouse, .keyboard), or
 * NULL before hid_input_init(). */
const td_mouse_ops_t *hid_input_mouse_ops(void);
const td_keyboard_ops_t *hid_input_keyboard_ops(void);

#endif
