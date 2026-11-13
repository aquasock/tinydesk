#pragma once
/* Stubs for the host test. The gamepad parser touches none of these; they exist
 * because hid_input.c's mouse path does and the file is compiled whole. */
static inline void td_wm_screen_size(int *c, int *r) { *c = 40; *r = 26; }
static inline int td_wm_mouse_pos(int *x, int *y) { (void)x; (void)y; return 0; }
static inline void td_wm_set_pointer(int on) { (void)on; }
