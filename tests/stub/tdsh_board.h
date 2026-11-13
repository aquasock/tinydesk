#pragma once
static inline int tdsh_board_set(const char *k, const char *v) { (void)k; (void)v; return 0; }
static inline const char *tdsh_board_str(const char *k, const char *d) { (void)k; return d; }
static inline int tdsh_board_int(const char *k, int d) { (void)k; return d; }
