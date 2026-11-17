/*
 * klio.h - the BHI260AP's self-learning gesture engine (klio.c).
 */
#ifndef KLIO_H
#define KLIO_H

#include <stdbool.h>

/* Register the `klio` diagnostic command with the shell. */
bool klio_register_command(void);

#endif
