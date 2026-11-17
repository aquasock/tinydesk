/*
 * memdiag.h - `mem`, the internal-RAM accounting command (memdiag.c).
 */
#ifndef MEMDIAG_H
#define MEMDIAG_H

#include <stdbool.h>

/* Register the command with the shell; false when that failed. */
bool memdiag_register_command(void);

#endif
