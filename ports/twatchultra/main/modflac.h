/* modflac.h - `modflac`: play a FLAC through the decoder in the `modules`
 * partition. See modflac.c for what it proves and how it differs from `music`. */
#ifndef TWATCH_MODFLAC_H
#define TWATCH_MODFLAC_H

#include <stdbool.h>

bool modflac_register_command(void);

#endif
