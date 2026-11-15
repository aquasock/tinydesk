#include "sdpaths.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

const char *sd_media(const char *organized, const char *flat)
{
    struct stat st;

    if (organized && stat(organized, &st) == 0)
        return organized;

    if (flat && stat(flat, &st) == 0)
        return flat;

    /* Neither exists. Name the one the caller was reaching for, because that is
     * the layout the card is supposed to have and the message should lead there
     * rather than to a path that was retired. */
    return organized ? organized : flat;
}

const char *sd_out(const char *dir, const char *name)
{
    static char buf[2][96];
    static int which;

    /* Directories are created on demand. The card may have been formatted and
     * filled by hand, and a diagnostic that silently fails because its directory
     * is missing is worse than one that makes it. Failure here is ignored on
     * purpose: the open() that follows will fail and say so. */
    if (dir && dir[0])
        (void)mkdir(dir, 0777);

    char *p = buf[which];
    which ^= 1;

    snprintf(p, sizeof(buf[0]), "%s/%s", dir ? dir : "/sd", name ? name : "");

    return p;
}
