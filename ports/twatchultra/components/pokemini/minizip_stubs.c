/*
 * minizip_stubs.c - stubs for PokeMini's optional ZIP loader
 *
 * PokeMini can open a .min file that is really a ZIP archive, which pulls in
 * minizip and zlib. Neither is needed to run a bare cartridge image, and neither
 * is wanted on the target, where flash and RAM are the scarcest things there
 * are.
 *
 * PokeMini.c calls these unconditionally when the loader is compiled in, so
 * without them the link fails even though the code path is never taken. Each
 * stub fails the way an unreadable archive fails, which is what a caller should
 * see on a build without ZIP support.
 *
 * If zipped carts are ever wanted, link the real minizip instead of this file.
 * Nothing else in the core depends on it.
 */
#include <stddef.h>
#include "unzip.h"

unzFile unzOpen(const char *path)
{
    (void)path;
    return NULL;
}

int unzGetGlobalInfo64(unzFile file, unz_global_info64 *info)
{
    (void)file;
    (void)info;
    return -1;
}

int unzGetCurrentFileInfo64(unzFile file, unz_file_info64 *info, char *name,
                            unsigned long name_len, void *extra,
                            unsigned long extra_len, char *comment,
                            unsigned long comment_len)
{
    (void)file; (void)info; (void)name; (void)name_len;
    (void)extra; (void)extra_len; (void)comment; (void)comment_len;
    return -1;
}

int unzGoToFirstFile(unzFile file)
{
    (void)file;
    return -1;
}

int unzGoToNextFile(unzFile file)
{
    (void)file;
    return -1;
}

int unzLocateFile(unzFile file, const char *name, int case_sensitive)
{
    (void)file; (void)name; (void)case_sensitive;
    return -1;
}

int unzOpenCurrentFile(unzFile file)
{
    (void)file;
    return -1;
}

int unzReadCurrentFile(unzFile file, void *buf, unsigned len)
{
    (void)file; (void)buf; (void)len;
    return 0;
}

int unzCloseCurrentFile(unzFile file)
{
    (void)file;
    return 0;
}

int unzClose(unzFile file)
{
    (void)file;
    return 0;
}
