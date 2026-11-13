/*
 * unzip.h - minimal minizip interface, for building PokeMini without zlib
 *
 * PokeMini can treat a .min file as a ZIP archive. That support needs minizip,
 * zlib and bzip2; none of them is wanted on a 240 MHz microcontroller whose
 * flash and RAM are the scarcest resources on the board, and none is needed to
 * run a bare cartridge image.
 *
 * `minizip_stubs.c` provides failing implementations of every function below,
 * so the loader compiles, links, and reports an unreadable archive — which is
 * the correct answer on a build without ZIP support.
 *
 * This header declares only the subset PokeMini.c actually uses, taken from
 * minizip's public interface. It is not a general replacement: if ZIP support
 * is ever wanted, link the real minizip and delete both this file and the
 * stubs.
 */
#ifndef POKEMINI_MINIZIP_SUBSET_H
#define POKEMINI_MINIZIP_SUBSET_H

#include <stdint.h>

#define UNZ_OK      (0)
#define UNZ_END_OF_LIST_OF_FILE (-100)

typedef void *unzFile;

typedef struct {
    uint64_t number_entry;      /* total number of entries in the central dir */
} unz_global_info64;

typedef struct {
    uint16_t version;
    uint16_t version_needed;
    uint16_t flag;
    uint16_t compression_method;
    uint32_t dos_date;
    uint32_t crc32;
    uint64_t compressed_size;
    uint64_t uncompressed_size;
    uint16_t size_filename;
    uint16_t size_file_extra;
    uint16_t size_file_comment;
    uint16_t disk_num_start;
    uint16_t internal_fa;
    uint32_t external_fa;
} unz_file_info64;

unzFile unzOpen(const char *path);
int     unzClose(unzFile file);
int     unzGetGlobalInfo64(unzFile file, unz_global_info64 *info);
int     unzGetCurrentFileInfo64(unzFile file, unz_file_info64 *info,
                                char *filename, unsigned long filename_len,
                                void *extra, unsigned long extra_len,
                                char *comment, unsigned long comment_len);
int     unzGoToFirstFile(unzFile file);
int     unzGoToNextFile(unzFile file);
int     unzLocateFile(unzFile file, const char *filename, int case_sensitive);
int     unzOpenCurrentFile(unzFile file);
int     unzReadCurrentFile(unzFile file, void *buf, unsigned len);
int     unzCloseCurrentFile(unzFile file);

#endif
