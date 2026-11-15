#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef RETRO_GO
#include <rg_system.h>
#define LOG_PRINTF(level, x...) rg_system_log(RG_LOG_PRINTF, NULL, x)
#else
#define LOG_PRINTF(level, x...) printf(x)
/*
 * IRAM_ATTR IS ALREADY DEFINED BY ESP-IDF, and redefining it is a -Werror
 * failure rather than a harmless duplicate. Left alone when it exists: IDF's
 * version marks a function for internal RAM, which gnuboy's use of the same name
 * wants anyway, so the two agree and the ESP-IDF definition is the better one.
 *
 * THIS IS THE ONLY EDIT TO THE VENDORED SOURCES, and it is an #ifdef around a
 * definition rather than a change of behaviour: on a non-IDF build gnuboy.h still
 * supplies its own empty macro, so the core compiles unchanged anywhere else.
 */
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#endif

#define MESSAGE_ERROR(x, ...) LOG_PRINTF(1, "!! %s: " x, __func__, ## __VA_ARGS__)
#define MESSAGE_WARN(x, ...)  LOG_PRINTF(2, "** %s: " x, __func__, ## __VA_ARGS__)
#define MESSAGE_INFO(x, ...)  LOG_PRINTF(3, " * %s: " x, __func__, ## __VA_ARGS__)
// #define MESSAGE_DEBUG(x, ...) LOG_PRINTF(4, ">> %s: " x, __func__, ## __VA_ARGS__)
#define MESSAGE_DEBUG(x, ...) {}

#define GB_WIDTH (160)
#define GB_HEIGHT (144)

typedef uint8_t byte;

typedef enum
{
	GB_HW_DMG,
	GB_HW_CGB,
	GB_HW_SGB,
} gb_hwtype_t;

typedef enum
{
	GB_PAD_RIGHT = 0x01,
	GB_PAD_LEFT = 0x02,
	GB_PAD_UP = 0x04,
	GB_PAD_DOWN = 0x08,
	GB_PAD_A = 0x10,
	GB_PAD_B = 0x20,
	GB_PAD_SELECT = 0x40,
	GB_PAD_START = 0x80,
} gb_padbtn_t;

typedef enum
{
	GB_PALETTE_0,
	GB_PALETTE_1,
	GB_PALETTE_2,
	GB_PALETTE_3,
	GB_PALETTE_4,
	GB_PALETTE_5,
	GB_PALETTE_6,
	GB_PALETTE_7,
	GB_PALETTE_8,
	GB_PALETTE_9,
	GB_PALETTE_10,
	GB_PALETTE_11,
	GB_PALETTE_12,
	GB_PALETTE_13,
	GB_PALETTE_14,
	GB_PALETTE_15,
	GB_PALETTE_16,
	GB_PALETTE_17,
	GB_PALETTE_18,
	GB_PALETTE_19,
	GB_PALETTE_20,
	GB_PALETTE_21,
	GB_PALETTE_22,
	GB_PALETTE_23,
	GB_PALETTE_24,
	GB_PALETTE_25,
	GB_PALETTE_26,
	GB_PALETTE_27,
	GB_PALETTE_28,
	GB_PALETTE_29,
	GB_PALETTE_30,
	GB_PALETTE_31,
	GB_PALETTE_DMG,
	GB_PALETTE_MGB0,
	GB_PALETTE_MGB1,
	GB_PALETTE_CGB,
	GB_PALETTE_SGB,
	GB_PALETTE_COUNT,
} gb_palette_t;

typedef enum
{
	GB_PIXEL_PALETTED,
	GB_PIXEL_565_LE,
	GB_PIXEL_565_BE,
} gb_video_fmt_t;

typedef enum
{
	GB_AUDIO_STEREO_S16,
	GB_AUDIO_MONO_S16,
} gb_audio_fmt_t;

typedef void (gb_video_cb_t)(void *buffer);
typedef void (gb_audio_cb_t)(void *buffer, size_t length);

int  gnuboy_init(int samplerate, gb_audio_fmt_t audio_fmt, gb_video_fmt_t video_fmt, gb_video_cb_t *video_callback, gb_audio_cb_t *audio_callback);
int  gnuboy_load_bios(const byte *data, size_t size);
int  gnuboy_load_bios_file(const char *file);
void gnuboy_free_bios(void);
int  gnuboy_load_rom(const byte *data, size_t size);
int  gnuboy_load_rom_file(const char *file);
void gnuboy_free_rom(void);
void gnuboy_reset(bool hard);
void gnuboy_run(bool draw);
bool gnuboy_sram_dirty(void);
void gnuboy_load_bank(int);
void gnuboy_set_pad(int);

void gnuboy_set_framebuffer(void *buffer);
void gnuboy_set_soundbuffer(void *buffer, size_t length);

void gnuboy_get_time(int *day, int *hour, int *minute, int *second);
void gnuboy_set_time(int day, int hour, int minute, int second);
int  gnuboy_get_hwtype(void);
void gnuboy_set_hwtype(gb_hwtype_t type);
int  gnuboy_get_palette(void);
void gnuboy_set_palette(gb_palette_t pal);

int gnuboy_load_sram(const char *file);
int gnuboy_save_sram(const char *file, bool quick_save);

/*
 * Copy the battery-backed RAM -- and the MBC3 time block when the cartridge has
 * one -- into `dst`, in exactly the byte order gnuboy_save_sram() writes to a
 * file. Returns the number of bytes written, or 0 when the cartridge has no
 * battery or `cap` is too small.
 *
 * This exists so a host can take the snapshot on its own thread of execution and
 * do the file I/O somewhere else. That matters on hardware where a card write
 * costs longer than a video frame: calling gnuboy_save_sram() directly from the
 * emulation loop stalls the guest for the whole transfer, while a memcpy of the
 * same bytes is two orders of magnitude cheaper. The format is not reimplemented
 * here -- it is copied from gnuboy_save_sram(), which is why the two cannot
 * drift.
 *
 * TAKING THE SNAPSHOT CLEARS THE DIRTY FLAG, because ownership of those bytes has
 * moved to the caller. That has to happen here, at the moment of the copy and in
 * the thread that owns the banks, rather than after the write completes: a write
 * takes a tenth of a second on this board, and anything the guest stored during
 * that window must set the flag again so the NEXT save picks it up. Clearing
 * afterwards would silently drop whatever landed in between.
 *
 * If the write then fails, the caller must call gnuboy_sram_mark_dirty() so the
 * save is retried instead of being forgotten.
 */
size_t gnuboy_sram_snapshot(byte *dst, size_t cap);

/* Mark the cartridge's save RAM as unsaved, so the next save writes it. Used
 * after a failed write, since gnuboy_sram_snapshot() cleared the flag. */
void gnuboy_sram_mark_dirty(void);
int gnuboy_load_state(const char *file);
int gnuboy_save_state(const char *file);
