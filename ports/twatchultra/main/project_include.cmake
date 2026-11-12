# Build-wide compile definitions for this port.
#
# This file exists because of where the definition has to land, not what it
# does. `TD_SIZE_POLL_MS` is consulted in `src/td.c`, which belongs to the
# `tinydesk` component and is shared with every other port, so it must not be
# changed there. A definition set on this port's own component
# (`main/CMakeLists.txt`) reaches `main/` only, and `add_compile_definitions`
# in the top-level CMakeLists comes too late: ESP-IDF has already registered
# and configured the components by the time `project()` returns, so only the
# components added afterwards (the port's own `main`) inherit that directory
# scope. The `tinydesk` component is not one of them.
#
# ESP-IDF includes every component's `project_include.cmake` before that
# happens (tools/cmake/build.cmake, __build_process_project_includes), and a
# property set on the build there is applied to all components. So the
# definition has to be set from a project include, and this is the one
# belonging to the port's `main` component.
#
# TD_SIZE_POLL_MS=0 stops the desktop asking the terminal for its size once a
# second. The question exists because a serial terminal cannot report a
# resize; this panel's grid is fixed at build time, so there is nothing to
# discover. It is not free either: the query parks the cursor in the
# bottom-right corner to measure the terminal, and the cells it touches are
# marked changed, so the frame's dirty span reached from the top of the screen
# to the bottom and repainted the whole panel once a second -- on the glass
# and on the USB mirror.
idf_build_set_property(COMPILE_DEFINITIONS "TD_SIZE_POLL_MS=0" APPEND)

# Frame pacing off for the same kind of reason: the mechanism assumes a remote
# terminal on a serial line. After each frame the desktop sends ESC[5n and
# waits for a status reply before the next frame, so a slow terminal (PuTTY)
# never has two frames queued. Here the terminal is this board -- display.c's
# td_vterm, which answers ESC[5n with ESC[0n (src/vterm.c), and that reply is
# injected back as desktop input, so pacing is answered every time and never
# turns itself off. Every frame therefore pays a query, a flush and a reply
# for a round trip that is measured in microseconds and controls nothing.
# TD_ACK_TIMEOUT_MS=0 is the documented way to turn it off (td_config.h).
idf_build_set_property(COMPILE_DEFINITIONS "TD_ACK_TIMEOUT_MS=0" APPEND)

# Apps open maximised (frames and title bars kept; dialogs and tools are not
# affected -- src/wm.c, td_win_create). On a 40x26 panel an app that asks for a
# desktop-sized window has most of its content below row 26, which is off the
# glass: the Bluetooth window's own WIN_H is 34. Maximised, an app gets the
# whole desk area and its layout code needs no change at all.
idf_build_set_property(COMPILE_DEFINITIONS "TD_OPEN_MAXIMIZED=1" APPEND)

# No On-screen keyboard in the Start menu. A 40-column panel cannot hold the
# keys at a size a finger can hit, so the app is not usable here, and this board
# has a Bluetooth keyboard. The keyboard itself is still built and still opens
# from Sign in and the kiosk way out -- this removes the icon, not the feature.
idf_build_set_property(COMPILE_DEFINITIONS "TD_HIDE_OSK_APP=1" APPEND)

# --- WOLFSSH_STOREHANDLE: stops an aborted SFTP session leaking file handles
#
# Measured on this board on 2026-11-12: five killed transfers and the SD card
# refuses every open -- from the shell too, `echo hi > /sd/x` returning 1 with
# `vfs_fat: open: no free file descriptors`. Clean transfers do not leak: eight
# 1 KiB puts in one session and a 233 KB transfer both leave the card writable.
# So the leak is per abandoned session, and it is permanent until a reboot.
#
# THE MECHANISM, traced rather than guessed. This port uses wolfSSH's stdio
# path, so `WFOPEN` maps to `wfopen` (ports/esp_idf/components/wolfssh_local/
# include/wolfssh/port.h) and files land in ESP-IDF's VFS, whose FAT mount is
# created with `max_files = 5` (tdsh_sdcard.c, SD_MAX_FILES). wolfSSH closes
# those descriptors in two places: the SFTP CLOSE request, and
# `SFTP_FreeHandles()` from `wolfSSH_SFTP_free()`. The second is compiled out
# unless `WOLFSSH_STOREHANDLE` is defined, and `settings.h` only defines it
# automatically for `WOLFSSH_FATFS` -- which is the *other* I/O path, the one
# that talks to FatFs directly through a private descriptor pool. This port
# never takes that path, so `WOLFSSH_STOREHANDLE` was off, the handle list was
# never populated, and teardown had nothing to close.
#
# The consequence is that the leak only appears on an ABORTED session: a normal
# one ends with the client's own CLOSE requests, which free the descriptors
# whatever this setting says. That is why it looked like a size limit at first.
#
# With it defined, `SFTP_RecvOpen` records each descriptor in the handle list
# and `SFTP_FreeHandles` closes whatever is left when the session ends. Both
# halves are already in the vendored wolfSSH and are gated on this one macro --
# no source file has to change, which matters because wolfSSH here lives in the
# tinydesk-shell submodule and is not this project's to edit.
#
# Cost: one node per open file, ~300 bytes, from the SSH server's own 64 KiB
# PSRAM heap. A handful of concurrently open files is what this watch does.
idf_build_set_property(COMPILE_DEFINITIONS "WOLFSSH_STOREHANDLE" APPEND)
