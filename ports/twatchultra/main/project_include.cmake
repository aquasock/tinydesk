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
