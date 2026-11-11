# The platform-independent tinydesk core and apps, used straight from the
# repository's src/, include/ and apps/ (single source of truth).
get_filename_component(TD_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)

idf_component_register(
    SRCS
        "${TD_ROOT}/src/canvas.c"
        "${TD_ROOT}/src/picture.c"
        "${TD_ROOT}/src/font.c"
        "${TD_ROOT}/src/draw.c"
        "${TD_ROOT}/src/event.c"
        "${TD_ROOT}/src/input.c"
        "${TD_ROOT}/src/render.c"
        "${TD_ROOT}/src/screen.c"
        "${TD_ROOT}/src/td.c"
        "${TD_ROOT}/src/theme.c"
        "${TD_ROOT}/src/timer.c"
        "${TD_ROOT}/src/utf8.c"
        "${TD_ROOT}/src/vterm.c"
        "${TD_ROOT}/src/widgets.c"
        "${TD_ROOT}/src/wm.c"
        "${TD_ROOT}/apps/about.c"
        "${TD_ROOT}/apps/apps.c"
        "${TD_ROOT}/apps/counter.c"
        "${TD_ROOT}/apps/pixels.c"
        "${TD_ROOT}/apps/imagefile.c"
        "${TD_ROOT}/apps/imageview.c"
        "${TD_ROOT}/apps/mouse.c"
        "${TD_ROOT}/apps/keyboard.c"
        "${TD_ROOT}/apps/datetime.c"
        "${TD_ROOT}/apps/desktop.c"
        "${TD_ROOT}/apps/editor.c"
        "${TD_ROOT}/apps/editor_py.c"
        "${TD_ROOT}/apps/files.c"
        "${TD_ROOT}/apps/logview.c"
        "${TD_ROOT}/apps/modbus.c"
        "${TD_ROOT}/apps/mqtt.c"
        "${TD_ROOT}/apps/network.c"
        "${TD_ROOT}/apps/bluetooth.c"
        "${TD_ROOT}/apps/displays.c"
        "${TD_ROOT}/apps/pyui.c"
        "${TD_ROOT}/apps/pyui_cache.c"
        "${TD_ROOT}/apps/pyui_font.c"
        "${TD_ROOT}/apps/trash.c"
        "${TD_ROOT}/apps/search.c"
        "${TD_ROOT}/apps/kiosk.c"
        "${TD_ROOT}/apps/session.c"
        "${TD_ROOT}/apps/login.c"
        "${TD_ROOT}/apps/users.c"
        "${TD_ROOT}/apps/osk.c"
        "${TD_ROOT}/apps/store.c"
        "${TD_ROOT}/apps/remote.c"
        "${TD_ROOT}/apps/shortcut.c"
        "${TD_ROOT}/apps/settings.c"
        "${TD_ROOT}/apps/sysmon.c"
        "${TD_ROOT}/apps/taskmgr.c"
        "${TD_ROOT}/apps/terminal.c"
        "${TD_ROOT}/apps/update.c"
        "${TD_ROOT}/ports/common/td_fs_stdio.c"
        "${TD_ROOT}/proto/td_modbus.c"
        "${TD_ROOT}/proto/td_mqtt.c"
        "${TD_ROOT}/proto/td_mqtt_conf.c"
        "${TD_ROOT}/proto/td_sock.c"
        "${TD_ROOT}/proto/td_tls.c"
    INCLUDE_DIRS
        "${TD_ROOT}/include"
        "${TD_ROOT}/apps"
        "${TD_ROOT}/ports/common"
        "${TD_ROOT}/proto"
    REQUIRES vfs lwip esp_timer mbedtls
)

# Screen and terminal limits. The screen buffers cost 2 x cols x rows x 8
# bytes (32 KB at 80x25); the Terminal adds cols x rows x 4 bytes plus 4
# bytes per cell of scrollback. The desktop follows the terminal's size up
# to TD_MAX_COLS x TD_MAX_ROWS (tinydesk asks for the size every second).
#
# Without PSRAM (the C6) these come out of internal RAM, which TinyDesk Shell, Wi-Fi
# and SSH need, so the limit is 80x25 (100x30 would cost +16 KB). With PSRAM
# (the classic ESP32 port, whose tinydesk statics are in PSRAM, see
# ports/esp32/main/extram.lf) a maximised PuTTY fits (about 190x50 on a
# 1920-pixel screen): 256x96 costs about 390 KB of screen buffers and 300 KB
# of terminal with 200 lines of scrollback, out of 4 MB.
#
# The widget pool is shared by all windows (about 144 bytes a widget on the
# ESP32; all apps open need about 115): 96 without PSRAM (14 KB of internal
# RAM), the default 128 with it. A window that still does not fit, or gets
# no timer, is closed with a message instead of opening with missing parts.
#
# A PORT WITH A FIXED PANEL OVERRIDES THE GRID. These limits are an upper
# bound for a terminal whose size is not known until it answers (a PuTTY
# window), which is why they are generous. A board whose screen is a soldered
# LCD has no such question: its grid is known at build time, and letting a
# host terminal answer a larger size makes the desktop lay itself out for
# columns the glass does not have -- windows are measured to fit the host's
# window and then clipped, and text is cut mid-word. Setting TD_MAX_COLS and
# TD_MAX_ROWS in the port's own CMakeLists now sets the grid, because
# td_config.h documents exactly that ("Override any value from the build
# system, for example -DTD_MAX_COLS=100") and the window manager already
# clamps every reported size into these. Only one port sets them today
# (twatchultra, 40x26), so every other board is unaffected.
if(DEFINED TD_MAX_COLS AND DEFINED TD_MAX_ROWS)
    if(CONFIG_SPIRAM)
        set(TD_SCREEN_LIMITS TD_MAX_COLS=${TD_MAX_COLS} TD_MAX_ROWS=${TD_MAX_ROWS}
            TD_VT_SCROLLBACK=200 TD_PYUI_MAX_CELLS=32000
            TD_PYUI_MAX_PIXELS=2097152 TD_PICTURE_MAX=128)
    else()
        set(TD_SCREEN_LIMITS TD_MAX_COLS=${TD_MAX_COLS} TD_MAX_ROWS=${TD_MAX_ROWS}
            TD_VT_SCROLLBACK=12 TD_MAX_WIDGETS=96)
    endif()
    # FIXED MEANS FIXED IN BOTH DIRECTIONS. The grid is declared because the
    # panel is soldered, so a terminal reporting a smaller size must not shrink
    # the desktop either: answering 24 rows for a 26-row panel lost the bottom
    # two rows and left a black band in the framebuffer. Pinning the minimum to
    # the declared grid makes src/td.c's clamp degenerate -- every reported size
    # lands on exactly one value -- which is what "fixed" should mean.
    list(APPEND TD_SCREEN_LIMITS TD_MIN_COLS=${TD_MAX_COLS} TD_MIN_ROWS=${TD_MAX_ROWS})
    message(STATUS "tinydesk grid fixed by the port: ${TD_MAX_COLS}x${TD_MAX_ROWS}")
elseif(CONFIG_SPIRAM)
    # Python canvases may cover a whole large screen several times over
    # (pages: 100x30 is 3000 cells, 6 bytes each); they are allocated in
    # PSRAM while a script runs, 192 KB at most. Their pixel layers and the
    # script's images (a byte a pixel) may take 2 MB of PSRAM.
    set(TD_SCREEN_LIMITS TD_MAX_COLS=256 TD_MAX_ROWS=96 TD_VT_SCROLLBACK=200 TD_PYUI_MAX_CELLS=32000
        TD_PYUI_MAX_PIXELS=2097152 TD_PICTURE_MAX=128)
else()
    set(TD_SCREEN_LIMITS TD_MAX_COLS=80 TD_MAX_ROWS=25 TD_VT_SCROLLBACK=12 TD_MAX_WIDGETS=96)
endif()
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    TD_HAVE_TLS
    ${TD_SCREEN_LIMITS}
    TD_OUT_BUF_SIZE=1024
    TD_TEXT_MAX=48
    TD_LOG_LINES=32
    TD_LOG_LINE_MAX=88
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
option(TD_RENDER_OPTIMIZE "Optimize native rendering hot paths" ON)
if(TD_RENDER_OPTIMIZE)
    set_source_files_properties("${TD_ROOT}/apps/pyui.c" "${TD_ROOT}/apps/pyui_cache.c"
        "${TD_ROOT}/src/picture.c" "${TD_ROOT}/src/canvas.c" "${TD_ROOT}/src/screen.c"
        "${TD_ROOT}/src/render.c" "${TD_ROOT}/src/wm.c" PROPERTIES COMPILE_OPTIONS "-O2")
endif()
