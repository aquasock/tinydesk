#!/usr/bin/env python3
"""Read the watch's desktop over USB into a grid of characters.

    tools/watch-screen.py                 # repaint and print the screen
    tools/watch-screen.py '\x1b[21~' 3     # send keys first, then read

This exists because the watch has no serial console: the panel is write-only and
the only way to see what the desktop is doing from a host is the USB mirror,
which speaks raw VT100. Parsing it needs a screen model, and that model had to
be correct before it could be trusted -- see the DECSC note below.

TWO THINGS LEARNED THE HARD WAY, both of which produced a confident wrong answer:

  1. DRAIN BEFORE SENDING. Bytes from a previous frame are still in flight; a
     read that starts immediately parses the tail of the old frame and reports
     a blank or stale screen.

  2. IMPLEMENT DECSC/DECRC. The desktop brackets its cursor-position query with
     save (ESC 7) and restore (ESC 8) around a jump to 999;999. A model that
     ignores those is left parked at row 999 and writes every glyph off-screen,
     which looks exactly like a blank desktop. That cost a false report that the
     taskbar had disappeared when it was working.

Positions are clamped for the same reason: 999;999 is a legitimate query and
must not poison every later write.
"""
import os, select, subprocess, sys, termios, time, tty

COLS, ROWS = 40, 26

class Screen:
    def __init__(self, cols=COLS, rows=ROWS):
        self.cols, self.rows = cols, rows
        self.g = [[" "] * cols for _ in range(rows)]
        self.x = self.y = 0
        self.saved = (0, 0)
        self.csi = b""

    def feed(self, data):
        i = 0
        while i < len(data):
            b = data[i]
            if b == 0x1B:
                # DECSC / DECRC. The desktop brackets its cursor-position
                # query with these (save, jump to 999;999, ask, restore), so
                # ignoring them leaves the model parked at row 999 and every
                # glyph after it is written off-screen -- which looks exactly
                # like a blank desktop.
                if i + 1 < len(data) and data[i + 1] == ord("7"):
                    self.saved = (self.x, self.y); i += 2; continue
                if i + 1 < len(data) and data[i + 1] == ord("8"):
                    self.x, self.y = self.saved; i += 2; continue
                if i + 1 < len(data) and data[i + 1] == ord("["):
                    j = i + 2
                    while j < len(data) and not (0x40 <= data[j] <= 0x7E):
                        j += 1
                    if j < len(data):
                        self._csi(data[i + 2:j], chr(data[j]))
                        i = j + 1
                        continue
                i += 2
                continue
            if b == 0x0A:
                self.y = min(self.y + 1, self.rows - 1); i += 1; continue
            if b == 0x0D:
                self.x = 0; i += 1; continue
            if b == 0x08:
                self.x = max(0, self.x - 1); i += 1; continue
            if b < 0x20:
                i += 1; continue
            # UTF-8 sequence
            n = 1
            if b & 0xE0 == 0xC0: n = 2
            elif b & 0xF0 == 0xE0: n = 3
            elif b & 0xF8 == 0xF0: n = 4
            ch = data[i:i + n].decode("utf-8", "replace")
            if 0 <= self.y < self.rows and 0 <= self.x < self.cols:
                self.g[self.y][self.x] = ch
            self.x += 1
            if self.x >= self.cols:
                self.x = self.cols - 1
            i += n

    def _csi(self, params, final):
        p = params.decode("ascii", "replace")
        if final in "Hf":
            parts = (p.split(";") + ["", ""])[:2]
            y = int(parts[0]) if parts[0] else 1
            x = int(parts[1]) if parts[1] else 1
            # Clamp: a position past the grid is legitimate (the size query
            # uses 999;999) and must not poison every later write.
            self.y = max(0, min(y - 1, self.rows - 1))
            self.x = max(0, min(x - 1, self.cols - 1))
        elif final == "J" and (p in ("2", "")):
            self.g = [[" "] * self.cols for _ in range(self.rows)]
            self.x = self.y = 0
        elif final == "K":
            for c in range(self.x, self.cols):
                if 0 <= self.y < self.rows:
                    self.g[self.y][c] = " "

    def text(self):
        return "\n".join("".join(r).rstrip() for r in self.g)


def _udev_props(port):
    """The kernel's description of the device behind a tty path."""
    try:
        real = os.path.realpath(port)
        name = os.path.basename(real)
        out = subprocess.run(
            ["udevadm", "info", "-q", "property", "-n", "/dev/" + name],
            capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return {}
    props = {}
    for line in out.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            props[k] = v
    return props


def _describe(port):
    p = _udev_props(port)
    return "%s (%s %s)" % (os.path.basename(port),
                           p.get("ID_VENDOR_ID", "?"),
                           p.get("ID_MODEL", "?"))


# The watch is the ESP32-S3's own USB-Serial-JTAG: vendor 303a, model 1001.
_WATCH_VID_MODEL = ("303a", "1001")
_WATCH_FORCED = [False]


def _is_watch(port):
    if _WATCH_FORCED[0]:
        return True
    p = _udev_props(port)
    return (p.get("ID_VENDOR_ID"), p.get("ID_MODEL_ID")) == _WATCH_VID_MODEL


def watch_port():
    """The watch's port, by serial number rather than by ttyACM number.

    THE WATCH AND THE TINYTANG SWAP ttyACM0 AND ttyACM1 BETWEEN REBOOTS, so
    neither number identifies a board. /dev/serial/by-id names the device by its
    USB serial and is the only stable handle; read_modaddr.py in the port already
    resolves the watch this way for the same reason.
    """
    by_id = "/dev/serial/by-id"
    watch_serial = "usb-Espressif_USB_JTAG_serial_debug_unit_80:45:6B:46:DF:54-if00"
    path = os.path.join(by_id, watch_serial)
    if os.path.exists(path):
        return path
    for name in sorted(os.listdir(by_id)) if os.path.isdir(by_id) else []:
        if name.startswith("usb-Espressif_USB_JTAG_serial_debug_unit"):
            return os.path.join(by_id, name)
    return "/dev/ttyACM1"


def open_port(port=None):
    """Open the board's USB port, refusing anything that is not the watch.

    THE DEFAULT USED TO BE /dev/ttyACM0, AND ON THIS WORKSTATION THAT IS THE
    TINYTANG. A run with no argument therefore wrote its Ctrl+L repaint into a
    different board's shell, which is how the operator came to say "you are
    sending crap to the tang". The numbers swap between reboots, so identity is
    checked from the kernel's own description of the device and gates a WRITE: a
    mismatch raises before a byte is sent. --force-port exists for a different
    board, not as a convenience.
    """
    port = port or watch_port()
    if not _is_watch(port):
        raise SystemExit(
            "%s is not the watch (expected an Espressif USB-JTAG debug unit).\n"
            "  found: %s\n"
            "  refusing to write to it." % (port, _describe(port)))
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)
    tty.setraw(fd)
    return fd


def drain(fd, seconds):
    out, end = b"", time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            try:
                c = os.read(fd, 65536)
            except BlockingIOError:
                continue
            if not c:
                break
            out += c
    return out


def drain_quiet(fd, quiet=0.15, limit=3.0):
    """Read until the line has been silent for `quiet` seconds.

    A fixed-duration drain is not enough: bytes from the previous frame arrive
    across a window, and parsing a partial frame is what produced a blank
    screen in the report that the taskbar had vanished.
    """
    out, end, last = b"", time.time() + limit, time.time()
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], quiet)
        if not r:
            if time.time() - last >= quiet:
                break
            continue
        try:
            c = os.read(fd, 65536)
        except BlockingIOError:
            continue
        if not c:
            break
        out += c
        last = time.time()
    return out


def read_screen(keys=b"", wait=2.0, fresh=True, port=None):
    fd = open_port(port or watch_port())
    drain_quiet(fd)
    if keys:
        os.write(fd, keys)
    data = drain(fd, wait)
    if not keys:
        os.write(fd, b"\x0c")          # Ctrl+L: force a full repaint
        data += drain_quiet(fd, limit=max(wait, 3.0))
    os.close(fd)
    s = Screen()          # a fresh model per capture, as the method requires
    s.feed(data)
    return s


if __name__ == "__main__":
    argv = sys.argv[1:]
    port, rest = None, []
    i = 0
    while i < len(argv):
        if argv[i] == "--port":
            port = argv[i + 1]
            i += 2
        elif argv[i].startswith("--port="):
            port = argv[i].split("=", 1)[1]
            i += 1
        elif argv[i] == "--force-port":
            _WATCH_FORCED[0] = True
            i += 1
        else:
            rest.append(argv[i])
            i += 1
    keys = rest[0].encode().decode("unicode_escape").encode("latin-1") if rest else b""
    wait = float(rest[1]) if len(rest) > 1 else 2.0
    print(read_screen(keys, wait, port=port).text())
