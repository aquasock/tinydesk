#!/usr/bin/env python3
"""Scroll-aware reader for the watch's USB mirror.

    ./scrollread.py                 # capture a few seconds and dump scrollback
    ./scrollread.py 'mem tasks\\r' 4 # send keys first, then capture

WHY THIS EXISTS. tools/watch-screen.py models a fixed 40x26 grid with no scroll
region, so anything that scrolls off the top is lost -- which is why the Start
menu and any long command output have been unreadable from the host. This adds
the missing piece: DECSTBM (CSI r) scroll regions and scrolling when the cursor
runs off the bottom of its region, keeping a scrollback buffer of lines that
left the top.

That is the same limitation core-log entry 37 recorded against the other tool,
and the same one that made `mem tasks` unreadable on a 40-column screen.
"""
import os, select, sys, termios, time, tty

COLS, ROWS = 40, 26


class ScrollScreen:
    def __init__(self, cols=COLS, rows=ROWS):
        self.cols, self.rows = cols, rows
        self.g = [[" "] * cols for _ in range(rows)]
        self.x = self.y = 0
        self.saved = (0, 0)
        self.scroll_top = 0
        self.scroll_bot = rows - 1
        self.scrollback = []

    def _scroll_up(self):
        top, bot = self.scroll_top, self.scroll_bot
        if top == 0:
            self.scrollback.append("".join(self.g[0]).rstrip())
        for r in range(top, bot):
            self.g[r] = self.g[r + 1]
        self.g[bot] = [" "] * self.cols

    def linefeed(self):
        if self.y == self.scroll_bot:
            self._scroll_up()
        else:
            self.y = min(self.y + 1, self.rows - 1)

    def put(self, ch):
        if 0 <= self.y < self.rows and 0 <= self.x < self.cols:
            self.g[self.y][self.x] = ch
        self.x += 1
        if self.x >= self.cols:
            self.x = 0
            self.linefeed()

    def csi(self, params, final):
        p = [int(v) if v.isdigit() else 0 for v in params.split(";")] if params else []
        if final == "r":
            top = (p[0] if len(p) > 0 and p[0] else 1) - 1
            bot = (p[1] if len(p) > 1 and p[1] else self.rows) - 1
            self.scroll_top, self.scroll_bot = top, bot
            self.x, self.y = 0, top
        elif final == "H" or final == "f":
            r = (p[0] if len(p) > 0 and p[0] else 1) - 1
            c = (p[1] if len(p) > 1 and p[1] else 1) - 1
            self.y = max(0, min(r, self.rows - 1))
            self.x = max(0, min(c, self.cols - 1))
        elif final == "J":
            for r in range(self.rows):
                self.g[r] = [" "] * self.cols
        elif final == "K":
            if 0 <= self.y < self.rows:
                for c in range(self.x, self.cols):
                    self.g[self.y][c] = " "
        elif final == "A":
            self.y = max(self.scroll_top, self.y - (p[0] or 1))
        elif final == "B":
            self.y = min(self.scroll_bot, self.y + (p[0] or 1))
        elif final == "C":
            self.x = min(self.cols - 1, self.x + (p[0] or 1))
        elif final == "D":
            self.x = max(0, self.x - (p[0] or 1))

    def feed(self, data):
        i = 0
        while i < len(data):
            b = data[i]
            if b == 0x1B:
                if i + 1 < len(data) and data[i + 1] == ord("7"):
                    self.saved = (self.x, self.y); i += 2; continue
                if i + 1 < len(data) and data[i + 1] == ord("8"):
                    self.x, self.y = self.saved; i += 2; continue
                if i + 1 < len(data) and data[i + 1] == ord("["):
                    j = i + 2
                    while j < len(data) and not (0x40 <= data[j] <= 0x7E):
                        j += 1
                    if j < len(data):
                        self.csi(data[i + 2:j].decode("ascii", "replace"),
                                 chr(data[j]))
                        i = j + 1
                        continue
                i += 2
                continue
            if b == 0x0A:
                self.linefeed(); i += 1; continue
            if b == 0x0D:
                self.x = 0; i += 1; continue
            if b == 0x08:
                self.x = max(0, self.x - 1); i += 1; continue
            if b < 0x20:
                i += 1; continue
            n = 1
            if b & 0xE0 == 0xC0: n = 2
            elif b & 0xF0 == 0xE0: n = 3
            elif b & 0xF8 == 0xF0: n = 4
            self.put(data[i:i + n].decode("utf-8", "replace"))
            i += n


def main():
    keys = sys.argv[1].encode().decode("unicode_escape").encode() if len(sys.argv) > 1 else b""
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

    fd = os.open("/dev/ttyACM0", os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    # Drain whatever is in flight before sending, or the first read parses the
    # tail of the previous frame.
    while select.select([fd], [], [], 0.3)[0]:
        os.read(fd, 65536)

    if keys:
        os.write(fd, keys)
        time.sleep(0.6)

    scr = ScrollScreen()
    t0 = time.time()
    while time.time() - t0 < secs:
        if select.select([fd], [], [], 0.3)[0]:
            try:
                d = os.read(fd, 65536)
            except OSError:
                break
            if not d:
                break
            scr.feed(d)
    os.close(fd)

    for line in scr.scrollback:
        if line.strip():
            print(line)
    print("-" * 40)
    for row in scr.g:
        print("".join(row).rstrip())


if __name__ == "__main__":
    main()
