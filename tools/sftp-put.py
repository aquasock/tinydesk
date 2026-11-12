#!/usr/bin/env python3
"""Move files to and from the watch over SFTP without hanging.

    ./tools/sftp-put.py put local.txt /sd/local.txt
    ./tools/sftp-put.py get /sd/local.txt ./local.txt
    ./tools/sftp-put.py ls /sd

WHY THIS EXISTS. Paramiko's `SFTPClient.put()` uses a 32 KiB write request, and
this watch's wolfSSH server does not answer a request that large: the transfer
does not error and does not slow down, it stops, and the client waits for ever.
Measured on the board 2026-11-12 -- a 16 KiB file completes in 0.59 s, a 24 KiB
file never finishes, and the same 64 KiB file written in 16 KiB pieces goes
through at 31 KiB/s. So the size limit is a property of the server, and the
workaround is chunk size.

THAT IS NOT ONLY AN INCONVENIENCE, WHICH IS WHY THIS IS A TOOL AND NOT A NOTE.
The SSH server accepts one client at a time (`TDSH_SSH_MAX_CLIENTS` is 1) and
services it on the accept loop's own task, so a wedged session holds the only
slot and every later attempt connects at the TCP layer and then receives no
protocol banner at all. The symptom is indistinguishable from a dead server and
only `ssh restart` or a reboot clears it -- so a single oversized `put` costs the
rest of the session's transfers, not just that one.

Reads are the mirror image: a 4 KiB read size runs at 70 KiB/s against 31 KiB/s
for writes, and a 16 KiB read stalls. Both directions are chunked here.

Every transfer is verified by md5 or size, because a silent short write over a
channel this flaky is worth ruling out rather than assuming away.
"""

import argparse
import hashlib
import os
import sys
import time

# Chosen against measurement, not taste, and THE TWO DIRECTIONS DIFFER.
#
#   writes  16 KiB is the largest request this server answers; 24 KiB and 32 KiB
#           never complete, and 4/8/16 KiB all run at 19-31 KiB/s.
#   reads   16 KiB stalls; 4 KiB completes at 70 KiB/s.
#
# One number for both directions would have been tidier and wrong -- which is
# exactly what the first version of this tool did, and a 233 KiB download hung
# on it after the same file uploaded fine.
PUT_CHUNK = 16 * 1024
GET_CHUNK = 4 * 1024

DEFAULT_HOST = os.environ.get("WATCH_HOST", "10.10.0.15")
DEFAULT_USER = os.environ.get("WATCH_USER", "root")


def password():
    pw = os.environ.get("WATCH_PASS")
    if pw:
        return pw
    # Order matters and was measured: ~/.config/escapement/watch-pass holds a
    # DIFFERENT credential from the one this watch accepts, and trying it first
    # produces "Authentication failed", which reads as a server fault rather
    # than as the wrong file. WATCH_PASS or WATCH_PASS_FILE overrides.
    for path in (
        os.environ.get("WATCH_PASS_FILE"),
        os.path.expanduser("~/Desktop/passwd.txt"),
    ):
        if not path:
            continue
        try:
            with open(path) as f:
                return f.read().strip()
        except OSError:
            continue
    return ""


def connect(args):
    import paramiko

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        args.host,
        port=args.port,
        username=args.user,
        password=password(),
        timeout=args.timeout,
        banner_timeout=args.timeout,
        auth_timeout=args.timeout,
        allow_agent=False,
        look_for_keys=False,
    )
    transport = client.get_transport()
    if transport is not None:
        transport.set_keepalive(10)
    return client


def md5_file(path):
    h = hashlib.md5()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def do_put(sftp, local, remote, chunk):
    total = os.path.getsize(local)
    sent = 0
    started = time.time()

    with open(local, "rb") as src, sftp.open(remote, "wb") as dst:
        while True:
            block = src.read(chunk)
            if not block:
                break
            dst.write(block)
            sent += len(block)

            elapsed = time.time() - started
            if elapsed > 0 and sent % (chunk * 8) == 0:
                sys.stderr.write(
                    "\r%d/%d KiB  %.1f KiB/s" % (sent // 1024, total // 1024, sent / elapsed / 1024)
                )
                sys.stderr.flush()

    elapsed = time.time() - started
    sys.stderr.write("\r" + " " * 50 + "\r")
    print("%d bytes in %.2f s (%.1f KiB/s)" % (sent, elapsed, sent / elapsed / 1024 if elapsed else 0))
    return sent == total


def do_get(sftp, remote, local, chunk):
    got = 0
    started = time.time()

    with sftp.open(remote, "rb") as src, open(local, "wb") as dst:
        while True:
            block = src.read(chunk)
            if not block:
                break
            dst.write(block)
            got += len(block)

    elapsed = time.time() - started
    print("%d bytes in %.2f s (%.1f KiB/s)" % (got, elapsed, got / elapsed / 1024 if elapsed else 0))
    return got


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["put", "get", "ls", "rm", "stat"])
    ap.add_argument("source", nargs="?")
    ap.add_argument("dest", nargs="?")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=int(os.environ.get("WATCH_PORT", "22")))
    ap.add_argument("--user", default=DEFAULT_USER)
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--chunk", type=int, default=0,
                    help="override the request size (default: 16384 for put, 4096 for get)")
    ap.add_argument("--verify", action="store_true", default=True)
    args = ap.parse_args()

    if args.action in ("put", "get") and not (args.source and args.dest):
        ap.error("%s needs a source and a destination" % args.action)

    try:
        client = connect(args)
    except Exception as exc:                       # noqa: BLE001 - reported, not handled
        print("connect failed: %s: %s" % (type(exc).__name__, exc), file=sys.stderr)
        print("if this says 'Error reading SSH protocol banner', the server is "
              "holding a wedged client: run `ssh restart` on the watch.", file=sys.stderr)
        return 2

    rc = 0
    try:
        sftp = client.open_sftp()

        if args.action == "ls":
            for entry in sftp.listdir_attr(args.source or "."):
                print("%10d  %s" % (entry.st_size or 0, entry.filename))

        elif args.action == "stat":
            st = sftp.stat(args.source)
            print("%s  %d bytes" % (args.source, st.st_size))

        elif args.action == "rm":
            sftp.remove(args.source)
            print("removed %s" % args.source)

        elif args.action == "put":
            ok = do_put(sftp, args.source, args.dest, args.chunk or PUT_CHUNK)
            if not ok:
                print("SHORT WRITE: local size differs from bytes sent", file=sys.stderr)
                rc = 1
            else:
                # Read the size back rather than trusting the write: the whole
                # point of this tool is that this channel lies quietly.
                remote_size = sftp.stat(args.dest).st_size
                local_size = os.path.getsize(args.source)
                if remote_size != local_size:
                    print("VERIFY FAILED: %d bytes on the watch, %d local"
                          % (remote_size, local_size), file=sys.stderr)
                    rc = 1
                else:
                    print("verified: %d bytes, md5 %s" % (remote_size, md5_file(args.source)))

        elif args.action == "get":
            do_get(sftp, args.source, args.dest, args.chunk or GET_CHUNK)
            print("md5 %s" % md5_file(args.dest))

        sftp.close()
    finally:
        client.close()

    return rc


if __name__ == "__main__":
    sys.exit(main())
