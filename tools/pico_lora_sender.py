# TinyNode bench tool: a RYLR998 sender on a Raspberry Pi Pico 2.
#
# Purpose: prove TinyNode's LoRa *receive* path, which core-reference RADIO-009
# records as never having been exercised. The node's transmit side is proven;
# this drives the other direction from a known-good endpoint, so a failure
# points at one thing instead of three.
#
# Wiring (module header is VDD, NRST, RXD, TXD, GND):
#
#   module VDD -> Pico 3V3 OUT (pin 36)     module TXD -> Pico GP1 (pin 2)
#   module GND -> Pico GND     (pin 38)     module RXD -> Pico GP0 (pin 1)
#   module NRST -> leave unconnected (100 kOhm pull-up on the module)
#
# UART0 at 115200 8N1, the module's default. 3.3 V only: the module's UART is
# not 5 V tolerant and its absolute maximum is 3.6 V.
#
# The settings below mirror TinyNode's, because two modules only talk when the
# network id, band and air parameters agree. The node runs network 6, 915 MHz,
# SF7 / 125 kHz / 4-5 / preamble 12, expects the far end at address 2, and
# transmits to broadcast address 0.
#
# The frame this produces is the one `nano-node/include/lora_frame.h` specifies:
#
#   byte 0    NNNN M III   nonce nibble, more-frames flag, message id
#   byte 1..  up to 179 bytes of payload
#
# base64-encoded into the AT field, because the AT syntax is comma-delimited and
# line-terminated.

import time
import ubinascii
from machine import UART, Pin

BAUD = 115200
NODE_ADDRESS = 1          # TinyNode's module address
THIS_ADDRESS = 2          # the address the node expects its peer to hold
NETWORK_ID = 6
PARAMETER = (7, 7, 1, 12) # SF7, BW index 7 (125 kHz), CR 1 (4/5), preamble 12
POWER_DBM = 14            # CE ceiling, and what TinyNode uses
BAND_HZ = 915000000

AT_DATA_MAX = 240         # module's <data> ceiling
CHUNK = 179               # payload bytes per frame at that ceiling

uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1), timeout=300)


def drain(ms=400):
    """Read whatever the module has said in the next `ms` milliseconds."""
    end = time.ticks_add(time.ticks_ms(), ms)
    out = b""
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        chunk = uart.read()
        if chunk:
            out += chunk
        time.sleep_ms(20)
    return out.decode().strip()


def cmd(text, ms=400):
    uart.write(text + "\r\n")
    return drain(ms)


def configure():
    """Set the module to the node's network, then read every value back."""
    print("version   :", cmd("AT+VER?"))
    cmd("AT+ADDRESS=%d" % THIS_ADDRESS)
    cmd("AT+NETWORKID=%d" % NETWORK_ID)
    cmd("AT+PARAMETER=%d,%d,%d,%d" % PARAMETER)
    cmd("AT+CRFOP=%d" % POWER_DBM)
    cmd("AT+BAND=%d" % BAND_HZ)
    print("address   :", cmd("AT+ADDRESS?"))
    print("network   :", cmd("AT+NETWORKID?"))
    print("parameter :", cmd("AT+PARAMETER?"))
    print("power     :", cmd("AT+CRFOP?"))
    print("band      :", cmd("AT+BAND?"))


def frame(nonce, more, msg_id, payload):
    """One frame, base64-encoded, ready for the AT <data> field."""
    header = ((nonce & 0x0F) << 4) | (0x08 if more else 0x00) | (msg_id & 0x07)
    return ubinascii.b2a_base64(bytes([header]) + payload).strip().decode()


def send(payload, msg_id=0, nonce=3, ms=1200):
    body = frame(nonce, False, msg_id, payload)
    if len(body) > AT_DATA_MAX:
        raise ValueError("frame too long for the AT field")
    reply = cmd("AT+SEND=%d,%d,%s" % (NODE_ADDRESS, len(body), body), ms)
    print("sent %2d bytes (id %d): %s" % (len(payload), msg_id, reply))
    return reply


def send_message(data, first_id=0):
    """Split a message into frames the way the node reassembles them."""
    msg_id = first_id & 0x07
    chunks = [data[i:i + CHUNK] for i in range(0, len(data), CHUNK)] or [b""]
    for index, chunk in enumerate(chunks):
        more = index < len(chunks) - 1
        body = frame(3, more, msg_id, chunk)
        cmd("AT+SEND=%d,%d,%s" % (NODE_ADDRESS, len(body), body), 900)
        print("  frame %d/%d: %d payload bytes" % (index + 1, len(chunks), len(chunk)))
    print("message of %d bytes sent as %d frame(s), id %d" %
          (len(data), len(chunks), msg_id))


def main():
    print("TinyNode LoRa receive test, Pico 2")
    configure()

    # 1. A single frame. The node's driver should count one frame and one
    #    packet; Reticulum will discard the payload, and that is expected --
    #    the counters are the evidence, not the log line.
    print("\n[1] single 30-byte frame")
    send(bytes(range(30)))

    # 2. A 500-byte message, the Reticulum MTU, which must arrive as exactly
    #    three frames of 179 + 179 + 142 and reassemble to one packet.
    print("\n[2] full-MTU message, three frames")
    payload = bytes((i * 7) & 0xFF for i in range(500))
    send_message(payload, first_id=1)

    print("\ndone: on the node, `L` should show rx 2, frames 4, errors 0")


if __name__ == "__main__":
    main()
