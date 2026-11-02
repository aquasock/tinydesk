// Wire framing for the RYLR998 UART LoRa link.
//
// The RYLR998 is an AT-command module: it carries text, not bytes. Sending is
//
//     AT+SEND=<address>,<length>,<data>
//
// with an ASCII <data> field of at most 240 characters, and reception arrives
// as a whole text line:
//
//     +RCV=<address>,<length>,<data>,<rssi>,<snr>
//
// Reticulum packets are binary, so <data> is base64. That is not decoration:
// the AT syntax is comma-delimited and line-terminated, so a raw packet would
// both corrupt the command parser and be cut short at the first 0x0D. Base64
// is comma-free and newline-free, and 180 encoded bytes fill the 240-character
// field exactly, with no '=' padding.
//
// The binary frame carried inside that base64 is:
//
//     byte 0    header:  NNNN M III
//                        NNNN  nonce nibble, randomised per frame so identical
//                              packets do not look identical on air
//                        M     1 when more frames follow for this message
//                        III   message id, rolling 0-7 per message
//     byte 1..  up to 179 bytes of the Reticulum packet
//
// A Reticulum packet is at most 500 bytes (RNS::Type::Reticulum::MTU), so a
// full packet travels as three frames of 179 + 179 + 142 bytes. The receiver
// appends frames carrying the same id until one arrives with M clear.
//
// This unit deliberately depends on nothing but the C library: it is the part
// that can be exercised on a host (host-tests/test_lora.cpp), and it is the
// exact specification the far end (the Tang) has to implement.

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace loraframe {

// Largest <data> field the module accepts (AT+SEND payload ceiling).
constexpr size_t AT_DATA_MAX = 240;

// Largest binary frame: 1 header byte + a chunk that base64-encodes to exactly
// AT_DATA_MAX characters (180 bytes -> 240 chars, no padding).
constexpr size_t FRAME_MAX = 180;
constexpr size_t MAX_CHUNK = FRAME_MAX - 1;   // 179

// A full Reticulum packet (RNS::Type::Reticulum::MTU). Anything larger is not
// something the stack will hand us; it is rejected rather than silently cut.
constexpr size_t MAX_MESSAGE = 500;

constexpr uint8_t HDR_NONCE_MASK = 0xF0;
constexpr uint8_t HDR_MORE       = 0x08;
constexpr uint8_t HDR_ID_MASK    = 0x07;

inline uint8_t makeHeader(uint8_t nonce_nibble, bool more, uint8_t id) {
    return (uint8_t)(((nonce_nibble & 0x0F) << 4) | (more ? HDR_MORE : 0) | (id & HDR_ID_MASK));
}
inline bool    headerMore(uint8_t header) { return (header & HDR_MORE) != 0; }
inline uint8_t headerId(uint8_t header)   { return (uint8_t)(header & HDR_ID_MASK); }

// Standard base64. encode() writes a NUL-terminated string and returns its
// length, or 0 if the output buffer is too small. decode() skips whitespace and
// stops at '=' padding; it returns the number of bytes written, or 0 when the
// input cannot be decoded.
size_t base64Encode(const uint8_t* in, size_t n, char* out, size_t cap);
size_t base64Decode(const char* in, size_t n, uint8_t* out, size_t cap);

// Header + chunk -> the base64 body of one AT+SEND. Returns characters written,
// or 0 on error (chunk too large, or output buffer too small).
size_t encodeFrame(uint8_t nonce_nibble, bool more, uint8_t id,
                   const uint8_t* chunk, size_t chunk_len,
                   char* out, size_t cap);

// One parsed +RCV line. The frame is still base64-decoded binary at this point;
// split it with headerId()/headerMore() before handing the payload on.
struct Rcv {
    uint16_t from      = 0;
    int      rssi      = 0;
    int      snr       = 0;
    uint8_t  frame[FRAME_MAX] = {0};
    size_t   frame_len = 0;
};

// Parse "+RCV=<addr>,<len>,<data>,<rssi>,<snr>". Tolerates the spaces some
// firmware revisions put after the commas. Returns false on anything malformed:
// a bad length, a data field that is not valid base64, a frame that is empty or
// oversized.
bool parseRcv(const char* line, Rcv& out);

// Reassembles a message from frames that share a message id. Single-slot: one
// message in flight at a time, which is enough for a link that is half duplex
// anyway, and keeps the buffer at one MTU. A new id while a partial message is
// buffered means frames were lost; the partial is dropped and counted.
class Reassembler {
public:
    enum Result {
        INCOMPLETE,   // more frames expected
        COMPLETE,     // message ready: data()/size() are valid
        OVERFLOW,     // frame would exceed MAX_MESSAGE; message abandoned
    };

    Result push(const uint8_t* chunk, size_t n, uint8_t id, bool more, uint32_t now_ms);

    // Drop a partial message that has been waiting longer than the timeout, so
    // one lost frame does not swallow the next message that reuses the id.
    void expire(uint32_t now_ms);

    const uint8_t* data() const { return _buf; }
    size_t size() const { return _len; }
    void clear() { _len = 0; _active = false; }

    // Partial messages abandoned: lost frames, interference, or a timeout.
    uint32_t dropped() const { return _dropped; }

private:
    uint8_t  _buf[MAX_MESSAGE] = {0};
    size_t   _len       = 0;
    uint8_t  _id        = 0;
    bool     _active    = false;
    uint32_t _started_ms = 0;
    uint32_t _dropped   = 0;
};

// How long a partial message may wait for its remaining frames. Generous: at
// the default parameters a 240-character frame occupies the air for ~0.6 s.
constexpr uint32_t REASSEMBLY_TIMEOUT_MS = 8000;

}  // namespace loraframe
