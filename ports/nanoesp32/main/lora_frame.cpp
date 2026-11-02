#include "lora_frame.h"

#include <cstring>

namespace loraframe {

namespace {

const char B64_ALPHABET[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int b64Value(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

inline bool isSpace(char c) { return c == ' ' || c == '\t'; }

void trimRange(const char*& begin, const char*& end) {
    while (begin < end && isSpace(*begin)) ++begin;
    while (end > begin && isSpace(*(end - 1))) --end;
}

// Strict non-negative integer, for the address and length fields.
bool parseUnsigned(const char* begin, const char* end, long& out) {
    trimRange(begin, end);
    if (begin == end) return false;
    long value = 0;
    for (const char* p = begin; p != end; ++p) {
        if (*p < '0' || *p > '9') return false;
        value = value * 10 + (*p - '0');
        if (value > 1000000L) return false;
    }
    out = value;
    return true;
}

// Lenient signed number for RSSI/SNR: some firmware revisions report SNR with a
// fractional part ("-7.25"), so accept and round one. Anything else is a
// malformed line rather than a value worth guessing at.
bool parseSigned(const char* begin, const char* end, long& out) {
    trimRange(begin, end);
    if (begin == end) return false;

    bool negative = false;
    if (*begin == '-') { negative = true; ++begin; }
    else if (*begin == '+') { ++begin; }
    if (begin == end) return false;

    long value        = 0;
    bool afterPoint   = false;
    int  decimals     = 0;
    long firstDecimal = 0;
    for (const char* p = begin; p != end; ++p) {
        if (*p == '.') {
            if (afterPoint) return false;    // a second decimal point
            afterPoint = true;
            continue;
        }
        if (*p < '0' || *p > '9') return false;
        if (!afterPoint) {
            value = value * 10 + (*p - '0');
            if (value > 1000000L) return false;
        }
        else if (decimals++ == 0) {
            firstDecimal = *p - '0';         // one decimal place is enough
        }
    }
    if (firstDecimal >= 5) ++value;          // round to nearest whole
    out = negative ? -value : value;
    return true;
}

// Last comma strictly inside [begin, end), or nullptr.
const char* findCommaBack(const char* begin, const char* end) {
    for (const char* p = end; p > begin; --p) {
        if (*(p - 1) == ',') return p - 1;
    }
    return nullptr;
}

}  // namespace

size_t base64Encode(const uint8_t* in, size_t n, char* out, size_t cap) {
    if (out == nullptr) return 0;
    if (n > 0 && in == nullptr) return 0;
    const size_t need = 4 * ((n + 2) / 3);
    if (need + 1 > cap) return 0;

    size_t o = 0;
    for (size_t i = 0; i < n; i += 3) {
        const size_t left = n - i;
        const uint32_t b0 = in[i];
        const uint32_t b1 = left > 1 ? in[i + 1] : 0;
        const uint32_t b2 = left > 2 ? in[i + 2] : 0;
        const uint32_t v  = (b0 << 16) | (b1 << 8) | b2;
        out[o++] = B64_ALPHABET[(v >> 18) & 0x3F];
        out[o++] = B64_ALPHABET[(v >> 12) & 0x3F];
        out[o++] = left > 1 ? B64_ALPHABET[(v >> 6) & 0x3F] : '=';
        out[o++] = left > 2 ? B64_ALPHABET[v & 0x3F] : '=';
    }
    out[o] = '\0';
    return o;
}

size_t base64Decode(const char* in, size_t n, uint8_t* out, size_t cap) {
    if (in == nullptr || out == nullptr) return 0;

    size_t   o    = 0;
    uint32_t acc  = 0;
    int      bits = 0;
    for (size_t i = 0; i < n; ++i) {
        const char c = in[i];
        if (c == '=') break;                      // padding ends the payload
        if (isSpace(c) || c == '\r' || c == '\n') continue;
        const int v = b64Value(c);
        if (v < 0) return 0;                      // not base64: do not guess
        acc = (acc << 6) | (uint32_t)v;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (o >= cap) return 0;
            out[o++] = (uint8_t)((acc >> bits) & 0xFF);
        }
    }
    return o;
}

size_t encodeFrame(uint8_t nonce_nibble, bool more, uint8_t id,
                   const uint8_t* chunk, size_t chunk_len,
                   char* out, size_t cap) {
    if (chunk_len > MAX_CHUNK) return 0;
    if (chunk_len > 0 && chunk == nullptr) return 0;

    uint8_t frame[FRAME_MAX];
    frame[0] = makeHeader(nonce_nibble, more, id);
    if (chunk_len > 0) std::memcpy(frame + 1, chunk, chunk_len);
    return base64Encode(frame, chunk_len + 1, out, cap);
}

bool parseRcv(const char* line, Rcv& out) {
    if (line == nullptr) return false;
    if (std::strncmp(line, "+RCV=", 5) != 0) return false;

    const char* begin = line + 5;
    const char* end   = line + std::strlen(line);

    // Split from the right: the data field holds base64, which contains no
    // commas, so the last two commas always delimit RSSI and SNR.
    const char* comma_snr  = findCommaBack(begin, end);
    if (comma_snr == nullptr) return false;
    const char* comma_rssi = findCommaBack(begin, comma_snr);
    if (comma_rssi == nullptr) return false;
    const char* comma_data = findCommaBack(begin, comma_rssi);
    if (comma_data == nullptr) return false;
    const char* comma_len  = findCommaBack(begin, comma_data);
    if (comma_len == nullptr) return false;

    long address = 0, length = 0, rssi = 0, snr = 0;
    if (!parseUnsigned(begin, comma_len, address)) return false;
    if (!parseUnsigned(comma_len + 1, comma_data, length)) return false;
    if (!parseSigned(comma_rssi + 1, comma_snr, rssi)) return false;
    if (!parseSigned(comma_snr + 1, end, snr)) return false;
    if (address > 65535) return false;
    if (length <= 0 || (size_t)length > AT_DATA_MAX) return false;

    const char* data_begin = comma_data + 1;
    const char* data_end   = comma_rssi;
    trimRange(data_begin, data_end);
    if ((size_t)(data_end - data_begin) != (size_t)length) return false;

    const size_t frame_len = base64Decode(data_begin, (size_t)(data_end - data_begin),
                                          out.frame, sizeof(out.frame));
    if (frame_len == 0 || frame_len > FRAME_MAX) return false;

    out.from      = (uint16_t)address;
    out.rssi      = (int)rssi;
    out.snr       = (int)snr;
    out.frame_len = frame_len;
    return true;
}

Reassembler::Result Reassembler::push(const uint8_t* chunk, size_t n, uint8_t id,
                                      bool more, uint32_t now_ms) {
    if (n > MAX_CHUNK) return OVERFLOW;
    if (n > 0 && chunk == nullptr) return OVERFLOW;

    if (_active && _id != id) {
        // A different message has started, so the frames of this one are gone.
        if (_len > 0) ++_dropped;
        clear();
    }
    if (!_active) {
        _id         = id;
        _active     = true;
        _started_ms = now_ms;
        _len        = 0;
    }
    if (_len + n > sizeof(_buf)) {
        clear();
        ++_dropped;
        return OVERFLOW;
    }
    if (n > 0) std::memcpy(_buf + _len, chunk, n);
    _len += n;

    if (more) return INCOMPLETE;

    _active = false;
    if (_len == 0) {
        ++_dropped;
        return OVERFLOW;
    }
    return COMPLETE;
}

void Reassembler::expire(uint32_t now_ms) {
    if (!_active) return;
    if ((uint32_t)(now_ms - _started_ms) < REASSEMBLY_TIMEOUT_MS) return;
    ++_dropped;
    clear();
}

}  // namespace loraframe
