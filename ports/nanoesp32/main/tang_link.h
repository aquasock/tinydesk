// TinyNode <-> Tang link: the requests the Tang makes and the answers it gets.
//
// SPDX-License-Identifier: MIT
//
// The Tang is the only party that initiates. Every message here is a request
// from it, answered once; the node never speaks first. Requests arrive as
// application messages (tang_proto.h) inside the LoRa framing, on frames whose
// sender address is the Tang's, and the reply goes back addressed to it.
//
// The answer text comes from the node's own command set -- the same strings an
// LXMF user gets -- so a new command on the node needs no change here or on the
// Tang. `status` has a compact form of its own, because the full one is aimed
// at a 63-byte LXMF budget and does not fit one application message.

#ifndef TANG_LINK_H
#define TANG_LINK_H

#include <stddef.h>
#include <stdint.h>

#include "tang_proto.h"

namespace tanglink {

// Turn one received application message into the reply bytes to send back.
// Returns the reply length, or 0 when the message is invalid, is not a request,
// or is a request with no answer (the caller then sends nothing).
size_t answer(const uint8_t* data, size_t length, uint8_t* out, size_t capacity);

// The node's answer to one command word, in main.cpp. Returns false when the
// word is not one of the node's commands.
bool node_answer(const char* command, size_t length, char* out, size_t capacity);

}  // namespace tanglink

#endif  // TANG_LINK_H
