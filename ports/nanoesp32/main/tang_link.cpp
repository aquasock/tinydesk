// TinyNode <-> Tang link.  See tang_link.h.
//
// SPDX-License-Identifier: MIT

#include "tang_link.h"

#include <string.h>

namespace tanglink {

size_t answer(const uint8_t* data, size_t length, uint8_t* out, size_t capacity)
{
	tang_proto_t msg;
	if (!tang_proto_decode(data, length, &msg)) {
		// A message that lost a frame or was corrupted in the air. There is
		// nothing to answer, and the Tang's timeout is what recovers it.
		return 0;
	}
	if (tang_proto_is_reply(msg.type)) {
		// Replies travel the other way; a reply arriving here is not ours.
		return 0;
	}

	char text[TANG_PROTO_MAX_BODY];
	text[0] = '\0';

	switch (msg.type) {
	case TANG_REQ_STATUS:
		if (!node_answer("status", 6, text, sizeof text)) return 0;
		break;

	case TANG_REQ_COMMAND:
		if (msg.length == 0 || msg.length >= sizeof text) return 0;
		if (!node_answer((const char*)msg.body, msg.length, text, sizeof text)) {
			static const char unknown[] = "unknown command; send help";
			memcpy(text, unknown, sizeof unknown);
		}
		break;

	case TANG_REQ_MESSAGES:
	case TANG_REQ_BODY:
	case TANG_REQ_SEND:
	default: {
		// Recognised as a request but not answered yet, or not recognised at
		// all. Saying so is better than silence: the Tang can display it.
		static const char pending[] = "not implemented yet";
		memcpy(text, pending, sizeof pending);
		break;
	}
	}

	const uint8_t type = tang_proto_reply_type(msg.type);
	const size_t text_len = strlen(text);
	return tang_proto_encode(type, msg.seq, (const uint8_t*)text,
	                         (uint16_t)text_len, out, capacity);
}

}  // namespace tanglink
