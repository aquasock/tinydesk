// RYLR998 UART LoRa interface for the TinyNode Nano port.
//
// Ported from the standalone TinyNode (P1), less the TinyTang protocol: this
// node speaks Reticulum only, so every frame that arrives with a valid framing
// header goes up the stack and nothing is answered locally.
//
// Wiring (module pin -> board pin), 3.3 V logic only -- the module is not 5 V
// tolerant and its VDD range is 2.3-3.6 V:
//
//     RYLR998 VDD   -> 3V3
//     RYLR998 GND   -> GND
//     RYLR998 TXD   -> D5   (GPIO 8,  UART RX)
//     RYLR998 RXD   -> D6   (GPIO 9,  UART TX)
//     RYLR998 NRST  -> D7   optional; define LORA_NRST_PIN to use it
//
// D5/D6 are free on this port: the I2C bus is on A4/A5 (GPIO 11/12), the panel
// is on the SPI pins (GPIO 18/21/38/47/48), and the console is USB
// Serial/JTAG, so UART0 on D0/D1 stays free as the recovery console.
//
// The air protocol is described in lora_frame.h; the AT command set is in
// REYAX's "LoRa AT COMMAND GUIDE" (RYLR998/RYLR498).

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <string>

namespace lora {

// Where the driver's console lines go. The port passes its own logger so LoRa
// bring-up appears in /node.log beside everything else; left unset the driver
// stays silent rather than writing to a stdout that may not exist.
typedef void (*log_fn)(const char* line);
void set_logger(log_fn fn);

// Configure the module and register the interface with Reticulum. Returns false
// when the module does not answer, in which case the node keeps running on
// WiFi alone -- a missing LoRa harness must not stop the node from booting.
bool begin();

bool online();

// One line, no newline: address, network, band, air parameters and counters.
// Returns false when the interface was never started.
bool status_line(char* out, size_t n);

// Very short form for an LXMF reply, kept under the 63-byte single-packet
// content limit so it stays usable over the LoRa link it describes.
std::string short_status();

// Multi-line detail for the `lora` console command.
void console_report();

// Just the traffic counters and last signal, for a reply that has to fit one
// LoRa frame: the full status line is ~75 bytes and would cost a second frame.
bool counters(uint32_t *tx, uint32_t *rx, int *rssi, int *snr);

// Log every message that reassembles, whatever it turns out to be. Off by
// default; `lora verbose on` turns it on. This is what separates "the radio
// heard something and the stack rejected it" from "the radio heard nothing".
void set_verbose(bool on);

}  // namespace lora
