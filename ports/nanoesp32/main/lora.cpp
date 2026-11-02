// RYLR998 (REYAX) LoRa interface: AT driver plus a Reticulum interface.
//
// The module is a text device, not a radio chip: everything goes through
// "AT+..." commands on one UART, and received packets come back as "+RCV=..."
// lines. lora_frame.h carries the binary framing (base64 body, header byte,
// multi-frame reassembly); this file is the AT state machine around it.
//
// Two rules shape the implementation:
//
//   1. Nothing blocks the main loop for longer than one AT command. A 240-
//      character frame occupies the air for ~0.6 s at SF7/125 kHz, so a
//      blocking transmit would also stall the GPS, the I2C sensors and the LXMF
//      router. Outgoing packets are queued and drained one frame at a time from
//      loop(), waiting for +OK before the next.
//   2. Settings are read back before they are written. The module stores some
//      of them in flash (200k erase cycles), so writing the same value on every
//      boot is wear for nothing; what is actually held is also printed, so a
//      mismatch with the far end is visible rather than silent.
//
// Ported from the standalone TinyNode. The differences are all mechanical: the
// Arduino HardwareSerial is replaced by the ESP-IDF UART driver because this
// port builds against ESP-IDF with no Arduino, and the TinyTang protocol is
// gone because this node speaks only Reticulum.

#include "lora.h"
#include "lora_frame.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <microReticulum/Interface.h>
#include <microReticulum/Bytes.h>
#include <microReticulum/Type.h>
#include <microReticulum/Log.h>
#include <microReticulum/Transport.h>
#include <microReticulum/Cryptography/Random.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// GPIO numbers, not Arduino pin names: this port drives the SoC directly.
// D5 = GPIO 8 is the board's RX (module TXD -> here); D6 = GPIO 9 is its TX.
#ifndef LORA_RX_GPIO
#define LORA_RX_GPIO 8
#endif
#ifndef LORA_TX_GPIO
#define LORA_TX_GPIO 9
#endif
#ifndef LORA_NRST_PIN
#define LORA_NRST_PIN -1        // wire NRST to a spare GPIO and set it to use
#endif

// 0 keeps whatever band the module already holds (factory default is 915 MHz).
// Both ends of the link must be on the same band; set it to 868500000 or
// 915000000 to make the firmware authoritative. Pass with -DLORA_BAND_HZ=...
#ifndef LORA_BAND_HZ
#define LORA_BAND_HZ 0
#endif

namespace lora {

namespace {

using namespace RNS;

// UART1, not UART0: UART0 is the ROM console on D0/D1 and is deliberately left
// free as the recovery path if a flash ever goes wrong.
constexpr uart_port_t UART_PORT = UART_NUM_1;

constexpr uint32_t MODULE_BAUD        = 115200;
constexpr int      RX_BUFFER_BYTES    = 1024;  // module bursts a whole +RCV line
constexpr int      TX_BUFFER_BYTES    = 512;
constexpr size_t   AT_LINE_MAX        = 320;   // "+RCV=65535,240,<240>,-120,9"
constexpr size_t   TX_QUEUE_DEPTH     = 6;
constexpr uint32_t PROBE_TIMEOUT_MS   = 400;
constexpr uint32_t COMMAND_TIMEOUT_MS = 1500;
constexpr uint32_t TX_TIMEOUT_MS      = 6000;  // one frame's airtime, generously

// Module identity and air parameters. Both ends must agree on the network id,
// band and parameters. Preamble is 12 because any network id other than 18
// restricts it to the default; SF7/125 kHz is a bench-range compromise that
// keeps a 240-character frame under a second.
constexpr uint16_t MODULE_ADDRESS = 1;
constexpr uint8_t  NETWORK_ID     = 6;
constexpr uint16_t PEER_ADDRESS   = 0;      // 0 = every module on the network
constexpr uint8_t  SPREADING      = 7;      // SF7
constexpr uint8_t  BANDWIDTH_IDX  = 7;      // 7 = 125 kHz
constexpr uint8_t  CODING_RATE    = 1;      // 1 = 4/5
constexpr uint8_t  PREAMBLE       = 12;
constexpr uint8_t  RF_POWER_DBM   = 14;     // CE ceiling; 22 is the module max

log_fn g_log = nullptr;
class LoRaInterface;              // defined below; the handle is store-wide
LoRaInterface* g_impl = nullptr;
Interface g_interface({Type::NONE});

uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

// A short sleep during start() only: the AT plumbing is blocking by design and
// never runs from loop(), so this cannot stall a live node.
void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

void logf(const char* fmt, ...) {
	if (g_log == nullptr) return;
	char line[192];
	va_list ap;
	va_start(ap, fmt);
	std::vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	g_log(line);
}

uint32_t bitrate_for(uint8_t sf, uint8_t bw_index, uint8_t cr_index) {
	const double bw_khz = (bw_index == 7) ? 125.0 : (bw_index == 8 ? 250.0 : 500.0);
	const double rate = (double)sf * (bw_khz * 1000.0 / std::pow(2.0, (double)sf)) *
	                    (4.0 / (double)(cr_index + 4));
	return (uint32_t)(rate + 0.5);
}

void trim_line(char* s) {
	if (s == nullptr) return;
	size_t len = std::strlen(s);
	while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == ' ' || s[len - 1] == '\t')) {
		s[--len] = '\0';
	}
	size_t lead = 0;
	while (s[lead] == ' ' || s[lead] == '\t') ++lead;
	if (lead > 0) std::memmove(s, s + lead, len - lead + 1);
}

class LoRaInterface : public InterfaceImpl {

public:
	LoRaInterface() : InterfaceImpl("RYLR998") {
		_IN      = true;
		_OUT     = true;
		_HW_MTU  = Type::Reticulum::MTU;   // full packets are carried, in frames
		_bitrate = bitrate_for(SPREADING, BANDWIDTH_IDX, CODING_RATE);
	}
	virtual ~LoRaInterface() { stop(); }

	virtual bool start() override;
	virtual void stop() override;
	virtual void loop() override;

	// InterfaceImpl keeps _online protected and exposes no accessor of its own;
	// the public getters live on the Interface wrapper.
	bool up() const { return _online; }

	bool status_line(char* out, size_t n) const;
	std::string short_status() const;
	void console_report() const;
	void set_verbose(bool on) { _verbose = on; }

private:
	virtual bool send_outgoing(const Bytes& data) override;

	// AT plumbing (blocking, used only from start()).
	bool read_line(char* out, size_t cap, uint32_t timeout_ms);
	bool command(const char* cmd, char* reply, size_t cap, uint32_t timeout_ms);
	bool query(const char* query, const char* primary, const char* alternate,
	           long& out, const char* label);
	bool apply(const char* query, const char* primary, const char* alternate,
	           long wanted, const char* setter_fmt, const char* label,
	           int* read_back = nullptr);

	// Runtime, called from loop().
	void drain();
	void dispatch(const char* line);
	void on_receive(const char* line);
	void pump_tx();
	uint8_t next_nonce() { return (uint8_t)(Cryptography::randomnum(256) & 0x0F); }

	const char* module_version() const { return _version[0] ? _version : "?"; }

	// Link state
	struct Outgoing {
		Bytes data;
		uint16_t address;
	};
	std::deque<Outgoing> _queue;         // packets waiting for airtime
	Bytes             _current;          // packet being framed
	uint16_t          _dest        = PEER_ADDRESS;  // address in flight
	size_t            _offset      = 0;  // bytes of _current already sent
	bool              _frame_active = false;
	loraframe::Reassembler _rx;
	uint8_t           _msg_id      = 0;
	bool              _awaiting_ok = false;
	uint32_t          _awaiting_since = 0;

	char              _line[AT_LINE_MAX] = {0};
	size_t            _line_len    = 0;
	bool              _discarding  = false;   // dropping an overlong line

	// Reported state
	char     _version[48] = {0};
	uint32_t _band_hz     = 0;
	// What the module reports back, as opposed to what we asked for. -1 means
	// the query was not understood, which is itself worth showing.
	int      _rb_address  = -1;
	int      _rb_network  = -1;
	int      _rb_sf       = -1;
	int      _rb_bw       = -1;
	int      _rb_cr       = -1;
	int      _rb_pre      = -1;
	int      _rb_power    = -1;
	uint32_t _frames_tx   = 0;
	uint32_t _frames_rx   = 0;
	uint32_t _packets_tx  = 0;
	uint32_t _packets_rx  = 0;
	uint32_t _errors      = 0;   // +ERR=-n replies and TX timeouts
	uint32_t _dropped     = 0;   // packets we could not queue or frame
	uint32_t _rx_invalid  = 0;   // unparseable receive lines
	uint32_t _overlong    = 0;   // receive lines longer than the buffer
	uint32_t _rx_early    = 0;   // packets heard while still configuring
	int      _last_rssi   = 0;
	int      _last_snr    = 0;

	/* What the last complete message looked like, whoever sent it.
	 *
	 * A frame that reassembles but is not a Reticulum packet is handed to the
	 * transport, which discards it without a word -- correct for the stack, but
	 * it leaves nothing behind to tell "heard it and rejected it" from "heard
	 * nothing at all". During bring-up that distinction is the whole question,
	 * so the last message is kept here and can be streamed with `verbose`. */
	uint32_t _last_rx_len  = 0;
	uint16_t _last_rx_from = 0;
	uint8_t  _last_rx_head[8] = {0};
	uint32_t _rx_complete  = 0;  // messages reassembled, valid packet or not
	bool     _verbose      = false;
};

// ---------------------------------------------------------------------------
// AT plumbing
// ---------------------------------------------------------------------------

bool LoRaInterface::read_line(char* out, size_t cap, uint32_t timeout_ms) {
	if (out == nullptr || cap == 0) return false;
	size_t n = 0;
	const uint32_t start = now_ms();
	while ((uint32_t)(now_ms() - start) < timeout_ms) {
		uint8_t c = 0;
		if (uart_read_bytes(UART_PORT, &c, 1, 0) != 1) {
			delay_ms(1);
			continue;
		}
		if (c == '\n') {
			out[n] = '\0';
			trim_line(out);
			return true;
		}
		if (c == '\r') continue;
		if (n + 1 < cap) out[n++] = (char)c;
		else n = 0;   // overlong: resynchronise on the next newline
	}
	out[0] = '\0';
	return false;
}

// One command, one response line. +OK is success, +ERR=n is failure, and any
// other "+X=..." line is the answer to a query. Receive lines that arrive in
// the middle are handed on rather than dropping a packet on the floor.
bool LoRaInterface::command(const char* cmd, char* reply, size_t cap, uint32_t timeout_ms) {
	const size_t cmd_len = std::strlen(cmd);
	uart_write_bytes(UART_PORT, cmd, cmd_len);
	uart_write_bytes(UART_PORT, "\r\n", 2);

	const uint32_t start = now_ms();
	while ((uint32_t)(now_ms() - start) < timeout_ms) {
		const uint32_t left = timeout_ms - (uint32_t)(now_ms() - start);
		if (!read_line(reply, cap, left)) return false;
		if (std::strncmp(reply, "+OK", 3) == 0) return true;
		if (std::strncmp(reply, "+ERR=", 5) == 0) {
			++_errors;
			return false;
		}
		if (std::strncmp(reply, "+RCV=", 5) == 0) { on_receive(reply); continue; }
		if (std::strncmp(reply, "AT", 2) == 0) continue;   // command echo
		if (reply[0] == '+') return true;                  // query answer
	}
	return false;
}

bool LoRaInterface::query(const char* q, const char* primary, const char* alternate,
                         long& out, const char* label) {
	char reply[AT_LINE_MAX];
	if (!command(q, reply, sizeof(reply), COMMAND_TIMEOUT_MS)) return false;

	// Reply spellings differ between firmware revisions: the guide documents
	// "+NETWORK=6" for AT+NETWORKID?, while the shipping V1.2.4 answers
	// "+NETWORKID=6". Accept either rather than silently failing to read a
	// value we then write blind.
	const char* prefix = nullptr;
	const size_t primary_len = std::strlen(primary);
	if (std::strncmp(reply, primary, primary_len) == 0) {
		prefix = primary;
	}
	else if (alternate != nullptr) {
		const size_t alt_len = std::strlen(alternate);
		if (std::strncmp(reply, alternate, alt_len) == 0) prefix = alternate;
	}

	if (prefix == nullptr) {
		// Printed once, at boot: an unparsed reply used to be invisible, which
		// hid a real firmware difference.
		logf("%s: unexpected answer to %s: \"%s\"", label, q, reply);
		return false;
	}

	out = std::strtol(reply + std::strlen(prefix), nullptr, 10);
	return true;
}

bool LoRaInterface::apply(const char* q, const char* primary, const char* alternate,
                          long wanted, const char* setter_fmt, const char* label,
                          int* read_back) {
	long current = 0;
	const bool known = query(q, primary, alternate, current, label);
	if (known && current == wanted) {
		if (read_back != nullptr) *read_back = (int)current;
		logf("%s %ld (unchanged)", label, current);
		return true;
	}

	char cmd[48];
	std::snprintf(cmd, sizeof(cmd), setter_fmt, wanted);
	char reply[AT_LINE_MAX];
	if (!command(cmd, reply, sizeof(reply), COMMAND_TIMEOUT_MS)) {
		logf("!! %s: %s was not accepted", label, cmd);
		if (read_back != nullptr) *read_back = known ? (int)current : -1;
		return false;
	}

	/* Read the value back after writing rather than reporting the pre-write one.
	 * The first version stored what the module held when it was asked, so the
	 * `lora` report said "address 0" while the module had just been set to 1 --
	 * the exact silent-mismatch this display exists to make visible. If the
	 * re-read fails, say so instead of inventing a value. */
	long now = 0;
	const bool confirmed = query(q, primary, alternate, now, label);
	if (read_back != nullptr) *read_back = confirmed ? (int)now : -1;

	if (!confirmed) {
		logf("%s set to %ld (read-back failed)", label, wanted);
		return true;
	}
	if (now == wanted) {
		logf("%s %ld -> %ld", label, known ? current : -1, now);
		return true;
	}
	logf("!! %s accepted %ld but holds %ld", label, wanted, now);
	return false;
}

// ---------------------------------------------------------------------------
// Start / stop
// ---------------------------------------------------------------------------

bool LoRaInterface::start() {
	_online = false;

#if LORA_NRST_PIN >= 0
	gpio_set_direction((gpio_num_t)LORA_NRST_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level((gpio_num_t)LORA_NRST_PIN, 1);
	delay_ms(10);
	gpio_set_level((gpio_num_t)LORA_NRST_PIN, 0);   // hold low >= 100 ms
	delay_ms(150);
	gpio_set_level((gpio_num_t)LORA_NRST_PIN, 1);
	delay_ms(350);                                  // boot, ends with +READY
#endif

	uart_config_t cfg = {};
	cfg.baud_rate  = MODULE_BAUD;
	cfg.data_bits  = UART_DATA_8_BITS;
	cfg.parity     = UART_PARITY_DISABLE;
	cfg.stop_bits  = UART_STOP_BITS_1;
	cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
	cfg.source_clk = UART_SCLK_DEFAULT;

	// A driver left installed by an earlier start() is reused rather than
	// reinstalled: uart_driver_install fails on a port it already owns.
	if (!uart_is_driver_installed(UART_PORT)) {
		if (uart_driver_install(UART_PORT, RX_BUFFER_BYTES, TX_BUFFER_BYTES,
		                        0, nullptr, 0) != ESP_OK) {
			logf("!! uart: driver install failed on port %d", (int)UART_PORT);
			return false;
		}
	}
	if (uart_param_config(UART_PORT, &cfg) != ESP_OK) {
		logf("!! uart: parameter configuration failed");
		return false;
	}
	if (uart_set_pin(UART_PORT, LORA_TX_GPIO, LORA_RX_GPIO,
	                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
		logf("!! uart: could not route GPIO %d/%d", LORA_TX_GPIO, LORA_RX_GPIO);
		return false;
	}
	uart_flush_input(UART_PORT);

	// The module needs a moment after power-up, so probe a few times instead of
	// declaring a cold boot a missing harness.
	char reply[AT_LINE_MAX];
	bool alive = false;
	for (int attempt = 0; attempt < 6 && !alive; ++attempt) {
		alive = command("AT", reply, sizeof(reply), PROBE_TIMEOUT_MS);
		if (!alive) delay_ms(200);
	}
	if (!alive) {
		logf("!! RYLR998 did not answer AT: check 3V3, GND, GPIO%d<-TXD, GPIO%d->RXD",
		     LORA_RX_GPIO, LORA_TX_GPIO);
		logf("   (and that the module is not asleep from an earlier AT+MODE=1)");
		return false;
	}

	// Informational; a module that answers AT but not AT+VER? is still usable.
	// Only a well-formed "+VER=" counts: a bare "+OK" or a timeout must not end
	// up reported as the module's firmware version.
	if (command("AT+VER?", reply, sizeof(reply), COMMAND_TIMEOUT_MS) &&
	    std::strncmp(reply, "+VER=", 5) == 0) {
		std::snprintf(_version, sizeof(_version), "%.42s", reply + 5);
	}

	command("AT+MODE=0", reply, sizeof(reply), COMMAND_TIMEOUT_MS);   // always awake

	apply("AT+ADDRESS?",   "+ADDRESS=",  nullptr,       MODULE_ADDRESS,
	      "AT+ADDRESS=%ld",   "address", &_rb_address);
	apply("AT+NETWORKID?", "+NETWORK=", "+NETWORKID=", NETWORK_ID,
	      "AT+NETWORKID=%ld", "network id", &_rb_network);

	long band = 0;
	const bool band_known = query("AT+BAND?", "+BAND=", nullptr, band, "band");
#if LORA_BAND_HZ != 0
	apply("AT+BAND?", "+BAND=", nullptr, LORA_BAND_HZ, "AT+BAND=%ld,M", "band Hz");
	band = LORA_BAND_HZ;
#endif
	_band_hz = band_known ? (uint32_t)band : (uint32_t)0;

	// The parameter reply is "+PARAMETER=<sf>,<bw>,<cr>,<preamble>".
	long sf = -1, bw = -1, cr = -1, pre = -1;
	const bool params_known =
	    command("AT+PARAMETER?", reply, sizeof(reply), COMMAND_TIMEOUT_MS) &&
	    std::strncmp(reply, "+PARAMETER=", 11) == 0 &&
	    std::sscanf(reply + 11, "%ld,%ld,%ld,%ld", &sf, &bw, &cr, &pre) == 4;

	if (!params_known || sf != SPREADING || bw != BANDWIDTH_IDX ||
	    cr != CODING_RATE || pre != PREAMBLE) {
		char cmd[48];
		std::snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%u,%u,%u,%u",
		              (unsigned)SPREADING, (unsigned)BANDWIDTH_IDX,
		              (unsigned)CODING_RATE, (unsigned)PREAMBLE);
		if (!command(cmd, reply, sizeof(reply), COMMAND_TIMEOUT_MS)) {
			logf("!! air parameters were not accepted");
		}
		else if (params_known) {
			logf("air parameters %ld,%ld,%ld,%ld -> %u,%u,%u,%u",
			     sf, bw, cr, pre, (unsigned)SPREADING, (unsigned)BANDWIDTH_IDX,
			     (unsigned)CODING_RATE, (unsigned)PREAMBLE);
		}
		/* Re-read so the report shows what the module now holds, not what it
		 * held before the write. */
		sf = bw = cr = pre = -1;
		if (command("AT+PARAMETER?", reply, sizeof(reply), COMMAND_TIMEOUT_MS) &&
		    std::strncmp(reply, "+PARAMETER=", 11) == 0)
			std::sscanf(reply + 11, "%ld,%ld,%ld,%ld", &sf, &bw, &cr, &pre);
	}
	_rb_sf = (int)sf; _rb_bw = (int)bw; _rb_cr = (int)cr; _rb_pre = (int)pre;

	apply("AT+CRFOP?", "+CRFOP=", nullptr, RF_POWER_DBM, "AT+CRFOP=%ld",
	      "power dBm", &_rb_power);

	_online = true;
	logf("RYLR998 %s on UART%d (GPIO%d/%d): addr %u net %u %lu.%03lu MHz sf%u bw%u",
	     module_version(), (int)UART_PORT, LORA_RX_GPIO, LORA_TX_GPIO,
	     (unsigned)MODULE_ADDRESS, (unsigned)NETWORK_ID,
	     (unsigned long)(_band_hz / 1000000UL),
	     (unsigned long)((_band_hz % 1000000UL) / 1000UL),
	     (unsigned)SPREADING, (unsigned)BANDWIDTH_IDX);
	return true;
}

void LoRaInterface::stop() {
	_online = false;
	if (!_queue.empty()) _queue.clear();
}

// ---------------------------------------------------------------------------
// Runtime
// ---------------------------------------------------------------------------

void LoRaInterface::loop() {
	if (!_online) return;
	drain();
	pump_tx();
	_rx.expire(now_ms());   // a lost frame must not swallow the next message
}

void LoRaInterface::drain() {
	for (;;) {
		uint8_t c = 0;
		if (uart_read_bytes(UART_PORT, &c, 1, 0) != 1) break;
		if (c == '\n') {
			// An overlong line is dropped whole: its tail would otherwise look
			// like a short, valid-looking line to the parser.
			if (!_discarding) {
				_line[_line_len] = '\0';
				trim_line(_line);
				dispatch(_line);
			}
			_line_len   = 0;
			_discarding = false;
		}
		else if (c != '\r') {
			if (_discarding) continue;
			if (_line_len + 1 < sizeof(_line)) {
				_line[_line_len++] = (char)c;
			}
			else {
				_line_len   = 0;
				_discarding = true;
				++_overlong;
			}
		}
	}
}

void LoRaInterface::dispatch(const char* line) {
	if (line[0] == '\0') return;

	if (std::strncmp(line, "+RCV=", 5) == 0) { on_receive(line); return; }
	if (std::strncmp(line, "+OK", 3) == 0)   { _awaiting_ok = false; return; }
	if (std::strncmp(line, "+ERR=", 5) == 0) {
		_awaiting_ok  = false;
		_frame_active = false;      // the peer cannot assemble the rest
		++_errors;
		logf("RYLR998: send error %s", line + 5);
		return;
	}
	if (std::strncmp(line, "AT", 2) == 0) return;   // command echo

	logf("RYLR998: unhandled line \"%s\"", line);
}

void LoRaInterface::on_receive(const char* line) {
	loraframe::Rcv rcv;
	if (!loraframe::parseRcv(line, rcv)) {
		++_rx_invalid;
		logf("RYLR998: unparseable receive line \"%s\"", line);
		return;
	}
	++_frames_rx;
	_last_rssi = rcv.rssi;
	_last_snr  = rcv.snr;

	// Nothing is running yet during configuration; the packet is already lost
	// on the air, so count it rather than hand it to an unstarted transport.
	if (!_online) { ++_rx_early; return; }

	const uint8_t header = rcv.frame[0];
	const loraframe::Reassembler::Result result =
	    _rx.push(rcv.frame + 1, rcv.frame_len - 1,
	             loraframe::headerId(header), loraframe::headerMore(header), now_ms());

	if (result == loraframe::Reassembler::COMPLETE) {
		++_rx_complete;
		_last_rx_len  = _rx.size();
		_last_rx_from = rcv.from;
		for (size_t i = 0; i < sizeof(_last_rx_head) && i < _rx.size(); ++i)
			_last_rx_head[i] = _rx.data()[i];

		/* A frame that reassembles but is not a Reticulum packet is dropped by
		 * the transport without a word. That is right for the stack and useless
		 * for bring-up, where "heard it and rejected it" and "heard nothing"
		 * look identical from outside. */
		if (_verbose)
			logf("rx %u bytes from %u, rssi %d snr %d, first %02x %02x %02x %02x",
			     (unsigned)_rx.size(), (unsigned)rcv.from, rcv.rssi, rcv.snr,
			     _rx.size() > 0 ? _rx.data()[0] : 0,
			     _rx.size() > 1 ? _rx.data()[1] : 0,
			     _rx.size() > 2 ? _rx.data()[2] : 0,
			     _rx.size() > 3 ? _rx.data()[3] : 0);

		// Signal quality rides along on the packet the transport builds. Those
		// fields are private to InterfaceImpl, so they are reached through the
		// wrapper's setters -- the same wrapping handle_incoming() does itself.
		std::shared_ptr<InterfaceImpl> self = shared_from_this();
		Interface wrapper(self);
		wrapper.r_stat_rssi((float)rcv.rssi);
		wrapper.r_stat_snr((float)rcv.snr);
		++_packets_rx;
		handle_incoming(Bytes(_rx.data(), _rx.size()));
		_rx.clear();
	}
	else if (result == loraframe::Reassembler::OVERFLOW) {
		logf("RYLR998: message larger than one MTU, dropped");
	}
}

void LoRaInterface::pump_tx() {
	if (_awaiting_ok) {
		if ((uint32_t)(now_ms() - _awaiting_since) < TX_TIMEOUT_MS) return;
		_awaiting_ok  = false;
		_frame_active = false;
		++_errors;
		logf("RYLR998: no +OK after send, packet dropped");
	}

	if (!_frame_active) {
		if (_queue.empty()) return;
		_current    = _queue.front().data;
		_dest       = _queue.front().address;
		_queue.pop_front();
		if (_current.size() == 0 || _current.size() > loraframe::MAX_MESSAGE) {
			++_dropped;
			return;
		}
		_offset       = 0;
		_frame_active = true;
		_msg_id       = (uint8_t)((_msg_id + 1) & loraframe::HDR_ID_MASK);
	}

	const size_t remaining = _current.size() - _offset;
	const size_t chunk = remaining > loraframe::MAX_CHUNK ? loraframe::MAX_CHUNK : remaining;
	const bool   more  = remaining > chunk;

	char body[loraframe::AT_DATA_MAX + 1];
	const size_t body_len = loraframe::encodeFrame(next_nonce(), more, _msg_id,
	                                               _current.data() + _offset, chunk,
	                                               body, sizeof(body));
	if (body_len == 0) {
		++_dropped;
		_frame_active = false;
		return;
	}

	char cmd[32 + loraframe::AT_DATA_MAX];
	const int n = std::snprintf(cmd, sizeof(cmd), "AT+SEND=%u,%u,%s\r\n",
	                            (unsigned)_dest, (unsigned)body_len, body);
	if (n <= 0 || (size_t)n >= sizeof(cmd)) {
		++_dropped;
		_frame_active = false;
		return;
	}

	uart_write_bytes(UART_PORT, cmd, (size_t)n);
	_offset += chunk;
	if (!more) _frame_active = false;
	++_frames_tx;
	_awaiting_ok    = true;
	_awaiting_since = now_ms();
}

bool LoRaInterface::send_outgoing(const Bytes& data) {
	if (!_online) return false;
	if (data.size() == 0 || data.size() > loraframe::MAX_MESSAGE) {
		++_dropped;
		return false;
	}
	if (_queue.size() >= TX_QUEUE_DEPTH) {
		// LoRa is slower than the stack can produce: drop rather than grow
		// without bound. Lost packets are what the link layer is for.
		++_dropped;
		logf("RYLR998: queue full, dropping %u bytes", (unsigned)data.size());
		return false;
	}

	_queue.push_back(Outgoing{data, PEER_ADDRESS});
	++_packets_tx;
	InterfaceImpl::handle_outgoing(data);
	return true;
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

bool LoRaInterface::status_line(char* out, size_t n) const {
	if (out == nullptr || n == 0) return false;
	if (!_online) {
		std::snprintf(out, n, "down (RYLR998 not responding on UART%d)", (int)UART_PORT);
		return true;
	}
	std::snprintf(out, n,
	              "up addr%u net%u %.3fMHz sf%u/bw%u tx%lu rx%lu q%u err%lu rssi%d snr%d",
	              (unsigned)MODULE_ADDRESS, (unsigned)NETWORK_ID,
	              (double)_band_hz / 1000000.0,
	              (unsigned)SPREADING, (unsigned)BANDWIDTH_IDX,
	              (unsigned long)_packets_tx, (unsigned long)_packets_rx,
	              (unsigned)_queue.size(), (unsigned long)_errors,
	              _last_rssi, _last_snr);
	return true;
}

std::string LoRaInterface::short_status() const {
	if (!_online) return "lora down";
	char b[64];
	std::snprintf(b, sizeof(b), "lora up tx%lu rx%lu rssi%d snr%d",
	              (unsigned long)_packets_tx, (unsigned long)_packets_rx,
	              _last_rssi, _last_snr);
	return std::string(b);
}

void LoRaInterface::console_report() const {
	if (!_online) {
		logf("LoRa: down (RYLR998 not responding on UART%d GPIO%d/%d)",
		     (int)UART_PORT, LORA_RX_GPIO, LORA_TX_GPIO);
		return;
	}
	// What the module holds is shown beside what was asked for, because a
	// mismatch with the far end is silent on the air and obvious here.
	logf("LoRa: %s on UART%d GPIO%d(RX)/%d(TX)", module_version(),
	     (int)UART_PORT, LORA_RX_GPIO, LORA_TX_GPIO);
	logf("  band %.3f MHz   address %d   network %d   power %d dBm",
	     (double)_band_hz / 1000000.0, _rb_address, _rb_network, _rb_power);
	logf("  air sf%d bw%d cr%d preamble %d (module reports %d,%d,%d,%d)",
	     (int)SPREADING, (int)BANDWIDTH_IDX, (int)CODING_RATE, (int)PREAMBLE,
	     _rb_sf, _rb_bw, _rb_cr, _rb_pre);
	logf("  packets tx %lu rx %lu   frames tx %lu rx %lu",
	     (unsigned long)_packets_tx, (unsigned long)_packets_rx,
	     (unsigned long)_frames_tx, (unsigned long)_frames_rx);
	logf("  queued %u   errors %lu   dropped %lu",
	     (unsigned)_queue.size(), (unsigned long)_errors, (unsigned long)_dropped);
	logf("  last rssi %d dBm   snr %d dB", _last_rssi, _last_snr);
	logf("  messages reassembled %lu   last %lu bytes from %u",
	     (unsigned long)_rx_complete, (unsigned long)_last_rx_len,
	     (unsigned)_last_rx_from);
	if (_last_rx_len)
		logf("  last head %02x %02x %02x %02x %02x %02x %02x %02x",
		     _last_rx_head[0], _last_rx_head[1], _last_rx_head[2], _last_rx_head[3],
		     _last_rx_head[4], _last_rx_head[5], _last_rx_head[6], _last_rx_head[7]);
	logf("  verbose %s", _verbose ? "on" : "off");
	logf("  bad lines %lu   overlong %lu   heard-while-configuring %lu",
	     (unsigned long)_rx_invalid, (unsigned long)_overlong,
	     (unsigned long)_rx_early);
}

}  // namespace

// ---------------------------------------------------------------------------
// Public surface
// ---------------------------------------------------------------------------

void set_logger(log_fn fn) { g_log = fn; }

bool begin() {
	if (g_impl != nullptr) return g_impl->up();

	g_impl = new LoRaInterface();
	g_interface = g_impl;   // takes ownership of the impl
	g_interface.mode(RNS::Type::Interface::MODE_GATEWAY);

	if (!g_impl->start()) {
		// Deliberately not registered: an interface that cannot transmit has no
		// business in the path table. WiFi still carries the node.
		return false;
	}

	RNS::Transport::register_interface(g_interface);
	return true;
}

bool online() { return g_impl != nullptr && g_impl->up(); }

bool status_line(char* out, size_t n) {
	if (g_impl == nullptr) {
		if (out != nullptr && n > 0) std::snprintf(out, n, "not started");
		return false;
	}
	return g_impl->status_line(out, n);
}

std::string short_status() {
	if (g_impl == nullptr) return "lora off";
	return g_impl->short_status();
}

void console_report() {
	if (g_impl == nullptr) {
		logf("LoRa: not started");
		return;
	}
	g_impl->console_report();
}

void set_verbose(bool on) {
	if (g_impl != nullptr) g_impl->set_verbose(on);
}

}  // namespace lora
