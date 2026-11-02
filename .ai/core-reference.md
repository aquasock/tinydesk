# TINYNODE TECHNICAL REFERENCE

> **Project:** TinyNode
> **Purpose:** The authoritative lookup for externally defined facts this project depends on: the board's wiring and fitted parts, the ESP32-S3's memory and radio limits, the FFat/NVS layout, the interface contracts of the projects we vendor, the RYLR998 AT surface and the LoRa wire framing the far end must implement, the sensors on the I2C bus, and the behaviour of the toolchain.
> **Authority:** Per `.ai/core.md`, consult this file before looking anything up online, and open the record behind any hardware fact before acting on it. A record is authoritative to the extent of its `status` and cited `sources`. When a better or newer source is found online, add or revise a record and notify the user.

This file records what an outside source (a datasheet, vendor manual, external repository, or measurement on this bench) says, plus observations that confirm or refute it. It does not record project history, build results, or discussion; those belong in `core-log.md`. It does not record project design choices unless an external fact forces them.

The schema adapts the Tang family's reference format to a board that has no FPGA and no companion MCU: the ESP32-S3 is both. `DEVICE` therefore names a fitted peripheral rather than silicon, and `SOC` carries the SoC's own limits.

---

## 1. AI operating rules

```yaml
schema_version: 1

record_kinds:
  - BOARD      # the Arduino Nano ESP32 board as wired: pins, harnesses, power, bootloader
  - SOC        # ESP32-S3: memory, radio band, partitions as fitted
  - FLASH      # FFat and NVS layout, mount options, what persists where
  - PROTOCOL   # wire contracts: the RYLR998 AT surface, TinyNode's LoRa framing
  - DEVICE     # a fitted part: DS3231, LIS3MDL, SHT45, PA1616D, RYLR998
  - EXTERNAL   # behaviour owned by microReticulum, microLXMF, MsgPack, RNS, Sideband
  - TOOLCHAIN  # PlatformIO, ESP32 Arduino core, host tests, host-side RNS tooling

statuses:
  VERIFIED:   "Primary source cited AND confirmed on this project's hardware or tools."
  SOURCED:    "Primary source cited; not yet confirmed on this project's hardware or tools."
  INFERRED:   "Derived from secondary sources, reverse inspection, or consistent observation; no primary source. Flag before relying on it."

rules:
  - "One fact or one tightly coupled fact group per record."
  - "Every record cites at least one source with enough detail to re-check it: repository path plus symbol, file and line, register name, datasheet revision, or tool version."
  - "Separate what the source says (statement) from what it means for this project (consequence)."
  - "Diagnostic or implementation limits chosen by this project are never recorded as standard or device limits. A frame ceiling this firmware chose is not an AT limit."
  - "A record holds what is true now. When a fact changes or proves wrong, edit the record in place: revise statement and consequence, update verified_date and sources, and end verification with 'Revised <date> (core-log entry N): <what changed>'. Git and core-log.md keep the history; this file does not."
  - "Before acting on a record, check that the code paths and symbols it cites still exist; fix the record in the same cycle if they do not."
  - "Never reuse a record_id. When a record is folded into another, or split, move its id to the retired-id table in section 8 with the record that now holds its facts, after checking that every fact still true has been carried across."
  - "An INFERRED record must say how it was inferred, in verification."
  - "A negative result is a fact. Record what the hardware cannot do with the same rigour as what it can. An open defect may be recorded as its established facts and ruled-out causes, with any lead labelled as a lead."
  - "When an online lookup was needed because this file had no answer, add the finding here with the existing syntax, then notify the user if the source is newer or more valid than a record already here."
  - "Use technical names (ESP32-S3, RYLR998, DS3231, microReticulum). The user's conversational names are not used in this file."
  - "A fact that is only true of this bench (a USB serial number, an address on the LAB network) must be labelled as bench state, not as device behaviour."
```

---

## 2. Topic catalog

Topic IDs are the `record_id` prefix. An entry reserves a name; it does not claim coverage.

```yaml
- topic_id: BRD
  name: "Arduino Nano ESP32 as wired"
  description: "The board's harness map, the bench identity it presents to the host, and its DFU recovery path."

- topic_id: SOC
  name: "ESP32-S3"
  description: "Memory, flash and radio band as fitted to this board: what the stack can and cannot use."

- topic_id: FLS
  name: "FFat, NVS and what persists where"
  description: "The factory partition table, the FFat mount options this firmware needs, the identity/path/message stores, and the clock dependency that keeps them."

- topic_id: NET
  name: "Reticulum over WiFi/UDP"
  description: "The UDP interface contract, the gateway mode this node runs in, and the client-side consequences of speaking plain UDP rather than AutoInterface."

- topic_id: RNS
  name: "microReticulum (vendored)"
  description: "The vendored C++ RNS port: version, the local patch, the MTU, and the persistence semantics its flush and clock behaviour impose."

- topic_id: LXMF
  name: "microLXMF (vendored)"
  description: "The vendored LXMF router: version and licence, the constructor contract, where persistence is and is not wired, and the content limits replies must fit."

- topic_id: MSG
  name: "MsgPack (vendored)"
  description: "The vendored MsgPack 0.4.2 copy and the two members microLXMF needs to be public."

- topic_id: RADIO
  name: "REYAX RYLR998 LoRa module"
  description: "The UART AT module fitted on Serial2: identity, the settings both ends must agree on, its AT surface, and its electrical limits."

- topic_id: FRM
  name: "TinyNode LoRa framing"
  description: "The base64 frame format, header byte, multi-frame reassembly and timing that the far end (the Tang) must implement."

- topic_id: GPS
  name: "Ultimate GPS (PA1616D) and PPS"
  description: "The NMEA receiver on Serial1, its role as master clock, and the pulse-per-second input on D4."

- topic_id: SEN
  name: "I2C sensors and RTC"
  description: "The three devices sharing A4/A5: the DS3231 clock, the LIS3MDL magnetometer, and the SHT45 temperature/humidity sensor."

- topic_id: HOST
  name: "Host-side tooling"
  description: "The RNS instance in host-config, the Python test scripts, and the two virtualenvs that own the host toolchain."

- topic_id: TOOL
  name: "PlatformIO build"
  description: "The build environment and the flags and dependencies that change what the firmware can do."

- topic_id: PROV
  name: "Provenance and attribution"
  description: "The upstream commits vendored here, the change each carries, and the licence obligations that follow."
```

---

## 3. Active routing

| Question | Consult first | Fast records |
|---|---|---|
| Which port is the node, and which one is the Tang? | BRD | BRD-001 |
| Which pins does each harness use? | BRD | BRD-002 |
| Why did the firmware appear on the wrong pin? | BRD | BRD-003 |
| Can the board be reflashed without the button? | BRD | BRD-004 |
| How much RAM, flash and PSRAM is there? | SOC | SOC-001 |
| Can the node join a 5 GHz network? | SOC | SOC-002 |
| Where does the filesystem live, and how big is it? | FLS | FLS-001 |
| Why did opening a file fail with too many handles? | FLS | FLS-002, FLS-003 |
| Why did the message log fail to open for append? | FLS | FLS-003 |
| Does clearing WiFi settings lose the node's address? | FLS | FLS-004 |
| Why was the path table empty after a reboot? | FLS | FLS-005 |
| Where are received messages kept, and how do they rotate? | FLS | FLS-006 |
| Why was nothing ever written to flash? | FLS | FLS-007 |
| What port does the node speak, and to whom does it forward? | NET | NET-001 |
| Does the node relay between LAN and radio? | NET | NET-002 |
| Why can't a stock Reticulum client see the node? | NET | NET-003 |
| Why does a second local client fail to bind? | NET | NET-004 |
| What breaks if the static IP moves? | NET | NET-005 |
| Which microReticulum version is vendored, and what was patched? | RNS | RNS-001 |
| What is the packet size limit? | RNS | RNS-002 |
| Why did learned paths vanish on reboot? | RNS | RNS-003 |
| Why do store timestamps need a real clock? | RNS | RNS-004 |
| Does path persistence actually work? | RNS | RNS-005 |
| What are the `has_path=0 hops=128` entries? | RNS | RNS-006 |
| Which microLXMF version, and under what licence? | LXMF | LXMF-001 |
| Why did the first announce have no display name? | LXMF | LXMF-002 |
| Why is nothing persisted by the library? | LXMF | LXMF-003 |
| How long may a reply be over LoRa? | LXMF | LXMF-004 |
| Is a delivery proof an application ack? | LXMF | LXMF-005 |
| What destinations does the node announce? | LXMF | LXMF-006 |
| How do I report a sensor to a client so it is machine-readable? | LXMF | LXMF-008 |
| What field carries a request, and with what command ids? | LXMF | LXMF-009 |
| Why does a client reject my FIELD_TELEMETRY as "not bytes"? | LXMF | LXMF-010 |
| How big can a telemetry reply be, and what does 159 bytes mean? | LXMF | LXMF-011 |
| Does a large telemetry reply actually arrive? | LXMF | LXMF-012 |
| How is a heading reported, when no sensor id exists for one? | LXMF | LXMF-013 |
| Where does a node describe itself to the client? | LXMF | LXMF-014 |
| How does a node set its icon and colours in the client? | LXMF | LXMF-015 |
| Why does the Reticulum Transport sensor not appear at all? | LXMF | LXMF-016 |
| Why does Identity::recall() fail for a peer that just announced? | RNS | RNS-007 |
| Why does the path count read as a huge number? | RNS | RNS-008 |
| Where can a message carry custom data without colliding? | LXMF | LXMF-007 |
| Why does the ESP32 build fail against stock MsgPack? | MSG | MSG-001 |
| What is the LoRa module, and how is it driven? | RADIO | RADIO-001 |
| Which settings must both ends agree on? | RADIO | RADIO-002 |
| Which AT commands does the driver use? | RADIO | RADIO-003 |
| Which network-id reply form does V1.2.4 use? | RADIO | RADIO-004 |
| Why are settings read before they are written? | RADIO | RADIO-005 |
| Does the module filter traffic by address? | RADIO | RADIO-006 |
| Why do packets go missing during transmit? | RADIO | RADIO-007 |
| Can the module run at 5 V, or at 22 dBm? | RADIO | RADIO-008 |
| Is LoRa transmit actually proven on hardware? | RADIO | RADIO-009 |
| What exactly is sent over the air? | FRM | FRM-001, FRM-002, FRM-003 |
| How many frames is a full packet, and when is a partial dropped? | FRM | FRM-004 |
| How slow is the link? | FRM | FRM-005 |
| What happens on an AT error? | FRM | FRM-006 |
| Is the UART or the radio the bottleneck? | FRM | FRM-007 |
| Which bus does the GPS use, and at what rate? | GPS | GPS-005 |
| Where does time come from? | GPS | GPS-002 |
| What does `pps` measure, and what does it show when unwired? | GPS | GPS-003 |
| How accurate is the NMEA timestamp? | GPS | GPS-004 |
| What is at each I2C address? | SEN | SEN-001, SEN-002, SEN-003, SEN-006 |
| Which device is the pressure sensor? | SEN | SEN-006 |
| Why is there no chip id for the humidity sensor? | SEN | SEN-007 |
| Why is the RTC powered from 3V3 and not VBUS? | SEN | SEN-001 |
| Why is a sensor reading cached rather than read on demand? | SEN | SEN-004 |
| Why is heading wrong after moving the node? | SEN | SEN-005 |
| Why does |B| still vary after calibrating? | SEN | SEN-008 |
| Why does |B| read high when the dip is right? | SEN | SEN-010 |
| How much does the bench distort the field? | SEN | SEN-011 |
| Can a heading come from the magnetometer alone? | SEN | SEN-012 |
| Why does a 6-byte sensor read return one repeated value? | SEN | SEN-009 |
| Why did a host-side calibration check say the board never moved? | HOST | HOST-008 |
| How is a client configured to see the node? | HOST | HOST-001, HOST-003 |
| How is delivery proven end to end? | HOST | HOST-002 |
| Which Python environment runs RNS or PlatformIO? | HOST | HOST-004 |
| Do the sensor and framing units test without hardware? | HOST | HOST-005 |
| Does Sideband work against this node? | HOST | HOST-006 |
| Why did a local client's inbound messages vanish with no error? | HOST | HOST-007 |
| Can two programs hold one identity at once? | HOST | HOST-007 |
| What does the build environment pin? | TOOL | TOOL-001 |
| Which build flags change behaviour? | TOOL | TOOL-002 |
| What code is vendored into this project, and under what licence? | PROV | PROV-001, PROV-003 |
| What licence is the combined firmware under? | PROV | PROV-002 |
| Which pins drive the relays, and what does `relay 0000` set? | BRD | BRD-005 |
| Which input level closes the fitted relay module? | BRD | BRD-005 |

---

## 4. Fast lookup index

One line per live record, in the order of section 5. Retired ids are in section 8.

```yaml
BRD-001: "The node is the board with USB serial 206EF131675C and IP 10.10.0.35 on the LAB network; its /dev/ttyACM* name is unstable bench state -- /dev/ttyACM1 on 2026-10-26 with the TinyTang on /dev/ttyACM0, /dev/ttyACM0 for the node in core-log entries 4, 5 and 10, and no ttyACM device at all on 2026-11-01 -- so pio upload must name and confirm its port every session"
BRD-002: "Harnesses: GPS on Serial1 D2 (RX) and D3 (TX), PPS on D4 (GPIO 7), LoRa on Serial2 D5 (RX) and D6 (TX) = GPIO 8 and 9, I2C on A4 (SDA) and A5 (SCL), the RGB LED active-low on the built-in part, one reset button on PB1, and UART0 on D0/D1 left free as the ROM console"
BRD-003: "HardwareSerial::begin() and attachInterrupt() are remapped through digitalPinToGPIONumber() (BOARD_HAS_PIN_REMAP), so the arguments are logical Arduino pins D2/D3/D4/D5/D6, not GPIO numbers; passing a GPIO number silently selects a different physical pin"
BRD-004: "No button press is needed to flash: the board's tinyuf2 bootloader accepts DFU directly over USB; bootloader mode is otherwise a double-tap inside the ~0.7 s RGB pulse, and that path is app-only (ALT_COUNT 1), so it cannot write a partition table"
BRD-005: "Four relay channels on D9..D12, which is also the unused SPI pair; the fitted module closes a relay on a HIGH input (ACTIVE_LOW false), the pins stay INPUT at boot until the first command drives them, and the digits follow the spliced cable rather than the header -- 1 = D12 VBUS, 2 = D11 D-, 3 = D10 D+, 4 = D9 GND; `status` reports the same digits"
SOC-001: "ESP32-S3 with 16 MB flash, 320 KB DRAM and 8 MB Octal-SPI PSRAM on a NORA-W106-10B module, per the Arduino Nano ESP32 datasheet; the PlatformIO build reports RAM and flash as percentages of 320 KB and the flash part"
SOC-002: "The ESP32-S3 radio is 2.4 GHz only; the 5 GHz SSID cheese (ch 161) can never be joined, and yogurt is the same access point's 2.4 GHz radio (BSSIDs ...58:F2 versus ...58:F3), which is what the node uses"
FLS-001: "The factory partition table already carries a 9 MB FAT partition named ffat at 0x610000, so no partition write is needed; the build uses app3M_spiffs9M_fact512k_16MB.csv and mounts the volume at /ffat"
FLS-002: "FFat.begin(true, \"/ffat\", 24) raises maxOpenFiles to 24 because the Arduino default of 10 is consumed by RNS's stores: path_store index and segment, known_store, and three hashlist_store handles"
FLS-003: "/ffat/messages.log is appended with POSIX fopen(path, \"a\") rather than FFat.open(..., FILE_APPEND), because the Arduino FS layer is subject to maxOpenFiles and failed with '!! could not open message log for append'"
FLS-004: "WiFi SSID, password, IP configuration and display name live in NVS; the identity, learned paths and message log live on FFat; a factory reset clears NVS and reboots into the wizard while keeping /ffat/identity, so the node's address survives"
FLS-005: "microStore::sweep() runs at every store init and drops any record whose timestamp is ahead of the current clock, so without a real clock every record written late in a session looked like the future and the whole table was discarded; the firmware calls OS::setTimeOffset() from the DS3231 before the stores initialise"
FLS-006: "Received LXMF messages are appended to /ffat/messages.log as JSON Lines with rx, rtc, from, sent, title and content, rotating at 512 KB to /ffat/messages.1.log with one generation kept; verified across a reboot with one 152-byte entry"
FLS-007: "microStore passes paths to the POSIX adapter verbatim, so a PrefixedFileSystem prefixes them with /ffat; -DRNS_USE_FS is required or write_path_table() is compiled out, and -DUSTORE_USE_POSIXFS selects the adapter -- with either missing nothing is ever written"
NET-001: "The node speaks IPv4 UDP on port 4242 and broadcasts to 255.255.255.255:4242; DEFAULT_UDP_PORT is 4242 in UDPInterface.h and the port is compile-time only, so a different port per user needs a firmware patch"
NET-002: "Both interfaces are registered with mode MODE_GATEWAY and the stack runs transport_enabled(true), so the board answers path requests and relays between the LAN and the radio"
NET-003: "A stock Reticulum install will not see this node: the default AutoInterface is IPv6 link-local multicast, which the node does not use; the client needs an explicit UDPInterface entry with listen_port 4242"
NET-004: "Only one process may own UDP 4242 at a time; with share_instance = Yes behind one rnsd every other local client attaches to that instance, and rnsd must be started with setsid or it dies with the calling shell"
NET-005: "A static IP means the board does no DHCP, so it will not notice an address conflict and the address must be picked outside the router's pool; if the subnet changes the board needs a reflash"
RNS-001: "microReticulum 0.5.0 at commit 40fa628, Apache-2.0, vendored with exactly one local change: examples/common/udp_interface/UDPInterface.cpp takes WiFi credentials from wifi_config.h or -DRNS_WIFI_SSID instead of hardcoding them"
RNS-002: "Type::Reticulum::MTU is 500 in this port (the header comments that upstream RNS defaults to 507); MDU is MTU minus HEADER_MAXSIZE and IFAC_MIN_SIZE, and link MTU discovery is enabled"
RNS-003: "RNS flushes its stores only from exit_handler(), which never runs on a headless node, so the firmware calls Transport::persist_data() every 60 s and on demand from the p console command"
RNS-004: "OS::setTimeOffset() sets the wall clock every store timestamp is derived from, and must run after the Reticulum instance is constructed but before start() initialises the stores"
RNS-005: "Path persistence is verified end to end: learn a path from a host announce, flush with p, reboot, and the entry returns with has_path=1 hops=1 while nothing is announcing, so flash is the only possible source"
RNS-006: "Records written during the no-RTC era are resurrected by the index rebuild and appear in the d listing with has_path=0 hops=128; RNS refuses to route with them and they age out on their own"
RNS-007: "On another bench the file-backed known-destination store did not retain, so Identity::recall() returned nothing for an identity that had just announced; RNS_PERSIST_KNOWN_DESTINATIONS=0 selects the heap store instead. This project leaves that default at 1, so it sits on the same path and owes the test"
RNS-008: "Transport::path_table() is not the live path store: inserts and has_path() use new_path_table(), so a status line counting path_table() reads zero; use paths_added() for paths learned, and note that size() on the path store is a record count, not bytes"
LXMF-001: "microLXMF 0.1.0 at commit 9876dff, GPL-3.0-only and unmodified; linking it into the firmware makes the combined image GPL-3.0"
LXMF-002: "LXMRouter(identity, storage_path, announce_at_start) defaults announce_at_start to false, and the router must be announced after set_display_name() or the first announce carries empty app-data and clients show no name"
LXMF-003: "MessageStore exists in the library but is never referenced by LXMRouter.cpp, which notes persistence to it as a follow-up; nothing is persisted by the library, so the firmware writes /ffat/messages.log itself"
LXMF-004: "A reply must fit LORA_LINK_PACKET_MAX_CONTENT, 63 bytes of content (175 minus 112) in one link packet; longer content moves to Resource transfer, so every command reply except status is kept under 63 bytes"
LXMF-005: "A delivery proof is returned by the receiving stack once the message is accepted and validated, not when an application acts on it; that is what a client shows as Delivered, and an application ack would only double traffic"
LXMF-006: "The node announces the destination nano/esp32 and the LXMF delivery destination lxmf.delivery; the display name is the one set in the wizard or 'Nano ESP32 ' plus the last four hex digits of the identity"
LXMF-007: "Message fields are a numbered registry: TELEMETRY 0x02, TELEMETRY_STREAM 0x03, COMMANDS 0x09, RESULTS 0x0A, RENDERER 0x0F; 0x00-0x80 reserved for extension and 0xFB-0xFD the escape hatch for non-native data"
LXMF-008: "FIELD_TELEMETRY is a msgpack map of Sideband sensor ids: TIME 0x01, LOCATION 0x02, PRESSURE 0x03, TEMPERATURE 0x07, HUMIDITY 0x08, MAGNETIC_FIELD 0x09, plus RAM 0x14, NVM 0x15 and RNS_TRANSPORT 0x19; FIELD_TELEMETRY_STREAM is a list of them"
LXMF-009: "FIELD_COMMANDS is a list of single-key dicts using Sideband's command ids: PLUGIN 0x00, TELEMETRY_REQUEST 0x01, PING 0x02, ECHO 0x03, SIGNAL_REPORT 0x04; the answer goes in FIELD_TELEMETRY or FIELD_RESULTS with the content left empty"
LXMF-010: "A message field value must itself be msgpack-encoded bytes: microLXMF re-emits fields with packRawBytes, so a bare map in the field is parsed by the outer msgpack into a dict and the client rejects it with 'packed data must be type bytes'"
LXMF-011: "159 bytes is the threshold for sending a message as one link-free OPPORTUNISTIC packet, not a size limit: above it LXMF uses DIRECT and a RESOURCE transfer, so any telemetry map arrives, just slower; 114 bytes are committed before sensors, leaving a 45-byte map in which TIME costs 6, LOCATION 41, MAGNETIC_FIELD 29 and the environment trio 10 each"
LXMF-012: "Link delivery works to Sideband: a 614-byte LOCATION+MAGNETIC_FIELD reply arrived over a DIRECT resource transfer, parsed and was committed to Sideband's database; Sideband's telemetry panel then needed a restart to repaint, which is a client display artefact, not a delivery fault"
LXMF-013: "Sideband has no sensor id for heading, and the two slots that look like candidates refuse it: MAGNETIC_FIELD unpacks exactly three floats and drops a fourth, and LOCATION's `bearing` is only rendered when speed exceeds 0.1 km/h, so a stationary node never shows it; CUSTOM (0xFF) carries [label, [value, icon]] entries and renders each as its own row regardless of motion"
LXMF-014: "SID_INFORMATION (0x0F) is a single plain msgpack string that the client shows as one copyable row under the name Information, so it is the place for firmware version, uptime and address in one line"
LXMF-015: "FIELD_ICON_APPEARANCE (0x04) is [icon name, fg, bg] with the two colours as three raw bytes each rather than hex strings, costs about 26 bytes per message and is honoured from untrusted peers by default, so it must not be attached to the compact telemetry reply whose whole purpose is staying inside one 151-byte packet"
LXMF-016: "Sideband's RNSTransport.render() reads the interface list at d['ifstats']['interfaces'] and needs d['path_table'], d['link_count'] and d['interface_count'] at the top level, and any one missing raises KeyError that discards the entire sensor"
MSG-001: "MsgPack 0.4.2 (MIT) is vendored with Packer::packRawBytes (both overloads) and Unpacker::indices made public, because microLXMF's LXMessage.cpp uses them and 0.4.2 keeps them private while upstream CI builds that path only for native targets"
RADIO-001: "The radio is a REYAX RYLR998, an 868/915 MHz module driven entirely by AT commands on a UART with no SPI path and no RadioLib; the module on this bench reports RYLR998_REYAX_V1.2.4"
RADIO-002: "Configured values: address 1, network id 6, peer address 0 (broadcast), SF7, bandwidth index 7 (125 kHz), coding rate 1 (4/5), preamble 12, 14 dBm, UART 115200 8N1; the band is left at whatever the module holds (factory 915 MHz) unless -DLORA_BAND_HZ is set"
RADIO-003: "AT surface used by the driver: AT+VER?, AT+MODE=0, AT+ADDRESS? and AT+ADDRESS=, AT+NETWORKID? and AT+NETWORKID=, AT+BAND? and AT+BAND=<hz>,M, AT+PARAMETER? and AT+PARAMETER=<sf>,<bw>,<cr>,<pre>, AT+CRFOP? and AT+CRFOP=<dbm>, plus AT+SEND=, with +RCV= arriving unsolicited"
RADIO-004: "The AT guide documents AT+NETWORKID? answering +NETWORK=<id> while V1.2.4 answers +NETWORKID=<id>; the driver accepts both and prints the raw reply when neither matches, because a silent parse failure there once left the network id written blind"
RADIO-005: "Settings are read back before being written because the module stores some of them in flash rated 200k erase cycles; a value that differs from what was asked for is reported as an extra '!! module holds ...' line rather than left silent"
RADIO-006: "The module does not filter by address: every packet in the network arrives as a +RCV line with the sender's address, and the decision is left to the host; filtering is Reticulum's job and it already does it by destination hash"
RADIO-007: "The module is half duplex and cannot receive while transmitting, so packets from the peer during a transmission are lost; Reticulum's retries make this a throughput question rather than a correctness one"
RADIO-008: "Electrical: VDD is 2.3-3.6 V and the UART pins are not 5 V tolerant, so the module runs from 3V3 and never VBUS/5V; it peaks at about 140 mA while transmitting at 22 dBm, and 14 dBm is the CE ceiling against the module's 22 dBm maximum"
RADIO-009: "Transmit and reception are both proven: announces on 2026-10-26, and 16 of 17 frames from a second module on a Pico 2 on 2026-10-27, with the +RCV parser, reassembler and RSSI/SNR path exercised and all error counters at zero"
FRM-001: "One AT+SEND=<address>,<length>,<data> per frame answered with +OK; reception arrives as one line per frame, +RCV=<from>,<length>,<data>,<rssi>,<snr>"
FRM-002: "<data> is standard base64, not decoration: the AT syntax is comma-delimited and line-terminated, so a raw packet would both corrupt the parser and be cut at the first 0x0D; 180 encoded bytes fill the 240-character field exactly, with no padding and no comma"
FRM-003: "The binary frame inside the base64 is one header byte NNNN M III -- nonce nibble randomised per frame, M set while more frames follow, rolling message id 0-7 -- followed by up to 179 bytes of Reticulum packet"
FRM-004: "A Reticulum packet is at most 500 bytes, so a full packet travels as three frames of 179 + 179 + 142; the receiver appends frames with the same id until one arrives with M clear, and a partial is dropped after 8000 ms without its remaining frames"
FRM-005: "The datasheet's SF7/125 kHz test point carries about 550 B/s of <data>; base64 costs a third, so expect about 400 B/s of Reticulum traffic and roughly 1.5 s of airtime for a full-MTU packet"
FRM-006: "An +ERR=n reply drops the rest of that packet rather than spending airtime on a message the peer can never assemble; errors are counted and shown by the L command"
FRM-007: "At these payload sizes the UART is the bottleneck rather than the radio: a 260-character command is about 23 ms of wire time at 115200 before anything is on air"
FRM-008: "The frame header has no length and no checksum, so a message missing a middle frame is delivered as a complete, silently short packet; a 500-byte message with one frame lost arrived as 321 bytes, which is why the Tang protocol needs its own length and CRC"
FRM-009: "The Tang protocol above the framing: version, type, sequence, big-endian body length, body and a CRC-16/CCITT-FALSE, with a full body making exactly one 500-byte MTU; the length and CRC are what make FRM-008's silent truncation detectable, and replies carry the request's sequence"
FRM-010: "Frames from address 2 are the Tang protocol and everything else is a Reticulum packet, so the LoRa interface stays registered with Reticulum; replies go to address 2, Reticulum keeps broadcasting, and the Tang initiates every exchange"
GPS-005: "The receiver is an Adafruit Mini GPS PA1010D (MTK3333) polled over I2C at 0x10 on A4/A5, streaming NMEA 0183 at 1 Hz with both $GP and $GN talkers and padding an idle read with 0x0A, which must not be mistaken for a sentence terminator"
GPS-002: "The GPS is the master clock: with a valid fix the DS3231 is corrected from RMC UTC whenever it is more than 2 s out, NTP takes over when there is WiFi but no fix, and the DS3231 carries the gaps at its own drift rate; NTP values below 1600000000 are treated as unsynchronised"
GPS-003: "PPS is captured on D4 (GPIO 7) as a rising-edge interrupt and measured only -- the last interval, the mean of the recent 16, the pulse count and intervals more than 2 ms off nominal; an unwired PPS pad leaves D4 floating, which reads as several hertz of noise instead of one pulse per second"
GPS-004: "An NMEA sentence identifies the second it describes but is only available after the receiver has sent it, so it identifies a second rather than timing it; with the receiver polled over I2C the delay is bounded by the poll interval rather than by a UART line rate"
SEN-001: "The DS3231 answers at 0x68 (RTClib's DS3231_ADDRESS) on SDA A4 and SCL A5, powered from 3V3 and not VBUS/5V because the breakout's pull-ups would put 5 V on a 3.3 V bus; a CR1220 keeps the clock across power loss, and lostPower() seeds the clock from the firmware build time"
SEN-002: "The LIS3MDL answers at 0x1C with WHO_AM_I 0x3D, register map per DS9463 Rev 7: CTRL_REG1 20h, CTRL_REG2 21h with the FS bits at 6 and 4, CTRL_REG3 22h, CTRL_REG4 23h, CTRL_REG5 24h with BDU, STATUS_REG 27h, and little-endian signed 16-bit outputs at 28h-2Dh"
SEN-003: "The SHT45 answers at 0x44: command 0xFD measures T and RH at high repeatability in up to 8.3 ms and returns six bytes; T = -45 + 175*ticks/65535 degC, RH = -6 + 125*ticks/65535 percent clamped to 0-100, and each 16-bit word carries a CRC-8 with polynomial 0x31 and init 0xFF"
SEN-004: "Because a high-precision SHT45 measurement blocks for up to 8.3 ms, it is not run in the status path: the main loop refreshes a cached reading every two seconds and status only formats it, so an LXMF reply never stalls on the sensor bus"
SEN-005: "Heading is relative to a bearing declared with h<bearing> or cal heading <deg>, which absorbs magnetic declination without a table; hard-iron offsets come from a rotation calibration and persist with the heading reference in /mag_cal, and both are invalidated by moving the node"
SEN-006: "A Bosch BMP581 answers at 0x47 with chip-id register 0x01 returning 0x50, giving calibrated pressure and temperature with no PROM coefficient read; identified on the TinyDesk bench, with this node's own 0x47 device still awaiting a chip-id read"
SEN-007: "The SHT45 at 0x44 has no chip-id register, so it is identified by an ACK plus a successful 0xFD measurement rather than an id read"
SEN-008: "The bench field is 46.6 uT (IGRF-14: 23.8 H, 40.0 Z, dip 59.3 deg); this board's magnetometer samples lie on an ellipsoid with 2.2 uT scatter, so it needs a 3x3 soft-iron matrix plus a centre, not three offsets, which take |B| spread from 32 percent to 2"
SEN-009: "Multi-byte register reads on the LIS3MDL and BMP581 need the sub-address MSB set, reg | 0x80, or the device repeats one register and a six-byte read yields a degenerate vector rather than an I2C error"
SEN-010: "An ellipsoid fit determines the shape of a soft-iron correction but not its absolute scale, which follows how much of the ellipsoid the samples covered; a partial tumble leaves the fitted surface small, so every corrected vector lands outside it and |B| reads high while the dip stays correct -- measured here as 53.4 uT against a 46.6 uT field at a dip of 58.6 deg, corrected by a uniform factor of 0.87251 to 46.3-46.8 uT"
SEN-011: "The bench itself is magnetic: a board whose gain was set in free air reads 44.8 to 45.9 uT and a dip of 61.6 deg once set down on the desk, against 46.3 to 46.8 uT and 60.3 deg in the air above it, so a calibration must be gained in the place it will be used"
SEN-012: "Magnetic heading is atan2 of the corrected horizontal field, meaningful only while the board is level, so with the magnetometer fixed flat and aligned to the GPS module it is a valid reading and needs no tilt compensation; the board has no accelerometer, so nothing could provide that compensation"
HOST-001: "host-config/config is an RNS instance with an explicit UDPInterface: listen 0.0.0.0:4242 and forward 255.255.255.255:4242, while the Sideband variant forwards to the node's address 10.10.0.35:4242"
HOST-002: "e2e_test.py takes the destination hash from k, owns its own RNS instance rather than attaching to a daemon, requests a path, sends a 30-byte packet and waits for the cryptographic proof; the last recorded run passed with rtt 0.676 s"
HOST-003: "rnsd must be started with setsid or it dies when the launching shell ends; rnpath and rnprobe sometimes hit a shared-instance auth-key mismatch on this box, which owning the instance avoids entirely"
HOST-004: "The host toolchain lives in two virtualenvs: ~/.venvs/rns for RNS and its CLI tools, and ~/.venvs/pio for PlatformIO; the system python3 has neither"
HOST-005: "nano-node/host-tests/run.sh compiles and runs four units with g++ -std=c++17 against a shim include directory -- GPS, magnetometer, SHT45 and LoRa framing -- with no hardware attached; all four passed on 2026-10-27"
HOST-006: "Sideband 2.2.0 is verified against the node on the same LAN; on this machine the AppImage needs --appimage-extract-and-run because FUSE mounts are blocked by the kernel's no-new-privs flag"
HOST-007: "A shared-instance client authenticates its RPC against the instance transport identity from its own config directory, so it must load the directory the running instance uses (~/.reticulum here); and one identity may have only one LXMF router at a time, because a second one announces its own ratchet and the two then drop each other's mail silently"
HOST-008: "A calibration cannot be judged by polling telemetry from a host: an LXMF round trip costs about 3 s, so a minute of turning yields ~20 clustered samples against the 20 Hz the board itself samples at"
TOOL-001: "PlatformIO env nano-esp32: platform espressif32, board arduino_nano_esp32, framework arduino, C++17 with -fexceptions, upload_speed 460800, monitor 115200 with the esp32_exception_decoder filter"
TOOL-002: "Build flags that change behaviour: -DRNS_USE_FS, -DUSTORE_USE_POSIXFS, -DUSTORE_USE_NOOPFS, -DMSGPACK_USE_BOOST=OFF, -DRNS_LOG_LEVEL=RNS_LOG_LEVEL_INFO, -Iinclude and -I../microReticulum/src because microLXMF includes microReticulum headers bare"
TOOL-003: "lib_deps: adafruit/RTClib ^2.1.4, ArduinoJson ^7.4.2, vendored MsgPack, attermann's Crypto and microStore from GitHub, and symlinks to microReticulum, microLXMF and the common udp_interface example"
PROV-001: "Three trees are vendored rather than submoduled, each carrying a local change: microReticulum 40fa628 Apache-2.0 (WiFi credentials from wifi_config.h), microLXMF 9876dff GPL-3.0-only (unmodified), MsgPack 0.4.2 MIT (two members made public); VENDORED.md is the record"
PROV-002: "microReticulum is Apache-2.0 and MsgPack is MIT, but microLXMF is GPL-3.0-only, so linking it makes the combined firmware image GPL-3.0; acceptable for personal or research use and relevant if the image is ever distributed"
PROV-003: "Each vendored tree keeps its own LICENSE file in its directory, and the local change to each is documented in VENDORED.md rather than left to be rediscovered in a diff"
```

---

## 5. Records

Records are grouped in the order of the topic catalog, and numbered within each topic.

```yaml
- record_id: BRD-001
  kind: BOARD
  topic_id: BRD
  title: "The node's serial port is bench state and has moved between /dev/ttyACM1 and /dev/ttyACM0"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "The board is an Arduino Nano ESP32 with USB serial 206EF131675C that held 10.10.0.35 on the LAB network. Its /dev/ttyACM* name is bench state and is not stable. On 2026-10-26 the node was on /dev/ttyACM1 and a second device on /dev/ttyACM0 was the TinyTang. The sessions recorded in core-log entries 4, 5 and 10 flashed the node over /dev/ttyACM0 instead. On 2026-11-01 at 07:43 only /dev/ttyACM0 existed, and by 07:56 no ttyACM device was present at all, so the node was not attached."
  consequence: "Never carry a port name forward from a previous session. PlatformIO picks the first candidate, so every upload and monitor invocation must name its port explicitly, and the port must be confirmed against the device rather than remembered. Reading the wrong port silently shows another board's console, which is the fastest way to misattribute a boot log; writing to the wrong one is how a stray console byte reaches the node's own command handler."
  sources:
    - "README.md, 'Verified on 2026-10-26 with the board on /dev/ttyACM1 (USB serial 206EF131675C, IP 10.10.0.35)' and 'Build and flash': 'On this machine /dev/ttyACM0 is the TinyTang, not the node.'"
    - "core-log entries 4, 5 and 10, each recording an upload over /dev/ttyACM0 after which the node answered on its own console"
    - "Enumeration on 2026-11-01: /dev/ttyACM0 present at 07:43 with no /dev/ttyACM1; no ttyACM device by 07:56, with only a Sipeed debug probe on /dev/ttyUSB0 and /dev/ttyUSB1"
  verification: "Bench state, not device behaviour: the serial number and address are this bench's. Revised 2026-11-01 (core-log entry 11): the record asserted that /dev/ttyACM0 was the TinyTang and the node was on /dev/ttyACM1, which core-log entries 4, 5 and 10 contradict; the title, statement and consequence now record that the name is unstable and has to be confirmed each session."

- record_id: BRD-002
  kind: BOARD
  topic_id: BRD
  title: "Harness pin map: two UARTs, PPS, I2C, LED and button"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "GPS on Serial1: module TX to D2, module RX to D3. PPS to D4 (GPIO 7). LoRa on Serial2: module TXD to D5 (GPIO 8), module RXD to D6 (GPIO 9). I2C sensors and RTC on SDA A4 and SCL A5. The built-in RGB LED is active-low and driven as 255 minus the value. One physical button on PB1 is reset. UART0 on D0/D1 is left free."
  consequence: "D2/D3, D4 and D5/D6 are consecutive, so the two UART harnesses and the PPS wire sit next to each other. D0/D1 staying free matters: it is the ROM console and the recovery path if a flash goes wrong."
  sources:
    - "README.md, 'Ultimate GPS (PA1616D) on Serial1', 'GPS PPS on D4 (GPIO 7)', 'LoRa (RYLR998 on the second UART)' wiring table, and 'RTC: Adafruit DS3231 on I2C'"
    - "nano-node/src/lora.cpp: LORA_RX_PIN D5, LORA_TX_PIN D6 with the comment that D5 is GPIO 8"
    - "nano-node/include/pps.h: 'Wiring: GPS PPS -> D4 (GPIO 7)'"
  verification: "Pin names read from the driver sources and matched against the README's hardware-verified wiring tables; the LoRa pairing was confirmed on the module in the 2026-10-26 run."

- record_id: BRD-003
  kind: BOARD
  topic_id: BRD
  title: "Serial and interrupt pin arguments are logical pins, not GPIO numbers"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "On this board HardwareSerial::begin() remaps its pin arguments through digitalPinToGPIONumber() (BOARD_HAS_PIN_REMAP), and attachInterrupt() is macro-remapped the same way, so the correct arguments are the logical Arduino pin names D2, D3, D4, D5 and D6."
  consequence: "Passing a raw GPIO number compiles and runs, but silently selects a different physical pin -- a failure mode with no diagnostic. Both driver headers carry the warning for this reason, and log lines print the pin name as written rather than its GPIO number so nobody is sent to the wrong pin."
  sources:
    - "nano-node/include/gps.h: 'Passing a GPIO number here silently lands on a different physical pin.'"
    - "nano-node/src/lora.cpp: the LORA_STR macro and the comment that a log line saying 'RX D8' would send someone to the wrong pin"
    - "nano-node/include/pps.h: 'attachInterrupt() is macro-remapped through digitalPinToGPIONumber() on this board'"
  verification: "The remap is documented in the ESP32 Arduino core variant for this board and is reproduced in all three driver headers; the firmware's own pin choices work on hardware (GPS, PPS and LoRa all initialise)."

- record_id: BRD-004
  kind: BOARD
  topic_id: BRD
  title: "Flashing needs no button press; the button path is app-only"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The board's tinyuf2 bootloader accepts DFU directly over USB, so `pio run -t upload` flashes with no button press. Entering the bootloader by hand is a double-tap inside the roughly 0.7 s RGB pulse (variant.cpp boot_double_tap_logic), and that path runs with ALT_COUNT 1, so it can write the application only."
  consequence: "Recovery from a bad application is easy; recovering a damaged partition table is not, and needs a different tool. Reboots during normal work are done from the serial console (r) rather than by pressing the button."
  sources:
    - "README.md, 'Build and flash': 'No button press needed: the board's tinyuf2 bootloader accepts DFU directly.'"
    - "README.md, 'Board control without the button': the boot_double_tap_logic double-tap and 'that DFU path is app-only (ALT_COUNT 1), so it cannot write a partition table'"
  verification: "DFU uploads were performed repeatedly on this board over /dev/ttyACM1, including the 2026-10-26 LoRa firmware. The ALT_COUNT 1 limit is read from the board variant source cited in the README."

- record_id: BRD-005
  kind: BOARD
  topic_id: BRD
  title: "Four relay outputs on D9..D12, numbered by cable order and closed by a HIGH"
  status: VERIFIED
  verified_date: 2026-10-29
  statement: "Four opto-isolated relay channels are driven from D9, D10, D11 and D12, which carry no other role: the GPS holds D2/D3, PPS D4, LoRa D5/D6 and I2C A4/A5, and D11 and D12 are also the SPI pair, which nothing here opens. The module fitted on this bench closes a relay when its input is HIGH, so ACTIVE_LOW is false in nano-node/src/relay.cpp; the first build assumed the common pulled-up module and every relay closed on `relay 0000` instead of opening. The four digits of the `relay` command follow the cable the channels are spliced into rather than the header: digit 1 is D12, digit 2 D11, digit 3 D10 and digit 4 D9. The pins are left as INPUT at boot and become outputs only when a command first drives them, because an output must be driven HIGH or LOW the moment it is configured and one of those two closes every relay on a module of the wrong polarity."
  consequence: "`relay <0000>` (cmd_relay() in nano-node/src/main.cpp) sets the outputs from that bit order and is served both on the Tang's LoRa link and over LXMF; relay::set() reads each pad back after a 100 us settle and the command answers `relay states updated` only when every pin took its level, `relay states not updated` otherwise. A module of the opposite polarity is one constant. relay::pin_number() names the pin behind a bit for anything that has to report it, and the `R` and `Z` serial console keys remain for bench work. `status` reports the same four digits on its own line, in the full LXMF reply and in the compact one the Tang's link receives."
  sources:
    - "nano-node/include/relay.h and nano-node/src/relay.cpp: PINS = {D12, D11, D10, D9}, ACTIVE_LOW = false, the read-back in set()"
    - "nano-node/src/main.cpp: cmd_relay(), the node_answer() and handle_command() branches, and the R/Z console keys"
    - "README.md, 'Four relay outputs on D9..D12' and the command table"
  verification: "Bench run 2026-10-29: the `R` console key stepped each channel and the user heard exactly one relay per step, which is what proved the pins reach the module; `relay 1010` over LXMF answered `relay states updated` and a following `relay` answered `relay 1010`; a malformed `relay 12` answered the usage line and drove nothing. Polarity and digit order were each corrected against the user's observation of which relay closed, not inferred from the module's model number."

- record_id: SOC-001
  kind: SOC
  topic_id: SOC
  title: "ESP32-S3 with 16 MB flash, 320 KB DRAM and 8 MB octal PSRAM"
  status: SOURCED
  verified_date: 2026-10-27
  statement: "The board is an ESP32-S3 with 16 MB of flash, 320 KB of DRAM and 8 MB of Octal-SPI PSRAM on a NORA-W106-10B module, per the Arduino Nano ESP32 datasheet."
  consequence: "The stack, the LXMF router and the sensors all fit in DRAM with room to spare, and the 16 MB flash is what makes a 9 MB FFat partition possible. PSRAM is available but the current firmware does not depend on it, so the build must not assume it."
  sources:
    - "README.md, 'What is running': 'ESP32-S3, 16 MB flash, 320 KB DRAM, 8 MB Octal-SPI PSRAM (NORA-W106-10B, per the Arduino datasheet)'"
    - "nano-node/platformio.ini header comment, same figures"
  verification: "Quoted from the datasheet through the README; the 320 KB DRAM figure is echoed by the PlatformIO build's RAM percentage, but the datasheet itself was not opened in this session, which is why this is SOURCED."

- record_id: SOC-002
  kind: SOC
  topic_id: SOC
  title: "The radio is 2.4 GHz only; a 5 GHz SSID can never be joined"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "The ESP32-S3 has no 5 GHz radio. On this bench the 5 GHz SSID 'cheese' runs on channel 161 and can never be joined; 'yogurt' is the same access point's 2.4 GHz radio, and its BSSID differs from cheese's only in the last octet (...58:F2 versus ...58:F3)."
  consequence: "Association failures that look like a credential problem can be a band problem. When a network is reported as unreachable, check the band before the password, and compare the BSSID suffix to tell the two radios apart."
  sources:
    - "README.md, 'Things to know', first bullet"
  verification: "Observed on hardware: the node associates with yogurt and cannot associate with cheese."

- record_id: FLS-001
  kind: FLASH
  topic_id: FLS
  title: "The factory partition table already carries the 9 MB FFat volume"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The board's default partition table already defines a 9 MB FAT partition named ffat with subtype 0x610000, and the build uses app3M_spiffs9M_fact512k_16MB.csv. The firmware finds the partition by name with esp_partition_find_first() and mounts it at /ffat."
  consequence: "No partition-table write is needed, which matters because the DFU path cannot perform one (BRD-004). A rebuild that changes the CSV would invalidate the stored identity and path table, so the choice is not free."
  sources:
    - "README.md, 'Storage: FFat, on the factory partition table'"
    - "nano-node/src/main.cpp: esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, \"ffat\") and FFat.begin(true, \"/ffat\", 24)"
  verification: "The mount succeeds on hardware and the boot log prints the partition offset and size; the identity and path table survive reboots on that volume."

- record_id: FLS-002
  kind: FLASH
  topic_id: FLS
  title: "FFat's default open-handle cap is too small for RNS's stores"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "FFat.begin() defaults to 10 open file handles. RNS's stores hold most of them: path_store index and segment, known_store, and three hashlist_store handles. The firmware mounts with maxOpenFiles 24."
  consequence: "Any new long-lived file handle competes for this budget. Raising the cap masks a leak rather than fixing one, so a new store or log should either close its handles or be counted here deliberately."
  sources:
    - "README.md, 'Two implementation notes that cost real debugging time': 'FFat.begin() defaults to 10 open handles ... Fixed by raising maxOpenFiles to 24'"
    - "nano-node/src/main.cpp: FFat.begin(true, \"/ffat\", 24)"
  verification: "Observed as a hard failure on hardware -- opening the message log for append failed until the cap was raised -- and the mount with 24 has run since."

- record_id: FLS-003
  kind: FLASH
  topic_id: FLS
  title: "The message log is appended through POSIX, not the Arduino FS layer"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Received messages are appended to /ffat/messages.log using POSIX fopen(path, \"a\") and fprintf, not FFat.open(path, FILE_APPEND), because the Arduino FS layer is subject to the maxOpenFiles cap."
  consequence: "The log therefore survives even when the handle budget is tight, and it is visible to any code that can open the volume. Code that appends through the Arduino layer would reintroduce the original failure."
  sources:
    - "README.md, 'Two implementation notes that cost real debugging time'"
    - "nano-node/src/main.cpp: the MESSAGE_LOG constant and the comment that POSIX append is used because FFat's open is capped by maxOpenFiles"
  verification: "The failure was seen on hardware ('!! could not open message log for append') and the POSIX path has written and read back entries across a reboot."

- record_id: FLS-004
  kind: FLASH
  topic_id: FLS
  title: "NVS holds configuration; FFat holds identity and learned state"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "WiFi SSID, password, IP configuration and display name are stored in NVS. The identity (/ffat/identity, 64 bytes), learned paths and the message log are stored on FFat. A factory reset from the wizard clears NVS and reboots into setup while leaving /ffat/identity in place."
  consequence: "Clearing WiFi settings never changes the node's address, so peers keep their paths. Only the wizard's explicit identity-change action deletes /ffat/identity, and that is deliberately destructive: every peer that knows the old address loses it."
  sources:
    - "README.md, 'First boot: WiFi setup wizard': 'Credentials live in a separate NVS namespace from FFat ... Clearing WiFi settings never touches /ffat/identity'"
    - "nano-node/include/settings.h: 'NVS holds what the user configured. FFat holds who the node is (identity) and what the network has learned (paths).'"
    - "nano-node/src/main.cpp: the factory-reset console path that deliberately does not touch /ffat/identity"
  verification: "Exercised on hardware: a factory reset keeps the address, and the identity-change action changes it. The boot log marks whether configuration came from [nvs] or [header]."

- record_id: FLS-005
  kind: FLASH
  topic_id: FLS
  title: "microStore evicts records stamped ahead of the clock, so the stores need real time"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "microStore::sweep() runs at every store init and drops any record whose timestamp is ahead of the current clock. With no RTC, microStore::time() is uptime plus an offset, so after a reboot every record written late in the previous session looked like it came from the future and the entire table was discarded."
  consequence: "The stores report zero entries unless wall-clock time is available before they initialise, which is why the firmware applies the DS3231 through OS::setTimeOffset() between constructing Reticulum and calling start(). Removing the RTC is not a cosmetic regression: persistence stops working."
  sources:
    - "README.md, 'Persistence works, and it needs the RTC'"
    - "nano-node/src/main.cpp: init_rtc(), apply_rtc_to_rns() and its placement relative to the Reticulum instance"
  verification: "Established on hardware from the failure and its fix: with no RTC the store reported 0 entries after a reboot; with the DS3231 applied first, the entry survives. The mechanism is a documented library behaviour, which is what makes this VERIFIED rather than INFERRED."

- record_id: FLS-006
  kind: FLASH
  topic_id: FLS
  title: "Message history is JSON Lines on /ffat, rotating at 512 KB"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Each received message is one JSON object per line in /ffat/messages.log with fields rx, rtc, from, sent, title and content. rx is the board's DS3231 time at arrival as a Unix epoch, rtc the same instant formatted as 'YYYY-MM-DD HH:MM:SSZ', sent the sender's timestamp and from the sender's LXMF address. The file rotates at 512 KB into /ffat/messages.1.log, keeping one generation."
  consequence: "A chatty network cannot fill the 9 MB volume, and a truncated final line can be detected by parsing failure rather than corrupting earlier entries. The serial console's m prints the count and the last eight messages, and c clears the log."
  sources:
    - "README.md, 'Message history on flash' with the sample object"
    - "nano-node/src/main.cpp: MESSAGE_LOG '/ffat/messages.log', MESSAGE_LOG_OLD, and MESSAGE_LOG_MAX 512*1024"
  verification: "Verified across a reboot on hardware with one 152-byte entry written, the board restarted, and the same entry read back."

- record_id: FLS-007
  kind: FLASH
  topic_id: FLS
  title: "Paths reach the filesystem prefixed, and two build flags are mandatory"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "microStore hands relative paths to the POSIX adapter verbatim, so a PrefixedFileSystem adapter prefixes them with /ffat, turning ./path_store/ into /ffat/path_store/. Two build flags are required for any of it to work: -DRNS_USE_FS, without which write_path_table() is compiled out, and -DUSTORE_USE_POSIXFS, which selects the POSIX adapter."
  consequence: "A build that drops either flag compiles, runs and silently never persists anything. The failure is invisible at runtime, so the flags belong in the platformio.ini record (TOOL-002) and any new build configuration should be checked against it."
  sources:
    - "README.md, 'Storage: FFat, on the factory partition table': 'Two flags matter: -DRNS_USE_FS (without it, write_path_table() is compiled out ...) and -DUSTORE_USE_POSIXFS'"
    - "nano-node/platformio.ini: the two flags beside -DUSTORE_USE_NOOPFS"
  verification: "Established on hardware: this was missing at first and nothing was ever written, which is how the flag was found to be load-bearing."

- record_id: NET-001
  kind: PROTOCOL
  topic_id: NET
  title: "The node speaks IPv4 UDP on port 4242 and broadcasts"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The node carries Reticulum over IPv4 UDP on port 4242, forwarding to 255.255.255.255:4242. DEFAULT_UDP_PORT is 4242 in UDPInterface.h, with DEFAULT_UDP_LOCAL_PORT and DEFAULT_UDP_REMOTE_PORT both defaulting to it, and the port is set at compile time."
  consequence: "Because the node broadcasts, it does not need to be told the client's address, but a client on a routed subnet will not hear it without changing the node's default remote host and rebuilding. Distinct per-user ports would need a firmware patch, since the port is not configurable at runtime."
  sources:
    - "microReticulum/examples/common/udp_interface/UDPInterface.h: DEFAULT_UDP_PORT 4242 and the local/remote port defaults"
    - "README.md, 'Connecting a client to this node'"
  verification: "A host RNS instance with the documented UDPInterface entry finds the node's destinations at 1 hop; this is the path every e2e run takes."

- record_id: NET-002
  kind: PROTOCOL
  topic_id: NET
  title: "Both interfaces run in gateway mode with transport enabled"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The UDP interface and the LoRa interface are both registered with mode MODE_GATEWAY, and the stack runs transport_enabled(true)."
  consequence: "The board answers path requests and can relay between the LAN and the radio, so a WiFi client can reach a peer that only has a radio. Reachability does not depend on announces for this reason: a client without a path can request one."
  sources:
    - "nano-node/src/main.cpp: udp_interface.mode(RNS::Type::Interface::MODE_GATEWAY) and reticulum.transport_enabled(true) with the comment 'answer path requests + relay for the mesh'"
    - "README.md, 'What is running'"
  verification: "Confirmed on hardware: rnsd on the host learns the node's destinations at 1 hop while the LoRa interface is registered."

- record_id: NET-003
  kind: PROTOCOL
  topic_id: NET
  title: "A stock Reticulum install cannot see this node"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "Reticulum's default AutoInterface uses IPv6 link-local multicast, which this node does not use, so a client running a stock configuration never discovers it. The client must carry an explicit UDPInterface entry with listen_port and forward_port 4242."
  consequence: "Every client setup starts from this, and a node that appears missing is usually a client configuration problem rather than a firmware one. Sideband has the same requirement and needs the same block in its own configuration."
  sources:
    - "README.md, 'Connecting a client to this node' and 'Chatting from Sideband'"
  verification: "Observed on hardware: stock clients see nothing until the UDP entry is added, after which the node's display name and lxm:// address are both usable."

- record_id: NET-004
  kind: PROTOCOL
  topic_id: NET
  title: "One process owns UDP 4242; shared instance serves the rest"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "Only one process may bind UDP 4242 at a time. With share_instance enabled behind a single rnsd, every other local program attaches to that instance instead of binding its own socket, and clients keep separate identities while the link to the board stays single."
  consequence: "Run rnsd and let other tools attach, or run Sideband alone -- not both as owners. rnsd must be launched with setsid, otherwise it dies when the invoking shell ends. Throughput and the node's single 2.4 GHz radio are shared, so isolation between users is at the identity layer, not the transport layer."
  sources:
    - "README.md, 'Multiple clients on one machine' and the setsid note under 'Verify from the host'"
  verification: "Verified on hardware with two independent CLI identities sending simultaneously through one shared instance, both reaching DELIVERED, and then Sideband attaching as a third client while rnsd kept the socket."

- record_id: NET-005
  kind: PROTOCOL
  topic_id: NET
  title: "A static address must sit outside the DHCP pool and needs a reflash to move"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "With a static IP configured, the board does not run DHCP, so it cannot detect an address conflict, and the address should be chosen outside the router's DHCP pool. A change of subnet requires reflashing the board, since the static configuration is compiled in or set through the wizard rather than discovered."
  consequence: "Moving the node between networks is a reconfiguration task, not a plug-in task. The wizard's static fields are validated as dotted quads before saving, which catches typos but not conflicts."
  sources:
    - "README.md, 'Credentials and network config' caveats"
    - "nano-node/include/wifi_config.h.example: RNS_WIFI_STATIC_IP, RNS_WIFI_GATEWAY, RNS_WIFI_NETMASK"
  verification: "Verified on hardware: the boot log reports 'static IP configured: 10.10.0.35 (gw 10.10.0.1)' followed by 'WiFi up, IP: 10.10.0.35', with delivery still working and the RTC unchanged across the reboot."

- record_id: RNS-001
  kind: EXTERNAL
  topic_id: RNS
  title: "microReticulum 0.5.0 at 40fa628, with one local patch"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The vendored microReticulum is version 0.5.0 (declared in library.json) at upstream commit 40fa628, Apache-2.0. It carries exactly one local change, in examples/common/udp_interface/UDPInterface.cpp: WiFi credentials come from wifi_config.h or -DRNS_WIFI_SSID instead of being hardcoded, so the credentials are not committed."
  consequence: "Re-vendoring upstream means re-applying that one patch, and the patch is small enough to audit by eye. Because the credentials arrive through the include path, a project without wifi_config.h still compiles and falls back to the placeholder strings."
  sources:
    - "VENDORED.md, first table row"
    - "microReticulum/examples/common/udp_interface/UDPInterface.cpp: __has_include(\"wifi_config.h\") and the RNS_WIFI_SSID fallback"
    - "microReticulum/library.json: version 0.5.0, license Apache-2.0"
  verification: "Read from the vendored tree and VENDORED.md in this session; the firmware builds against it and authenticates with the locally held credentials."

- record_id: RNS-002
  kind: EXTERNAL
  topic_id: RNS
  title: "The MTU is 500 in this port"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Type::Reticulum::MTU is 500 in microReticulum. The header's comment records that upstream RNS defaults to 507 and that peers must agree on the MTU to communicate. MDU is MTU minus HEADER_MAXSIZE and IFAC_MIN_SIZE, and LINK_MTU_DISCOVERY is enabled."
  consequence: "500 is the number the LoRa framing is built around: three frames of 179 + 179 + 142 (FRM-004). A port that changed the MTU would break the framing contract silently, so the value is worth asserting if the vendored tree is ever updated."
  sources:
    - "microReticulum/src/microReticulum/Type.h: static const uint16_t MTU = 500, with the surrounding comment about upstream's 507 default"
  verification: "Read directly from the header in this session, and the framing unit tests assert a 500-byte message spans exactly three frames."

- record_id: RNS-003
  kind: EXTERNAL
  topic_id: RNS
  title: "RNS flushes only at exit, so the firmware flushes on a timer"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "RNS writes its stores from exit_handler(), which never runs on a headless node that is not shut down. The firmware therefore calls Transport::persist_data() every 60 s and on demand from the p console command."
  consequence: "Without the explicit call, a reboot loses everything learned since the last flush. The interval bounds how much is at risk, and p exists so a planned reboot can lose nothing."
  sources:
    - "README.md, 'Storage: FFat, on the factory partition table'"
    - "nano-node/src/main.cpp: the 60 s persist cadence and the p command path"
  verification: "Demonstrated on hardware: p followed by a reboot preserves a learned path (RNS-005), which is only possible if the flush is explicit."

- record_id: RNS-004
  kind: EXTERNAL
  topic_id: RNS
  title: "OS::setTimeOffset must run before start() initialises the stores"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "OS::setTimeOffset() sets the offset applied to the platform clock, and every store timestamp derives from it. It must be called after the Reticulum instance exists and before start() initialises the stores."
  consequence: "The ordering is load-bearing rather than stylistic: called later, the stores initialise with uptime time and sweep their own records away (FLS-005). The firmware performs this handoff in one function for that reason."
  sources:
    - "nano-node/src/main.cpp: apply_rtc_to_rns(), its comment about the stores being stamped with real time, and its call site between construction and start()"
  verification: "The ordering is visible in main.cpp and is the documented fix for the empty-store failure; the fix was confirmed on hardware."

- record_id: RNS-005
  kind: EXTERNAL
  topic_id: RNS
  title: "Path persistence is proven across a reboot"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "A path learned from a host announce survives a reboot: after p and a restart the entry returns with has_path=1 hops=1 while nothing is announcing anywhere on the network."
  consequence: "Flash is the only possible source for that entry, so the storage path, the prefix adapter, the build flags and the clock ordering are all proven together by one test. This is the reference test for any change to storage."
  sources:
    - "README.md, 'Persistence works, and it needs the RTC'"
  verification: "Run on hardware on 2026-10-26 by learning a path, flushing, rebooting with the client silent, and reading the entry back."

- record_id: RNS-006
  kind: EXTERNAL
  topic_id: RNS
  title: "Pre-RTC records reappear as has_path=0 hops=128 and cannot be routed"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Stale records written during the era when the clock was wrong are resurrected by the index rebuild and appear in the d listing with has_path=0 hops=128. RNS refuses to route with them, and they age out on their own."
  consequence: "These entries are cosmetic residue, not a fault to chase or a reason to wipe the volume. Their presence is also a useful tell that the volume predates the RTC fix."
  sources:
    - "README.md, 'Things to know', 'Cosmetic residue'"
  verification: "Observed on hardware in the d listing; routing behaviour is consistent, and the entries disappear over time without intervention."

- record_id: RNS-007
  kind: EXTERNAL
  topic_id: RNS
  title: "The file-backed known-destination store does not retain on an ESP-IDF build"
  status: SOURCED
  verified_date: 2026-11-01
  statement: "With RNS_USE_FS and RNS_PERSIST_KNOWN_DESTINATIONS both set, the known-destination store is microStore's BasicFileStore rather than BasicHeapStore, and on a port observed on another bench a peer's announce arrived and was added to the path table while Identity::recall() never returned that peer's key, so no reply could be addressed. Building with RNS_PERSIST_KNOWN_DESTINATIONS=0 selects the heap store instead and recall works."
  consequence: "A node that must answer inbound messages may need the heap store on this port, or its own peer-key table. The table is small, so the RAM cost is minor and no flash is written per announce; path persistence is unaffected because it is a separate store. This project builds with -DRNS_USE_FS and leaves RNS_PERSIST_KNOWN_DESTINATIONS at its default of 1, which selects the file store, so it sits on the same code path as the observation."
  sources:
    - "microReticulum Persistence/IdentityEntry.h: KnownStore is BasicFileStore only when RNS_USE_FS && RNS_PERSIST_KNOWN_DESTINATIONS, otherwise BasicHeapStore"
    - "microReticulum Type.h: RNS_PERSIST_KNOWN_DESTINATIONS defaults to 1 when undefined"
    - "microReticulum Identity.cpp: remember() writes to _known_destinations, recall() reads it, with a path-table announce fallback gated by RNS_IDENTITY_ANNOUNCE_RECALL"
    - "nano-node/platformio.ini: -DRNS_USE_FS in build_flags, with no RNS_PERSIST_KNOWN_DESTINATIONS define"
  verification: "The store selection and the default are confirmed by inspection of this repository's vendored tree on 2026-11-01. The recall failure itself was observed on the TinyDesk bench, a different node, on 2026-11-01, and has not been reproduced on this one; the fix there was a build-system define in that project's components/rns/CMakeLists.txt, which here would be a -DRNS_PERSIST_KNOWN_DESTINATIONS=0 entry in build_flags. Revised 2026-11-01 (core-log entry 11): the record was previously VERIFIED, which overstated it — no TinyNode hardware or session confirmed the recall failure, so it is SOURCED and owes a test on this board before the messages and body request types are built on opportunistic replies."

- record_id: RNS-008
  kind: EXTERNAL
  topic_id: RNS
  title: "Transport::path_table() is not the live path store"
  status: SOURCED
  verified_date: 2026-11-01
  statement: "Path inserts and has_path() use Transport::_new_path_table, a microStore TypedStore; Transport::path_table() returns a separate container, _path_table, which the insert path no longer touches. Transport::paths_added() is the counter for paths learned."
  consequence: "A status line counting Transport::path_table().size() reads zero on a node with working paths, because that container is not the one being filled. Use paths_added() for paths learned and new_path_table() for membership. This project's path_store_count() resolves new_path_table() to FileStore::size(), which returns _index.size() and is therefore a record count, so its figures are sound."
  sources:
    - "microReticulum Transport.h: path_table() returns _path_table and new_path_table() returns _new_path_table, with paths_added() declared beside them"
    - "microReticulum Transport.cpp: has_path() consults _new_path_table.exists(), with the previous _path_table.find() left commented out under a 'CBA microStore' marker"
    - "microStore FileStore.h: size() returns _index.size(), a record count"
    - "nano-node/src/main.cpp: path_store_count() takes new_path_table() and calls size() on it"
  verification: "The container split and the counter are confirmed by inspection of this repository's vendored tree on 2026-11-01, and this project's own counts rise with announces as a count should, namely `paths 35` in core-log entry 3 and `paths 58` in core-log entry 6. The claim that the path store's size() reports bytes of a stored path string is not supported by this tree and has been withdrawn. One contrary observation, a status line printing paths=1634742062, came from the TinyDesk bench and remains unexplained; it is recorded here as a lead rather than a mechanism. Revised 2026-11-01 (core-log entry 11): the size()-returns-bytes clause was withdrawn after it failed inspection, and the record's provenance was corrected."

- record_id: LXMF-001
  kind: EXTERNAL
  topic_id: LXMF
  title: "microLXMF 0.1.0 at 9876dff is GPL-3.0-only and unmodified"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The vendored microLXMF is version 0.1.0 (library.json) at upstream commit 9876dff, licensed GPL-3.0-only, vendored with no local changes."
  consequence: "The library needs no patching but sets the licence of the whole firmware image (PROV-002). Because it is unmodified, re-vendoring is a straight copy, and any change made here would become the project's own GPL obligation to track."
  sources:
    - "VENDORED.md, second table row"
    - "microLXMF/library.json: version 0.1.0, license GPL-3.0-only"
  verification: "Read from the vendored tree and VENDORED.md; the firmware links against it and delivers messages on hardware."

- record_id: LXMF-002
  kind: EXTERNAL
  topic_id: LXMF
  title: "The router announces before the display name unless told not to"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "LXMRouter(identity, storage_path, announce_at_start) defaults announce_at_start to false. If the router announces at construction, it does so before set_display_name() has been called, and the announce carries empty app-data, so clients show no name."
  consequence: "The firmware constructs the router with announce_at_start false, sets the display name, and only then announces. A client joining later discovers the node by name and never needs the destination hash."
  sources:
    - "microLXMF/src/LXMF/LXMRouter.h: the constructor declaration and its @param announce_at_start 'default: false'"
    - "nano-node/src/main.cpp: the comment that announce_at_start is false because the constructor would announce before the display name is set"
  verification: "Read from the header and the firmware's construction site; the name is visible in clients on hardware, which is the observable consequence."

- record_id: LXMF-003
  kind: EXTERNAL
  topic_id: LXMF
  title: "MessageStore is never wired into LXMRouter"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "microLXMF ships a MessageStore class, but LXMRouter.cpp never instantiates or references it; the only mention is a comment calling persistence to it a follow-up. Nothing is persisted by the library."
  consequence: "Message history on the node exists because the firmware appends to /ffat/messages.log itself (FLS-006), not because the router stores anything. Anyone expecting /ffat/lxmf to hold a message database should read this first."
  sources:
    - "microLXMF/src/LXMF/LXMRouter.cpp: the comment 'Persistence to MessageStore is a follow-up' and the absence of any other MessageStore reference in that file"
    - "microLXMF/src/LXMF/MessageStore.h and MessageStore.cpp: the class exists and is complete"
    - "README.md, 'Two implementation notes that cost real debugging time', first bullet"
  verification: "Established by grepping the whole library: MessageStore appears only in its own files plus that one comment. The consequence was seen on hardware as missing history before the firmware log was written."

- record_id: LXMF-004
  kind: EXTERNAL
  topic_id: LXMF
  title: "A single link packet carries 63 bytes of content"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "LORA_LINK_PACKET_MAX_CONTENT is LINK_PACKET_MDU minus LXMF_OVERHEAD, which the header states as 175 minus 112, or 63 bytes of content in one link packet. Content longer than that is carried by Resource transfer."
  consequence: "Replies are deliberately kept under 63 bytes so the same command set stays usable over the radio, where a Resource transfer would be slow and fragile. status is the documented exception because it reports everything at once."
  sources:
    - "microLXMF/src/LXMF/Type.h: LORA_LINK_PACKET_MAX_CONTENT and the comment 'max content in single link packet = 175 - 112 = 63 bytes'"
    - "microLXMF/src/LXMF/LXMessage.cpp: the comment about using the LoRa-constrained limit so link packets fit the LoRa wire MTU"
    - "README.md, LXMF command table note: 'Everything except status is kept under the 63-byte LXMF single-packet content limit'"
  verification: "Read from the library headers and matched against the command set's documented sizing."

- record_id: LXMF-005
  kind: EXTERNAL
  topic_id: LXMF
  title: "A delivery proof is not an application acknowledgement"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "LXMF returns a cryptographic delivery proof to the sender once the receiving stack has accepted, decrypted and validated the message. That proof is what clients display as Delivered, and it says nothing about what an application then did with the content."
  consequence: "The node needs no application ack, and adding one would double traffic for no information. The honest reading of a proof is that the router delivered the message to its callback, which here means it was logged."
  sources:
    - "README.md, 'Why no ack is needed'"
    - "delivery_test.py: asserts LXMF state DELIVERED and that no reply is sent"
  verification: "Observed on hardware: delivery_test reported PASS with 2 of 2 delivered and 0 replies, at 2.8 s and 2.1 s, with both messages present in the serial log."

- record_id: LXMF-006
  kind: EXTERNAL
  topic_id: LXMF
  title: "Destinations, display name and the identity file"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "The node announces the destination nano/esp32 and the LXMF delivery destination lxmf.delivery. The identity is generated per board on first boot and stored as 64 bytes in /ffat/identity, so the address is stable across reboots and differs between boards. The display name is the one set in the wizard, or 'Nano ESP32 ' plus the last four hex digits of the identity."
  consequence: "Two boards are distinguishable in a client's peer list, and one board keeps its address across firmware updates. The address changes only when the identity file is deleted, which the wizard does deliberately after a typed confirmation."
  sources:
    - "nano-node/src/main.cpp: APP_NAME 'nano', APP_ASPECT 'esp32', IDENTITY_FILE '/identity', and the derived display-name fallback"
    - "README.md, 'LXMF messaging' and 'First boot: WiFi setup wizard' (identity change screen)"
  verification: "Confirmed on hardware: k prints the address and an lxm:// URI, e2e delivery works against that address, and resetting the identity changes it."

- record_id: LXMF-007
  kind: EXTERNAL
  topic_id: LXMF
  title: "LXMF message fields: a numbered registry, not free-form metadata"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "An LXMF message carries, besides timestamp, content and title, a map of numbered fields. The reference implementation allocates them in LXMF/LXMF.py: telemetry and stream (0x02, 0x03), icon appearance (0x04), file attachments (0x05), image (0x06), audio (0x07), thread (0x08), commands and results (0x09, 0x0A), group (0x0B), ticket (0x0C), event (0x0D), rnr refs (0x0E), renderer (0x0F), reply-to and reply-quote (0x30, 0x31), reaction, comment and continuation (0x40-0x42). The header states that unallocated values 0x00-0x80 are reserved for future extensibility, that experimental features should use values above 0xFF, and that 0xFB-0xFD are the sanctioned escape hatches for data that is not native to LXMF."
  consequence: "Anything a node reports to a client should use the allocated field rather than text in the content, because that is what makes it machine-readable and what other clients already render. For this project the two that matter are FIELD_TELEMETRY (0x02) for current readings and FIELD_TELEMETRY_STREAM (0x03) for a list of them over time; FIELD_COMMANDS (0x09) and FIELD_RESULTS (0x0A) carry request and answer. FIELD_RENDERER (0x0F) with RENDERER_MICRON (0x01) is how to signal formatted text when it is wanted. Content stays empty for a telemetry-only message so clients do not show it as chat."
  sources:
    - "LXMF/LXMF.py (reference implementation, lxmf 1.2.1 in ~/.venvs/rns): the FIELD_ block and the comments on reserved ranges and the CUSTOM_TYPE/CUSTOM_DATA escape hatch"
    - "Sideband sbapp/sideband/core.py: the branch that squelches a notification when a message carries only FIELD_TELEMETRY, FIELD_TELEMETRY_STREAM or FIELD_COMMANDS, confirming those are the intended carriers"
  verification: "Re-read on 2026-11-01 from the sources present on this bench: LXMF 1.2.1 at ~/.venvs/rns/lib/python3.14/site-packages/LXMF/LXMF.py carries the FIELD_ block above value for value, from FIELD_EMBEDDED_LXMS 0x01 through FIELD_CONTINUATION 0x42, together with the 0x00-0x80 reservation comment, the advice to use values above 0xFF for unstable features, and the 0xFB-0xFD custom trio; Sideband 2.2.0's sbapp/sideband/core.py carries the branch that squelches a notification for a message holding only FIELD_TELEMETRY, FIELD_TELEMETRY_STREAM or FIELD_COMMANDS. Revised 2026-11-01 (core-log entry 11): the verification previously rested on a hardware session on the TinyDesk bench; the registry is a source fact, so it is now cited from the sources held here."

- record_id: LXMF-008
  kind: EXTERNAL
  topic_id: LXMF
  title: "Telemetry is a msgpack map of sensor ids, defined by Sideband"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "FIELD_TELEMETRY carries a msgpack map of sensor-id to value. The ids are defined in Sideband's sbapp/sideband/sense.py as SIDs: TIME 0x01, LOCATION 0x02, PRESSURE 0x03, BATTERY 0x04, PHYSICAL_LINK 0x05, ACCELERATION 0x06, TEMPERATURE 0x07, HUMIDITY 0x08, MAGNETIC_FIELD 0x09, AMBIENT_LIGHT 0x0A, GRAVITY 0x0B, ANGULAR_VELOCITY 0x0C, PROXIMITY 0x0E, INFORMATION 0x0F, RECEIVED 0x10, POWER_CONSUMPTION 0x11, POWER_PRODUCTION 0x12, PROCESSOR 0x13, RAM 0x14, NVM 0x15, TANK 0x16, FUEL 0x17, LXMF_PROPAGATION 0x18, RNS_TRANSPORT 0x19, CONNECTION_MAP 0x1A, CUSTOM 0xFF. Telemeter.packed() always writes SID_TIME as an integer and each active sensor under its own SID. FIELD_TELEMETRY_STREAM carries a list of such maps for history."
  consequence: "A node that reports its sensors under these ids appears in Sideband's telemetry view with no client-side work, which is the whole reason to use them rather than a bespoke text format. This project's fitted parts map directly: the PA1010D gives LOCATION and can give TIME, the SHT45 gives TEMPERATURE and HUMIDITY, the LIS3MDL gives MAGNETIC_FIELD, the BMP581 gives PRESSURE and TEMPERATURE. SID_TIME must be present in any telemetry map."
  sources:
    - "Sideband sbapp/sideband/sense.py: the Sensor SID constants, the available-name table in Telemeter.__init__(), and Telemeter.packed()"
    - "LXMF/LXMF.py: FIELD_TELEMETRY 0x02 and FIELD_TELEMETRY_STREAM 0x03"
  verification: "Re-read on 2026-11-01 from Sideband 2.2.0's sbapp/sideband/sense.py, extracted from the AppImage installed on this bench: the Sensor class carries the SID values above, and Telemeter.packed() assigns packed[Sensor.SID_TIME] = int(time.time()) before adding each active sensor under its own sid, which is the integer-TIME rule stated here. The end-to-end render, in which a node's FIELD_TELEMETRY map was parsed and displayed as Timestamp, Ambient Pressure, Temperature and Relative Humidity, happened on the TinyDesk bench on 2026-11-01 and has not been repeated on this node. Revised 2026-11-01 (core-log entry 11): the SID list is now cited from the source held here, and the sensor addresses this record leaned on through SEN-006, SEN-007 and GPS-005 belong to that other bench rather than to this node."

- record_id: LXMF-009
  kind: EXTERNAL
  topic_id: LXMF
  title: "Commands travel in FIELD_COMMANDS as a list of single-key dicts"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "FIELD_COMMANDS (0x09) carries a list of dicts, each with one key naming the command. Sideband's Commands constants are PLUGIN_COMMAND 0x00, TELEMETRY_REQUEST 0x01, PING 0x02, ECHO 0x03 and SIGNAL_REPORT 0x04. A telemetry request is sent as {TELEMETRY_REQUEST: [timebase, is_collector_request]}, and the answer is expected in FIELD_TELEMETRY or FIELD_TELEMETRY_STREAM. An echo request carries the bytes to echo under ECHO."
  consequence: "A node that answers TELEMETRY_REQUEST and PING is usable from Sideband's own controls, and one that answers ECHO gives the operator a connectivity check they can read. Content stays empty; the command is in the field, and a message with no content but a recognised field is treated by clients as telemetry rather than as an empty chat message."
  sources:
    - "Sideband sbapp/sideband/sense.py: the Commands class"
    - "Sideband sbapp/sideband/core.py: the telemetry request construction with FIELD_COMMANDS and the TELEMETRY_REQUEST key, and the originator test on FIELD_COMMANDS"
    - "Sideband sbapp/ui/messages.py: rendering of FIELD_COMMANDS entries, including the ECHO case"
  verification: "Re-read on 2026-11-01 from the sources held on this bench: sbapp/sideband/sense.py carries the Commands class with PLUGIN_COMMAND 0x00, TELEMETRY_REQUEST 0x01, PING 0x02, ECHO 0x03 and SIGNAL_REPORT 0x04, and sbapp/sideband/core.py builds the request as a FIELD_COMMANDS list holding one dict keyed by Commands.TELEMETRY_REQUEST whose value is the two-element list of request timebase and collector flag. The end-to-end run, in which a node acted on a host client's TELEMETRY_REQUEST and answered in FIELD_TELEMETRY, happened on the TinyDesk bench on 2026-11-01 and has not been repeated on this node. Revised 2026-11-01 (core-log entry 11): the wire form is now cited from the sources held here, and the hardware half is attributed to that other bench."

- record_id: LXMF-010
  kind: EXTERNAL
  topic_id: LXMF
  title: "A field value must itself be msgpack-encoded bytes"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "microLXMF stores message fields as raw bytes and re-emits them with MsgPack Packer::packRawBytes(), so a field value is written into the payload verbatim. A FIELD_TELEMETRY value must therefore be a msgpack bin containing the packed map, not the map itself; sending the bare map makes the outer msgpack parse it into a dictionary on the receiving side. The Python reference produces a bin implicitly, because it packs the field value with umsgpack.packb() and the outer packer emits those bytes as a bin."
  consequence: "A node that writes the map directly is rejected by clients with 'An error occurred while unpacking telemetry. The contained exception was: packed data must be type bytes or bytearray', which reads like a client bug and is not. The same rule applies in reverse inbound: FIELD_COMMANDS arrives as a raw msgpack array, so a parser must read it as msgpack rather than expect a bin."
  sources:
    - "microLXMF LXMessage.cpp: pack() writes each field with packer.packRawBytes(key) and packer.packRawBytes(value)"
    - "microLXMF LXMessage.cpp unpack: fields are captured as raw byte spans between msgpack items"
    - "Sideband sbapp/sideband/sense.py: Telemeter.from_packed() calls umsgpack.unpackb on the field value"
  verification: "Re-read on 2026-11-01 from sources held here: this repository's microLXMF LXMessage.cpp writes every field with packer.packRawBytes() on both key and value, at four separate sites, so the stored bytes are re-emitted verbatim rather than re-encoded; and Sideband's sbapp/sideband/sense.py parses an incoming telemetry value with umsgpack.unpackb, so a bare map arrives as a dictionary and is refused as not bytes. The hardware confirmation, in which the client reported a 4-entry dict and then read 'FIELD_TELEMETRY: 33 bytes' once the map was wrapped in a bin header, came from the TinyDesk bench on 2026-11-01 and has not been repeated here. Revised 2026-11-01 (core-log entry 11): the encoding rule is now cited from the vendored source in this repository and the client-side parser, with the hardware half attributed to that other bench."

- record_id: LXMF-011
  kind: EXTERNAL
  topic_id: LXMF
  title: "159 bytes is a delivery-method threshold, not a telemetry size limit"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "LXMF has no maximum message size. A message is sent one of two ways: as a single OPPORTUNISTIC packet with no link established, or by DIRECT delivery over a link, where LINK_PACKET_MAX_CONTENT may travel in one link packet and anything larger becomes a RESOURCE transfer. The 159-byte LORA_ENCRYPTED_PACKET_MDU is only the threshold for the first case: above it LXMF stops trying to fit the message in one packet and moves it to a link. Both implementations default to DIRECT, so an oversized telemetry message is not refused; the Python reference logs 'Falling back to link-based delivery' and microLXMF logs 'Too large for OPPORTUNISTIC, using DIRECT resource'. What the threshold decides is setup cost and latency, not whether the data arrives."
  consequence: "No sensor has to be discarded to make telemetry fit, and the earlier reading of this threshold as a size limit was wrong. What it does decide is how a reply is carried over LoRa, where airtime is the scarce resource: a message at or under 159 packed bytes goes as one packet, while a larger one needs a link established and then a resource transfer, which costs seconds to tens of seconds of radio time rather than a fraction of a second. The real question for sensor choice is therefore how often a reading is worth that cost, not whether it can be sent. For budgeting a single packet, the committed overhead is 114 bytes -- 16 destination hash, 16 source hash, 64 signature, 14 msgpack envelope, 1 fields-map header, 1 FIELD_TELEMETRY key and 2 bin header -- leaving 45 bytes of packed sensor map, in which TIME costs 6, LOCATION 41, MAGNETIC_FIELD 29, and PRESSURE, TEMPERATURE and HUMIDITY 10 each, measured by tools/telemetry_budget.py."
  sources:
    - "LXMF 1.2.1 LXMessage.py: desired_method defaults to DIRECT when unset; an oversized OPPORTUNISTIC request falls back to DIRECT; and the DIRECT branch selects PACKET or RESOURCE by LINK_PACKET_MAX_CONTENT with no upper bound"
    - "microLXMF LXMessage.h: _desired_method defaults to Type::Message::DIRECT"
    - "microLXMF LXMessage.cpp: 'Too large for OPPORTUNISTIC, using DIRECT resource' when the packed size exceeds LORA_ENCRYPTED_PACKET_MDU"
    - "microLXMF Type.h: LORA_ENCRYPTED_PACKET_MDU 159, ENCRYPTED_PACKET_MDU 391, LINK_PACKET_MAX_CONTENT = LINK_PACKET_MDU - LXMF_OVERHEAD"
    - "Sideband sbapp/sideband/sense.py: Time.pack() returns an int, Pressure/Temperature/Humidity return floats, MagneticField.pack() returns three floats, Location.pack() returns seven items"
    - "tools/telemetry_budget.py: the deterministic per-sensor measurement"
  verification: "The delivery-method behaviour is read directly from both implementations on this bench on 2026-11-01, and the byte budget was measured with tools/telemetry_budget.py, cross-checked the same session against the vendored MsgPack::Packer compiled on the host: both report per-sensor marginals of 41, 10, 10, 10 and 29 bytes and identical packed sizes for every candidate reply. Revised 2026-11-01 (core-log entry 12): this record previously presented the 159-byte threshold as a size limit and concluded that LOCATION could not be telemetered; that was wrong, since the threshold only selects single-packet versus link delivery and an oversized message still arrives as a resource. The title, statement and consequence were rewritten and the earlier 495-byte and 33-byte figures, which came from another bench, were dropped."

- record_id: LXMF-012
  kind: EXTERNAL
  topic_id: LXMF
  title: "Link delivery works to Sideband; its telemetry panel needs a restart to repaint"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "A 614-byte FIELD_TELEMETRY reply carrying LOCATION and MAGNETIC_FIELD was sent by this node over a DIRECT link as a resource transfer and was received, decrypted, parsed and committed to Sideband 2.2.0's own database. The node logged 'telemetry: queued 499 bytes (full/link)' then 'lxmf: sent ... (614 bytes, method 2)' then 'lxmf: delivered', and Sideband's sqlite telemetry table gained a row stamped 2026-11-01T16:02:20Z with a ten-entry sensor map. Sideband also recorded 'telemetry.<dest>.last_request_success_timebase', so it considered the request answered. The link-delivery failure reported from another bench did not reproduce here with Sideband as the peer."
  consequence: "Position and magnetic field can be telemetered over LXMF on this stack; a link is needed, but a link works. One display caveat is a client-side artefact rather than a delivery fault: Sideband's conversation telemetry panel kept showing entries hours old after the reply had already been stored, and restarting Sideband repainted it with the current reading. The database evidence distinguishes the two cases, so a 'stale telemetry' report should be checked against the telemetry table's newest row before any firmware is suspected."
  sources:
    - "Sideband 2.2.0's sqlite database at ~/.config/sideband/app_storage/sideband.db, table telemetry, row stamped 1793548940 with a 10-entry map"
    - "The node's serial console for the same request: telemetry queued 499 bytes (full/link), sent 614 bytes method 2, delivered"
    - "core-log entry 13: the full sequence and the restart that repainted the panel"
  verification: "Confirmed end to end on this bench 2026-11-01 with Sideband 2.2.0 as the receiver. The same payload was also fed to Sideband's own Telemeter.from_packed() offline, which parsed it and returned all nine sensors with a current timebase, so encoding and delivery were confirmed independently of the panel. Revised 2026-11-01 (core-log entry 13): this record previously said the question was unestablished and carried another bench's failure; it is now VERIFIED, with the failure attributed to that bench and not to this stack."

- record_id: LXMF-013
  kind: EXTERNAL
  topic_id: LXMF
  title: "Sideband has no heading sensor id; CUSTOM is the only slot that renders one"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "Sideband 2.2.0's Sensor class defines twenty-seven sensor ids and none of them is a heading, compass or orientation. The two that appear to be candidates both refuse one. MAGNETIC_FIELD (0x09) is exactly three values: pack() writes [x, y, z], unpack() reads packed[0] through packed[2], and render() emits x, y and z, so a fourth element is read by nobody. LOCATION (0x02) does carry an element the client calls `bearing`, and Location.render() renames it to `heading`, but the UI prints it only inside a branch taken when speed exceeds 0.1 km/h: objectdetails.py builds the row as a single string combining speed and heading, so a node reporting zero speed never displays the heading at all, and the element's real meaning is GPS course over ground rather than compass direction. CUSTOM (0xFF) carries a list of [label, [value, custom_icon]] entries; Custom.unpack() rebuilds it as a dict and Custom.render() emits one entry per label, which objectdetails.py prints as `{label} {value}` with the given icon or the sensor's own. Nothing in that path tests speed."
  consequence: "A heading from a stationary node must travel in CUSTOM, as an entry such as [\"Magnetic heading\", [\"359.8 degrees\", \"compass-rose\"]]. Putting it in LOCATION would appear correct in a schema reading and fail silently in the field, since the failure depends on speed rather than on anything the node can detect. MAGNETIC_FIELD must stay three floats. CUSTOM entries cost roughly forty bytes each, which matters for the compact telemetry map: it sits at 151 packed bytes against a 159-byte single-packet limit, so a heading there would push the reply onto a link and lose the property that makes compact mode worth having."
  sources:
    - "Sideband 2.2.0 sbapp/sideband/sense.py: the Sensor SID constants, MagneticField.pack/unpack/render, Location.unpack/render, Custom.pack/unpack/render, and Telemeter.from_packed()"
    - "Sideband 2.2.0 sbapp/ui/objectdetails.py: the Location branch gated on speed > 0.1, and the Custom branch building the row as f'{label} [b]{value}[/b]'"
  verification: "Read from the Sideband 2.2.0 AppImage on this bench 2026-11-01, and confirmed live: the node's own 660-byte telemetry reply, captured as raw field bytes, was passed to Telemeter.from_packed() from that same installation and produced the entry {'Magnetic heading': ['359.8\u00b0', 'compass-rose']}, which rendered as the row 'Magnetic heading [b]359.8\u00b0[/b]' with the compass-rose icon. The same bytes were also decoded by tools/telemetry_request_test.py, which showed SID 0xff carrying that list under the map."

- record_id: LXMF-014
  kind: EXTERNAL
  topic_id: LXMF
  title: "INFORMATION is one plain string the client shows as a copyable row"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "Sideband's SID_INFORMATION (0x0F) carries a single msgpack string rather than a structure: Information.pack() returns self.data['contents'] and unpack() wraps whatever arrives as {'contents': str(packed)}. Information.render() emits the row with the icon information-variant under the name Information, and the UI branch in objectdetails.py treats the value as free text, displaying it whole and offering a copy-to-clipboard action on it. There is no length limit in the client and no sub-field to fill in."
  consequence: "This is the place to say what a node is, in one line, for an operator who has just discovered it: firmware version, uptime, and the address to reach it on all fit and all read well in the row. Because the client copies the string verbatim, it should be a plain sentence rather than a formatted blob. The firmware builds it from the app descriptor the build embeds rather than from a literal, so the reported version cannot drift from the image actually running, which is the failure mode a hardcoded string has. It is in the full telemetry map only, to keep the compact map inside one packet."
  sources:
    - "Sideband 2.2.0 sbapp/sideband/sense.py: the Sensor.SID_INFORMATION constant, Information.pack(), unpack() and render()"
    - "Sideband 2.2.0 sbapp/ui/objectdetails.py: the 'Information' branch, which copies the value to the clipboard on release"
    - "ports/nanoesp32/main/node.cpp: info_string(), using esp_app_get_description()->version"
  verification: "Confirmed on this board 2026-11-01: the node's 752-byte telemetry reply, captured as raw field bytes, parsed through Sideband 2.2.0's Telemeter.from_packed() into eleven recognised sensors including 'information', and the map carried SID 0x0f as the string 'TinyNode Nano, tinydesk 0.2.0, up 1m36s, ip 10.10.0.35'."

- record_id: LXMF-015
  kind: EXTERNAL
  topic_id: LXMF
  title: "Icon appearance is a name and two three-byte colours, and it costs a packet"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "FIELD_ICON_APPEARANCE (0x04) carries a three-element array: an icon name as a string, then the foreground and background colours as three raw bytes each, in that order. The client writes it to itself with struct.pack('!BBB') and reads it back with a matching three-byte unpack that divides by 255, so hex colour strings are not accepted. The icon name indexes the MaterialDesignIcons webfont shipped inside the application; a name the font does not carry renders blank rather than raising an error, so it fails silently. The packed field costs about 26 bytes on every message that carries it, and the client accepts it from an untrusted peer by default, because telemetry_receive_trusted_only and display_style_from_all both default false and true respectively."
  consequence: "A node can present its own icon and colours in the client's conversation list and on the map without the operator configuring anything, but the cost has to be watched: the compact telemetry reply is a 151-byte one-packet message and adding this field took it to 175 bytes and onto a link, destroying the property that makes compact mode useful. The firmware therefore attaches it only to replies that already travel over a link. Any future field added to outgoing messages has to be measured against packed_size the same way, because the message still arrives when it crosses the threshold -- it simply stops being link-free, and nothing in the code fails."
  sources:
    - "Sideband 2.2.0 sbapp/sideband/core.py: the fth() colour packer producing struct.pack('!BBB', ...), and the appearance read path with its matching three-byte unpack"
    - "Sideband 2.2.0 sbapp/ui/map.py and sbapp/ui/objectdetails.py: appearance[0] used as the icon name, [1] as icon_color and [2] as md_bg_color"
    - "The MaterialDesignIcons webfont's glyph table inside the Sideband 2.2.0 AppImage, where 'radio-tower' was confirmed present"
  verification: "Confirmed on this board 2026-11-01: a full telemetry reply arrived carrying field 0x04 as ['radio-tower', f5f5f5, 00695c], and those bytes decode through the client's own three-byte read to the Kivy floats 0.9608 and 0.4118, 0.3608. The packet cost was measured rather than assumed: with the field on every reply the compact message packed to 175 bytes at method 2, and with it withheld from compact the same reply packed to 151 bytes at method 1."

- record_id: LXMF-016
  kind: EXTERNAL
  topic_id: LXMF
  title: "The Reticulum Transport map must nest its interfaces and carry four counts"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "Sideband's RNSTransport.render() reads the interface list at d['ifstats']['interfaces'] and reads d['path_table'], d['link_count'] and d['interface_count'] from the top level of the map, alongside transport_enabled, transport_identity, transport_uptime, traffic_rxb, traffic_txb, speed_rx, speed_tx, speed_rx_inst, speed_tx_inst and memory_used. It builds its per-interface path and link counts itself from the top-level path_table, so an interface entry needs only its own descriptor fields, of which name is required because render() keys its interface dictionary by it. Any missing key raises KeyError inside a bare except that logs and returns nothing, so the client discards the entire sensor rather than showing the fields it did receive."
  consequence: "A transport map that is wrong in one key is not partly wrong: the operator sees no Reticulum Transport row at all, and the log line names only the missing key, so repairing one reported KeyError can simply expose the next. All four were missing or misplaced on this node, and fixing only the nested list would have moved the failure from 'interfaces' to 'path_table'. Any node implementing this sensor has to match the whole map shape, not the parts a client happens to complain about."
  sources:
    - "Sideband 2.2.0 sbapp/sideband/sense.py: RNSTransport.render(), including the d['ifstats']['interfaces'] loop and the seventeen-value dictionary it builds"
    - "Sideband 2.2.0 sbapp/ui/objectdetails.py: the 'Reticulum Transport' branch, using interface_count, link_count, path_count and the traffic fields"
    - "ports/nanoesp32/main/node.cpp: telemetry_pack()'s SID_RNS_TRANSPORT block"
  verification: "Measured on this board 2026-11-01, in both directions. Before the repair the client logged KeyError 'interfaces' and produced no render; after it, the node's 752-byte reply parsed through Telemeter.from_packed() produced a complete Reticulum Transport render with all seventeen values, including an interface dictionary keyed by 'UDP [TinyNode]'."

- record_id: MSG-001
  kind: EXTERNAL
  topic_id: MSG
  title: "MsgPack 0.4.2 is vendored to expose two private members"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "MsgPack 0.4.2 (MIT) is vendored with Packer::packRawBytes (both overloads) and Unpacker::indices made public. microLXMF's LXMessage.cpp calls packRawBytes() to append pre-formed bytes and reads Unpacker::indices for field offsets, and 0.4.2 keeps both private."
  consequence: "The change is the smallest one that keeps microLXMF itself unmodified, which keeps that tree at its upstream commit. Upstream's CI only builds that path for native targets, so the Arduino/ESP32 breakage is not something upstream would see, and a future MsgPack update should be checked for these members before re-vendoring."
  sources:
    - "VENDORED.md, third table row and the 'Why the MsgPack patch' section"
    - "microLXMF/src/LXMF/LXMessage.cpp: the packRawBytes() call and the indices access"
    - "MsgPack-vendored/MsgPack/Packer.h, Unpacker.h: the members' visibility"
  verification: "Read from the vendored trees; the firmware builds and LXMF messages encode and decode on hardware, which exercises both members."

- record_id: RADIO-001
  kind: DEVICE
  topic_id: RADIO
  title: "The radio is a REYAX RYLR998: an AT-command UART module"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "The LoRa radio is a REYAX RYLR998, an 868/915 MHz module driven entirely by AT commands over one UART, with no SPI path and therefore no RadioLib or radio-register access. The module on this bench reports RYLR998_REYAX_V1.2.4."
  consequence: "The module carries text, not bytes, which is what forces base64 framing (FRM-002), and every transmission costs a command and a response. A missing module is not fatal: the firmware logs '!! no LoRa: continuing on WiFi only', skips registration and boots normally."
  sources:
    - "README.md, 'LoRa (RYLR998 on the second UART)'"
    - "nano-node/src/lora.cpp: the header comment describing the AT state machine, and the AT+VER? probe"
    - "The 2026-10-26 hardware run printing RYLR998_REYAX_V1.2.4"
  verification: "Probe and configuration were exercised against the real module on 2026-10-26 and the version string was captured."

- record_id: RADIO-002
  kind: DEVICE
  topic_id: RADIO
  title: "The settings both ends must agree on"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "The driver asks for address 1, network id 6, peer address 0 (broadcast), spreading factor 7, bandwidth index 7 (125 kHz), coding rate 1 (4/5), preamble 12, RF power 14 dBm, and 115200 8N1 on the UART. The band is left at whatever the module holds, factory 915 MHz, unless -DLORA_BAND_HZ is set."
  consequence: "Two modules only talk when network id, band and air parameters match. The band is deliberately not forced by firmware, because which band may legally be used is the operator's decision; setting it makes the firmware authoritative instead. Preamble stays 12 because any network id other than 18 restricts it to the default."
  sources:
    - "nano-node/src/lora.cpp: MODULE_ADDRESS, NETWORK_ID, PEER_ADDRESS, SPREADING, BANDWIDTH_IDX, CODING_RATE, PREAMBLE, RF_POWER_DBM, MODULE_BAUD and the LORA_BAND_HZ comment"
    - "README.md, the settings table under 'Configuration, and why both ends must match'"
  verification: "The values read back from the module on hardware and survived a reboot unchanged, so the flash-wear guard is working."

- record_id: RADIO-003
  kind: PROTOCOL
  topic_id: RADIO
  title: "The AT command surface the driver uses"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Queries and setters: AT+VER?, AT+MODE=0, AT+ADDRESS? / AT+ADDRESS=<n>, AT+NETWORKID? / AT+NETWORKID=<n>, AT+BAND? / AT+BAND=<hz>,M, AT+PARAMETER? / AT+PARAMETER=<sf>,<bw>,<cr>,<pre>, AT+CRFOP? / AT+CRFOP=<dbm>. Transmission is AT+SEND=<address>,<length>,<data>, answered with +OK. Reception arrives unsolicited as +RCV=<from>,<length>,<data>,<rssi>,<snr>."
  consequence: "The far end has to implement only the +RCV half and one AT+SEND form. Anything the driver does not send -- mode changes, save commands, per-module settings -- is out of scope for the link contract and should not be assumed present."
  sources:
    - "nano-node/src/lora.cpp: the command strings at their call sites"
    - "README.md, 'The wire format'"
  verification: "Every command listed is issued by the driver on hardware during the 2026-10-26 run; the probe, the reads, the conditional writes and AT+SEND all completed."

- record_id: RADIO-004
  kind: PROTOCOL
  topic_id: RADIO
  title: "V1.2.4 answers +NETWORKID=, the guide says +NETWORK="
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "The AT guide documents AT+NETWORKID? answering +NETWORK=<id>, while module firmware V1.2.4 answers +NETWORKID=<id>. The driver accepts both forms and prints the raw reply when neither matches."
  consequence: "A strict parser written from the guide would have failed silently, and the network id would be written blind on every boot -- which is how that bug appeared. Any new parser should keep the fallback and the raw-reply diagnostic rather than trusting one documented form."
  sources:
    - "README.md, 'One firmware difference worth knowing'"
    - "nano-node/src/lora.cpp: the NETWORKID parse and the raw-reply print path"
  verification: "Observed on hardware against V1.2.4; the mismatch is documented in the README as a result of that observation."

- record_id: RADIO-005
  kind: DEVICE
  topic_id: RADIO
  title: "Settings are read back before being written, because the module's flash wears"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "Some module settings live in the module's own flash, rated 200k erase cycles. The driver reads each value first and writes only what differs, and it prints what the module actually holds so drift from the far end is visible."
  consequence: "Writing the same value on every boot is wear for nothing. A mismatch between the requested and held values appears as an extra '!! module holds ...' line under the air parameters, which is the fastest way to diagnose two modules that cannot hear each other."
  sources:
    - "README.md, 'Configuration, and why both ends must match'"
    - "nano-node/src/lora.cpp: the read-back, compare and conditional write sequence"
  verification: "Verified on hardware: address, network id, air parameters and power all survived a reboot and read back unchanged, so the guard skipped the writes."

- record_id: RADIO-006
  kind: PROTOCOL
  topic_id: RADIO
  title: "The module does not filter by address"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "The module reports every packet in its network as a +RCV line carrying the sender's address, and makes no delivery decision of its own."
  consequence: "Filtering is the host's job, and Reticulum already does it by destination hash, so no firmware-side address filtering is needed. Any future peer must expect to see traffic not addressed to it."
  sources:
    - "README.md, 'Things that will bite', second bullet"
  verification: "Observed on hardware; the statement is about the module's reported behaviour rather than an inference from a manual."

- record_id: RADIO-007
  kind: DEVICE
  topic_id: RADIO
  title: "The module is half duplex"
  status: SOURCED
  verified_date: 2026-10-27
  statement: "The module cannot receive while transmitting, so any packet arriving from the peer during a transmission is lost."
  consequence: "Lost packets during a send are expected rather than a firmware bug to chase; Reticulum's retries turn the effect into a throughput question. Pacing sends one frame at a time for the same family of reasons."
  sources:
    - "README.md, 'Things that will bite', first bullet"
    - "nano-node/src/lora.cpp: the header comment that a 240-character frame occupies the air for about 0.6 s and transmits are queued"
  verification: "Documented behaviour taken from the README, which records it as observed; it has not been isolated by a receive-during-transmit experiment on this bench, hence SOURCED."

- record_id: RADIO-008
  kind: DEVICE
  topic_id: RADIO
  title: "Electrical limits: 3.3 V only, about 140 mA peak"
  status: SOURCED
  verified_date: 2026-10-27
  statement: "The module's VDD range is 2.3-3.6 V and its UART pins are not 5 V tolerant. It draws about 140 mA while transmitting at 22 dBm. 22 dBm is the module maximum and 14 dBm is the CE ceiling."
  consequence: "The module runs from 3V3 and never from VBUS/5V, on the same rule as the RTC. The power draw matters because it shares the board's regulator with the sensors and the radio."
  sources:
    - "README.md, LoRa 'Wiring' table and power row, and the settings table's Power row"
    - "nano-node/src/lora.cpp: RF_POWER_DBM 14 with the comment 'CE ceiling; 22 is the module max'"
  verification: "Taken from the README's wiring table, which cites the module's datasheet; the electrical figures were not measured on this bench."

- record_id: RADIO-009
  kind: DEVICE
  topic_id: RADIO
  title: "Transmit and reception are both proven on hardware"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Transmit was proven on 2026-10-26: two announce presses moved the counters from tx 3 / frames 5 to tx 7 / frames 13 with errors 0. Reception was proven on 2026-10-27 from a second RYLR998 at 14 dBm on a Raspberry Pi Pico 2, configured to this node's network 6 / SF7 / BW index 7 / CR 4-5 / preamble 12 at 915 MHz and address 2. Across three runs 17 frames were sent and 16 received, assembling 7 packets, with bad lines 0, overlong 0, early 0 and errors 0 throughout; the module reported rssi -16 dBm and snr 12 to 13 for the traffic."
  consequence: "The +RCV line parser, the base64 field, the multi-frame reassembler and the RSSI/SNR path are all exercised against real V1.2.4 output, so the node's receive side no longer rests on host tests alone. One frame in 17 was lost with no error recorded anywhere, and it did not stop a packet being delivered: see FRM-008, which is why any application protocol over this framing needs its own length and checksum. The -16 dBm reading means the two modules were centimetres apart and very likely overdriving each other, which is a plausible cause of that loss; separating them or lowering AT+CRFOP is the first thing to try if losses persist."
  sources:
    - "README.md, 'Verified on hardware' under the LoRa section, for the 2026-10-26 transmit counters"
    - "tools/pico_lora_sender.py: the Pico 2 sender, its AT configuration and the frames it builds"
    - "This board: the L command's counters before and after each run (core-log entry 2)"
  verification: "Transmit measured on 2026-10-26; reception measured on 2026-10-27 across three runs of 4, 9 and 4 frames, reading L before and after each. Revised 2026-10-27 (core-log entry 2): reception moved from unexercised to measured, with one intermittent frame loss."

- record_id: FRM-001
  kind: PROTOCOL
  topic_id: FRM
  title: "One AT+SEND per frame, one +RCV line per frame"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "Sending is one AT+SEND=<address>,<length>,<data> per frame, answered with +OK. Receiving is one line per frame: +RCV=<from>,<length>,<data>,<rssi>,<snr>."
  consequence: "The far end (the Tang) implements exactly these two forms and nothing else. Because a frame and a command are the same unit, back-pressure is per frame, and a lost +OK means the frame's fate is unknown rather than lost."
  sources:
    - "nano-node/include/lora_frame.h: the wire framing comment"
    - "nano-node/src/lora.cpp: the AT+SEND call site and the +RCV line parser"
    - "README.md, 'The wire format'"
  verification: "Transmit observed on hardware with the counters and errors 0; the parser is covered by host tests."

- record_id: FRM-002
  kind: PROTOCOL
  topic_id: FRM
  title: "The data field is base64, and 180 bytes fill it exactly"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The <data> field is standard base64. This is required, not decorative: AT syntax is comma-delimited and line-terminated, so a raw Reticulum packet would corrupt the command parser and be cut at the first 0x0D. 180 encoded bytes produce exactly 240 characters with no '=' padding and no comma."
  consequence: "The far end needs a base64 table and nothing more -- no hex, no escaping. 240 characters is the module's <data> ceiling, which is what fixes the frame size at 180 bytes including its header."
  sources:
    - "nano-node/include/lora_frame.h: AT_DATA_MAX 240, FRAME_MAX 180, MAX_CHUNK 179 and the rationale comment"
    - "nano-node/host-tests/test_lora.cpp: the assertion that 180 bytes encode to exactly 240 characters with no padding"
  verification: "Asserted by the host test suite, which passed on 2026-10-27."

- record_id: FRM-003
  kind: PROTOCOL
  topic_id: FRM
  title: "The header byte is NNNN M III"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Byte 0 of each frame is NNNN M III: four bits of per-frame random nonce so identical packets do not look identical on air, one bit set while more frames follow for this message, and three bits of rolling message id 0-7. Bytes 1 onward carry up to 179 bytes of Reticulum packet."
  consequence: "The nonce costs nothing and defeats trivial traffic-pattern comparison. The message id is what lets the receiver interleave-safe reassemble a message, and the more-flag is the only end-of-message signal, so a frame that loses its final bit strands the partial until the timeout."
  sources:
    - "nano-node/include/lora_frame.h: HDR_NONCE_MASK 0xF0, HDR_MORE 0x08, HDR_ID_MASK 0x07, makeHeader() and the accessors"
  verification: "Header construction and parsing are covered by the host tests, including the more-flag and id handling across multi-frame messages."

- record_id: FRM-004
  kind: PROTOCOL
  topic_id: FRM
  title: "A 500-byte packet is three frames; a partial dies after 8 s"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "A Reticulum packet is at most 500 bytes (RNS-002), so a full packet travels as three frames of 179 + 179 + 142 payload bytes. The receiver appends frames carrying the same message id until one arrives with the more-flag clear. A partial message is dropped after 8000 ms without its remaining frames."
  consequence: "One lost frame cannot swallow the next message that reuses the id, and the timeout bounds how long a stranded partial occupies memory. Anything larger than 500 bytes is refused rather than silently truncated."
  sources:
    - "nano-node/include/lora_frame.h: MAX_MESSAGE 500, MAX_CHUNK 179 and REASSEMBLY_TIMEOUT_MS 8000"
    - "nano-node/host-tests/test_lora.cpp: the three-frame 500-byte case, the lost-frame cases and the timeout boundary"
  verification: "Covered by host tests that assert exactly three frames for 500 bytes, correct reassembly, that a lost first frame does not prevent the next message, and that a partial survives just under the timeout and is dropped just past it."

- record_id: FRM-005
  kind: PROTOCOL
  topic_id: FRM
  title: "Roughly 400 B/s of Reticulum traffic at SF7/125 kHz"
  status: SOURCED
  verified_date: 2026-10-27
  statement: "The module's own SF7/125 kHz test point carries about 550 B/s of <data>. Base64 costs a third, leaving roughly 400 B/s of Reticulum traffic, and a full-MTU packet is about 1.5 s of airtime."
  consequence: "This is why replies are kept under 63 bytes (LXMF-004) and why status, at about four frames, is the slow command. It also sets the expectation that LoRa is for commands and telemetry, not for bulk transfer."
  sources:
    - "README.md, 'Rough throughput' paragraph"
    - "nano-node/src/lora.cpp: bitrate_for() computing the interface bitrate from SF, bandwidth and coding rate"
  verification: "Derived from the datasheet test point and the base64 overhead; not measured end to end on this bench, so SOURCED."

- record_id: FRM-006
  kind: PROTOCOL
  topic_id: FRM
  title: "An AT error drops the rest of that packet"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "An +ERR=n reply causes the remainder of that packet to be dropped rather than spending airtime on a message the peer can never assemble. Errors are counted and reported by the L command."
  consequence: "A failed frame ends the message immediately instead of leaving a partial to rot until the 8 s timeout, which keeps the channel clear. The error counters are the way to notice the module rejecting commands at all."
  sources:
    - "README.md, 'Things that will bite', 'The AT parser is the weak point'"
    - "nano-node/src/lora.cpp: the error path and its counter"
  verification: "Visible in the driver's control flow and in the L report, which prints 'errors 0' on a healthy run; the error path itself was not provoked deliberately on hardware."

- record_id: FRM-007
  kind: PROTOCOL
  topic_id: FRM
  title: "The UART, not the radio, is the bottleneck at these sizes"
  status: SOURCED
  verified_date: 2026-10-27
  statement: "At these payload sizes the 115200 baud UART, not the radio, limits throughput: a 260-character command occupies about 23 ms of wire time before anything is on air."
  consequence: "Raising the module's UART rate would help more than air-parameter tuning for short commands. It also explains why one media access at a time in the driver loop is acceptable: the wire time dominates."
  sources:
    - "README.md, 'Things that will bite', 'The UART is a bottleneck, not the radio'"
    - "nano-node/src/lora.cpp: MODULE_BAUD 115200"
  verification: "Arithmetic from the documented baud rate and command length; not instrumented on this bench, hence SOURCED."

- record_id: FRM-008
  kind: PROTOCOL
  topic_id: FRM
  title: "A lost frame still completes the message, silently and short"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The frame header carries a nonce nibble, a more-frames flag and a rolling message id, and nothing else: there is no length field and no checksum. The receiver appends frames carrying the same id until one arrives with the more flag clear, so a message that loses a middle frame is delivered as a complete message made of the frames that did arrive. Measured on 2026-10-27: a 500-byte message sent as three frames of 179 + 179 + 142 with one frame lost arrived as a packet of 321 bytes, and the driver counted it as a delivered packet rather than a damaged one."
  consequence: "For Reticulum traffic the damage is caught downstream, because the packet's own CRC fails and the stack discards it. For anything carried above this framing the loss is invisible, so the Tang protocol must add its own length and checksum to the application payload rather than trusting the reassembler to have delivered a whole message. A caller that wants to know whether a message was complete cannot ask this layer."
  sources:
    - "nano-node/include/lora_frame.h: the header byte's three fields and the reassembly comment"
    - "nano-node/src/lora.cpp: the reassembler push and the packet counter increment on completion"
    - "tools/pico_lora_sender.py and the L counters from the 2026-10-27 runs (core-log entry 2)"
  verification: "Observed once on 2026-10-27: 4 frames sent, 3 received, 2 packets assembled, the second of 321 bytes where 500 were sent. Two further runs delivered every frame, so the loss is intermittent rather than a property of the pacing; what the record states is the behaviour when a frame is missing, which follows from the format and was seen in practice."

- record_id: FRM-009
  kind: PROTOCOL
  topic_id: FRM
  title: "The Tang speaks an application protocol above the framing"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Every message between the Tang and this node is one application message carried inside the LoRa framing: a version byte (1), a type (requests 0x01-0x0f, replies the same value with 0x80 set), a sequence number chosen by the Tang, a reserved byte, the body length big-endian, the body, and a CRC-16/CCITT-FALSE over everything before it. Requests are status, command, messages, body and send; the body of a command request is one of the node's own command words. A full body makes exactly one Reticulum MTU, 500 bytes, so an application message never spans two packets."
  consequence: "This is the answer to FRM-008: the length and the CRC are what make a message that lost a middle frame detectable, and nano-node/host-tests/test_tang_proto.cpp asserts the refusals -- short buffer, one byte short, a lying length, a flipped bit in the body and in the header, and an unknown version even when its CRC is valid. The codec is plain C in nano-node/src/tang_proto.c so the same two files serve both ends; the Tang's copy is ports/bl616/tang_proto.c. Replies carry the request's sequence, which is what lets the Tang recognise a late answer to an earlier attempt and treat a duplicate as harmless."
  sources:
    - "This project's nano-node/src/tang_proto.h, tang_proto.cpp and host-tests/test_tang_proto.cpp"
    - "This project's nano-node/src/tang_link.cpp: the request handler, and main.cpp's tanglink::node_answer()"
    - "The Tang's ports/bl616/tang_proto.c and tang_lora.c, which carry the same protocol and the client"
  verification: "Measured on 2026-10-27 over the radio: the Tang asked for status, env, gps, ping, time, help and an unknown word, and each reply decoded with a valid CRC and the matching sequence; a reply to an unknown command came back as 'unknown command; send help' (core-log entry 3)."

- record_id: FRM-010
  kind: PROTOCOL
  topic_id: FRM
  title: "The Tang is told apart by the sender address, not by a frame type"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The RYLR998 reports the sender's address in every +RCV line, and this node's driver now uses it: a frame whose sender is address 2 is an application message for the Tang protocol, and every other frame is handed to Reticulum as a packet exactly as before. Replies to the Tang are addressed to 2; Reticulum traffic keeps going to broadcast address 0."
  consequence: "The LoRa interface stays registered with Reticulum, so the node remains a full Reticulum node that announces, answers path requests and bridges the LAN and the radio, while the radio also carries the Tang link. The Tang initiates every exchange; the node sends to address 2 only in reply. The cost is that the node's own Reticulum transmissions share the air: the Tang's client must discard anything that is not a reply to its own sequence and retry, which is what it does."
  sources:
    - "This project's nano-node/src/lora.cpp: the per-packet destination address, send_to_tang(), and the sender-address branch on reception"
    - "core-reference RADIO-006: the module does not filter by address"
    - "The Tang's ports/bl616/tang_lora.c: wait_reply(), which discards any message that is not the reply it asked for"
  verification: "Measured on 2026-10-27: with the Tang's core loaded and the module on PMOD0, `node status` and five other commands answered from the node while its own Reticulum stack reported 35 known paths, so both users of the radio were live at once (core-log entry 3)."

- record_id: GPS-005
  kind: DEVICE
  topic_id: GPS
  title: "A Mini GPS PA1010D polled over I2C at 0x10"
  status: VERIFIED
  verified_date: 2026-10-30
  statement: "The receiver is an Adafruit Mini GPS PA1010D, an MTK3333 module that answers at I2C address 0x10 on SDA A4 and SCL A5. Every read returns a full chunk of NMEA 0183, padded with 0x0A when the module has nothing to send; Adafruit's own drivers read 32 bytes at a time and discard a line feed that does not follow a carriage return. It reports at 1 Hz, both $GP and $GN talkers, over GPS, GLONASS, GALILEO and QZSS. Its I2C pins are level shifted with 10K pull-ups to VIN, and it draws about 30 mA."
  consequence: "The driver in nano-node/src/gps.cpp polls the module from loop(), rate-limited to one read every 5 ms, and feeds the bytes through the same sentence assembler the retired UART receiver used, so the parsed state, the fix snapshot and the UTC extraction are unchanged. The filler rule is load-bearing: without it an idle bus reads as a stream of empty sentences and a sentence split across two reads can be cut in half. VIN is taken from 3V3, not 5V, to keep the pull-ups from putting 5 V on a 3.3 V bus (SEN-001)."
  sources:
    - "Adafruit Mini GPS PA1010D learn guide, 'Pinouts': VIN 3-5 VDC, 30 mA, level-shifted I2C with 10K pull-ups to VIN, PPS output on fix"
    - "adafruit/Adafruit_GPS, src/Adafruit_GPS.h: GPS_DEFAULT_I2C_ADDR 0x10 and GPS_MAX_I2C_TRANSFER 32; src/Adafruit_GPS.cpp: the I2C begin() ACK probe"
    - "adafruit/Adafruit_CircuitPython_GPS, adafruit_gps.py: GPS_I2C._fill() discarding a 0x0A that does not follow 0x0D, documented as the GTop I2C protocol's idle filler"
    - "nano-node/src/gps.cpp: GPS_I2C_ADDR, I2C_CHUNK, READ_MIN_MS and the filler test in poll()"
  verification: "On hardware 2026-10-30 the module answered an ACK probe at 0x10, appeared in the firmware's boot-time I2C scan as 0x10 alongside 0x44 and 0x68, and its sentences reached the parser: status reported 'no fix, 3 in view, 130s searching' indoors. The filler and split-sentence behaviour is covered by nano-node/host-tests/test_gps.cpp, which hands out 0x0A padding between the halves of a sentence and requires the fix to land. Revised 2026-11-01 (core-log entry 11): this record's revision note carried an entry-number citation that pointed at the TinyDesk project's log rather than this one, and the bench observation behind it came from the TinyDesk bench, a different node, where a module was silent at 0x10 while the other I2C devices answered because of a short in the module rather than the bus. The PA1010D on this node was working on 2026-10-30 and has not failed here, so that observation is recorded as another bench's and not as this module's behaviour. The chain is pass-through -- STEMMA QT V+ and the data lines are common across both connectors -- so devices downstream of a faulty one still work, which makes a silent 0x10 easy to misread as a bus fault. The chip runs from the board's own ultra-low-dropout regulator, fed from the same VCC net the downstream devices tap directly, so a shorted module drags the shared rail instead of only itself."

- record_id: GPS-002
  kind: DEVICE
  topic_id: GPS
  title: "Time hierarchy: GPS, then NTP, then the DS3231"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "With a valid fix the DS3231 is corrected from RMC UTC whenever it is more than 2 s out. With WiFi but no fix, NTP is the reference. Otherwise the DS3231 carries time at its own drift rate. An NTP value below 1600000000 is treated as unsynchronised and ignored."
  consequence: "Time neither needs the sky nor reverts to 1980 after a power cycle, and a stale fix is never allowed to steer the RTC. The 2 s threshold keeps a jittery fix from rewriting the RTC constantly, and the epoch floor keeps a pre-sync SNTP default from being adopted."
  sources:
    - "README.md, 'Ultimate GPS (PA1616D) on Serial1' and the RTC section"
    - "nano-node/src/main.cpp: the sync function's source selection, the 2 s delta threshold and the 1600000000 floor"
  verification: "Observed on hardware: the RTC holds across reboots, is corrected after a fix, and the sync source is reported by the clock command."

- record_id: GPS-003
  kind: DEVICE
  topic_id: GPS
  title: "PPS is measured, not yet used; an unwired pad floats"
  status: VERIFIED
  verified_date: 2026-10-30
  statement: "The pulse-per-second output is captured on D4 (GPIO 7) by a rising-edge interrupt. The firmware reports the last interval, the mean of the recent 16 intervals, the pulse count and the number of intervals more than 2 ms off nominal. The PA1010D offers this pulse on a pad that pulses high for 50-100 ms once a second while it has a fix; as fitted on 2026-10-30 that pad is not connected."
  consequence: "With a real pulse the outlier count is the interesting figure: it is dominated by interrupt latency, mostly flash-cache stalls during FFat writes. With the pad unwired D4 floats, and the counter then reports several hertz of irregular intervals -- 1134 intervals in 130 s, mean 657 ms -- which is noise and must not be read as a timing reference. The pps command reports its numbers either way, so the rate is the tell."
  sources:
    - "nano-node/include/pps.h: the wiring, the Stats struct and the 'using it to discipline the RTC is a separate step' note"
    - "README.md, 'GPS PPS on D4 (GPIO 7)'"
    - "Adafruit Mini GPS PA1010D learn guide, 'Pinouts': the PPS pad is low and pulses high 3.3 V once a second for 50-100 ms while there is a fix"
  verification: "Observed on hardware. With the retired Ultimate GPS wired to D4 the interval was avg 1000.000 ms with a handful of outliers; on 2026-10-30, with the PA1010D fitted and its PPS pad unconnected, the same command reported n=1134 over a 130 s uptime and an average of 657 ms, which is a floating pin rather than a pulse train. Revised 2026-10-30 (core-log entry 10): the receiver changed to the PA1010D, whose PPS pad is not wired, and the floating-pin reading is now recorded as a negative fact."

- record_id: GPS-004
  kind: DEVICE
  topic_id: GPS
  title: "The NMEA timestamp lags the second it describes"
  status: SOURCED
  verified_date: 2026-10-27
  statement: "An NMEA sentence identifies the second it describes, and is only available after the receiver has produced it. With the retired UART receiver the sentence for a second arrived 50-80 ms after that second; with the PA1010D polled over I2C the delay is instead bounded by the poll interval, which is capped at one read every 5 ms."
  consequence: "The sentence is good enough to identify which second it is and to steer the RTC, but not to time one. That is why a PPS edge would be two orders of magnitude better as a timing reference, and why utc_epoch() adds the whole seconds elapsed since the sentence arrived."
  sources:
    - "nano-node/include/pps.h: the same comparison in the header comment"
    - "nano-node/src/gps.cpp: READ_MIN_MS and the utc_epoch() age correction"
  verification: "The 50-80 ms UART figure came from the README's hardware notes and was never independently measured, hence SOURCED. Revised 2026-10-30 (core-log entry 10): the receiver is now polled over I2C and the UART delay no longer applies; the I2C figure has not been measured."

- record_id: SEN-001
  kind: DEVICE
  topic_id: SEN
  title: "DS3231 at 0x68 on A4/A5, powered from 3V3"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The RTC is an Adafruit DS3231 breakout at I2C address 0x68 (RTClib's DS3231_ADDRESS) on SDA A4 and SCL A5. It is powered from 3V3 and not VBUS/5V, because the breakout's pull-ups would put 5 V on a 3.3 V bus. A CR1220 keeps the clock across power loss; without it, lostPower() reports true and the firmware seeds the clock from the firmware build time."
  consequence: "The 3V3 restriction is a bus-protection rule, not a preference: 5 V pull-ups against a 3.3 V ESP32 could damage the part. Losing the coin cell degrades to build-time seeding, which keeps the stores usable (FLS-005) but wrong by however long the firmware has existed."
  sources:
    - "nano-node/.pio/libdeps/nano-esp32/RTClib/src/RTC_DS3231.cpp: #define DS3231_ADDRESS 0x68"
    - "README.md, 'RTC: Adafruit DS3231 on I2C'"
    - "nano-node/src/main.cpp: init_rtc(), lostPower() handling and the build-time seed"
  verification: "The device answers on hardware, the clock survives reboots with a cell fitted, and the lost-power path was seen and logs 'DS3231 lost power'."

- record_id: SEN-002
  kind: DEVICE
  topic_id: SEN
  title: "LIS3MDL magnetometer at 0x1C"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The magnetometer is an LIS3MDL answering at 0x1C with WHO_AM_I 0x3D. Register map per DS9463 Rev 7: CTRL_REG1 20h, CTRL_REG2 21h with the full-scale bits at 6 and 4, CTRL_REG3 22h, CTRL_REG4 23h, CTRL_REG5 24h with BDU, STATUS_REG 27h, and little-endian signed 16-bit outputs at 28h-2Dh."
  consequence: "The device shares the DS3231 bus and needs no separate wiring. The full-scale bits sit at 6 and 4 rather than adjacently, which is the kind of detail that produces a silently wrong scale if read from a different part's map; the register map lives in the header for that reason."
  sources:
    - "nano-node/include/mag.h: the register table attributed to DS9463 Rev 7, including the FS bit positions and WHO_AM_I 0x3D"
    - "nano-node/src/mag.cpp: the probe that rejects a wrong identity"
  verification: "The device probes on hardware and reports a field magnitude near 48 uT, which is the geomagnetic sanity check; counts-to-microtesla conversion is covered by host tests."

- record_id: SEN-003
  kind: DEVICE
  topic_id: SEN
  title: "SHT45 at 0x44: 0xFD measurement, CRC-8 per word"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The temperature/humidity sensor is an SHT45 at 0x44. Command 0xFD measures both at high repeatability in up to 8.3 ms and returns six bytes: T msb, T lsb, T crc, RH msb, RH lsb, RH crc. T = -45 + 175 * ticks / 65535 in degrees Celsius; RH = -6 + 125 * ticks / 65535 percent, clamped to 0-100. Each 16-bit word carries a CRC-8 with polynomial 0x31 and init 0xFF."
  consequence: "A frame failing either CRC is discarded rather than reported, so a corrupted reading never reaches a reply. The clamping is required because the linear form can exceed 100 percent at the top of the range."
  sources:
    - "nano-node/include/sht.h: the command, the formulas, the CRC parameters and the byte order"
    - "nano-node/host-tests/test_sht.cpp: CRC and conversion coverage"
  verification: "The formulas, CRC and clamping are asserted by host tests that pass; the device itself answers on hardware and reports plausible room readings."

- record_id: SEN-004
  kind: DEVICE
  topic_id: SEN
  title: "The SHT45 reading is cached, because a measurement blocks the bus"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "A high-precision SHT45 measurement takes up to 8.3 ms. The main loop therefore refreshes a cached reading every two seconds, and the status path only formats the cached value."
  consequence: "An LXMF reply never stalls on the sensor bus, which matters because replies are delivered over a link that times out. The cost is that status can report a value up to two seconds old, which is acceptable for environment data and is why the design exists."
  sources:
    - "nano-node/include/sht.h: 'Measurement is 8.3 ms in the worst case, so it is not performed inside the status handler'"
    - "README.md, 'SHT45 temperature/humidity on I2C at 0x44'"
    - "nano-node/src/sht.cpp: the poll cadence and cached value"
  verification: "The behaviour is visible in the code path and the reason is a documented device timing limit; the reply latency improvement was the motivation recorded in the README."

- record_id: SEN-005
  kind: DEVICE
  topic_id: SEN
  title: "Heading is relative to a declared bearing; calibration is invalidated by moving"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Heading is reported relative to a bearing declared with h<bearing> on the console or cal heading <deg> over LXMF, which absorbs magnetic declination without needing a declination table. Hard-iron offsets come from a rotation calibration that must tumble the unit through every orientation including inverted; both the offsets and the heading reference persist in /mag_cal."
  consequence: "Moving the node invalidates the hard-iron fit, because the offsets describe the metal around the board. A flat spin leaves the Z axis with no span and the fit is rejected as POOR FIT, leaving the stored offsets untouched. During the calibration window mag and status report nonsense, which is expected rather than a fault."
  sources:
    - "README.md, 'LIS3MDL magnetometer on I2C at 0x1C' and the 'Calibration' section"
    - "nano-node/src/main.cpp: MAG_CAL_FILE '/mag_cal', load_mag_cal(), save_mag_cal() and the heading-reference path"
  verification: "Exercised on hardware: a 30 s tumble produced 'mag calibrated: n=67983 span 6163/6141/6061, radius ~44.7 uT', and a flat spin is rejected."

- record_id: SEN-006
  kind: DEVICE
  topic_id: SEN
  title: "The BMP581 answers at 0x47 with chip id 0x50"
  status: SOURCED
  verified_date: 2026-11-01
  statement: "A Bosch BMP581 has been identified at I2C address 0x47 on the TinyDesk bench, chip-id register 0x01 returning 0x50. It reports calibrated pressure and temperature directly, with no PROM coefficient read like the MS5611 requires."
  consequence: "If this node's own 0x47 device is the same part, it is the SID_PRESSURE source for the node's telemetry (LXMF-008) and a second SID_TEMPERATURE source. This node has had a device answering at 0x47 since 2026-10-30 but its chip id has never been read here, so that identification is an open one-register test rather than an established fact."
  sources:
    - "Read of register 0x01 at address 0x47, returning 0x50, on 2026-11-01 from the TinyDesk node firmware on an Arduino Nano ESP32"
    - "Bosch BMP581 chip-id value 0x50 for register 0x01"
    - "core-log entry 10: a device answering at 0x47 on this node with no driver and no chip-id read"
  verification: "Observed on the TinyDesk bench on 2026-11-01, where the firmware logged 'i2c: BMP581 chip_id=0x50 (expect 0x50)'. Not confirmed on this node: core-log entry 10 records an undriven device answering at 0x47 here without a chip-id read, and the operator knows which pressure sensor was fitted to this board. Revised 2026-11-01 (core-log entry 11): the record was VERIFIED against another bench's hardware and asserted that it explained this node's 0x47; it is now SOURCED, states that the identification came from the TinyDesk bench, and leaves the chip-id read on this board as the confirming step."

- record_id: SEN-007
  kind: DEVICE
  topic_id: SEN
  title: "The SHT45 has no chip-id register"
  status: SOURCED
  verified_date: 2026-11-01
  statement: "The Sensirion SHT45 at 0x44 exposes no identification register; it cannot be confirmed by a chip-id read the way the LIS3MDL (0x3d at 0x0F) and BMP581 (0x50 at 0x01) can. Presence is established by an ACK, and a reading by issuing the measure command 0xFD and checking the CRC."
  consequence: "A bus-identification routine must not treat a missing chip id as a missing device. This project's identify step reports the SHT45 by an ACK plus a successful measurement instead, and a silent 0x44 in a scan is the failure signal."
  sources:
    - "SEN-003: the SHT45 answers at 0x44 and returns six bytes for command 0xFD, each 16-bit word CRC-8 protected, with no id read in that command set"
    - "Absence of an id register observed on the TinyDesk bench on 2026-11-01, where the address ACKs in the scan but a chip-id read is not applicable"
  verification: "The SHT45 answers and measures on this node: core-log entry 10 records 25.4 C and 43.7 percent RH, which confirms presence and the measurement path but says nothing about an id register, because none was attempted here. The absence of one is derived from Sensirion's SHT4x command set as recorded in SEN-003 and from the other bench's identification routine, so it is SOURCED rather than VERIFIED. Revised 2026-11-01 (core-log entry 11): the record was VERIFIED against another bench's hardware; it is now SOURCED with the derivation stated."
- record_id: SEN-008
  kind: DEVICE
  topic_id: SEN
  title: "This board's magnetometer needs a soft-iron matrix, not three offsets"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "The geomagnetic field at the bench, from IGRF-14 evaluated at 33.2939 N, -111.8810 E, 377 m for 2026, is 46.6 uT total: 23.8 uT horizontal, 40.0 uT vertical, dip 59.3 degrees, declination 9.6 degrees east. Uncorrected, the LIS3MDL on this board reads magnitudes from about 13 to 72 uT depending on orientation. Subtracting three hard-iron offsets narrows that to about 35 to 65 uT, and the samples then still scatter 2.2 uT about the best sphere, roughly six times the sensor's own noise floor, which means they lie on an ellipsoid rather than a sphere. Fitting the general quadric v M v + 2 n v = 1 and applying sqrt(M) after removing the centre brings the spread to about 2 uT."
  consequence: "A three-number offset correction cannot describe this board's distortion, whatever estimator computes it: offsets shift a sphere's centre and cannot reshape an ellipsoid, which is why every offset-only calibration left |B| varying by 16 to 32 percent. The firmware stores a 3x3 matrix plus a centre instead. Any future sensor moved to this mounting inherits the same distortion; a board with only hard iron still works with offsets alone, which is what the three-integer file format selects."
  sources:
    - "IGRF-14 coefficients from https://www.ngdc.noaa.gov/IAGA/vmod/coeffs/igrf14coeffs.txt, evaluated with pyIGRF at the bench coordinates; checked against London 49.1 uT, the equator 26.8 uT with 9.3 degrees dip, and the south pole with negative Z before use"
    - "ports/nanoesp32/main/sensors.cpp: ellipsoid_fit(), the 9-unknown quadric solve, jacobi3() for sqrt(M), and mag_read() applying s_corr"
    - "A prototype of the same algebra, run against a known distortion with correct rotation matrices: 59.0 uT raw spread, 29.9 uT with offsets, 2.2 uT with the matrix, centre recovered to 0.04 uT"
  verification: "Measured on this board 2026-11-01. With the matrix applied, an on-board rotating sample reported under 3 percent spread, and a host check across orientations the board had visibly moved between read 45.21 to 46.11 uT, a 2.0 percent spread against the 46.6 uT field. The offset-only stage was measured at 16 to 32 percent spread in several runs."

- record_id: SEN-009
  kind: DEVICE
  topic_id: SEN
  title: "Multi-byte register reads need the sub-address MSB set"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "Reading more than one register from the LIS3MDL and BMP581 relies on the device auto-incrementing its pointer, which is requested by setting the MSB of the register address before the transaction: the standalone node writes reg | 0x80 for its six-byte output reads. A single-byte read takes the plain address."
  consequence: "A driver that sends the plain address for a six-byte read gets the same register repeated, which looks like a working sensor producing a degenerate vector rather than an I2C error. On this port the first version of the shared read helper omitted the bit for every transfer; single-byte reads were unaffected, so probing and chip-id reads still succeeded and the fault was invisible."
  sources:
    - "nano-node/src/mag.cpp: read_regs() writing static_cast<uint8_t>(reg | 0x80) with the comment 'auto-increment'"
    - "ports/nanoesp32/main/sensors.cpp: rd() now setting the MSB when len > 1"
  verification: "Both parts read correct, distinct axis values after the change on this board 2026-11-01, where before the change the magnetometer data path had never been checked against a reference read."

- record_id: SEN-010
  kind: DEVICE
  topic_id: SEN
  title: "A soft-iron fit fixes the correction's shape, not its scale"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "The ellipsoid fit of section SEN-008 determines the shape of the correction but not its absolute magnitude. That magnitude follows from how much of the ellipsoid the samples covered: the fit normalises the corrected radius to whatever surface the points actually lie on, so a tumble that reaches only part of the way around leaves the fitted surface small and every subsequently corrected vector longer than the true field. Measured on this board, a run covering 92, 86 and 74 percent of the three axes produced a correction that read 53.4 uT against the 46.6 uT field, 14 percent high, while its dip was already correct at 58.6 degrees. Multiplying the matrix by the uniform factor 46.6/53.4 = 0.87251 brought the same board to 46.3 to 46.8 uT."
  consequence: "A magnitude error with the direction already correct is a gain error, not a shape error, and the two need different repairs: shape faults are answered by refitting, gain faults by one uniform scale factor. A uniform factor changes magnitude only, so bearing and dip are left untouched and the soft-iron asymmetry the fit found survives it. Rescaling each axis separately toward the LIS3MDL's nominal 68.42 counts/uT would erase exactly the information the matrix exists to carry, which is why `magcal gain` scales all nine coefficients by one number. The fit's own coverage gate does not catch this, because partial coverage still fits cleanly to a small ellipsoid."
  sources:
    - "ports/nanoesp32/main/sensors.cpp: ellipsoid_fit(), which scales the correction by target_ut / sqrt(k) against the surface the samples define, and mag_cal_rescale(), which applies the uniform factor"
    - "ports/nanoesp32/main/node.cpp: cmd_magcal's 'gain' branch, averaging eight half-second samples before scaling and re-saving"
  verification: "Measured on this board 2026-11-01. In free air, scale 0.87251 took |B| from 53.3 to 46.3 uT while the bearing moved 1.6 degrees, and five subsequent readings held 46.3 to 46.8 uT at a dip of 60.3 degrees against the IGRF figures of 46.6 uT and 59.3 degrees recorded in SEN-008."

- record_id: SEN-011
  kind: DEVICE
  topic_id: SEN
  title: "The bench desk is magnetic enough to bias a calibration"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "The same board, with one calibration and one gain, read 46.3 to 46.8 uT at a dip of 60.3 degrees held clear of the desk in the air, and 44.8 to 45.9 uT at a dip of 61.6 degrees set flat on the desk a few seconds later. The desk therefore reduces the measured magnitude by about 2.6 percent and steepens the apparent dip by about 1.3 degrees. In an earlier state of the same board, before the gain was corrected, the desk steepened the dip by about 8 degrees."
  consequence: "A calibration cannot be judged, gained, or trusted in a place different from the one it describes. The desk contribution is small enough that a corrected board remains usable on it, but large enough that setting the gain there bakes the desk into the correction and makes every reading elsewhere wrong by the same amount. `magcal gain` must be run where the field is known and clean, and the resulting residual on the desk is the desk's, not the sensor's."
  sources:
    - "Measurements on this bench 2026-11-01, twelve readings from `magcal show` at 2.5 s intervals, the board held flat at the same orientation in both positions"
    - "The IGRF-14 field for the bench coordinates, recorded in SEN-008"
  verification: "Confirmed on this board 2026-11-01 after the gain correction, by readings taken in the air and then on the desk within the same minute, with no parameter changed between them."

- record_id: SEN-012
  kind: DEVICE
  topic_id: SEN
  title: "A level-fixed magnetometer gives a heading without tilt compensation"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "The magnetic heading of a two-axis magnetometer measurement is atan2 of the horizontal components of the field, corrected for hard and soft iron. That angle describes the direction the sensor faces only while the sensor is level: if the board tilts, the vertical component of the field projects into the horizontal pair and the apparent heading moves by tens of degrees, with no way to detect it from the magnetic data alone. Tilt compensation requires an accelerometer or an inclinometer to measure the gravity vector. This board carries a LIS3MDL, an SHT45, a BMP581 and a DS3231, and no accelerometer, so nothing on it can sense tilt. The magnetometer, however, is fixed to the GPS module, flat and aligned with it, and the assembly is permanently mounted in one orientation, so the board is only ever read level."
  consequence: "A magnetic heading is valid on this node because of how it is mounted rather than because of anything the firmware measures, so the firmware reports it without tilt compensation and without any indication of how level the board is. If the assembly is ever moved, re-aimed or mounted at an angle, the heading will be wrong by an amount the node cannot report, and the correct repair is to mount it level again rather than to correct the numbers. The heading is magnetic, not true: no declination is applied, because computing it would need either a fixed constant for one location or a geomagnetic model in firmware, and the operator asked for magnetic. At this bench the geomagnetic declination is 9.6 degrees east, so the heading reads that far anti-clockwise of a true bearing. Below about 5 uT of horizontal field the angle is dominated by sensor noise and is withheld rather than reported."
  sources:
    - "ports/nanoesp32/main/sensors.cpp: the snapshot fill computing atan2(y, x) from the corrected field and gating it on MAG_HEADING_MIN_UT"
    - "The IGRF-14 field and declination for this bench, recorded in SEN-008"
    - "The fitted accelerometer list for this board: SEN-002, SEN-003, SEN-006 and SEN-001, none of which senses gravity"
  verification: "Read on this board 2026-11-01: `magcal show` and the telemetry CUSTOM entry both report a heading from the corrected horizontal field, and repeated readings with the board still tracked within about 1.5 degrees of each other. The absence of tilt compensation is a property of the fitted parts rather than a measurement, and was confirmed by there being no accelerometer on the I2C bus."

- record_id: HOST-001
  kind: TOOLCHAIN
  topic_id: HOST
  title: "host-config is an RNS instance pointed at the node"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "host-config/config is a Reticulum instance with an explicit UDPInterface: listen_ip 0.0.0.0, listen_port 4242, forward_ip 255.255.255.255, forward_port 4242, with enable_transport Yes and share_instance Yes. The Sideband variant of the same block forwards to the node's address 10.10.0.35:4242 instead of broadcasting."
  consequence: "The host config and the client config are not interchangeable: one broadcasts so it can find the node, the other unicasts because it already knows the node's address. Keeping both in the README is what stops that distinction being lost."
  sources:
    - "host-config/config"
    - "README.md, the client configuration block and the Sideband block"
  verification: "Used on hardware: rnsd and the one-shot test both find the node at 1 hop using these settings."

- record_id: HOST-002
  kind: TOOLCHAIN
  topic_id: HOST
  title: "e2e_test.py owns its own RNS instance and proves delivery"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "e2e_test.py takes the destination hash printed by k, creates its own RNS instance over host-config rather than attaching to a daemon, requests a path, sends a 30-byte encrypted packet and waits for the cryptographic proof, which it treats as the pass condition."
  consequence: "Owning the instance sidesteps the shared-instance auth-key mismatch that makes rnpath and rnprobe unreliable on this box, so the test is deterministic. It is also the reference for any scripted interaction with the node."
  sources:
    - "e2e_test.py: the CONFIG_DIR/APP_NAME/APP_ASPECT constants and the path, send and proof steps"
    - "README.md, 'Verify from the host'"
  verification: "Last recorded run passed: path at 1 hop, identity recalled with a 64-byte public key, 30 bytes sent, PROOF RECEIVED with rtt 0.676 s, and the board logged the same 30-byte packet."

- record_id: HOST-003
  kind: TOOLCHAIN
  topic_id: HOST
  title: "rnsd needs setsid; the CLI tools sometimes fight the shared instance"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "rnsd must be started with setsid, otherwise it dies when the invoking shell or tool call ends. rnpath and rnprobe sometimes hit a shared-instance auth-key mismatch on this machine, while a process that owns its own instance, such as e2e_test.py, is unaffected."
  consequence: "The daemon recipe is setsid plus nohup plus a log file. When a CLI tool reports an auth-key mismatch, the fault is the attachment path, not the node, and the fastest check is to run a test that owns its instance."
  sources:
    - "README.md, 'Verify from the host' note and 'Multiple clients on one machine'"
  verification: "Observed on hardware and on this host: a setsid-launched rnsd persists after the launching tool returns, and the mismatch is reproduced with rnpath/rnprobe."

- record_id: HOST-004
  kind: TOOLCHAIN
  topic_id: HOST
  title: "Two virtualenvs: ~/.venvs/rns and ~/.venvs/pio"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The host toolchain lives in two virtualenvs: ~/.venvs/rns provides RNS and its CLI tools, and ~/.venvs/pio provides PlatformIO. The system python3 has neither."
  consequence: "Commands run with the wrong interpreter fail in ways that look like missing functionality rather than a missing environment. Any scripted use should name the interpreter explicitly, as the README's examples do."
  sources:
    - "README.md, 'Build and flash' and 'Verify from the host': the /home/vash/.venvs/pio and /home/vash/.venvs/rns paths"
  verification: "Read from the README's commands; the system python3 in this environment has no RNS module, which is consistent with the split."

- record_id: HOST-005
  kind: TOOLCHAIN
  topic_id: HOST
  title: "Four host units run without hardware"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "nano-node/host-tests/run.sh compiles and runs four units with g++ -std=c++17 against a shim include directory: gps.cpp, mag.cpp, sht.cpp and lora_frame.cpp, each with its test file. No hardware is attached and nothing is written into the repository; the binaries go to /tmp."
  consequence: "The parts that can be wrong without showing up on the board -- NMEA parsing, counts-to-microtesla conversion, CRC and conversion formulas, base64 framing and reassembly -- are testable on any Linux host. This is the cheapest regression gate in the project and the only one available when no board is attached."
  sources:
    - "nano-node/host-tests/run.sh"
    - "nano-node/host-tests/test_gps.cpp, test_mag.cpp, test_sht.cpp, test_lora.cpp and the shim directory"
  verification: "Ran on 2026-10-27 in this environment: all four units passed, RESULT: PASS, exit 0. The LoRa unit covers the three-frame 500-byte case, lost frames, timeout boundaries and malformed +RCV lines."

- record_id: HOST-006
  kind: TOOLCHAIN
  topic_id: HOST
  title: "Sideband 2.2.0 works, with an AppImage workaround on this box"
  status: VERIFIED
  verified_date: 2026-10-26
  statement: "Sideband 2.2.0 is verified against the node on the same LAN once its Reticulum instance carries an explicit UDP interface. On this machine the AppImage must be launched with --appimage-extract-and-run because FUSE mounts are blocked by the kernel's no-new-privs flag."
  consequence: "A normal desktop session may not need the flag, so it is a bench workaround rather than a requirement. The UDP interface entry is the real requirement and applies everywhere (NET-003)."
  sources:
    - "README.md, 'Chatting from Sideband'"
  verification: "Verified on hardware with Sideband 2.2.0 exchanging messages with the node."

- record_id: HOST-007
  kind: TOOLCHAIN
  topic_id: HOST
  title: "One identity, one router, and the instance's own config directory"
  status: VERIFIED
  verified_date: 2026-10-30
  statement: "A program that attaches to a running Reticulum shared instance authenticates its RPC calls against the instance's transport identity, which it reads from its own config directory; pointing it at a different directory makes every RPC fail with 'digest sent was rejected' and no destination-addressed packet is delivered to it, although link proofs still arrive. Separately, an identity may have only one LXMF router at a time: a second router on the same identity announces its own ratchet for the same destination, so a reply encrypted against one router's ratchet is discarded by the other with no error on either side while the sender still reports the message sent."
  consequence: "Tools that join the mesh from this host must load ~/.reticulum, which is where Sideband's instance keeps its transport identity, and must refuse to start twice. tools/agent_endpoint.py does both: CONFIG_DIR defaults there and take_lock() holds an flock for the life of the process. A failure of either rule looks like a working sender with a silent receiver, which is why neither is left to the operator's memory."
  sources:
    - "RNS 1.5.7 / LXMF 1.2.1 shared-instance behaviour, observed on this bench 2026-10-30 with Sideband 2.2.0 holding 0.0.0.0:4242 and client RPC replies reading 'Shared instance RPC failed while setting destination data use: digest sent was rejected'"
    - "tools/agent_endpoint.py: CONFIG_DIR and take_lock()"
    - "README.md, 'A persistent agent endpoint'"
  verification: "Both halves reproduced on 2026-10-30. The config-directory half: a client loaded host-config/ while Sideband owned the port, logged the digest rejection on every RPC, and never received a destination-addressed reply, while proofs for its own outbound messages arrived normally. The one-router half: two endpoint processes sharing one identity made every node reply disappear while the node's console reported 'Message sent successfully via link'; killing the duplicate and leaving one process made the same command answer within seconds, with the node confirming 'DIRECT delivery confirmed'."

- record_id: HOST-008
  kind: TOOLCHAIN
  topic_id: HOST
  title: "A hand-held calibration cannot be judged by polling telemetry from a host"
  status: VERIFIED
  verified_date: 2026-11-01
  statement: "An LXMF telemetry request and reply over UDP takes about three seconds, so polling from the host gathers roughly twenty samples a minute. Judging a magnetometer calibration needs the sample cloud, and those twenty points from a minute of turning are far too clustered to fit a sphere to. The node's own poll runs at 20 Hz, so the same minute produces over a thousand points."
  consequence: "Calibration and its verification belong on the board, behind a command that samples at full rate, not in a host script. Two sets of host-side measurements during this cycle reported 'the board did not move' and produced no verdict; the same rotation measured on the board gave hundreds of samples and a clear answer."
  sources:
    - "ports/nanoesp32/main/node.cpp: the node task polling at 50 ms and mag_cal_poll()"
    - "tools/telemetry_request_test.py and the host sampling scripts, whose round trip is dominated by link setup and LXMF processing"
  verification: "Measured on this bench 2026-11-01: host polling produced 20 to 25 points per run and twice concluded the board had not moved, while `magcal check` on the same rotation collected 900 to 1400 samples and a definite verdict."

- record_id: TOOL-001
  kind: TOOLCHAIN
  topic_id: TOOL
  title: "PlatformIO environment nano-esp32"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "The build uses PlatformIO env nano-esp32: platform espressif32, board arduino_nano_esp32, framework arduino, C++17 with -fexceptions and -Wall, upload_speed 460800, monitor_speed 115200 with the esp32_exception_decoder filter, and gnu++11 unset."
  consequence: "The board definition is what supplies the partition CSV and the pin remap (BRD-003); changing the board id would change both silently. upload_speed is a bench choice and can be lowered if a cable proves marginal."
  sources:
    - "nano-node/platformio.ini"
  verification: "Read from the file, and the environment has produced firmware that runs on this board."

- record_id: TOOL-002
  kind: TOOLCHAIN
  topic_id: TOOL
  title: "Build flags that change what the firmware does"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Load-bearing flags: -DRNS_USE_FS and -DUSTORE_USE_POSIXFS (persistence, see FLS-007), -DUSTORE_USE_NOOPFS, -DMSGPACK_USE_BOOST=OFF, -DRNS_LOG_LEVEL=RNS_LOG_LEVEL_INFO (5 of 8, because TRACE is very chatty at 115200), -Iinclude, and -I../microReticulum/src plus -I../microReticulum/src/microReticulum because microLXMF includes microReticulum headers bare."
  consequence: "Two of these change observable behaviour rather than build hygiene, and their absence is invisible at runtime. The include paths are a vendoring artefact: a microLXMF that included properly would not need them."
  sources:
    - "nano-node/platformio.ini: the build_flags block"
    - "README.md, 'Build notes, if you rebuild this from scratch'"
  verification: "Read from the file; the -DRNS_USE_FS omission was the original cause of nothing being persisted, which is the recorded proof that these flags matter."

- record_id: TOOL-003
  kind: TOOLCHAIN
  topic_id: TOOL
  title: "Dependencies: two registry libraries, two GitHub libraries, three symlinks"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "lib_deps pins adafruit/RTClib ^2.1.4 and ArduinoJson ^7.4.2 from the registry, attermann's Crypto and microStore from GitHub URLs, and symlinks to the vendored MsgPack, microReticulum, microLXMF and the common udp_interface example."
  consequence: "The two GitHub dependencies are unpinned by commit, so a rebuild is not bit-reproducible without freezing them. The symlinks are what make the vendored trees the build inputs rather than any cached copy."
  sources:
    - "nano-node/platformio.ini: the lib_deps block"
  verification: "Read from the file; the vendored dependencies are present in the tree and in .pio/libdeps after a build."

- record_id: PROV-001
  kind: EXTERNAL
  topic_id: PROV
  title: "Three vendored trees, each with one recorded local change"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "microReticulum at 40fa628 (v0.5.0), Apache-2.0, changed to read WiFi credentials from wifi_config.h or -DRNS_WIFI_SSID. microLXMF at 9876dff (v0.1.0), GPL-3.0-only, unmodified. MsgPack at 0.4.2, MIT, with Packer::packRawBytes (both overloads) and Unpacker::indices made public. VENDORED.md records each upstream, commit, licence and change."
  consequence: "Because each tree carries a change needed to build here, re-vendoring is not a plain checkout: the change has to be re-applied and the reason re-checked. VENDORED.md is the checklist, and it is the reason the diff against upstream stays explainable."
  sources:
    - "VENDORED.md: the table and the three notes that follow"
    - "microReticulum/library.json, microLXMF/library.json, MsgPack-vendored/library.json: versions and licences"
  verification: "Read from the vendored manifest files and VENDORED.md in this session."

- record_id: PROV-002
  kind: EXTERNAL
  topic_id: PROV
  title: "The combined firmware image is GPL-3.0"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "microReticulum is Apache-2.0 and MsgPack is MIT, but microLXMF is GPL-3.0-only, so linking it into the firmware makes the combined image GPL-3.0."
  consequence: "Fine for personal or research use; it matters the moment the image is distributed, at which point the corresponding source has to be offered. The three permissive components impose no further condition beyond notice retention."
  sources:
    - "VENDORED.md, 'Licensing note'"
    - "microLXMF/library.json: license GPL-3.0-only"
  verification: "Read from the manifests and VENDORED.md; the licences are as stated in each tree."

- record_id: PROV-003
  kind: EXTERNAL
  topic_id: PROV
  title: "Each vendored tree keeps its own LICENSE"
  status: VERIFIED
  verified_date: 2026-10-27
  statement: "Each vendored directory carries its own LICENSE file, and its local change is documented in VENDORED.md rather than left to be reconstructed from a diff."
  consequence: "The notices travel with the code, which is what Apache-2.0 and MIT require and what GPL-3.0 requires of a distributed image. A future vendoring step must copy the LICENSE along with the sources."
  sources:
    - "VENDORED.md, closing line: 'Each vendored tree keeps its own LICENSE file.'"
    - "microReticulum/LICENSE, microLXMF/LICENSE, MsgPack-vendored/LICENSE"
  verification: "The three LICENSE files are present in the tree."
```

---

## 6. Record template

```yaml
- record_id: <TOPIC>-<NNN>
  kind: BOARD | SOC | FLASH | PROTOCOL | DEVICE | EXTERNAL | TOOLCHAIN
  topic_id: <TOPIC>
  title: "Short noun phrase"
  status: VERIFIED | SOURCED | INFERRED
  verified_date: YYYY-MM-DD
  statement: "What the source says, with exact values and units"
  consequence: "What it means for this project, or where it is implemented"
  sources:
    - "Document or repository, revision/path/symbol, or datasheet name and revision"
  verification: "How and when it was confirmed, or why it is not yet confirmed; then any 'Revised <date> (core-log entry N): ...' notes"
```

---

## 7. Maintenance boundary

- Keep external facts, their sources, and their verification status here.
- Facts shared with the Tang belong to the Tang's reference, not this file. The LoRa link has two ends: this file records the AT surface and the framing TinyNode implements (`FRM`), because both are fixed here, and it does not attempt to record the Tang's side of the contract.
- Keep build results, failures, rationale, and chronology in `core-log.md`; cite a log entry number in `verification` instead of repeating it. A build's RAM and flash percentages are log material, not reference material.
- Do not add speculative records for topics that have not been looked up. An unanswered question stays out of this file until a source answers it.
- Keep this file current rather than historical: when a record's source is replaced by a newer document or tool version, or its fact changes, revise the record in place and let git and `core-log.md` carry the history (section 1).
- A record whose status is SOURCED is a to-do with a citation: the first hardware or tool observation that confirms it should move it to VERIFIED and date it.
- A `core-syntax.md` audit is required whenever this file changes, per `.ai/core.md`.

```yaml
last_reviewed: 2026-11-01
```

---

## 8. Retired record ids

These ids are no longer records. Each one's facts that are still true were carried into the record named, so code, documents and log entries that cite a retired id should be read as citing that record. The summary is what the retired record said, kept so an old citation can be understood; it is not current fact. The full text is in git history.

| Retired id | Now in | What it said |
|---|---|---|
| GPS-001 | GPS-005 | The receiver was an Ultimate GPS breakout (PA1616D / MTK3333) on Serial1, module TX to D2 and module RX to D3, streaming NMEA 0183 at 9600 8N1 with both $GP and $GN talkers. Retired 2026-10-30 when the PA1010D replaced it; the NMEA parsing, talker handling and receive-only behaviour carried across unchanged. |
