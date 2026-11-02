## 1 COMMIT Unreleased 2026-10-27T08:28:41-07:00

#### Coming From:

Unreleased 9b756bb

#### Purpose:

Initialize this project's core memory by creating the first active-log entry and building a reference database for the external facts this firmware depends on, following the format the Tang family uses.

#### Outcome:

`.ai/core-reference.md` was written from empty into a 68-record database covering fourteen topics -- the board and its harness map, the ESP32-S3's memory and radio band, the FFat and NVS layout, the UDP gateway contract, the vendored microReticulum, microLXMF and MsgPack trees, the RYLR998 AT surface, the LoRa framing the Tang must implement, the GPS and PPS inputs, the three I2C devices, the host tooling, the PlatformIO environment, and provenance -- with a topic catalog, an active-routing table, a fast lookup index, a record template, a maintenance boundary and an empty retired-id section. Every record was derived from sources read in this session: the README's hardware-verified notes from 2026-10-26, the driver sources and headers under `nano-node/`, `VENDORED.md`, the three vendored `library.json` manifests, the microReticulum `Type.h` MTU and microLXMF `Type.h` link-packet limits, `LXMRouter.h` and `LXMRouter.cpp`, and RTClib's `DS3231_ADDRESS`. Statuses were assigned against evidence rather than convenience: 62 records are VERIFIED because a primary source plus a hardware or tool confirmation exists, six are SOURCED because the citation reaches the README's datasheet or manual notes without an independent measurement, namely SOC-001, RADIO-007, RADIO-008, FRM-005, FRM-007 and GPS-004, and no record is INFERRED. The receive path is the one functional gap recorded rather than smoothed over: RADIO-009 states that transmit is proven on hardware while reception has never been exercised, so a later cycle inherits the uncertainty instead of a false confidence. As the only build evidence available this cycle, all four host units were run on 2026-10-27 and passed (`RESULT: PASS`, exit 0), covering GPS parsing, magnetometer conversion, SHT45 CRC and conversion, and LoRa framing including the three-frame 500-byte case and the reassembly timeout; the firmware build and deployment were not exercised. `.ai/core.md` was not modified, as directed. The required core-syntax audit was performed against the rule in `core-syntax.md`: `core.md` and `core-syntax.md` were re-read, the complete `.ai/` diff was inspected and touches only `core-log.md` and `core-reference.md`, `core.md` was confirmed unchanged, this entry was validated for numbering, the six-section order, prose-only sections, allowed Status values and the 100-entry limit, and the reference database was machine-checked for schema, contiguous record ids, resolvable cross-references and parseable YAML blocks. The audit also records a standing discrepancy it did not act on: `core.md`'s Build Environment still carries FPGA GW5AST-138, SystemVerilog and BL616/Bouffalo SDK blocks and a workflow step that deploys to the Tang, none of which describe this repository, and by instruction that file waits for the user's own correction.

#### Next Steps:

Two engineering items are open and one of them should be the next cycle: the LoRa receive path has never been exercised, because it needs a second RYLR998 on the Tang side, so until then no claim of end-to-end radio delivery can be made and the reassembler rests on host tests alone; and the LXMF router is constructed with `/ffat/lxmf` as its storage path while the library never writes through MessageStore, so the directory's existence and whether it should hold anything are unresolved. Before either is reported as working, the firmware should be built and deployed once and the result recorded, since this cycle exercised only the host tests.

#### Files Modified:

None.

#### Status:

- Build: N/A
- Deployment: N/A
- User Test: N/A

---

## 2 COMMIT Unreleased 2026-10-27T11:55:08-07:00

#### Coming From:

Unreleased 90db3ea

#### Purpose:

Prove the node's LoRa receive path, which RADIO-009 recorded as never exercised, using a Raspberry Pi Pico 2 and a second RYLR998 as the far end.

#### Outcome:

`tools/pico_lora_sender.py` was written as the far end: it configures the module to the node's settings, reads every value back, then sends one single-frame message and one full-MTU message split into three frames exactly as `lora_frame.h` specifies. The Pico had to be reflashed to get there. It arrived running the old JTAG probe firmware, and its flashed MicroPython carried a `main.py` from an earlier project that crashed reading an absent I2C RTC and left the board rebooting, so it was erased with an `no_flash` nuke built from the pico-sdk and MicroPython 1.29.0 written over it; the probe firmware and the previous project's files are gone, which the user approved beforehand. With both boards connected the sender's output confirmed the module reports `+VER=RYLR998_REYAX_V1.2.4`, address 2, network 6, `+PARAMETER=7,7,1,12`, 14 dBm and 915 MHz, and that every `AT+SEND` answered `+OK`. Three runs followed: 4 frames delivered as 3 frames and 2 packets, 9 delivered as 9 and 3, and 4 delivered as 4 and 2, so 16 of 17 frames and 7 packets, with `bad lines 0, overlong 0, early 0, errors 0` throughout and the module reporting `rssi -16 dBm, snr 12` to `13`. The single loss did not announce itself: the 500-byte message arrived as a 321-byte packet and was counted as delivered, because the frame header carries no length and no checksum, which is now FRM-008 and a constraint on the protocol the Tang will speak. The `-16 dBm` reading means the modules were centimetres apart and probably overdriving each other, which is the first hypothesis for the loss rather than a conclusion. No node firmware was changed, and the only addition to the tree is the tool. RADIO-009 was revised from unexercised to measured. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and no settled entry was rewritten, checked that the reference's six YAML blocks parse with the index and records naming the same 69 ids in the same order, and validated this entry as number 2 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified.

#### Next Steps:

Stage 1 of the LoRa plan is next and is the long pole: restoring a UART on the Tang, since `pmod_uart.sv` and PMOD personality 7 were deleted in `800d11b` and a rebuilt desktop core needs a seed sweep and a board test. The framing's missing length and checksum (FRM-008) must be answered in the Tang protocol rather than in the frame, and the intermittent frame loss should be retested with the two modules further apart or at lower power before it is treated as a radio limit. The Pico now runs bare MicroPython with an empty filesystem, and the JTAG probe firmware in `~/Desktop/JTAGprobe_pico2.uf2` would have to be reflashed if that probe is wanted back.

#### Files Modified:

- tools/pico_lora_sender.py

#### Status:

- Build: N/A
- Deployment: N/A
- User Test: N/A

---

## 3 COMMIT Unreleased 2026-10-27T16:18:05-07:00

#### Coming From:

Unreleased 231e449

#### Purpose:

Give the Tang a way to ask this node questions over the LoRa link without becoming a Reticulum peer itself.

#### Outcome:

An application protocol was defined above the existing framing and implemented at both ends. `nano-node/src/tang_proto.{h,cpp}` carries one message: version, type, sequence, reserved, big-endian body length, body and a CRC-16/CCITT-FALSE, with a full body making exactly one 500-byte MTU; that length and CRC are the answer to FRM-008, whose silent truncation the codec now refuses, and `nano-node/host-tests/test_tang_proto.cpp` asserts 27 cases including a short buffer, one byte short, a lying length, a flipped bit in the body and in the header, an unknown version with a valid CRC, and the standard CCITT-FALSE check value. `tang_link.cpp` decodes a request and answers it from `tanglink::node_answer()` in `main.cpp`, which returns the node's own reply strings -- so an existing command needs nothing new here -- plus a compact status, because the LXMF status is written for a 63-byte budget and reports everything at once. `lora.cpp` gained a destination address per queued packet, a `send_to_tang()` that addresses replies to 2, and a sender-address branch that hands frames from address 2 to the protocol instead of to Reticulum; everything else is passed up unchanged, so the LoRa interface stays registered and the node remains a full Reticulum node. On the hardware the Tang's `node status` returned the node's full text with `paths 35` while its Reticulum stack was live, and status, env, gps, ping, time, help and an unknown word all answered with valid CRCs and matching sequences; the Tang retries a request ten times with the same sequence, and six of six commands answered without a timeout. FRM-009 and FRM-010 record the protocol and the demultiplexing. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and no settled entry was rewritten, checked that the reference's six YAML blocks parse with the index and records naming the same 71 ids in the same order, and validated this entry as number 3 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified.

#### Next Steps:

The `messages`, `body` and `send` request types are defined and answered with "not implemented yet", so the next cycle is to serve them: a list from `/ffat/messages.log` since a sequence point, one message in full by index, and an LXMF send through the node's identity, each bounded by the one-MTU message size. The Tang has no screen for any of this yet -- `node` is a console command -- so a Messages app follows. Two smaller items stand: the Tang's core is loaded by hand after every boot because the boot script still names the old core, and the receive path has no back-pressure beyond the 512-byte FIFO.

#### Files Modified:

- nano-node/host-tests/run.sh
- nano-node/host-tests/test_tang_proto.cpp
- nano-node/include/tang_link.h
- nano-node/src/lora.cpp
- nano-node/src/main.cpp
- nano-node/src/tang_link.cpp
- nano-node/src/tang_proto.cpp
- nano-node/src/tang_proto.h

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: N/A

---

## 4 COMMIT Unreleased 2026-10-29T18:21:11-07:00

#### Coming From:

Unreleased c48b3a4

#### Purpose:

Give the Arduino four relay outputs on D9..D12 and a `relay` command that sets them from a four-digit state and answers only when the hardware took it.

#### Outcome:

`nano-node/include/relay.h` and `nano-node/src/relay.cpp` add four relay outputs on D9, D10, D11 and D12, which carry no other role on this board because the GPS holds D2/D3, PPS D4, LoRa D5/D6 and I2C A4/A5, and D11/D12 are the SPI pair that nothing here opens. `relay::set(mask)` drives the four pads, waits 100 us and reads each one back, answering false when any pad did not reach the level it was driven to; `relay::off()` opens all four; `relay::pin_number(bit)` names the Arduino pin behind a bit; and `begin()` leaves the four pins as INPUT rather than picking a level, because an output has to be driven HIGH or LOW the instant it is configured and one of those two closes every relay on a module of the wrong polarity. `nano-node/src/main.cpp` gained `cmd_relay()`, reached from both `tanglink::node_answer()` and the LXMF `handle_command()`, so `relay 0000` works over the Tang's LoRa link and over LXMF with no other change; a bare `relay` answers the current state, a well-formed set answers `relay states updated` only after the read-back passes, `relay states not updated` when a pad refuses, and anything that is not four digits answers the usage line and drives nothing. Two serial console keys were added for bench work: `R` steps each channel for 500 ms in turn and leaves them open, `Z` opens all four. The polarity and the digit order were both decided on the bench rather than assumed. The first build followed the common pulled-up module, and the user heard every relay close on `relay 0000`, so `ACTIVE_LOW` was set false; the user then asked for the digits to follow the cut USB-C cable's connector rather than the header, so the pin array was reversed to {D12, D11, D10, D9} and `pin_number()` was added to keep the console's labels truthful, which leaves digit 1 = D12, digit 2 = D11, digit 3 = D10 and digit 4 = D9. On the hardware, `R` stepped all four channels and the user heard one relay per step, which is what proved the pins reach the module; over LXMF `relay 1010` answered `relay states updated`, a following `relay` answered `relay 1010`, `relay 12` answered the usage line unchanged, and `relay 0000` left all four open, with the user reporting that the relays all work. The firmware built at RAM 28.4 percent and flash 66.1 percent (2,078,897 bytes) and was uploaded over /dev/ttyACM0, after which the node answered `t` with its DS3231 time; BRD-005 records the pins, the cable-order digits and the measured polarity. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and no settled entry was rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 72 ids in the same order, and validated this entry as number 4 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified.

#### Next Steps:

The relay state is not reported in `status`, so a caller that wants to know it must ask `relay` on its own; adding the four digits to the status text is the obvious next step if the Tang ever shows the relay state on screen. The Tang already reaches the command through `TANG_REQ_COMMAND`, so `node relay 1010` works from that console with no Tang-side change, and a desktop control is the work that follows on that side. The `messages`, `body` and `send` request types remain answered with "not implemented yet", as entry 3 left them, and the node's serial console still has no key that reports GPS counters.

#### Files Modified:

- README.md
- nano-node/include/relay.h
- nano-node/src/relay.cpp
- nano-node/src/main.cpp

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: PASS

---

## 5 COMMIT Unreleased 2026-10-29T19:45:53-07:00

#### Coming From:

Unreleased a6b1859

#### Purpose:

Report the four relay states in `status`, on both of the surfaces a caller can reach it from.

#### Outcome:

`status_text()` gained a `relay:` line after `lora:`, and the compact status that `tanglink::node_answer()` returns to the Tang gained `relay <digits>` as a sixth line, which costs eleven bytes of the 492 the message can carry. `relay_states()` was moved above `status_text()` so both callers can use it, and its comment was corrected from the stale "D9 first" to the cable order the digits actually follow. README's `status` row now lists relay states and its relay paragraph says `status` reports them in both replies. The firmware built at RAM 28.4 percent and flash 66.1 percent (2,079,145 bytes) and was flashed over /dev/ttyACM0; over LXMF `status` then answered with `relay:   0000`, `relay 0001` answered `relay states updated`, and the following `status` answered `relay:   0001`, so the line follows the mask rather than a constant. A false alarm followed and is worth recording: the user reported that `relay 1010` closed every relay, asked for the firmware to be reverted, and it was — the uncommitted change was saved to a diff, the tree was restored to a6b1859, rebuilt and flashed. One datum taken before that revert is what made the diagnosis quick: the node's own record read `relay 1001`, so the last set it had accepted was neither 1111 nor 1010, and the command had not been applied at all. With the reverted firmware running, `relay 0000`, `relay 1000` and `relay 1010` each behaved, and the user found the fault to be a cable in the relay loom rather than the firmware. The status change was then reapplied from that diff, rebuilt to the same size and flashed again; the board came up with a fresh uptime, `relay:   0000`, a 3D fix on nine satellites and the LoRa interface up, and the user reports the relays working. The compact status line is the one part not exercised, because it needs the Tang in two-wire with a LoRa core loaded. BRD-005 and its index line now record that `status` reports the digits. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and no settled entry was rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 72 ids in the same order, and validated this entry as number 5 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified.

#### Next Steps:

The `relay <digits>` line in the compact status is unproven: it needs the Tang in two-wire mode with a LoRa-capable core loaded, after which `node status` shows the whole reply on the console. The `messages`, `body` and `send` request types are still answered with "not implemented yet", and the serial console still has no key that reports the GPS counters, which are the next two items on this side.

#### Files Modified:

- README.md
- nano-node/src/main.cpp

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: PASS

---

## 6 COMMIT Unreleased 2026-10-29T19:54:02-07:00

#### Coming From:

Unreleased 1f17aa6

#### Purpose:

Close entry 5's one open item by proving the `relay <digits>` line in the compact status actually crosses the LoRa link.

#### Outcome:

With the Tang in two-wire mode and the ABI 1.6 desktop core loaded with `layout = lora`, `lora ver` answered `+VER=RYLR998_REYAX_V1.2.4` and the Tang's `node status` returned the compact reply in full: `TinyNode 2026-10-30 02:52:41Z`, `up 0h5m`, `wifi 10.10.0.35 -48 dBm`, `heap 157 KB`, `paths 58` and `relay 1111` as the sixth line, read from a node whose four channels were closed at the time. The request took three attempts -- two logged `node: no reply, retrying` at 2/10 and 3/10 before the reply arrived -- which is the lossiness PROT-011 and the Tang's retry policy exist for, and no hang occurred, so the command returned to the shell prompt. Nothing was built, flashed or changed in this repository: the firmware answering was entry 5's, 2,079,145 bytes, already deployed. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff and confirmed it touches only this log, that `.ai/core.md` was not changed and no settled entry was rewritten, and validated this entry as number 6 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and `None.` in Files Modified.

#### Next Steps:

Nothing of entry 5's is outstanding. The node side's remaining items are the `messages`, `body` and `send` request types, still answered with "not implemented yet", and a serial console key that reports the GPS counters; on the Tang's side the `node` command's unbounded stale-byte drain and its unbounded link lock are the two defects that made an earlier run of this same command wedge the console, and they belong to that project's next cycle.

#### Files Modified:

None.

#### Status:

- Build: N/A
- Deployment: N/A
- User Test: N/A

---

## 7 COMMIT Unreleased 2026-10-29T21:43:12-07:00

#### Coming From:

Unreleased 2ff9b07

#### Purpose:

Carry the node's sensors in the compact status the Tang receives, so one request brings back the GPS, position, magnetometer, heading, environment, PPS and radio state as well as the housekeeping.

#### Outcome:

`node_answer("status")` now builds its reply with a small `append_line()` helper: the five housekeeping lines are formatted first, then `gps`, `pos`, `mag`, `env`, `pps`, the LoRa short status and `relay` are added one at a time, and any line that would not fit inside `TANG_PROTO_MAX_BODY` is dropped whole rather than truncating the reply -- the message is 492 bytes and the finished block measures about 290, so everything fits with room to spare. The order matches the full LXMF status, and an empty label is passed for `lora::short_status()` because that string already begins with "lora up"; the first build would have printed `lora lora up ...` and was corrected before deployment. The comment above the function was rewritten to say what the budget now is, since the old text still described a 63-byte LXMF single-packet limit that only the full reply ever had. The firmware built at RAM 28.4 percent and flash 66.1 percent (2,080,193 bytes) and was flashed to /dev/ttyACM0; over LXMF the full status answered with all thirteen lines, and the Tang's own `node status`, on its third attempt, returned the twelve-line compact block -- `TinyNode`, `up`, `wifi`, `heap`, `paths`, `gps 3D,7sat,hdop1.1,age1.0s`, `pos 33.293873,-111.880902,357m`, `mag X-16.8,Y-6.9,Z-36.8 uT,hdg299.0`, `env 24.8C,42.2%`, `pps last 999.999ms avg 999.999ms (16) n=156 out=26`, `lora up tx5 rx3 rssi-29 snr12` and `relay 1111`. The Tang side needed no change, because the new TinyNode window's list already shows whatever lines arrive, and the user reports the block appearing in it after Update. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff and confirmed it touches only this log, that `.ai/core.md` was not changed and no settled entry was rewritten, and validated this entry as number 7 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified.

#### Next Steps:

The compact block is close to its whole-message budget, so anything added from here has to displace something or the reply starts dropping lines: a storage line and a hop count are the two candidates a caller might still want, and either would need deciding what goes. The `messages`, `body` and `send` types remain unanswered, and the serial console still has no key that reports the GPS counters.

#### Files Modified:

- nano-node/src/main.cpp

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: PASS

---

## 8 COMMIT Unreleased 2026-10-30T08:30:00-07:00

#### Coming From:

Unreleased 55e11dd

#### Purpose:

Give a long-running agent one stable LXMF address and a mailbox on disk to answer from, after a fault in which the node's replies vanished with no error at either end.

#### Outcome:

`tools/agent_endpoint.py` was written as a single Python script that holds an LXMF address for an agent and never interprets what it carries: it loads the identity from `~/.tinynode-agent/identity` and adopted the key from the scratch endpoint, so the address stayed `83e0d9fa0f01d0a0f495dcd6600600c4` and the conversation the operator already had open in Sideband continued instead of a new peer appearing; it takes an `flock` so a second copy cannot run; it loads `~/.reticulum` so its RPC authenticates against the Sideband instance; it announces at start and every fifteen minutes; it appends every message to `~/.tinynode-agent/inbox.jsonl` and sends anything dropped into `outbox/`, moving each file to `sent/` only once the send succeeds. Only the operator's address and the node's are acted on, a message from anybody else is recorded with `kind: "other"`, and an operator message beginning with `node ` is forwarded to the board with the answer returned prefixed `node: `. The lock exists because this cycle hit the fault it prevents: two endpoint processes held the same identity at once, each announced its own ratchet for the same destination, and every reply disappeared while the node's console reported `Sending LXMF message via link` and `Message sent successfully via link` for each one; the LoRa counters held at `tx 11 rx 0, frames tx 21` across a ping, which ruled the radio out, and Sideband as the instance owner received the same reply the client could not, which confined the fault to the shared-instance hop. Killing the duplicate and leaving one process made `pong` arrive within seconds and `status` arrive as a 614-byte resource with the node logging `DIRECT delivery confirmed`, and a second copy now exits 1 naming the pid holding the lock. The relay was then exercised from the operator's own Sideband, where `node status` returned the full status text ending `relay: 1111` and an ordinary message was recorded with `kind: "operator"` and answered, and the operator reported the conversation working. `lxmf_send.py`'s hardcoded destination was stale, `9ad89c31...` belonging to a retired identity, and now carries `854ff4d2...`, the address the node confirmed from a path request for its own `4023cedd...` nano.esp32 destination. HOST-007 records the shared-instance contract, and README gained the endpoint's contract, its two-file interface, the three invariants and the identity backup warning. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and no settled entry was rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 73 ids in the same order, and validated this entry as number 8 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified. No firmware file changed, so nothing was built or flashed.

#### Next Steps:

The endpoint runs from the repository under `setsid` and does not survive a reboot, so a systemd user unit is the next step, followed by a `--follow` mode that streams new inbox lines so a model can block on the file instead of polling it. The node currently reports `relay: 1111`, taken from the operator's earlier relay test, and whoever uses it next should know the four contactors are closed at that mask until someone sends `node relay 0000`. The scratch bridge under `/tmp/tnbridge` is superseded and should be deleted rather than reused, its identity having been copied into `~/.tinynode-agent` already. The `messages`, `body` and `send` request types on the Tang side remain unanswered, as entry 7 left them, and the node still has no serial console key that reports the GPS counters.

#### Files Modified:

- README.md
- lxmf_send.py
- tools/agent_endpoint.py

#### Status:

- Build: N/A
- Deployment: N/A
- User Test: PASS

---

## 9 COMMIT Unreleased 2026-10-30T11:42:00-07:00

#### Coming From:

Unreleased 1361f83

#### Purpose:

Let the local model answer Sideband messages on its own, so the operator's chat window is the only interface needed.

#### Outcome:

`tools/agent_daemon.py` closes the loop the endpoint opened: it reads new records from `~/.tinynode-agent/inbox.jsonl`, answers only those from the operator, sends `thinking...` before it starts because Sideband shows no typing indicator and a cold turn is slow, runs one `opencode run --format json -m ollama/qwen3.6:35b-a3b-coding --auto --dir ~/.tinynode-agent` per message with the session id kept in `~/.tinynode-agent/opencode-session`, parses the `text` events for the answer, and queues the reply into the endpoint's outbox; an answer over 1800 characters is sent in numbered parts with the whole text written under `answers/`, the word `new` clears the session, and `--once` and `--new` cover the one-shot cases. Two `opencode run` behaviours were found by testing rather than by reading and both are worked around: it blocks forever on a non-tty stdin, stopping after `init` and never reaching the model, so the daemon passes `stdin=DEVNULL`; and `--attach` to a running server returns after the first event and drops the answer, leaving it in the session database with nothing on stdout, so the daemon drives a fresh run per message and lets it finish, which measured 46.3 s cold and 2.0 s warm with the follow-up correctly recalling the previous question. Both processes are now user units, `tinynode-endpoint.service` and `tinynode-agent.service` with `Restart=always`, enabled and started with `loginctl enable-linger` so they come up at boot without a login; copies are kept in `tools/systemd/` and the hand-started endpoint was stopped. `--auto` is a deliberate trade recorded in the README: an unattended agent that waits for an approval nobody can see is a hung agent, so the working directory is held to `~/.tinynode-agent` and the endpoint's allowlist remains the only peer that can reach it, with a dedicated opencode agent carrying explicit denials left as the next security step. The operator tested the loop from Sideband and reported it working. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and no settled entry was rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 73 ids in the same order, and validated this entry as number 9 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified. No firmware file changed, so nothing was built or flashed.

#### Next Steps:

The replacement GPS module, an Adafruit Mini GPS PA1010D on the I2C bus, is the work that follows this entry: it retires the PA1616D on Serial1 and needs a new read path, new reference records, a wiring table and a host test before it can be trusted as the board's clock source. On the agent side, a dedicated opencode agent with explicit permission denials should replace the global `--auto` before this runs unattended for long periods, and a `--follow` mode in the endpoint would remove the one-second polling latency. The node's relays currently read `0000`.

#### Files Modified:

- README.md
- tools/agent_daemon.py
- tools/systemd/tinynode-agent.service
- tools/systemd/tinynode-endpoint.service

#### Status:

- Build: N/A
- Deployment: N/A
- User Test: PASS

---

## 10 COMMIT Unreleased 2026-10-30T13:36:19-07:00

#### Coming From:

Unreleased 2878c8d

#### Purpose:

Replace the retired Ultimate GPS on Serial1 with an Adafruit Mini GPS PA1010D polled over I2C, so the board keeps a hardware clock source on a receiver that is still available.

#### Outcome:

`nano-node/src/gps.cpp` now polls the PA1010D at I2C address `0x10` on A4/A5 instead of reading Serial1: `begin()` probes the address and logs `GPS: PA1010D at 0x10 present`, and `poll()` reads a 32-byte chunk at most every 5 ms and feeds the bytes through the same sentence assembler, `Fix` snapshot and `utc_epoch()` the UART receiver used, so the parsing, the talker handling and the time hierarchy did not change. The one new rule is load-bearing and comes from Adafruit's own driver: the module answers every read with a full chunk padded with `0x0A` when it has nothing to send, so a line feed that does not follow a carriage return is filler and is never treated as the end of a sentence. `nano-node/host-tests/test_gps.cpp` was rewritten around a streaming I2C mock that pads the same way; it now asserts 110 checks with no failures, including a sentence split across two reads with filler handed out between the halves, an absent module reporting `absent (no ACK at 0x10)` rather than looking stale, and the original fix, checksum, staleness and UTC cases. The firmware built at RAM 28.4 percent and flash 66.1 percent (2,080,597 bytes) and was flashed over /dev/ttyACM0; on hardware the boot log reads `GPS: PA1010D at 0x10 present`, the I2C scan lists `0x10 0x1C 0x44 0x47 0x68`, and `status` reports `gps: 3D fix, 12 sats, 33.293893N -111.880882W, alt 377m, hdop 0.8`, which the operator confirmed as passing. The reference was brought in line: GPS-001 (the PA1616D on Serial1) is retired into the new GPS-005, and GPS-003 and GPS-004 were revised to record that the PA1010D's PPS pad is unwired and that the NMEA delay is now bounded by the poll interval rather than by a UART line rate. During the hardware check the node cleared its own settings and rebooted into the setup wizard: something delivered an `F` to the serial console, and `main.cpp`'s `F` handler clears NVS and restarts with no confirmation of any kind, two seconds after the node had processed an LXMF message, a path that cannot reach that handler. The node was offline until the wizard was refilled from `nano-node/include/wifi_config.h` (name, SSID, PSK) and saved, after which it rejoined `10.10.0.35` with its identity and learned paths intact. The same check found the LIS3MDL back at `0x1C` after the operator reconnected it, reading `X +35.3 Y +4.9 Z -44.2 uT, |B| 56.8 uT, hdg 31.6`; the SHT45 reading `25.4 C 43.7 %RH`; the DS3231 holding time across the reset; an undriven device answering at `0x47`; and D4 reporting about 30 Hz of irregular intervals, which is a floating PPS input rather than the module's one pulse per second. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and no settled entry was rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 73 ids in the same order, and validated this entry as number 10 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified.

#### Next Steps:

The device answering at `0x47` has no driver and has not been identified; its chip-ID register should be read before anything is written against it, and the operator knows which pressure sensor was fitted. The serial console's `F` key clears the stored WiFi settings with no confirmation at all, which is how this cycle lost the node's provisioning, so it should require a typed confirmation the way the setup wizard's identity reset already does. D4 carries a floating PPS input: either wire the PA1010D's PPS pad to it or stop reporting the line, because roughly 30 Hz of noise is worse than no figure. On the Tang side, the `messages`, `body` and `send` request types remain unanswered, and the agent daemon still runs opencode with a global `--auto`.

#### Files Modified:

- README.md
- nano-node/host-tests/test_gps.cpp
- nano-node/include/gps.h
- nano-node/src/gps.cpp
- nano-node/src/main.cpp

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: PASS

---

## 11 COMMIT Unreleased 2026-11-01T07:56:21-07:00

#### Coming From:

Unreleased 0c59ffd

#### Purpose:

Repair this project's control memory after two reference-only commits landed without log entries and recorded facts confirmed on the TinyDesk bench, a different node, as though this project's hardware had verified them.

#### Outcome:

The recovery policy was followed from `.ai/core.md` through `.ai/core-log.md` to `.ai/core-reference.md`, which showed a clean tree on main level with origin, ten log entries, 82 live records and a host test gate that still passes, but also two commits, `3e84097` and `0c59ffd`, that added seven records with no log entry and no core-syntax audit: RNS-007, RNS-008, LXMF-007 through LXMF-011, SEN-006 and SEN-007. Their verifications named the TinyDesk bench and a `components/rns/CMakeLists.txt` that does not exist in this repository, and GPS-005 carried a citation to "core-log entry 11", which is that other project's log and not this one, so this project's memory had come to hold another bench's hardware facts as VERIFIED. All seven records were kept in place because each describes a stack this project actually uses, and their provenance was corrected rather than deleted: where a primary source is held here the claim was re-checked against it and kept as VERIFIED, and where the only confirmation was that other bench's hardware the record was demoted to SOURCED with the gap named. LXMF-007 was re-confirmed field by field against LXMF 1.2.1's `LXMF.py` in `~/.venvs/rns`; LXMF-008 and LXMF-009 against Sideband 2.2.0's `sense.py` and `core.py`, extracted from the AppImage on this bench, which supplied the SID list, the integer SID_TIME rule and the two-element TELEMETRY_REQUEST list; LXMF-010 against the four `packRawBytes()` call sites in this repository's vendored `LXMessage.cpp` together with the client's `umsgpack.unpackb`; the 159-byte gate in LXMF-011 against `LXMRouter.cpp` and `Type.h`; and RNS-007's store selection against `IdentityEntry.h` and the `RNS_PERSIST_KNOWN_DESTINATIONS` default of 1 in `Type.h`, which this project leaves at that default. RNS-007, LXMF-011, SEN-006 and SEN-007 are now SOURCED because the recall failure, the link-delivery failure, the 495 and 33 byte measurements and both sensor identifications were seen only on that other bench. RNS-008 lost a clause that did not survive inspection: `FileStore::size()` returns `_index.size()` and is therefore a record count, so the assertion that it reports bytes of a path string was withdrawn and the 1634742062 reading was left as an unexplained lead, while the record kept the container split that `Transport.cpp` confirms by consulting `_new_path_table` in `has_path()` and leaving the old `_path_table.find()` commented out. BRD-001 was rewritten because it claimed `/dev/ttyACM0` was the TinyTang and the node was on `/dev/ttyACM1`, which core-log entries 4, 5 and 10 contradict, and the record now states that the port name is unstable bench state: `/dev/ttyACM0` existed at 07:43 on 2026-11-01 and no ttyACM device remained by 07:56, so the node is not attached. GPS-005's dangling citation was replaced with plain attribution to the other bench. Section 5's records were also realigned into topic order, because the seven imported records had been appended after PROV-003 rather than filed under LXMF and SEN as section 5 requires, which left section 4's index and section 5 in disagreement; the reorder preserved every record's text verbatim and the index and records now name the same 82 ids in the same order. No firmware was built or flashed and no source file changed, which is why all three Status values are N/A; the node was also not attached at the time of this cycle. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and that no settled entry was rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 82 ids in the same order under the allowed Status values and topic ids, and validated this entry as number 11 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and `None.` in Files Modified.

#### Next Steps:

Four one-step tests would clear the records this cycle demoted: read register 0x01 at 0x47 on this board to settle whether its unidentified device is the BMP581, reproduce the `Identity::recall()` failure under this project's `-DRNS_USE_FS` build before the messages work depends on opportunistic replies, send a message larger than 159 packed bytes to a Python peer to see whether link delivery really fails here, and measure a telemetry map's packed size rather than inheriting another bench's 495 and 33 bytes. The engineering items the log already carried are unchanged: `messages`, `body` and `send` still answer "not implemented yet" in `tang_link.cpp`, and the console's `F` key still clears NVS and reboots with no confirmation, which is how entry 10 lost the node's provisioning. The node is not attached, so it must be reconnected and its port confirmed before any build or flash. `.ai/core.md`'s Build Environment still describes an FPGA, SystemVerilog and a BL616/Bouffalo toolchain that this repository does not contain, and by instruction that file waits for the user's own correction.

#### Files Modified:

None.

#### Status:

- Build: N/A
- Deployment: N/A
- User Test: N/A

---

## 12 COMMIT Unreleased 2026-11-01T08:19:12-07:00

#### Coming From:

Unreleased e66aa4e

#### Purpose:

Settle how a telemetry reply is carried and what each sensor costs, after the operator questioned reading the 159-byte packet threshold as a limit on telemetry size.

#### Outcome:

The operator was right and this project's reference was wrong. LXMF has no maximum message size: a message is either sent as one OPPORTUNISTIC packet with no link, or by DIRECT delivery over a link, where a payload larger than one link packet becomes a RESOURCE transfer, as `LXMF 1.2.1 LXMessage.py` shows by selecting PACKET or RESOURCE from `LINK_PACKET_MAX_CONTENT` with no upper bound. `LORA_ENCRYPTED_PACKET_MDU` of 159 is only the threshold for the first case, and both implementations default to DIRECT, so an oversized telemetry map is not refused: the Python reference logs `Falling back to link-based delivery` and microLXMF logs `Too large for OPPORTUNISTIC, using DIRECT resource`. `tools/telemetry_budget.py` was written to measure the budget rather than estimate it and reports the single-packet arithmetic: 114 bytes are committed before any sensor, being 16 destination hash, 16 source hash, 64 signature, a 14-byte msgpack envelope, the fields-map header, the `FIELD_TELEMETRY` key and a 2-byte bin header, which leaves a 45-byte packed sensor map in which TIME costs 6 bytes, LOCATION 41, MAGNETIC_FIELD 29 and PRESSURE, TEMPERATURE and HUMIDITY 10 each, so the environment triad plus TIME is 151 packed bytes and TIME plus LOCATION is 162. Every one of those figures was cross-checked the same session against the vendored `MsgPack::Packer` compiled on the host, which reproduced the same per-sensor marginals and the same packed sizes for all seven candidate replies. The first version of the tool was wrong in a way worth recording: it modelled the fields map with the sensor ids as LXMF field keys, when a telemetry reply is one `FIELD_TELEMETRY` entry whose value is a bin holding the sensor map, which undercounted every row and reported LOCATION as fitting at 158 bytes; compiling the real packer is what caught it. LXMF-011 was rewritten to describe a delivery-method threshold instead of a size limit and LXMF-012 was added for the separate and still unsettled question of whether link delivery assembles on this stack, since this node recorded a 614-byte status reply arriving as a resource in entry 8 while another bench reported a 607-byte link message failing. The link-delivery test itself could not run. `tools/link_delivery_test.py` found no path to the node after three attempts over 90 seconds, and identity recall succeeded, so the failure is path discovery rather than a missing key. Investigating that produced the cycle's most consequential finding: the board on `/dev/ttyACM0`, USB serial `206EF131675C`, is not running this repository's firmware. Its console emits a 60-second heartbeat reading `clock: system=1793546285 gps=1793546285`, `sensors: env=1 26.4C/43.6% press=1 968.1hPa mag=1 gps=1 fix=1 unix=1793546285` and `alive t=960s free=8240728 paths=2 acc=2 ann=2 rx=0 pkts=6/7`, and none of those strings exist anywhere in `nano-node/`, which has no pressure driver at all; the 8.2 MB free figure is PSRAM rather than the 320 KB DRAM this firmware reports. The last message the endpoint recorded from the board was on 2026-10-30 and said `TinyNode status`, so the board was reflashed to other firmware after that. This is almost certainly the TinyDesk node firmware, and it explains the origin of the seven records that arrived in this project's reference on 2026-11-01 claiming BMP581 and telemetry verification: that work was real, but it was done on a different firmware in a different repository, and the board it ran on is the board now attached here. The bridge under `microLXMF/conformance-bridge` was tried as a way to test link delivery without the board, but its `CMakeLists.txt` pins microReticulum at `3ee2bd83`, which fails to compile against `RNS::Utilities::Memory::allocator_info` having no `free_count` member, and the `lxmf-conformance` harness that drives it is not present on this machine, so that route is closed for now. No firmware was built or flashed, so all three Status values are N/A. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and that entries 1 to 11 were not rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 83 ids in the same order under the allowed Status values, and validated this entry as number 12 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified.

#### Next Steps:

The board's firmware has to be settled before anything else on the node: either this repository's TinyNode firmware is built and flashed over the image now running, which discards that image, or the work continues on the firmware that is actually there, and that decision is the operator's. Until then no measurement or reply from this board describes code in this repository. Once the right firmware is on the board and it announces, `tools/link_delivery_test.py` should answer LXMF-012 by sending a command whose reply exceeds 159 packed bytes and checking that the peer assembles it; the node's announce schedule is 90 to 300 minutes, so an `a` on the serial console is the fast way to make it discoverable. The sensor question itself is now a cost question rather than a fit question, since nothing has to be discarded: position and magnetic field cost a link plus a resource transfer, which over LoRa at about 400 B/s is seconds of airtime, so how often those readings are worth sending is the real design choice. If pressure is to be added to this repository's firmware it still has no driver here, and the board's own BMP581 at 0x47 reading 968 hPa while this tree has no code for it is the cleanest possible demonstration of that gap.

#### Files Modified:

- tools/link_delivery_test.py
- tools/telemetry_budget.py

#### Status:

- Build: N/A
- Deployment: N/A
- User Test: N/A

---

## 13 COMMIT Unreleased 2026-11-01T09:20:32-07:00

#### Coming From:

Unreleased 96b7768

#### Purpose:

Put LOCATION and MAGNETIC_FIELD into the node's telemetry reply, after the measurement in entry 12 showed they cost a link rather than a packet.

#### Outcome:

The active firmware is project 4, the tinydesk nanoesp32-node branch that runs beside the desktop, reached from `/run/media/vash/GIT/tinydesk-work/tinydesk` on `nanoesp32-node`; P1 in this repository is the standalone node the work is being migrated from, and the board on `/dev/ttyACM0` serial `206EF131675C` was confirmed running P4 by matching its heartbeat strings to `ports/nanoesp32/main/node.cpp`. P4 had never sent its full telemetry map: `lxmf_send_telemetry()` passed `compact` unconditionally and replied OPPORTUNISTIC with only TIME, pressure, temperature and humidity, while LOCATION and MAGNETIC_FIELD were gated behind the same flag. The reply now takes that flag from a new `s_telemetry_full`, defaulting to true, so a request answers with the full map over DIRECT, and `node telemetry full` and `node telemetry compact` switch it at runtime so the bench can compare the two without a reflash; a bare `node telemetry` still dry-runs and now reports which map it packed and how it would be carried. Exercising the full map for the first time exposed a real encoding bug that only the compact path had hidden: three msgpack map headers declared the wrong entry counts, `RNS_TRANSPORT` at 13 for 12 entries, `ifstats` at 6 for 5, and the interface map at 9 for 10, and since an under-declared map consumes its parent's items the whole 499-byte field overran and `umsgpack` threw `InsufficientDataException`. The first full request therefore arrived and could not be read at all; after correcting the three counts the map parses end to end. The firmware built at 2,624,160 bytes with 17 percent of the 3 MB app partition free and was flashed over /dev/ttyACM0 with every image hash verified, and the identity survived, `14f4ab99...` presenting as delivery address `89f609774be9dc1b2640bc5f23d81714`. On the hardware a telemetry request then returned a 614-byte reply by DIRECT method 2 that decoded to TIME, LOCATION at 33.293845N -111.881003W 371.8 m, PRESSURE 968.46 hPa, TEMPERATURE 27.1 C, HUMIDITY 40.8 percent, MAGNETIC_FIELD 12.4 -2.0 -26.9 uT, RAM, NVM and RNS_TRANSPORT, and the same payload fed to Sideband 2.2.0's own `Telemeter.from_packed()` parsed with a current timebase and all nine sensors. That settles LXMF-012 in the affirmative: link delivery works on this stack, and the failure recorded from another bench did not reproduce with Sideband as the receiver. The operator then reported that Sideband's Request Update button left the panel showing readings six hours old, and this was diagnosed from Sideband's own database rather than from the firmware: the reply had been stored, a row stamped 16:02:20Z holding a ten-entry map, with `last_request_success_timebase` set, but the panel was still painting rows from 02:49 and 02:50 while the conversation row's `last_rx` had not moved since the night before. No `lxm` row exists with the P4 address as a source, so the reply was never registered as a conversation message and the panel had nothing to repaint from; restarting Sideband showed the current reading, which the operator confirmed as a pass. Two of this cycle's own readings were wrong and are recorded as such: an earlier claim that link delivery did not reproduce was generalised from a client running LXMF 1.2.1 while Sideband bundles 1.2.0, and a claim that the raw field value contradicted LXMF-010 was a misreading of Python LXMF unwrapping the bin before it was inspected, so LXMF-010 stands and the firmware complies. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and that entries 1 to 12 were not rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 83 ids in the same order under the allowed Status values, and validated this entry as number 13 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and only eligible paths in Files Modified.

#### Next Steps:

The firmware change lives in P4's working tree on `nanoesp32-node` and is not committed there, so it should be committed in that repository before it is lost or before the branch is cleaned up into the real project. Sideband's telemetry panel needs a restart to repaint after a reply arrives, which is a client-side defect worth confirming against a second client and worth a note to the Sideband author rather than a workaround in firmware. LOCATION ships with speed, bearing and accuracy all written as zero because the PA1010D driver has no source for them, and a client may render that as a suspiciously precise fix; either report the field honestly or omit what cannot be measured. MAGNETIC_FIELD carries no heading because the LIS3MDL calibration that P1 has was not carried across. Telemetry replies now cost a link and a resource transfer, which over LoRa is seconds of airtime rather than a fraction of a second, so `node telemetry compact` is the setting to use if the radio link ever has to carry them. P1 remains to be reconciled once P4 is brought up fully, and `.ai/core.md` still describes an FPGA, SystemVerilog and a BL616 toolchain that neither project contains.

#### Files Modified:

- tools/link_delivery_test.py
- tools/telemetry_request_test.py

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: PASS

---

## 14 COMMIT Unreleased 2026-11-01T10:44:01-07:00

#### Coming From:

Unreleased 4d77478

#### Purpose:

Finish the LOCATION sensor, calibrate the magnetometer so its readings mean something, and give the desktop a second sink over USB, before deciding what the board's console should be.

#### Outcome:

Three pieces of work landed in P4, the tinydesk nanoesp32-node tree at `/run/media/vash/GIT/tinydesk-work/tinydesk`, and the third of them broke the node's network, so this entry records a checkpoint rather than a working state. First, the LOCATION sensor had been reporting speed, bearing and accuracy as literal zeros; RMC fields 7 and 8 and GGA field 8 were already arriving and being discarded, so the parser now keeps speed in knots, course in degrees and HDOP, and accuracy is HDOP times an assumed 5 m range error, labelled as a project choice because the receiver measures HDOP and not metres. A stationary receiver omits course rather than reporting zero and the positional seven-element array cannot express that, so absent still reads zero. Second, hard-iron calibration was ported from P1 and made a guided command: `magcal` prints the current offsets and reading, warns about steel nearby, then samples for about 36 seconds in three named phases (flat, on edge, upside down) with a live per-axis span meter, reports the fit or why it failed, and offers to save. Sampling runs in the node task rather than in the command, because that task owns the I2C bus and a command reading the sensor would race the background refresh. Offsets persist to `/fs/mag_cal` as three integers in text so a future struct change fails to parse rather than being misread. The fit gate was tightened to reject spans differing by more than 1.6x after a first attempt that spread 2x and passed, producing offsets biased by about 18 percent. Third, and this is the part that regressed, the port had been building `link_lcd.c`, which sends the desktop to the panel and takes input from USB, so typing appeared only on the LCD and the USB side received nothing but `node_log` lines. A new `link_nano.c` was written to pick a sink at runtime, USB when a host terminal is attached and the panel otherwise, with `TD_MAX_COLS`/`TD_MAX_ROWS` raised from 40x15 to 160x50 for the USB case. The desktop renders correctly over USB after this, but the board came up with WiFi dead. `10.10.0.35` produces no ARP reply at all and the node cannot be reached over Reticulum, while the desktop still draws, so the board is alive and the network stack is not. The prime suspect is the USB transmit ring buffer, which this change set from 1 KB to 64 KB: IDF allocates it with `xRingbufferCreate` in internal RAM, which is the same pool WiFi's roughly 50 KB stack needs, and the symptom is a network that never associates rather than a crash. That diagnosis is a hypothesis and not established. While diagnosing this the operator decided the desktop is not wanted on this board at all: the plan is a console only, with TinyDesk Shell serving the USB serial link when a host is attached and the 320x240 panel's VT100 emitter otherwise, which also removes the large screen buffers and should let WiFi recover. A correction also belongs here: the field strength this bench should read was quoted as "about 48 uT" for two cycles and was wrong. IGRF-14 evaluated at 33.2939N -111.8810E, 377 m for 2026 gives 46.6 uT total, 23.8 horizontal, 40.0 vertical, a dip of 59.3 degrees and a declination of 9.6 degrees east, and the model was checked against London, the equator and the south pole before being relied on; those figures now live in `magcal`'s guidance text and in the per-axis radius report that names which axis was under-rotated. The calibration achieved before the regression measured 39.6 uT, 85 percent of the true field, with a dip 1.5 degrees off, which is consistent with one axis under-covered rather than with soft iron. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and that entries 1 to 13 were not rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 83 ids in the same order under the allowed Status values, and validated this entry as number 14 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and `None.` in Files Modified.

#### Next Steps:

The console-only rewrite is the next cycle and it should also repair the network: replace the port's entry point so TinyDesk Shell's console loop runs over the USB serial link when attached and over the panel's VT100 emitter when not, drop tinydesk and with it the 125 KB of screen buffers, and size the USB rings small again so internal RAM is free for WiFi. Until that flash lands the board has no network, so telemetry, Sideband and `tools/telemetry_request_test.py` cannot be used to check anything. Two smaller items are open on the calibration: no axis radius has yet been seen reaching 46.6 uT, so a clean tumble is still owed, and whether the saved offsets survive a reboot has never been shown, since the boot-log line that would prove it was never captured. No heading is derived or reported, deliberately, because without an accelerometer there is no tilt compensation and a bearing would be a confident lie; P1 records the same limit. P4's working tree is committed as a checkpoint and pushed to the fork; it is known to be a state with the network down.

#### Files Modified:

None.

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: FAIL

---

## 15 COMMIT Unreleased 2026-11-01T17:55:23-07:00

#### Coming From:

Unreleased 79c81ec

#### Purpose:

Bring P4 up as a console-only node with working WiFi provisioning, and get the magnetometer actually calibrated, after the desktop build could not bring up its network and magnetic readings varied by a third with orientation.

#### Outcome:

This cycle produced a working board and cost far more than it should have; both are recorded here. The desktop is gone from P4: `app_main` runs TinyDesk Shell on its own, `link_nano.c` serves it over USB Serial/JTAG when a host terminal is attached and on the ILI9340 panel through `vt.c` when not, and input is USB only because the panel HAT has no touchscreen or keys. The binary went from 2.63 MB to 1.93 MB. The console needed three fixes before it was usable: the shell task stack was 12 KB where `TDSH_SHELL_TASK_STACK` is 24 KB, which overran inside the line editor and left the board silent; `stream_write()` now translates a bare LF to CRLF, because with no tty a bare LF moves down a line without returning to column 0 and every prompt appeared indented by the length of the line above it; and the line editor cannot measure a `funopen()` stream since `fileno(stdin)` is -1, so `link_console_columns()` supplies the width and `cols` sets it by hand. Two attempts at measuring the terminal's width automatically were removed: reading the `ESC[6n` reply meant reading the USB FIFO the shell also reads its input from, and both attempts ate the operator's keystrokes. WiFi had been arriving only as `-DTD_WIFI_SSID` cache variables, so the credentials lived in CMake's cache and a plain build silently shipped a board with no network; clearing that cache once took the node off the air for an hour and I first blamed the USB ring buffers. `wifi` now writes `/fs/etc/wifi.conf`, and it was proven by building an image containing no credential string at all and watching the board still associate. The magnetometer took eight attempts and the fault was mine every time. Per-axis span, then a nominal radius target, then convergence, then octant coverage, then a timed cap each failed differently, and the operator finally said to let them press Enter when the numbers settled, which is what it should have been. The deeper error was the estimator: I replaced P1's min/max with a sphere fit on the strength of a simulation that reported them equal, and that simulation was broken -- its rotation matrix was not orthonormal, so the test points were never on a sphere or ellipsoid at all. That one bad test sent the work back and forth between estimators for several cycles and produced contradictory advice. Re-run correctly, min/max and the sphere fit are both accurate on a full tumble, but the best coverage reachable on this board is 91/86/71 percent, at which a min/max midpoint is 19, 24 and 33 uT out -- so the sphere fit was right for the wrong reason. What the data actually showed was scatter of 2.2 uT, six times the noise floor, meaning the samples lie on an ellipsoid and no three offsets could ever correct them. `magcal` now fits `v M v + 2 n v = 1` and stores a 3x3 matrix plus centre, which maps the ellipsoid back onto a sphere; validated in a prototype at 59 uT raw spread, 30 uT with offsets, 2.2 uT with the matrix, with the centre recovered to 0.04 uT. The operator calibrated, and two independent measurements agree: their on-board rotating test under 3 percent, and a host check reading 45.2 to 46.1 uT across orientations the board had visibly moved between, a 2.0 percent spread against an IGRF field of 46.6 uT for these coordinates. While chasing this I also computed IGRF-14 for the bench -- 46.6 uT total, 23.8 horizontal, 40.0 vertical, dip 59.3 degrees, declination 9.6 east -- which replaced an "about 48 uT" figure I had quoted from memory for two cycles. Two real bugs were found on the way: `rd()` did not set the sub-address MSB for multi-byte reads, which is how the LIS3MDL and BMP581 are told to auto-increment, and `magcal check` added the fitted centre to the stored offsets when that centre is the full bias, double-counting it. The calibration file gained a format tag -- 12 for the matrix, 3 for offsets -- and a file with neither is read as the older bare three-integer format, after a first version of that change silently zeroed the operator's saved offsets. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, inspected the complete `.ai/` diff, confirmed that `.ai/core.md` was not changed and that entries 1 to 14 were not rewritten, checked that the reference's six YAML blocks parse with the index and the records naming the same 83 ids in the same order under the allowed Status values, and validated this entry as number 15 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values and `None.` in Files Modified.

#### Next Steps:

The reference needs the facts this cycle established and they are not written yet: the IGRF field for the bench, that soft-iron distortion on this board requires a matrix rather than offsets, that the LIS3MDL and BMP581 need the sub-address MSB set for multi-byte reads, and that this port's console is TinyDesk Shell over USB or the ILI9340 panel rather than a desktop. The ellipsoid matrices and the `magcal check` verdict path have only been exercised on this one board, so a second unit would be worth running through the same command before the approach is trusted generally. Three defects remain open from earlier entries: LOCATION ships speed, bearing and accuracy derived rather than measured, `node magcal` derives no heading because without an accelerometer there is no tilt compensation, and the P4 telemetry full reply needs a link so it can fail under WiFi jitter, where `node telemetry compact` in one packet always arrives. P4 remains uncommitted to any release and P1 still holds the `.ai` control folder, which will need resolving when P4 becomes its own project.

#### Files Modified:

None.

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: PASS

---

## 16 COMMIT Unreleased 2026-11-01T18:24:40-07:00

#### Coming From:

Unreleased 3e38340

#### Purpose:

Finish the magnetometer calibration, which entry 15 recorded as measuring 2.0 percent but which the operator's own readings then showed 11 to 14 percent high, and bring the `.ai` control folder into P4 now that P4 is the project of record.

#### Outcome:

The calibration loaded and applied correctly, and the fault was in the fit's magnitude rather than its direction.

Reading the board with the matrix loaded gave |B| of 54.6 to 55.7 uT on the desk and 52.6 to 53.8 uT held in the air, against the 46.6 uT field of SEN-008. The dip was the giveaway: in free air it read 58.6 degrees against the modelled 59.3, already correct, while the magnitude was 14 percent high. A vector whose direction is right and whose length is wrong is a gain error and not a shape error, and the two call for different repairs, so no refit was warranted.

The gain error has a cause worth recording because the fit cannot detect it. `ellipsoid_fit()` normalises the corrected radius to the surface the samples actually define, so the correction's absolute scale follows from how much of the ellipsoid was swept. The operator's last run reached 92, 86 and 74 percent of the three axes, which is a partial tumble: the fitted surface was small, every corrected vector landed outside it, and the readings came out long. The fit's own coverage gate passed because a partly covered ellipsoid still fits cleanly; it simply fits to a smaller surface.

`magcal gain` was added for this. It averages eight half-second samples from the shared snapshot, divides the target field magnitude by what it measured, and multiplies all nine coefficients of the stored matrix by that one factor before saving. Scaling all nine coefficients uniformly changes magnitude only, leaving bearing and dip untouched and preserving the soft-iron asymmetry the matrix exists to carry; scaling each axis toward the LIS3MDL's nominal 68.42 counts per microtesla would have thrown that away. Run in free air it measured 53.4 uT, computed 0.87251, and left the board reading 46.3 uT with the bearing moved 1.6 degrees, which is what a pure gain change should do. Five further readings held 46.3 to 46.8 uT at a dip of 60.3 degrees, within 0.2 percent of the modelled field and a degree of its dip. Set back down on the desk the same board reads 44.8 to 45.9 uT and a dip of 61.6 degrees, so the desk costs about 2.6 percent of magnitude and 1.3 degrees of dip, down from the 8 degrees of dip distortion it was causing before the gain was corrected.

Two smaller faults were fixed on the way. `magcal show` printed the offsets-only fallback, so a board running on a matrix reported `X 0 Y 0 Z 0` and the operator could not tell whether anything was loaded; it now prints the matrix and its centre. This entry also records an error of mine that the operator corrected: the loaded matrix had been reported as suspect when in fact it had loaded and was being applied, and the contradiction was resolved only by inverting the matrix against measured vectors to recover the implied raw counts, which is cheaper than building a new firmware to test a theory and should have been the first move.

The control folder moved. P4 now carries `.ai/` with `core.md`, `core-syntax.md`, `core-bootstrap.md`, `core-log.md` and `core-reference.md` copied unmodified except for this entry and the records below, and P1 keeps its own copy for the history it holds. This entry lands in a commit of its own rather than beside the code it describes, because `.ai` arrived in P4 after commit 3e38340 was already pushed to the fork.

The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, confirmed that neither was changed, that entries 1 to 15 were not rewritten, and that the reference's six YAML blocks all parse, with the index and the records naming the same 88 ids in the same order under the allowed Status values, and validated this entry as number 16 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values, and engineering paths in Files Modified.

#### Next Steps:

The rescaled matrix has not yet been through a reboot. It is saved in the same twelve-number format whose loading is proven, so it should survive, but the board has not been restarted since the gain changed and `magcal show` after a reset is the check that closes it. A second Nano ESP32 would show whether the shape-versus-scale split generalises beyond this board, since every measurement behind SEN-008 and SEN-010 comes from one unit. The bearing still cannot be tied to true north without a surveyed reference: readings of 9.3 to 11.4 degrees and of about 0.0 degrees were taken at orientations the assistant could not see, and only the first matched the 9.6 degree declination. Three defects from earlier entries remain open: LOCATION ships speed, bearing and accuracy derived rather than measured, no heading is derived because without an accelerometer there is no tilt compensation, and the P4 telemetry full reply needs a link so it can fail under WiFi jitter where the compact reply always arrives. P1 still holds the copy of `.ai` this one was copied from, and whether that copy is deleted or kept as the old project's archive is the operator's call.

#### Files Modified:

- ports/nanoesp32/main/node.cpp
- ports/nanoesp32/main/sensors.cpp
- ports/nanoesp32/main/sensors.h

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: PASS

---

## 17 COMMIT Unreleased 2026-11-01T18:52:00-07:00

#### Coming From:

Unreleased bbc15b7

#### Purpose:

Report a magnetic heading from the calibrated magnetometer, now that the calibration makes the bearing a number worth sending, and find the slot in the client's telemetry schema that will actually display it.

#### Outcome:

A magnetic heading now travels to Sideband as a CUSTOM telemetry entry and renders as its own labelled row. The firmware computes it in the sensor snapshot from the corrected horizontal field, so the console and the telemetry cannot disagree about the same measurement, and it is withheld below about 5 uT of horizontal field where the angle is noise rather than direction.

Finding the right slot meant reading the client rather than the schema. Sideband defines twenty-seven sensor ids and none of them is a heading. MAGNETIC_FIELD unpacks exactly three floats, so a fourth is read by nobody. LOCATION does carry an element the client calls `bearing` which Location.render() renames to `heading`, and that looked like the intended home until the interface source showed the row is built only inside a branch taken when speed exceeds 0.1 km/h: a stationary node would never have displayed it, and the element's real meaning is GPS course over ground. CUSTOM carries a list of `[label, [value, icon]]` entries and the UI prints each as its own row with the given icon, testing nothing about motion, so the heading goes there as `Magnetic heading` with a compass-rose icon.

Two mistakes were mine and both were caught before the flash. The first CUSTOM payload was one array level short, which would have handed the client a two-element array where it expects a list of entries; the client iterates the value, so it would have walked a string character by character and produced nonsense rather than an error. That was found by encoding the intended bytes in Python and feeding them to Sideband's own Telemeter.from_packed() from the AppImage installed here, which failed and said why. The second was in that same test, which was missing the msgpack map header and produced an "'int' object is not iterable" that looked like a protocol fault and was entirely my harness. The API takes the map, not the field header, and the pack function writes that header first.

Verification was done with the client's own code on the board's own bytes, not by reasoning about the format. The node's 660-byte reply was captured as raw field bytes, passed to Telemeter.from_packed() from the Sideband 2.2.0 installation on this bench, and produced `{'Magnetic heading': ['359.8 degrees', 'compass-rose']}` rendering as the row `Magnetic heading 359.8 degrees` with the compass-rose icon. The same bytes decoded through tools/telemetry_request_test.py showed SID 0xff carrying that list.

The heading is magnetic, as the operator asked, so declination is not applied and it reads 9.6 degrees anti-clockwise of a true bearing at this bench. It is valid without tilt compensation only because the magnetometer is fixed flat and aligned to the GPS module and that assembly is permanently mounted; the board has no accelerometer, so nothing in the firmware could detect tilt if the assembly were moved. That reasoning is recorded rather than assumed, because it is the whole justification for reporting an uncompensated angle.

The heading is in the full map only. Compact sits at 151 packed bytes against the 159-byte single-packet limit measured in entry 15, and roughly forty more would push it onto a link and destroy the property that makes compact mode worth having.

One unrelated fault surfaced while verifying and was not fixed: the client's render of the RNS_TRANSPORT map raised KeyError 'interfaces', because the client reads `ifstats.interfaces` while the node writes `interfaces` as a sibling of `ifstats`. That is a pre-existing mismatch in the transport packing and not a consequence of this change.

The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, confirmed that neither was changed and that entries 1 to 16 were not rewritten, and that the reference's six YAML blocks all parse, with the index and the records naming the same 90 ids in the same order under the allowed Status values, and validated this entry as number 17 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values, and engineering paths in Files Modified.

#### Next Steps:

The RNS_TRANSPORT packing mismatch is real and unfixed: the client expects the interface list inside `ifstats` and the node writes it beside it, so Sideband logs a KeyError while rendering transport telemetry. Whether to move the list or to write both is a small decision nobody has made. The heading itself still has no reference check against a surveyed bearing, so its absolute correctness rests on the magnetometer's alignment to the GPS module rather than on a measurement, and the readings of about 0.0 degrees against a 9.6 degree declination remain unexplained. A second Nano ESP32 would show whether the heading logic and the calibration behind it generalise beyond this board. Three defects from earlier entries remain open: LOCATION ships speed, bearing and accuracy derived rather than measured, the compact telemetry map still omits position and magnetic field, and P1 retains the copy of `.ai` this one came from.

#### Files Modified:

- ports/nanoesp32/main/node.cpp
- ports/nanoesp32/main/sensors.cpp
- ports/nanoesp32/main/sensors.h

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: NOT RUN

---

## 18 COMMIT Unreleased 2026-11-01T18:58:00-07:00

#### Coming From:

Unreleased b2d69a4

#### Purpose:

Record the operator's acceptance of the magnetic heading delivered by entry 17, which that entry left as `User Test: NOT RUN` because the agent-side verification was not the operator looking at the client.

#### Outcome:

The operator confirmed the heading and reported it working, so entry 17's `User Test: NOT RUN` is superseded by acceptance recorded here. Entry 17 is not amended: its status line was accurate when it was written and the log treats committed entries as settled history, so the new information belongs in this entry rather than in an edit to that one.

Entry 17 was also committed a step early, and that is worth recording because the mistake will otherwise repeat. `.ai/core-syntax.md` requires an entry to be written once a cycle reaches a terminal point, and one of those points is the build, deployment and user test having completed. Entry 17 was written and pushed after deployment but before the operator had looked at Sideband, which is why its status line could not honestly read PASS. Marking it NOT RUN and saying so was the right call given the commit had to happen then; holding the entry until the operator had tested would have been better. The rule this restates is that the user test is part of the cycle and not a formality after it.

The heading itself is now confirmed end to end by the person who has to read it: the node computes it from the corrected horizontal field, ships it as a CUSTOM telemetry entry, and Sideband renders it as a row labelled Magnetic heading with a compass-rose icon. What was verified by the agent before this acceptance was that Sideband 2.2.0's own parsing code produced that entry from the board's own field bytes; what the operator has now verified is that the same thing happens in the running client.

No engineering files changed in this cycle. The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, confirmed that neither was changed and that entries 1 to 17 were not rewritten, and that the reference's six YAML blocks all parse, with the index and the records naming the same 90 ids in the same order under the allowed Status values, and validated this entry as number 18 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values, and `None.` in Files Modified.

#### Next Steps:

The RNS_TRANSPORT packing mismatch that entry 17 found is still unfixed: Sideband builds the transport row from `ifstats.interfaces` while the node writes `interfaces` as a sibling of `ifstats`, so the client logs a KeyError every time it renders that sensor. Moving the list or writing it in both places is a small change nobody has approved yet. The heading has still never been checked against a surveyed bearing, so its absolute correctness rests on the magnetometer being physically aligned to the GPS module rather than on a measurement, and the readings near 0.0 degrees against a 9.6 degree declination remain unexplained. A second Nano ESP32 would show whether the calibration, the gain correction and the heading logic generalise beyond this board. Three defects from earlier entries remain open: LOCATION ships speed, bearing and accuracy derived rather than measured, the compact telemetry map omits position and magnetic field, and P1 retains the copy of `.ai` this one came from.

#### Files Modified:

None.

#### Status:

- Build: N/A
- Deployment: N/A
- User Test: PASS

---

## 19 COMMIT Unreleased 2026-11-01T19:14:00-07:00

#### Coming From:

Unreleased 88c654f

#### Purpose:

Take the three cheapest integrations with Sideband that the previous survey identified: an INFORMATION row identifying the node, an icon and colours for it in the client's conversation list, and the repair of the Reticulum Transport sensor that raised a KeyError and was therefore lost entirely.

#### Outcome:

All three are in the firmware and each was confirmed with real bytes rather than by reading the schema. The node now reports SID_INFORMATION as a single line, `TinyNode Nano, tinydesk 0.2.0, up 1m36s, ip 10.10.0.35`, whose version comes from the app descriptor the build embeds rather than from a literal, so it cannot drift from the image. Outgoing messages carry FIELD_ICON_APPEARANCE as `radio-tower` with near-white on teal, and the Reticulum Transport sensor renders instead of vanishing.

The transport repair turned out to be four faults and not the one that had been noticed. Sideband's RNSTransport.render() reads `d["ifstats"]["interfaces"]`, and the interface list had been written as a sibling of `ifstats`, so the lookup raised KeyError and the client discarded the whole sensor. It also reads `d["path_table"]`, `d["link_count"]` and `d["interface_count"]` from the top level, and none of the three was present, so fixing only the nested list would have traded one KeyError for another. The list moved inside `ifstats`, the three keys were added, and the per-interface `path_table` and `link_count` that had been sitting in an interface entry were removed because render() computes those per interface itself.

The appearance field cost compact telemetry its only distinguishing property, and the bench caught it. Adding it made every reply about 26 bytes longer, which took the compact reply from a one-packet 151-byte message to a 175-byte link transfer, so the mode whose entire purpose is to need no link and no known path stopped doing that. Nothing in the code failed and the reply still arrived; only the measured packed_size showed it. The field is now attached only to replies that already travel over a link, and compact measurably returned to 151 bytes at method 1. The two colours are three raw bytes each rather than hex strings, which is what the client reads with its own three-byte unpack, and the icon name was checked against the MaterialDesignIcons glyph table inside the AppImage rather than assumed to exist.

Verification used the client's own code on the board's own bytes. The 752-byte full reply captured as raw field bytes parsed through Sideband 2.2.0's Telemeter.from_packed() to eleven recognised sensors including `information` and a complete `rns_transport` rendering with seventeen values and no error, where before the repair that sensor was dropped. The same message carried field 0x04 as `['radio-tower', f5f5f5, 00695c]`, which decodes through the client's own htf() to the Kivy floats 0.9608 and 0.4118, 0.3608. Two client settings decide whether a peer's appearance is honoured at all, `telemetry_receive_trusted_only` and `display_style_from_all`, and both default such that an untrusted peer is still shown, so the icon does not require the operator to trust the node first.

The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, confirmed that neither was changed and that entries 1 to 18 were not rewritten, and that the reference's six YAML blocks all parse, with the index and the records naming the same 93 ids in the same order under the allowed Status values, and validated this entry as number 19 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values, and engineering paths in Files Modified.

#### Next Steps:

The operator has not yet looked at Sideband since this build, so the icon and the Information row are verified only as far as the client's parsing and rendering code goes. Text sent to the node still does nothing: the delivery callback returns before reading the content unless FIELD_COMMANDS is present, so the message box in Sideband remains a dead end, and that was the largest gain in the survey behind this cycle. The compact map still omits position, magnetic field and heading, so an operator who leaves the node in compact mode never sees where it is. Appearance is deliberately withheld from compact replies and a client that only ever asks for compact telemetry will never receive an icon. Three older items remain open: LOCATION ships speed, bearing and accuracy derived rather than measured, the heading has never been checked against a surveyed bearing, and P1 retains the copy of `.ai` this one came from.

#### Files Modified:

- ports/nanoesp32/main/node.cpp

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: NOT RUN

---

## 20 COMMIT Unreleased 2026-11-01T20:05:00-07:00

#### Coming From:

Unreleased f561a78

#### Purpose:

Port the RYLR998 LoRa interface from the standalone TinyNode into P4, so the node can carry Reticulum over the radio as well as WiFi, and record the operator's acceptance of the work entry 19 delivered.

#### Outcome:

LoRa is running on this board. The framing layer came across unmodified -- it has no Arduino dependencies at all -- and P1's own host test suite passes against the P4 copy with no changes, covering base64 framing, multi-frame reassembly, abandoned-partial counting and the one-MTU overflow refusal. The AT driver did not port so cleanly: it is written against Arduino HardwareSerial, and this port's Arduino shim provides delay, random, String and Esp but no serial class, so the whole driver was rewritten onto the ESP-IDF UART driver on UART1. The TinyTang protocol was removed rather than ported, since this node speaks only Reticulum and every frame that arrives goes up the stack.

The pins did not collide, which was worth checking rather than assuming. P1 had LoRa on D5/D6, which are GPIO 8 and 9; P4's I2C is on A4/A5, which are GPIO 11 and 12, and the panel is on GPIO 18, 21, 38, 47 and 48. So the harness plugs in unchanged. The board's own board_builtin.conf agrees, describing D5/D6 as the pair the TinyNode harness used for its LoRa module.

Two defects were found by using the thing rather than by reading it. First, the `lora` report showed the values the module held before the driver wrote to it: it said address 0, network 18 and power 22 dBm while the boot log two lines earlier had said those had been changed to 1, 6 and 14. That is precisely the silent mismatch the display exists to expose, and it is inherited from P1 rather than introduced here. Each setting is now read back after writing, and a write the module accepts but does not hold is reported as such instead of being logged as success. Second, the Reticulum Transport telemetry still reported one interface after the radio registered, because the interface list was hand-built for UDP alone. It is now enumerated from Transport::get_interfaces(), and Sideband renders both, keyed UDPInterface and RYLR998.

Bring-up on hardware: the module answered AT on the first boot after wiring, reported RYLR998_REYAX_V1.2.4, and accepted the address, network id, air parameters and power settings -- 0 to 1, 18 to 6, SF9 to SF7, and 22 to 14 dBm. The second boot logged every one of those as unchanged, which shows both that the module keeps them in flash and that the read-before-write path works. The link then carried 11 packets in 18 frames with no +ERR and nothing dropped or left queued, so the module accepted and transmitted every frame the driver gave it.

What that does not prove is reception. There is one module on this bench and nothing else on the air, so the receive path -- the +RCV parser, the reassembler and the RSSI/SNR hand-off to the transport -- has not been exercised on this port, and the interface has received nothing. The standalone node had this proven with a second module on a Pico 2, and this port needs the same before the radio can be called working in both directions. Separately, microReticulum exposes no way to enumerate the transport's live paths, only cumulative counters, so the transport sensor reports an empty path table and claims no path count rather than deriving a number that would be wrong.

Entry 19 is superseded on one point: the operator confirmed the icon, the Information row and the Reticulum Transport row in Sideband and reported them working, so its `User Test: NOT RUN` is answered here. Entry 19 is not amended, since a committed entry is settled history.

The core-syntax audit re-read `.ai/core.md` and `.ai/core-syntax.md`, confirmed that neither was changed and that entries 1 to 19 were not rewritten, and that the reference's six YAML blocks all parse, with the index and the records naming the same 94 ids in the same order under the allowed Status values, and validated this entry as number 20 with exactly six sections, prose in Purpose, Outcome and Next Steps, allowed Status values, and engineering paths in Files Modified.

#### Next Steps:

Reception is the next hardware step and it needs a second RYLR998: the same framing and parameters at both ends, then a packet heard through +RCV and handed to the transport. Until then the radio is proven to transmit and unproven to receive, and the path table on this node cannot fill from the air. The transport sensor still under-reports in two ways that microReticulum's API does not currently allow fixing: it claims no path count and an empty path table, so Sideband shows a transport with no reachable destinations even while it is relaying. Text sent to the node in Sideband is still ignored, which remains the largest gap in the operator's access to it and the cheapest thing left to close. Three older items are still open: LOCATION ships speed, bearing and accuracy derived rather than measured, the heading has never been checked against a surveyed bearing, and P1 retains the copy of `.ai` this one came from.

#### Files Modified:

- ports/nanoesp32/main/CMakeLists.txt
- ports/nanoesp32/main/lora.cpp
- ports/nanoesp32/main/lora.h
- ports/nanoesp32/main/lora_frame.cpp
- ports/nanoesp32/main/lora_frame.h
- ports/nanoesp32/main/node.cpp

#### Status:

- Build: PASS
- Deployment: PASS
- User Test: NOT RUN

---
