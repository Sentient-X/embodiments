# ZLG USBCANFD-200U / 400U — USB protocol specification

Status: functional specification for a clean-room implementation. It states facts about the
device's USB protocol as observed from the behaviour of a reference host driver (vendor Linux
driver, version 2.10). It contains no source code from that driver. Everything marked
**[derived]** was constructed by the spec author from the facts below and has not been checked
against hardware. Everything marked **[unknown]** is not established by the reference.

Conventions: all multi-byte integers are **little-endian** unless stated. "Byte n" counts from 0.
Hex bytes are written `55 F2 ...`. Bit 0 is the least-significant bit.

---

## 1. Device identification

| Vendor ID | Product ID | Product |
| --- | --- | --- |
| 0x04CC | 0x1240 | USBCANFD-200U |
| 0x3068 | 0x0009 | USBCANFD-400U |

The reference host matches on VID/PID only (no interface class/subclass match) and uses interface 0.

### 1.1 Endpoints (fixed addresses)

| Address | Direction | Type | Purpose |
| --- | --- | --- | --- |
| 0x01 | OUT | bulk | Command request (configuration commands) |
| 0x81 | IN | bulk | Command reply |
| 0x02 | OUT | bulk | Frame transmit request |
| 0x82 | IN | bulk | Frame transmit reply |
| 0x83 | IN | bulk | Received frames (unsolicited stream) |

The command and transmit paths are independent: a transmit may be in flight while a command is
in progress. The host serialises commands among themselves (one outstanding at a time) and
transmits among themselves (one outstanding at a time).

### 1.2 Vendor control requests on endpoint 0

All are control IN transfers with bmRequestType = **0xA1** (device-to-host, class, recipient
interface), wValue = 0, wIndex = 0.

| bRequest | wLength | Response | Notes |
| --- | --- | --- | --- |
| 0x00 | 4 | Bytes 0–1: protocol version, u16 LE. High byte = major, low byte = minor (minor is conventionally printed in hex, e.g. 0x0116 → "1.16"). Bytes 2–3: **[unknown]**, ignore. | Issue first at probe. On failure treat version as 0. |
| 0x01 | 4 | u32 LE: maximum bulk transfer size the device accepts on the transmit endpoint, in bytes. | Only issue if protocol version ≥ 0x0113. If not issued or it fails, assume 1024. |
| 0xFF | 2 | 2 bytes, content **[unknown]** | Device reset. Issued by the reference host when probing fails, and on explicit user request. The device presumably re-enumerates **[unknown]**. |

### 1.3 Probe sequence

1. Read protocol version (control request 0x00).
2. If version ≥ 0x0113, read max transfer size (control request 0x01); else use 1024.
3. If version ≥ 0x0116: send command 0x46 (read transfer properties, §3.5). If the reply byte is 0,
   send command 0x47 with payload `01` to switch received-frame timestamps to microsecond mode.
   (If the read fails, the host leaves the setting alone.)
4. Compute the frames-per-transmit limit: `floor(max_size / 1024) × 12`, capped at 96.
   With max_size = 1024 this is 12.
5. Send command 0x41 (device information, §3.6). Reply payload is 68 bytes.
6. Choose the CAN controller clock: **80 000 000 Hz** if the reported hardware version ≥ 0x0300
   **or** the reported CAN port count is 4; otherwise **60 000 000 Hz**.
7. Number of CAN channels = min(4, reported CAN port count); LIN channels = min(2, reported LIN port count).
8. On any failure in steps 5–7, the reference host issues the reset control request (0xFF).

---

## 2. Framing common to all bulk messages

Every bulk message (command, command reply, transmit, received data) starts with a header whose
first byte is the sync value **0x55**.

### 2.1 Request header (8 bytes) — used on 0x01, 0x02 and 0x83

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 1 | Sync, always 0x55 |
| 1 | 1 | Command / message type code (§3, §4) |
| 2 | 1 | Argument; for channel-scoped commands this is the **channel index** (0-based), else 0 |
| 3 | 1 | Sequence number. The reference host always sends 0. The device echoes it in the reply. |
| 4 | 4 | Payload length in bytes, u32 LE. Does **not** include the header or the CRC. |

Payload follows, then a 2-byte CRC (§2.3).

### 2.2 Command reply header (12 bytes) — used on 0x81

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 1 | Sync 0x55 |
| 1 | 1 | Echo of the request's command code |
| 2 | 1 | Echo of the request's argument (presumed) |
| 3 | 1 | Echo of the request's sequence number |
| 4 | 1 | Status: 0 = success, any non-zero value = failure (meanings **[unknown]**) |
| 5 | 3 | Reserved |
| 8 | 4 | Reply payload length in bytes, u32 LE |

### 2.3 CRC

* Algorithm: CRC-16 with polynomial 0x8005, processed LSB-first (reflected; the reflected polynomial
  constant is 0xA001), initial value **0xFFFF**, input and output reflected, **no final XOR**.
  This is the parameter set commonly catalogued as CRC-16/MODBUS.
* Coverage: the 8-byte request header **plus** the `length` payload bytes, contiguous.
* Placement: immediately after the last payload byte, stored **little-endian** (low byte first).
* Check value: CRC of ASCII `123456789` = **0x4B37** (bytes `37 4B`).

Bitwise definition: `crc = 0xFFFF; for each byte b: crc ^= b; repeat 8 times: if crc & 1: crc = (crc >> 1) ^ 0xA001 else crc >>= 1`.

Worked examples (computed independently with two implementations — the reflected bitwise form
above and a non-reflected 0x8005 form on bit-reversed input — which agree):

| Bytes covered | CRC | On the wire |
| --- | --- | --- |
| `31 32 33 34 35 36 37 38 39` ("123456789") | 0x4B37 | `37 4B` |
| `55 01 00 00 00 00 00 00` (device open) | 0xC895 | `95 C8` |
| `55 41 00 00 00 00 00 00` (device information) | 0x0CD4 | `D4 0C` |
| `55 12 00 00 00 00 00 00` (start channel 0) | 0x09B7 | `B7 09` |
| `55 13 00 00 00 00 00 00` (stop channel 0) | 0xC9A7 | `A7 C9` |
| `55 02 00 00 00 00 00 00` (device close) | 0xC8A6 | `A6 C8` |
| `55 46 00 00 00 00 00 00` (read transfer properties) | 0xCCA2 | `A2 CC` |
| `55 47 00 00 01 00 00 00 01` (enable µs timestamps) | 0x7570 | `70 75` |
| `55 18 00 00 01 00 00 00 55` (termination on, ch 0) | 0xF634 | `34 F6` |

The reference host does **not** verify CRCs on anything it receives (command replies, transmit
replies, received data). Whether the device appends a CRC to those is therefore not proven; the
evidence (a received transfer must be ≥ 2 bytes longer than its header + payload to be accepted in
size checks, and a zero-payload command reply's data phase returns ≥ 2 bytes) suggests it does
**[unknown]**.

---

## 3. Command channel (endpoints 0x01 / 0x81)

### 3.1 Transaction

Each command is four bulk transfers, in order:

1. OUT 0x01: the 8-byte request header, exactly 8 bytes.
2. OUT 0x01: a data block of **exactly 1024 bytes**: the payload, then the 2-byte CRC, then zero
   fill to 1024. This block is sent even when the payload length is 0 (it then begins with the CRC).
3. IN 0x81: read exactly 12 bytes — the reply header (§2.2).
4. IN 0x81: read up to 1024 bytes — the reply data (payload first; trailing bytes presumed CRC).

A reply is accepted when: reply command = request command, reply sequence = request sequence,
status = 0, and reply payload length = the length expected for that command (table below).
The reference host additionally requires the data phase in step 4 to have returned at least 2 bytes.

Timeouts: 500 ms per bulk transfer (reference default, configurable). If any of transfers 1, 3, 4
fails at the USB level, the whole four-step transaction is repeated, up to **3 attempts** total.
A failure of step 2 alone is tolerated by the reference host (it carries on to read the reply).
A non-zero status is **not** retried; it is reported as failure.

### 3.2 Command codes

| Code | Name (neutral) | Argument | Request payload | Expected reply payload length | Used by reference |
| --- | --- | --- | --- | --- | --- |
| 0x01 | Device open | 0 | none | 0 | yes |
| 0x02 | Device close | 0 | none | 0 | yes |
| 0x03 | Security verification | ? | **[unknown]** | ? | no |
| 0x04 | Read log size | ? | **[unknown]** | ? | no |
| 0x05 | Read log data | ? | **[unknown]** | ? | no |
| 0x06 | Set real-time clock | ? | **[unknown]** | ? | no |
| 0x07 | Test | ? | **[unknown]** | ? | no |
| 0x11 | CAN channel initialise | channel | 20 bytes, §3.3 | 0 | yes |
| 0x12 | CAN channel start | channel | none | 0 | yes |
| 0x13 | CAN channel stop | channel | none | 0 | yes |
| 0x14 | Configure acceptance filter | channel (presumed) | filter entries, §3.7 | ? | no |
| 0x15 | Read error counters | channel (presumed) | **[unknown]** | ? | no |
| 0x16 | Periodic-transmit configuration | ? | **[unknown]** | ? | no |
| 0x17 | Periodic-transmit control | ? | **[unknown]** | ? | no |
| 0x18 | Termination resistor | channel | 1 byte, §3.4 | 0 | yes |
| 0x41 | Device information | 0 | none | 68 (0x44) | yes |
| 0x44 | Set transmit timeout | 0 (device-wide) | 4 bytes, §3.4 | 0 | yes |
| 0x46 | Read transfer properties | 0 | none | 1 | yes (version ≥ 0x0116) |
| 0x47 | Write transfer properties | 0 | 1 byte | 0 | yes (version ≥ 0x0116) |
| 0x60 | LIN channel initialise | LIN channel | ASCII JSON text | 0 | LIN only — out of scope |

Codes 0xF1–0xF4 are frame-transfer types (§4, §5), not commands.

### 3.3 CAN channel initialise (0x11) — 20-byte payload

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 4 | Controller clock in Hz, u32 LE (80 000 000 or 60 000 000, chosen per §1.3 step 6) |
| 4 | 4 | Mode bits, u32 LE. Bit 0: 0 = normal, 1 = listen-only. Bit 1: selects ISO vs non-ISO (Bosch) CAN FD; presumably 0 = ISO, 1 = Bosch **[unknown polarity]**. Other bits reserved, 0. The reference host always sends 0. |
| 8 | 1 | Nominal (arbitration) TSEG1 − 1, where TSEG1 = propagation segment + phase segment 1, in time quanta |
| 9 | 1 | Nominal TSEG2 − 1 (phase segment 2, in tq) |
| 10 | 1 | Nominal SJW − 1 |
| 11 | 1 | Nominal sampling mode. Reference always sends 0. Presumably 1 = triple sampling **[unknown]** |
| 12 | 2 | Nominal prescaler − 1, u16 LE |
| 14 | 1 | Data-phase TSEG1 − 1 |
| 15 | 1 | Data-phase TSEG2 − 1 |
| 16 | 1 | Data-phase SJW − 1 |
| 17 | 1 | Data-phase sampling mode (0) |
| 18 | 2 | Data-phase prescaler − 1, u16 LE |

Bit rate = clock / (prescaler × (1 + TSEG1 + TSEG2)) using the real (un-decremented) values.
Sample point = (1 + TSEG1) / (1 + TSEG1 + TSEG2).

Ranges the reference host permits (real values, before the −1 encoding):

| Field | Nominal | Data phase |
| --- | --- | --- |
| TSEG1 | 1 – 256 | 1 – 16 |
| TSEG2 | 1 – 128 | 1 – 16 |
| SJW | 1 – 128 | 1 – 16 |
| Prescaler | 1 – 512 | 1 – 32 |

Classic CAN (FD disabled): the data-phase fields are filled with the same values as the nominal
fields. There is no separate "FD enable" bit in this command; FD vs classic is chosen per frame
(§4.2, format field).

### 3.4 Single-value commands

* **Termination resistor (0x18)**, argument = channel, 1-byte payload: `55` = enable the 120 Ω
  termination, `AA` = disable. The reference default is enable.
* **Transmit timeout (0x44)**, argument = 0, 4-byte payload: u32 LE milliseconds, clamped by the
  reference host to 1 – 4000; reference default **20 ms**. Interpreted as how long the device keeps
  trying to transmit a frame before giving up **[presumed]**. It is device-wide (argument is 0, not
  a channel), but the reference re-sends it on every channel open.
* **Write transfer properties (0x47)**, 1-byte payload: `01` = received-frame timestamps in
  microseconds (§5.3), `00` = 100 µs units.

### 3.5 Read transfer properties (0x46)

Reply payload 1 byte: 1 = microsecond timestamp mode active, 0 = not.

### 3.6 Device information reply (0x41), 68 bytes

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 2 | Hardware version, u16 LE (e.g. 0x0300 = 3.00) |
| 2 | 2 | Firmware version, u16 LE |
| 4 | 2 | Bootloader version, u16 LE |
| 6 | 40 | Device identity string, ASCII, NUL-padded (presumed) |
| 46 | 20 | Serial number, ASCII, **may not be NUL-terminated** (treat as up to 20 characters) |
| 66 | 1 | Number of CAN ports |
| 67 | 1 | Number of LIN ports |

### 3.7 Acceptance filter entry (command 0x14; unused by reference)

One entry is 12 bytes: byte 0 = 0 standard / 1 extended identifier; bytes 1–3 reserved;
bytes 4–7 first identifier of range, u32 LE; bytes 8–11 last identifier of range, u32 LE.
How entries are counted, enabled or cleared in the payload is **[unknown]**.

### 3.8 Channel lifecycle

Open a CAN channel N:

1. Ensure bulk IN reads on 0x83 are already posted (§5) — the reference posts them before step 2.
2. If no channel of this device is currently open: command **0x01** (device open). Keep an open-count.
3. Command **0x11** with argument N (timing).
4. Command **0x44** (transmit timeout).
5. Command **0x18** with argument N (termination).
6. Command **0x12** with argument N (start). The channel is now error-active and on the bus.

Close channel N: command **0x13** with argument N. If it was the last open channel, then command
**0x02** (device close).

Reset: there is no per-channel reset command; stop (0x13) then re-initialise (0x11…0x12). A whole-
device reset is the control request 0xFF.

To change bit rate, stop the channel and repeat steps 3–6 **[presumed; reference never re-inits a
running channel]**.

---

## 4. Transmitting frames (endpoints 0x02 / 0x82)

### 4.1 Transmit request

One bulk OUT transfer on 0x02 of exactly `8 + length + 2` bytes (no padding to 1024):

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 8 | Request header (§2.1): sync 0x55, type **0xF2**, argument = channel index, sequence 0, length = `4 + count × 80` |
| 8 | 2 | Frame count, u16 LE (1 … limit from §1.3 step 4, absolute maximum 96) |
| 10 | 2 | Reserved, 0 |
| 12 | count × 80 | Frame records, §4.2 |
| 12 + count×80 | 2 | CRC (§2.3) over everything before it |

The reference host always uses type 0xF2 and 80-byte records for transmit, for classic frames as
well as FD frames, and puts frames for **one channel** per transfer (argument and every record's
channel byte agree).

Largest transfer with 12 frames: 8 + 4 + 960 + 2 = 974 bytes. With 96 frames: 7694 bytes (needs a
device that reported max size ≥ 8192).

### 4.2 Frame record (80 bytes; also the receive record for type 0xF2)

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 4 | Timestamp low 32 bits, u32 LE. Transmit: 0. Receive: §5.3 |
| 4 | 4 | Identifier, u32 LE: 11-bit or 29-bit value in bits 0–28. (The reference host leaves its OS's flag bits in bits 29–31 on transmit — bit 31 extended, bit 30 remote, bit 29 error — and the device evidently ignores them; an implementation should send them as 0.) |
| 8 | 4 | Flags word, u32 LE, §4.3 |
| 12 | 2 | Transmit: 0. Receive: timestamp bits 32–47 in microsecond mode (§5.3) |
| 14 | 1 | Channel index (0-based) |
| 15 | 1 | Data length in **bytes** (0–8 classic; 0–8, 12, 16, 20, 24, 32, 48, 64 FD) — not a DLC code |
| 16 | 64 | Data, first `length` bytes significant, rest zero |

A classic-only receive record (type 0xF1, §5.2) has the same 16-byte head followed by **8** data
bytes (24 bytes total).

### 4.3 Flags word

| Bits | Meaning |
| --- | --- |
| 0–3 | Transmit mode: 0 = normal (automatic retransmission), 1 = single-shot (no retransmission), 2 = normal + self-reception, 3 = single-shot + self-reception. Reference always sends 0. Receive: **[unknown]**, ignore |
| 4–7 | Format: 0 = classic CAN 2.0, 1 = CAN FD |
| 8 | Remote frame (1 = RTR) |
| 9 | Extended identifier (1 = 29-bit) |
| 10 | Error flag **[meaning on data records unknown]** |
| 11 | Bit-rate switch (FD only) |
| 12 | Error-state indicator (ESI; FD only) |
| 13 | "Transmitted" flag — presumably set on received records that are this device's own transmissions **[unknown]** |
| 14 | Echo flag — presumably set on self-reception copies **[unknown]** |
| 15 | Quick-send flag **[unknown]**; send 0 |
| 16–19 | Timestamp overflow count (receive), §5.3 |
| 20 | Timestamp unit: 1 = microseconds, 0 = 100 µs (receive) |
| 21–31 | Reserved, 0 |

The reference host sets BRS and ESI only for FD frames.

### 4.4 Transmit reply

Read one bulk IN transfer on 0x82, 8 bytes:

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 1 | Sync 0x55 |
| 1 | 1 | Echo of type (0xF2) |
| 2 | 1 | Echo of argument (channel) |
| 3 | 1 | Echo of sequence |
| 4 | 1 | Status: 0 = accepted; non-zero = failure **[codes unknown]** |
| 5 | 1 | Number of frames sent successfully |
| 6 | 2 | CRC **[coverage unverified; reference does not check]** |

The reference accepts the reply when sync = 0x55, type matches and status = 0. It does not use the
sent-count; whether a partial count can occur with status 0 is **[unknown]**.

Timing / flow control: the write has a 1000 ms timeout; the reply read has (1000 + transmit
timeout) ms. The host does not send the next transmit transfer until the reply (or timeout) for the
previous one. That ack-per-batch is the only flow control; there is no credit or window. The reply
is presumed to arrive after the device has attempted the frames (hence the timeout is extended by
the device transmit timeout) **[presumed]**.

Echo semantics: the reference host reports frames as transmitted locally as soon as they are handed
to USB; it does not wait for a device echo, and it never enables self-reception. A failed transmit
is logged but the frames are not retried.

---

## 5. Receiving frames (endpoint 0x83)

### 5.1 Transfer

The host keeps several bulk IN reads posted on 0x83 at all times while any channel is open
(reference: 16 reads of 16384 bytes each) and re-posts each one as soon as it completes. The device
sends data unsolicited.

Each completed transfer is one message:

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 8 | Header (§2.1): sync 0x55, type (table below), argument **[meaning unknown; ignore]**, sequence **[ignore]**, length = `4 + count × record_size` |
| 8 | 2 | Record count, u16 LE |
| 10 | 2 | Reserved |
| 12 | count × record_size | Records, all of the same type |
| … | 2 | Presumed CRC **[unverified]** |

| Type | Records | Record size |
| --- | --- | --- |
| 0xF1 | Classic CAN frames | 24 (16-byte head + 8 data) |
| 0xF2 | CAN FD-capable frames (may contain classic frames; check format bits 4–7) | 80 |
| 0xF3 | CAN error events | 24 (16-byte head + 8 data) |
| 0xF4 | LIN frames | 24 — skip (§6) |

Validation the reference applies: sync must be 0x55, type must be one of the four, and length must
not exceed (buffer size − 10). Invalid transfers are dropped whole. Records whose channel byte is
≥ the number of ports of that kind are skipped. A transfer contains frames from any mix of channels;
each record's channel byte is authoritative.

For 0xF1 records, the format bits may also be consulted; the reference does so, and treats format 1
as an FD frame.

### 5.2 Decoding a data record (0xF1 / 0xF2)

* Identifier = bytes 4–7 (bits 0–28).
* Extended = flags bit 9; remote = bit 8; FD = format ≠ 0; BRS = bit 11; ESI = bit 12.
* Length = byte 15 (bytes); data = the next `length` bytes from offset 16.
* Channel = byte 14.

### 5.3 Timestamps

* Microsecond mode (flags bit 20 = 1): time in µs = `ts32 | (bytes12-13 << 32) | (overflow << 48)`.
* 100 µs mode (bit 20 = 0): time in µs = `(ts32 | (overflow << 32)) × 100`.

Timestamps are relative to a device-internal epoch **[unknown origin]**; the reference host anchors
the first one it sees to the host clock and offsets all later ones. Wrap behaviour beyond the
overflow field is **[unknown]**.

### 5.4 Error records (0xF3)

The reference treats each 0xF3 record as one bus error on the channel in byte 14 and does not decode
its 8 data bytes. Their layout, error-type codes, bus-state (error-warning / passive / bus-off)
reporting and error counter values are **[unknown]**. The reference never reads error counters
(command 0x15) and never reports bus-off, so a clean-room implementation cannot get bus state from
the reference; it must be discovered on hardware.

---

## 6. LIN

Out of scope, except:

* LIN frames arrive on the same endpoint 0x83 as type **0xF4**, 24-byte records whose channel byte
  (byte 8 of the LIN record, not byte 14) is a LIN channel index. A CAN-only implementation must skip
  0xF4 transfers entirely.
* LIN transmits share endpoint 0x02 (type 0xF4); LIN configuration is command 0x60 with a JSON text
  payload. A CAN-only implementation need not send either.
* Disabling a LIN port is command 0x60 with a JSON text disabling it; not needed for CAN.

---

## 7. Worked examples **[derived]**

### 7.1 Initialise channel 0 for classic CAN at 1 Mbit/s (80 MHz clock)

Timing chosen: prescaler 2, TSEG1 29, TSEG2 10, SJW 2 → 80 MHz / (2 × 40) = 1 000 000 bit/s,
sample point 30/40 = 75 %. Encoded: TSEG1−1 = 0x1C, TSEG2−1 = 0x09, SJW−1 = 0x01, prescaler−1 = 0x0001.
Data-phase fields repeat the nominal ones.

Header (OUT 0x01, 8 bytes):

```
55 11 00 00 14 00 00 00
```

Data block (OUT 0x01, 1024 bytes): payload (20 bytes), CRC, zero fill:

```
00 B4 C4 04   00 00 00 00   1C 09 01 00 01 00   1C 09 01 00 01 00   97 EA   00 … (to 1024 bytes)
clock=80e6    mode=0        nominal             data                CRC 0xEA97
```

Expected reply header (IN 0x81): `55 11 00 00 00 00 00 00 00 00 00 00` (status 0, length 0),
followed by a data phase of ≥ 2 bytes.

The full open of channel 0 is, in order (each a §3.1 transaction; header shown, then the start of
the 1024-byte data block):

| Step | Header | Data block begins |
| --- | --- | --- |
| Device open | `55 01 00 00 00 00 00 00` | `95 C8 00 …` |
| Initialise | `55 11 00 00 14 00 00 00` | as above |
| Transmit timeout 20 ms | `55 44 00 00 04 00 00 00` | `14 00 00 00 EC 3C 00 …` |
| Termination on | `55 18 00 00 01 00 00 00` | `55 34 F6 00 …` |
| Start | `55 12 00 00 00 00 00 00` | `B7 09 00 …` |

For a 60 MHz unit at 1 Mbit/s: prescaler 1, TSEG1 44, TSEG2 15 (60 tq, 75 %) is one valid choice.

### 7.2 Transmit one classic frame on channel 0

Standard id 0x7FF, 8 bytes `01 00 33 0A 00 00 00 00`. One OUT transfer on 0x02, 94 bytes:

```
55 F2 00 00 54 00 00 00                      header: type F2, ch 0, length 84
01 00 00 00                                  count 1, reserved
00 00 00 00                                  timestamp 0
FF 07 00 00                                  id 0x7FF
00 00 00 00                                  flags: normal tx, classic, data, standard
00 00                                        reserved
00                                           channel 0
08                                           length 8
01 00 33 0A 00 00 00 00                      data
00 × 56                                      data padding to 64
3B 0E                                        CRC 0x0E3B over the preceding 92 bytes
```

Expected reply on 0x82: `55 F2 00 00 00 01 xx xx` (status 0, 1 frame sent, CRC bytes unverified).

### 7.3 Decode a received classic frame

Received on 0x83 (38 bytes):

```
55 F1 00 00 1C 00 00 00   01 00 00 00
40 E2 01 00   23 01 00 00   00 00 10 00   00 00   01   08
DE AD BE EF 01 02 03 04
E3 53
```

* Header: sync 0x55, type 0xF1 (classic), length 0x1C = 28 = 4 + 1 × 24. Count = 1.
* Record: ts32 = 0x0001E240 = 123 456; id = 0x123; flags = 0x00100000 → bit 20 set (µs mode),
  format 0 (classic), not remote, not extended; bytes 12–13 = 0 (ts high); channel = 1; length = 8.
* Time = 123 456 µs (0.123456 s since device epoch). Frame: channel 1, std id 0x123, data
  `DE AD BE EF 01 02 03 04`.
* Trailing `E3 53` is the CRC 0x53E3 (low byte first) over the 36 preceding bytes, if the device appends one.

---

## 8. Unknowns and ambiguities

1. Whether the device appends a CRC to command replies, transmit replies and received transfers, and
   whether it checks/rejects requests with a bad CRC (reference never sends a bad one).
2. Meaning of non-zero status codes in command and transmit replies.
3. Meaning of the argument and sequence bytes on received (0x83) transfers.
4. Whether the sequence number must increment; reference always uses 0.
5. Bytes 2–3 of the protocol-version control reply; the content of the reset reply; whether reset
   causes re-enumeration.
6. Polarity of mode bit 1 (ISO vs Bosch FD); whether listen-only (bit 0) works — never exercised.
7. The sampling-mode byte's values (presumed 0 = single, 1 = triple sample).
8. Exact semantics of the transmit timeout (per frame vs per batch) and of the transmit-reply
   sent-count; whether partial success is possible with status 0; whether the reply waits for
   bus completion or only for queueing.
9. Self-reception / echo: what the device returns on 0x83 with transmit modes 2 or 3, and the meaning
   of flags bits 10, 13, 14, 15 on received records.
10. Error records (0xF3): data-byte layout, error kinds, and how/if bus state (warning, passive,
    bus-off) and error counters are reported; payload format of command 0x15.
11. Filters (0x14), periodic transmit (0x16/0x17), RTC (0x06), log (0x04/0x05), security (0x03), test
    (0x07): payload formats not established.
12. Whether a single transmit transfer may mix channels or mix 0xF1-size records; reference always
    uses one channel and 80-byte 0xF2 records.
13. Whether channel-scoped commands may be sent while another channel is running (reference does so
    for open/init of additional channels, so presumably yes).
14. Device behaviour on bulk-OUT short writes for commands (the 1024-byte data block is always
    sent full; whether a shorter block is accepted is unknown).
15. Byte order is inferred from a host driver that only runs on little-endian machines.
16. USB speed and the endpoints' wMaxPacketSize are not stated by the reference; read them from the
    descriptors.
17. Timestamp epoch and wrap behaviour; whether 100 µs mode is the power-on default on firmware with
    protocol version < 0x0116.
18. Clock selection rule (80 vs 60 MHz) is a heuristic from hardware version / port count; the device
    may report its clock elsewhere.

---

## 9. Performance-relevant behaviour

This section describes what the reference host does that sets its throughput and latency, so a
port can match it when measured. Items marked **[inferred]** follow from the facts but were not
directly observed.

### 9.1 Receive path

* **Reads kept in flight:** 16 bulk IN reads on 0x83 (configurable, at most 16), each with a
  **16384-byte** buffer (configurable). They are posted when the first channel opens and stay posted
  until the device is unplugged. Closing every channel does not cancel them.
* **Re-posting:** each read is posted again from its own completion handler, right after its
  records are handed on, so up to 16 completions can queue in the host controller with no gap.
  A userspace port should keep at least as many reads of the same size in flight (for example, async
  libusb transfers) and re-submit as soon as each completes. A single synchronous read loop will
  fall behind at high bus load **[inferred]**.
* **Records per transfer:** the device decides how many. The host only requires that the payload
  fit the buffer (payload length ≤ buffer − 10). With a 16384-byte buffer that is at most
  **204** records of 80 bytes (type 0xF2), or **682** records of 24 bytes (0xF1 / 0xF3). The count
  field is u16. How long the device waits before sending a partly filled transfer, and whether a
  larger host buffer changes its packing, are **[unknown]**. They set receive latency under light
  load and should be measured.
* **One type per transfer:** every record in a transfer is the same type. Records from different
  channels share a transfer.
* **Host work per record:** a channel range check, the timestamp arithmetic (§5.3) and a copy of the
  data. The offset from device time to wall-clock time is fixed once, at the first frame. Nothing
  else happens per record: no reply, no request.

### 9.2 Transmit path

* **Batching:** the host keeps a per-channel software queue of up to **10240** frames. One host
  worker serves all channels in turn. On each pass it takes whatever is queued on a channel, up to
  the per-transfer limit, and sends it as **one** transfer. It never waits to fill a batch: a single
  queued frame is sent alone at once, and batches only form when frames pile up during the previous
  round trip. The per-transfer limit is `floor(max_size / 1024) × 12`, capped at 96, where max_size
  comes from control request 0x01 (1024 if not reported, giving 12).
* **Waiting for the reply:** after each transmit transfer the host blocks reading the 8-byte reply on
  **0x82** before it sends anything else. Only **one transmit transfer is outstanding at a time,
  across all channels**: channel B's frames wait while channel A's reply is pending. Peak transmit
  rate is therefore about (frames per batch) ÷ (one write + reply round trip). The reply holds the
  status and the count of frames sent (§4.4). The typical reply delay is **[unknown]**; measure it.
  Because the read timeout is extended by the device transmit timeout, the reply is probably sent
  after the device has tried the frames on the bus, not on receipt **[inferred]**. Whether the
  device accepts several transmit transfers in flight (pipelining) is **[unknown]**. The reference
  never tries, so to match it a port need not either.
* **Reply delay compared with bus time [inferred, unmeasured]:** if the reply waits for the device
  to finish the batch on the bus, its delay ≈ the bus time of the batch + USB turnaround. A classic
  8-byte standard-id frame takes about 111 bits plus stuffing (≈ 111–135 µs at 1 Mbit/s), so a
  12-frame batch is ≈ 1.3–1.6 ms. An FD 64-byte frame at 1 M/5 M is ≈ 150–200 µs, so 12 of them
  ≈ 2 ms. USB turnaround is ≥ one bus interval (125 µs microframe at high speed, 1 ms frame at
  full speed). If instead the device replies on queueing, the delay is close to the USB turnaround
  alone. The reference's 1000 + transmit-timeout read limit does not tell the two apart; measure it.
* **Wake-up:** putting a frame in the queue wakes the worker immediately. When every queue is empty,
  the worker sleeps until the next frame is queued (it also checks again periodically while idle),
  so the idle period adds no latency.
* **Host queue full:** the per-channel software queue holds 10240 frames (CAN and LIN alike). When 1 or fewer slots are free, the OS send queue is stopped, so senders block or get a 'no buffer' error. If it is already full, the frame is refused with 'busy' for the OS to retry later. Nothing is dropped silently. After a USB disconnect error, sends are refused 'busy' and the queue stops.
* **Back-pressure:** the operating system's send queue for a channel is stopped only when its
  software queue has 1 or fewer free slots, and restarted after each batch is sent. The only signal
  from the device is the transmit reply. There is no credit count, no "buffer full" message and no
  NAK-based pacing that the host interprets. Device-side FIFO depths are **[unknown]**.
* **Failure handling:** a failed transmit (bad reply or timeout) is logged. The frames are counted as
  sent and **not** retried.

### 9.3 Echo, loopback, polling

* **Local echo:** each transmitted frame is echoed back to local listeners from the host's own copy,
  when the batch is built — before the transfer is even written and regardless of the reply. A port
  needs this only if its users expect to see their own transmissions. It costs no USB traffic.
* **Device self-reception** (transmit modes 2 and 3) is never used. Loopback is advertised but forced
  off. No echoed frames come back over USB from the device.
* **No polling:** the host sends no periodic requests. It never reads error counters or bus state, and
  error records (0xF3) arrive only when the device pushes them. After channel start, the command
  endpoints are idle until the channel stops. A port that adds error-counter or status polling (for
  example, to report bus-off) adds command traffic the reference does not have. That traffic runs on
  0x01/0x81, which is independent of the frame endpoints, but its effect on frame timing is
  **[unknown]**.

### 9.4 Timestamps

* **Microsecond mode** (selected at probe on protocol ≥ 0x0116): 1 µs resolution, value up to 52 bits
  (≈ 142 years). **100 µs mode:** 100 µs resolution, 36-bit tick count (≈ 80 days).
* What the timestamp marks (start of frame, end of frame, or time put into the device FIFO) and its
  accuracy are **[unknown]**. The device clock is not synchronised to the host. The reference only
  adds a fixed offset taken at the first received frame, so drift between the two clocks is not
  corrected.
* Transmitted frames carry timestamp 0. The host gets no device time for its own transmissions.

### 9.5 Bounding timeouts (worst-case latency)

| Operation | Bound |
| --- | --- |
| One command transfer | 500 ms each (4 per transaction); 3 attempts on USB error → worst ≈ 6 s per command |
| Transmit write | 1000 ms |
| Transmit reply | 1000 ms + device transmit timeout (default 20 ms; 1–4000) = 1020 ms by default |
| Device transmit timeout | 20 ms default: presumably how long the device tries a frame before giving up **[inferred]**. With bus-off or no acknowledging node, a batch costs about this long |
| Receive | no timeout; reads wait indefinitely |
| Control requests at probe | the OS's standard control timeout (5 s on Linux) |

A stalled transmit reply blocks all channels' transmissions for up to about 1 s, because the path is
single-file.

### 9.6 USB transfer sizes

* The reference never reads endpoint descriptors and never states the USB speed. Read the speed and
  each endpoint's wMaxPacketSize from the descriptors **[unknown]**. The "1024" used throughout is a
  transfer size, not a packet size.
* Transmit transfers are `14 + 80 × n` bytes. That is never a multiple of 64 or 512, so a transmit
  transfer always ends in a short packet, and no zero-length packet question arises.
* The command data block is exactly 1024 bytes, a whole number of 512- or 64-byte packets. The
  reference sends **no** zero-length packet after it, so the device must use the length field
  rather than a short packet to detect the end. A port should likewise not add one **[inferred]**.
* Receive transfers end when the device sends a short packet (or fills the buffer). A full 16384-byte
  buffer is a whole number of packets.
