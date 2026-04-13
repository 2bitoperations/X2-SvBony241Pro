# SVBony SV241 Pro Power Box — Serial Protocol Reference

## Source & Attribution

This protocol documentation was reverse-engineered from the INDI driver source:

- **Project:** INDI (Instrument-Neutral Distributed Interface)
- **Repository:** https://github.com/indilib/indi
- **Files:** `drivers/power/svbony_powerbox.cpp`, `drivers/power/svbony_powerbox.h`
- **Author:** Tetsuya Kakura
- **License:** GNU General Public License v2.0 or later (GPL-2.0+)

---

## Table of Contents

1. [Connection Parameters](#1-connection-parameters)
2. [Device Architecture](#2-device-architecture)
3. [Message Framing](#3-message-framing)
4. [Checksum Algorithm](#4-checksum-algorithm)
5. [Command Set Reference](#5-command-set-reference)
6. [Response Parsing](#6-response-parsing)
7. [Power Port Control](#7-power-port-control)
8. [USB Port Control](#8-usb-port-control)
9. [Dew Heater Control](#9-dew-heater-control)
10. [Variable Voltage Port Control](#10-variable-voltage-port-control)
11. [Sensor Readings](#11-sensor-readings)
12. [Status / State Polling](#12-status--state-polling)
13. [Initialization Sequence](#13-initialization-sequence)
14. [Error Handling and Timeouts](#14-error-handling-and-timeouts)
15. [Power Cycle Sequence](#15-power-cycle-sequence)
16. [Dew Point Calculation](#16-dew-point-calculation)
17. [Worked Examples](#17-worked-examples)
18. [Quick Reference Tables](#18-quick-reference-tables)

---

## 1. Connection Parameters

| Parameter    | Value         |
|-------------|---------------|
| Baud rate   | 115200        |
| Data bits   | 8             |
| Parity      | None          |
| Stop bits   | 1             |
| Flow control| None (RTS/DTR are explicitly **cleared** during handshake — see Section 13) |
| Connector   | USB CDC (appears as a serial port, e.g. `/dev/ttyUSB0` or `/dev/ttyACM0`) |

> **Important:** The firmware runs on an ESP32 microcontroller (evident from the boot log messages containing `ets Jun 8 2016`, `POWERON_RESET`, `SPI_FAST_FLASH_BOOT`, `entry 0x400805f0`). The USB connection is a USB-to-serial bridge. On connect the ESP32 resets itself, producing a burst of boot-loader text before it is ready for binary commands.

---

## 2. Device Architecture

The SV241 Pro exposes the following controllable and measurable resources:

| Resource           | Count | Notes                                              |
|--------------------|-------|----------------------------------------------------|
| DC power ports     | 5     | Full on/off, labeled DC 1–5                        |
| USB port groups    | 2     | Group 0 = USB-C + USB 1 + USB 2; Group 1 = USB 3–5 |
| PWM dew heaters    | 2     | PWM 1 and PWM 2, 0–100% duty cycle                |
| Regulated (variable) output | 1 | 0.0 V – 15.3 V, labeled REGULATED       |
| INA219 power/voltage/current sensor | 1 | Measures overall bus power         |
| DS18B20 temperature sensor | 1 | External lens temperature probe               |
| SHT40 combined sensor | 1   | Ambient temperature + relative humidity          |
| Dew point          | —     | Calculated in software from SHT40 readings         |

**Unsupported features** (driver stubs return false):
- Per-port current sensing
- LED toggle
- Auto-dew
- Over-voltage protection
- Power-off on disconnect

---

## 3. Message Framing

All communication after the boot sequence uses a **fixed-length binary framing scheme**. There is no ASCII text, no newline delimiter, and no null terminator. Every frame begins with the byte `0x24` (`'$'`).

### 3.1 Command Frame Layout

```
Byte offset   Field           Size    Description
-----------   -----           ----    -----------
0             FRAME_HEADER    1 byte  Always 0x24 ('$')
1             DATA_LEN        1 byte  Total frame length (header + data_len + cmd bytes + checksum)
2 ... N       CMD             1–3 bytes  Command byte(s) and optional parameter bytes
N+1           CHECKSUM        1 byte  Sum of all preceding bytes modulo 0xFF
```

**Formula for DATA_LEN:**

```
DATA_LEN = 2 + cmd_payload_len + 1
         = cmd_payload_len + 3
```

Where `cmd_payload_len` is the number of bytes in the CMD field (1, 2, or 3 bytes depending on the command). So:

| cmd_payload_len | DATA_LEN | Total frame length |
|-----------------|----------|--------------------|
| 1               | 4        | 4                  |
| 2               | 5        | 5                  |
| 3               | 6        | 6                  |

> **Note:** `CMD_MAX_LEN` in the source is defined as 6, which is the maximum total command frame length.

### 3.2 Response Frame Layout

```
Byte offset   Field           Size      Description
-----------   -----           ----      -----------
0             FRAME_HEADER    1 byte    Always 0x24 ('$') (echoed back)
1             DATA_LEN        1 byte    Frame length value
2             STATUS          1 byte    0xAA = failure; any other value = success
3 ... 3+N-1   DATA            N bytes   Response payload (N = res_len requested)
3+N           CHECKSUM        1 byte    Sum of all preceding bytes modulo 0xFF
```

**Formula for full response length:**

```
full_res_len = 3 + res_len + 1
             = res_len + 4
```

Where `res_len` is the number of data payload bytes expected.

| res_len | Full response bytes |
|---------|---------------------|
| 2       | 6                   |
| 4       | 8                   |
| 10      | 14                  |

### 3.3 Frame Header Byte

| Symbol         | Hex    | ASCII |
|----------------|--------|-------|
| FRAME_HEADER   | `0x24` | `$`   |

---

## 4. Checksum Algorithm

The checksum is a simple **byte-sum modulo 0xFF** over all bytes that precede the checksum byte in the frame. It applies identically to both command and response frames.

### 4.1 Command Checksum

```
checksum = (FRAME_HEADER + DATA_LEN + cmd[0] + cmd[1] + ...) % 0xFF
```

Covering bytes at offsets 0 through `(full_cmd_len - 2)` inclusive (i.e., every byte except the checksum itself).

### 4.2 Response Checksum Verification

```
checksum = (response[0] + response[1] + ... + response[nbytes_read - 2]) % 0xFF
```

The receiver sums all received bytes except the last, computes `% 0xFF`, and compares against `response[nbytes_read - 1]`. A mismatch is a hard error; the driver logs "Serial read error: checksum mismatch" and returns false.

### 4.3 Status Byte Interpretation

After checksum verification the driver inspects `response[2]` (the STATUS byte):

| Value  | Meaning  |
|--------|----------|
| `0xAA` | Command **failed** — device rejected the command |
| Any other value | Command **succeeded** |

---

## 5. Command Set Reference

All single-byte command codes and their role:

| Cmd byte | Hex    | Direction | payload bytes (after cmd) | res_len | Purpose                         |
|----------|--------|-----------|---------------------------|---------|----------------------------------|
| `0x01`   | `0x01` | Write     | 2 (port index + value)    | 2       | Set GPIO / PWM / voltage channel |
| `0x02`   | `0x02` | Read      | —                         | 4       | Read INA219 power (mW × 100)    |
| `0x03`   | `0x03` | Read      | —                         | 4       | Read INA219 load voltage (V × 100) |
| `0x04`   | `0x04` | Read      | —                         | 4       | Read DS18B20 temperature (raw)  |
| `0x05`   | `0x05` | Read      | —                         | 4       | Read SHT40 temperature (raw)    |
| `0x06`   | `0x06` | Read      | —                         | 4       | Read SHT40 humidity (raw)       |
| `0x07`   | `0x07` | Read      | —                         | 4       | Read INA219 current (mA × 100)  |
| `0x08`   | `0x08` | Read      | —                         | 10      | Read full GPIO + PWM state      |
| `0xFF 0xFF` | —   | Write     | —                         | 2       | Power cycle: turn all off       |
| `0xFE 0xFE` | —   | Write     | —                         | 2       | Power cycle: turn all back on   |

> **Note on cmd `0x01` (`SET`):** The command byte is always `0x01`, but the full cmd payload is 3 bytes: `[0x01, port_index, value]`. See Section 7–10 for the exact `port_index` and `value` encoding per output type.

---

## 6. Response Parsing

### 6.1 Four-Byte Sensor Payload

Commands `0x02` through `0x07` all return a 4-byte big-endian unsigned integer payload. The driver decodes it as follows:

```cpp
uint32_t raw = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
double result = (double)raw / scale;   // scale = 100 for all sensor commands
```

The payload bytes are at response offsets `[3]`, `[4]`, `[5]`, `[6]` (i.e., after the 3-byte header).

### 6.2 Sensor Value Conversion Table

| Cmd  | Raw formula                               | Final unit   | Additional adjustment                              |
|------|-------------------------------------------|--------------|----------------------------------------------------|
| 0x02 | `raw / 100.0`                             | mW           | Divide by 1000 to display as W in INDI             |
| 0x03 | `raw / 100.0`                             | V            | Used directly as volts                             |
| 0x04 | `raw / 100.0`                             | —            | Subtract 255.5, round to 2 decimals → °C           |
| 0x05 | `raw / 100.0`                             | —            | Subtract 254.0, round to 1 decimal → °C            |
| 0x06 | `raw / 100.0`                             | —            | Subtract 254.0, round to 1 decimal → % RH          |
| 0x07 | `raw / 100.0`                             | mA           | Divide by 1000 to display as A in INDI             |

**DS18B20 temperature conversion (cmd 0x04):**
```
raw_value = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3])
scaled    = raw_value / 100.0
temp_C    = round((scaled - 255.5) * 100.0) / 100.0
```

**SHT40 temperature conversion (cmd 0x05):**
```
raw_value = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3])
scaled    = raw_value / 100.0
temp_C    = round((scaled - 254.0) * 10.0) / 10.0
```

**SHT40 humidity conversion (cmd 0x06):**
```
raw_value = (data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3])
scaled    = raw_value / 100.0
humidity  = round((scaled - 254.0) * 10.0) / 10.0     (units: % RH)
```

### 6.3 Ten-Byte Status Payload (cmd 0x08)

The 10 payload bytes (at response offsets `[3]` through `[12]`) map to hardware channels:

| Payload byte index | Offset in response | Channel            | Interpretation                                  |
|--------------------|--------------------|--------------------|--------------------------------------------------|
| `res[0]`           | `response[3]`      | DC port 1 (GPIO1)  | `0x00` = off, non-zero = on                     |
| `res[1]`           | `response[4]`      | DC port 2 (GPIO2)  | `0x00` = off, non-zero = on                     |
| `res[2]`           | `response[5]`      | DC port 3 (GPIO3)  | `0x00` = off, non-zero = on                     |
| `res[3]`           | `response[6]`      | DC port 4 (GPIO4)  | `0x00` = off, non-zero = on                     |
| `res[4]`           | `response[7]`      | DC port 5 (GPIO5)  | `0x00` = off, non-zero = on                     |
| `res[5]`           | `response[8]`      | USB group 0 (GPIO6)| `0x00` = off, non-zero = on                     |
| `res[6]`           | `response[9]`      | USB group 1 (GPIO7)| `0x00` = off, non-zero = on                     |
| `res[7]`           | `response[10]`     | Regulated voltage  | Raw 0–255, convert: `V = round(byte × 15.3 / 255.0, 1)` |
| `res[8]`           | `response[11]`     | PWM 1 (dew heater 1) | Raw 0–255, convert: `duty% = round(byte / 255.0 × 100)` |
| `res[9]`           | `response[12]`     | PWM 2 (dew heater 2) | Raw 0–255, convert: `duty% = round(byte / 255.0 × 100)` |

**Regulated voltage decode:**
```
voltage_V = round(res[7] * 15.3 / 255.0, 1 decimal place)
if voltage_V <= 0.0 → channel is OFF
else               → channel is ON at voltage_V volts
```

**Dew heater duty cycle decode:**
```
duty_pct = round(res[8 or 9] / 255.0 * 100)
if duty_pct == 0 → channel is OFF
else             → channel is ON at duty_pct %
```

---

## 7. Power Port Control

### 7.1 Command Structure

Command code: `0x01`
Total cmd payload: 3 bytes `[0x01, port_index, value]`
Expected `res_len`: 2

| Parameter    | Value                        |
|--------------|------------------------------|
| `cmd[0]`     | `0x01` (SET command)         |
| `cmd[1]`     | Port index — see table below |
| `cmd[2]`     | `0xFF` = on, `0x00` = off    |

### 7.2 DC Port Index Map

| INDI port number (0-based) | `cmd[1]` (port_index) | Label  |
|----------------------------|-----------------------|--------|
| 0                          | `0x00`                | DC 1   |
| 1                          | `0x01`                | DC 2   |
| 2                          | `0x02`                | DC 3   |
| 3                          | `0x03`                | DC 4   |
| 4                          | `0x04`                | DC 5   |

### 7.3 Full Command Frame — Turn DC Port ON

Example: turn DC 1 (port 0) on.

```
cmd payload:  [0x01, 0x00, 0xFF]   (3 bytes)
full_cmd_len: 2 + 3 + 1 = 6
DATA_LEN:     6

Frame bytes (before checksum):
  [0] 0x24  FRAME_HEADER
  [1] 0x06  DATA_LEN (= 6)
  [2] 0x01  cmd byte
  [3] 0x00  port_index (DC 1)
  [4] 0xFF  value (ON)
  [5] CS    checksum

Checksum = (0x24 + 0x06 + 0x01 + 0x00 + 0xFF) % 0xFF
         = (36 + 6 + 1 + 0 + 255) % 255
         = 298 % 255 = 43 = 0x2B

Full command frame: 24 06 01 00 FF 2B
```

### 7.4 Full Command Frame — Turn DC Port OFF

Example: turn DC 3 (port 2) off.

```
cmd payload:  [0x01, 0x02, 0x00]
Frame:        24 06 01 02 00 CS
Checksum = (0x24 + 0x06 + 0x01 + 0x02 + 0x00) % 0xFF
         = (36 + 6 + 1 + 2 + 0) % 255 = 45 % 255 = 45 = 0x2D
Full frame:   24 06 01 02 00 2D
```

---

## 8. USB Port Control

### 8.1 Command Structure

Identical structure to DC port control (cmd `0x01`), with different port indices.

| INDI USB port (0-based) | `cmd[1]` (port_index) | Label (physical ports)   |
|-------------------------|-----------------------|--------------------------|
| 0                       | `0x05`                | USB-C, USB 1, USB 2      |
| 1                       | `0x06`                | USB 3, USB 4, USB 5      |

```
cmd payload: [0x01, port_index, 0xFF/0x00]
```

> **Note:** USB port indices are at `port + 5` in the driver source (`GPIO5` = index 5, `GPIO6` = index 6). Each INDI USB "port" controls a physical group of ports, not an individual port.

### 8.2 Example — Enable USB group 0 (USB-C, 1, 2)

```
cmd payload:  [0x01, 0x05, 0xFF]
Frame bytes:
  [0] 0x24
  [1] 0x06
  [2] 0x01
  [3] 0x05
  [4] 0xFF
  [5] checksum
Checksum = (0x24 + 0x06 + 0x01 + 0x05 + 0xFF) % 0xFF
         = (36 + 6 + 1 + 5 + 255) % 255 = 303 % 255 = 48 = 0x30
Full frame: 24 06 01 05 FF 30
```

---

## 9. Dew Heater Control

### 9.1 Command Structure

Command code: `0x01`, port_index `0x08` or `0x09` (= driver port + 8).
The value byte encodes PWM duty cycle as a raw 8-bit integer.

| INDI dew port (0-based) | `cmd[1]` (port_index) | Label  |
|-------------------------|-----------------------|--------|
| 0                       | `0x08`                | PWM 1  |
| 1                       | `0x09`                | PWM 2  |

**Value encoding (duty cycle to raw byte):**
```
raw_value = round(255.0 * (duty_pct / 100.0))
cmd[2]    = raw_value    (when enabled)
cmd[2]    = 0x00         (when disabled / off)
```

Duty cycle range: 0–100% maps to raw 0x00–0xFF.

### 9.2 Duty Cycle Lookup (selected values)

| Duty (%) | `cmd[2]` (raw) | `cmd[2]` (hex) |
|----------|----------------|----------------|
| 0        | 0              | `0x00`         |
| 10       | 26             | `0x1A`         |
| 25       | 64             | `0x40`         |
| 50       | 128            | `0x80`         |
| 75       | 191            | `0xBF`         |
| 90       | 230            | `0xE6`         |
| 100      | 255            | `0xFF`         |

### 9.3 Example — Set PWM 1 to 50% duty cycle

```
cmd payload:  [0x01, 0x08, 0x80]
full_cmd_len: 6

Frame:
  [0] 0x24
  [1] 0x06
  [2] 0x01
  [3] 0x08
  [4] 0x80
  [5] checksum
Checksum = (0x24 + 0x06 + 0x01 + 0x08 + 0x80) % 0xFF
         = (36 + 6 + 1 + 8 + 128) % 255 = 179 % 255 = 179 = 0xB3
Full frame: 24 06 01 08 80 B3
```

### 9.4 Example — Turn PWM 2 off

```
cmd payload:  [0x01, 0x09, 0x00]
Checksum = (0x24 + 0x06 + 0x01 + 0x09 + 0x00) % 0xFF
         = (36 + 6 + 1 + 9 + 0) % 255 = 52 = 0x34
Full frame: 24 06 01 09 00 34
```

---

## 10. Variable Voltage Port Control

### 10.1 Command Structure

Command code: `0x01`, port_index `0x07` (hard-coded).

| Parameter | Value                            |
|-----------|----------------------------------|
| `cmd[0]`  | `0x01`                           |
| `cmd[1]`  | `0x07` (regulated output port)   |
| `cmd[2]`  | Raw 0–255 encoding voltage, or `0x00` when off |

**Voltage to raw byte encoding:**
```
raw = (voltage / 15.3) * 255.0
cmd[2] = clamp(round(raw), 0, 255)   (when enabled)
cmd[2] = 0x00                         (when disabled)
```

Voltage range: 0.0 V to 15.3 V (step 1 V as exposed in INDI, but raw encoding is continuous).

### 10.2 Voltage Lookup (selected values)

| Voltage (V) | `cmd[2]` (raw) | `cmd[2]` (hex) |
|-------------|----------------|----------------|
| 0.0         | 0              | `0x00`         |
| 3.0         | 50             | `0x32`         |
| 5.0         | 83             | `0x53`         |
| 6.0         | 100            | `0x64`         |
| 9.0         | 150            | `0x96`         |
| 12.0        | 200            | `0xC8`         |
| 15.0        | 250            | `0xFA`         |
| 15.3        | 255            | `0xFF`         |

### 10.3 Example — Set regulated output to 12 V

```
raw = (12.0 / 15.3) * 255.0 = 200.0 → 0xC8
cmd payload:  [0x01, 0x07, 0xC8]
Frame:
  [0] 0x24
  [1] 0x06
  [2] 0x01
  [3] 0x07
  [4] 0xC8
  [5] checksum
Checksum = (0x24 + 0x06 + 0x01 + 0x07 + 0xC8) % 0xFF
         = (36 + 6 + 1 + 7 + 200) % 255 = 250 % 255 = 250 = 0xFA
Full frame: 24 06 01 07 C8 FA
```

### 10.4 Example — Turn regulated output off

```
cmd payload:  [0x01, 0x07, 0x00]
Checksum = (0x24 + 0x06 + 0x01 + 0x07 + 0x00) % 0xFF
         = (36 + 6 + 1 + 7 + 0) % 255 = 50 = 0x32
Full frame: 24 06 01 07 00 32
```

---

## 11. Sensor Readings

All sensor read commands are 1-byte payloads and return 4 bytes of payload.

### 11.1 Read INA219 Power — cmd 0x02

```
cmd payload (1 byte): [0x02]
full_cmd_len = 4
Command frame: 24 04 02 CS
  Checksum = (0x24 + 0x04 + 0x02) % 0xFF = (36 + 4 + 2) % 255 = 42 = 0x2A
Full command: 24 04 02 2A

Response (8 bytes total, 4 data bytes):
  [0] 0x24  header
  [1] DATA_LEN
  [2] STATUS (non-0xAA = success)
  [3..6] 4-byte big-endian payload
  [7] checksum

Decode:
  raw = (res[0]<<24) | (res[1]<<16) | (res[2]<<8) | res[3]
  power_mW = raw / 100.0
  power_W  = power_mW / 1000.0   (as displayed in INDI SENSOR_POWER)
```

### 11.2 Read INA219 Load Voltage — cmd 0x03

```
Command frame: 24 04 03 CS
  Checksum = (0x24 + 0x04 + 0x03) % 0xFF = 43 = 0x2B
Full command: 24 04 03 2B

Decode:
  raw = big-endian uint32 from res[0..3]
  voltage_V = raw / 100.0   (displayed directly as V in INDI SENSOR_VOLTAGE)
```

### 11.3 Read DS18B20 Temperature — cmd 0x04

```
Command frame: 24 04 04 CS
  Checksum = (0x24 + 0x04 + 0x04) % 0xFF = 44 = 0x2C
Full command: 24 04 04 2C

Decode:
  raw    = big-endian uint32 from res[0..3]
  scaled = raw / 100.0
  temp_C = round((scaled - 255.5) * 100.0) / 100.0
```

The constant `255.5` is a firmware-side offset; the physical sensor range after adjustment is approximately -100 °C to +200 °C.

### 11.4 Read SHT40 Temperature — cmd 0x05

```
Command frame: 24 04 05 CS
  Checksum = (0x24 + 0x04 + 0x05) % 0xFF = 45 = 0x2D
Full command: 24 04 05 2D

Decode:
  raw    = big-endian uint32 from res[0..3]
  scaled = raw / 100.0
  temp_C = round((scaled - 254.0) * 10.0) / 10.0
```

The constant `254.0` is a firmware-side offset applied to the SHT40 reading.

### 11.5 Read SHT40 Humidity — cmd 0x06

```
Command frame: 24 04 06 CS
  Checksum = (0x24 + 0x04 + 0x06) % 0xFF = 46 = 0x2E
Full command: 24 04 06 2E

Decode:
  raw       = big-endian uint32 from res[0..3]
  scaled    = raw / 100.0
  humidity  = round((scaled - 254.0) * 10.0) / 10.0   (units: % RH, range 0–100)
```

### 11.6 Read INA219 Current — cmd 0x07

```
Command frame: 24 04 07 CS
  Checksum = (0x24 + 0x04 + 0x07) % 0xFF = 47 = 0x2F
Full command: 24 04 07 2F

Decode:
  raw       = big-endian uint32 from res[0..3]
  current_mA = raw / 100.0
  current_A  = current_mA / 1000.0   (as displayed in INDI SENSOR_CURRENT)
```

---

## 12. Status / State Polling

### 12.1 Get Full Device State — cmd 0x08

This single command retrieves the on/off state of all DC ports, USB groups, the regulated voltage level, and both PWM dew heater levels simultaneously.

```
cmd payload (1 byte): [0x08]
full_cmd_len = 4
res_len = 10
full_res_len = 14

Command frame: 24 04 08 CS
  Checksum = (0x24 + 0x04 + 0x08) % 0xFF = (36 + 4 + 8) % 255 = 48 = 0x30
Full command: 24 04 08 30

Response: 14 bytes total
  [0]  0x24      header
  [1]  DATA_LEN
  [2]  STATUS    (non-0xAA = success)
  [3]  GPIO1     DC port 1 state:  0=off, nonzero=on
  [4]  GPIO2     DC port 2 state
  [5]  GPIO3     DC port 3 state
  [6]  GPIO4     DC port 4 state
  [7]  GPIO5     DC port 5 state
  [8]  GPIO6     USB group 0 state (USB-C, USB1, USB2)
  [9]  GPIO7     USB group 1 state (USB3, USB4, USB5)
  [10] pwmA raw  Regulated voltage raw byte (0–255)
  [11] pwmB raw  PWM 1 dew heater raw byte (0–255)
  [12] pwmC raw  PWM 2 dew heater raw byte (0–255)
  [13] CHECKSUM
```

> **Naming note:** The debug log in the source code names the fields as `GPIO1..GPIO7` and `pwmA`, `pwmB`, `pwmC`. In this response, `pwmA` (response byte 10) corresponds to the regulated voltage channel, while `pwmB` and `pwmC` (bytes 11–12) correspond to dew heater channels 1 and 2 respectively. This naming may seem counterintuitive but matches the hardware PWM outputs on the ESP32.

### 12.2 Polling Interval

The INDI driver polls via `TimerHit()` every **100 ms** (SetTimer(100)). Each `Get_State()` call issues **7 sequential commands** (0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08). With each command taking up to ~200 ms worst-case (100 ms post-write sleep + 500 ms read timeout), the practical poll cycle is on the order of 1–2 seconds.

> **TheSkyX implementation note:** A 1–2 second polling interval is safe. Polling more frequently than once per second is unlikely to be beneficial and may cause command overlap or buffer stalls.

---

## 13. Initialization Sequence

### 13.1 Step 1 — Open Serial Port

Open the serial port at 115200 8N1 with no hardware flow control.

### 13.2 Step 2 — Assert RTS/DTR Low (ESP32 Reset)

Immediately after opening:

```c
ioctl(PortFD, TIOCMGET, &flags);
flags &= ~(TIOCM_RTS | TIOCM_DTR);   // Clear both RTS and DTR
ioctl(PortFD, TIOCMSET, &flags);
```

Clearing both RTS and DTR triggers the ESP32 hardware reset via the standard Arduino/ESP32 auto-reset circuit.

### 13.3 Step 3 — Wait for Boot Log to Drain

The ESP32 emits a boot log over serial before the firmware is ready. The driver reads it line by line (using `tty_nread_section_expanded` with newline `'\n'` as the section terminator) and waits until none of the following substrings appear in a received line:

| Substring | Present in boot log indicating... |
|-----------|-----------------------------------|
| `\n`      | Any line (line still arriving)    |
| `POW`     | `POWERON_RESET` message           |
| `0x00`    | Clock driver / config lines       |
| `rst`     | Reset cause line                  |
| `loa`     | `load:` memory load lines         |
| `len`     | `len:` memory length lines        |
| `ts`      | `ets` bootloader line             |

Parameters: read timeout = 500 ms (`TIMEOUT_SEC=0`, `TIMEOUT_MSEC=500`), with 50 ms sleep before each read attempt. Maximum 10 retry iterations. If the boot log has not cleared within 10 × (50 ms + 50 ms + 500 ms) ≈ 6 seconds, the handshake fails.

After the loop exits, both input and output serial buffers are flushed with `tcflush(PortFD, TCIOFLUSH)`.

**Expected boot log (for reference):**
```
ets Jun  8 2016 00:22:57\r\n
\r\n
rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\r\n
configsip: 188777542, SPIWP:0xee\r\n
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00\r\n
mode:DIO, clock div:1\r\n
load:0x3fff0030,len:1344\r\n
load:0x40078000,len:13964\r\n
load:0x40080400,len:3600\r\n
entry 0x400805f0\r\n
```

### 13.4 Step 4 — Verify with State Query

Send command `0x08` (get full state) as a handshake probe:

```
Full command: 24 04 08 30
Expected response: 14 bytes, STATUS ≠ 0xAA
```

If this command succeeds (returns true), the handshake is complete. If it fails, log "Handshake failed." and return false.

### 13.5 Step 5 — Post-Handshake Configuration

After a successful handshake the driver:

1. Sets the variable voltage channel range to 0.0–15.3 V, step 1 V.
2. Relabels DC port properties as "DC 1" through "DC 5".
3. Relabels dew heater properties as "PWM 1" and "PWM 2".
4. Relabels USB port properties as "USB C,1,2" and "USB 3,4,5".
5. Relabels variable channel property as "REGULATED".

No commands are sent to the device during this labeling step — it is purely an INDI UI operation.

---

## 14. Error Handling and Timeouts

### 14.1 Read Timeout

| Parameter       | Value   |
|-----------------|---------|
| `TIMEOUT_SEC`   | 0       |
| `TIMEOUT_MSEC`  | 500     |

The driver waits up to **500 ms** for a response after writing a command.

### 14.2 Inter-Command Timing

The driver inserts deliberate sleeps around each command:

| Event                            | Sleep duration |
|----------------------------------|----------------|
| After `tcdrain()` (post-write)   | 100 ms         |
| After `tcflush()` (post-read)    | 100 ms         |
| Between boot-log read attempts   | 50 ms (before) + 50 ms (after read) |
| After power-cycle OFF command    | 1000 ms        |

### 14.3 Possible Failure Conditions and Behavior

| Condition                              | Driver response                                      |
|----------------------------------------|------------------------------------------------------|
| `tty_write()` returns non-TTY_OK       | Log "Serial write error: <msg>", return false        |
| Bytes written ≠ expected               | Log "Serial write error: expected N, wrote M", return false |
| `tty_read_expanded()` returns non-TTY_OK | Log "Serial read error: <msg>", return false       |
| Bytes read ≠ `full_res_len`            | Log "Serial read error: expected N, got M", return false |
| Checksum mismatch                      | Log "Serial read error: checksum mismatch", return false |
| STATUS byte = `0xAA`                   | Return false (command rejected by device)            |
| Boot log doesn't clear within 10 retries | Handshake fails                                   |

> **TheSkyX implementation note:** All errors in `sendCommand` return false. The calling layer (`Get_State`, `SetPowerPort`, etc.) checks this return value but simply logs and continues polling on the next timer tick. There is no retry logic within a single command invocation; the next poll cycle will try again.

### 14.4 Buffer Management

Before every command write, the driver calls `tcflush(PortFD, TCIOFLUSH)` to discard any stale bytes in both the input and output kernel buffers. A second `tcflush` is called after reading the response. This prevents accumulated junk from corrupting subsequent command/response sequences.

---

## 15. Power Cycle Sequence

The power cycle is a two-step procedure using special 2-byte command payloads.

### 15.1 Step 1 — All Outputs Off

```
cmd payload: [0xFF, 0xFF]   (2 bytes)
full_cmd_len = 2 + 2 + 1 = 5
DATA_LEN = 5

Frame:
  [0] 0x24
  [1] 0x05
  [2] 0xFF
  [3] 0xFF
  [4] checksum
Checksum = (0x24 + 0x05 + 0xFF + 0xFF) % 0xFF
         = (36 + 5 + 255 + 255) % 255 = 551 % 255 = 41 = 0x29
Full frame: 24 05 FF FF 29

res_len: 2  →  full_res_len: 6
```

### 15.2 Wait 1000 ms

The driver sleeps for 1 second between the two power-cycle commands.

### 15.3 Step 2 — All Outputs Back On

```
cmd payload: [0xFE, 0xFE]   (2 bytes)
full_cmd_len = 5
DATA_LEN = 5

Frame:
  [0] 0x24
  [1] 0x05
  [2] 0xFE
  [3] 0xFE
  [4] checksum
Checksum = (0x24 + 0x05 + 0xFE + 0xFE) % 0xFF
         = (36 + 5 + 254 + 254) % 255 = 549 % 255 = 39 = 0x27
Full frame: 24 05 FE FE 27

res_len: 2  →  full_res_len: 6
```

Both steps expect a 2-byte data payload in the response and check STATUS ≠ 0xAA. If either step fails, `CyclePower()` returns false immediately (the second step is skipped if the first fails).

---

## 16. Dew Point Calculation

The dew point is calculated entirely in software from the SHT40 temperature and humidity readings. The firmware does not provide it directly.

### 16.1 Saturation Vapor Pressure (SVP)

```
SVP(T) = 6.11 × 10^(7.5 × T / (237.7 + T))     [hPa]
```

where T is temperature in °C.

### 16.2 Actual Vapor Pressure (VP)

```
VP = RH × SVP / 100     [hPa]
```

where RH is relative humidity in %.

### 16.3 Dew Point Search

The dew point is found by iterating T from -100.0 °C to +100.0 °C in steps of 0.01 °C, computing `SVP(T)` at each step, and selecting the temperature at which `|VP - SVP(T)|` is minimized. The result is rounded to 2 decimal places.

> **TheSkyX implementation note:** This brute-force search iterates 20,001 steps per call. At ~1–2 second poll intervals the CPU cost is negligible on modern hardware. A closed-form Magnus formula inverse could be substituted for efficiency:
>
> ```
> dewpoint = 237.7 × log10(VP/6.11) / (7.5 - log10(VP/6.11))
> ```
>
> Both approaches produce equivalent results within floating-point precision.

---

## 17. Worked Examples

### 17.1 Turn DC Port 2 ON

```
Goal: enable DC port 2 (INDI 0-based index = 1)

cmd payload:  [0x01, 0x01, 0xFF]
full_cmd_len: 6

Byte-by-byte frame construction:
  [0] 0x24  (frame header)
  [1] 0x06  (DATA_LEN = 6)
  [2] 0x01  (SET command)
  [3] 0x01  (port_index = DC port 2)
  [4] 0xFF  (value = ON)

Checksum = (0x24 + 0x06 + 0x01 + 0x01 + 0xFF) % 0xFF
         = (36 + 6 + 1 + 1 + 255) % 255
         = 299 % 255 = 44 = 0x2C
  [5] 0x2C

TX: 24 06 01 01 FF 2C

Expected response (6 bytes, res_len=2):
  [0] 0x24  (header)
  [1] DATA_LEN
  [2] STATUS (not 0xAA)
  [3] data byte 0
  [4] data byte 1
  [5] checksum
```

### 17.2 Read INA219 Power

```
Goal: read bus power in watts

cmd payload: [0x02]
full_cmd_len: 4

Frame:
  [0] 0x24
  [1] 0x04  (DATA_LEN = 4)
  [2] 0x02  (READ POWER command)
  [3] checksum = (0x24 + 0x04 + 0x02) % 0xFF = 42 = 0x2A

TX: 24 04 02 2A

Expected response (8 bytes, res_len=4):
  [0] 0x24
  [1] DATA_LEN
  [2] STATUS (not 0xAA)
  [3] MSB of uint32
  [4]
  [5]
  [6] LSB of uint32
  [7] checksum

Example: response data bytes [3..6] = 0x00 0x00 0x27 0x10
  raw = 0x00002710 = 10000
  power_mW = 10000 / 100.0 = 100.0 mW
  power_W  = 100.0 / 1000.0 = 0.1 W
```

### 17.3 Read Full Device State (cmd 0x08)

```
TX: 24 04 08 30

Expected response (14 bytes):
  [0]  0x24  header
  [1]  0x0E  DATA_LEN (14)
  [2]  0x00  STATUS = success
  [3]  0x01  DC port 1 = ON
  [4]  0x01  DC port 2 = ON
  [5]  0x00  DC port 3 = OFF
  [6]  0x00  DC port 4 = OFF
  [7]  0x01  DC port 5 = ON
  [8]  0x01  USB group 0 = ON (USB-C, 1, 2)
  [9]  0x00  USB group 1 = OFF (USB 3, 4, 5)
  [10] 0x64  Regulated voltage raw = 100  → V = round(100 × 15.3/255, 1) = 6.0 V
  [11] 0x80  PWM 1 raw = 128  → duty = round(128/255 × 100) = 50%
  [12] 0x00  PWM 2 raw = 0    → channel OFF
  [13] CS    checksum

Decoded state:
  DC 1: ON    DC 2: ON    DC 3: OFF   DC 4: OFF   DC 5: ON
  USB C/1/2: ON    USB 3/4/5: OFF
  Regulated output: 6.0 V (ON)
  PWM 1 (Dew 1): 50% duty (ON)
  PWM 2 (Dew 2): OFF
```

### 17.4 Set Dew Heater 2 to 75%

```
Goal: PWM 2 (INDI index 1) to 75% duty cycle

raw_value = round(255.0 * (75.0 / 100.0)) = round(191.25) = 191 = 0xBF
cmd payload: [0x01, 0x09, 0xBF]
full_cmd_len: 6

Frame:
  [0] 0x24
  [1] 0x06
  [2] 0x01
  [3] 0x09
  [4] 0xBF
  [5] checksum = (0x24 + 0x06 + 0x01 + 0x09 + 0xBF) % 0xFF
               = (36 + 6 + 1 + 9 + 191) % 255 = 243 % 255 = 243 = 0xF3

TX: 24 06 01 09 BF F3

Expected response (6 bytes): STATUS ≠ 0xAA
```

### 17.5 Set Regulated Output to 9 V

```
raw = (9.0 / 15.3) * 255.0 = 150.0 → 0x96
cmd payload: [0x01, 0x07, 0x96]

Frame:
  [0] 0x24
  [1] 0x06
  [2] 0x01
  [3] 0x07
  [4] 0x96
  [5] checksum = (0x24 + 0x06 + 0x01 + 0x07 + 0x96) % 0xFF
               = (36 + 6 + 1 + 7 + 150) % 255 = 200 % 255 = 200 = 0xC8

TX: 24 06 01 07 96 C8
```

### 17.6 Complete Power Cycle

```
Step 1 — All OFF:
TX: 24 05 FF FF 29
Wait 1000 ms for hardware to cycle.

Step 2 — All ON:
TX: 24 05 FE FE 27
```

### 17.7 Read DS18B20 Lens Temperature

```
TX: 24 04 04 2C

Example response data bytes [3..6] = 0x00 0x00 0x63 0xBE
  raw    = 0x000063BE = 25534
  scaled = 25534 / 100.0 = 255.34
  temp_C = round((255.34 - 255.5) × 100.0) / 100.0
         = round(-16.0) / 100.0 = -0.16 °C
```

---

## 18. Quick Reference Tables

### 18.1 All Port Indices for cmd 0x01 (SET)

| Port type        | INDI port index | `cmd[1]` byte | Label           |
|------------------|-----------------|---------------|-----------------|
| DC port 1        | 0               | `0x00`        | DC 1            |
| DC port 2        | 1               | `0x01`        | DC 2            |
| DC port 3        | 2               | `0x02`        | DC 3            |
| DC port 4        | 3               | `0x03`        | DC 4            |
| DC port 5        | 4               | `0x04`        | DC 5            |
| USB group 0      | 0               | `0x05`        | USB-C, USB 1, 2 |
| USB group 1      | 1               | `0x06`        | USB 3, 4, 5     |
| Regulated output | 0               | `0x07`        | REGULATED       |
| Dew heater 1     | 0               | `0x08`        | PWM 1           |
| Dew heater 2     | 1               | `0x09`        | PWM 2           |

### 18.2 All Read Commands

| Command | `cmd[1]` byte | res_len | Total TX bytes | Total RX bytes | Output           |
|---------|---------------|---------|----------------|----------------|------------------|
| `0x02`  | —             | 4       | 4              | 8              | Power (mW)       |
| `0x03`  | —             | 4       | 4              | 8              | Voltage (V)      |
| `0x04`  | —             | 4       | 4              | 8              | DS18B20 temp     |
| `0x05`  | —             | 4       | 4              | 8              | SHT40 temp       |
| `0x06`  | —             | 4       | 4              | 8              | SHT40 humidity   |
| `0x07`  | —             | 4       | 4              | 8              | Current (mA)     |
| `0x08`  | —             | 10      | 4              | 14             | Full state       |

### 18.3 Checksum Reference for All Read Commands

| Command | TX bytes           | Checksum |
|---------|--------------------|----------|
| `0x02`  | `24 04 02 [CS]`    | `0x2A`   |
| `0x03`  | `24 04 03 [CS]`    | `0x2B`   |
| `0x04`  | `24 04 04 [CS]`    | `0x2C`   |
| `0x05`  | `24 04 05 [CS]`    | `0x2D`   |
| `0x06`  | `24 04 06 [CS]`    | `0x2E`   |
| `0x07`  | `24 04 07 [CS]`    | `0x2F`   |
| `0x08`  | `24 04 08 [CS]`    | `0x30`   |

### 18.4 Value Encoding Summary

| Output type      | ON value                                   | OFF value |
|------------------|--------------------------------------------|-----------|
| DC port          | `0xFF`                                     | `0x00`    |
| USB group        | `0xFF`                                     | `0x00`    |
| Regulated output | `round((V / 15.3) × 255)`, clamped 0–255  | `0x00`    |
| Dew heater PWM   | `round((duty% / 100.0) × 255)`            | `0x00`    |

### 18.5 Sensor Decode Summary

| cmd  | INDI property            | Decode formula                                   | Unit  |
|------|--------------------------|--------------------------------------------------|-------|
| 0x02 | SENSOR_POWER             | `(raw/100.0) / 1000.0`                           | W     |
| 0x03 | SENSOR_VOLTAGE           | `raw / 100.0`                                    | V     |
| 0x04 | DS18B20_TEMP             | `round((raw/100.0 - 255.5) × 100) / 100`        | °C    |
| 0x05 | SHT40_TEMP               | `round((raw/100.0 - 254.0) × 10) / 10`          | °C    |
| 0x06 | SHT40_HUMI               | `round((raw/100.0 - 254.0) × 10) / 10`          | % RH  |
| 0x07 | SENSOR_CURRENT           | `(raw/100.0) / 1000.0`                           | A     |

Where `raw = big-endian uint32 from 4 payload bytes`.

---

*End of PROTOCOL.md*
