# X2 Power Control Driver — SVBony SV241 Pro USB Power Hub

This repository contains a TheSkyX Professional Edition X2 plugin that exposes
the SVBony SV241 Pro USB Power Hub as a **Power Control Box** device within
TheSkyX.

---

## What This Driver Does

The driver implements the TheSkyX `PowerControlDriverInterface` X2 API and
allows TheSkyX to:

- Switch individual DC power output ports on or off.
- Control dew heater ports (on/off state exposed; PWM duty-cycle control
  requires protocol details — see the **TODO** section below).
- Toggle USB hub power.
- Toggle the adjustable voltage output.

The driver communicates with the SV241 Pro over its USB virtual serial port
(CDC/ACM) at 115200 baud.

---

## Circuit Layout

| Circuit Index | Label              | Type                  |
|---------------|--------------------|-----------------------|
| 0             | DC Port 1 (12V)    | Switchable 12V DC     |
| 1             | DC Port 2 (12V)    | Switchable 12V DC     |
| 2             | DC Port 3 (12V)    | Switchable 12V DC     |
| 3             | DC Port 4 (12V)    | Switchable 12V DC     |
| 4             | Dew Heater 1       | PWM-controlled output |
| 5             | Dew Heater 2       | PWM-controlled output |
| 6             | USB Hub Power      | Switchable USB power  |
| 7             | Adjustable Output  | Variable voltage out  |

---

## Source File Overview

| File                          | Purpose                                              |
|-------------------------------|------------------------------------------------------|
| `main.h`                      | DLL entry-point declarations                         |
| `main.cpp`                    | `sbPlugInName2` and `sbPlugInFactory2` implementations |
| `x2svbony241pro.h`            | Driver class declaration                             |
| `x2svbony241pro.cpp`          | Driver class implementation                          |
| `CMakeLists.txt`              | CMake build script                                   |
| `powercontrollist Svbony.txt` | TheSkyX hardware registration file                  |

---

## Build Instructions

### Prerequisites

- CMake 3.10 or later
- A C++11-capable compiler (GCC, Clang, MSVC)
- The TheSkyX X2 SDK (licensed interfaces) checked out at `../X2-Examples/`
  relative to this directory.  The canonical location is:
  `https://github.com/theskyxdeveloper/X2-Examples`

### Steps

```bash
# From the repository root
mkdir build
cd build
cmake ..
cmake --build .
```

The output library will be placed in `build/` as:

| Platform | File                      |
|----------|---------------------------|
| Linux    | `libx2svbony241pro.so`    |
| macOS    | `libx2svbony241pro.dylib` |
| Windows  | `x2svbony241pro.dll`      |

---

## Installation Instructions

### 1. Copy the shared library

Copy the compiled library to the TheSkyX Power Control Box plugin directory:

**macOS:**
```
/Applications/TheSkyX Professional Edition.app/Contents/Resources/Common/Plugins/PowerControlBox/
```

**Windows:**
```
C:\Program Files\Software Bisque\TheSkyX Professional Edition\Plugins\PowerControlBox\
```

**Linux:**
```
~/TheSkyXLinux/Plugins/PowerControlBox/
```

### 2. Copy the hardware list file

Copy `powercontrollist Svbony.txt` to the same **PowerControlBox** directory,
or append its single line to an existing `powercontrollist.txt` in that
directory, matching the format used by the other entries.

### 3. Restart TheSkyX

After restarting, the **SVBony SV241 Pro** entry will appear in the
**Telescope > Power Control Box** device selector.

---

## Protocol

The USB serial protocol for the SV241 Pro has been documented from the INDI
driver source. See **[PROTOCOL.md](PROTOCOL.md)** for the full specification,
including frame structure, all command/response pairs, sensor decoding, and
the ESP32 boot sequence that must be handled on connect.

Key facts:
- 115200 baud, 8N1, no flow control
- Binary framing: `0x24` header, length byte, payload, checksum
- ESP32 resets on RTS/DTR clear — boot log must be drained before binary commands

---

## Attribution

The protocol documentation in `PROTOCOL.md` was derived from the INDI driver
for the SVBony SV241 Pro, authored by **Tetsuya Kakura** and published under
the **GNU General Public License v2.0 or later**.

- **INDI Project:** https://github.com/indilib/indi
- **Driver source:** `drivers/power/svbony_powerbox.cpp` / `svbony_powerbox.h`
- **License:** GPL-2.0+

This X2 driver is an independent implementation and does not incorporate any
GPL-licensed code; it uses only the protocol specification derived from study
of the INDI driver.

---

## License

This driver scaffold is provided as-is for the astronomy community.
No warranty is expressed or implied.  Contributions welcome via pull request.
