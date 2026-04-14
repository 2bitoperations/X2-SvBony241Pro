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
| `Makefile`                    | Build and install script                             |
| `powercontrollist Svbony.txt` | TheSkyX hardware registration file                  |

---

## Build Instructions

### Prerequisites

- A C++11-capable compiler (GCC or Clang)
- The TheSkyX X2 SDK (licensed interfaces) checked out at `../X2-Examples/`
  relative to this directory.  The canonical location is:
  `https://github.com/theskyxdeveloper/X2-Examples`

### Steps

```bash
# From the repository root
make clean && make install
```

`make install` builds the library and copies it (along with the UI file and
hardware registration text) into your local TheSkyX installation.  Restart
TheSkyX after installing.

The output library is placed in the repo root as:

| Platform | File                      |
|----------|---------------------------|
| Linux    | `libx2svbony241pro.so`    |
| macOS    | `libx2svbony241pro.dylib` |

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

## Stable Device Name

### The problem

On Linux, USB-serial adapters are assigned a kernel node such as `/dev/ttyUSB0`
or `/dev/ttyUSB1` based on the order they enumerate at boot time.  If you have
more than one USB-serial device (very common in astronomy setups — focuser
controllers, filter wheels, etc.) the SV241 Pro may appear as a different
`/dev/ttyUSBx` path every time you reboot or replug.  This breaks TheSkyX's
saved serial-port setting.

### The solution — install a udev rule

The repository ships with a udev rule file `99-sv241pro.rules` that matches the
SV241 Pro by its USB vendor/product ID (QinHeng CH340, VID `1a86` / PID `7523`)
and creates a permanent symlink `/dev/ttyUSBSV241Pro` pointing to whichever
`/dev/ttyUSBx` the device happens to occupy.

Install the rule with:

```bash
sudo make udev-install
```

This copies `99-sv241pro.rules` to `/etc/udev/rules.d/`, reloads the udev rule
database, and triggers udev so existing devices are re-evaluated.

After installation, **replug the SV241 Pro USB cable**.  The symlink
`/dev/ttyUSBSV241Pro` will appear immediately and will persist across reboots.

In TheSkyX, open **Telescope > Power Control Box**, select **SVBony SV241 Pro**,
and set the serial port to:

```
/dev/ttyUSBSV241Pro
```

### Multiple CH340/CH341 devices

The QinHeng CH340/CH341 chip is used by many inexpensive USB-serial adapters.
If you have more than one such device connected simultaneously, the VID+PID
rule above will match all of them and the symlink will point to whichever one
udev processes last — which is non-deterministic.

To pin the rule to the SV241 Pro specifically, add a serial-number attribute.
Find the serial number with:

```bash
udevadm info -a -n /dev/ttyUSBSV241Pro | grep '{serial}'
```

Then edit `/etc/udev/rules.d/99-sv241pro.rules` and add the serial attribute:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
    ATTRS{serial}=="YOUR_SERIAL_HERE", SYMLINK+="ttyUSBSV241Pro"
```

Note: the cheaper CH340 variant (as opposed to CH341) often has no unique serial
number programmed at the factory.  In that case you may need to use
`KERNELS=="N-N.N"` (the USB port path) to distinguish devices by physical port.
Run `udevadm info -a -n /dev/ttyUSBx` and look for the `KERNELS` line to find
the stable port path for your hardware.

---

## Troubleshooting / Log File

The driver writes a persistent plain-text log alongside the in-app
TheSkyX Communication Log window.

**Default log path:**

| Platform | Path |
|----------|------|
| Linux / macOS | `~/TheSkyX/x2svbony241pro.log` |
| Fallback (if `$HOME` is not set) | `/tmp/x2svbony241pro.log` |
| Windows | `%USERPROFILE%\TheSkyX\x2svbony241pro.log` |

Each line is prefixed with a local timestamp in `YYYY-MM-DD HH:MM:SS` format.

**What gets logged where:**

| Debug Level setting | TheSkyX window | Log file |
|---------------------|----------------|----------|
| 0 — Off | nothing | errors only (level 1) |
| 1 — Errors | errors | errors |
| 2 — Commands | errors + commands | errors + commands |
| 3 — Full I/O | everything | everything |

Level-1 (Error) messages — connection failures, protocol errors, timeouts —
are **always** written to the log file regardless of the debug-level setting.
This means you get a persistent record of connection problems even when the
debug level is set to Off.

To read the log on Linux/macOS, open a terminal and run:

```bash
tail -f ~/TheSkyX/x2svbony241pro.log
```

The log is opened in append mode for every message, so it survives TheSkyX
restarts and accumulates across sessions. Truncate or delete it at any time
to start fresh; it will be recreated automatically on the next log event.

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
