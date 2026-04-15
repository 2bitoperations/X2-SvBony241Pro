# X2 Power Control Driver — SVBony SV241 Pro USB Power Hub

TheSkyX Professional Edition plugin that exposes the SVBony SV241 Pro as a
**Power Control Box** device.

---

## What This Driver Does

- Switches 4 × 12V DC outputs, USB hub power, and a 0–15.3V adjustable output on/off
- Controls 2 × dew heater ports via PWM (0–100% duty cycle)
- Auto dew-point tracking: reads DS18B20 (ambient) + SHT40 (humidity), computes
  dew point via the Magnus formula, and drives heater PWM automatically
- Monitors INA219 (bus voltage, current, power), DS18B20, and SHT40 sensors
- Optionally restores all circuit states at connect
- Communicates over USB virtual serial port at 115200 baud 8N1

---

## Circuit Layout

| Circuit | Label             | Type                            |
|---------|-------------------|---------------------------------|
| 0–3     | DC Port 1–4 (12V) | Switchable 12V DC               |
| 4–5     | Dew Heater 1–2    | PWM-controlled, manual or auto  |
| 6       | USB Hub Power     | Switchable (USB-C + USB1 + USB2)|
| 7       | Adjustable Output | Regulated 0–15.3V               |

---

## Build & Install

**TL;DR:** `make clean install`

| Target           | What it does                                                |
|------------------|-------------------------------------------------------------|
| `make`           | Build the shared library (validates UI file first)          |
| `make install`   | Build + copy library, UI file, and hardware list to TheSkyX|
| `make clean`     | Remove build artifacts                                      |
| `make udev-install` | Install udev rule for stable device name (Linux, needs sudo)|

Prerequisites: GCC or Clang with C++11 support. No other dependencies.

Restart TheSkyX after `make install`, then select **SVBony SV241 Pro** from
**Telescope > Power Control Box**.

---

## Stable Device Name (Linux)

Run `sudo make udev-install` to install a udev rule that creates the stable
symlink `/dev/ttyUSBSV241Pro` → whichever `/dev/ttyUSBx` the device lands on.
Replug the cable after installing. Use `/dev/ttyUSBSV241Pro` as the port in TheSkyX.

> If you have multiple CH340/CH341 devices, you may need to pin the rule to a
> specific USB port path (`KERNELS=="N-N.N"`) — run
> `udevadm info -a -n /dev/ttyUSBSV241Pro` to find it.

---

## Troubleshooting / Log File

Errors are always written to `~/TheSkyX/x2svbony241pro.log` regardless of the
in-app debug level setting. `tail -f` that file while connecting if things go
wrong. Verbosity (Off / Errors / Commands / Full I/O) is set in the driver's
settings dialog.

---

## Protocol

See **[PROTOCOL.md](PROTOCOL.md)**.

---

## Attribution & License

The protocol documentation in `PROTOCOL.md` was derived from the INDI driver for
the SVBony SV241 Pro, authored by **Tetsuya Kakura** and published under the
**GNU General Public License v2.0 or later**
([indi-lib/indi](https://github.com/indilib/indi),
`drivers/power/svbony_powerbox.cpp`).

Because this project's documentation and implementation are derived from
GPL-licensed work, this project is released under the same terms:

**GNU General Public License v2.0 or later (GPL-2.0+)**

See [https://www.gnu.org/licenses/gpl-2.0.html](https://www.gnu.org/licenses/gpl-2.0.html).
Contributions welcome via pull request.
