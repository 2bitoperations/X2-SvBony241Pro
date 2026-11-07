# Agent Guide

This is a TheSkyX X2 Power Control Box plugin for the SVBony SV241 Pro USB
Power Hub. See [README.md](README.md) for user-facing docs,
[PROTOCOL.md](PROTOCOL.md) for the serial protocol.

## Build & install

**Always `make clean && make install`** — never cmake. Build must produce zero
warnings. The `UI OK.` line from the install script confirms the .ui file
passed XML validation.

## Architecture

Single class `X2Svbony241Pro` in `x2svbony241pro.h/.cpp` implements:

| Interface | Purpose |
|---|---|
| `PowerControlDriverInterface` | Core API: `numberOfCircuits`, `circuitState`, `setCircuitState` |
| `ModalSettingsDialogInterface` | Settings dialog: `execModalSettingsDialog` |
| `X2GUIEventInterface` | UI callbacks: `uiEvent` |
| `CircuitLabelsInterface` | Per-circuit label strings |
| `SerialPortParams2Interface` | Serial port selection |

All binary I/O: `buildFrame` → `sendFrame` → `readFrame`. All device commands:
`cmd*` methods. Never call `SerXInterface` directly outside those helpers.

Circuit ↔ device port mapping: `kPortIndex[]` table; see header comment block.
Device ports 0x04 (DC5) and 0x06 (USB group 1) exist but are not exposed as X2 circuits.

Settings persistence: `BasicIniUtilInterface`, section `"SV241Pro"`, keys
scoped by `m_nInstanceIndex`. `saveDewConfig`/`loadDewConfig` for dew settings;
`saveCircuitStates`/`restoreCircuitStates` for on/off state.

## Code rules

- **C++11** only (`-std=gnu++11`). No C++14/17. No exceptions. No RTTI except
  `dynamic_cast` in `queryAbstraction`.
- Error returns: `SB_OK` / `ERR_*` from `sberrorx.h`. Never throw.
- Logging: `logDebug(minLevel, fmt, ...)`. Level 1=Errors always go to file.
- Do not add new base classes without updating `queryAbstraction`.
- `licensedinterfaces/` is vendored (Software Bisque copyright). The
  `X2_FLAT_INCLUDES` patch in `x2guiinterface.h` is load-bearing — do not
  upgrade headers without re-verifying it.

## UI file rules

Widget naming: `lbl` `spin` `dblSpin` `combo` `btn` `grp` `progress` `chk`
prefixes; layouts/spacers use plain descriptive names.

**`pushButtonOK` and `pushButtonCancel` must keep those exact names.** The X2
framework hardcodes them to wire up accept/reject. Renaming breaks the OK button.

**No `QScrollArea`.** Crashes TheSkyX on OK click during dialog teardown.

**`uiEvent` event strings derive from widget names:** `"on_<objectName>_<signal>"`.
When renaming a widget, update the matching `strcmp` in `uiEvent()`.

**Dialog geometry is ignored.** `X2ModalUIUtil` ignores the `.ui` file's
`geometry` and `minimumSize` properties — dialog size is controlled by TheSkyX
internally. Two-column `QHBoxLayout` also does not render correctly.

## Serial port and the ESP32 reset (Linux)

Measured on hardware (CH340 + ESP32 auto-reset circuit):

| Lines | ESP32 |
|---|---|
| RTS asserted (any DTR) | Held in reset — USB hub power drops, every USB device behind the box disconnects |
| RTS released | Runs (boots if it was held) |
| DTR alone | No effect |

The Linux `ch341` driver asserts DTR+RTS on every `open()` unless the saved
termios speed is `B0`, and again whenever the speed changes from `B0` to a real
rate. So `SerXInterface::open()` always resets the box. On Linux all port I/O
goes through the `port*()` helpers on a private fd that is left at `B0`: the
chip keeps running at its last real rate (115200 8N1), and later opens never
touch RTS. Only the first open after the CH340 enumerates resets the box.
Do not route Linux I/O back through `m_pSerX`, and do not set a real baud on
the fd except in `portOpen()`/`portResetDevice()`.

## Sensor decoding

Firmware encoding: `wire_value = (physical + offset) × 100` as big-endian uint32.
Offsets: DS18B20 = 255.5, SHT40 temp/RH = 254.0.

- **Do not** use the DS18B20 datasheet formula (int16 × 0.0625) — gives ~800 °C.
- **Do not** use the SHT40 native ADC formula — firmware doesn't send raw counts.
- **Do not** apply a Magnus Psat correction to SHT40 RH — the enclosure is open
  so RH reads ambient correctly; correction overcorrects (~52 % → ~74 %).
- Dew point input: DS18B20 temperature + SHT40 RH. SHT40 self-heats ~5–6 °C
  so its temperature is not used for the dew algorithm (display only as "Hub temp").

## What didn't work

| Approach | Why |
|---|---|
| `QScrollArea` | TSX crashes on OK — X2GUI teardown can't handle the extra container |
| Renaming `pushButtonOK`/`pushButtonCancel` | X2 hardcodes these names |
| Magnus Psat correction on SHT40 RH | Overcorrects in open enclosure |
| DS18B20 decode as int16 × 0.0625 | Wrong formula; firmware uses offset encoding |
| Blocking `nanosleep` in `establishLink()` | Hangs TSX UI thread 5–15 s |
| Opening via `SerXInterface` on Linux | Kernel asserts RTS on open → ESP32 reset → USB hub power drop on every connect |
| Clearing DTR/RTS right after `open()` | Too late: a 0.8 ms RTS pulse still resets the ESP32 |
| Asserting DTR+RTS together "to keep the ESP32 running" | Wrong: RTS asserted *is* the reset; only releasing RTS lets it run |
| Reading sensors before warmup guard | ESP32 returns sentinel values for ~15 s post-reset |
| `geometry`/`minimumSize` in .ui | Ignored by `X2ModalUIUtil` |
| Two-column `QHBoxLayout` | Does not render correctly under TheSkyX's `QUiLoader` |
