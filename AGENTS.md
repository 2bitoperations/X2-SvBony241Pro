# Agent Guide

## Project purpose

This is a [TheSkyX X2 plugin](https://www.bisque.com/theskyx/) that exposes the
**SVBony SV241 Pro USB Power Hub** as a **Power Control Box** device within
TheSkyX Professional Edition. It implements `PowerControlDriverInterface` so
TheSkyX can switch DC ports, dew heaters, USB hub power, and a regulated
variable-voltage output through a single UI panel.

The plugin is a standard C++ shared library (`libx2svbony241pro.so` /
`.dylib` / `.dll`) that TheSkyX loads at runtime. It communicates with the
hardware over a USB CDC virtual serial port (115200 8N1). The device firmware
runs on an ESP32 and uses a binary framing protocol; see
[PROTOCOL.md](PROTOCOL.md) for the full specification.

## Repository layout

```
main.h / main.cpp              Plugin entry points (sbPlugInName2, sbPlugInFactory2)
x2svbony241pro.h               Driver class declaration — all interfaces and private API
x2svbony241pro.cpp             Driver class implementation — protocol I/O, dew control, UI events
sv241pro.ui                    Qt Designer UI file for the settings dialog
version.h                      Version constants; BUILD_NUMBER injected by CI
licensedinterfaces/            Vendored TheSkyX SDK headers (Software Bisque copyright)
powercontrollist Svbony.txt    Hardware list entry consumed by TheSkyX
Makefile                       Cross-platform build (Linux/macOS/Windows via MSYS2)
PROTOCOL.md                    Full binary serial protocol reference (derived from INDI driver)
.github/workflows/build.yml    CI: builds all three platforms, auto-releases on main
```

## Circuit layout

`X2_NUM_CIRCUITS` is 8. The X2 circuit index maps to device port_index bytes
as follows (see also `kPortIndex[]` in `x2svbony241pro.cpp` and
PROTOCOL.md §7–10):

| Circuit | Label              | Device port_index | Notes                              |
|---------|--------------------|-------------------|------------------------------------|
| 0       | DC Port 1 (12V)    | 0x00              | Switchable 12V DC                  |
| 1       | DC Port 2 (12V)    | 0x01              | Switchable 12V DC                  |
| 2       | DC Port 3 (12V)    | 0x02              | Switchable 12V DC                  |
| 3       | DC Port 4 (12V)    | 0x03              | Switchable 12V DC                  |
| 4       | Dew Heater 1       | 0x08              | PWM duty cycle, default 50%        |
| 5       | Dew Heater 2       | 0x09              | PWM duty cycle, default 50%        |
| 6       | USB Hub Power      | 0x05              | USB-C + USB1 + USB2 (group 0)      |
| 7       | Adjustable Output  | 0x07              | Regulated voltage, default 12V     |

The device has a fifth DC port (port_index 0x04) and a second USB group
(port_index 0x06, USB3–5) that are not exposed in the X2 interface.

## Building

**Local (Linux or macOS):**
```sh
make
```
Produces `libx2svbony241pro.so` (Linux) or `libx2svbony241pro.dylib` (macOS).
No external dependencies beyond a C++11 compiler and `math.h`. The
`licensedinterfaces/` headers are vendored in-tree.

**Windows:** use MSYS2/MinGW64 — `make` works identically; CI uses this path.

CI builds all three platforms on every push to `main` and publishes a GitHub
release tagged `v1.<commit-count>`. Each release artifact is a zip containing
the shared library, `sv241pro.ui`, and `powercontrollist Svbony.txt`.

## Code style

- **C++11** (`-std=gnu++11`), compiled with `-Wall -Wextra -O2`. Do not use
  C++14/17 features; TheSkyX's ABI is conservative.
- **No exceptions, no RTTI** beyond `dynamic_cast` in `queryAbstraction`,
  which is required by the X2 protocol.
- **Single-class architecture**: `X2Svbony241Pro` owns both the wire protocol
  and the TheSkyX interface. All binary I/O goes through `buildFrame` /
  `sendFrame` / `readFrame`; all device commands go through the `cmd*` methods.
  Do not call `SerXInterface` methods from anywhere except those helpers.
- **Error returns**: use `SB_OK` (0) and the `ERR_*` constants from
  `sberrorx.h`. Do not throw.
- **Logging**: call `logDebug(minLevel, fmt, ...)`. Levels are 0=Off,
  1=Errors, 2=Commands, 3=Full I/O. The level is persisted in the TheSkyX ini
  store under section `"SV241Pro"` and is user-selectable in the settings UI.
- **UI strings**: set via `X2GUIExchangeInterface::setText` inside `uiEvent`.
  Do not add Qt-specific includes or link against Qt directly — the UI is
  rendered by TheSkyX's own Qt runtime.
- **State cache**: `m_bCircuitState[]` is the authoritative on/off cache.
  Always refresh it via `queryAllCircuitStates()` / `parseStateResponse()`
  rather than tracking state locally in `setCircuitState`.

## Binary serial protocol

See **[PROTOCOL.md](PROTOCOL.md)** for the complete specification. Key facts:

- 115200 baud, 8N1, no flow control.
- Every frame: `[0x24, DATA_LEN, payload..., CHECKSUM]` where CHECKSUM is the
  sum of all payload bytes mod 0xFF.
- Command 0x01 sets any output channel; commands 0x02–0x07 read individual
  sensors; command 0x08 reads the full device state (10 bytes); 0xFF/0xFE
  triggers a power cycle.
- On connect the ESP32 resets, emitting a burst of ASCII boot-loader text.
  `drainBootLog()` must complete before binary commands are sent. RTS/DTR must
  be cleared to prevent an inadvertent reset on subsequent opens.
- Response frames: `[0x24, DATA_LEN, STATUS, data..., CHECKSUM]`. If STATUS
  is `0xAA` the command failed; `sendFrame` returns `ERR_CMDFAILED`.
- Sensor values are 4-byte big-endian integers requiring formula decoding;
  see `decodeSensor4`, `decodeDS18B20Temp`, `decodeSHT40Temp`,
  `decodeSHT40Humidity` in `x2svbony241pro.h`.

## Auto-dew control

Dew heaters (circuits 4 and 5) support two modes, persisted per-heater in the
TheSkyX ini store:

- `DEW_MODE_MANUAL` — heater runs at a fixed duty cycle (`m_nDewFixedDutyPct[]`,
  0–100%).
- `DEW_MODE_AUTO` — `updateDewControl()` reads the SHT40 ambient temperature
  and humidity, calculates the dew point via the Magnus formula
  (`calcDewPoint`), and drives the PWM to a duty cycle proportional to
  how far the lens temperature is above the dew point. The aggressiveness
  factor (1–10, default 5) scales the response. Sensor readings are
  rate-limited to once per 30 s (`kDewUpdateIntervalMs`).

If the SHT40 sensor read fails in AUTO mode the driver applies the configured
fallback (`DEW_FALLBACK_OFF` or `DEW_FALLBACK_ON`; default is
`DEW_FALLBACK_ON`, which keeps the heater running at the fixed duty cycle).

`updateDewControl()` is called from `setCircuitState` and `uiEvent`; it is
a no-op when no heater is in AUTO mode.

## SDK interfaces

`x2svbony241pro.h` inherits from four X2 interfaces:

| Interface                      | Purpose                                              |
|-------------------------------|------------------------------------------------------|
| `PowerControlDriverInterface` | Core power box API (`numberOfCircuits`, `circuitState`, `setCircuitState`) |
| `ModalSettingsDialogInterface`| Settings dialog lifecycle (`execModalSettingsDialog`) |
| `X2GUIEventInterface`         | UI event callbacks (`uiEvent`)                       |
| `CircuitLabelsInterface`      | Per-circuit label strings (`circuitLabel`)           |

All four are in `licensedinterfaces/`. The `queryAbstraction` method
advertises each interface by name string. Do not change the inheritance order
or add new base classes without updating `queryAbstraction` to match.

## Vendored headers

`licensedinterfaces/` contains the TheSkyX X2 SDK headers (copyright Software
Bisque). They are vendored because the SDK is not distributed publicly.
`x2guiinterface.h` carries the `X2_FLAT_INCLUDES` preprocessor guard patch
(also applied via `-DX2_FLAT_INCLUDES` in the Makefile). Do not upgrade these
headers without re-checking that patch.

## Version scheme

`version.h` defines `PLUGIN_VERSION_MAJOR = 1`. CI appends the git commit
count as `BUILD_NUMBER`, producing versions like `v1.57`. CI also injects
`GIT_HASH` (short SHA) at compile time for display in `driverInfoDetailedInfo`.
Bump `PLUGIN_VERSION_MAJOR` manually only for significant or breaking changes.

## Installation

1. Copy the compiled shared library to the TheSkyX **PowerControlBox** plugin
   directory:
   - **Linux:** `~/TheSkyXLinux/Plugins/PowerControlBox/`
   - **macOS:** `/Applications/TheSkyX Professional Edition.app/Contents/Resources/Common/Plugins/PowerControlBox/`
   - **Windows:** `C:\Program Files\Software Bisque\TheSkyX Professional Edition\Plugins\PowerControlBox\`

2. Copy `sv241pro.ui` to the same directory (TheSkyX loads it at runtime to
   render the settings dialog).

3. Copy `powercontrollist Svbony.txt` to the same directory (or append its
   single line to an existing `powercontrollist.txt` in that directory).

4. Restart TheSkyX. The **SVBony SV241 Pro** entry will appear in
   **Telescope > Power Control Box**.

---

## Agent Directives & Lessons Learned

Hard-won lessons from development. Read these before making changes.

### Build system
- **Always use `make clean && make install`** — never cmake.
- Confirm zero warnings and zero errors before reporting done.
- The install script prints `UI OK.` after XML validation — missing means
  the .ui file is malformed.

### UI file strict rules

**Widget naming convention** (descriptive camelCase with type prefix):

| Prefix     | Widget type       |
|------------|-------------------|
| `lbl`      | QLabel            |
| `spin`     | QSpinBox          |
| `dblSpin`  | QDoubleSpinBox    |
| `combo`    | QComboBox         |
| `btn`      | QPushButton       |
| `grp`      | QGroupBox         |
| `progress` | QProgressBar      |
| `chk`      | QCheckBox         |

Layouts and spacers use plain descriptive names (`layoutMain`, `spacerButtons`) with no type prefix.

**Exception — OK/Cancel buttons must keep their framework names.**
TheSkyX's X2GUI framework locates accept/reject buttons by the hardcoded names
`pushButtonOK` and `pushButtonCancel`. Renaming them causes `pUI->exec()` to
never set `bPressedOK = true` — the button appears to do nothing. Do not rename
these two, even during a blanket widget-rename pass.

**No QScrollArea.** Wrapping dialog content in a `QScrollArea` crashes TheSkyX
when OK is clicked. The `QUiLoader`-based X2GUI framework does not survive the
extra container layer during dialog teardown.

**uiEvent event names derive from widget object names.** When a widget is
renamed, update the corresponding `strcmp` in `uiEvent()` to match:
`"on_<objectName>_<signal>"`. E.g. renaming `comboBox_dew1Mode` →
`comboDew1Mode` requires changing `"on_comboBox_dew1Mode_currentIndexChanged"`
→ `"on_comboDew1Mode_currentIndexChanged"`.

### Sensor decoding

All sensor commands use the firmware encoding `wire_value = (physical + offset) × 100`
(big-endian uint32). Offsets: SHT40 temp/RH = 254.0, DS18B20 = 255.5.
**Do not use the raw DS18B20 datasheet formula (int16 × 0.0625)** — it gives
~800 °C. **Do not use the SHT40 native ADC formula** — firmware does not send
raw ADC counts.

SHT40 is mounted inside the powered hub and self-heats ~5–6 °C above ambient.
The RH reading is empirically close to true ambient RH (enclosure is not sealed).
**Do not apply a Magnus Psat correction to SHT40 RH** — it overcorrects
(observed: ~52 % → ~74 %). Use DS18B20 temperature + SHT40 RH as inputs to
`calcDewPoint()` when DS18B20 is valid. Set `m_dAmbientTempC` before calling
`calcDewPoint()` so the dew-point uses the correct temperature.

### What didn't work

| Approach | Why it failed |
|---|---|
| `QScrollArea` in dialog | TSX crashes on OK click — X2GUI can't handle extra container during teardown |
| Renaming `pushButtonOK`/`pushButtonCancel` | X2 framework hardcodes these names; OK appears broken |
| Magnus Psat correction on SHT40 RH | SHT40 reads ambient RH correctly in open enclosure; correction overcorrects |
| DS18B20 decode as int16 × 0.0625 | Firmware uses offset encoding, not raw register; result ~800 °C |
| Blocking `nanosleep` in `establishLink()` | Makes connect hang 5–15 s; use async warmup guard instead |
| SHT40 native ADC formula | Firmware uses offset encoding, not raw ADC counts |
| Calling `updateDewControl()` before warmup guard | Caches sentinel/garbage values during 15 s boot window |
