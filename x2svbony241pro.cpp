#include "x2svbony241pro.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Circuit name table — keep in sync with X2 circuit index definitions
// ---------------------------------------------------------------------------
static const char* kCircuitNames[X2_NUM_CIRCUITS] =
{
    "DC Port 1 (12V)",      // 0
    "DC Port 2 (12V)",      // 1
    "DC Port 3 (12V)",      // 2
    "DC Port 4 (12V)",      // 3
    "Dew Heater 1",         // 4
    "Dew Heater 2",         // 5
    "USB Hub Power",        // 6
    "Adjustable Output",    // 7
};

// Device port_index byte (for cmd 0x01), indexed by X2 circuit number.
// See PROTOCOL.md §7–10 for full mapping.
static const uint8_t kPortIndex[X2_NUM_CIRCUITS] =
{
    0x00,   // Circuit 0 → DC 1
    0x01,   // Circuit 1 → DC 2
    0x02,   // Circuit 2 → DC 3
    0x03,   // Circuit 3 → DC 4
    0x08,   // Circuit 4 → Dew Heater 1 (PWM 1)
    0x09,   // Circuit 5 → Dew Heater 2 (PWM 2)
    0x05,   // Circuit 6 → USB group 0 (USB-C, USB1, USB2)
    0x07,   // Circuit 7 → Regulated voltage output
};

// ---------------------------------------------------------------------------
// Protocol / timing constants
// ---------------------------------------------------------------------------
static const int     kBaudRate              = 115200;
static const uint8_t kFrameHeader           = 0x24;
static const uint8_t kStatusFailure         = 0xAA;

static const int     kPostWriteSleepMs      = 100;  // mirrors INDI tcdrain+sleep
static const int     kReadTimeoutMs         = 500;  // max wait for response bytes
static const int     kBootDrainInitSleepMs  = 500;  // initial wait for ESP32 boot
static const int     kBootDrainRetries      = 10;
static const int     kBootDrainSleepMs      = 50;
static const int     kPowerCycleSleepMs     = 1000;

// Default analogue levels used when toggling an analogue channel ON for the first time
static const int     kDefaultDewDutyPct         = 50;    // 50 % PWM duty cycle
static const double  kDefaultRegulatedVoltageV  = 12.0;  // 12 V

// Auto dew-point control
static const int           kDefaultDewAggressiveness  = 5;           // mid-range
static const DewHeaterMode kDefaultDewMode             = DEW_MODE_MANUAL;
static const DewFallback   kDefaultDewFallback         = DEW_FALLBACK_ON;
static const unsigned long kDewUpdateIntervalMs        = 30000;       // 30 s between sensor reads

// Ini section key for persisted dew settings
static const char* kIniSection = "SV241Pro";

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

X2Svbony241Pro::X2Svbony241Pro(
    const char*                         pszDisplayName,
    const int&                          nInstanceIndex,
    SerXInterface*                      pSerXIn,
    TheSkyXFacadeForDriversInterface*   pTheSkyXIn,
    SleeperInterface*                   pSleeperIn,
    BasicIniUtilInterface*              pIniUtilIn,
    LoggerInterface*                    pLoggerIn,
    MutexInterface*                     pIOMutexIn,
    TickCountInterface*                 pTickCountIn)
    : m_pSerX               (pSerXIn)
    , m_pTheSkyX            (pTheSkyXIn)
    , m_pSleeper            (pSleeperIn)
    , m_pIniUtil            (pIniUtilIn)
    , m_pLogger             (pLoggerIn)
    , m_pIOMutex            (pIOMutexIn)
    , m_pTickCount          (pTickCountIn)
    , m_nInstanceIndex      (nInstanceIndex)
    , m_bLinked             (false)
    , m_dPowerW             (0.0)
    , m_dVoltageV           (0.0)
    , m_dCurrentA           (0.0)
    , m_dRegulatedVoltageV  (kDefaultRegulatedVoltageV)
    , m_dAmbientTempC       (0.0)
    , m_dAmbientHumidityPct (0.0)
    , m_dDewPointC          (0.0)
    , m_bSensorValid        (false)
    , m_nLastDewUpdateTick  (0)
{
    (void)pszDisplayName;   // display name is managed by TheSkyX

    for (int i = 0; i < X2_NUM_CIRCUITS; ++i)
        m_bCircuitState[i] = false;

    for (int i = 0; i < 2; ++i)
    {
        m_nDewFixedDutyPct[i]   = kDefaultDewDutyPct;
        m_eDewMode[i]           = kDefaultDewMode;
        m_nDewAggressiveness[i] = kDefaultDewAggressiveness;
        m_eDewFallback[i]       = kDefaultDewFallback;
    }
}

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------

X2Svbony241Pro::~X2Svbony241Pro()
{
    // TSX owns all interface pointers — do NOT delete them here.
    if (m_bLinked)
        terminateLink();
}

// ---------------------------------------------------------------------------
// DriverRootInterface
// ---------------------------------------------------------------------------

int X2Svbony241Pro::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LoggerInterface::IID_LoggerInterface))
        *ppVal = m_pLogger;
    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);

    return SB_OK;   // return SB_OK even for unknown interfaces (*ppVal == NULL)
}

// ---------------------------------------------------------------------------
// DriverInfoInterface
// ---------------------------------------------------------------------------

void X2Svbony241Pro::driverInfoDetailedInfo(BasicStringInterface& str) const
{
    str = "X2 Power Control driver for the SVBony SV241 Pro USB Power Hub. "
          "Supports 4 x 12V DC outputs, 2 x dew heater ports (PWM 0-100%), "
          "integrated USB hub power, and one adjustable voltage output (0-15.3V). "
          "Environmental sensors: INA219 (power/voltage/current), "
          "DS18B20 (lens temperature), SHT40 (ambient temperature & humidity).";
}

double X2Svbony241Pro::driverInfoVersion() const { return 1.0; }

// ---------------------------------------------------------------------------
// DeviceInfoInterface
// ---------------------------------------------------------------------------

void X2Svbony241Pro::deviceInfoNameShort(BasicStringInterface& str) const
    { str = "SVBony SV241 Pro"; }

void X2Svbony241Pro::deviceInfoNameLong(BasicStringInterface& str) const
    { str = "SVBony SV241 Pro USB Power Hub"; }

void X2Svbony241Pro::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
    str = "SVBony SV241 Pro — USB-connected astronomy power control box and USB hub. "
          "4 x switchable 12V DC outputs, 2 x PWM dew heater ports, "
          "USB hub (2 groups), 1 x adjustable voltage output (0-15.3V), "
          "INA219 power monitor, DS18B20 and SHT40 environmental sensors.";
}

void X2Svbony241Pro::deviceInfoFirmwareVersion(BasicStringInterface& str)
    { str = m_bLinked ? "ESP32 firmware (version not reported by device)" : "N/A"; }

void X2Svbony241Pro::deviceInfoModel(BasicStringInterface& str)
    { str = "SV241 Pro"; }

// ---------------------------------------------------------------------------
// LinkInterface
// ---------------------------------------------------------------------------

int X2Svbony241Pro::establishLink()
{
    if (m_pSerX == NULL)
        return ERR_COMMNOLINK;

    // Open the USB virtual COM port at 115200 8N1, no parity, no flow control.
    int nErr = m_pSerX->open(NULL, kBaudRate, SerXInterface::B_NOPARITY, 0);
    if (nErr != SB_OK)
        return nErr;

    m_bLinked = true;

    // Restore persisted dew-heater configuration before touching the device.
    loadDewConfig();

    // Drain the ESP32 boot log.  The device resets when the serial port opens;
    // we must consume the boot text before sending binary commands.
    // Non-fatal: if drain times out we attempt the handshake anyway.
    (void)drainBootLog();

    // Handshake: cmd 0x08 (full state query).  If the device responds correctly
    // we consider the link established and pre-populate the circuit cache.
    uint8_t state10[10] = {0};
    nErr = cmdGetState(state10);
    if (nErr != SB_OK)
    {
        if (m_pLogger)
            m_pLogger->out("X2Svbony241Pro: handshake failed — closing port");
        m_pSerX->close();
        m_bLinked = false;
        return nErr;
    }

    parseStateResponse(state10);

    if (m_pLogger)
        m_pLogger->out("X2Svbony241Pro: link established");

    return SB_OK;
}

int X2Svbony241Pro::terminateLink()
{
    saveDewConfig();

    if (m_pSerX != NULL)
        m_pSerX->close();

    m_bLinked = false;

    if (m_pLogger)
        m_pLogger->out("X2Svbony241Pro: link terminated");

    return SB_OK;
}

bool X2Svbony241Pro::isLinked() const { return m_bLinked; }

// ---------------------------------------------------------------------------
// PowerControlDriverInterface
// ---------------------------------------------------------------------------

int X2Svbony241Pro::numberOfCircuits(int& nNumber)
{
    nNumber = X2_NUM_CIRCUITS;
    return SB_OK;
}

int X2Svbony241Pro::circuitName(const int& nIndex, BasicStringInterface& str)
{
    if (nIndex < 0 || nIndex >= X2_NUM_CIRCUITS)
    {
        str = "Unknown";
        return ERR_DATAOUT_OF_RANGE;
    }
    str = kCircuitNames[nIndex];
    return SB_OK;
}

int X2Svbony241Pro::circuitState(const int& nIndex, bool& bOn)
{
    if (!m_bLinked)
        return ERR_COMMNOLINK;
    if (nIndex < 0 || nIndex >= X2_NUM_CIRCUITS)
        return ERR_DATAOUT_OF_RANGE;

    // For dew heater circuits in auto mode, check whether it is time to run
    // a sensor read and PWM update.  TheSkyX polls circuitState() periodically,
    // so this is the natural place to drive the control loop.
    if (nIndex == 4 || nIndex == 5)
    {
        int heaterIdx = nIndex - 4;
        if (m_eDewMode[heaterIdx] == DEW_MODE_AUTO && m_bCircuitState[nIndex])
        {
            unsigned long now = m_pTickCount ? m_pTickCount->elapsed() : 0;
            if (now == 0 || (now - m_nLastDewUpdateTick) >= kDewUpdateIntervalMs)
                updateDewControl();
        }
    }

    bOn = m_bCircuitState[nIndex];
    return SB_OK;
}

int X2Svbony241Pro::setCircuitState(const int& nIndex, const bool& bOn)
{
    if (!m_bLinked)
        return ERR_COMMNOLINK;
    if (nIndex < 0 || nIndex >= X2_NUM_CIRCUITS)
        return ERR_DATAOUT_OF_RANGE;

    uint8_t portIdx = portIndexForCircuit(nIndex);
    int     nErr    = SB_OK;

    if ((nIndex == 4 || nIndex == 5) && m_eDewMode[nIndex - 4] == DEW_MODE_AUTO && bOn)
    {
        // Turning an auto-mode heater ON: run the control algorithm immediately
        // rather than waiting for the next poll cycle.  This gives instant
        // feedback rather than a potentially 30-second delay.
        m_bCircuitState[nIndex] = true;     // mark on so updateDewControl sees it
        m_nLastDewUpdateTick    = 0;        // force an immediate sensor read
        nErr = updateDewControl();
        // updateDewControl() sends the PWM command and may flip the circuit
        // back to false if the algorithm/fallback yields 0 %.
    }
    else
    {
        uint8_t value = bOn ? onValueForCircuit(nIndex) : static_cast<uint8_t>(0x00);
        nErr = cmdSetPort(portIdx, value);
        if (nErr == SB_OK)
            m_bCircuitState[nIndex] = bOn;
    }

    return nErr;
}

// ===========================================================================
// Binary frame I/O — private implementation
// ===========================================================================

// ---------------------------------------------------------------------------
// buildFrame
//
// Constructs the complete binary command frame:
//   [0x24 | DATA_LEN | cmd[0..nCmdLen-1] | CHECKSUM]
// where DATA_LEN = nCmdLen + 3  (see PROTOCOL.md §3).
// pFrameOut must be at least nCmdLen+3 bytes.
// ---------------------------------------------------------------------------
int X2Svbony241Pro::buildFrame(const uint8_t* pCmd, int nCmdLen,
                                uint8_t* pFrameOut, int& nFrameLenOut) const
{
    int nFrameLen = nCmdLen + 3;        // header + data_len + cmd... + checksum
    pFrameOut[0] = kFrameHeader;        // 0x24
    pFrameOut[1] = (uint8_t)nFrameLen; // DATA_LEN
    for (int i = 0; i < nCmdLen; ++i)
        pFrameOut[2 + i] = pCmd[i];
    // Checksum covers all bytes except the checksum itself
    pFrameOut[2 + nCmdLen] = calcChecksum(pFrameOut, nFrameLen - 1);
    nFrameLenOut = nFrameLen;
    return SB_OK;
}

// ---------------------------------------------------------------------------
// calcChecksum
//
// sum of all bytes in pBuf[0..nLen-1] modulo 0xFF.
// ---------------------------------------------------------------------------
uint8_t X2Svbony241Pro::calcChecksum(const uint8_t* pBuf, int nLen) const
{
    uint32_t sum = 0;
    for (int i = 0; i < nLen; ++i)
        sum += pBuf[i];
    return static_cast<uint8_t>(sum % 0xFF);
}

// ---------------------------------------------------------------------------
// sendFrame
//
// Sends a binary command and reads back the response.
//
// Command frame: [0x24, DATA_LEN, cmd[0..nCmdLen-1], CS]
// Response frame: [0x24, DATA_LEN, STATUS, data[0..nResDataLen-1], CS]
//
// On return pResDataOut contains nResDataLen payload bytes (bytes 3..3+n-1).
// ---------------------------------------------------------------------------
int X2Svbony241Pro::sendFrame(const uint8_t* pCmd, int nCmdLen,
                               int nResDataLen, uint8_t* pResDataOut)
{
    if (!m_bLinked || m_pSerX == NULL)
        return ERR_COMMNOLINK;

    // Build command frame (max frame size = 6 bytes for 3-byte cmd payload)
    uint8_t frame[16];
    int nFrameLen = 0;
    buildFrame(pCmd, nCmdLen, frame, nFrameLen);

    // Flush stale bytes before writing
    m_pSerX->purgeTxRx();

    // Write command frame
    unsigned long nWritten = 0;
    int nErr = m_pSerX->writeFile(frame, (unsigned long)nFrameLen, nWritten);
    if (nErr != SB_OK)
    {
        if (m_pLogger) m_pLogger->out("X2Svbony241Pro: writeFile error");
        return nErr;
    }
    if ((int)nWritten != nFrameLen)
    {
        if (m_pLogger) m_pLogger->out("X2Svbony241Pro: writeFile short write");
        return ERR_CMDFAILED;
    }

    // Post-write sleep: mirrors the INDI driver's tcdrain() + 100 ms sleep
    // to ensure the ESP32 has fully received the command before we read back.
    if (m_pSleeper)
        m_pSleeper->sleep(kPostWriteSleepMs);

    // Read response: header(1) + data_len(1) + status(1) + payload(n) + checksum(1)
    int nFullResLen = nResDataLen + 4;
    uint8_t resBuf[32];    // 14 bytes max (cmd 0x08); 32 is safe
    nErr = readFrame(nFullResLen, resBuf);
    if (nErr != SB_OK)
        return nErr;

    // Flush after read to clear any residual bytes
    m_pSerX->purgeTxRx();

    // Verify response checksum
    uint8_t csExpected = calcChecksum(resBuf, nFullResLen - 1);
    if (resBuf[nFullResLen - 1] != csExpected)
    {
        if (m_pLogger) m_pLogger->out("X2Svbony241Pro: checksum mismatch in response");
        return ERR_CMDFAILED;
    }

    // Check device status byte (response[2] == 0xAA means device rejected the command)
    if (resBuf[2] == kStatusFailure)
    {
        if (m_pLogger) m_pLogger->out("X2Svbony241Pro: device returned failure status (0xAA)");
        return ERR_CMDFAILED;
    }

    // Return the payload bytes to the caller (bytes at offsets 3..3+nResDataLen-1)
    for (int i = 0; i < nResDataLen; ++i)
        pResDataOut[i] = resBuf[3 + i];

    return SB_OK;
}

// ---------------------------------------------------------------------------
// readFrame
//
// Reads exactly nExpectedBytes from the serial port.
// Retries every 1 ms up to kReadTimeoutMs total.
// ---------------------------------------------------------------------------
int X2Svbony241Pro::readFrame(int nExpectedBytes, uint8_t* pBufOut)
{
    int nRead = 0;
    int nIdle = 0;

    while (nRead < nExpectedBytes && nIdle < kReadTimeoutMs)
    {
        unsigned long nGot = 0;
        int nErr = m_pSerX->readFile(pBufOut + nRead,
                                     (unsigned long)(nExpectedBytes - nRead),
                                     nGot);
        if (nErr != SB_OK)
            return nErr;

        if (nGot == 0)
        {
            ++nIdle;
            if (m_pSleeper) m_pSleeper->sleep(1);
        }
        else
        {
            nRead += (int)nGot;
            nIdle  = 0;     // reset idle counter on any progress
        }
    }

    if (nRead < nExpectedBytes)
    {
        if (m_pLogger) m_pLogger->out("X2Svbony241Pro: read timeout");
        return ERR_COMMTIMEOUT;
    }

    return SB_OK;
}

// ===========================================================================
// Initialization helpers
// ===========================================================================

// ---------------------------------------------------------------------------
// drainBootLog
//
// The SV241 Pro's ESP32 resets when the serial port opens (standard
// Arduino/ESP32 auto-reset via USB-CDC DTR/RTS).  It emits ~10 lines of
// boot text at 115200 before the firmware accepts binary commands.
//
// Strategy: wait 500 ms for the boot to start, then read and discard until
// we see kBootDrainRetries consecutive quiet reads (no bytes arriving).
//
// Note: the INDI driver uses ioctl(TIOCMSET) to explicitly clear RTS and DTR
// before draining.  SerXInterface does not expose ioctl, so we rely on the
// USB-CDC chip's built-in auto-reset behavior, which is present on standard
// ESP32 DevKit hardware.
// ---------------------------------------------------------------------------
int X2Svbony241Pro::drainBootLog()
{
    if (m_pLogger)
        m_pLogger->out("X2Svbony241Pro: waiting for ESP32 boot log to drain...");

    // Give the ESP32 time to complete its reset and emit the boot log
    if (m_pSleeper)
        m_pSleeper->sleep(kBootDrainInitSleepMs);

    uint8_t discard[64];
    int nQuietRounds = 0;
    const int kQuietTarget = 3;     // 3 consecutive empty reads → log is done

    for (int i = 0; i < kBootDrainRetries && nQuietRounds < kQuietTarget; ++i)
    {
        if (m_pSleeper)
            m_pSleeper->sleep(kBootDrainSleepMs);

        unsigned long nRead = 0;
        m_pSerX->readFile(discard, sizeof(discard), nRead);

        if (nRead == 0)
            ++nQuietRounds;
        else
            nQuietRounds = 0;
    }

    // Final flush of any residual bytes in the kernel buffer
    m_pSerX->purgeTxRx();

    if (m_pLogger)
        m_pLogger->out("X2Svbony241Pro: boot log drain complete");

    return SB_OK;
}

// ===========================================================================
// Device commands — one method per protocol command byte
// ===========================================================================

// ---------------------------------------------------------------------------
// cmdSetPort  (cmd 0x01)
//
// Payload: [0x01, portIndex, value]   (3 bytes)
// Response data: 2 bytes (acknowledged echo / status)
//
// portIndex: 0x00-0x04 = DC ports 1-5
//            0x05-0x06 = USB groups 0-1
//            0x07      = regulated voltage output
//            0x08-0x09 = dew heater PWM channels 1-2
// value:     0x00 = off
//            0xFF = full on (DC / USB)
//            0x01-0xFF = PWM raw (dew heaters) or voltage raw (regulated)
//
// See PROTOCOL.md §7–10 for per-channel encoding details.
// ---------------------------------------------------------------------------
int X2Svbony241Pro::cmdSetPort(uint8_t portIndex, uint8_t value)
{
    uint8_t cmd[3] = { 0x01, portIndex, value };
    uint8_t res[2] = { 0 };
    return sendFrame(cmd, 3, 2, res);
}

// ---------------------------------------------------------------------------
// cmdReadPower  (cmd 0x02)
//
// Payload: [0x02]  (1 byte)
// Response data: 4-byte big-endian unsigned integer
// Decode: raw / 100.0 = power in mW → divide by 1000 for W
// ---------------------------------------------------------------------------
int X2Svbony241Pro::cmdReadPower(double& powerW)
{
    uint8_t cmd[1] = { 0x02 };
    uint8_t res[4] = { 0 };
    int nErr = sendFrame(cmd, 1, 4, res);
    if (nErr == SB_OK)
        powerW = decodeSensor4(res) / 1000.0;   // mW × 100 → W
    return nErr;
}

// ---------------------------------------------------------------------------
// cmdReadVoltage  (cmd 0x03)
//
// Response data: 4-byte big-endian unsigned integer
// Decode: raw / 100.0 = bus voltage in V
// ---------------------------------------------------------------------------
int X2Svbony241Pro::cmdReadVoltage(double& voltageV)
{
    uint8_t cmd[1] = { 0x03 };
    uint8_t res[4] = { 0 };
    int nErr = sendFrame(cmd, 1, 4, res);
    if (nErr == SB_OK)
        voltageV = decodeSensor4(res);
    return nErr;
}

// ---------------------------------------------------------------------------
// cmdReadDS18B20Temp  (cmd 0x04)
//
// Response data: 4-byte big-endian unsigned integer
// Decode: (raw/100.0 - 255.5) rounded to 2 d.p. → °C
// The 255.5 is a firmware-baked offset on the ESP32 side.
// ---------------------------------------------------------------------------
int X2Svbony241Pro::cmdReadDS18B20Temp(double& tempC)
{
    uint8_t cmd[1] = { 0x04 };
    uint8_t res[4] = { 0 };
    int nErr = sendFrame(cmd, 1, 4, res);
    if (nErr == SB_OK)
        tempC = decodeDS18B20Temp(res);
    return nErr;
}

// ---------------------------------------------------------------------------
// cmdReadSHT40Temp  (cmd 0x05)
//
// Response data: 4-byte big-endian unsigned integer
// Decode: (raw/100.0 - 254.0) rounded to 1 d.p. → °C
// ---------------------------------------------------------------------------
int X2Svbony241Pro::cmdReadSHT40Temp(double& tempC)
{
    uint8_t cmd[1] = { 0x05 };
    uint8_t res[4] = { 0 };
    int nErr = sendFrame(cmd, 1, 4, res);
    if (nErr == SB_OK)
        tempC = decodeSHT40Temp(res);
    return nErr;
}

// ---------------------------------------------------------------------------
// cmdReadSHT40Humidity  (cmd 0x06)
//
// Response data: 4-byte big-endian unsigned integer
// Decode: (raw/100.0 - 254.0) rounded to 1 d.p. → % RH
// ---------------------------------------------------------------------------
int X2Svbony241Pro::cmdReadSHT40Humidity(double& humidityPct)
{
    uint8_t cmd[1] = { 0x06 };
    uint8_t res[4] = { 0 };
    int nErr = sendFrame(cmd, 1, 4, res);
    if (nErr == SB_OK)
        humidityPct = decodeSHT40Humidity(res);
    return nErr;
}

// ---------------------------------------------------------------------------
// cmdReadCurrent  (cmd 0x07)
//
// Response data: 4-byte big-endian unsigned integer
// Decode: raw / 100.0 = current in mA → divide by 1000 for A
// ---------------------------------------------------------------------------
int X2Svbony241Pro::cmdReadCurrent(double& currentA)
{
    uint8_t cmd[1] = { 0x07 };
    uint8_t res[4] = { 0 };
    int nErr = sendFrame(cmd, 1, 4, res);
    if (nErr == SB_OK)
        currentA = decodeSensor4(res) / 1000.0;  // mA × 100 → A
    return nErr;
}

// ---------------------------------------------------------------------------
// cmdGetState  (cmd 0x08)
//
// Payload: [0x08]  (1 byte)
// Response data: 10 bytes
//
// Byte layout of the 10-byte payload (see PROTOCOL.md §12):
//   [0] GPIO1  DC port 1 state    (0 = off, non-zero = on)
//   [1] GPIO2  DC port 2 state
//   [2] GPIO3  DC port 3 state
//   [3] GPIO4  DC port 4 state
//   [4] GPIO5  DC port 5 state
//   [5] GPIO6  USB group 0 state
//   [6] GPIO7  USB group 1 state
//   [7] pwmA   Regulated voltage raw (0–255, V = byte×15.3/255)
//   [8] pwmB   Dew heater 1 raw     (0–255, duty% = byte/255×100)
//   [9] pwmC   Dew heater 2 raw     (0–255, duty% = byte/255×100)
// ---------------------------------------------------------------------------
int X2Svbony241Pro::cmdGetState(uint8_t* pState10Out)
{
    uint8_t cmd[1] = { 0x08 };
    return sendFrame(cmd, 1, 10, pState10Out);
}

// ---------------------------------------------------------------------------
// cmdPowerCycle
//
// Two-step sequence (see PROTOCOL.md §15):
//   Step 1: send [0xFF, 0xFF] → all outputs off
//   Wait 1000 ms
//   Step 2: send [0xFE, 0xFE] → all outputs back on
// ---------------------------------------------------------------------------
int X2Svbony241Pro::cmdPowerCycle()
{
    uint8_t res[2] = { 0 };

    // Step 1 — all off
    uint8_t cmdOff[2] = { 0xFF, 0xFF };
    int nErr = sendFrame(cmdOff, 2, 2, res);
    if (nErr != SB_OK)
        return nErr;

    if (m_pSleeper)
        m_pSleeper->sleep(kPowerCycleSleepMs);

    // Step 2 — all back on
    uint8_t cmdOn[2] = { 0xFE, 0xFE };
    return sendFrame(cmdOn, 2, 2, res);
}

// ===========================================================================
// Sensor decode helpers
// ===========================================================================

// ---------------------------------------------------------------------------
// decodeSensor4
//
// Interprets pData4 as a 4-byte big-endian unsigned integer and divides by
// 100.0.  This is the base decode for all sensor commands (0x02–0x07).
// ---------------------------------------------------------------------------
double X2Svbony241Pro::decodeSensor4(const uint8_t* pData4)
{
    uint32_t raw = (static_cast<uint32_t>(pData4[0]) << 24)
                 | (static_cast<uint32_t>(pData4[1]) << 16)
                 | (static_cast<uint32_t>(pData4[2]) <<  8)
                 |  static_cast<uint32_t>(pData4[3]);
    return static_cast<double>(raw) / 100.0;
}

// ---------------------------------------------------------------------------
// decodeDS18B20Temp  (cmd 0x04)
//
// scaled = raw / 100.0
// temp   = scaled - 255.5       (firmware-baked offset)
// Result rounded to 2 decimal places.
// ---------------------------------------------------------------------------
double X2Svbony241Pro::decodeDS18B20Temp(const uint8_t* pData4)
{
    double scaled = decodeSensor4(pData4);
    double tempC  = scaled - 255.5;
    return floor(tempC * 100.0 + 0.5) / 100.0;
}

// ---------------------------------------------------------------------------
// decodeSHT40Temp  (cmd 0x05)
//
// scaled = raw / 100.0
// temp   = scaled - 254.0       (firmware-baked offset)
// Result rounded to 1 decimal place.
// ---------------------------------------------------------------------------
double X2Svbony241Pro::decodeSHT40Temp(const uint8_t* pData4)
{
    double scaled = decodeSensor4(pData4);
    double tempC  = scaled - 254.0;
    return floor(tempC * 10.0 + 0.5) / 10.0;
}

// ---------------------------------------------------------------------------
// decodeSHT40Humidity  (cmd 0x06)
//
// Same formula as SHT40 temperature decode; units are % RH.
// ---------------------------------------------------------------------------
double X2Svbony241Pro::decodeSHT40Humidity(const uint8_t* pData4)
{
    double scaled    = decodeSensor4(pData4);
    double humidity  = scaled - 254.0;
    return floor(humidity * 10.0 + 0.5) / 10.0;
}

// ===========================================================================
// Circuit ↔ device port mapping
// ===========================================================================

// ---------------------------------------------------------------------------
// portIndexForCircuit
//
// Returns the device port_index byte (for cmd 0x01) for X2 circuit nCircuit.
// ---------------------------------------------------------------------------
uint8_t X2Svbony241Pro::portIndexForCircuit(int nCircuit)
{
    if (nCircuit < 0 || nCircuit >= X2_NUM_CIRCUITS)
        return 0xFF;    // invalid — caller checks return code before this
    return kPortIndex[nCircuit];
}

// ---------------------------------------------------------------------------
// onValueForCircuit
//
// Returns the raw value byte that represents "ON" for the given X2 circuit.
//
// DC ports / USB:  0xFF (fully on, digital)
// Dew heaters:     encoded duty cycle byte (0x01–0xFF) from m_nDewFixedDutyPct[]
// Regulated output: encoded voltage byte (0x01–0xFF) from m_dRegulatedVoltageV
//
// For analogue channels we clamp to at least 0x01 so that "on" never sends
// the same byte as "off" (0x00) when the stored level is at its minimum.
// ---------------------------------------------------------------------------
uint8_t X2Svbony241Pro::onValueForCircuit(int nCircuit) const
{
    if (nCircuit == 4 || nCircuit == 5)     // Dew Heater 1 or 2
    {
        int   dutyIdx = nCircuit - 4;       // 0 or 1
        double duty   = static_cast<double>(m_nDewFixedDutyPct[dutyIdx]);
        uint8_t raw   = static_cast<uint8_t>(floor(255.0 * duty / 100.0 + 0.5));
        return raw ? raw : 0x01;
    }

    if (nCircuit == 7)                      // Regulated voltage output
    {
        double v = m_dRegulatedVoltageV;
        if (v <= 0.0) v = kDefaultRegulatedVoltageV;
        if (v > 15.3) v = 15.3;
        uint8_t raw = static_cast<uint8_t>(floor(v / 15.3 * 255.0 + 0.5));
        return raw ? raw : 0x01;
    }

    // DC ports (circuits 0-3) and USB hub (circuit 6): fully on
    return 0xFF;
}

// ---------------------------------------------------------------------------
// parseStateResponse
//
// Decodes the 10-byte payload from cmdGetState() and updates:
//   m_bCircuitState[]         (cached on/off for X2 interface)
//   m_nDewFixedDutyPct[]           (current dew heater duty %, if channel is on)
//   m_dRegulatedVoltageV      (current regulated voltage, if channel is on)
//
// Payload byte layout — see PROTOCOL.md §12 and cmdGetState() above.
// ---------------------------------------------------------------------------
void X2Svbony241Pro::parseStateResponse(const uint8_t* pState10)
{
    // DC ports 1-4 (circuits 0-3).  Device has a 5th DC port (pState10[4])
    // which we do not expose in the X2 circuit map.
    m_bCircuitState[0] = (pState10[0] != 0);
    m_bCircuitState[1] = (pState10[1] != 0);
    m_bCircuitState[2] = (pState10[2] != 0);
    m_bCircuitState[3] = (pState10[3] != 0);

    // USB group 0 → circuit 6 (group 1 at pState10[6] is not exposed)
    m_bCircuitState[6] = (pState10[5] != 0);

    // Regulated voltage output → circuit 7
    // Raw byte (pState10[7]) encodes voltage as: V = byte × 15.3 / 255.0
    double regV = static_cast<double>(pState10[7]) * 15.3 / 255.0;
    m_bCircuitState[7] = (regV > 0.0);
    if (m_bCircuitState[7])
        m_dRegulatedVoltageV = regV;    // keep stored level in sync with device

    // Dew heater 1 → circuit 4;  raw byte encodes duty%: pct = byte/255×100
    int duty0 = static_cast<int>(floor(static_cast<double>(pState10[8]) / 255.0 * 100.0 + 0.5));
    m_bCircuitState[4] = (duty0 > 0);
    // Only back-sync the fixed level when in manual mode; auto mode owns the duty value.
    if (m_bCircuitState[4] && m_eDewMode[0] == DEW_MODE_MANUAL)
        m_nDewFixedDutyPct[0] = duty0;

    // Dew heater 2 → circuit 5
    int duty1 = static_cast<int>(floor(static_cast<double>(pState10[9]) / 255.0 * 100.0 + 0.5));
    m_bCircuitState[5] = (duty1 > 0);
    if (m_bCircuitState[5] && m_eDewMode[1] == DEW_MODE_MANUAL)
        m_nDewFixedDutyPct[1] = duty1;
}

// ---------------------------------------------------------------------------
// queryAllCircuitStates
//
// Issues cmd 0x08 and refreshes m_bCircuitState[] from the response.
// Called on connect and may be called by the TheSkyX polling loop.
// ---------------------------------------------------------------------------
int X2Svbony241Pro::queryAllCircuitStates()
{
    uint8_t state10[10] = {0};
    int nErr = cmdGetState(state10);
    if (nErr != SB_OK)
        return nErr;

    parseStateResponse(state10);
    return SB_OK;
}

// ===========================================================================
// Auto dew-point control
// ===========================================================================

// ---------------------------------------------------------------------------
// calcDewPoint
//
// Magnus formula for dew point temperature (°C).
//
//   γ  = ln(RH/100) + a·T / (b + T)
//   Td = b·γ / (a − γ)
//
// Constants: a = 17.625, b = 243.04 °C  (August-Roche-Magnus approximation)
// Valid range: −40 °C to +60 °C, 1–100 % RH.  Returns a reasonable value
// outside that range but accuracy degrades.
// ---------------------------------------------------------------------------
double X2Svbony241Pro::calcDewPoint(double ambientTempC, double humidityPct)
{
    // Guard against domain errors in log(); clamp to a physically valid range.
    if (humidityPct <= 0.0)  humidityPct = 0.1;
    if (humidityPct > 100.0) humidityPct = 100.0;

    const double a = 17.625;
    const double b = 243.04;    // °C

    double gamma  = log(humidityPct / 100.0) + a * ambientTempC / (b + ambientTempC);
    return b * gamma / (a - gamma);
}

// ---------------------------------------------------------------------------
// calcAutoDutyPct
//
// Returns the duty cycle % (0–100) that the auto algorithm prescribes for
// heater heaterIdx (0 or 1).
//
// Algorithm
// ---------
// Dew-point depression (DPD) = ambientTemp − dewPoint.
// A larger DPD means the air is drier / farther from condensation, so less
// heating is needed.
//
// The "response window" W (°C) is the DPD range over which the heater ramps
// from 100 % down to 0 %:
//
//   W = (11 − aggressiveness) × 2
//
// Aggressiveness 10 → W = 2 °C  (heater hits 0 % at DPD ≥ 2 °C — aggressive)
// Aggressiveness  5 → W = 12 °C (mid-range)
// Aggressiveness  1 → W = 20 °C (gentle; only full-off at DPD ≥ 20 °C)
//
// duty % = clamp(100 × (1 − DPD / W), 0, 100)
//
// If m_bSensorValid is false the configured fallback is applied instead.
// ---------------------------------------------------------------------------
int X2Svbony241Pro::calcAutoDutyPct(int heaterIdx) const
{
    if (!m_bSensorValid)
    {
        if (m_eDewFallback[heaterIdx] == DEW_FALLBACK_ON)
            return m_nDewFixedDutyPct[heaterIdx];
        return 0;
    }

    double dpd    = m_dAmbientTempC - m_dDewPointC;
    if (dpd < 0.0) dpd = 0.0;

    int    agg    = m_nDewAggressiveness[heaterIdx];
    if (agg < 1)  agg = 1;
    if (agg > 10) agg = 10;

    double window = static_cast<double>(11 - agg) * 2.0;  // 2–20 °C

    int duty;
    if (dpd <= 0.0)
        duty = 100;
    else if (dpd >= window)
        duty = 0;
    else
        duty = static_cast<int>(floor(100.0 * (1.0 - dpd / window) + 0.5));

    return duty;
}

// ---------------------------------------------------------------------------
// updateDewControl
//
// 1. Reads SHT40 temperature and humidity.
// 2. Calculates the dew point.
// 3. For each heater in DEW_MODE_AUTO that is currently on, computes the
//    target duty cycle and sends the PWM command to the device.
// 4. If the algorithm yields 0 % (heater should be off), the circuit state
//    cache is updated to reflect that and 0x00 is sent.
//
// Rate-limited: does nothing if called within kDewUpdateIntervalMs of the
// last update (uses TickCountInterface).
//
// Never returns a fatal error to the caller — sensor failures are handled
// internally via the per-heater fallback configuration.
// ---------------------------------------------------------------------------
int X2Svbony241Pro::updateDewControl()
{
    if (!m_bLinked)
        return ERR_COMMNOLINK;

    // Rate-limit: skip if updated recently.
    unsigned long now = m_pTickCount ? m_pTickCount->elapsed() : 0;
    if (m_nLastDewUpdateTick != 0 && now != 0 &&
        (now - m_nLastDewUpdateTick) < kDewUpdateIntervalMs)
        return SB_OK;

    m_nLastDewUpdateTick = now;

    // --- Read sensors ---
    double tempC    = 0.0;
    double humidity = 0.0;
    int nErrT = cmdReadSHT40Temp(tempC);
    int nErrH = cmdReadSHT40Humidity(humidity);

    if (nErrT == SB_OK && nErrH == SB_OK &&
        humidity >= 0.0 && humidity <= 100.0)
    {
        m_dAmbientTempC         = tempC;
        m_dAmbientHumidityPct   = humidity;
        m_dDewPointC            = calcDewPoint(tempC, humidity);
        m_bSensorValid          = true;

        if (m_pLogger)
        {
            char szMsg[128];
            snprintf(szMsg, sizeof(szMsg),
                     "X2Svbony241Pro: dew update — ambient %.1f°C  RH %.0f%%  dewpoint %.1f°C",
                     m_dAmbientTempC, m_dAmbientHumidityPct, m_dDewPointC);
            m_pLogger->out(szMsg);
        }
    }
    else
    {
        m_bSensorValid = false;
        if (m_pLogger)
            m_pLogger->out("X2Svbony241Pro: dew update — sensor read failed, applying fallback");
    }

    // --- Apply auto PWM to each heater that is on and in auto mode ---
    for (int i = 0; i < 2; ++i)
    {
        if (m_eDewMode[i] != DEW_MODE_AUTO)
            continue;
        if (!m_bCircuitState[4 + i])
            continue;

        int duty = calcAutoDutyPct(i);

        if (m_pLogger)
        {
            char szMsg[80];
            snprintf(szMsg, sizeof(szMsg),
                     "X2Svbony241Pro: dew heater %d auto → %d%%", i + 1, duty);
            m_pLogger->out(szMsg);
        }

        uint8_t raw = static_cast<uint8_t>(
            floor(255.0 * static_cast<double>(duty) / 100.0 + 0.5));

        // Send the new PWM value to the device.
        // Non-fatal: if the command fails we keep the cached state unchanged
        // and will retry on the next poll cycle.
        int nErr = cmdSetPort(kPortIndex[4 + i], raw);
        if (nErr == SB_OK && duty == 0)
        {
            // Algorithm says fully off — reflect that in the circuit cache so
            // TheSkyX sees the heater as off.
            m_bCircuitState[4 + i] = false;
        }
    }

    return SB_OK;
}

// ---------------------------------------------------------------------------
// saveDewConfig
//
// Writes per-heater dew control settings to the TheSkyX ini store so they
// survive across sessions.  Called from terminateLink().
// ---------------------------------------------------------------------------
void X2Svbony241Pro::saveDewConfig()
{
    if (m_pIniUtil == NULL)
        return;

    // Build instance-scoped key names so multiple devices don't collide.
    char szKey[64];
    for (int i = 0; i < 2; ++i)
    {
        snprintf(szKey, sizeof(szKey), "DewMode%d_%d",           i, m_nInstanceIndex);
        m_pIniUtil->writeInt(kIniSection, szKey, static_cast<int>(m_eDewMode[i]));

        snprintf(szKey, sizeof(szKey), "DewFixedDuty%d_%d",      i, m_nInstanceIndex);
        m_pIniUtil->writeInt(kIniSection, szKey, m_nDewFixedDutyPct[i]);

        snprintf(szKey, sizeof(szKey), "DewAggressiveness%d_%d", i, m_nInstanceIndex);
        m_pIniUtil->writeInt(kIniSection, szKey, m_nDewAggressiveness[i]);

        snprintf(szKey, sizeof(szKey), "DewFallback%d_%d",       i, m_nInstanceIndex);
        m_pIniUtil->writeInt(kIniSection, szKey, static_cast<int>(m_eDewFallback[i]));
    }
}

// ---------------------------------------------------------------------------
// loadDewConfig
//
// Reads per-heater dew control settings from the TheSkyX ini store.
// Called from establishLink() before any device commands are sent.
// Missing keys leave the in-memory defaults untouched.
// ---------------------------------------------------------------------------
void X2Svbony241Pro::loadDewConfig()
{
    if (m_pIniUtil == NULL)
        return;

    char szKey[64];
    int  nVal = 0;

    for (int i = 0; i < 2; ++i)
    {
        snprintf(szKey, sizeof(szKey), "DewMode%d_%d", i, m_nInstanceIndex);
        m_pIniUtil->readInt(kIniSection, szKey,
                            static_cast<int>(kDefaultDewMode), nVal);
        m_eDewMode[i] = (nVal == static_cast<int>(DEW_MODE_AUTO))
                        ? DEW_MODE_AUTO : DEW_MODE_MANUAL;

        snprintf(szKey, sizeof(szKey), "DewFixedDuty%d_%d", i, m_nInstanceIndex);
        m_pIniUtil->readInt(kIniSection, szKey, kDefaultDewDutyPct, nVal);
        if (nVal < 0)   nVal = 0;
        if (nVal > 100) nVal = 100;
        m_nDewFixedDutyPct[i] = nVal;

        snprintf(szKey, sizeof(szKey), "DewAggressiveness%d_%d", i, m_nInstanceIndex);
        m_pIniUtil->readInt(kIniSection, szKey, kDefaultDewAggressiveness, nVal);
        if (nVal < 1)  nVal = 1;
        if (nVal > 10) nVal = 10;
        m_nDewAggressiveness[i] = nVal;

        snprintf(szKey, sizeof(szKey), "DewFallback%d_%d", i, m_nInstanceIndex);
        m_pIniUtil->readInt(kIniSection, szKey,
                            static_cast<int>(kDefaultDewFallback), nVal);
        m_eDewFallback[i] = (nVal == static_cast<int>(DEW_FALLBACK_ON))
                            ? DEW_FALLBACK_ON : DEW_FALLBACK_OFF;
    }

    if (m_pLogger)
    {
        char szMsg[160];
        snprintf(szMsg, sizeof(szMsg),
                 "X2Svbony241Pro: dew config loaded — "
                 "H1 mode=%s agg=%d fallback=%s fixed=%d%%  "
                 "H2 mode=%s agg=%d fallback=%s fixed=%d%%",
                 m_eDewMode[0] == DEW_MODE_AUTO ? "AUTO" : "MANUAL",
                 m_nDewAggressiveness[0],
                 m_eDewFallback[0] == DEW_FALLBACK_ON ? "ON" : "OFF",
                 m_nDewFixedDutyPct[0],
                 m_eDewMode[1] == DEW_MODE_AUTO ? "AUTO" : "MANUAL",
                 m_nDewAggressiveness[1],
                 m_eDewFallback[1] == DEW_FALLBACK_ON ? "ON" : "OFF",
                 m_nDewFixedDutyPct[1]);
        m_pLogger->out(szMsg);
    }
}

// ===========================================================================
// Setup dialog  (ModalSettingsDialogInterface + X2GUIEventInterface)
// ===========================================================================

// ---------------------------------------------------------------------------
// Helper: show or hide all auto-only rows for one heater in the dialog.
// heaterIdx 0 = Dew Heater 1, 1 = Dew Heater 2.
// bAutoMode true → show the auto-only widgets; false → hide them.
// ---------------------------------------------------------------------------
static void setDewAutoRowsVisible(X2GUIExchangeInterface* uiex, int heaterIdx, bool bAutoMode)
{
    // Widget name suffix: "" for heater 0, "2" for heater 1 — matches the .ui names
    // dew1 = heaterIdx 0, dew2 = heaterIdx 1
    const char* suffix = (heaterIdx == 0) ? "1" : "2";

    char szName[64];
    const char* kAutoWidgets[] = {
        "label_dew%sAggCaption",
        "spinBox_dew%sAggressiveness",
        "label_dew%sAggHint",
        "label_dew%sFallbackCaption",
        "comboBox_dew%sFallback",
        "label_dew%sAutoDutyCaption",
        "label_dew%sAutoDuty",
    };
    const char* method = bAutoMode ? "show" : "hide";

    for (int i = 0; i < 7; ++i)
    {
        snprintf(szName, sizeof(szName), kAutoWidgets[i], suffix);
        uiex->invokeMethod(szName, method);
    }
}

// ---------------------------------------------------------------------------
// execModalSettingsDialog
//
// Loads sv241pro.ui, pre-populates controls from current driver state,
// runs the dialog modally, and saves settings on OK.
// ---------------------------------------------------------------------------
int X2Svbony241Pro::execModalSettingsDialog()
{
    X2ModalUIUtil uiutil(this, m_pTheSkyX);
    X2GUIInterface* pUI = uiutil.X2UI();
    if (pUI == NULL)
        return ERR_POINTER;

    X2GUIExchangeInterface* uiex = NULL;
    bool bPressedOK = false;
    int  nErr = SB_OK;

    do {
        nErr = pUI->loadUserInterface("sv241pro.ui", deviceType(), m_nInstanceIndex);
        if (nErr) break;

        uiex = pUI->X2DX();
        if (uiex == NULL) { nErr = ERR_POINTER; break; }

        // --- Populate static labels ---
        uiex->setText("label_connStatus",
                       m_bLinked ? "Connected" : "Not connected");
        uiex->setText("label_sensorStatus",
                       m_bSensorValid ? "OK" : (m_bLinked ? "Read error" : "—"));

        if (m_bLinked && m_bSensorValid)
        {
            char szBuf[64];
            snprintf(szBuf, sizeof(szBuf), "%.1f °C", m_dAmbientTempC);
            uiex->setText("label_ambientTemp", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.1f %%", m_dAmbientHumidityPct);
            uiex->setText("label_humidity", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.1f °C", m_dDewPointC);
            uiex->setText("label_dewPoint", szBuf);
        }

        // DS18B20 and INA219 — attempt a quick live read if linked
        if (m_bLinked)
        {
            char szBuf[64];
            double val = 0.0;

            if (cmdReadDS18B20Temp(val) == SB_OK)
            {
                snprintf(szBuf, sizeof(szBuf), "%.1f °C", val);
                uiex->setText("label_lensTemp", szBuf);
            }
            if (cmdReadVoltage(val) == SB_OK)
            {
                snprintf(szBuf, sizeof(szBuf), "%.2f V", val);
                uiex->setText("label_voltage", szBuf);
            }
            if (cmdReadCurrent(val) == SB_OK)
            {
                snprintf(szBuf, sizeof(szBuf), "%.3f A", val);
                uiex->setText("label_current", szBuf);
            }

            // Port states from cache
            const char* kDCLabels[] = { "label_dc1","label_dc2","label_dc3","label_dc4" };
            for (int i = 0; i < 4; ++i)
                uiex->setText(kDCLabels[i], m_bCircuitState[i] ? "ON" : "OFF");
            uiex->setText("label_usb",
                           m_bCircuitState[6] ? "ON" : "OFF");

            if (m_bCircuitState[7])
            {
                snprintf(szBuf, sizeof(szBuf), "%.1f V", m_dRegulatedVoltageV);
                uiex->setText("label_regulated", szBuf);
            }
            else
            {
                uiex->setText("label_regulated", "OFF");
            }
        }

        // --- Dew heater settings ---
        for (int i = 0; i < 2; ++i)
        {
            const char* modeCombo  = (i == 0) ? "comboBox_dew1Mode"         : "comboBox_dew2Mode";
            const char* dutyBox    = (i == 0) ? "spinBox_dew1FixedDuty"     : "spinBox_dew2FixedDuty";
            const char* aggBox     = (i == 0) ? "spinBox_dew1Aggressiveness" : "spinBox_dew2Aggressiveness";
            const char* fallCombo  = (i == 0) ? "comboBox_dew1Fallback"     : "comboBox_dew2Fallback";
            const char* autoDuty   = (i == 0) ? "label_dew1AutoDuty"        : "label_dew2AutoDuty";

            uiex->setCurrentIndex(modeCombo,
                                  (m_eDewMode[i] == DEW_MODE_AUTO) ? 1 : 0);
            uiex->setPropertyInt(dutyBox, "value", m_nDewFixedDutyPct[i]);
            uiex->setPropertyInt(aggBox,  "value", m_nDewAggressiveness[i]);
            uiex->setCurrentIndex(fallCombo,
                                  (m_eDewFallback[i] == DEW_FALLBACK_ON) ? 1 : 0);

            // Show/hide auto-only rows
            bool bAuto = (m_eDewMode[i] == DEW_MODE_AUTO);
            setDewAutoRowsVisible(uiex, i, bAuto);

            // Computed duty label
            if (bAuto && m_bSensorValid)
            {
                char szBuf[32];
                snprintf(szBuf, sizeof(szBuf), "%d %%", calcAutoDutyPct(i));
                uiex->setText(autoDuty, szBuf);
            }
        }

        nErr = pUI->exec(bPressedOK);
        if (nErr) break;

        if (bPressedOK)
        {
            // Read back dew heater settings and persist
            for (int i = 0; i < 2; ++i)
            {
                const char* modeCombo = (i == 0) ? "comboBox_dew1Mode"          : "comboBox_dew2Mode";
                const char* dutyBox   = (i == 0) ? "spinBox_dew1FixedDuty"      : "spinBox_dew2FixedDuty";
                const char* aggBox    = (i == 0) ? "spinBox_dew1Aggressiveness"  : "spinBox_dew2Aggressiveness";
                const char* fallCombo = (i == 0) ? "comboBox_dew1Fallback"      : "comboBox_dew2Fallback";

                m_eDewMode[i]           = (uiex->currentIndex(modeCombo) == 1)
                                          ? DEW_MODE_AUTO : DEW_MODE_MANUAL;
                m_eDewFallback[i]       = (uiex->currentIndex(fallCombo) == 1)
                                          ? DEW_FALLBACK_ON : DEW_FALLBACK_OFF;

                int nDuty = 50, nAgg = 5;
                uiex->propertyInt(dutyBox, "value", nDuty);
                uiex->propertyInt(aggBox,  "value", nAgg);

                if (nDuty < 0)   nDuty = 0;
                if (nDuty > 100) nDuty = 100;
                if (nAgg  < 1)   nAgg  = 1;
                if (nAgg  > 10)  nAgg  = 10;

                m_nDewFixedDutyPct[i]   = nDuty;
                m_nDewAggressiveness[i] = nAgg;
            }

            saveDewConfig();

            // Force an immediate auto-control update so the new settings take effect now
            if (m_bLinked)
            {
                m_nLastDewUpdateTick = 0;
                updateDewControl();
            }
        }

    } while (false);

    return nErr;
}

// ---------------------------------------------------------------------------
// uiEvent
//
// Called by TheSkyX for every GUI event while the dialog is open.
//
// "on_timer"
//     Refresh the live-data labels: port states, sensor readings, computed duty.
//
// "on_comboBox_dew{1,2}Mode_currentIndexChanged"
//     Show or hide the auto-only rows depending on selected mode.
// ---------------------------------------------------------------------------
void X2Svbony241Pro::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    if (uiex == NULL || pszEvent == NULL)
        return;

    // ── Timer tick: refresh live data ────────────────────────────────────
    if (!strcmp(pszEvent, "on_timer"))
    {
        if (!m_bLinked)
        {
            uiex->setText("label_connStatus", "Not connected");
            return;
        }

        // Refresh port state cache (single fast command)
        queryAllCircuitStates();

        // Trigger sensor read at the normal auto-dew rate (30 s throttle)
        updateDewControl();

        // Update connection / sensor status badges
        uiex->setText("label_connStatus", "Connected");
        uiex->setText("label_sensorStatus", m_bSensorValid ? "OK" : "Read error");

        // Environmental sensor labels
        char szBuf[64];
        if (m_bSensorValid)
        {
            snprintf(szBuf, sizeof(szBuf), "%.1f °C", m_dAmbientTempC);
            uiex->setText("label_ambientTemp", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.1f %%", m_dAmbientHumidityPct);
            uiex->setText("label_humidity", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.1f °C", m_dDewPointC);
            uiex->setText("label_dewPoint", szBuf);
        }
        else
        {
            uiex->setText("label_ambientTemp", "—");
            uiex->setText("label_humidity",    "—");
            uiex->setText("label_dewPoint",    "—");
        }

        // DS18B20 — read live (one command, ~200ms with sleep)
        double val = 0.0;
        if (cmdReadDS18B20Temp(val) == SB_OK)
        {
            snprintf(szBuf, sizeof(szBuf), "%.1f °C", val);
            uiex->setText("label_lensTemp", szBuf);
        }

        // INA219 voltage and current
        if (cmdReadVoltage(val) == SB_OK)
        {
            snprintf(szBuf, sizeof(szBuf), "%.2f V", val);
            uiex->setText("label_voltage", szBuf);
        }
        if (cmdReadCurrent(val) == SB_OK)
        {
            snprintf(szBuf, sizeof(szBuf), "%.3f A", val);
            uiex->setText("label_current", szBuf);
        }

        // DC port on/off states
        const char* kDCLabels[] = { "label_dc1","label_dc2","label_dc3","label_dc4" };
        for (int i = 0; i < 4; ++i)
            uiex->setText(kDCLabels[i], m_bCircuitState[i] ? "ON" : "OFF");

        uiex->setText("label_usb", m_bCircuitState[6] ? "ON" : "OFF");

        if (m_bCircuitState[7])
        {
            snprintf(szBuf, sizeof(szBuf), "%.1f V", m_dRegulatedVoltageV);
            uiex->setText("label_regulated", szBuf);
        }
        else
        {
            uiex->setText("label_regulated", "OFF");
        }

        // Auto dew computed duty labels (reflect current algorithm output)
        for (int i = 0; i < 2; ++i)
        {
            const char* autoDutyLabel = (i == 0) ? "label_dew1AutoDuty" : "label_dew2AutoDuty";
            if (m_eDewMode[i] == DEW_MODE_AUTO)
            {
                if (m_bSensorValid)
                {
                    snprintf(szBuf, sizeof(szBuf), "%d %%", calcAutoDutyPct(i));
                    uiex->setText(autoDutyLabel, szBuf);
                }
                else
                {
                    // Show fallback description so the user knows what the driver is doing
                    uiex->setText(autoDutyLabel,
                        (m_eDewFallback[i] == DEW_FALLBACK_ON) ? "fallback ON" : "fallback OFF");
                }
            }
        }

        return;
    }

    // ── Mode combo changed: toggle auto-only row visibility ───────────────
    if (!strcmp(pszEvent, "on_comboBox_dew1Mode_currentIndexChanged"))
    {
        bool bAuto = (uiex->currentIndex("comboBox_dew1Mode") == 1);
        setDewAutoRowsVisible(uiex, 0, bAuto);
        return;
    }

    if (!strcmp(pszEvent, "on_comboBox_dew2Mode_currentIndexChanged"))
    {
        bool bAuto = (uiex->currentIndex("comboBox_dew2Mode") == 1);
        setDewAutoRowsVisible(uiex, 1, bAuto);
        return;
    }
}
