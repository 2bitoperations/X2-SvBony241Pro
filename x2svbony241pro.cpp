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
    , m_dDS18B20TempC       (0.0)
    , m_dSHT40TempC         (0.0)
    , m_dSHT40HumidityPct   (0.0)
    , m_dRegulatedVoltageV  (kDefaultRegulatedVoltageV)
{
    (void)pszDisplayName;   // display name is managed by TheSkyX

    for (int i = 0; i < X2_NUM_CIRCUITS; ++i)
        m_bCircuitState[i] = false;

    m_nDewDutyPct[0] = kDefaultDewDutyPct;
    m_nDewDutyPct[1] = kDefaultDewDutyPct;
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
    uint8_t value   = bOn ? onValueForCircuit(nIndex) : static_cast<uint8_t>(0x00);

    int nErr = cmdSetPort(portIdx, value);
    if (nErr != SB_OK)
        return nErr;

    m_bCircuitState[nIndex] = bOn;
    return SB_OK;
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
// Dew heaters:     encoded duty cycle byte (0x01–0xFF) from m_nDewDutyPct[]
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
        double duty   = static_cast<double>(m_nDewDutyPct[dutyIdx]);
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
//   m_nDewDutyPct[]           (current dew heater duty %, if channel is on)
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

    // Dew heater 1 → circuit 4;  raw byte encodes duty%: pct = byte/255.0×100
    int duty0 = static_cast<int>(floor(static_cast<double>(pState10[8]) / 255.0 * 100.0 + 0.5));
    m_bCircuitState[4] = (duty0 > 0);
    if (m_bCircuitState[4])
        m_nDewDutyPct[0] = duty0;

    // Dew heater 2 → circuit 5
    int duty1 = static_cast<int>(floor(static_cast<double>(pState10[9]) / 255.0 * 100.0 + 0.5));
    m_bCircuitState[5] = (duty1 > 0);
    if (m_bCircuitState[5])
        m_nDewDutyPct[1] = duty1;
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
