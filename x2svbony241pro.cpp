#include "x2svbony241pro.h"

#include <string.h>
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <time.h>
#include <stdlib.h>

#if defined(SB_LINUX_BUILD) || defined(SB_MACOSX_BUILD)
#  include <fcntl.h>
#  include <unistd.h>
#  include <sys/ioctl.h>
#  include <termios.h>
#  include <poll.h>
#endif


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

static const int     kPostWriteSleepMs      = 100;  // post-write settling (match INDI driver)
static const int     kReadTimeoutMs         = 500;  // max wait for response bytes
static const int     kBootDrainInitSleepMs  = 500;  // initial wait for ESP32 boot (match INDI)
static const int     kBootDrainRetries      = 6;    // max drain iterations
static const int     kBootDrainSleepMs      = 30;   // sleep between drain reads
static const int     kBootDrainMaxMs        = 2000; // hard wall-clock cap (match INDI's ~6 s budget)
// Timeout passed to readFile() during the drain phase.  This MUST be short —
// readFile() blocks for up to dwTimeOut ms waiting for bytes.  The default is
// 1000 ms, which means the wall-clock cap cannot fire until AFTER readFile()
// returns, adding up to 1000 ms per iteration and making the cap useless.
// 50 ms is enough to drain whatever arrived during kBootDrainSleepMs.
static const int     kBootDrainReadTimeoutMs = 50;
static const int     kPowerCycleSleepMs     = 1000;

// Default analogue levels used when toggling an analogue channel ON for the first time
static const int     kDefaultDewDutyPct         = 50;    // 50 % PWM duty cycle
static const double  kDefaultRegulatedVoltageV  = 12.0;  // 12 V

// Auto dew-point control
static const int           kDefaultDewAggressiveness  = 5;           // mid-range
static const DewHeaterMode kDefaultDewMode             = DEW_MODE_MANUAL;
static const DewFallback   kDefaultDewFallback         = DEW_FALLBACK_ON;
static const int           kDewUpdateIntervalMs        = 30000;       // 30 s background sensor poll rate
static const int           kDialogPollIntervalMs       = 5000;        //  5 s poll rate while dialog is open
static const int           kSensorWarmupMs             = 15000; // ESP32 needs ~15s after reset to init sensors

// Ini section key for persisted dew settings
static const char* kIniSection = "SV241Pro";

// Ini keys for serial port selection
static const char* kIniKeyPortName  = "PortName";
static const char* kDefaultPortName = "No port selected";

// Debug level constants (also used as comboBox index in the UI)
static const int kDebugOff      = 0;
static const int kDebugErrors   = 1;
static const int kDebugCommands = 2;
static const int kDebugFullIO   = 3;

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
    , m_nDebugLevel         (kDebugOff)
    , m_dPowerW             (0.0)
    , m_dVoltageV           (0.0)
    , m_dCurrentA           (0.0)
    , m_dRegulatedVoltageV  (kDefaultRegulatedVoltageV)
    , m_dAmbientTempC       (0.0)
    , m_dAmbientHumidityPct (0.0)
    , m_dDewPointC          (0.0)
    , m_dHubTempC           (0.0)
    , m_bSensorValid        (false)
    , m_dLensTempC          (0.0)
    , m_bLensTempValid      (false)
    , m_nLastDewUpdateTick      (0)
    , m_nLastDialogSensorTick  (0)
    , m_nResetTimeTick          (0)
    // NOTE: initializer list order matches member declaration order in the header.
    // C++ initialises in declaration order, not initialiser-list order; keeping
    // them consistent prevents subtle bugs if a later field ever depends on an earlier one.
    , m_bRestoreOnConnect       (false)
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
        m_nLastSentDutyPct[i]   = -1;
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
// logDebug
//
// Writes a formatted message to:
//   1. The TheSkyX Communication Log window (via LoggerInterface::out) if
//      m_nDebugLevel >= minLevel.
//   2. A plain-text file (append mode, one fopen/fclose per call) at:
//        $HOME/TheSkyX/x2svbony241pro.log   (preferred)
//        /tmp/x2svbony241pro.log             (fallback if HOME is unset)
//      Level-1 (Error) messages are always written to the file, regardless of
//      m_nDebugLevel.  Levels 2 and 3 are gated by m_nDebugLevel as usual.
//
// This means connection failures and hard errors produce a persistent record
// even when the user has not opened the TSX Communication Log window.
//
// minLevel: 1=Errors, 2=Commands, 3=Full I/O.
// ---------------------------------------------------------------------------
void X2Svbony241Pro::logDebug(int minLevel, const char* fmt, ...) const
{
    // Decide whether this message reaches each sink.
    const bool bSendToTSX  = (m_pLogger != NULL) && (m_nDebugLevel >= minLevel);
    const bool bSendToFile = (minLevel == 1) || (m_nDebugLevel >= minLevel);

    if (!bSendToTSX && !bSendToFile)
        return;

    // Format the message body.
    char szBuf[384];   // large enough for the dew-config summary line (~180 chars)
    va_list args;
    va_start(args, fmt);
    vsnprintf(szBuf, sizeof(szBuf), fmt, args);
    va_end(args);

    // --- TheSkyX Communication Log window ---
    if (bSendToTSX)
        m_pLogger->out(szBuf);

    // --- Persistent file log ---
    if (bSendToFile)
    {
        // Build log path: $HOME/TheSkyX/x2svbony241pro.log, or /tmp fallback.
        char szLogPath[512];
        const char* pszHome = getenv("HOME");
        if (pszHome && pszHome[0] != '\0')
            snprintf(szLogPath, sizeof(szLogPath),
                     "%s/TheSkyX/x2svbony241pro.log", pszHome);
        else
            snprintf(szLogPath, sizeof(szLogPath),
                     "/tmp/x2svbony241pro.log");

        // Build a human-readable local timestamp.
        char szTime[32];
        time_t now = time(NULL);
        struct tm* pTm = localtime(&now);
        if (pTm)
            strftime(szTime, sizeof(szTime), "%Y-%m-%d %H:%M:%S", pTm);
        else
            snprintf(szTime, sizeof(szTime), "0000-00-00 00:00:00");

        FILE* pf = fopen(szLogPath, "a");
        if (pf)
        {
            fprintf(pf, "[%s] %s\n", szTime, szBuf);
            fclose(pf);
        }
    }
}

// ---------------------------------------------------------------------------
// DriverRootInterface
// ---------------------------------------------------------------------------

int X2Svbony241Pro::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = m_pLogger;
    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);
    else if (!strcmp(pszName, CircuitLabelsInterface_Name))
        *ppVal = dynamic_cast<CircuitLabelsInterface*>(this);
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);

    return SB_OK;   // return SB_OK even for unknown interfaces (*ppVal == NULL)
}

// ---------------------------------------------------------------------------
// SerialPortParams2Interface
// ---------------------------------------------------------------------------

void X2Svbony241Pro::getPortName(std::string& sPortName) const
{
    sPortName = kDefaultPortName;
    if (m_pIniUtil)
    {
        char szPort[256];
        m_pIniUtil->readString(kIniSection, kIniKeyPortName,
                               kDefaultPortName, szPort, sizeof(szPort));
        sPortName = szPort;
    }
}

void X2Svbony241Pro::portName(BasicStringInterface& str) const
{
    std::string sPortName;
    getPortName(sPortName);
    str = sPortName.c_str();
}

void X2Svbony241Pro::setPortName(const char* szPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(kIniSection, kIniKeyPortName, szPort);
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

    // Capture a start tick so we can log total establishLink() elapsed time.
    // This makes it easy to correlate the log against any perceived UI freeze.
    int nLinkStartTick = m_pTickCount ? m_pTickCount->elapsed() : 0;

    // Open the USB virtual COM port at 115200 8N1, no parity, no flow control.
    // Use the port name selected by the user via SerialPortParams2Interface.
    std::string sPortName;
    getPortName(sPortName);

    logDebug(kDebugErrors, "X2Svbony241Pro: establishLink start (port=%s)", sPortName.c_str());

    // Keep m_bLinked = false throughout the drain+handshake window.
    loadDebugLevel();
    loadDewConfig();

    // -----------------------------------------------------------------------
    // Connection strategy for CH341 + ESP32
    //
    // The Linux ch341 driver asserts DTR whenever tcsetattr() configures the
    // port (which SerXInterface::open() triggers internally).  DTR asserted →
    // CH341 pulls ESP32 EN low → ESP32 held in reset → never responds.
    //
    // To work around this we open a private "control" fd BEFORE calling
    // m_pSerX->open().  Because TIOCEXCL is only set by TSX after its open(),
    // our fd is guaranteed to open successfully.  We:
    //
    //   1. Assert DTR on the ctrl fd  (ESP32 goes into reset — harmless)
    //   2. Call m_pSerX->open()       (TSX configures the port; device still
    //                                  in reset so any DTR glitch is a no-op)
    //   3. Release DTR on the ctrl fd (ESP32 starts booting; boot log flows
    //                                  into TSX's RX buffer)
    //   4. drainBootLog()             (consumes boot text through SerXInterface)
    //   5. Handshake
    //   6. Close ctrl fd with HUPCL cleared (DTR stays high; ESP32 keeps running)
    //
    // Attempt 1 tries without a reset first.  If the device is already running
    // (e.g. re-connect without unplugging) it responds immediately and we skip
    // the boot cycle entirely, avoiding any disruption to powered outputs.
    // -----------------------------------------------------------------------

    // Open the control fd while the port is free (before TSX's TIOCEXCL).
    int nCtrlFd = -1;
#if defined(SB_LINUX_BUILD) || defined(SB_MACOSX_BUILD)
    nCtrlFd = ::open(sPortName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (nCtrlFd >= 0)
    {
        struct termios tio;
        if (::tcgetattr(nCtrlFd, &tio) == 0)
        {
            tio.c_cflag &= ~HUPCL;   // don't drop DTR on close
            tio.c_cflag |=  CLOCAL;  // ignore carrier detect
            ::tcsetattr(nCtrlFd, TCSANOW, &tio);
        }
        logDebug(kDebugCommands, "X2Svbony241Pro: ctrl fd opened (fd=%d)", nCtrlFd);
    }
    else
    {
        logDebug(kDebugErrors, "X2Svbony241Pro: ctrl fd open failed (errno=%d) — no DTR control", errno);
    }
#endif

    // Helper lambdas to set DTR+RTS via the control fd.
    // The ESP32 auto-reset circuit on the SV241 Pro is cross-coupled:
    //   DTR → EN pin (active-low reset)
    //   RTS → GPIO0 pin (boot-mode select)
    // Both lines must be asserted together to pull EN low cleanly;
    // leaving RTS high (as TSX sets it) prevents DTR from fully
    // resetting the chip.  This replicates the exact esptool sequence.
#if defined(SB_LINUX_BUILD) || defined(SB_MACOSX_BUILD)
    auto setDtrRts = [&](bool bAssert) {
        if (nCtrlFd < 0) return;
        int modem = 0;
        ::ioctl(nCtrlFd, TIOCMGET, &modem);
        if (bAssert) { modem |=  TIOCM_DTR; modem |=  TIOCM_RTS; }
        else         { modem &= ~TIOCM_DTR; modem &= ~TIOCM_RTS; }
        ::ioctl(nCtrlFd, TIOCMSET, &modem);
    };
    auto closeCtrlFd = [&]() {
        if (nCtrlFd >= 0) { ::close(nCtrlFd); nCtrlFd = -1; }
    };
#else
    auto setDtrRts   = [&](bool) {};
    auto closeCtrlFd = [&]() {};
#endif

    // --- Attempt 1: try without reset ---
    // Leave DTR in its current state and see if the device is already alive.
    int nErr = m_pSerX->open(sPortName.c_str(), kBaudRate, SerXInterface::B_NOPARITY, 0);
    if (nErr != SB_OK)
    {
        logDebug(kDebugErrors, "X2Svbony241Pro: serial port open failed (err %d)", nErr);
        closeCtrlFd();
        return nErr;
    }

    // TSX's open() may have left RTS asserted (or DTR in an unknown state).
    // Release both now so the ESP32 is free to run if it was held in reset.
    setDtrRts(false);

    m_bLinked = true;
    uint8_t state10[10] = {0};
    nErr = cmdGetState(state10);

    if (nErr == SB_OK)
    {
        logDebug(kDebugErrors, "X2Svbony241Pro: handshake OK on first attempt");
        closeCtrlFd();
    }
    else
    {
        // First attempt failed.
        int nFirstFailMs = m_pTickCount ? (m_pTickCount->elapsed() - nLinkStartTick) : -1;
        logDebug(kDebugErrors,
                 "X2Svbony241Pro: first attempt failed at +%d ms (err %d) — doing ESP32 reset",
                 nFirstFailMs, nErr);
        m_bLinked = false;

        // Assert DTR+RTS together → ESP32 in reset (200ms), then release both.
        // The SV241 Pro uses the standard ESP32 auto-reset circuit:
        //   DTR → EN (active-low reset), RTS → GPIO0 (boot-mode select).
        // The transistors are cross-coupled so both lines must be asserted
        // to pull EN cleanly low.  Releasing both lets the chip boot normally.
        setDtrRts(true);

        struct timespec ts200 = { 0, 200000000L };
        ::nanosleep(&ts200, NULL);

        setDtrRts(false);

        // Record the reset time so updateDewControl() can defer sensor reads
        // until the INA219 and SHT40 are ready (~15 s after reset).
        m_nResetTimeTick = m_pTickCount ? m_pTickCount->elapsed() : 0;

        logDebug(kDebugErrors, "X2Svbony241Pro: DTR+RTS released — draining boot log");

        // Drain the boot log that will appear in TSX's RX buffer as the ESP32 boots.
        (void)drainBootLog();

        int nAfterDrainMs = m_pTickCount ? (m_pTickCount->elapsed() - nLinkStartTick) : -1;
        logDebug(kDebugErrors,
                 "X2Svbony241Pro: drain done at +%d ms — handshake attempt 2",
                 nAfterDrainMs);

        m_bLinked = true;
        memset(state10, 0, sizeof(state10));
        nErr = cmdGetState(state10);

        // Done with the ctrl fd regardless of handshake outcome.
        closeCtrlFd();

        if (nErr != SB_OK)
        {
            int nFailMs = m_pTickCount ? (m_pTickCount->elapsed() - nLinkStartTick) : -1;
            logDebug(kDebugErrors,
                     "X2Svbony241Pro: handshake failed after reset at +%d ms (err %d) — closing",
                     nFailMs, nErr);
            m_bLinked = false;
            m_pSerX->close();
            return nErr;
        }

        logDebug(kDebugErrors, "X2Svbony241Pro: handshake OK after DTR reset");
    }

    parseStateResponse(state10);

    // Restore last saved circuit states if the user has enabled that option.
    // Done after parseStateResponse() so m_bCircuitState[] reflects the device's
    // actual current state before we layer the saved state on top.
    restoreCircuitStates();

    int nTotalMs = m_pTickCount ? (m_pTickCount->elapsed() - nLinkStartTick) : -1;
    logDebug(kDebugErrors, "X2Svbony241Pro: link established (total=%d ms)", nTotalMs);

    return SB_OK;
}

int X2Svbony241Pro::terminateLink()
{
    saveDebugLevel();
    saveDewConfig();
    saveCircuitStates();

    // Clear m_bLinked and close the port under the mutex.  Any thread that is
    // blocked waiting to acquire the lock inside sendFrame() will re-check
    // m_bLinked after acquiring the lock and return ERR_COMMNOLINK rather
    // than issuing I/O on the now-closed port.
    {
        X2MutexLocker ml(m_pIOMutex);
        m_bLinked = false;
        if (m_pSerX != NULL)
            m_pSerX->close();
    }

    logDebug(kDebugCommands, "X2Svbony241Pro: link terminated");

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
        return ERR_INDEX_OUT_OF_RANGE;
    }
    str = kCircuitNames[nIndex];
    return SB_OK;
}

// ---------------------------------------------------------------------------
// CircuitLabelsInterface — lets TheSkyX display human-readable port names
// ---------------------------------------------------------------------------
int X2Svbony241Pro::circuitLabel(const int& nZeroBasedIndex, BasicStringInterface& str)
{
    if (nZeroBasedIndex < 0 || nZeroBasedIndex >= X2_NUM_CIRCUITS)
    {
        str = "Unknown";
        return ERR_INDEX_OUT_OF_RANGE;
    }
    str = kCircuitNames[nZeroBasedIndex];
    return SB_OK;
}

int X2Svbony241Pro::circuitState(const int& nIndex, bool& bOn)
{
    if (!m_bLinked)
        return ERR_COMMNOLINK;
    if (nIndex < 0 || nIndex >= X2_NUM_CIRCUITS)
        return ERR_INDEX_OUT_OF_RANGE;

    // Drive the sensor update loop on every poll call.
    // updateDewControl() reads all sensors (SHT40, INA219, DS18B20) and applies
    // auto-PWM to any heater in AUTO mode.  It is rate-limited internally so it
    // does nothing when the interval (kDewUpdateIntervalMs) has not elapsed.
    updateDewControl();

    bOn = m_bCircuitState[nIndex];
    return SB_OK;
}

int X2Svbony241Pro::setCircuitState(const int& nIndex, const bool& bOn)
{
    if (!m_bLinked)
        return ERR_COMMNOLINK;
    if (nIndex < 0 || nIndex >= X2_NUM_CIRCUITS)
        return ERR_INDEX_OUT_OF_RANGE;

    logDebug(kDebugCommands, "X2Svbony241Pro: setCircuitState circuit=%d (%s) → %s",
             nIndex, kCircuitNames[nIndex], bOn ? "ON" : "OFF");

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
        else
            logDebug(kDebugErrors, "X2Svbony241Pro: setCircuitState circuit=%d failed (err %d)",
                     nIndex, nErr);
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
    // Fast pre-check before acquiring the lock (optimistic path).
    if (!m_bLinked || m_pSerX == NULL)
        return ERR_COMMNOLINK;

    // Serialize all serial I/O through the TSX-provided mutex.
    X2MutexLocker ml(m_pIOMutex);

    // Re-check m_bLinked inside the lock.  terminateLink() clears m_bLinked
    // and closes the port while holding the same mutex, so by the time we
    // acquire the lock here the flag is guaranteed to reflect the actual state.
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
        logDebug(kDebugErrors, "X2Svbony241Pro: writeFile error %d", nErr);
        return nErr;
    }
    if ((int)nWritten != nFrameLen)
    {
        logDebug(kDebugErrors, "X2Svbony241Pro: short write %lu/%d", nWritten, nFrameLen);
        return ERR_CMDFAILED;
    }

    // Force bytes out of the OS TX buffer — equivalent to INDI's tcdrain().
    // Without this, bytes may sit in the kernel buffer and never reach the device.
    m_pSerX->flushTx();

    if (m_nDebugLevel >= kDebugFullIO)
    {
        char szHex[64];
        int pos = 0;
        for (int i = 0; i < nFrameLen && pos < (int)sizeof(szHex) - 4; ++i)
            pos += snprintf(szHex + pos, sizeof(szHex) - pos, "%02X ", frame[i]);
        logDebug(kDebugFullIO, "X2Svbony241Pro: TX [%s]", szHex);
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

    if (m_nDebugLevel >= kDebugFullIO)
    {
        char szHex[64];
        int pos = 0;
        for (int i = 0; i < nFullResLen && pos < (int)sizeof(szHex) - 4; ++i)
            pos += snprintf(szHex + pos, sizeof(szHex) - pos, "%02X ", resBuf[i]);
        logDebug(kDebugFullIO, "X2Svbony241Pro: RX [%s]", szHex);
    }

    // Verify response checksum
    uint8_t csExpected = calcChecksum(resBuf, nFullResLen - 1);
    if (resBuf[nFullResLen - 1] != csExpected)
    {
        logDebug(kDebugErrors, "X2Svbony241Pro: checksum mismatch (got %02X expected %02X)",
                 resBuf[nFullResLen - 1], csExpected);
        return ERR_CMDFAILED;
    }

    // Check device status byte (response[2] == 0xAA means device rejected the command)
    if (resBuf[2] == kStatusFailure)
    {
        logDebug(kDebugErrors, "X2Svbony241Pro: device returned failure status (0xAA)");
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
// Retries every 1 ms up to kReadTimeoutMs total *elapsed* time.
//
// Use waitForBytesRx() + readFile() — the correct TSX pattern.
// waitForBytesRx() blocks efficiently for up to kReadTimeoutMs ms waiting
// for at least one byte to arrive, then readFile() collects whatever is
// buffered.  This avoids fighting with SerXInterface's internal timeout
// handling in readFile() (which may ignore small dwTimeOut values).
// ---------------------------------------------------------------------------
int X2Svbony241Pro::readFrame(int nExpectedBytes, uint8_t* pBufOut)
{
    int nRead = 0;

    while (nRead < nExpectedBytes)
    {
        // Wait for at least one more byte within our deadline.
        int nErr = m_pSerX->waitForBytesRx(1, kReadTimeoutMs);
        if (nErr != SB_OK)
        {
            logDebug(kDebugErrors,
                     "X2Svbony241Pro: waitForBytesRx timeout/error %d (%d/%d bytes)",
                     nErr, nRead, nExpectedBytes);
            return ERR_TXTIMEOUT;
        }

        // Drain whatever arrived — readFile with a short timeout to avoid blocking.
        unsigned long nGot = 0;
        nErr = m_pSerX->readFile(pBufOut + nRead,
                                 (unsigned long)(nExpectedBytes - nRead),
                                 nGot,
                                 (unsigned long)kReadTimeoutMs);
        if (nErr != SB_OK)
        {
            logDebug(kDebugErrors, "X2Svbony241Pro: readFile error %d", nErr);
            return nErr;
        }

        nRead += (int)nGot;
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
// Strategy: wait kBootDrainInitSleepMs for the boot to start, then read and
// discard until we see kQuietTarget consecutive quiet reads (no bytes
// arriving), or until the hard wall-clock cap kBootDrainMaxMs elapses.
//
// CRITICAL: readFile() blocks for up to dwTimeOut ms waiting for bytes.
// The SerXInterface default is 1000 ms.  Each drain loop iteration calls
// readFile(), so without an explicit short timeout the effective per-iteration
// cost is kBootDrainSleepMs + 1000 ms = ~1030 ms, and the wall-clock cap
// (checked only at the top of the loop, before readFile) cannot fire until
// after readFile() returns — adding up to 1000 ms of extra latency per
// iteration.  We therefore always pass kBootDrainReadTimeoutMs (50 ms) to
// readFile() so the cap is effective and the total drain time is bounded.
//
// The wall-clock cap is the critical safety net: without it a continuously
// chatty device (wrong port, modem, still-booting device with a long log)
// would hold the TSX UI thread blocked for up to
//   kBootDrainInitSleepMs + kBootDrainRetries × (kBootDrainSleepMs + kBootDrainReadTimeoutMs)
// milliseconds, making TheSkyX appear frozen.  The cap plus the short
// readFile timeout ensures the drain phase never takes meaningfully more
// than kBootDrainMaxMs ms in total.
//
// Note: the INDI driver uses ioctl(TIOCMSET) to explicitly clear RTS and DTR
// before draining.  SerXInterface does not expose ioctl, so we rely on the
// USB-CDC chip's built-in auto-reset behavior, which is present on standard
// ESP32 DevKit hardware.
int X2Svbony241Pro::drainBootLog()
{
    // No mutex needed here: drainBootLog() is only ever called from
    // establishLink() while m_bLinked is still false.  No other thread can
    // reach sendFrame() (which also requires m_bLinked == true), so there is
    // no concurrent access to the serial port at this point.  Taking the mutex
    // here would hold it for up to ~1 second while sleeping, unnecessarily
    // blocking any thread that tries to call isLinked() or other fast paths.

    // Record the tick at which we enter drain so we can enforce a hard
    // wall-clock cap (kBootDrainMaxMs).  This prevents a continuously chatty
    // device (wrong port, modem, still-booting ESP32 with a very long boot log)
    // from keeping the TSX UI thread blocked indefinitely.
    int nDrainStartTick = m_pTickCount ? m_pTickCount->elapsed() : 0;

    logDebug(kDebugCommands, "X2Svbony241Pro: drainBootLog start (initSleep=%d ms, maxMs=%d)",
             kBootDrainInitSleepMs, kBootDrainMaxMs);

    // Give the ESP32 time to complete its reset and begin emitting the boot log.
    if (m_pSleeper)
        m_pSleeper->sleep(kBootDrainInitSleepMs);

    uint8_t discard[64];
    int nQuietRounds  = 0;
    int nTotalRead    = 0;
    const int kQuietTarget = 3;   // 3 consecutive empty reads → log is done

    for (int i = 0; i < kBootDrainRetries && nQuietRounds < kQuietTarget; ++i)
    {
        // Hard wall-clock cap: stop reading even if we haven't hit kQuietTarget.
        // This bounds the entire drain phase to kBootDrainMaxMs regardless of
        // how much data the device sends, preventing an indefinite UI-thread stall
        // when the wrong port is selected or the device has an unusually long boot log.
        if (m_pTickCount)
        {
            int nElapsed = m_pTickCount->elapsed() - nDrainStartTick;
            if (nElapsed >= kBootDrainMaxMs)
            {
                logDebug(kDebugErrors,
                         "X2Svbony241Pro: drainBootLog wall-clock cap hit after %d ms "
                         "(i=%d, quietRounds=%d, bytesRead=%d) — continuing to handshake",
                         nElapsed, i, nQuietRounds, nTotalRead);
                break;
            }
        }

        if (m_pSleeper)
            m_pSleeper->sleep(kBootDrainSleepMs);

        unsigned long nRead = 0;
        // Pass kBootDrainReadTimeoutMs explicitly — do NOT rely on the 1000 ms
        // default.  Without this, readFile blocks for up to 1000 ms per call,
        // which completely defeats the kBootDrainMaxMs wall-clock cap (the cap
        // is only checked at the top of the loop, so each readFile call can add
        // a full second of latency before the cap fires on the next iteration).
        m_pSerX->readFile(discard, sizeof(discard), nRead, kBootDrainReadTimeoutMs);
        nTotalRead += (int)nRead;

        if (nRead == 0)
            ++nQuietRounds;
        else
            nQuietRounds = 0;   // reset quiet counter — still receiving boot bytes
    }

    // Final flush of any residual bytes in the kernel buffer.
    m_pSerX->purgeTxRx();

    int nTotalElapsed = m_pTickCount ? (m_pTickCount->elapsed() - nDrainStartTick) : -1;
    logDebug(kDebugCommands,
             "X2Svbony241Pro: drainBootLog done (elapsed=%d ms, bytesDiscarded=%d, quietRounds=%d)",
             nTotalElapsed, nTotalRead, nQuietRounds);

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
    logDebug(kDebugCommands, "X2Svbony241Pro: cmdSetPort port=0x%02X val=0x%02X", portIndex, value);
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
    if (!m_bLinked || m_pSerX == NULL)
        return ERR_COMMNOLINK;

    // The power-cycle is a two-step atomic sequence: "all off", sleep 1 s,
    // "all on".  We must hold the mutex across both steps and the sleep so
    // that no other command can be injected between them (which would restart
    // one or more outputs before the full cycle completes).
    //
    // This is the one place in the driver where the mutex is intentionally
    // held while sleeping.  The sleep is short (1 s) and power-cycle is a
    // user-initiated, infrequent operation, so blocking other callers is
    // acceptable.
    X2MutexLocker ml(m_pIOMutex);

    // Re-check inside the lock (mirrors the pattern in sendFrame).
    if (!m_bLinked || m_pSerX == NULL)
        return ERR_COMMNOLINK;

    // The two helper lambdas below duplicate a small subset of sendFrame's
    // logic without re-locking the already-held mutex.  We build and send the
    // frame manually, then read the response.

    auto sendRaw = [&](const uint8_t* pCmd, int nCmdLen) -> int {
        uint8_t frame[16];
        int nFrameLen = 0;
        buildFrame(pCmd, nCmdLen, frame, nFrameLen);

        m_pSerX->purgeTxRx();

        unsigned long nWritten = 0;
        int nErr = m_pSerX->writeFile(frame, (unsigned long)nFrameLen, nWritten);
        if (nErr != SB_OK) return nErr;
        if ((int)nWritten != nFrameLen) return ERR_CMDFAILED;

        if (m_pSleeper)
            m_pSleeper->sleep(kPostWriteSleepMs);

        // Response: header(1) + data_len(1) + status(1) + 2 payload + checksum(1)
        const int kResLen = 6;
        uint8_t res[kResLen] = {0};
        nErr = readFrame(kResLen, res);
        if (nErr != SB_OK) return nErr;

        m_pSerX->purgeTxRx();

        uint8_t csExpected = calcChecksum(res, kResLen - 1);
        if (res[kResLen - 1] != csExpected) return ERR_CMDFAILED;
        if (res[2] == kStatusFailure)       return ERR_CMDFAILED;
        return SB_OK;
    };

    logDebug(kDebugCommands, "X2Svbony241Pro: cmdPowerCycle — all outputs off");

    uint8_t cmdOff[2] = { 0xFF, 0xFF };
    int nErr = sendRaw(cmdOff, 2);
    if (nErr != SB_OK)
    {
        logDebug(kDebugErrors, "X2Svbony241Pro: cmdPowerCycle step 1 (off) failed (err %d)", nErr);
        return nErr;
    }

    if (m_pSleeper)
        m_pSleeper->sleep(kPowerCycleSleepMs);

    logDebug(kDebugCommands, "X2Svbony241Pro: cmdPowerCycle — all outputs back on");

    uint8_t cmdOn[2] = { 0xFE, 0xFE };
    nErr = sendRaw(cmdOn, 2);
    if (nErr != SB_OK)
        logDebug(kDebugErrors, "X2Svbony241Pro: cmdPowerCycle step 2 (on) failed (err %d)", nErr);
    return nErr;
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
// The ESP32 firmware encodes DS18B20 temperature as a 4-byte big-endian uint32
// using: wire_value = (tempC + 255.5) * 100, i.e. tempC = raw32/100.0 - 255.5.
// Same firmware encoding pattern as SHT40 (offset 254.0) and INA219.
// Confirmed by live serial capture: raw32=27644 (0x00006BFC) -> 20.94 deg C.
// Returns a value outside [-55, 125] when the probe is absent/erroring.
// ---------------------------------------------------------------------------
double X2Svbony241Pro::decodeDS18B20Temp(const uint8_t* pData4)
{
    uint32_t raw32 = (static_cast<uint32_t>(pData4[0]) << 24)
                   | (static_cast<uint32_t>(pData4[1]) << 16)
                   | (static_cast<uint32_t>(pData4[2]) <<  8)
                   |  static_cast<uint32_t>(pData4[3]);
    double tempC = static_cast<double>(raw32) / 100.0 - 255.5;
    return floor(tempC * 100.0 + 0.5) / 100.0;
}

// ---------------------------------------------------------------------------
// decodeSHT40Temp  (cmd 0x05)
//
// The ESP32 firmware encodes SHT40 temperature as a 4-byte big-endian uint32
// using: wire_value = (tempC + 254.0) * 100, i.e. tempC = raw32/100.0 - 254.0.
// It does NOT send raw SHT40 ADC counts — the SHT40 datasheet formula does
// not apply here.  Confirmed by live serial capture: raw32=28190 → 27.9°C,
// consistent with a warm room.
// ---------------------------------------------------------------------------
double X2Svbony241Pro::decodeSHT40Temp(const uint8_t* pData4)
{
    uint32_t raw32 = (static_cast<uint32_t>(pData4[0]) << 24)
                   | (static_cast<uint32_t>(pData4[1]) << 16)
                   | (static_cast<uint32_t>(pData4[2]) <<  8)
                   |  static_cast<uint32_t>(pData4[3]);
    double tempC = static_cast<double>(raw32) / 100.0 - 254.0;
    return floor(tempC * 10.0 + 0.5) / 10.0;
}

// ---------------------------------------------------------------------------
// decodeSHT40Humidity  (cmd 0x06)
//
// The ESP32 firmware encodes SHT40 humidity as a 4-byte big-endian uint32
// using: wire_value = (rh% + 254.0) * 100, i.e. rh = raw32/100.0 - 254.0.
// Same encoding as temperature (same firmware formula for both channels).
// Confirmed by live serial capture: raw32=32552 → 71.5% RH, consistent with
// a very humid room (user-reported 70%+). Clamped to [0, 100].
// ---------------------------------------------------------------------------
double X2Svbony241Pro::decodeSHT40Humidity(const uint8_t* pData4)
{
    uint32_t raw32 = (static_cast<uint32_t>(pData4[0]) << 24)
                   | (static_cast<uint32_t>(pData4[1]) << 16)
                   | (static_cast<uint32_t>(pData4[2]) <<  8)
                   |  static_cast<uint32_t>(pData4[3]);
    double rh = static_cast<double>(raw32) / 100.0 - 254.0;
    if (rh < 0.0)   rh = 0.0;
    if (rh > 100.0) rh = 100.0;
    return floor(rh * 10.0 + 0.5) / 10.0;
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
    // PRECONDITION: pState10 points to at least 10 valid bytes as returned by
    // cmdGetState().  All callers in this file guarantee this — do not call
    // with a shorter buffer.

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
        int fallbackDuty;
        const char* fallbackStr;
        if (m_eDewFallback[heaterIdx] == DEW_FALLBACK_ON)
        {
            fallbackDuty = m_nDewFixedDutyPct[heaterIdx];
            fallbackStr  = "FALLBACK_ON";
        }
        else
        {
            fallbackDuty = 0;
            fallbackStr  = "FALLBACK_OFF";
        }
        logDebug(kDebugErrors,
                 "X2Svbony241Pro: auto duty heater %d: sensor invalid → fallback %s duty=%d%%",
                 heaterIdx + 1, fallbackStr, fallbackDuty);
        return fallbackDuty;
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

    logDebug(kDebugCommands,
             "X2Svbony241Pro: auto duty heater %d: ambient=%.1f°C (%s) dew=%.1f°C dpd=%.1f°C agg=%d window=%.1f°C → duty=%d%%",
             heaterIdx + 1, m_dAmbientTempC,
             m_bLensTempValid ? "DS18B20" : "SHT40 fallback",
             m_dDewPointC, dpd, agg, window, duty);

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
int X2Svbony241Pro::updateDewControl(bool bForce)
{
    if (!m_bLinked)
        return ERR_COMMNOLINK;

    // Skip sensor reads during post-reset sensor warmup period.
    // The ESP32 returns sentinel values for ~15s after a hardware reset.
    // Snapshot m_nResetTimeTick once — terminateLink() can clear it from another
    // thread, and a snapshot avoids a TOCTOU race.
    {
        int nResetTick = m_nResetTimeTick;  // thread-safe snapshot
        if (nResetTick > 0 && m_pTickCount)
        {
            int msSinceReset = m_pTickCount->elapsed() - nResetTick;
            if (msSinceReset < 0) msSinceReset = kSensorWarmupMs;  // wrap-safe
            if (msSinceReset < kSensorWarmupMs)
            {
                logDebug(kDebugCommands,
                         "X2Svbony241Pro: sensor warmup in progress (%d ms / %d ms)",
                         msSinceReset, kSensorWarmupMs);
                return SB_OK;  // don't stamp m_nLastDewUpdateTick; allow retry on next poll
            }
        }
    }

    // Rate-limit: skip if updated recently — unless bForce is set.
    // bForce is used by the dialog timer (5 s) to bypass the slower background
    // rate (30 s) without resetting m_nLastDewUpdateTick, so both clocks coexist.
    // elapsed() returns int (ms since TSX started).  Use int throughout to
    // avoid signed/unsigned comparison pitfalls and to match the interface type.
    int now = m_pTickCount ? m_pTickCount->elapsed() : 0;
    if (!bForce && m_nLastDewUpdateTick != 0)
    {
        int sinceUpdate = now - m_nLastDewUpdateTick;
        if (sinceUpdate < 0) sinceUpdate = kDewUpdateIntervalMs;  // wrap-safe
        if (sinceUpdate < kDewUpdateIntervalMs)
            return SB_OK;
    }

    // --- Read all sensors ---
    // Read order: DS18B20 first so its value is available when we compute the
    // corrected ambient temperature and RH from the SHT40 readings.
    //
    // SHT40 is mounted inside the device and runs ~5-6°C warmer than true
    // ambient due to self-heating.  DS18B20 is the external probe and gives
    // the accurate ambient temperature.  We therefore:
    //   • Use DS18B20 as m_dAmbientTempC (fallback: SHT40 when probe absent).
    //   • Dew point is calculated from ambient T + SHT40 RH.
    //   • SHT40 RH is used directly — empirically it reads approximately
    //     correct ambient RH despite the sensor being warmer than ambient.

    // DS18B20 lens temperature — non-fatal; probe may not be installed.
    // Valid range: -55°C to +125°C per DS18B20 spec.
    {
        double lensT = 0.0;
        if (cmdReadDS18B20Temp(lensT) == SB_OK &&
            lensT >= -55.0 && lensT <= 125.0)
        {
            m_dLensTempC    = lensT;
            m_bLensTempValid = true;
        }
        else
        {
            m_bLensTempValid = false;
        }
    }

    // SHT40 (internal sensor — provides T+RH for dew point and ambient humidity).
    // INA219 (voltage + current) is read further below.
    // Stamp the rate-limit timestamp only on success so transient failures
    // cause a retry on the next poll rather than waiting the full interval.

    double tempC    = 0.0;
    double humidity = 0.0;
    int nErrT = cmdReadSHT40Temp(tempC);
    int nErrH = cmdReadSHT40Humidity(humidity);

    if (nErrT == SB_OK && nErrH == SB_OK &&
        humidity >= 0.0 && humidity <= 100.0)
    {
        // Reject sentinel values that indicate firmware hasn't initialized sensors yet.
        // Sentinel: raw==25600 → decoded as exactly 2.0°C / 2.0% RH.
        if (tempC < 3.0 && humidity < 3.0)
        {
            m_bSensorValid = false;
            logDebug(kDebugErrors, "X2Svbony241Pro: SHT40 sentinel values — skipping (warmup?)");
            return SB_OK;   // don't stamp timestamp; allow retry on next poll
        }

        // Store SHT40 raw temperature for display ("Hub temp") — self-heated,
        // not used by the dew algorithm, but useful diagnostics for the user.
        m_dHubTempC = tempC;

        // Ambient temperature: prefer DS18B20 (external, accurate); fall back to
        // SHT40 when the probe is absent or out of range.
        m_dAmbientTempC = m_bLensTempValid ? m_dLensTempC : tempC;

        // SHT40 RH is used as-is — empirically it reads ambient RH correctly.
        m_dAmbientHumidityPct = humidity;

        // Dew point from ambient temperature + SHT40 RH.
        m_dDewPointC = calcDewPoint(m_dAmbientTempC, humidity);

        m_bSensorValid       = true;
        m_nLastDewUpdateTick = now;

        logDebug(kDebugCommands,
                 "X2Svbony241Pro: sensor update — "
                 "SHT40 hub %.1f C / %.0f%% RH  |  "
                 "DS18B20 ambient=%.1f C  |  dewpoint=%.1f C",
                 m_dHubTempC, humidity,
                 m_dAmbientTempC, m_dDewPointC);
    }
    else
    {
        m_bSensorValid = false;
        logDebug(kDebugErrors,
                 "X2Svbony241Pro: SHT40 read failed (errT=%d errH=%d)",
                 nErrT, nErrH);
    }

    // INA219 — non-fatal if unavailable; update cache on success.
    {
        double v = 0.0, a = 0.0, p = 0.0;
        if (cmdReadVoltage(v) == SB_OK && v > 1.0)  m_dVoltageV = v;
        if (cmdReadCurrent(a) == SB_OK)              m_dCurrentA = a;
        if (cmdReadPower(p)   == SB_OK)              m_dPowerW   = p;
        logDebug(kDebugCommands,
                 "X2Svbony241Pro: INA219 — %.2f V  %.3f A  %.2f W",
                 m_dVoltageV, m_dCurrentA, m_dPowerW);
    }

    // --- Apply auto PWM to each heater that is on and in auto mode ---
    for (int i = 0; i < 2; ++i)
    {
        if (m_eDewMode[i] != DEW_MODE_AUTO)
            continue;
        if (!m_bCircuitState[4 + i])
            continue;

        int duty = calcAutoDutyPct(i);

        uint8_t raw = static_cast<uint8_t>(
            floor(255.0 * static_cast<double>(duty) / 100.0 + 0.5));

        logDebug(kDebugCommands,
                 "X2Svbony241Pro: dew%d PWM send: duty=%d%% raw=0x%02X V=%.2fV I=%.3fA",
                 i + 1, duty, raw, m_dVoltageV, m_dCurrentA);

        // Send the new PWM value to the device.
        // Non-fatal: if the command fails we keep the cached state unchanged
        // and will retry on the next poll cycle.
        int nErr = cmdSetPort(kPortIndex[4 + i], raw);
        if (nErr != SB_OK)
        {
            logDebug(kDebugErrors,
                     "X2Svbony241Pro: dew heater %d auto PWM command failed (err %d)", i + 1, nErr);
        }
        else
        {
            m_nLastSentDutyPct[i] = duty;
            if (duty == 0)
            {
                // Algorithm says fully off — reflect that in the circuit cache so
                // TheSkyX sees the heater as off.
                m_bCircuitState[4 + i] = false;
            }
        }
    }

    return SB_OK;
}

// ---------------------------------------------------------------------------
// saveDebugLevel / loadDebugLevel
// ---------------------------------------------------------------------------
void X2Svbony241Pro::saveDebugLevel()
{
    if (m_pIniUtil == NULL)
        return;
    char szKey[32];
    snprintf(szKey, sizeof(szKey), "DebugLevel_%d", m_nInstanceIndex);
    m_pIniUtil->writeInt(kIniSection, szKey, m_nDebugLevel);
}

void X2Svbony241Pro::loadDebugLevel()
{
    if (m_pIniUtil == NULL)
        return;
    char szKey[32];
    snprintf(szKey, sizeof(szKey), "DebugLevel_%d", m_nInstanceIndex);
    int nVal = kDebugOff;
    nVal = m_pIniUtil->readInt(kIniSection, szKey, kDebugOff);
    if (nVal < kDebugOff)    nVal = kDebugOff;
    if (nVal > kDebugFullIO) nVal = kDebugFullIO;
    m_nDebugLevel = nVal;
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

    // Regulated voltage — stored as integer tenths of a volt (e.g. 120 = 12.0 V)
    snprintf(szKey, sizeof(szKey), "RegulatedVoltageTenths_%d", m_nInstanceIndex);
    m_pIniUtil->writeInt(kIniSection, szKey,
                         static_cast<int>(m_dRegulatedVoltageV * 10.0 + 0.5));
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
        nVal = m_pIniUtil->readInt(kIniSection, szKey,
                            static_cast<int>(kDefaultDewMode));
        m_eDewMode[i] = (nVal == static_cast<int>(DEW_MODE_AUTO))
                        ? DEW_MODE_AUTO : DEW_MODE_MANUAL;

        snprintf(szKey, sizeof(szKey), "DewFixedDuty%d_%d", i, m_nInstanceIndex);
        nVal = m_pIniUtil->readInt(kIniSection, szKey, kDefaultDewDutyPct);
        if (nVal < 0)   nVal = 0;
        if (nVal > 100) nVal = 100;
        m_nDewFixedDutyPct[i] = nVal;

        snprintf(szKey, sizeof(szKey), "DewAggressiveness%d_%d", i, m_nInstanceIndex);
        nVal = m_pIniUtil->readInt(kIniSection, szKey, kDefaultDewAggressiveness);
        if (nVal < 1)  nVal = 1;
        if (nVal > 10) nVal = 10;
        m_nDewAggressiveness[i] = nVal;

        snprintf(szKey, sizeof(szKey), "DewFallback%d_%d", i, m_nInstanceIndex);
        nVal = m_pIniUtil->readInt(kIniSection, szKey,
                            static_cast<int>(kDefaultDewFallback));
        m_eDewFallback[i] = (nVal == static_cast<int>(DEW_FALLBACK_ON))
                            ? DEW_FALLBACK_ON : DEW_FALLBACK_OFF;
    }

    // Regulated voltage — stored as integer tenths of a volt
    snprintf(szKey, sizeof(szKey), "RegulatedVoltageTenths_%d", m_nInstanceIndex);
    nVal = m_pIniUtil->readInt(kIniSection, szKey,
                               static_cast<int>(kDefaultRegulatedVoltageV * 10.0 + 0.5));
    if (nVal < 0)   nVal = 0;
    if (nVal > 153) nVal = 153;
    m_dRegulatedVoltageV = static_cast<double>(nVal) / 10.0;

    // Restore-on-connect flag
    snprintf(szKey, sizeof(szKey), "RestoreOnConnect_%d", m_nInstanceIndex);
    m_bRestoreOnConnect = (m_pIniUtil->readInt(kIniSection, szKey, 0) != 0);

    logDebug(kDebugCommands,
             "X2Svbony241Pro: dew config loaded — "
             "H1 mode=%s agg=%d fallback=%s fixed=%d%%  "
             "H2 mode=%s agg=%d fallback=%s fixed=%d%%  "
             "restoreOnConnect=%s",
             m_eDewMode[0] == DEW_MODE_AUTO ? "AUTO" : "MANUAL",
             m_nDewAggressiveness[0],
             m_eDewFallback[0] == DEW_FALLBACK_ON ? "ON" : "OFF",
             m_nDewFixedDutyPct[0],
             m_eDewMode[1] == DEW_MODE_AUTO ? "AUTO" : "MANUAL",
             m_nDewAggressiveness[1],
             m_eDewFallback[1] == DEW_FALLBACK_ON ? "ON" : "OFF",
             m_nDewFixedDutyPct[1],
             m_bRestoreOnConnect ? "YES" : "NO");
}

// ---------------------------------------------------------------------------
// saveCircuitStates
//
// Persists the RestoreOnConnect flag and the current on/off state of every
// X2 circuit so they can be reapplied by restoreCircuitStates() on the next
// connect.  Called from terminateLink() and from the dialog OK handler.
// ---------------------------------------------------------------------------
void X2Svbony241Pro::saveCircuitStates()
{
    if (m_pIniUtil == NULL)
        return;

    char szKey[64];
    snprintf(szKey, sizeof(szKey), "RestoreOnConnect_%d", m_nInstanceIndex);
    m_pIniUtil->writeInt(kIniSection, szKey, m_bRestoreOnConnect ? 1 : 0);

    for (int i = 0; i < X2_NUM_CIRCUITS; ++i)
    {
        snprintf(szKey, sizeof(szKey), "SavedCircuitState%d_%d", i, m_nInstanceIndex);
        m_pIniUtil->writeInt(kIniSection, szKey, m_bCircuitState[i] ? 1 : 0);
    }
}

// ---------------------------------------------------------------------------
// restoreCircuitStates
//
// Reads the saved circuit on/off states from the ini store and applies any
// that differ from the device's current state.  Only called when
// m_bRestoreOnConnect is true, after the link is established and the device's
// actual state has been read into m_bCircuitState[] via parseStateResponse().
//
// Uses setCircuitState() for each circuit so auto-dew logic runs correctly
// for heaters in DEW_MODE_AUTO.
// ---------------------------------------------------------------------------
void X2Svbony241Pro::restoreCircuitStates()
{
    if (!m_bRestoreOnConnect || m_pIniUtil == NULL)
        return;

    logDebug(kDebugErrors, "X2Svbony241Pro: restoring saved circuit states");

    char szKey[64];
    for (int i = 0; i < X2_NUM_CIRCUITS; ++i)
    {
        snprintf(szKey, sizeof(szKey), "SavedCircuitState%d_%d", i, m_nInstanceIndex);
        bool bWasOn = (m_pIniUtil->readInt(kIniSection, szKey, 0) != 0);

        if (bWasOn == m_bCircuitState[i])
            continue;   // device already in the desired state; no command needed

        logDebug(kDebugErrors,
                 "X2Svbony241Pro: restore circuit %d (%s) → %s",
                 i, kCircuitNames[i], bWasOn ? "ON" : "OFF");
        setCircuitState(i, bWasOn);
    }
}

// ===========================================================================
// Setup dialog  (ModalSettingsDialogInterface + X2GUIEventInterface)
// ===========================================================================

// ---------------------------------------------------------------------------
// Helper: enable or disable all auto-only rows for one heater in the dialog.
// heaterIdx 0 = Dew Heater 1, 1 = Dew Heater 2.
// bAutoMode true → enable the auto-only widgets; false → disable (grey out).
// Widgets remain always visible so the user can see what controls exist.
// ---------------------------------------------------------------------------
static void setDewAutoRowsEnabled(X2GUIExchangeInterface* uiex, int heaterIdx, bool bAutoMode)
{
    const char* suffix = (heaterIdx == 0) ? "1" : "2";

    char szName[64];
    const char* kAutoWidgets[] = {
        "lblDew%sAggressivenessCaption",
        "spinDew%sAggressiveness",
        "lblDew%sAggressivenessHint",
        "lblDew%sFallbackCaption",
        "comboDew%sFallback",
        "lblDew%sAutoDutyCaption",
        "lblDew%sAutoDuty",
    };

    for (int i = 0; i < 7; ++i)
    {
        snprintf(szName, sizeof(szName), kAutoWidgets[i], suffix);
        uiex->setEnabled(szName, bAutoMode);
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
        uiex->setText("lblConnectionStatus",
                       m_bLinked ? "Connected" : "Not connected");
        // Show warmup progress during post-reset sensor warmup; otherwise the
        // usual OK / Read error / — status.
        {
            char szWarmupBuf[64];
            const char* szSensorStatus = "—";  // safe default; always overwritten below
            bool bInWarmup = false;
            // Snapshot m_nResetTimeTick — terminateLink() can clear it concurrently.
            int nResetTick = m_nResetTimeTick;
            if (nResetTick > 0 && m_pTickCount)
            {
                int msSinceReset = m_pTickCount->elapsed() - nResetTick;
                if (msSinceReset < 0) msSinceReset = kSensorWarmupMs;  // wrap-safe
                if (msSinceReset < kSensorWarmupMs)
                {
                    bInWarmup = true;
                    int secsElapsed  = msSinceReset / 1000;
                    snprintf(szWarmupBuf, sizeof(szWarmupBuf),
                             "Warming up… ready in %d s",
                             (kSensorWarmupMs / 1000) - secsElapsed);
                    szSensorStatus = szWarmupBuf;
                    uiex->invokeMethod("progressWarmup", "show");
                    uiex->setPropertyInt("progressWarmup", "value", secsElapsed);
                }
            }
            if (!bInWarmup)
            {
                szSensorStatus =
                    m_bSensorValid              ? "OK"         :
                    (m_nLastDewUpdateTick != 0) ? "Read error" : "—";
                uiex->invokeMethod("progressWarmup", "hide");
            }
            uiex->setText("lblSensorStatus", szSensorStatus);
        }

        if (m_bLinked && m_bSensorValid)
        {
            char szBuf[64];
            snprintf(szBuf, sizeof(szBuf), "%.1f °C", m_dAmbientTempC);
            uiex->setText("lblAmbientTemp", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.1f %%", m_dAmbientHumidityPct);
            uiex->setText("lblHumidity", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.1f °C", m_dDewPointC);
            uiex->setText("lblDewPoint", szBuf);
        }

        // Populate status labels from cache only — no serial I/O before exec().
        // The dialog timer (on_timer uiEvent) refreshes these values while the
        // dialog is open.  Performing serial reads here would block the calling
        // thread for up to ~600 ms per command (100 ms post-write sleep + 500 ms
        // read timeout) before the dialog even appears, and could contend with
        // circuitState() for the I/O mutex on a concurrent TSX polling thread.
        if (m_bLinked)
        {
            char szBuf[64];

            // INA219 — from cache (updated on the last TSX poll cycle)
            snprintf(szBuf, sizeof(szBuf), "%.2f V", m_dVoltageV);
            uiex->setText("lblVoltage", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.3f A", m_dCurrentA);
            uiex->setText("lblCurrent", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.2f W", m_dPowerW);
            uiex->setText("lblPower", szBuf);

            // Hub temperature — SHT40 raw (self-heated); shown for diagnostics
            if (m_bSensorValid)
            {
                char szLens[32];
                snprintf(szLens, sizeof(szLens), "%.1f °C", m_dHubTempC);
                uiex->setText("lblLensTemp", szLens);
            }
            else
            {
                uiex->setText("lblLensTemp", "—");
            }

            // Port states from cache
            const char* kDCLabels[] = { "lblDC1State","lblDC2State","lblDC3State","lblDC4State" };
            for (int i = 0; i < 4; ++i)
                uiex->setText(kDCLabels[i], m_bCircuitState[i] ? "ON" : "OFF");
            uiex->setText("lblUSBState",
                           m_bCircuitState[6] ? "ON" : "OFF");

            if (m_bCircuitState[7])
            {
                snprintf(szBuf, sizeof(szBuf), "%.1f V", m_dRegulatedVoltageV);
                uiex->setText("lblRegulatedState", szBuf);
            }
            else
            {
                uiex->setText("lblRegulatedState", "OFF");
            }
        }

        // --- Debug level ---
        uiex->setCurrentIndex("comboDebugLevel", m_nDebugLevel);

        // --- Dew heater settings ---
        for (int i = 0; i < 2; ++i)
        {
            const char* modeCombo  = (i == 0) ? "comboDew1Mode"          : "comboDew2Mode";
            const char* dutyBox    = (i == 0) ? "spinDew1FixedDuty"      : "spinDew2FixedDuty";
            const char* aggBox     = (i == 0) ? "spinDew1Aggressiveness" : "spinDew2Aggressiveness";
            const char* fallCombo  = (i == 0) ? "comboDew1Fallback"      : "comboDew2Fallback";
            const char* autoDuty   = (i == 0) ? "lblDew1AutoDuty"        : "lblDew2AutoDuty";

            uiex->setCurrentIndex(modeCombo,
                                  (m_eDewMode[i] == DEW_MODE_AUTO) ? 1 : 0);
            uiex->setPropertyInt(dutyBox, "value", m_nDewFixedDutyPct[i]);
            uiex->setPropertyInt(aggBox,  "value", m_nDewAggressiveness[i]);
            uiex->setCurrentIndex(fallCombo,
                                  (m_eDewFallback[i] == DEW_FALLBACK_ON) ? 1 : 0);

            // Show/hide auto-only rows
            bool bAuto = (m_eDewMode[i] == DEW_MODE_AUTO);
            setDewAutoRowsEnabled(uiex, i, bAuto);

            // Computed/actual duty label
            if (bAuto)
            {
                char szBuf[64];
                if (!m_bCircuitState[4 + i])
                {
                    snprintf(szBuf, sizeof(szBuf), "OFF");
                }
                else if (m_nLastSentDutyPct[i] >= 0)
                {
                    snprintf(szBuf, sizeof(szBuf), "%d%% (sent)", m_nLastSentDutyPct[i]);
                }
                else
                {
                    snprintf(szBuf, sizeof(szBuf), "%d%% (computed)", calcAutoDutyPct(i));
                }
                uiex->setText(autoDuty, szBuf);
            }
        }

        // --- Adjustable voltage spinbox ---
        uiex->setPropertyDouble("dblSpinRegulatedVoltage", "value", m_dRegulatedVoltageV);

        // --- Startup options ---
        uiex->setChecked("chkRestoreOnConnect", m_bRestoreOnConnect);

        nErr = pUI->exec(bPressedOK);
        if (nErr) break;

        if (bPressedOK)
        {
            // Read back dew heater settings and persist
            for (int i = 0; i < 2; ++i)
            {
                const char* modeCombo = (i == 0) ? "comboDew1Mode"          : "comboDew2Mode";
                const char* dutyBox   = (i == 0) ? "spinDew1FixedDuty"      : "spinDew2FixedDuty";
                const char* aggBox    = (i == 0) ? "spinDew1Aggressiveness"  : "spinDew2Aggressiveness";
                const char* fallCombo = (i == 0) ? "comboDew1Fallback"      : "comboDew2Fallback";

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

            // Adjustable voltage output
            double dVoltage = m_dRegulatedVoltageV;
            uiex->propertyDouble("dblSpinRegulatedVoltage", "value", dVoltage);
            if (dVoltage < 0.0)  dVoltage = 0.0;
            if (dVoltage > 15.3) dVoltage = 15.3;
            m_dRegulatedVoltageV = dVoltage;

            // If circuit 7 is already ON, push the new voltage to the device immediately
            if (m_bLinked && m_bCircuitState[7])
            {
                uint8_t raw = onValueForCircuit(7);
                cmdSetPort(kPortIndex[7], raw);
            }

            // Debug level
            int nDbg = uiex->currentIndex("comboDebugLevel");
            if (nDbg < kDebugOff)    nDbg = kDebugOff;
            if (nDbg > kDebugFullIO) nDbg = kDebugFullIO;
            m_nDebugLevel = nDbg;

            // Startup options
            m_bRestoreOnConnect = uiex->isChecked("chkRestoreOnConnect");

            saveDebugLevel();
            saveDewConfig();
            saveCircuitStates();

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
// "on_comboDew{1,2}Mode_currentIndexChanged"
//     Show or hide the auto-only rows depending on selected mode.
// ---------------------------------------------------------------------------
void X2Svbony241Pro::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    if (uiex == NULL || pszEvent == NULL)
        return;

    // ── Timer tick: refresh UI from cached values only (no serial I/O) ───
    // Serial reads happen on TSX polling threads (circuitState / setCircuitState).
    // Touching the port here would block the TSX UI thread.
    if (!strcmp(pszEvent, "on_timer"))
    {
        uiex->setText("lblConnectionStatus", m_bLinked ? "Connected" : "Not connected");

        if (!m_bLinked)
            return;

        // Warmup progress or normal sensor status.
        // TSX suspends circuitState() polling while this modal dialog is open,
        // so the 30 s background rate never fires.  We drive a 5 s dialog poll
        // rate here (bForce=true) so readings stay fresh while the dialog is open.
        {
            char szWarmupBuf[64];
            const char* szSensorStatus = "—";  // safe default; always overwritten below
            bool bInWarmup = false;
            // Snapshot m_nResetTimeTick — terminateLink() can clear it concurrently.
            int nResetTick = m_nResetTimeTick;
            if (nResetTick > 0 && m_pTickCount)
            {
                int msSinceReset = m_pTickCount->elapsed() - nResetTick;
                if (msSinceReset < 0) msSinceReset = kSensorWarmupMs;  // wrap-safe
                if (msSinceReset < kSensorWarmupMs)
                {
                    bInWarmup = true;
                    int secsElapsed  = msSinceReset / 1000;
                    snprintf(szWarmupBuf, sizeof(szWarmupBuf),
                             "Warming up… ready in %d s",
                             (kSensorWarmupMs / 1000) - secsElapsed);
                    szSensorStatus = szWarmupBuf;
                    uiex->invokeMethod("progressWarmup", "show");
                    uiex->setPropertyInt("progressWarmup", "value", secsElapsed);
                }
            }
            if (!bInWarmup)
            {
                // Dialog-driven polling at 5 s while this dialog is open.
                // TSX suspends circuitState() while the modal dialog runs, so
                // the 30 s background rate never fires; we drive our own clock.
                // bForce=true bypasses the 30 s rate limit but does NOT reset
                // m_nLastDewUpdateTick, so the background clock is undisturbed.
                int nowTick = m_pTickCount ? m_pTickCount->elapsed() : 0;
                int sinceDialogPoll = nowTick - m_nLastDialogSensorTick;
                if (sinceDialogPoll < 0) sinceDialogPoll = kDialogPollIntervalMs;  // wrap-safe
                if (m_nLastDialogSensorTick == 0 || sinceDialogPoll >= kDialogPollIntervalMs)
                {
                    m_nLastDialogSensorTick = nowTick;
                    logDebug(kDebugCommands,
                             "X2Svbony241Pro: dialog timer poll (5 s)");
                    updateDewControl(true /*bForce*/);
                }

                szSensorStatus =
                    m_bSensorValid              ? "OK"         :
                    (m_nLastDewUpdateTick != 0) ? "Read error" : "—";
                uiex->invokeMethod("progressWarmup", "hide");
            }
            uiex->setText("lblSensorStatus", szSensorStatus);
        }

        // Environmental sensor labels — from cache
        char szBuf[64];
        if (m_bSensorValid)
        {
            snprintf(szBuf, sizeof(szBuf), "%.1f °C", m_dAmbientTempC);
            uiex->setText("lblAmbientTemp", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.1f %%", m_dAmbientHumidityPct);
            uiex->setText("lblHumidity", szBuf);

            snprintf(szBuf, sizeof(szBuf), "%.1f °C", m_dDewPointC);
            uiex->setText("lblDewPoint", szBuf);
        }
        else
        {
            uiex->setText("lblAmbientTemp", "—");
            uiex->setText("lblHumidity",    "—");
            uiex->setText("lblDewPoint",    "—");
        }

        // INA219 — from cache
        snprintf(szBuf, sizeof(szBuf), "%.2f V", m_dVoltageV);
        uiex->setText("lblVoltage", szBuf);
        snprintf(szBuf, sizeof(szBuf), "%.3f A", m_dCurrentA);
        uiex->setText("lblCurrent", szBuf);
        snprintf(szBuf, sizeof(szBuf), "%.2f W", m_dPowerW);
        uiex->setText("lblPower", szBuf);

        // Hub temp (SHT40 raw, self-heated) — shown for diagnostics
        if (m_bSensorValid)
        {
            snprintf(szBuf, sizeof(szBuf), "%.1f °C", m_dHubTempC);
            uiex->setText("lblLensTemp", szBuf);
        }
        else
        {
            uiex->setText("lblLensTemp", "—");
        }

        // DC port on/off states — from cache
        const char* kDCLabels[] = { "lblDC1State","lblDC2State","lblDC3State","lblDC4State" };
        for (int i = 0; i < 4; ++i)
            uiex->setText(kDCLabels[i], m_bCircuitState[i] ? "ON" : "OFF");

        uiex->setText("lblUSBState", m_bCircuitState[6] ? "ON" : "OFF");

        if (m_bCircuitState[7])
        {
            snprintf(szBuf, sizeof(szBuf), "%.1f V", m_dRegulatedVoltageV);
            uiex->setText("lblRegulatedState", szBuf);
        }
        else
        {
            uiex->setText("lblRegulatedState", "OFF");
        }

        // Auto dew actual/computed duty labels — updated from cache on each timer tick
        for (int i = 0; i < 2; ++i)
        {
            const char* autoDutyLabel = (i == 0) ? "lblDew1AutoDuty" : "lblDew2AutoDuty";
            if (m_eDewMode[i] == DEW_MODE_AUTO)
            {
                if (!m_bCircuitState[4 + i])
                {
                    uiex->setText(autoDutyLabel, "OFF");
                }
                else if (m_nLastSentDutyPct[i] >= 0)
                {
                    snprintf(szBuf, sizeof(szBuf), "%d%% (sent)", m_nLastSentDutyPct[i]);
                    uiex->setText(autoDutyLabel, szBuf);
                }
                else
                {
                    snprintf(szBuf, sizeof(szBuf), "%d%% (computed)", calcAutoDutyPct(i));
                    uiex->setText(autoDutyLabel, szBuf);
                }
            }
        }

        return;
    }

    // ── Mode combo changed: enable/disable auto-only rows immediately ────────
    if (!strcmp(pszEvent, "on_comboDew1Mode_currentIndexChanged"))
    {
        bool bAuto = (uiex->currentIndex("comboDew1Mode") == 1);
        setDewAutoRowsEnabled(uiex, 0, bAuto);
        return;
    }

    if (!strcmp(pszEvent, "on_comboDew2Mode_currentIndexChanged"))
    {
        bool bAuto = (uiex->currentIndex("comboDew2Mode") == 1);
        setDewAutoRowsEnabled(uiex, 1, bAuto);
        return;
    }
}
