#include "x2svbony241pro.h"

#include <string.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Circuit names — keep in sync with the index definitions in x2svbony241pro.h
// ---------------------------------------------------------------------------
static const char* kCircuitNames[X2_NUM_CIRCUITS] =
{
    "DC Port 1 (12V)",          // 0
    "DC Port 2 (12V)",          // 1
    "DC Port 3 (12V)",          // 2
    "DC Port 4 (12V)",          // 3
    "Dew Heater 1",             // 4
    "Dew Heater 2",             // 5
    "USB Hub Power",            // 6
    "Adjustable Output"         // 7
};

// Serial baud rate used when opening the virtual COM port exposed by the
// SV241 Pro USB hub.
static const int kBaudRate = 115200;

// Maximum number of bytes expected in a single device response line.
static const int kRespBufLen = 256;

// ---------------------------------------------------------------------------
// Constructor / Destructor
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
    : m_pSerX       (pSerXIn)
    , m_pTheSkyX    (pTheSkyXIn)
    , m_pSleeper    (pSleeperIn)
    , m_pIniUtil    (pIniUtilIn)
    , m_pLogger     (pLoggerIn)
    , m_pIOMutex    (pIOMutexIn)
    , m_pTickCount  (pTickCountIn)
    , m_nInstanceIndex(nInstanceIndex)
    , m_bLinked     (false)
{
    (void)pszDisplayName;   // not used in this driver

    for (int i = 0; i < X2_NUM_CIRCUITS; ++i)
        m_bCircuitState[i] = false;
}

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
    {
        *ppVal = m_pLogger;
        return SB_OK;
    }

    return SB_OK;   // unknown interface — return SB_OK with *ppVal == NULL
}

// ---------------------------------------------------------------------------
// DriverInfoInterface
// ---------------------------------------------------------------------------

void X2Svbony241Pro::driverInfoDetailedInfo(BasicStringInterface& str) const
{
    str = "X2 Power Control driver for the SVBony SV241 Pro USB Power Hub. "
          "Supports 4 x 12V DC outputs, 2 x dew heater ports (PWM), "
          "USB hub power, and one adjustable output. "
          "Written as an open-source community driver. "
          "USB serial protocol stubs are present — see TODO comments in "
          "x2svbony241pro.cpp for details.";
}

double X2Svbony241Pro::driverInfoVersion() const
{
    return 1.0;
}

// ---------------------------------------------------------------------------
// DeviceInfoInterface
// ---------------------------------------------------------------------------

void X2Svbony241Pro::deviceInfoNameShort(BasicStringInterface& str) const
{
    str = "SVBony SV241 Pro";
}

void X2Svbony241Pro::deviceInfoNameLong(BasicStringInterface& str) const
{
    str = "SVBony SV241 Pro USB Power Hub";
}

void X2Svbony241Pro::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
    str = "SVBony SV241 Pro — USB-connected astronomy power control box and "
          "USB hub. Features: 4 x switchable 12V DC output ports, "
          "2 x PWM-controlled dew heater ports (0-100%), "
          "integrated USB hub, 1 x adjustable voltage output, "
          "and per-port current/voltage monitoring.";
}

void X2Svbony241Pro::deviceInfoFirmwareVersion(BasicStringInterface& str)
{
    // TODO: Once the USB serial protocol is known, issue a firmware-query
    //       command here and return the actual version string reported by
    //       the device.  For now return a placeholder.
    if (m_bLinked)
        str = "Unknown (query not yet implemented)";
    else
        str = "N/A";
}

void X2Svbony241Pro::deviceInfoModel(BasicStringInterface& str)
{
    str = "SV241 Pro";
}

// ---------------------------------------------------------------------------
// LinkInterface
// ---------------------------------------------------------------------------

int X2Svbony241Pro::establishLink()
{
    if (m_pSerX == NULL)
        return ERR_COMMNOLINK;

    // Open the serial (virtual COM) port at 115200 8N1.
    // TheSkyX will have already configured the port name via the UI.
    int nErr = m_pSerX->open(NULL, kBaudRate, SerXInterface::B_NOPARITY, 0);
    if (nErr != SB_OK)
        return nErr;

    m_bLinked = true;

    // Query the device to obtain the initial state of all circuits.
    // If this fails we still consider ourselves linked — the cached state
    // (all-off) will be used until a successful command round-trip occurs.
    (void)queryAllCircuitStates();

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

bool X2Svbony241Pro::isLinked() const
{
    return m_bLinked;
}

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

    // -----------------------------------------------------------------------
    // TODO: Replace this stub with the actual USB serial command required to
    //       switch circuit `nIndex` on (bOn == true) or off (bOn == false).
    //
    // The SV241 Pro USB serial protocol has not yet been documented or
    // reverse-engineered.  The expected command format and response format
    // are unknown at the time of writing.
    //
    // Suggested approach once the protocol is known:
    //
    //   char szCmd[64];
    //   char szResp[kRespBufLen];
    //   snprintf(szCmd, sizeof(szCmd), "SET %d %s\r\n",
    //            nIndex, bOn ? "ON" : "OFF");
    //   int nErr = sendCommand(szCmd, szResp, kRespBufLen);
    //   if (nErr != SB_OK)
    //       return nErr;
    //   // Parse szResp to confirm acknowledgement ...
    //
    // For now, just update the local state cache and return success so that
    // the TheSkyX UI reflects the requested state immediately during testing.
    // -----------------------------------------------------------------------

    if (m_pLogger)
    {
        char szMsg[128];
        snprintf(szMsg, sizeof(szMsg),
                 "X2Svbony241Pro: setCircuitState(%d, %s) — STUB, no actual device command sent",
                 nIndex, bOn ? "ON" : "OFF");
        m_pLogger->out(szMsg);
    }

    m_bCircuitState[nIndex] = bOn;
    return SB_OK;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

int X2Svbony241Pro::sendCommand(const char* pszCmd, char* pszResp, const int nRespLen)
{
    if (!m_bLinked || m_pSerX == NULL)
        return ERR_COMMNOLINK;

    // Flush any stale data already sitting in the receive buffer.
    m_pSerX->purgeTxRx();

    // Write the command string.
    unsigned long nBytesWritten = 0;
    int nLen = (int)strlen(pszCmd);
    int nErr = m_pSerX->writeFile((void*)pszCmd, nLen, nBytesWritten);
    if (nErr != SB_OK)
    {
        if (m_pLogger)
            m_pLogger->out("X2Svbony241Pro: sendCommand — writeFile failed");
        return nErr;
    }

    // Read back the response.
    return readResponse(pszResp, nRespLen);
}

int X2Svbony241Pro::readResponse(char* pszResp, const int nRespLen)
{
    if (!m_bLinked || m_pSerX == NULL)
        return ERR_COMMNOLINK;

    // -----------------------------------------------------------------------
    // TODO: Adapt the read logic to the actual SV241 Pro response framing.
    //
    // The loop below reads one byte at a time until it sees a newline '\n'
    // (or '\r'), which is typical for ASCII-framed protocols.  If the device
    // uses binary framing or a different terminator this section must be
    // updated accordingly.
    //
    // A timeout counter is used to avoid hanging indefinitely if the device
    // does not respond.
    // -----------------------------------------------------------------------

    const int kMaxRetries = 5000;   // rough timeout guard (iterations)
    int nIdx    = 0;
    int nRetry  = 0;
    bool bDone  = false;

    if (pszResp == NULL || nRespLen <= 0)
        return ERR_BADPARAMETER;

    pszResp[0] = '\0';

    while (!bDone && nRetry < kMaxRetries)
    {
        unsigned long nBytesRead = 0;
        char cByte = 0;

        int nErr = m_pSerX->readFile(&cByte, 1, nBytesRead);
        if (nErr != SB_OK)
            return nErr;

        if (nBytesRead == 0)
        {
            ++nRetry;
            if (m_pSleeper)
                m_pSleeper->sleep(1);   // 1 ms
            continue;
        }

        nRetry = 0;     // reset timeout on successful byte

        if (cByte == '\n' || cByte == '\r')
        {
            bDone = true;
        }
        else
        {
            if (nIdx < nRespLen - 1)
                pszResp[nIdx++] = cByte;
        }
    }

    pszResp[nIdx] = '\0';

    if (nRetry >= kMaxRetries)
    {
        if (m_pLogger)
            m_pLogger->out("X2Svbony241Pro: readResponse — timeout waiting for device");
        return ERR_COMMTIMEOUT;
    }

    return SB_OK;
}

int X2Svbony241Pro::queryAllCircuitStates()
{
    // -----------------------------------------------------------------------
    // TODO: Issue the appropriate status-query command to the SV241 Pro and
    //       parse its response to populate m_bCircuitState[].
    //
    // Once the device protocol is known this method should:
    //   1. Send a status/query command, e.g. "STATUS\r\n"
    //   2. Parse the response to extract the on/off state of each circuit.
    //   3. Populate m_bCircuitState[0..X2_NUM_CIRCUITS-1] accordingly.
    //
    // Example placeholder (protocol unknown):
    //
    //   char szResp[kRespBufLen];
    //   int nErr = sendCommand("STATUS\r\n", szResp, kRespBufLen);
    //   if (nErr != SB_OK)
    //       return nErr;
    //   // ... parse szResp ...
    //
    // For now, initialise all circuits to OFF (safe default).
    // -----------------------------------------------------------------------

    for (int i = 0; i < X2_NUM_CIRCUITS; ++i)
        m_bCircuitState[i] = false;

    if (m_pLogger)
        m_pLogger->out("X2Svbony241Pro: queryAllCircuitStates — STUB, all circuits set to OFF");

    return SB_OK;
}
