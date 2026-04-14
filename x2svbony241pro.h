#pragma once

#include "licensedinterfaces/sberrorx.h"
#include "licensedinterfaces/basicstringinterface.h"
#include "licensedinterfaces/powercontroldriverinterface.h"
#include "licensedinterfaces/serxinterface.h"
#include "licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "licensedinterfaces/sleeperinterface.h"
#include "licensedinterfaces/basiciniutilinterface.h"
#include "licensedinterfaces/loggerinterface.h"
#include "licensedinterfaces/mutexinterface.h"
#include "licensedinterfaces/tickcountinterface.h"
#include "licensedinterfaces/modalsettingsdialoginterface.h"
#include "licensedinterfaces/x2guiinterface.h"
#include "licensedinterfaces/circuitlabels.h"
#include "licensedinterfaces/serialportparams2interface.h"

#include <stdint.h>
#include <string>

// ---------------------------------------------------------------------------
// Dew-heater operating mode (per heater)
// ---------------------------------------------------------------------------
enum DewHeaterMode
{
    DEW_MODE_MANUAL = 0,    // Fixed duty cycle (m_nDewFixedDutyPct[])
    DEW_MODE_AUTO   = 1     // Automatic: duty driven by dew-point algorithm
};

// What to do when the environmental sensor read fails in DEW_MODE_AUTO
enum DewFallback
{
    DEW_FALLBACK_OFF = 0,   // Turn the heater off
    DEW_FALLBACK_ON  = 1    // Run at the stored fixed duty cycle
};

/*
 * SVBony SV241 Pro USB Power Hub — X2 Power Control Driver
 *
 * X2 circuit layout:
 *   Circuit 0 : DC Port 1     (12V switchable)
 *   Circuit 1 : DC Port 2     (12V switchable)
 *   Circuit 2 : DC Port 3     (12V switchable)
 *   Circuit 3 : DC Port 4     (12V switchable)
 *   Circuit 4 : Dew Heater 1  (PWM, default 50% duty)
 *   Circuit 5 : Dew Heater 2  (PWM, default 50% duty)
 *   Circuit 6 : USB Hub Power (group 0: USB-C, USB1, USB2)
 *   Circuit 7 : Adjustable Output (regulated voltage, default 12 V)
 *
 * Device port_index mapping for cmd 0x01 (see PROTOCOL.md §7–10):
 *   0x00 : DC 1    0x01 : DC 2    0x02 : DC 3    0x03 : DC 4
 *   0x04 : DC 5    (device has 5 DC ports; we expose 4 in X2)
 *   0x05 : USB group 0 (USB-C, USB1, USB2)
 *   0x06 : USB group 1 (USB3, USB4, USB5)  (not exposed in X2)
 *   0x07 : Regulated voltage output
 *   0x08 : Dew heater PWM 1
 *   0x09 : Dew heater PWM 2
 */

#define X2_NUM_CIRCUITS 8

class X2Svbony241Pro : public PowerControlDriverInterface
                     , public ModalSettingsDialogInterface
                     , public X2GUIEventInterface
                     , public CircuitLabelsInterface
                     , public SerialPortParams2Interface
{
public:
    X2Svbony241Pro(
        const char*                         pszDisplayName,
        const int&                          nInstanceIndex,
        SerXInterface*                      pSerXIn,
        TheSkyXFacadeForDriversInterface*   pTheSkyXIn,
        SleeperInterface*                   pSleeperIn,
        BasicIniUtilInterface*              pIniUtilIn,
        LoggerInterface*                    pLoggerIn,
        MutexInterface*                     pIOMutexIn,
        TickCountInterface*                 pTickCountIn);

    virtual ~X2Svbony241Pro();

    // -----------------------------------------------------------------------
    // DriverRootInterface
    // -----------------------------------------------------------------------
    virtual DeviceType  deviceType()                                        { return DriverRootInterface::DT_POWERCONTROL; }
    virtual int         queryAbstraction(const char* pszName, void** ppVal);

    // -----------------------------------------------------------------------
    // DriverInfoInterface
    // -----------------------------------------------------------------------
    virtual void        driverInfoDetailedInfo(BasicStringInterface& str) const;
    virtual double      driverInfoVersion() const;

    // -----------------------------------------------------------------------
    // DeviceInfoInterface
    // -----------------------------------------------------------------------
    virtual void        deviceInfoNameShort(BasicStringInterface& str) const;
    virtual void        deviceInfoNameLong(BasicStringInterface& str) const;
    virtual void        deviceInfoDetailedDescription(BasicStringInterface& str) const;
    virtual void        deviceInfoFirmwareVersion(BasicStringInterface& str);
    virtual void        deviceInfoModel(BasicStringInterface& str);

    // -----------------------------------------------------------------------
    // LinkInterface
    // -----------------------------------------------------------------------
    virtual int         establishLink();
    virtual int         terminateLink();
    virtual bool        isLinked() const;

    // -----------------------------------------------------------------------
    // PowerControlDriverInterface
    // -----------------------------------------------------------------------
    virtual int         numberOfCircuits(int& nNumber);
    virtual int         circuitName(const int& nIndex, BasicStringInterface& str);
    virtual int         circuitState(const int& nIndex, bool& bOn);
    virtual int         setCircuitState(const int& nIndex, const bool& bOn);

    // -----------------------------------------------------------------------
    // CircuitLabelsInterface
    // -----------------------------------------------------------------------
    virtual int         circuitLabel(const int& nZeroBasedIndex, BasicStringInterface& str);

    // -----------------------------------------------------------------------
    // SerialPortParams2Interface
    // -----------------------------------------------------------------------
    virtual void            portName(BasicStringInterface& str) const;
    virtual void            setPortName(const char* szPort);
    virtual unsigned int    baudRate()       const   { return 115200; }
    virtual void            setBaudRate(unsigned int)       {}
    virtual bool            isBaudRateFixed() const         { return true; }
    virtual SerXInterface::Parity parity()  const   { return SerXInterface::B_NOPARITY; }
    virtual void            setParity(const SerXInterface::Parity&) {}
    virtual bool            isParityFixed()  const          { return true; }

    // -----------------------------------------------------------------------
    // ModalSettingsDialogInterface
    // -----------------------------------------------------------------------
    virtual int         initModalSettingsDialog()           { return SB_OK; }
    virtual int         execModalSettingsDialog();

    // -----------------------------------------------------------------------
    // X2GUIEventInterface
    // -----------------------------------------------------------------------
    virtual void        uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent);

private:
    // -----------------------------------------------------------------------
    // Binary frame I/O  (see PROTOCOL.md §3–4)
    // -----------------------------------------------------------------------

    // Build a command frame: [0x24, DATA_LEN, cmd[0..n-1], CHECKSUM].
    // pFrameOut must be at least nCmdLen+3 bytes.  Returns total frame length.
    int         buildFrame(const uint8_t* pCmd, int nCmdLen,
                           uint8_t* pFrameOut, int& nFrameLenOut) const;

    // Checksum = sum of pBuf[0..nLen-1] mod 0xFF.
    uint8_t     calcChecksum(const uint8_t* pBuf, int nLen) const;

    // Send nCmdLen-byte command payload and receive nResDataLen data bytes.
    // On success pResDataOut receives the payload (no header/status/checksum).
    // Returns SB_OK | ERR_COMMNOLINK | ERR_COMMTIMEOUT | ERR_CMDFAILED.
    int         sendFrame(const uint8_t* pCmd, int nCmdLen,
                          int nResDataLen, uint8_t* pResDataOut);

    // Read exactly nExpectedBytes from the port, with 500 ms timeout.
    int         readFrame(int nExpectedBytes, uint8_t* pBufOut);

    // -----------------------------------------------------------------------
    // Initialisation helpers
    // -----------------------------------------------------------------------

    // Read and discard the ESP32 boot log that arrives after port open.
    // Returns SB_OK when the port goes quiet (non-fatal if not fully drained).
    int         drainBootLog();

    // -----------------------------------------------------------------------
    // Device commands — one per protocol command byte (see PROTOCOL.md §5)
    // -----------------------------------------------------------------------

    // 0x01 — Set any output channel.
    //         portIndex: device port_index byte (0x00–0x09)
    //         value:     0x00=off, 0xFF=on (DC/USB), PWM raw byte, or voltage raw byte
    int         cmdSetPort(uint8_t portIndex, uint8_t value);

    // 0x02 — Read INA219 total input power; result in watts.
    int         cmdReadPower(double& powerW);

    // 0x03 — Read INA219 bus voltage; result in volts.
    int         cmdReadVoltage(double& voltageV);

    // 0x04 — Read DS18B20 lens temperature; result in °C.
    int         cmdReadDS18B20Temp(double& tempC);

    // 0x05 — Read SHT40 ambient temperature; result in °C.
    int         cmdReadSHT40Temp(double& tempC);

    // 0x06 — Read SHT40 relative humidity; result in % RH.
    int         cmdReadSHT40Humidity(double& humidityPct);

    // 0x07 — Read INA219 bus current; result in amps.
    int         cmdReadCurrent(double& currentA);

    // 0x08 — Read full device state.
    //         pState10Out receives 10 raw bytes: GPIO1-7, regulated, PWM1, PWM2.
    //         Caller must supply at least 10 bytes.
    int         cmdGetState(uint8_t* pState10Out);

    // 0xFF/0xFF → sleep 1 s → 0xFE/0xFE — Power-cycle all outputs.
    int         cmdPowerCycle();

    // -----------------------------------------------------------------------
    // Sensor decode helpers — pure functions, no side-effects
    // -----------------------------------------------------------------------

    // Decode 4-byte big-endian payload and divide by 100 (base conversion).
    static double decodeSensor4(const uint8_t* pData4);

    // DS18B20: (raw/100 - 255.5), rounded to 2 decimal places → °C
    static double decodeDS18B20Temp(const uint8_t* pData4);

    // SHT40 temp: (raw/100 - 254.0), rounded to 1 decimal place → °C
    static double decodeSHT40Temp(const uint8_t* pData4);

    // SHT40 humidity: (raw/100 - 254.0), rounded to 1 decimal place → % RH
    static double decodeSHT40Humidity(const uint8_t* pData4);

    // -----------------------------------------------------------------------
    // Circuit ↔ device port mapping helpers
    // -----------------------------------------------------------------------

    // Return the device port_index byte for X2 circuit nCircuit.
    static uint8_t  portIndexForCircuit(int nCircuit);

    // Return the raw ON value for cmd 0x01 for X2 circuit nCircuit.
    // Reads m_nDewFixedDutyPct[] and m_dRegulatedVoltageV for analogue channels.
    uint8_t         onValueForCircuit(int nCircuit) const;

    // Populate m_bCircuitState[] (and cached analogue levels) from a 10-byte
    // state response as returned by cmdGetState().
    void            parseStateResponse(const uint8_t* pState10);

    // Issue cmdGetState() and update all cached state.
    int             queryAllCircuitStates();

    // -----------------------------------------------------------------------
    // Auto dew-point control  (circuits 4 and 5)
    // -----------------------------------------------------------------------

    // Compute dew point (°C) from ambient temperature and relative humidity.
    // Uses the Magnus formula.  Pure function, no side-effects.
    static double   calcDewPoint(double ambientTempC, double humidityPct);

    // Return the auto-algorithm duty cycle % (0–100) for heater heaterIdx
    // (0 = circuit 4, 1 = circuit 5).  Returns the fallback value if
    // m_bSensorValid is false.  Does not write to device.
    int             calcAutoDutyPct(int heaterIdx) const;

    // Read SHT40 sensors, recalculate dew point, and push the resulting PWM
    // value to any heater in DEW_MODE_AUTO that is currently on.
    // Rate-limited to once per kDewUpdateIntervalMs via m_nLastDewUpdateTick.
    // Pass bForce=true to bypass the rate limit (used by the dialog timer for
    // the 5 s dialog-open poll rate without disturbing the 30 s background rate).
    // Never throws; sensor failures apply the configured fallback.
    int             updateDewControl(bool bForce = false);

    // Persist dew-heater configuration to the TheSkyX ini store.
    void            saveDewConfig();

    // Restore dew-heater configuration from the TheSkyX ini store.
    // Called on establishLink(); applies safe defaults if keys are absent.
    void            loadDewConfig();

    // -----------------------------------------------------------------------
    // TSX-provided interfaces (not owned by this object — do NOT delete)
    // -----------------------------------------------------------------------
    SerXInterface*                      m_pSerX;
    TheSkyXFacadeForDriversInterface*   m_pTheSkyX;
    SleeperInterface*                   m_pSleeper;
    BasicIniUtilInterface*              m_pIniUtil;
    LoggerInterface*                    m_pLogger;
    MutexInterface*                     m_pIOMutex;
    TickCountInterface*                 m_pTickCount;

    // Persist and reload debug level from ini (0=Off,1=Errors,2=Cmds,3=FullIO)
    void            saveDebugLevel();
    void            loadDebugLevel();

    // Log helper: only writes if m_nDebugLevel >= minLevel.
    // minLevel: 1=Errors only, 2=Commands, 3=Full I/O
    void            logDebug(int minLevel, const char* fmt, ...) const;

    // Helper used by portName() and establishLink()
    void        getPortName(std::string& sPortName) const;

    int     m_nInstanceIndex;
    bool    m_bLinked;
    int     m_nDebugLevel;      // 0=Off, 1=Errors, 2=Commands, 3=Full I/O

    // Cached on/off state for each X2 circuit
    bool    m_bCircuitState[X2_NUM_CIRCUITS];

    // Cached INA219 sensor readings
    double  m_dPowerW;
    double  m_dVoltageV;
    double  m_dCurrentA;

    // Persisted analogue output levels — used when toggling the channel ON
    int     m_nDewFixedDutyPct[2]; // Circuits 4-5, range 0–100 % (MANUAL level & FALLBACK_ON level)
    double  m_dRegulatedVoltageV;  // Circuit 7, range 0.0–15.3 V

    // -----------------------------------------------------------------------
    // Dew-heater auto-control state (per heater: [0]=circuit4, [1]=circuit5)
    // -----------------------------------------------------------------------
    DewHeaterMode   m_eDewMode[2];          // MANUAL or AUTO
    int             m_nDewAggressiveness[2]; // 1 (gentle) – 10 (aggressive)
    DewFallback     m_eDewFallback[2];       // what to do when sensor fails

    // Environmental readings refreshed by updateDewControl()
    double          m_dAmbientTempC;       // DS18B20 when valid, else SHT40 (used by algorithm + display)
    double          m_dAmbientHumidityPct; // SHT40 RH
    double          m_dDewPointC;
    double          m_dHubTempC;           // SHT40 raw temperature (device-internal, self-heated)
    bool            m_bSensorValid;        // false after any SHT40 read failure

    // DS18B20 lens/ambient temperature (separate validity flag; probe may be absent)
    double          m_dLensTempC;
    bool            m_bLensTempValid;

    // Actual duty % last successfully written to each dew heater; -1 = never sent
    int             m_nLastSentDutyPct[2];

    // TickCountInterface timestamp of last successful auto-dew sensor update.
    // Stored as int to match the return type of TickCountInterface::elapsed().
    // Zero means "never updated" (forces the first call to run immediately).
    int             m_nLastDewUpdateTick;

    // Timestamp of the last sensor read triggered from the settings dialog timer.
    // Independent of m_nLastDewUpdateTick so the dialog (5 s) and background
    // (30 s) polling rates can coexist without interfering with each other.
    int             m_nLastDialogSensorTick;

    // TickCountInterface timestamp of the last hardware reset (DTR pulse).
    // Non-zero means a reset was performed; updateDewControl() skips sensor
    // reads until kSensorWarmupMs have elapsed from this point.
    // Zero means no reset has been performed since construction.
    int             m_nResetTimeTick;
};
