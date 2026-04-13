#pragma once

#include "../X2-Examples/licensedinterfaces/powercontroldriverinterface.h"
#include "../X2-Examples/licensedinterfaces/serxinterface.h"
#include "../X2-Examples/licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../X2-Examples/licensedinterfaces/sleeperinterface.h"
#include "../X2-Examples/licensedinterfaces/basiciniutilinterface.h"
#include "../X2-Examples/licensedinterfaces/loggerinterface.h"
#include "../X2-Examples/licensedinterfaces/mutexinterface.h"
#include "../X2-Examples/licensedinterfaces/tickcountinterface.h"

/*
 * SVBony SV241 Pro USB Power Hub — X2 Power Control Driver
 *
 * Circuit layout:
 *   Circuit 0 : DC Port 1     (12V switchable)
 *   Circuit 1 : DC Port 2     (12V switchable)
 *   Circuit 2 : DC Port 3     (12V switchable)
 *   Circuit 3 : DC Port 4     (12V switchable)
 *   Circuit 4 : Dew Heater 1  (PWM 0-100%)
 *   Circuit 5 : Dew Heater 2  (PWM 0-100%)
 *   Circuit 6 : USB Hub Power (switchable)
 *   Circuit 7 : Adjustable Output
 */

#define X2_NUM_CIRCUITS 8

class X2Svbony241Pro : public PowerControlDriverInterface
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
    virtual DeviceType              deviceType()                    { return PowerControlBox; }
    virtual int                     queryAbstraction(const char* pszName, void** ppVal);

    // -----------------------------------------------------------------------
    // DriverInfoInterface
    // -----------------------------------------------------------------------
    virtual void                    driverInfoDetailedInfo(BasicStringInterface& str) const;
    virtual double                  driverInfoVersion() const;

    // -----------------------------------------------------------------------
    // DeviceInfoInterface
    // -----------------------------------------------------------------------
    virtual void                    deviceInfoNameShort(BasicStringInterface& str) const;
    virtual void                    deviceInfoNameLong(BasicStringInterface& str) const;
    virtual void                    deviceInfoDetailedDescription(BasicStringInterface& str) const;
    virtual void                    deviceInfoFirmwareVersion(BasicStringInterface& str);
    virtual void                    deviceInfoModel(BasicStringInterface& str);

    // -----------------------------------------------------------------------
    // LinkInterface
    // -----------------------------------------------------------------------
    virtual int                     establishLink();
    virtual int                     terminateLink();
    virtual bool                    isLinked() const;

    // -----------------------------------------------------------------------
    // PowerControlDriverInterface
    // -----------------------------------------------------------------------
    virtual int                     numberOfCircuits(int& nNumber);
    virtual int                     circuitName(const int& nIndex, BasicStringInterface& str);
    virtual int                     circuitState(const int& nIndex, bool& bOn);
    virtual int                     setCircuitState(const int& nIndex, const bool& bOn);

private:
    // Send a command string to the device and read back a response line.
    // Returns SB_OK on success, non-zero on error.
    int sendCommand(const char* pszCmd, char* pszResp, const int nRespLen);

    // Read a single line of response from the device into pszResp.
    int readResponse(char* pszResp, const int nRespLen);

    // Query the device for the current state of all circuits and
    // populate m_bCircuitState[].
    int queryAllCircuitStates();

    // TSX-provided interfaces (not owned by this object — do not delete)
    SerXInterface*                      m_pSerX;
    TheSkyXFacadeForDriversInterface*   m_pTheSkyX;
    SleeperInterface*                   m_pSleeper;
    BasicIniUtilInterface*              m_pIniUtil;
    LoggerInterface*                    m_pLogger;
    MutexInterface*                     m_pIOMutex;
    TickCountInterface*                 m_pTickCount;

    int                                 m_nInstanceIndex;
    bool                                m_bLinked;

    // Cached on/off state for each circuit (indexed 0..X2_NUM_CIRCUITS-1)
    bool                                m_bCircuitState[X2_NUM_CIRCUITS];
};
