#pragma once

#include "../X2-Examples/licensedinterfaces/basicstringinterface.h"
#include "../X2-Examples/licensedinterfaces/serxinterface.h"
#include "../X2-Examples/licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../X2-Examples/licensedinterfaces/sleeperinterface.h"
#include "../X2-Examples/licensedinterfaces/basiciniutilinterface.h"
#include "../X2-Examples/licensedinterfaces/loggerinterface.h"
#include "../X2-Examples/licensedinterfaces/mutexinterface.h"
#include "../X2-Examples/licensedinterfaces/tickcountinterface.h"

#ifdef _WIN32
#define PlugInExport __declspec(dllexport)
#else
#define PlugInExport
#endif

extern "C" PlugInExport int sbPlugInName2(BasicStringInterface& str);

extern "C" PlugInExport int sbPlugInFactory2(
    const char*                             pszDisplayName,
    const int&                              nInstanceIndex,
    SerXInterface*                          pSerXIn,
    TheSkyXFacadeForDriversInterface*       pTheSkyXIn,
    SleeperInterface*                       pSleeperIn,
    BasicIniUtilInterface*                  pIniUtilIn,
    LoggerInterface*                        pLoggerIn,
    MutexInterface*                         pIOMutexIn,
    TickCountInterface*                     pTickCountIn,
    void**                                  ppObjectOut);
