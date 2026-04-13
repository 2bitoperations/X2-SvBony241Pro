#pragma once

#include "licensedinterfaces/basicstringinterface.h"
#include "licensedinterfaces/serxinterface.h"
#include "licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "licensedinterfaces/sleeperinterface.h"
#include "licensedinterfaces/basiciniutilinterface.h"
#include "licensedinterfaces/loggerinterface.h"
#include "licensedinterfaces/mutexinterface.h"
#include "licensedinterfaces/tickcountinterface.h"

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
