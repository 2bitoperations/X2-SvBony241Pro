#include "main.h"
#include "x2svbony241pro.h"

extern "C" PlugInExport int sbPlugInName2(BasicStringInterface& str)
{
    str = "SVBony SV241 Pro USB Power Hub";
    return SB_OK;
}

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
    void**                                  ppObjectOut)
{
    *ppObjectOut = NULL;

    X2Svbony241Pro* pObj = new X2Svbony241Pro(
        pszDisplayName,
        nInstanceIndex,
        pSerXIn,
        pTheSkyXIn,
        pSleeperIn,
        pIniUtilIn,
        pLoggerIn,
        pIOMutexIn,
        pTickCountIn);

    if (pObj != NULL)
        *ppObjectOut = dynamic_cast<PowerControlDriverInterface*>(pObj);

    return SB_OK;
}
