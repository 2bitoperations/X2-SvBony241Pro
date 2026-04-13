#ifndef _PowerControlDriverInterface_H
#define _PowerControlDriverInterface_H

#ifdef THESKYX_FOLDER_TREE
#include "imagingsystem/hardware/interfaces/licensed/driverrootinterface.h"
#include "imagingsystem/hardware/interfaces/licensed/linkinterface.h"
#include "imagingsystem/hardware/interfaces/licensed/deviceinfointerface.h"
#include "imagingsystem/hardware/interfaces/licensed/driverinfointerface.h"
#elif X2_FLAT_INCLUDES
#include "driverrootinterface.h"
#include "linkinterface.h"
#include "deviceinfointerface.h"
#include "driverinfointerface.h"
#else
#include "../../licensedinterfaces/driverrootinterface.h"
#include "../../licensedinterfaces/linkinterface.h"
#include "../../licensedinterfaces/deviceinfointerface.h"
#include "../../licensedinterfaces/driverinfointerface.h"
#endif

/*!
\brief The PowerControlDriverInterface allows an X2 implementor to a write X2 power control driver.

\ingroup Driver

See the X2PowerControl for an example.

Power control capabilities were added to TheSky build 12391 and later, released roughly in October 2019.  Use the TheSkyXFacadeForDriversInterface::build() method to react by either requiring a certain minimum build or gracefully degrading functionality.

\sa The related, optional interface CircuitLabelsInterface
\sa The related, optional interface SetCircuitLabelsInterface
*/
class PowerControlDriverInterface : public DriverRootInterface, public LinkInterface, public HardwareInfoInterface, public DriverInfoInterface
{
public:
	virtual ~PowerControlDriverInterface(){}

	/*!\name DriverRootInterface Implementation
	See DriverRootInterface.*/
	//@{ 
	virtual DeviceType							deviceType(void)							  {return DriverRootInterface::DT_POWERCONTROL;}
	virtual int									queryAbstraction(const char* pszName, void** ppVal) = 0;
	//@} 

	/*!\name LinkInterface Implementation
	See LinkInterface.*/
	//@{ 
	virtual int									establishLink(void) = 0;
	virtual int									terminateLink(void) = 0;
	virtual bool								isLinked(void) const = 0;
	virtual bool								isEstablishLinkAbortable(void) const { return false; }
	//@} 

	/*!\name DriverInfoInterface Implementation
	See DriverInfoInterface.*/
	//@{ 
	virtual void								driverInfoDetailedInfo(BasicStringInterface& str) const = 0;
	virtual double								driverInfoVersion(void) const = 0;
	//@} 

	//
	/*!\name HardwareInfoInterface Implementation
	See HardwareInfoInterface.*/
	//@{ 
	virtual void deviceInfoNameShort(BasicStringInterface& str) const = 0;
	virtual void deviceInfoNameLong(BasicStringInterface& str) const = 0;
	virtual void deviceInfoDetailedDescription(BasicStringInterface& str) const = 0;
	virtual void deviceInfoFirmwareVersion(BasicStringInterface& str) = 0;
	virtual void deviceInfoModel(BasicStringInterface& str) = 0;
	//@} 

	/*! Return the number of circuits (outlets) that can be controlled by this power control device.
	\param nNumber (out) the number of circuits controlled by this power control device.  This method will be called once after a successfull establishLink().
	*/
	virtual int numberOfCircuits(int& nNumber)
	{
		nNumber = 0;
		return 0;
	}

	/*! Called by TheSky to determine (read) the current state of a circuit.
	\param nZeroBasedIndex (in) the index of the circuit, starting at zero to 1-number of circuits.
	\param bZeroForOffOneForOn (out) the current state of a circuit. Set bZeroForOffOneForOn to zero (0) to indcate the circuit is off and one (1) to indcate the circuit is on. 
	*/
	virtual int circuitState(const int& nZeroBasedIndex, bool& bZeroForOffOneForOn)
	{
		(void)nZeroBasedIndex;
		bZeroForOffOneForOn = 0;
		return 0;
	}

	/*! Called by TheSky to set the state of a circuit.
	\param nZeroBasedIndex (in) the index of the circuit, starting at zero to 1-number of circuits.
	\param bZeroForOffOneForOn (in) the desired state of a circuit. bZeroForOffOneForOn will be zero (0) to set the circuit off and one (1) to set the circuit on.
	*/
	virtual int setCircuitState(const int& nZeroBasedIndex, const bool& bZeroForOffOneForOn)
	{
		(void)nZeroBasedIndex;
		(void)bZeroForOffOneForOn;
		return 0;
	}
};

#endif