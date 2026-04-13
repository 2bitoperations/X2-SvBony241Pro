#ifndef _CircuitLabelsInterface_H
#define _CircuitLabelsInterface_H

#define CircuitLabelsInterface_Name "com.bisque.TheSkyX.CircuitLabelsInterface/1.0"

class BasicStringInterface;

/*!
\brief The CircuitLabelsInterface allows power control drivers to provide TheSky a text label assoicated with each circuit.

\ingroup Interface

This interface is optional.  By default, TheSky will control the text labels assoicated with each ciricuit of a power control device.  If the power control
device is capable and it and controls text labels associated with each circuit, support this interface.

Don't forget to respond accordingly in your queryAbstraction().
*/

class CircuitLabelsInterface
{
public:

	virtual ~CircuitLabelsInterface(){}

public:
	//CircuitLabelsInterface
	/*! Called by TheSky query (read) the text label of a circuit.
	\param nZeroBasedIndex (in) the zero based index of the circuit.
	\param str (out) the text label associated with this circuit.
	*/
	virtual int circuitLabel(const int& nZeroBasedIndex, BasicStringInterface& str) = 0;

};

#endif