/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers


#include <mrpt/utils/CStream.h>
#include <mrpt/slam/CObservationBatteryState.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationBatteryState, CObservation,mrpt::slam)

/** Constructor
 */
CObservationBatteryState::CObservationBatteryState( ) :
	 voltageMainRobotBattery(0),
	 voltageMainRobotComputer(0),
	 voltageMainRobotBatteryIsValid(false),
	 voltageMainRobotComputerIsValid(false),
	 voltageOtherBatteries(),
	 voltageOtherBatteriesValid()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBatteryState::writeToStream(CStream &out, int *version) const
{
	MRPT_UNUSED_PARAM(out);
	if (version)
		*version = 2;
	else
	{
		// The data
		out << voltageMainRobotBattery
			<< voltageMainRobotComputer
			<< voltageMainRobotBatteryIsValid
			<< voltageMainRobotComputerIsValid
			<< voltageOtherBatteries
			<< voltageOtherBatteriesValid
			<< sensorLabel
			<< timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBatteryState::readFromStream(CStream &in, int version)
{
	MRPT_UNUSED_PARAM(in);
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			in	>> voltageMainRobotBattery
				>> voltageMainRobotComputer
				>> voltageMainRobotBatteryIsValid
				>> voltageMainRobotComputerIsValid
				>> voltageOtherBatteries
				>> voltageOtherBatteriesValid;
			if (version>=1)
				in >> sensorLabel;
			else sensorLabel="";

			if (version>=2)
					in >> timestamp;
			else 	timestamp = INVALID_TIMESTAMP;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

