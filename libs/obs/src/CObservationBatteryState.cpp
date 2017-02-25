/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers


#include <mrpt/utils/CStream.h>
#include <mrpt/obs/CObservationBatteryState.h>
#include <mrpt/system/os.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationBatteryState, CObservation,mrpt::obs)

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
void  CObservationBatteryState::writeToStream(mrpt::utils::CStream &out, int *version) const
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
void  CObservationBatteryState::readFromStream(mrpt::utils::CStream &in, int version)
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

// See base class docs
void CObservationBatteryState::getSensorPose( CPose3D &out_sensorPose ) const { 
	out_sensorPose=CPose3D(0,0,0); 
}
		
// See base class docs
void CObservationBatteryState::setSensorPose( const CPose3D &newSensorPose ) {
	MRPT_UNUSED_PARAM(newSensorPose);
}

void CObservationBatteryState::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	o << format("Measured VoltageMainRobotBattery: %.02fV  isValid= %s \n",
		voltageMainRobotBattery,
		(voltageMainRobotBatteryIsValid == true)? "True":"False" );

	o << format("Measured VoltageMainRobotComputer: %.02fV  isValid= %s \n",
		voltageMainRobotComputer,
		(voltageMainRobotComputerIsValid == true)? "True":"False" );

	o << "VoltageOtherBatteries: \n";
	for(CVectorDouble::Index i=0; i<voltageOtherBatteries.size(); i++)
	{
		o << format("Index: %d --> %.02fV  isValid= %s \n",
		int(i),
		voltageOtherBatteries[i],
		(voltageOtherBatteriesValid[i] == true)? "True":"False" );
	}

}
