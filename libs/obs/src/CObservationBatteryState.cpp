/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs.h>   // Precompiled headers 



#include <mrpt/slam/CObservationBatteryState.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;

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

