/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

#include <mrpt/slam/CObservationRFID.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRFID, CObservation,mrpt::slam)

/** Constructor
 */
CObservationRFID::CObservationRFID() : tag_readings()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRFID::writeToStream(CStream &out, int *version) const
{
		//std::cout << "AP-1" << std::endl;
	MRPT_UNUSED_PARAM(out);
	if (version)
		*version = 4;
	else
	{
		// The data
		const uint32_t Ntags = tag_readings.size();
		out << Ntags;  // new in v4

        // (Fields are dumped in separate for loops for backward compatibility with old serialization versions)
		for (uint32_t i=0;i<Ntags;i++) out << tag_readings[i].power;
		for (uint32_t i=0;i<Ntags;i++) out << tag_readings[i].epc;
		for (uint32_t i=0;i<Ntags;i++) out << tag_readings[i].antennaPort;

		out	<< sensorLabel;
		out	<< timestamp;
		out	<< sensorPoseOnRobot; // Added in v3
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRFID::readFromStream(CStream &in, int version)
{
	//MRPT_UNUSED_PARAM(in);
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		{
		    uint32_t Ntags=0;
			if (version<4)
			{
				std::string ntags;
				in >> ntags;
				Ntags = atoi(ntags.c_str());
			}
			else
			{
				in >> Ntags;
			}

            // (Fields are read in separate for loops for backward compatibility with old serialization versions)
			tag_readings.resize(Ntags);
			for (uint32_t i=0;i<Ntags;i++) in >> tag_readings[i].power;
			for (uint32_t i=0;i<Ntags;i++) in >> tag_readings[i].epc;
			for (uint32_t i=0;i<Ntags;i++) in >> tag_readings[i].antennaPort;

			if (version>=1)
				in >> sensorLabel;
			else sensorLabel="";

			if (version>=2)
					in >> timestamp;
			else 	timestamp = INVALID_TIMESTAMP;

			if (version>=3)
					in >> sensorPoseOnRobot;
			else 	sensorPoseOnRobot = CPose3D();

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

}


void CObservationRFID::getSensorPose( CPose3D &out_sensorPose ) const
{
	out_sensorPose=sensorPoseOnRobot;
}

void CObservationRFID::setSensorPose( const CPose3D &newSensorPose )
{
	sensorPoseOnRobot = newSensorPose;
}

