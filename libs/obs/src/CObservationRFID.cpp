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

using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRFID, CObservation,mrpt::slam)

/** Constructor
 */
CObservationRFID::CObservationRFID()
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
		*version = 3;
	else
	{
		//std::cout << "AP0" << std::endl;
		

		// The data
		char buff[3];
		out << std::string(itoa(Ntags,buff,10));
		//std::cout << "AP1: " << std::string(itoa(Ntags,buff,10)) << std::endl;

		int i;
		for (i=0;i<Ntags;i++){
			out << power[i];
			//std::cout << "AP2: " << power[i] << std::endl;
		}

		for (i=0;i<Ntags;i++){
			out	<< epc[i];
			//std::cout << "AP3: " << epc[i] << std::endl;
		}
		
		for (i=0;i<Ntags;i++){
			out	<< antennaPort[i];
			//std::cout << "AP4: " << antennaPort[i] << std::endl;
		}

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
		{
			std::string ntags;
			
			in >> ntags;
			Ntags = atoi(ntags.c_str());
			int i;
			//std::cout << "P1: " << Ntags << std::endl;
			 double power_tmp;
			for (i=0;i<Ntags;i++)
			{
				in	>> power_tmp;
				power.push_back(power_tmp);
				//std::cout << power_tmp << std::endl;
			}
			//std::cout << "P2: ";
			char epc_tmp[40];
			for (i=0;i<Ntags;i++)
			{
				//std::cout << "a ";
				in >> epc_tmp;
				//in >> epc[i];
				//std::cout << "b: " << std::string(epc_tmp) << " ";
				epc.push_back(std::string(epc_tmp));
				//std::cout << "c ";
				free(epc_tmp);
				//std::cout << "d " << std::endl;
			}
			//std::cout << "P3" << std::endl;
			std::string ant_tmp;
			for (i=0;i<Ntags;i++)
			{
				in >> ant_tmp;
				antennaPort.push_back(ant_tmp);
				ant_tmp.clear();
			}
			//std::cout << "P4" << std::endl;
			
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

