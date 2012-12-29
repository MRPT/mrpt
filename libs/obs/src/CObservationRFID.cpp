/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

