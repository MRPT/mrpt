/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationRFID.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRFID, CObservation,mrpt::obs)

/** Constructor
 */
CObservationRFID::CObservationRFID() : tag_readings()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRFID::writeToStream(mrpt::utils::CStream &out, int *version) const
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
void  CObservationRFID::readFromStream(mrpt::utils::CStream &in, int version)
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

void CObservationRFID::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	std::cout << "Number of RFID tags sensed: " << tag_readings.size() << std::endl << std::endl;

	for (size_t i=0;i<tag_readings.size();i++)
	{
		const CObservationRFID::TTagReading &rfid = tag_readings[i];
		std::cout << "#"<< i
			<< ": Power=" << rfid.power
			<< " (dBm) | AntennaPort=" << rfid.antennaPort
			<< " | EPC=" << rfid.epc << std::endl;
	}
}

