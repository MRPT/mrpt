/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservationRFID.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRFID, CObservation, mrpt::obs)

/** Constructor
 */
CObservationRFID::CObservationRFID() : tag_readings() {}
uint8_t CObservationRFID::serializeGetVersion() const { return 4; }
void CObservationRFID::serializeTo(mrpt::serialization::CArchive& out) const
{
	// The data
	const uint32_t Ntags = tag_readings.size();
	out << Ntags;  // new in v4

	// (Fields are dumped in separate for loops for backward compatibility
	// with old serialization versions)
	for (uint32_t i = 0; i < Ntags; i++) out << tag_readings[i].power;
	for (uint32_t i = 0; i < Ntags; i++) out << tag_readings[i].epc;
	for (uint32_t i = 0; i < Ntags; i++) out << tag_readings[i].antennaPort;

	out << sensorLabel;
	out << timestamp;
	out << sensorPoseOnRobot;  // Added in v3
}

void CObservationRFID::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	// MRPT_UNUSED_PARAM(in);
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		{
			uint32_t Ntags = 0;
			if (version < 4)
			{
				std::string ntags;
				in >> ntags;
				Ntags = atoi(ntags.c_str());
			}
			else
			{
				in >> Ntags;
			}

			// (Fields are read in separate for loops for backward compatibility
			// with old serialization versions)
			tag_readings.resize(Ntags);
			for (uint32_t i = 0; i < Ntags; i++) in >> tag_readings[i].power;
			for (uint32_t i = 0; i < Ntags; i++) in >> tag_readings[i].epc;
			for (uint32_t i = 0; i < Ntags; i++)
				in >> tag_readings[i].antennaPort;

			if (version >= 1)
				in >> sensorLabel;
			else
				sensorLabel = "";

			if (version >= 2)
				in >> timestamp;
			else
				timestamp = INVALID_TIMESTAMP;

			if (version >= 3)
				in >> sensorPoseOnRobot;
			else
				sensorPoseOnRobot = CPose3D();
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationRFID::getSensorPose(CPose3D& out_sensorPose) const
{
	out_sensorPose = sensorPoseOnRobot;
}

void CObservationRFID::setSensorPose(const CPose3D& newSensorPose)
{
	sensorPoseOnRobot = newSensorPose;
}

void CObservationRFID::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);

	o << "Number of RFID tags sensed: " << tag_readings.size() << "\n";
	for (size_t i = 0; i < tag_readings.size(); i++)
	{
		const auto& rfid = tag_readings[i];
		o << "#" << i << ": Power=" << rfid.power
		  << " (dBm) | AntennaPort=" << rfid.antennaPort
		  << " | EPC=" << rfid.epc << std::endl;
	}
}
