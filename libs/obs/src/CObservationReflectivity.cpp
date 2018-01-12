/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservationReflectivity.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationReflectivity, CObservation, mrpt::obs)

uint8_t CObservationReflectivity::serializeGetVersion() const { return 1; }
void CObservationReflectivity::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << reflectivityLevel << channel << sensorPose;
	out << sensorLabel << timestamp;
}

void CObservationReflectivity::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			in >> reflectivityLevel;
			if (version >= 1) in >> channel;
			in >> sensorPose;
			in >> sensorLabel >> timestamp;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationReflectivity::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);

	o << "reflectivityLevel=" << reflectivityLevel << std::endl;
	o << "channel=" << channel << " (-1=any)" << std::endl;
}
