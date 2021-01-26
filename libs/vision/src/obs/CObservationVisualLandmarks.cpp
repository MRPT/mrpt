/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"	 // Precompiled headers
//
#include <mrpt/obs/CObservationVisualLandmarks.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationVisualLandmarks, CObservation, mrpt::obs)

/** Constructor
 */
CObservationVisualLandmarks::CObservationVisualLandmarks()
	: refCameraPose(), landmarks()
{
}

uint8_t CObservationVisualLandmarks::serializeGetVersion() const { return 1; }
void CObservationVisualLandmarks::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << refCameraPose
		<< timestamp
		// The landmarks:
		<< landmarks << sensorLabel;
}

void CObservationVisualLandmarks::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		{
			in >> refCameraPose >> timestamp

				// The landmarks:
				>> landmarks;

			if (version > 0) in >> sensorLabel;
			else
				sensorLabel = "";
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
 Inserts a pure virtual method for finding the likelihood between this
   and another observation, probably only of the same derived class. The
 operator
   may be asymmetric.

 \param anotherObs The other observation to compute likelihood with.
 \param anotherObsPose If known, the belief about the robot pose when the other
 observation was taken can be supplied here, or nullptr if it is unknown.

 \return Returns a likelihood measurement, in the range [0,1].
 \exception std::exception On any error, as another observation being of an
 invalid class.
  ---------------------------------------------------------------*/
float CObservationVisualLandmarks::likelihoodWith(
	[[maybe_unused]] const CObservation* anotherObs,
	[[maybe_unused]] const CPosePDF* anotherObsPose) const
{
	return 0;
}

void CObservationVisualLandmarks::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);
}
