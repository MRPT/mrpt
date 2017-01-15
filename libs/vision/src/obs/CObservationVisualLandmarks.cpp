/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationVisualLandmarks.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils; 
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationVisualLandmarks, CObservation,mrpt::obs)

/** Constructor
 */
CObservationVisualLandmarks::CObservationVisualLandmarks( ) :
	refCameraPose(),
	landmarks()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationVisualLandmarks::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		out << refCameraPose
			<< timestamp
		// The landmarks:
			<< landmarks
			<< sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationVisualLandmarks::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			in 	>> refCameraPose
				>> timestamp

			// The landmarks:
				>> landmarks;

			if (version>0)
					in >> sensorLabel;
			else	sensorLabel = "";

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
 Inserts a pure virtual method for finding the likelihood between this
   and another observation, probably only of the same derived class. The operator
   may be asymmetric.

 \param anotherObs The other observation to compute likelihood with.
 \param anotherObsPose If known, the belief about the robot pose when the other observation was taken can be supplied here, or NULL if it is unknown.

 \return Returns a likelihood measurement, in the range [0,1].
 \exception std::exception On any error, as another observation being of an invalid class.
  ---------------------------------------------------------------*/
float  CObservationVisualLandmarks::likelihoodWith(
	const CObservation		*anotherObs,
	const CPosePDF			*anotherObsPose ) const
{
	MRPT_UNUSED_PARAM(anotherObs); MRPT_UNUSED_PARAM(anotherObsPose);
	return 0;
}

void CObservationVisualLandmarks::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

}

