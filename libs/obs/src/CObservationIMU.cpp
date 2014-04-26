/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/utils/CStream.h>
#include <mrpt/slam/CObservationIMU.h>
//#include <mrpt/math/CMatrixD.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationIMU, CObservation,mrpt::slam)

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationIMU::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 3;  // v1->v2 was only done to fix a bug in the ordering of YAW/PITCH/ROLL rates.
	else
	{
		out << sensorPose
		    << dataIsPresent
		    << timestamp;

		out << rawMeasurements;
		// Version 3: Added 6 new raw measurements (IMU_MAG_X=15 to IMU_TEMPERATURE=20)

		out << sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationIMU::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		in >> sensorPose;
		in >> dataIsPresent;

		in >> timestamp;

		// In version 0 it was a vector of floats:
		if (version<1)
		{
			mrpt::math::CVectorFloat	tmp;
			in >> tmp;
			rawMeasurements.resize(tmp.size());
			for (size_t i=0;i<rawMeasurements.size();i++)
				rawMeasurements[i] = tmp[i];
		}
		else
		{
			in >> rawMeasurements;
		}

		if (version<2)
		{
			// A bug in the grabbing from XSens IMU's made /ROLL rates to be stored in the wrong order:
			std::swap(rawMeasurements[IMU_YAW_VEL],rawMeasurements[IMU_ROLL_VEL]);
		}
		else
		{
			// v2: nothing to do, data is already in the right order.
		}

		in >> sensorLabel;

		// Version 3: Added 6 new raw measurements (IMU_MAG_X=15 to IMU_TEMPERATURE=20)
		if (version<3)
		{
			// Fill the last 6 entries with default values:
			const size_t nOld = dataIsPresent.size();
			ASSERT_(nOld==15)
			ASSERT_(rawMeasurements.size()==nOld)

			dataIsPresent.resize(COUNT_IMU_DATA_FIELDS);
			rawMeasurements.resize(COUNT_IMU_DATA_FIELDS);
			for (size_t i=nOld;i<COUNT_IMU_DATA_FIELDS;i++)
			{
				dataIsPresent[i]=false;
				rawMeasurements[i]=0;
			}
		}

		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

