/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers


#include <mrpt/utils/CStream.h>
#include <mrpt/obs/CObservation6DFeatures.h>
#include <mrpt/system/os.h>
#include <mrpt/math/matrix_serialization.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation6DFeatures, CObservation,mrpt::obs)

/** Default constructor */
CObservation6DFeatures::CObservation6DFeatures( ) :
	minSensorDistance ( 0 ),
	maxSensorDistance ( 1e6f )
{
}

CObservation6DFeatures::TMeasurement::TMeasurement() :
	id(INVALID_LANDMARK_ID)
{
}


/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation6DFeatures::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << minSensorDistance << maxSensorDistance << sensorPose;

		const uint32_t n = sensedFeatures.size();
		out << n;
		for (uint32_t i=0;i<n;i++) 
		{
			const TMeasurement & m = sensedFeatures[i];
			out << m.pose << m.id << m.inf_matrix;
		}

		out << sensorLabel
			<< timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation6DFeatures::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in >> minSensorDistance >> maxSensorDistance >> sensorPose;

			uint32_t n;
			in >> n;
			sensedFeatures.clear();
			sensedFeatures.resize(n);
			for (uint32_t i=0;i<n;i++) 
			{
				TMeasurement & m = sensedFeatures[i];
				in >> m.pose >> m.id >> m.inf_matrix;
			}

			in >> sensorLabel
			   >> timestamp;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

}

void CObservation6DFeatures::getSensorPose( CPose3D &out_sensorPose ) const 
{ 
	out_sensorPose = sensorPose; 
}

void CObservation6DFeatures::setSensorPose( const CPose3D &newSensorPose ) 
{ 
	sensorPose = newSensorPose; 
}

void CObservation6DFeatures::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Sensor pose: " << sensorPose << endl;
	o << "Min range  : " << minSensorDistance << endl;
	o << "Max range  : " << maxSensorDistance << endl << endl;

	o << "Observation count : " << sensedFeatures.size() << endl << endl;

	for (size_t k=0;k<sensedFeatures.size();k++)
	{
		const CObservation6DFeatures::TMeasurement & m = sensedFeatures[k];
		o << "#" << k << ": ID=" << m.id << "; value=" << m.pose << "; inf=" <<m.inf_matrix.inMatlabFormat() << endl;
	}
}
