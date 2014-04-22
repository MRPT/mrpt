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
#include <mrpt/slam/CObservationBeaconRanges.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationBeaconRanges, CObservation,mrpt::slam)

/** Default constructor.
 */
CObservationBeaconRanges::CObservationBeaconRanges( ) :
	minSensorDistance ( 0 ),
	maxSensorDistance ( 1e2f ),
	stdError ( 1e-2f ),
	sensedData(),
	auxEstimatePose()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBeaconRanges::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 3;
	else
	{
		uint32_t	i,n;

		// The data
		out << minSensorDistance << maxSensorDistance << stdError;

		n = sensedData.size();
		out << n;
		for (i=0;i<n;i++)
			out << sensedData[i].sensorLocationOnRobot << sensedData[i].sensedDistance << sensedData[i].beaconID;

		out << auxEstimatePose;

		out << sensorLabel
			<< timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBeaconRanges::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			uint32_t		i,n,id;

			// The data
			in >> minSensorDistance >> maxSensorDistance >> stdError;

			in >> n;
			sensedData.resize(n);
			for (i=0;i<n;i++)
			{
				in >> sensedData[i].sensorLocationOnRobot >> sensedData[i].sensedDistance;
				in >> id; sensedData[i].beaconID = id;
			}

			if (version>=1)
				in >> auxEstimatePose;

			if (version>=2)
					in >> sensorLabel;
			else 	sensorLabel="";

			if (version>=3)
					in >> timestamp;
			else 	timestamp = INVALID_TIMESTAMP;


		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationBeaconRanges::debugPrintOut()
{
	printf("[CObservationBeaconRanges::debugPrintOut] Dumping:\n");
	printf("[CObservationBeaconRanges::debugPrintOut] minSensorDistance:\t%f\n",minSensorDistance);
	printf("[CObservationBeaconRanges::debugPrintOut] maxSensorDistance:\t%f:\n",maxSensorDistance);
	printf("[CObservationBeaconRanges::debugPrintOut] stdError:\t%f\n",stdError);
	printf("[CObservationBeaconRanges::debugPrintOut] %u ranges:\n",static_cast<unsigned>( sensedData.size() ));

	size_t		i, n = sensedData.size();
	for (i=0;i<n;i++)
		printf("[CObservationBeaconRanges::debugPrintOut] \tID[%u]: %f\n",
		sensedData[i].beaconID,
		sensedData[i].sensedDistance );
}

/*---------------------------------------------------------------
                     getSensorPose
 ---------------------------------------------------------------*/
void CObservationBeaconRanges::getSensorPose( CPose3D &out_sensorPose ) const
{
	if (sensedData.size())
		out_sensorPose=sensedData[0].sensorLocationOnRobot;
	else 	out_sensorPose = CPose3D(0,0,0);
}

/*---------------------------------------------------------------
                     setSensorPose
 ---------------------------------------------------------------*/
void CObservationBeaconRanges::setSensorPose( const CPose3D &newSensorPose )
{
	size_t		i, n = sensedData.size();
	if (n)
		for (i=0;i<n;i++)
			sensedData[i].sensorLocationOnRobot=CPoint3D(newSensorPose);
}

/*---------------------------------------------------------------
                     getSensedRangeByBeaconID
 ---------------------------------------------------------------*/
float CObservationBeaconRanges::getSensedRangeByBeaconID(int32_t beaconID)
{
	for (size_t i=0;i<sensedData.size();i++)
		if (sensedData[i].beaconID==beaconID)
			return sensedData[i].sensedDistance;
	return 0;
}
