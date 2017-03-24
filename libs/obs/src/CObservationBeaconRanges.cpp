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
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/system/os.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationBeaconRanges, CObservation,mrpt::obs)

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
void  CObservationBeaconRanges::writeToStream(mrpt::utils::CStream &out, int *version) const
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
void  CObservationBeaconRanges::readFromStream(mrpt::utils::CStream &in, int version)
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
	if (!sensedData.empty())
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

void CObservationBeaconRanges::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Auxiliary estimated pose (if available): " << auxEstimatePose << endl;

	o << format("minSensorDistance=%f m\n",minSensorDistance);
	o << format("maxSensorDistance=%f m\n",maxSensorDistance);
	o << format("stdError=%f m\n\n",stdError);

	o << format("There are %u range measurements:\n\n",(unsigned)sensedData.size());

	o << "  BEACON   RANGE     SENSOR POSITION ON ROBOT \n";
	o << "------------------------------------------------\n";
	for (deque<CObservationBeaconRanges::TMeasurement>::const_iterator it=sensedData.begin(); it!=sensedData.end(); it++)
	{
		o << format("   %i      %.04f      (%.03f,%.03f,%.03f)\n",
			(int)it->beaconID,it->sensedDistance,
			it->sensorLocationOnRobot.x(),it->sensorLocationOnRobot.y(),it->sensorLocationOnRobot.z());
	}
}
