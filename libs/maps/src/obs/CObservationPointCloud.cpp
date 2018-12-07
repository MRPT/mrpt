/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/serialization/CArchive.h>
#include <iostream>

using namespace mrpt::obs;

IMPLEMENTS_SERIALIZABLE(CObservationPointCloud, CObservation, mrpt::obs);

CObservationPointCloud::CObservationPointCloud(const CObservation3DRangeScan& o)
{
	pointcloud = mrpt::maps::CSimplePointsMap::Create();
	pointcloud->loadFromRangeScan(o);
}

void CObservationPointCloud::getSensorPose(
	mrpt::poses::CPose3D& out_sensorPose) const
{
	out_sensorPose = sensorPose;
}
void CObservationPointCloud::setSensorPose(const mrpt::poses::CPose3D& p)
{
	sensorPose = p;
}
void CObservationPointCloud::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);
	o << "Homogeneous matrix for the sensor pose wrt vehicle:\n";
	o << sensorPose.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>()
	  << sensorPose << std::endl;

	o << "Pointcloud class: ";
	if (!this->pointcloud)
	{
		o << "nullptr\n";
	}
	else
	{
		o << pointcloud->GetRuntimeClass()->className << "\n";
		o << "Number of points: " << pointcloud->size() << "\n";
	}
}

uint8_t CObservationPointCloud::serializeGetVersion() const { return 0; }
void CObservationPointCloud::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << sensorLabel << timestamp;  // Base class data

	out << sensorPose;
	out << pointcloud;
}

void CObservationPointCloud::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			pointcloud.reset();
			in >> sensorLabel >> timestamp;  // Base class data

			in >> sensorPose;
			in >> pointcloud;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}
