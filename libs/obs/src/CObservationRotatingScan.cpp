/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <iostream>

using namespace std;
using namespace mrpt::obs;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRotatingScan, CObservation, mrpt::obs)

// static CSinCosLookUpTableFor2DScans velodyne_sincos_tables;

using RotScan = CObservationRotatingScan;

mrpt::system::TTimeStamp RotScan::getOriginalReceivedTimeStamp() const
{
	return originalReceivedTimestamp;
}

uint8_t RotScan::serializeGetVersion() const { return 0; }
void RotScan::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << timestamp << sensorLabel << rowCount << columnCount;

	out.WriteAs<uint16_t>(rangeImage.cols());
	out.WriteAs<uint16_t>(rangeImage.rows());
	if (!rangeImage.empty())
		out.WriteBufferFixEndianness(&rangeImage(0, 0), rangeImage.size());

	out.WriteAs<uint16_t>(intensityImage.cols());
	out.WriteAs<uint16_t>(intensityImage.rows());
	if (!intensityImage.empty())
		out.WriteBufferFixEndianness(
			&intensityImage(0, 0), intensityImage.size());

	out.WriteAs<uint16_t>(rangeOtherLayers.size());
	for (const auto& ly : rangeOtherLayers)
	{
		out << ly.first;
		ASSERT_EQUAL_(ly.second.cols(), columnCount);
		ASSERT_EQUAL_(ly.second.rows(), rowCount);
		out.WriteBufferFixEndianness(&ly.second(0, 0), ly.second.size());
	}

	out << rangeResolution << startAzimuth << endAzimuth << sweepDuration
		<< lidarModel << minRange << maxRange << sensorPose
		<< originalReceivedTimestamp << has_satellite_timestamp;
}

void RotScan::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> timestamp >> sensorLabel;
#if 0

			in >> minRange >> maxRange >> sensorPose;
			{
				uint32_t N;
				in >> N;
				scan_packets.resize(N);
				if (N)
					in.ReadBuffer(
						&scan_packets[0], sizeof(scan_packets[0]) * N);
			}
			{
				uint32_t N;
				in >> N;
				calibration.laser_corrections.resize(N);
				if (N)
					in.ReadBuffer(
						&calibration.laser_corrections[0],
						sizeof(calibration.laser_corrections[0]) * N);
			}
			point_cloud.clear();
			in >> point_cloud.x >> point_cloud.y >> point_cloud.z >>
				point_cloud.intensity;
			if (version >= 1)
				in >> has_satellite_timestamp;
			else
				has_satellite_timestamp =
					(this->timestamp != this->originalReceivedTimestamp);
			if (version >= 2)
				in >> point_cloud.timestamp >> point_cloud.azimuth >>
					point_cloud.laser_id >> point_cloud.pointsForLaserID;
#endif
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void RotScan::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);
	o << "Homogeneous matrix for the sensor 3D pose, relative to "
		 "robot base:\n";
	o << sensorPose.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>()
	  << "\n"
	  << sensorPose << endl;
	o << format("Sensor min/max range: %.02f / %.02f m\n", minRange, maxRange);
	MRPT_TODO("continue");
}
