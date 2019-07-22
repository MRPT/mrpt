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
	MRPT_TODO("continue!");
#if 0
	out << timestamp << sensorLabel;
	out << minRange << maxRange << sensorPose;
	out.WriteAs<uint32_t>(scan_packets.size());
	if (!scan_packets.empty())
		out.WriteBuffer(
			&scan_packets[0], sizeof(scan_packets[0]) * scan_packets.size());
	out.WriteAs<uint32_t>(calibration.laser_corrections.size());
	if (!calibration.laser_corrections.empty())
		out.WriteBuffer(
			&calibration.laser_corrections[0],
			sizeof(calibration.laser_corrections[0]) *
				calibration.laser_corrections.size());
	out << point_cloud.x << point_cloud.y << point_cloud.z
		<< point_cloud.intensity;
	out << has_satellite_timestamp;  // v1
	// v2:
	out << point_cloud.timestamp << point_cloud.azimuth << point_cloud.laser_id
		<< point_cloud.pointsForLaserID;
#endif
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
