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

MRPT_TODO("fromVelodyne");
MRPT_TODO("fromScan2D");
MRPT_TODO("fromGenericObs");
MRPT_TODO("toPointCloud / calibration");

mrpt::system::TTimeStamp RotScan::getOriginalReceivedTimeStamp() const
{
	return originalReceivedTimestamp;
}

uint8_t RotScan::serializeGetVersion() const { return 0; }
void RotScan::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << timestamp << sensorLabel << rowCount << columnCount
		<< rangeResolution << startAzimuth << endAzimuth << sweepDuration
		<< lidarModel << minRange << maxRange << sensorPose
		<< originalReceivedTimestamp << has_satellite_timestamp;

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
}

void RotScan::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> timestamp >> sensorLabel >> rowCount >> columnCount >>
				rangeResolution >> startAzimuth >> endAzimuth >>
				sweepDuration >> lidarModel >> minRange >> maxRange >>
				sensorPose >> originalReceivedTimestamp >>
				has_satellite_timestamp;

			const auto nCols = in.ReadAs<uint16_t>(),
					   nRows = in.ReadAs<uint16_t>();
			rangeImage.resize(nRows, nCols);
			if (!rangeImage.empty())
				in.ReadBufferFixEndianness(
					&rangeImage(0, 0), rangeImage.size());

			{
				const auto nIntCols = in.ReadAs<uint16_t>(),
						   nIntRows = in.ReadAs<uint16_t>();
				intensityImage.resize(nIntRows, nIntCols);
				if (!intensityImage.empty())
					in.ReadBufferFixEndianness(
						&intensityImage(0, 0), intensityImage.size());
			}

			const auto nOtherLayers = in.ReadAs<uint16_t>();
			rangeOtherLayers.clear();
			for (size_t i = 0; i < nOtherLayers; i++)
			{
				std::string name;
				in >> name;
				auto& im = rangeOtherLayers[name];
				im.resize(nRows, nCols);
				in.ReadBufferFixEndianness(&im(0, 0), im.size());
			}
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

	o << "lidarModel: " << lidarModel << "\n";
	o << "Range rows=" << rowCount << " cols=" << columnCount << "\n";
	o << "Range resolution=" << rangeResolution << " [meter]\n";
	o << "Scan azimuth: start=" << mrpt::RAD2DEG(startAzimuth)
	  << " end=" << mrpt::RAD2DEG(endAzimuth) << "\n";
	o << "Sweep duration: " << sweepDuration << " [s]\n";
	o << mrpt::format(
		"Sensor min/max range: %.02f / %.02f m\n", minRange, maxRange);
	o << "has_satellite_timestamp: " << (has_satellite_timestamp ? "YES" : "NO")
	  << "\n";
	o << "originalReceivedTimestamp: "
	  << mrpt::system::dateTimeToString(originalReceivedTimestamp) << " (UTC)\n";
}


