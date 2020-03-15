/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>

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
	out << timestamp << sensorLabel << rowCount << columnCount
		<< rangeResolution << startAzimuth << azimuthSpan << sweepDuration
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
				rangeResolution >> startAzimuth >> azimuthSpan >>
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
	  << " span=" << mrpt::RAD2DEG(azimuthSpan) << "\n";
	o << "Sweep duration: " << sweepDuration << " [s]\n";
	o << mrpt::format(
		"Sensor min/max range: %.02f / %.02f m\n", minRange, maxRange);
	o << "has_satellite_timestamp: " << (has_satellite_timestamp ? "YES" : "NO")
	  << "\n";
	o << "originalReceivedTimestamp: "
	  << mrpt::system::dateTimeToString(originalReceivedTimestamp)
	  << " (UTC)\n";
}

MRPT_TODO("toPointCloud / calibration");

void RotScan::fromVelodyne(const mrpt::obs::CObservationVelodyneScan& o)
{
	using Velo = mrpt::obs::CObservationVelodyneScan;
	using degree_cents = uint16_t;
	using gps_microsecs = uint32_t;

	MRPT_START

	// Reset:
	*this = CObservationRotatingScan();

	// Copy properties:
	has_satellite_timestamp = o.has_satellite_timestamp;
	originalReceivedTimestamp = o.originalReceivedTimestamp;
	timestamp = o.timestamp;
	sensorPose = o.sensorPose;
	sensorLabel = o.sensorLabel;
	minRange = o.minRange;
	maxRange = o.maxRange;

	// Convert ranges to range images:

	uint8_t model = 0;
	gps_microsecs last_pkt_tim = std::numeric_limits<gps_microsecs>::max();
	degree_cents last_pkt_az = 0;  // last azimuth

	// Azimuth (wrt sensor) at the beginning of one packet, to its timestamp:
	std::map<degree_cents, gps_microsecs> azimuth2timestamp;
	std::multiset<double> rotspeed;  // per packet estimated rot speed (deg/sec)

	// column count:
	const size_t num_lasers = o.calibration.laser_corrections.size();
	ASSERT_ABOVE_(num_lasers, 2);
	rowCount = num_lasers;

	// row count:
	columnCount =
		Velo::SCANS_PER_BLOCK * o.scan_packets.size() * Velo::BLOCKS_PER_PACKET;

	const double timeBetweenLastTwoBlocks =
		1e-6 * (o.scan_packets.rbegin()->gps_timestamp() -
				(o.scan_packets.rbegin() + 1)->gps_timestamp());

	rangeImage.setZero(rowCount, columnCount);
	intensityImage.setZero(rowCount, columnCount);
	rangeOtherLayers.clear();
	rangeResolution = Velo::DISTANCE_RESOLUTION;
	azimuthSpan = 0;

	ASSERT_ABOVEEQ_(o.scan_packets.size(), 1);

	for (size_t pktIdx = 0; pktIdx < o.scan_packets.size(); pktIdx++)
	{
		const auto& pkt = o.scan_packets[pktIdx];

		model = pkt.velodyne_model_ID;

		const degree_cents pkt_azimuth = pkt.blocks[0].rotation();

		if (last_pkt_tim != std::numeric_limits<uint32_t>::max())
		{
			// Estimate rot speed:
			ASSERT_ABOVE_(pkt.gps_timestamp(), last_pkt_tim);
			const double dT = 1e-6 * (pkt.gps_timestamp() - last_pkt_tim);
			const auto dAzimuth = 1e-2 * (pkt_azimuth - last_pkt_az);
			const auto estRotVel = dAzimuth / dT;
			rotspeed.insert(estRotVel);
		}
		last_pkt_tim = pkt.gps_timestamp();
		last_pkt_az = pkt_azimuth;

		azimuth2timestamp[pkt_azimuth] = pkt.gps_timestamp();

		// Accum azimuth span:
		if (pktIdx + 1 == o.scan_packets.size())
		{
			// last packet:
			// sanity checks: rot speed should be fairly stable:
			const double maxRotSpeed = *rotspeed.rbegin(),
						 minRotSpeed = *rotspeed.begin();
			ASSERT_ABOVE_(maxRotSpeed, 0);
			ASSERT_BELOW_((maxRotSpeed - minRotSpeed) / maxRotSpeed, 0.01);

			// Median speed:
			const double rotVel_degps = [&]() {
				auto it = rotspeed.begin();
				std::advance(it, rotspeed.size() / 2);
				return *it;
			}();

			azimuthSpan +=
				mrpt::DEG2RAD(rotVel_degps * timeBetweenLastTwoBlocks);
		}
		else
		{
			// non-last packet:
			const double curAng =
				0.01 * o.scan_packets[pktIdx].blocks[0].rotation();
			const double nextAng =
				0.01 * o.scan_packets[pktIdx + 1].blocks[0].rotation();

			const double incrAng = mrpt::math::angDistance(
				mrpt::DEG2RAD(curAng), mrpt::DEG2RAD(nextAng));
			azimuthSpan += incrAng;
		}

		// Process each block in this packet:
		for (int block = 0; block < Velo::BLOCKS_PER_PACKET; block++)
		{
			const int dsr_offset =
				(pkt.blocks[block].header() == Velo::LOWER_BANK) ? 32 : 0;
			const bool block_is_dual_strongest_range =
				(pkt.laser_return_mode == Velo::RETMODE_DUAL &&
				 ((block & 0x01) != 0));
			const bool block_is_dual_last_range =
				(pkt.laser_return_mode == Velo::RETMODE_DUAL &&
				 ((block & 0x01) == 0));

			for (int dsr = 0, k = 0; dsr < Velo::SCANS_PER_FIRING; dsr++, k++)
			{
				if (!pkt.blocks[block]
						 .laser_returns[k]
						 .distance())  // Invalid return?
					continue;

				const auto rawLaserId = static_cast<uint8_t>(dsr + dsr_offset);
				uint8_t laserId = rawLaserId;

				// Detect VLP-16 data and adjust laser id if necessary
				// bool firingWithinBlock = false;
				if (num_lasers == 16)
				{
					if (laserId >= 16)
					{
						laserId -= 16;
						// firingWithinBlock = true;
					}
				}

				ASSERT_BELOW_(laserId, num_lasers);
				const auto& calib = o.calibration.laser_corrections[laserId];

				// In dual return, if the distance is equal in both ranges,
				// ignore one of them:

				const auto distance =
					pkt.blocks[block].laser_returns[k].distance() +
					static_cast<uint16_t>(
						calib.distanceCorrection / Velo::DISTANCE_RESOLUTION);

				const auto columnIdx = [&]() {
					switch (num_lasers)
					{
						case 16:
						case 32:
						case 64:
						{
							int c = (dsr + block * Velo::SCANS_PER_BLOCK +
									 pktIdx * Velo::SCANS_PER_PACKET) /
									num_lasers;
							if (pkt.laser_return_mode == Velo::RETMODE_DUAL)
								c /= 2;
							return c;
						}
						default:
						{
							THROW_EXCEPTION("Error: unhandled LIDAR model!");
						}
					};
				}();

				ASSERT_BELOW_(columnIdx, columnCount);
				if (pkt.laser_return_mode != Velo::RETMODE_DUAL ||
					block_is_dual_strongest_range)
				{
					// Regular range, or strongest in multi return mode:
					rangeImage(laserId, columnIdx) = distance;

					// Intensity:
					intensityImage(laserId, columnIdx) =
						pkt.blocks[block].laser_returns[k].intensity();
				}
				else if (block_is_dual_last_range)
				{
					// Regular range, or strongest in multi return mode:
					auto& r = rangeOtherLayers["STRONGEST"];
					// 1st time init:
					if (static_cast<size_t>(r.rows()) != num_lasers)
						r.setZero(rowCount, columnCount);

					r(laserId, columnIdx) = distance;
				}

			}  // end for k,dsr=[0,31]
		}  // end for each block [0,11]
	}

	// Start and end azimuth:
	startAzimuth =
		mrpt::DEG2RAD(o.scan_packets.begin()->blocks[0].rotation() * 1e-2);

	const auto microsecs_1st_pkt = o.scan_packets.begin()->gps_timestamp();
	const auto microsecs_last_pkt = o.scan_packets.rbegin()->gps_timestamp();
	sweepDuration = 1e-6 * (microsecs_last_pkt - microsecs_1st_pkt) +
					timeBetweenLastTwoBlocks;

	// Decode model byte:
	switch (model)
	{
		case 0x21:
			lidarModel = "HDL-32E";
			break;
		case 0x22:
			lidarModel = "VLP-16";
			break;
		default:
			lidarModel = "Unknown";
			break;
	};

	MRPT_END
}

void RotScan::fromScan2D(const mrpt::obs::CObservation2DRangeScan& o)
{
	MRPT_START

	// Reset:
	*this = CObservationRotatingScan();

	// Copy properties:
	this->has_satellite_timestamp = false;
	this->timestamp = o.timestamp;
	this->sensorPose = o.sensorPose;
	this->sensorLabel = o.sensorLabel;
	this->maxRange = o.maxRange;

	// Convert ranges to range images:
	this->rowCount = 1;
	this->columnCount = o.getScanSize();

	this->rangeImage.setZero(rowCount, columnCount);
	this->intensityImage.setZero(rowCount, columnCount);
	this->rangeOtherLayers.clear();
	this->rangeResolution = 0.01;
	this->azimuthSpan = o.aperture * (o.rightToLeft ? +1.0 : -1.0);
	this->startAzimuth = o.aperture * (o.rightToLeft ? -0.5 : +0.5);

	for (size_t i = 0; i < o.getScanSize(); i++)
	{
		uint16_t& range_out = rangeImage(0, i);
		uint8_t& intensity_out = intensityImage(0, i);
		range_out = 0;
		intensity_out = 0;

		// Convert range into discrete units:
		const float r = o.getScanRange(i);
		const uint16_t r_discr =
			static_cast<uint16_t>((r / d2f(rangeResolution)) + 0.5f);

		if (!o.getScanRangeValidity(i) || r <= 0 || r >= o.maxRange) continue;

		range_out = r_discr;

		if (o.hasIntensity()) intensity_out = o.getScanIntensity(i);
	}

	this->lidarModel = std::string("2D_SCAN_") + this->sensorLabel;

	MRPT_END
}

void RotScan::fromPointCloud(const mrpt::obs::CObservationPointCloud& o)
{
	MRPT_START
	MRPT_TODO("fromPointCloud");
	THROW_EXCEPTION("fromPointCloud() not implemented yet");
	MRPT_END
}

bool RotScan::fromGeneric(const mrpt::obs::CObservation& o)
{
	MRPT_START

	if (auto oVel = dynamic_cast<const CObservationVelodyneScan*>(&o); oVel)
	{
		fromVelodyne(*oVel);
		return true;
	}
	if (auto o2D = dynamic_cast<const CObservation2DRangeScan*>(&o); o2D)
	{
		fromScan2D(*o2D);
		return true;
	}
	if (auto oPc = dynamic_cast<const CObservationPointCloud*>(&o); oPc)
	{
		fromPointCloud(*oPc);
		return true;
	}
	return false;

	MRPT_END
}
