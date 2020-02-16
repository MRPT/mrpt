/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/containers/stl_containers_utils.h>
#include <mrpt/core/round.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <iostream>

using namespace std;
using namespace mrpt::obs;

// shortcut:
using Velo = mrpt::obs::CObservationVelodyneScan;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationVelodyneScan, CObservation, mrpt::obs)

static CSinCosLookUpTableFor2DScans velodyne_sincos_tables;

const float VLP16_BLOCK_TDURATION = 110.592f;  // [us]
const float VLP16_DSR_TOFFSET = 2.304f;  // [us]
const float VLP16_FIRING_TOFFSET = 55.296f;  // [us]

const double HDR32_DSR_TOFFSET = 1.152;  // [us]
const double HDR32_FIRING_TOFFSET = 46.08;  // [us]

mrpt::system::TTimeStamp Velo::getOriginalReceivedTimeStamp() const
{
	return originalReceivedTimestamp;
}

uint8_t Velo::serializeGetVersion() const { return 2; }
void Velo::serializeTo(mrpt::serialization::CArchive& out) const
{
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
}

void Velo::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			in >> timestamp >> sensorLabel;

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
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};

	// m_cachedMap.clear();
}

void Velo::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);
	o << "Homogeneous matrix for the sensor 3D pose, relative to robot base:\n";
	o << sensorPose.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>()
	  << "\n"
	  << sensorPose << endl;
	o << format("Sensor min/max range: %.02f / %.02f m\n", minRange, maxRange);
	o << "Raw packet count: " << scan_packets.size() << "\n";
}

/** [us] */
static double HDL32AdjustTimeStamp(int firingblock, int dsr)
{
	return (firingblock * HDR32_FIRING_TOFFSET) + (dsr * HDR32_DSR_TOFFSET);
}
/** [us] */
static double VLP16AdjustTimeStamp(
	int firingblock, int dsr, int firingwithinblock)
{
	return (firingblock * VLP16_BLOCK_TDURATION) + (dsr * VLP16_DSR_TOFFSET) +
		   (firingwithinblock * VLP16_FIRING_TOFFSET);
}

static void velodyne_scan_to_pointcloud(
	const Velo& scan, const Velo::TGeneratePointCloudParameters& params,
	Velo::PointCloudStorageWrapper& out_pc)
{
	// Initially based on code from ROS velodyne & from
	// vtkVelodyneHDLReader::vtkInternal::ProcessHDLPacket().
	using mrpt::round;

	// Access to sin/cos table:
	mrpt::obs::T2DScanProperties scan_props;
	scan_props.aperture = 2 * M_PI;
	scan_props.nRays = Velo::ROTATION_MAX_UNITS;
	scan_props.rightToLeft = true;
	// The LUT contains sin/cos values for angles in this order: [180deg ... 0
	// deg ... -180 deg]
	const CSinCosLookUpTableFor2DScans::TSinCosValues& lut_sincos =
		velodyne_sincos_tables.getSinCosForScan(scan_props);

	const int minAzimuth_int = round(params.minAzimuth_deg * 100);
	const int maxAzimuth_int = round(params.maxAzimuth_deg * 100);
	const float realMinDist = std::max(d2f(scan.minRange), params.minDistance);
	const float realMaxDist = std::min(params.maxDistance, d2f(scan.maxRange));
	const auto isolatedPointsFilterDistance_units = mrpt::round(
		params.isolatedPointsFilterDistance / Velo::DISTANCE_RESOLUTION);

	// This is: 16,32,64 depending on the LIDAR model
	const size_t num_lasers = scan.calibration.laser_corrections.size();

	out_pc.resizeLaserCount(num_lasers);
	out_pc.reserve(
		Velo::SCANS_PER_BLOCK * scan.scan_packets.size() *
			Velo::BLOCKS_PER_PACKET +
		16);

	for (size_t iPkt = 0; iPkt < scan.scan_packets.size(); iPkt++)
	{
		const Velo::TVelodyneRawPacket* raw = &scan.scan_packets[iPkt];

		mrpt::system::TTimeStamp pkt_tim;  // Find out timestamp of this pkt
		{
			const uint32_t us_pkt0 = scan.scan_packets[0].gps_timestamp();
			const uint32_t us_pkt_this = raw->gps_timestamp();
			// Handle the case of time counter reset by new hour 00:00:00
			const uint32_t us_ellapsed =
				(us_pkt_this >= us_pkt0)
					? (us_pkt_this - us_pkt0)
					: (1000000UL * 3600UL + us_pkt_this - us_pkt0);
			pkt_tim =
				mrpt::system::timestampAdd(scan.timestamp, us_ellapsed * 1e-6);
		}

		// Take the median rotational speed as a good value for interpolating
		// the missing azimuths:
		int median_azimuth_diff;
		{
			// In dual return, the azimuth rate is actually twice this
			// estimation:
			const unsigned int nBlocksPerAzimuth =
				(raw->laser_return_mode == Velo::RETMODE_DUAL) ? 2 : 1;
			const size_t nDiffs = Velo::BLOCKS_PER_PACKET - nBlocksPerAzimuth;
			std::vector<int> diffs(nDiffs);
			for (size_t i = 0; i < nDiffs; ++i)
			{
				int localDiff = (Velo::ROTATION_MAX_UNITS +
								 raw->blocks[i + nBlocksPerAzimuth].rotation() -
								 raw->blocks[i].rotation()) %
								Velo::ROTATION_MAX_UNITS;
				diffs[i] = localDiff;
			}
			std::nth_element(
				diffs.begin(), diffs.begin() + Velo::BLOCKS_PER_PACKET / 2,
				diffs.end());  // Calc median
			median_azimuth_diff = diffs[Velo::BLOCKS_PER_PACKET / 2];
		}

		// Firings per packet
		for (int block = 0; block < Velo::BLOCKS_PER_PACKET; block++)
		{
			// ignore packets with mangled or otherwise different contents
			if ((num_lasers != 64 &&
				 Velo::UPPER_BANK != raw->blocks[block].header()) ||
				(raw->blocks[block].header() != Velo::UPPER_BANK &&
				 raw->blocks[block].header() != Velo::LOWER_BANK))
			{
				cerr << "[Velo] skipping invalid packet: block " << block
					 << " header value is " << raw->blocks[block].header();
				continue;
			}

			const int dsr_offset =
				(raw->blocks[block].header() == Velo::LOWER_BANK) ? 32 : 0;
			const auto azimuth_raw_f = d2f(raw->blocks[block].rotation());
			const bool block_is_dual_2nd_ranges =
				(raw->laser_return_mode == Velo::RETMODE_DUAL &&
				 ((block & 0x01) != 0));
			const bool block_is_dual_last_ranges =
				(raw->laser_return_mode == Velo::RETMODE_DUAL &&
				 ((block & 0x01) == 0));

			for (int dsr = 0, k = 0; dsr < Velo::SCANS_PER_FIRING; dsr++, k++)
			{
				if (!raw->blocks[block]
						 .laser_returns[k]
						 .distance())  // Invalid return?
					continue;

				const auto rawLaserId = static_cast<uint8_t>(dsr + dsr_offset);
				uint8_t laserId = rawLaserId;

				// Detect VLP-16 data and adjust laser id if necessary
				bool firingWithinBlock = false;
				if (num_lasers == 16)
				{
					if (laserId >= 16)
					{
						laserId -= 16;
						firingWithinBlock = true;
					}
				}

				ASSERT_BELOW_(laserId, num_lasers);
				const mrpt::obs::VelodyneCalibration::PerLaserCalib& calib =
					scan.calibration.laser_corrections[laserId];

				// In dual return, if the distance is equal in both ranges,
				// ignore one of them:
				if (block_is_dual_2nd_ranges)
				{
					if (raw->blocks[block].laser_returns[k].distance() ==
						raw->blocks[block - 1].laser_returns[k].distance())
						continue;  // duplicated point
					if (!params.dualKeepStrongest) continue;
				}
				if (block_is_dual_last_ranges && !params.dualKeepLast) continue;

				// Return distance:
				const float distance =
					raw->blocks[block].laser_returns[k].distance() *
						Velo::DISTANCE_RESOLUTION +
					calib.distanceCorrection;
				if (distance < realMinDist || distance > realMaxDist) continue;

				// Isolated points filtering:
				if (params.filterOutIsolatedPoints)
				{
					bool pass_filter = true;
					const int16_t dist_this =
						raw->blocks[block].laser_returns[k].distance();
					if (k > 0)
					{
						const int16_t dist_prev =
							raw->blocks[block].laser_returns[k - 1].distance();
						if (!dist_prev ||
							std::abs(dist_this - dist_prev) >
								isolatedPointsFilterDistance_units)
							pass_filter = false;
					}
					if (k < (Velo::SCANS_PER_FIRING - 1))
					{
						const int16_t dist_next =
							raw->blocks[block].laser_returns[k + 1].distance();
						if (!dist_next ||
							std::abs(dist_this - dist_next) >
								isolatedPointsFilterDistance_units)
							pass_filter = false;
					}
					if (!pass_filter) continue;  // Filter out this point
				}

				// Azimuth correction: correct for the laser rotation as a
				// function of timing during the firings
				double timestampadjustment =
					0.0;  // [us] since beginning of scan
				double blockdsr0 = 0.0;
				double nextblockdsr0 = 1.0;
				switch (num_lasers)
				{
					// VLP-16
					case 16:
					{
						if (raw->laser_return_mode == Velo::RETMODE_DUAL)
						{
							timestampadjustment = VLP16AdjustTimeStamp(
								block / 2, laserId, firingWithinBlock);
							nextblockdsr0 =
								VLP16AdjustTimeStamp(block / 2 + 1, 0, 0);
							blockdsr0 = VLP16AdjustTimeStamp(block / 2, 0, 0);
						}
						else
						{
							timestampadjustment = VLP16AdjustTimeStamp(
								block, laserId, firingWithinBlock);
							nextblockdsr0 =
								VLP16AdjustTimeStamp(block + 1, 0, 0);
							blockdsr0 = VLP16AdjustTimeStamp(block, 0, 0);
						}
					}
					break;
					// HDL-32:
					case 32:
						timestampadjustment = HDL32AdjustTimeStamp(block, dsr);
						nextblockdsr0 = HDL32AdjustTimeStamp(block + 1, 0);
						blockdsr0 = HDL32AdjustTimeStamp(block, 0);
						break;
					case 64:
						break;
					default:
					{
						THROW_EXCEPTION("Error: unhandled LIDAR model!");
					}
				};

				const int azimuthadjustment = mrpt::round(
					median_azimuth_diff * ((timestampadjustment - blockdsr0) /
										   (nextblockdsr0 - blockdsr0)));

				const float azimuth_corrected_f =
					azimuth_raw_f + azimuthadjustment;
				const int azimuth_corrected =
					((int)round(azimuth_corrected_f)) %
					Velo::ROTATION_MAX_UNITS;

				// Filter by azimuth:
				if (!((minAzimuth_int < maxAzimuth_int &&
					   azimuth_corrected >= minAzimuth_int &&
					   azimuth_corrected <= maxAzimuth_int) ||
					  (minAzimuth_int > maxAzimuth_int &&
					   (azimuth_corrected <= maxAzimuth_int ||
						azimuth_corrected >= minAzimuth_int))))
					continue;

				// Vertical axis mis-alignment calibration:
				const float cos_vert_angle = d2f(calib.cosVertCorrection);
				const float sin_vert_angle = d2f(calib.sinVertCorrection);
				const float horz_offset = d2f(calib.horizontalOffsetCorrection);
				const float vert_offset = d2f(calib.verticalOffsetCorrection);

				float xy_distance = distance * cos_vert_angle;
				if (vert_offset != .0f)
					xy_distance += vert_offset * sin_vert_angle;

				const int azimuth_corrected_for_lut =
					(azimuth_corrected + (Velo::ROTATION_MAX_UNITS / 2)) %
					Velo::ROTATION_MAX_UNITS;
				const float cos_azimuth =
					lut_sincos.ccos[azimuth_corrected_for_lut];
				const float sin_azimuth =
					lut_sincos.csin[azimuth_corrected_for_lut];

				// Compute raw position
				const mrpt::math::TPoint3Df pt(
					xy_distance * cos_azimuth +
						horz_offset * sin_azimuth,  // MRPT +X = Velodyne +Y
					-(xy_distance * sin_azimuth -
					  horz_offset * cos_azimuth),  // MRPT +Y = Velodyne -X
					distance * sin_vert_angle + vert_offset);

				bool add_point = true;
				if (params.filterByROI &&
					(pt.x > params.ROI_x_max || pt.x < params.ROI_x_min ||
					 pt.y > params.ROI_y_max || pt.y < params.ROI_y_min ||
					 pt.z > params.ROI_z_max || pt.z < params.ROI_z_min))
					add_point = false;

				if (params.filterBynROI &&
					(pt.x <= params.nROI_x_max && pt.x >= params.nROI_x_min &&
					 pt.y <= params.nROI_y_max && pt.y >= params.nROI_y_min &&
					 pt.z <= params.nROI_z_max && pt.z >= params.nROI_z_min))
					add_point = false;

				if (!add_point) continue;

				// Insert point:
				out_pc.add_point(
					pt.x, pt.y, pt.z,
					raw->blocks[block].laser_returns[k].intensity(), pkt_tim,
					azimuth_corrected_f, laserId);

			}  // end for k,dsr=[0,31]
		}  // end for each block [0,11]
	}  // end for each data packet
}

void Velo::generatePointCloud(
	PointCloudStorageWrapper& dest, const TGeneratePointCloudParameters& params)
{
	velodyne_scan_to_pointcloud(*this, params, dest);
}

void Velo::generatePointCloud(const TGeneratePointCloudParameters& params)
{
	struct PointCloudStorageWrapper_Inner : public PointCloudStorageWrapper
	{
		Velo& me_;
		const TGeneratePointCloudParameters& params_;
		PointCloudStorageWrapper_Inner(
			Velo& me, const TGeneratePointCloudParameters& p)
			: me_(me), params_(p)
		{
			// Reset point cloud:
			me_.point_cloud.clear();
		}

		void resizeLaserCount(std::size_t n) override
		{
			me_.point_cloud.pointsForLaserID.resize(n);
		}

		void reserve(std::size_t n) override
		{
			me_.point_cloud.reserve(n);
			if (!me_.point_cloud.pointsForLaserID.empty())
			{
				const std::size_t n_per_ring =
					100 + n / (me_.point_cloud.pointsForLaserID.size());
				for (auto& v : me_.point_cloud.pointsForLaserID)
					v.reserve(n_per_ring);
			}
		}

		void add_point(
			float pt_x, float pt_y, float pt_z, uint8_t pt_intensity,
			const mrpt::system::TTimeStamp& tim, const float azimuth,
			uint16_t laser_id) override
		{
			const auto idx = me_.point_cloud.x.size();
			me_.point_cloud.x.push_back(pt_x);
			me_.point_cloud.y.push_back(pt_y);
			me_.point_cloud.z.push_back(pt_z);
			me_.point_cloud.intensity.push_back(pt_intensity);
			if (params_.generatePerPointTimestamp)
			{
				me_.point_cloud.timestamp.push_back(tim);
			}
			if (params_.generatePerPointAzimuth)
			{
				const int azimuth_corrected =
					round(azimuth) % Velo::ROTATION_MAX_UNITS;
				me_.point_cloud.azimuth.push_back(
					azimuth_corrected * ROTATION_RESOLUTION);
			}
			me_.point_cloud.laser_id.push_back(laser_id);
			if (params_.generatePointsForLaserID)
				me_.point_cloud.pointsForLaserID[laser_id].push_back(idx);
		}
	};

	PointCloudStorageWrapper_Inner my_pc_wrap(*this, params);
	point_cloud.clear();

	generatePointCloud(my_pc_wrap, params);
}

void Velo::generatePointCloudAlongSE3Trajectory(
	const mrpt::poses::CPose3DInterpolator& vehicle_path,
	std::vector<mrpt::math::TPointXYZIu8>& out_points,
	TGeneratePointCloudSE3Results& results_stats,
	const TGeneratePointCloudParameters& params)
{
	struct PointCloudStorageWrapper_SE3_Interp : public PointCloudStorageWrapper
	{
		Velo& me_;
		const mrpt::poses::CPose3DInterpolator& vehicle_path_;
		std::vector<mrpt::math::TPointXYZIu8>& out_points_;
		TGeneratePointCloudSE3Results& results_stats_;
		mrpt::system::TTimeStamp last_query_tim_;
		mrpt::poses::CPose3D last_query_;
		bool last_query_valid_;

		void reserve(std::size_t n) override
		{
			out_points_.reserve(out_points_.size() + n);
		}

		PointCloudStorageWrapper_SE3_Interp(
			Velo& me, const mrpt::poses::CPose3DInterpolator& vehicle_path,
			std::vector<mrpt::math::TPointXYZIu8>& out_points,
			TGeneratePointCloudSE3Results& results_stats)
			: me_(me),
			  vehicle_path_(vehicle_path),
			  out_points_(out_points),
			  results_stats_(results_stats),
			  last_query_tim_(INVALID_TIMESTAMP),
			  last_query_valid_(false)
		{
		}
		void add_point(
			float pt_x, float pt_y, float pt_z, uint8_t pt_intensity,
			const mrpt::system::TTimeStamp& tim,
			[[maybe_unused]] const float azimuth,
			[[maybe_unused]] uint16_t laser_id) override
		{
			// Use a cache since it's expected that the same timestamp is
			// queried several times in a row:
			if (last_query_tim_ != tim)
			{
				last_query_tim_ = tim;
				vehicle_path_.interpolate(tim, last_query_, last_query_valid_);
			}

			if (last_query_valid_)
			{
				mrpt::poses::CPose3D global_sensor_pose(
					mrpt::poses::UNINITIALIZED_POSE);
				global_sensor_pose.composeFrom(last_query_, me_.sensorPose);
				double gx, gy, gz;
				global_sensor_pose.composePoint(pt_x, pt_y, pt_z, gx, gy, gz);
				out_points_.emplace_back(gx, gy, gz, pt_intensity);
				++results_stats_.num_correctly_inserted_points;
			}
			++results_stats_.num_points;
		}
	};

	PointCloudStorageWrapper_SE3_Interp my_pc_wrap(
		*this, vehicle_path, out_points, results_stats);
	velodyne_scan_to_pointcloud(*this, params, my_pc_wrap);
}

void Velo::TPointCloud::clear()
{
	x.clear();
	y.clear();
	z.clear();
	intensity.clear();
	timestamp.clear();
	azimuth.clear();
	laser_id.clear();
	pointsForLaserID.clear();
}
void Velo::TPointCloud::clear_deep()
{
	using mrpt::containers::deep_clear;

	deep_clear(x);
	deep_clear(y);
	deep_clear(z);
	deep_clear(intensity);
	deep_clear(timestamp);
	deep_clear(azimuth);
	deep_clear(laser_id);
	deep_clear(pointsForLaserID);
}

void Velo::TPointCloud::reserve(std::size_t n)
{
	x.reserve(n);
	y.reserve(n);
	z.reserve(n);
	intensity.reserve(n);
	timestamp.reserve(n);
	azimuth.reserve(n);
	laser_id.reserve(n);
	pointsForLaserID.reserve(64);  // ring number
}

// Default ctor. Do NOT move to the .h, that causes build errors.
Velo::TGeneratePointCloudParameters::TGeneratePointCloudParameters() = default;
