/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/utils/round.h>

using namespace std;
using namespace mrpt::obs;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationVelodyneScan, CObservation,mrpt::obs)

CSinCosLookUpTableFor2DScans  velodyne_sincos_tables;

const float CObservationVelodyneScan::ROTATION_RESOLUTION = 0.01f; /**< degrees */
const float CObservationVelodyneScan::DISTANCE_MAX = 130.0f;        /**< meters */
const float CObservationVelodyneScan::DISTANCE_RESOLUTION = 0.002f; /**< meters */
const float CObservationVelodyneScan::DISTANCE_MAX_UNITS = (CObservationVelodyneScan::DISTANCE_MAX / CObservationVelodyneScan::DISTANCE_RESOLUTION + 1.0f);

CObservationVelodyneScan::CObservationVelodyneScan( ) :
	minRange(1.0),
	maxRange(130.0),
	sensorPose()
{
}

CObservationVelodyneScan::~CObservationVelodyneScan()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationVelodyneScan::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << timestamp << sensorLabel;

		out << minRange << maxRange << sensorPose;
		uint32_t N = scan_packets.size();
		out << N;
		if (N) out.WriteBuffer(&scan_packets[0],sizeof(scan_packets[0])*N);
		out << point_cloud.x << point_cloud.y << point_cloud.z << point_cloud.intensity;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationVelodyneScan::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in >> timestamp >> sensorLabel;

			in >> minRange >> maxRange >> sensorPose;
			uint32_t N; 
			in >> N;
			scan_packets.resize(N);
			if (N) in.ReadBuffer(&scan_packets[0],sizeof(scan_packets[0])*N);
			in >> point_cloud.x >> point_cloud.y >> point_cloud.z >> point_cloud.intensity;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

	//m_cachedMap.clear();
}

void CObservationVelodyneScan::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);
	o << "Homogeneous matrix for the sensor 3D pose, relative to robot base:\n";
	o << sensorPose.getHomogeneousMatrixVal() << sensorPose << endl;

	o << format("Sensor min/max range: %.02f / %.02f m\n", minRange / maxRange );

	o << "Raw packet count: " << scan_packets.size() << "\n";
}

void CObservationVelodyneScan::generatePointCloud(const TGeneratePointCloudParameters &params)
{
	// Initially based on code from ROS velodyne & from vtkVelodyneHDLReader::vtkInternal::ProcessHDLPacket(). 
	// CODE FOR VLP-16 ====================

	using mrpt::utils::round;

	// Reset point cloud:
	point_cloud.x.clear();
	point_cloud.y.clear();
	point_cloud.z.clear();
	point_cloud.intensity.clear();

	// Access to sin/cos table:
	mrpt::obs::T2DScanProperties scan_props;
	scan_props.aperture = 2*M_PI;
	scan_props.nRays = ROTATION_MAX_UNITS;
	scan_props.rightToLeft = true;
	const CSinCosLookUpTableFor2DScans::TSinCosValues & lut_sincos = velodyne_sincos_tables.getSinCosForScan(scan_props);

	MRPT_TODO("Support 64 LIDAR")
	int hdl64offset = 0;

	for (size_t iPkt = 0; iPkt<scan_packets.size();iPkt++)
	{
		const TVelodyneRawPacket *raw = &scan_packets[iPkt];
//		const double timestamp = this->ComputeTimestamp(dataPacket->gpsTimestamp);

		std::vector<int> diffs(BLOCKS_PER_PACKET - 1);
		for(int i = 0; i < BLOCKS_PER_PACKET-1; ++i) {
			int localDiff = (36000 + raw->blocks[i+1].rotation - raw->blocks[i].rotation) % 36000;
			diffs[i] = localDiff;
		}
		std::nth_element(
			diffs.begin(),
			diffs.begin() + BLOCKS_PER_PACKET/2,
			diffs.end());
		const int median_azimuth_diff = diffs[BLOCKS_PER_PACKET/2];

		for (int block = 0; block < BLOCKS_PER_PACKET; block++)  // Firings per packet
		{
			// ProcessFiring() in vtk's VeloView code:

			// ignore packets with mangled or otherwise different contents
			if (UPPER_BANK != raw->blocks[block].header) {
				// Do not flood the log with messages, only issue at most one
				// of these warnings per minute.
				cerr << "[CObservationVelodyneScan] skipping invalid VLP-16 packet: block "
					<< block << " header value is "
					<< raw->blocks[block].header;
				continue;                         // bad packet: skip the rest
			}

			MRPT_TODO("Support LIDAR dual data")
			
			const float azimuth_raw_f = (float)(raw->blocks[block].rotation);

			for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
			{
				for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE)
				{
					const uint8_t rawLaserId = static_cast<uint8_t>(dsr + hdl64offset);
					const uint8_t laserId = rawLaserId;

					ASSERT_BELOW_(laserId,calibration.laser_corrections.size())
					const mrpt::obs::VelodyneCalibration::PerLaserCalib &calib = calibration.laser_corrections[laserId];

					// Azimuth correction: correct for the laser rotation as a function of timing during the firings
					float azimuth_correction_f;
					if (1)
					{
						// VLP 16
						azimuth_correction_f = (median_azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
						//timestampadjustment = VLP16AdjustTimeStamp(firingBlock, laserId, firingWithinBlock);
						//nextblockdsr0 = VLP16AdjustTimeStamp(firingBlock+1,0,0);
						//blockdsr0 = VLP16AdjustTimeStamp(firingBlock,0,0);
						//int azimuthadjustment = vtkMath::Round(azimuthDiff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
					}
					else
					{
						// HDL-32:
						MRPT_TODO("Impl")
						//timestampadjustment = HDL32AdjustTimeStamp(firingBlock, dsr);
						//nextblockdsr0 = HDL32AdjustTimeStamp(firingBlock+1,0);
						//blockdsr0 = HDL32AdjustTimeStamp(firingBlock,0);
						//int azimuthadjustment = vtkMath::Round(azimuthDiff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
					}
					const float azimuth_corrected_f = azimuth_raw_f + azimuth_correction_f;
					const int azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

					// For the following code: See reference implementation in vtkVelodyneHDLReader::vtkInternal::PushFiringData()
					// -----------------------------------------

					// Azimuth correction:
					//double azimuthInRadians = HDL_Grabber_toRadians((static_cast<double> (azimuth) / 100.0) - correction->azimuthCorrection);


					/*condition added to avoid calculating points which are not
					in the interesting defined area (min_angle < area < max_angle)*/
					//if ((azimuth_corrected >= config_.min_angle && azimuth_corrected <= config_.max_angle && config_.min_angle < config_.max_angle)
					//	||(config_.min_angle > config_.max_angle && (azimuth_corrected <= config_.max_angle || azimuth_corrected >= config_.min_angle)))
					{
						/** Position Calculation */
						const float distance = raw->blocks[block].laser_returns[k].distance * DISTANCE_RESOLUTION + calib.distanceCorrection;

						// Vertical axis mis-alignment calibration:
						const float cos_vert_angle = calib.cosVertCorrection;
						const float sin_vert_angle = calib.sinVertCorrection;
						const float horz_offset = calib.horizontalOffsetCorrection;
						const float vert_offset = calib.verticalOffsetCorrection;

						const float xy_distance = distance * cos_vert_angle + vert_offset * sin_vert_angle;

						// cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
						// sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
						const float cos_azimuth = lut_sincos.ccos(azimuth_corrected);
						const float sin_azimuth = lut_sincos.csin(azimuth_corrected);
						MRPT_TODO("Integrate calibration corrections")

						// Compute raw position
						const mrpt::math::TPoint3Df pt_raw(
							xy_distance * sin_azimuth - horz_offset * cos_azimuth,
							xy_distance * cos_azimuth + horz_offset * sin_azimuth,
							distance * sin_vert_angle + vert_offset
							);

						MRPT_TODO("Process LIDAR dual mode here")

						// Intensity Calculation
						MRPT_TODO("corrections!")
						const float min_intensity = 0; //corrections.min_intensity;
						const float max_intensity = 255; //corrections.max_intensity;

						float intensity = raw->blocks[block].laser_returns[k].intensity;

#if 0
							float focal_offset = 256 
								* (1 - corrections.focal_distance / 13100) 
								* (1 - corrections.focal_distance / 13100);
							float focal_slope = corrections.focal_slope;
							intensity += focal_slope * (abs(focal_offset - 256 * 
								(1 - tmp.uint/65535)*(1 - tmp.uint/65535)));
							intensity = (intensity < min_intensity) ? min_intensity : intensity;
							intensity = (intensity > max_intensity) ? max_intensity : intensity;
#endif

						if (distance>=minRange && distance<=maxRange)
						{
							//point.ring = corrections.laser_ring;
							point_cloud.x.push_back( pt_raw.x );
							point_cloud.y.push_back( pt_raw.y );
							point_cloud.z.push_back( pt_raw.z );
							point_cloud.intensity.push_back( static_cast<uint8_t>(intensity) );
						}
					}
				}
			}
		}
	} // end for each data packet
}
