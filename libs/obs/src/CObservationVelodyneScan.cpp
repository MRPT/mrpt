/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/utils/round.h>
#include <mrpt/utils/CStream.h>

using namespace std;
using namespace mrpt::obs;

MRPT_TODO("API for accurate reconstruction of the sensor path in SE(3) over time")

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationVelodyneScan, CObservation,mrpt::obs)

CSinCosLookUpTableFor2DScans  velodyne_sincos_tables;

const float CObservationVelodyneScan::ROTATION_RESOLUTION = 0.01f; /**< degrees */
const float CObservationVelodyneScan::DISTANCE_MAX = 130.0f;        /**< meters */
const float CObservationVelodyneScan::DISTANCE_RESOLUTION = 0.002f; /**< meters */
const float CObservationVelodyneScan::DISTANCE_MAX_UNITS = (CObservationVelodyneScan::DISTANCE_MAX / CObservationVelodyneScan::DISTANCE_RESOLUTION + 1.0f);

const int SCANS_PER_FIRING = 16;

const float VLP16_BLOCK_TDURATION = 110.592f; // [us]
const float VLP16_DSR_TOFFSET = 2.304f; // [us]
const float VLP16_FIRING_TOFFSET = 55.296f; // [us]

const float HDR32_DSR_TOFFSET = 1.152f; // [us]
const float HDR32_FIRING_TOFFSET = 46.08f; // [us]




CObservationVelodyneScan::TGeneratePointCloudParameters::TGeneratePointCloudParameters() :
	minAzimuth_deg(0.0),
	maxAzimuth_deg(360.0)
{
}

CObservationVelodyneScan::CObservationVelodyneScan( ) :
	minRange(1.0),
	maxRange(130.0),
	sensorPose(),
	originalReceivedTimestamp(INVALID_TIMESTAMP),
	has_satellite_timestamp(false)
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
		*version = 1;
	else
	{
		out << timestamp << sensorLabel;

		out << minRange << maxRange << sensorPose;
		{
			uint32_t N = scan_packets.size();
			out << N;
			if (N) out.WriteBuffer(&scan_packets[0],sizeof(scan_packets[0])*N);
		}
		{
			uint32_t N = calibration.laser_corrections.size();
			out << N;
			if (N) out.WriteBuffer(&calibration.laser_corrections[0],sizeof(calibration.laser_corrections[0])*N);
		}
		out << point_cloud.x << point_cloud.y << point_cloud.z << point_cloud.intensity;
		out << has_satellite_timestamp; // v1
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
	case 1:
		{
			in >> timestamp >> sensorLabel;

			in >> minRange >> maxRange >> sensorPose;
			{
				uint32_t N; 
				in >> N;
				scan_packets.resize(N);
				if (N) in.ReadBuffer(&scan_packets[0],sizeof(scan_packets[0])*N);
			}
			{
				uint32_t N; 
				in >> N;
				calibration.laser_corrections.resize(N);
				if (N) in.ReadBuffer(&calibration.laser_corrections[0],sizeof(calibration.laser_corrections[0])*N);
			}
			in >> point_cloud.x >> point_cloud.y >> point_cloud.z >> point_cloud.intensity;
			if (version>=1)
				in >> has_satellite_timestamp;
			else has_satellite_timestamp = (this->timestamp!=this->originalReceivedTimestamp);

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
	o << sensorPose.getHomogeneousMatrixVal() << "\n" << sensorPose << endl;

	o << format("Sensor min/max range: %.02f / %.02f m\n", minRange, maxRange );

	o << "Raw packet count: " << scan_packets.size() << "\n";
}

double HDL32AdjustTimeStamp(int firingblock, int dsr)
{
	return 
		(firingblock * HDR32_FIRING_TOFFSET) + 
		(dsr * HDR32_DSR_TOFFSET);
}
double VLP16AdjustTimeStamp(int firingblock,int dsr,int firingwithinblock)
{
	return 
		(firingblock * VLP16_BLOCK_TDURATION) + 
		(dsr * VLP16_DSR_TOFFSET) + 
		(firingwithinblock * VLP16_FIRING_TOFFSET);
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
	// The LUT contains sin/cos values for angles in this order: [180deg ... 0 deg ... -180 deg]
	const CSinCosLookUpTableFor2DScans::TSinCosValues & lut_sincos = velodyne_sincos_tables.getSinCosForScan(scan_props);

	const int minAzimuth_int = round( params.minAzimuth_deg * 100 );
	const int maxAzimuth_int = round( params.maxAzimuth_deg * 100 );

	// This is: 16,32,64 depending on the LIDAR model
	const size_t num_lasers = calibration.laser_corrections.size();

	for (size_t iPkt = 0; iPkt<scan_packets.size();iPkt++)
	{
		const TVelodyneRawPacket *raw = &scan_packets[iPkt];
//		const double timestamp = this->ComputeTimestamp(dataPacket->gpsTimestamp);

		std::vector<int> diffs(BLOCKS_PER_PACKET - 1);
		for(int i = 0; i < BLOCKS_PER_PACKET-1; ++i) {
			int localDiff = (ROTATION_MAX_UNITS + raw->blocks[i+1].rotation - raw->blocks[i].rotation) % ROTATION_MAX_UNITS;
			diffs[i] = localDiff;
		}
		std::nth_element(
			diffs.begin(),
			diffs.begin() + BLOCKS_PER_PACKET/2,
			diffs.end());
		const int median_azimuth_diff = diffs[BLOCKS_PER_PACKET/2];

		for (int block = 0; block < BLOCKS_PER_PACKET; block++)  // Firings per packet
		{
			// ignore packets with mangled or otherwise different contents
			if ((num_lasers!=64 && UPPER_BANK != raw->blocks[block].header) ||
				(raw->blocks[block].header!=UPPER_BANK && raw->blocks[block].header!=LOWER_BANK) )
			{
				cerr << "[CObservationVelodyneScan] skipping invalid packet: block " << block << " header value is " << raw->blocks[block].header;
				continue;
			}

			MRPT_TODO("Support LIDAR dual data")

			const int dsr_offset = (raw->blocks[block].header==LOWER_BANK) ? 32:0;

			const float azimuth_raw_f = (float)(raw->blocks[block].rotation);

			for (int dsr=0,k=0; dsr < SCANS_PER_FIRING; dsr++, k++)
			{
				const uint8_t rawLaserId = static_cast<uint8_t>(dsr + dsr_offset);
				uint8_t laserId = rawLaserId;

				// Detect VLP-16 data and adjust laser id if necessary
				bool firingWithinBlock = false;
				if(num_lasers==16) {
					if(laserId >= 16) {
						laserId -= 16;
						firingWithinBlock = true;
					}
				}

				ASSERT_BELOW_(laserId,num_lasers)
				const mrpt::obs::VelodyneCalibration::PerLaserCalib &calib = calibration.laser_corrections[laserId];

				// Azimuth correction: correct for the laser rotation as a function of timing during the firings
				double timestampadjustment = 0.0;
				double blockdsr0 = 0.0;
				double nextblockdsr0 = 1.0;
				switch (num_lasers)
				{
				// VLP-16
				case 16:
					timestampadjustment = VLP16AdjustTimeStamp(block, laserId, firingWithinBlock);
					nextblockdsr0 = VLP16AdjustTimeStamp(block+1,0,0);
					blockdsr0 = VLP16AdjustTimeStamp(block,0,0);
					break;
				// HDL-32:
				case 32:
					timestampadjustment = HDL32AdjustTimeStamp(block, dsr);
					nextblockdsr0 = HDL32AdjustTimeStamp(block+1,0);
					blockdsr0 = HDL32AdjustTimeStamp(block,0);
					break;
				case 64:
					break;
				default: {
					THROW_EXCEPTION("Error: unhandled LIDAR model!")
					}
				};

				//Was: float azimuth_correction_f = (median_azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
				const int azimuthadjustment = mrpt::utils::round( median_azimuth_diff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
				timestampadjustment= mrpt::utils::round( timestampadjustment );

				const float azimuth_corrected_f = azimuth_raw_f + azimuthadjustment;
				const int azimuth_corrected = ((int)round(azimuth_corrected_f)) % ROTATION_MAX_UNITS;

				// For the following code: See reference implementation in vtkVelodyneHDLReader::vtkInternal::PushFiringData()
				// -----------------------------------------
				if ((minAzimuth_int < maxAzimuth_int && azimuth_corrected >= minAzimuth_int && azimuth_corrected <= maxAzimuth_int )
					||(minAzimuth_int > maxAzimuth_int && (azimuth_corrected <= maxAzimuth_int || azimuth_corrected >= minAzimuth_int)))
				{
					/** Position Calculation */
					if (!raw->blocks[block].laser_returns[k].distance)
						continue; 
					const float distance = raw->blocks[block].laser_returns[k].distance * DISTANCE_RESOLUTION + calib.distanceCorrection;

					MRPT_TODO("Process LIDAR dual mode here")

					if (distance>=minRange && distance<=maxRange)
					{
						// Vertical axis mis-alignment calibration:
						const float cos_vert_angle = calib.cosVertCorrection;
						const float sin_vert_angle = calib.sinVertCorrection;
						const float horz_offset = calib.horizontalOffsetCorrection;
						const float vert_offset = calib.verticalOffsetCorrection;

						float xy_distance = distance * cos_vert_angle; 
						if (vert_offset) xy_distance+= vert_offset * sin_vert_angle;

						const int azimuth_corrected_for_lut = (azimuth_corrected + (ROTATION_MAX_UNITS/2))%ROTATION_MAX_UNITS;
						const float cos_azimuth = lut_sincos.ccos[azimuth_corrected_for_lut];
						const float sin_azimuth = lut_sincos.csin[azimuth_corrected_for_lut];

						// Compute raw position
						const mrpt::math::TPoint3Df pt_raw(
							xy_distance * cos_azimuth + horz_offset * sin_azimuth, // MRPT +X = Velodyne +Y
							-(xy_distance * sin_azimuth - horz_offset * cos_azimuth), // MRPT +Y = Velodyne -X
							distance * sin_vert_angle + vert_offset
							);

						point_cloud.x.push_back( pt_raw.x );
						point_cloud.y.push_back( pt_raw.y );
						point_cloud.z.push_back( pt_raw.z );
						point_cloud.intensity.push_back( raw->blocks[block].laser_returns[k].intensity );
					}
				}

			} // end for k,dsr=[0,31]
		} // end for each block [0,11]
	} // end for each data packet
}
