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
	// CODE FOR VLP-16 ====================
	using mrpt::utils::round;
	float last_azimuth_diff;

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

MRPT_TODO("Repeat for each raw packet")
	for (size_t iPkt = 0; iPkt<scan_packets.size();iPkt++)
	{
		const TVelodyneRawPacket *raw = &scan_packets[iPkt];

		for (int block = 0; block < BLOCKS_PER_PACKET; block++) 
		{
			// ignore packets with mangled or otherwise different contents
			if (UPPER_BANK != raw->blocks[block].header) {
				// Do not flood the log with messages, only issue at most one
				// of these warnings per minute.
				cerr << "[CObservationVelodyneScan] skipping invalid VLP-16 packet: block "
					<< block << " header value is "
					<< raw->blocks[block].header;
				return;                         // bad packet: skip the rest
			}

			float azimuth = (float)(raw->blocks[block].rotation);
			float azimuth_diff;
			if (block < (BLOCKS_PER_PACKET-1)){
				azimuth_diff = (float)((36000 + raw->blocks[block+1].rotation - raw->blocks[block].rotation)%36000);
				last_azimuth_diff = azimuth_diff;
			}else{
				azimuth_diff = last_azimuth_diff;
			}

			for (int firing=0, k=0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
			{
				for (int dsr=0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k+=RAW_SCAN_SIZE)
				{
					//velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[dsr];


					/** correct for the laser rotation as a function of timing during the firings **/
					float azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr*VLP16_DSR_TOFFSET) + (firing*VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
					int azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

					/*condition added to avoid calculating points which are not
					in the interesting defined area (min_angle < area < max_angle)*/
					//if ((azimuth_corrected >= config_.min_angle && azimuth_corrected <= config_.max_angle && config_.min_angle < config_.max_angle)
					//	||(config_.min_angle > config_.max_angle && (azimuth_corrected <= config_.max_angle || azimuth_corrected >= config_.min_angle)))
					{
							/** Position Calculation */
							float distance = raw->blocks[block].laser_returns[k].distance * DISTANCE_RESOLUTION;
							MRPT_TODO("corrections!")
#if 0
							//distance += corrections.dist_correction;
							float cos_vert_angle = corrections.cos_vert_correction;
							float sin_vert_angle = corrections.sin_vert_correction;
							float cos_rot_correction = corrections.cos_rot_correction;
							float sin_rot_correction = corrections.sin_rot_correction;
#endif
							float cos_vert_angle = 1.0f;
							float sin_vert_angle = 0.0f;

							// cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
							// sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
							const float cos_rot_angle = lut_sincos.ccos(azimuth_corrected);
							const float sin_rot_angle = lut_sincos.csin(azimuth_corrected);
							MRPT_TODO("Integrate calibration corrections")
								//cos_rot_table_[azimuth_corrected] * cos_rot_correction + 
								//sin_rot_table_[azimuth_corrected] * sin_rot_correction;
								//sin_rot_table_[azimuth_corrected] * cos_rot_correction - 
								//cos_rot_table_[azimuth_corrected] * sin_rot_correction;
//							float horiz_offset = corrections.horiz_offset_correction;
//							float vert_offset = corrections.vert_offset_correction;
							const float horiz_offset = .0f; 
							const float vert_offset =  .0f; 

							// Compute the distance in the xy plane (w/o accounting for rotation)
							/**the new term of 'vert_offset * sin_vert_angle'
							* was added to the expression due to the mathemathical
							* model we used.
							*/
							float xy_distance = distance * cos_vert_angle + vert_offset * sin_vert_angle;

							// Calculate temporal X, use absolute value.
							float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
							// Calculate temporal Y, use absolute value
							float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
							if (xx < 0) xx=-xx;
							if (yy < 0) yy=-yy;

							// Get 2points calibration values,Linear interpolation to get distance
							// correction for X and Y, that means distance correction use
							// different value at different distance
							float distance_corr_x = 0;
							float distance_corr_y = 0;
#if 0
							if (corrections.two_pt_correction_available) {
								distance_corr_x = 
									(corrections.dist_correction - corrections.dist_correction_x)
									* (xx - 2.4) / (25.04 - 2.4) 
									+ corrections.dist_correction_x;
								distance_corr_x -= corrections.dist_correction;
								distance_corr_y = 
									(corrections.dist_correction - corrections.dist_correction_y)
									* (yy - 1.93) / (25.04 - 1.93)
									+ corrections.dist_correction_y;
								distance_corr_y -= corrections.dist_correction;
							}
#endif

							float distance_x = distance + distance_corr_x;
							/**the new term of 'vert_offset * sin_vert_angle'
							* was added to the expression due to the mathemathical
							* model we used.
							*/
							xy_distance = distance_x * cos_vert_angle + vert_offset * sin_vert_angle ;
							float x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

							float distance_y = distance + distance_corr_y;
							/**the new term of 'vert_offset * sin_vert_angle'
							* was added to the expression due to the mathemathical
							* model we used.
							*/
							xy_distance = distance_y * cos_vert_angle + vert_offset * sin_vert_angle ;
							float y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

							// Using distance_y is not symmetric, but the velodyne manual
							// does this.
							/**the new term of 'vert_offset * cos_vert_angle'
							* was added to the expression due to the mathemathical
							* model we used.
							*/
							float z = distance_y * sin_vert_angle + vert_offset*cos_vert_angle;


							/** Use standard ROS coordinate system (right-hand rule) */
							float x_coord = y;
							float y_coord = -x;
							float z_coord = z;

							/** Intensity Calculation */

							float min_intensity = 0; //corrections.min_intensity;
							float max_intensity = 255; //corrections.max_intensity;

							float intensity = raw->blocks[block].laser_returns[k].intensity; //  raw->blocks[block].data[k+2];

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
								point_cloud.x.push_back( x_coord );
								point_cloud.y.push_back( y_coord );
								point_cloud.z.push_back( z_coord );
								point_cloud.intensity.push_back( static_cast<uint8_t>(intensity) );
							}
					}
				}
			}
		}
	} // end for each data packet
}

