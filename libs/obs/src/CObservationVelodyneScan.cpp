/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/utils/round.h>
#include <mrpt/utils/CStream.h>

using namespace std;
using namespace mrpt::obs;


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


const CObservationVelodyneScan::TGeneratePointCloudParameters CObservationVelodyneScan::defaultPointCloudParams;


CObservationVelodyneScan::TGeneratePointCloudParameters::TGeneratePointCloudParameters() :
	minAzimuth_deg(0.0),
	maxAzimuth_deg(360.0),
	minDistance(1.0f),
	maxDistance(std::numeric_limits<float>::max()),
	ROI_x_min(-std::numeric_limits<float>::max()),
	ROI_x_max(+std::numeric_limits<float>::max()),
	ROI_y_min(-std::numeric_limits<float>::max()),
	ROI_y_max(+std::numeric_limits<float>::max()),
	ROI_z_min(-std::numeric_limits<float>::max()),
	ROI_z_max(+std::numeric_limits<float>::max()),
	nROI_x_min(.0f),
	nROI_x_max(.0f),
	nROI_y_min(.0f),
	nROI_y_max(.0f),
	nROI_z_min(.0f),
	nROI_z_max(.0f),
	isolatedPointsFilterDistance(2.0f),
	filterByROI(false),
	filterBynROI(false),
	filterOutIsolatedPoints(false),
	dualKeepStrongest(true),
	dualKeepLast(true)
{
}

CObservationVelodyneScan::TGeneratePointCloudSE3Results::TGeneratePointCloudSE3Results() : 
	num_points (0),
	num_correctly_inserted_points(0)
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

mrpt::system::TTimeStamp CObservationVelodyneScan::getOriginalReceivedTimeStamp() const {
	return originalReceivedTimestamp;
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

double HDL32AdjustTimeStamp(int firingblock, int dsr)  //!< [us]
{
	return 
		(firingblock * HDR32_FIRING_TOFFSET) + 
		(dsr * HDR32_DSR_TOFFSET);
}
double VLP16AdjustTimeStamp(int firingblock,int dsr,int firingwithinblock)  //!< [us]
{
	return 
		(firingblock * VLP16_BLOCK_TDURATION) + 
		(dsr * VLP16_DSR_TOFFSET) + 
		(firingwithinblock * VLP16_FIRING_TOFFSET);
}

/** Auxiliary class used to refactor CObservationVelodyneScan::generatePointCloud() */
struct PointCloudStorageWrapper {
	/** Process the insertion of a new (x,y,z) point to the cloud, in sensor-centric coordinates, with the exact timestamp of that LIDAR ray */
	virtual void add_point(double pt_x,double pt_y, double pt_z,uint8_t pt_intensity, const mrpt::system::TTimeStamp &tim) = 0;
};

static void velodyne_scan_to_pointcloud(
	const CObservationVelodyneScan & scan,
	const CObservationVelodyneScan::TGeneratePointCloudParameters &params,
	PointCloudStorageWrapper & out_pc)
{
	// Initially based on code from ROS velodyne & from vtkVelodyneHDLReader::vtkInternal::ProcessHDLPacket(). 
	using mrpt::utils::round;

	// Access to sin/cos table:
	mrpt::obs::T2DScanProperties scan_props;
	scan_props.aperture = 2*M_PI;
	scan_props.nRays = CObservationVelodyneScan::ROTATION_MAX_UNITS;
	scan_props.rightToLeft = true;
	// The LUT contains sin/cos values for angles in this order: [180deg ... 0 deg ... -180 deg]
	const CSinCosLookUpTableFor2DScans::TSinCosValues & lut_sincos = velodyne_sincos_tables.getSinCosForScan(scan_props);

	const int minAzimuth_int = round( params.minAzimuth_deg * 100 );
	const int maxAzimuth_int = round( params.maxAzimuth_deg * 100 );
	const float realMinDist = std::max(static_cast<float>(scan.minRange),params.minDistance);
	const float realMaxDist = std::min(params.maxDistance,static_cast<float>(scan.maxRange));
	const int16_t isolatedPointsFilterDistance_units = params.isolatedPointsFilterDistance/CObservationVelodyneScan::DISTANCE_RESOLUTION;

	// This is: 16,32,64 depending on the LIDAR model
	const size_t num_lasers = scan.calibration.laser_corrections.size();

	for (size_t iPkt = 0; iPkt<scan.scan_packets.size();iPkt++)
	{
		const CObservationVelodyneScan::TVelodyneRawPacket *raw = &scan.scan_packets[iPkt];

		mrpt::system::TTimeStamp pkt_tim; // Find out timestamp of this pkt
		{
			const uint32_t us_pkt0     = scan.scan_packets[0].gps_timestamp;
			const uint32_t us_pkt_this = raw->gps_timestamp;
			// Handle the case of time counter reset by new hour 00:00:00
			const uint32_t us_ellapsed = (us_pkt_this>=us_pkt0) ? (us_pkt_this-us_pkt0) : (1000000UL*3600UL + us_pkt_this-us_pkt0);
			pkt_tim = mrpt::system::timestampAdd(scan.timestamp,us_ellapsed*1e-6);
		}

		// Take the median rotational speed as a good value for interpolating the missing azimuths:
		int median_azimuth_diff;
		{
			// In dual return, the azimuth rate is actually twice this estimation:
			const int nBlocksPerAzimuth = (raw->laser_return_mode==CObservationVelodyneScan::RETMODE_DUAL) ? 2 : 1;
			std::vector<int> diffs(CObservationVelodyneScan::BLOCKS_PER_PACKET - nBlocksPerAzimuth);
			for(int i = 0; i < CObservationVelodyneScan::BLOCKS_PER_PACKET-nBlocksPerAzimuth; ++i) {
				int localDiff = (CObservationVelodyneScan::ROTATION_MAX_UNITS + raw->blocks[i+nBlocksPerAzimuth].rotation - raw->blocks[i].rotation) % CObservationVelodyneScan::ROTATION_MAX_UNITS;
				diffs[i] = localDiff;
			}
			std::nth_element(diffs.begin(), diffs.begin() + CObservationVelodyneScan::BLOCKS_PER_PACKET/2, diffs.end()); // Calc median
			median_azimuth_diff = diffs[CObservationVelodyneScan::BLOCKS_PER_PACKET/2];
		}

		for (int block = 0; block < CObservationVelodyneScan::BLOCKS_PER_PACKET; block++)  // Firings per packet
		{
			// ignore packets with mangled or otherwise different contents
			if ((num_lasers!=64 && CObservationVelodyneScan::UPPER_BANK != raw->blocks[block].header) ||
				(raw->blocks[block].header!=CObservationVelodyneScan::UPPER_BANK && raw->blocks[block].header!=CObservationVelodyneScan::LOWER_BANK) )
			{
				cerr << "[CObservationVelodyneScan] skipping invalid packet: block " << block << " header value is " << raw->blocks[block].header;
				continue;
			}

			const int dsr_offset = (raw->blocks[block].header==CObservationVelodyneScan::LOWER_BANK) ? 32:0;
			const float azimuth_raw_f = (float)(raw->blocks[block].rotation);
			const bool block_is_dual_2nd_ranges  = (raw->laser_return_mode==CObservationVelodyneScan::RETMODE_DUAL && ((block & 0x01)!=0));
			const bool block_is_dual_last_ranges = (raw->laser_return_mode==CObservationVelodyneScan::RETMODE_DUAL && ((block & 0x01)==0));

			for (int dsr=0,k=0; dsr < SCANS_PER_FIRING; dsr++, k++)
			{
				if (!raw->blocks[block].laser_returns[k].distance) // Invalid return?
					continue;

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
				const mrpt::obs::VelodyneCalibration::PerLaserCalib &calib = scan.calibration.laser_corrections[laserId];

				// In dual return, if the distance is equal in both ranges, ignore one of them:
				if (block_is_dual_2nd_ranges) {
					if (raw->blocks[block].laser_returns[k].distance == raw->blocks[block-1].laser_returns[k].distance)
						continue; // duplicated point
					if (!params.dualKeepStrongest)
						continue;
				}
				if (block_is_dual_last_ranges && !params.dualKeepLast)
					continue;

				// Return distance:
				const float distance = raw->blocks[block].laser_returns[k].distance * CObservationVelodyneScan::DISTANCE_RESOLUTION + calib.distanceCorrection;
				if (distance<realMinDist || distance>realMaxDist)
					continue;

				// Isolated points filtering:
				if (params.filterOutIsolatedPoints) {
					bool pass_filter = true;
					const int16_t dist_this = raw->blocks[block].laser_returns[k].distance;
					if (k>0) {
						const int16_t dist_prev = raw->blocks[block].laser_returns[k-1].distance;
						if (!dist_prev || std::abs(dist_this-dist_prev)>isolatedPointsFilterDistance_units)
							pass_filter=false;
					}
					if (k<(SCANS_PER_FIRING-1)) {
						const int16_t dist_next = raw->blocks[block].laser_returns[k+1].distance;
						if (!dist_next || std::abs(dist_this-dist_next)>isolatedPointsFilterDistance_units)
							pass_filter=false;
					}
					if (!pass_filter) continue; // Filter out this point
				}

				// Azimuth correction: correct for the laser rotation as a function of timing during the firings
				double timestampadjustment = 0.0; // [us] since beginning of scan
				double blockdsr0 = 0.0;
				double nextblockdsr0 = 1.0;
				switch (num_lasers)
				{
				// VLP-16
				case 16:
					{
						if (raw->laser_return_mode==CObservationVelodyneScan::RETMODE_DUAL) {
							timestampadjustment = VLP16AdjustTimeStamp(block/2, laserId, firingWithinBlock);
							nextblockdsr0 = VLP16AdjustTimeStamp(block/2+1,0,0);
							blockdsr0 = VLP16AdjustTimeStamp(block/2,0,0);
						}
						else {
							timestampadjustment = VLP16AdjustTimeStamp(block, laserId, firingWithinBlock);
							nextblockdsr0 = VLP16AdjustTimeStamp(block+1,0,0);
							blockdsr0 = VLP16AdjustTimeStamp(block,0,0);
						}
					}
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

				const int azimuthadjustment = mrpt::utils::round( median_azimuth_diff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));

				const float azimuth_corrected_f = azimuth_raw_f + azimuthadjustment;
				const int azimuth_corrected = ((int)round(azimuth_corrected_f)) % CObservationVelodyneScan::ROTATION_MAX_UNITS;

				// Filter by azimuth:
				if (!((minAzimuth_int < maxAzimuth_int && azimuth_corrected >= minAzimuth_int && azimuth_corrected <= maxAzimuth_int )
					||(minAzimuth_int > maxAzimuth_int && (azimuth_corrected <= maxAzimuth_int || azimuth_corrected >= minAzimuth_int))))
					continue;

				// Vertical axis mis-alignment calibration:
				const float cos_vert_angle = calib.cosVertCorrection;
				const float sin_vert_angle = calib.sinVertCorrection;
				const float horz_offset = calib.horizontalOffsetCorrection;
				const float vert_offset = calib.verticalOffsetCorrection;

				float xy_distance = distance * cos_vert_angle; 
				if (vert_offset) xy_distance+= vert_offset * sin_vert_angle;

				const int azimuth_corrected_for_lut = (azimuth_corrected + (CObservationVelodyneScan::ROTATION_MAX_UNITS/2))%CObservationVelodyneScan::ROTATION_MAX_UNITS;
				const float cos_azimuth = lut_sincos.ccos[azimuth_corrected_for_lut];
				const float sin_azimuth = lut_sincos.csin[azimuth_corrected_for_lut];

				// Compute raw position
				const mrpt::math::TPoint3Df pt(
					xy_distance * cos_azimuth + horz_offset * sin_azimuth, // MRPT +X = Velodyne +Y
					-(xy_distance * sin_azimuth - horz_offset * cos_azimuth), // MRPT +Y = Velodyne -X
					distance * sin_vert_angle + vert_offset
					);

				bool add_point = true;
				if (params.filterByROI && (
				 pt.x>params.ROI_x_max || pt.x<params.ROI_x_min ||
				 pt.y>params.ROI_y_max || pt.y<params.ROI_y_min ||
				 pt.z>params.ROI_z_max || pt.z<params.ROI_z_min))
				  add_point=false;

				if (params.filterBynROI && (
				 pt.x<=params.nROI_x_max && pt.x>=params.nROI_x_min &&
				 pt.y<=params.nROI_y_max && pt.y>=params.nROI_y_min &&
				 pt.z<=params.nROI_z_max && pt.z>=params.nROI_z_min))
				  add_point=false;

				if (!add_point)
					continue;

				// Insert point:
				out_pc.add_point(pt.x,pt.y,pt.z, raw->blocks[block].laser_returns[k].intensity,pkt_tim);

			} // end for k,dsr=[0,31]
		} // end for each block [0,11]
	} // end for each data packet
}


void CObservationVelodyneScan::generatePointCloud(const TGeneratePointCloudParameters &params)
{
	struct PointCloudStorageWrapper_Inner : public PointCloudStorageWrapper {
		CObservationVelodyneScan & me_;
		PointCloudStorageWrapper_Inner(CObservationVelodyneScan &me) : me_(me) {
			// Reset point cloud:
			me_.point_cloud.x.clear();
			me_.point_cloud.y.clear();
			me_.point_cloud.z.clear();
			me_.point_cloud.intensity.clear();
		}
		void add_point(double pt_x,double pt_y, double pt_z,uint8_t pt_intensity, const mrpt::system::TTimeStamp &tim) MRPT_OVERRIDE {
			me_.point_cloud.x.push_back( pt_x );
			me_.point_cloud.y.push_back( pt_y );
			me_.point_cloud.z.push_back( pt_z );
			me_.point_cloud.intensity.push_back( pt_intensity );
		}
	};

	PointCloudStorageWrapper_Inner my_pc_wrap(*this);

	velodyne_scan_to_pointcloud(*this,params, my_pc_wrap);

}

void CObservationVelodyneScan::generatePointCloudAlongSE3Trajectory(
	const mrpt::poses::CPose3DInterpolator & vehicle_path,
	std::vector<mrpt::math::TPointXYZIu8>      & out_points,
	TGeneratePointCloudSE3Results          & results_stats,
	const TGeneratePointCloudParameters &params )
{
	// Pre-alloc mem:
	out_points.reserve( out_points.size() + scan_packets.size() * BLOCKS_PER_PACKET * SCANS_PER_BLOCK + 16);

	struct PointCloudStorageWrapper_SE3_Interp : public PointCloudStorageWrapper {
		CObservationVelodyneScan & me_;
		const mrpt::poses::CPose3DInterpolator & vehicle_path_;
		std::vector<mrpt::math::TPointXYZIu8>      & out_points_;
		TGeneratePointCloudSE3Results &results_stats_;
		mrpt::system::TTimeStamp last_query_tim_;
		mrpt::poses::CPose3D last_query_;
		bool last_query_valid_;

		PointCloudStorageWrapper_SE3_Interp(CObservationVelodyneScan &me,const mrpt::poses::CPose3DInterpolator & vehicle_path,std::vector<mrpt::math::TPointXYZIu8> & out_points,TGeneratePointCloudSE3Results &results_stats) : 
			me_(me),vehicle_path_(vehicle_path),out_points_(out_points),results_stats_(results_stats),last_query_tim_(INVALID_TIMESTAMP),last_query_valid_(false) {
		}
		void add_point(double pt_x,double pt_y, double pt_z,uint8_t pt_intensity, const mrpt::system::TTimeStamp &tim) MRPT_OVERRIDE 
		{
			// Use a cache since it's expected that the same timestamp is queried several times in a row:
			if (last_query_tim_!=tim) {
				last_query_tim_ = tim;
				vehicle_path_.interpolate(tim,last_query_,last_query_valid_);
			}

			if (last_query_valid_) {
				mrpt::poses::CPose3D  global_sensor_pose(mrpt::poses::UNINITIALIZED_POSE);
				global_sensor_pose.composeFrom(last_query_, me_.sensorPose);
				double gx,gy,gz;
				global_sensor_pose.composePoint(pt_x,pt_y,pt_z, gx,gy,gz);
				out_points_.push_back( mrpt::math::TPointXYZIu8(gx,gy,gz,pt_intensity) );
				++results_stats_.num_correctly_inserted_points;
			}
			++results_stats_.num_points;
		}
	};

	PointCloudStorageWrapper_SE3_Interp my_pc_wrap(*this,vehicle_path,out_points,results_stats);
	velodyne_scan_to_pointcloud(*this,params, my_pc_wrap);
}
