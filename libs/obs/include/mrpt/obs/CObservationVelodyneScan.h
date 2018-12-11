/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSinCosLookUpTableFor2DScans.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/obs/VelodyneCalibration.h>
#include <vector>

namespace mrpt
{
namespace poses
{
class CPose3DInterpolator;
}
namespace obs
{
/** A `CObservation`-derived class for RAW DATA (and optionally, point cloud) of
 * scans from 3D Velodyne LIDAR scanners.
 * A scan comprises one or more "velodyne packets" (refer to Velodyne user
 * manual). Normally, a full 360 degrees sweep is included in one observation
 * object. Note that if a pointcloud is generated inside this class, each point
 * is annotated with per-point information about its exact azimuth and laser_id
 * (ring number), an information that is loss when inserting the observation in
 * a regular mrpt::maps::CPointsMap.
 *
 * <h2>Main data fields:</h2><hr>
 * - CObservationVelodyneScan::scan_packets with raw data packets.
 * - CObservationVelodyneScan::point_cloud normally empty after grabbing for
 * efficiency, can be generated calling \a
 * CObservationVelodyneScan::generatePointCloud()
 *
 * Dual return mode is supported (see mrpt::hwdrivers::CVelodyneScanner).
 *
 * Axes convention for point cloud (x,y,z) coordinates:
 *
 *  <div align=center> <img src="velodyne_axes.jpg"> </div>
 *
 * If it can be assumed that the sensor moves SLOWLY through the environment
 * (i.e. its pose can be approximated to be the same since the beginning to the
 * end of one complete scan)
 * then this observation can be converted / loaded into the following other
 * classes:
 *  - Maps of points (these require first generating the pointcloud in this
 * observation object with
 * mrpt::obs::CObservationVelodyneScan::generatePointCloud() ):
 *    - mrpt::maps::CPointsMap::loadFromVelodyneScan() (available in all
 * derived classes)
 *    - and the generic method:mrpt::maps::CPointsMap::insertObservation()
 *  - mrpt::opengl::CPointCloud and mrpt::opengl::CPointCloudColoured is
 * supported by first converting
 *    this scan to a mrpt::maps::CPointsMap-derived class, then loading it into
 * the opengl object.
 *
 * Otherwise, the following API exists for accurate reconstruction of the
 * sensor path in SE(3) over time:
 *  - CObservationVelodyneScan::generatePointCloudAlongSE3Trajectory()
 *
 *  Note that this object has \b two timestamp fields:
 *  - The standard CObservation::timestamp field in the base class, which
 * should contain the accurate satellite-based UTC timestamp, and
 *  - the field CObservationVelodyneScan::originalReceivedTimestamp, with the
 * local computer-based timestamp based on the reception of the message in the
 * computer.
 *  Both timestamps correspond to the firing of the <b>first</b> laser in the
 * <b>first</b> CObservationVelodyneScan::scan_packets packet.
 *
 * \note New in MRPT 1.4.0
 * \sa CObservation, mrpt::maps::CPointsMap, mrpt::hwdrivers::CVelodyneScanner
 * \ingroup mrpt_obs_grp
 */
class CObservationVelodyneScan : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationVelodyneScan)

   public:
	/** @name Raw scan fixed parameters
		@{ */
	static const int SIZE_BLOCK = 100;
	static const int RAW_SCAN_SIZE = 3;
	static const int SCANS_PER_BLOCK = 32;
	static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

	static const float ROTATION_RESOLUTION;  // = 0.01f; /**< degrees */
	static const uint16_t ROTATION_MAX_UNITS =
		36000; /**< hundredths of degrees */

	static const float DISTANCE_MAX; /**< meters */
	static const float DISTANCE_RESOLUTION; /**< meters */
	static const float DISTANCE_MAX_UNITS;

	/** Blocks 0-31 */
	static const uint16_t UPPER_BANK = 0xeeff;
	/** Blocks 32-63 */
	static const uint16_t LOWER_BANK = 0xddff;

	static const int PACKET_SIZE = 1206;
	static const int POS_PACKET_SIZE = 512;
	static const int BLOCKS_PER_PACKET = 12;
	static const int PACKET_STATUS_SIZE = 4;
	static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

	static const uint8_t RETMODE_STRONGEST = 0x37;
	static const uint8_t RETMODE_LAST = 0x38;
	static const uint8_t RETMODE_DUAL = 0x39;
	/** @} */

	/** @name Scan data
		@{ */

#pragma pack(push, 1)
	struct laser_return_t
	{
		uint16_t distance;
		uint8_t intensity;
	};
	/** Raw Velodyne data block.
	 *  Each block contains data from either the upper or lower laser
	 *  bank.  The device returns three times as many upper bank blocks. */
	struct raw_block_t
	{
		uint16_t header;  ///< Block id: UPPER_BANK or LOWER_BANK
		uint16_t rotation;  ///< 0-35999, divide by 100 to get degrees
		laser_return_t laser_returns[SCANS_PER_BLOCK];
	};

	/** One unit of data from the scanner (the payload of one UDP DATA packet)
	 */
	struct TVelodyneRawPacket
	{
		raw_block_t blocks[BLOCKS_PER_PACKET];
		/** us from top of hour */
		uint32_t gps_timestamp;
		/** 0x37: strongest, 0x38: last, 0x39: dual return */
		uint8_t laser_return_mode;
		/** 0x21: HDL-32E, 0x22: VLP-16 */
		uint8_t velodyne_model_ID;
	};

	/** Payload of one POSITION packet */
	struct TVelodynePositionPacket
	{
		char unused1[198];
		uint32_t gps_timestamp;
		uint32_t unused2;
		/** the full $GPRMC message, as received by Velodyne, terminated with
		 * "\r\n\0" */
		char NMEA_GPRMC[72 + 234];
	};
#pragma pack(pop)

	/** The maximum range allowed by the device, in meters (e.g. 100m). Stored
	 * here by the driver while capturing based on the sensor model. */
	double minRange{1.0}, maxRange{130.0};
	/** The 6D pose of the sensor on the robot/vehicle frame of reference */
	mrpt::poses::CPose3D sensorPose;
	/** The main content of this object: raw data packet from the LIDAR. \sa
	 * point_cloud */
	std::vector<TVelodyneRawPacket> scan_packets;
	/** The calibration data for the LIDAR device. See
	 * mrpt::hwdrivers::CVelodyneScanner and mrpt::obs::VelodyneCalibration for
	 * details. */
	mrpt::obs::VelodyneCalibration calibration;
	/** The local computer-based timestamp based on the reception of the message
	 * in the computer. \sa has_satellite_timestamp, CObservation::timestamp in
	 * the base class, which should contain the accurate satellite-based UTC
	 * timestamp.  */
	mrpt::system::TTimeStamp originalReceivedTimestamp{INVALID_TIMESTAMP};
	/** If true, CObservation::timestamp has been generated from accurate
	 * satellite clock. Otherwise, no GPS data is available and timestamps are
	 * based on the local computer clock. */
	bool has_satellite_timestamp{false};

	mrpt::system::TTimeStamp getOriginalReceivedTimeStamp()
		const override;  // See base class docs

	/** See \a point_cloud and \a scan_packets */
	struct TPointCloud
	{
		std::vector<float> x, y, z;
		/** Color [0,255] */
		std::vector<uint8_t> intensity;
		/** Timestamp for each point (if `generatePerPointTimestamp`=true in
		 * `TGeneratePointCloudParameters`), or empty vector if not populated
		 * (default). */
		std::vector<mrpt::system::TTimeStamp> timestamp;
		/** Original azimuth of each point (if `generatePerPointAzimuth`=true,
		 * empty otherwise ) */
		std::vector<float> azimuth;
		/** Laser ID ("ring number") for each point (0-15 for a VLP-16, etc.) */
		std::vector<int16_t> laser_id;
		/** The list of point indices (in x,y,z,...) generated for each laserID
		 * (ring number). Generated only if `generatePointsForLaserID`=true in
		 * `TGeneratePointCloudParameters`*/
		std::vector<std::vector<std::size_t>> pointsForLaserID;

		inline std::size_t size() const { return x.size(); }
		void reserve(std::size_t n);
		/** Sets all vectors to zero length */
		void clear();
		/** Like clear(), but also enforcing freeing memory */
		void clear_deep();
	};

	/** Optionally, raw data can be converted into a 3D point cloud (local
	 * coordinates wrt the sensor, not the vehicle)
	 * with intensity (graylevel) information. See axes convention in
	 * mrpt::obs::CObservationVelodyneScan \sa generatePointCloud()
	 */
	TPointCloud point_cloud;
	/** @} */

	/** @name Related to conversion to 3D point cloud
	 * @{ */
	struct TGeneratePointCloudParameters
	{
		TGeneratePointCloudParameters();
		/** Minimum azimuth, in degrees (Default=0). Points will be generated
		 * only the the area of interest [minAzimuth, maxAzimuth] */
		double minAzimuth_deg{.0};
		/** Minimum azimuth, in degrees (Default=360). Points will be generated
		 * only the the area of interest [minAzimuth, maxAzimuth] */
		double maxAzimuth_deg{360.};
		/** Minimum (default=1.0f) and maximum (default: Infinity)
		 * distances/ranges for a point to be considered. Points must pass this
		 * (application specific) condition and also the minRange/maxRange
		 * values in CObservationVelodyneScan (sensor-specific). */
		float minDistance{1.0f}, maxDistance{std::numeric_limits<float>::max()};
		/** The limits of the 3D box (default=infinity) in sensor (not vehicle)
		 * local coordinates for the ROI filter \sa filterByROI */
		float ROI_x_min{-std::numeric_limits<float>::max()},
			ROI_x_max{std::numeric_limits<float>::max()},
			ROI_y_min{-std::numeric_limits<float>::max()},
			ROI_y_max{std::numeric_limits<float>::max()},
			ROI_z_min{-std::numeric_limits<float>::max()},
			ROI_z_max{std::numeric_limits<float>::max()};
		/** The limits of the 3D box (default=0) in sensor (not vehicle) local
		 * coordinates for the nROI filter \sa filterBynROI */
		float nROI_x_min{0}, nROI_x_max{0}, nROI_y_min{0}, nROI_y_max{0},
			nROI_z_min{0}, nROI_z_max{0};
		/** (Default:2.0 meters) Minimum distance between a point and its two
		 * neighbors to be considered an invalid point. */
		float isolatedPointsFilterDistance{2.0f};

		/** Enable ROI filter (Default:false): add points inside a given 3D box
		 */
		bool filterByROI{false};
		/** Enable nROI filter (Default:false): do NOT add points inside a given
		 * 3D box */
		bool filterBynROI{false};
		/** (Default:false) Simple filter to remove spurious returns (e.g. Sun
		 * reflected on large water extensions) */
		bool filterOutIsolatedPoints{false};
		/** (Default:true) In VLP16 dual mode, keep both or just one of the
		 * returns. */
		bool dualKeepStrongest{true}, dualKeepLast{true};
		/** (Default:false) If `true`, populate the vector timestamp */
		bool generatePerPointTimestamp{false};
		/** (Default:false) If `true`, populate the vector azimuth */
		bool generatePerPointAzimuth{false};
		/** (Default:false) If `true`, populate pointsForLaserID */
		bool generatePointsForLaserID{false};
	};

	/** Derive from this class to generate pointclouds into custom containers.
	 * \sa generatePointCloud() */
	struct PointCloudStorageWrapper
	{
		virtual void reserve(std::size_t n) {}
		virtual void resizeLaserCount([[maybe_unused]] std::size_t n) {}

		/** Process the insertion of a new (x,y,z) point to the cloud, in
		 * sensor-centric coordinates, with the exact timestamp of that LIDAR
		 * ray */
		virtual void add_point(
			float pt_x, float pt_y, float pt_z, uint8_t pt_intensity,
			const mrpt::system::TTimeStamp& tim, const float azimuth,
			uint16_t laser_id) = 0;
	};

	/** Generates the point cloud into the point cloud data fields in \a
	 * CObservationVelodyneScan::point_cloud
	 * where it is stored in local coordinates wrt the sensor (neither the
	 * vehicle nor the world).
	 * So, this method does not take into account the possible motion of the
	 * sensor through the world as it collects LIDAR scans.
	 * For high dynamics, see the more costly API
	 * generatePointCloudAlongSE3Trajectory()
	 * \note Points with ranges out of [minRange,maxRange] are discarded; as
	 * well, other filters are available in \a params.
	 * \sa generatePointCloudAlongSE3Trajectory(),
	 * TGeneratePointCloudParameters
	 */
	void generatePointCloud(
		const TGeneratePointCloudParameters& params =
			TGeneratePointCloudParameters());

	/** \overload For custom data storage as destination of the pointcloud. */
	void generatePointCloud(
		PointCloudStorageWrapper& dest,
		const TGeneratePointCloudParameters& params =
			TGeneratePointCloudParameters());

	/** Results for generatePointCloudAlongSE3Trajectory() */
	struct TGeneratePointCloudSE3Results
	{
		/** Number of points in the observation */
		size_t num_points{0};
		/** Number of points for which a valid interpolated SE(3) pose could be
		 * determined */
		size_t num_correctly_inserted_points{0};
	};

	/** An alternative to generatePointCloud() for cases where the motion of the
	 * sensor as it grabs one scan (360 deg horiz FOV) cannot be ignored.
	 * \param[in] vehicle_path Timestamped sequence of known poses for the
	 * VEHICLE. Recall that the sensor has a relative pose wrt the vehicle
	 * according to CObservationVelodyneScan::getSensorPose() &
	 * CObservationVelodyneScan::setSensorPose()
	 * \param[out] out_points The generated points, in the same coordinates
	 * frame than \a vehicle_path. Points are APPENDED to the list, so prevous
	 * contents are kept.
	 * \param[out] results_stats Stats
	 * \param[in] params Filtering and other parameters
	 * \sa generatePointCloud(), TGeneratePointCloudParameters
	 */
	void generatePointCloudAlongSE3Trajectory(
		const mrpt::poses::CPose3DInterpolator& vehicle_path,
		std::vector<mrpt::math::TPointXYZIu8>& out_points,
		TGeneratePointCloudSE3Results& results_stats,
		const TGeneratePointCloudParameters& params =
			TGeneratePointCloudParameters());

	/** @} */

	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = sensorPose;
	}  // See base class docs
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		sensorPose = newSensorPose;
	}  // See base class docs
	void getDescriptionAsText(
		std::ostream& o) const override;  // See base class docs

};  // End of class def.

}  // namespace obs

namespace typemeta
{  // Specialization must occur in the same namespace
MRPT_DECLARE_TTYPENAME_PTR_NAMESPACE(CObservationVelodyneScan, ::mrpt::obs)
}

}  // namespace mrpt
