/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationVelodyneScan_H
#define CObservationVelodyneScan_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSinCosLookUpTableFor2DScans.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/obs/VelodyneCalibration.h>

namespace mrpt {
namespace obs 
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationVelodyneScan, CObservation, OBS_IMPEXP)

	/** A `CObservation`-derived class for RAW DATA (and optionally, point cloud) of scans from 3D Velodyne LIDAR scanners.
	  * A scan comprises one or more "velodyne packets" (refer to Velodyne user manual).
	  *
	  * <h2>Main data fields:</h2><hr>
	  * - CObservationVelodyneScan::scan_packets with raw data packets.
	  * - CObservationVelodyneScan::point_cloud normally empty after grabbing for efficiency, can be generated calling \a CObservationVelodyneScan::generatePointCloud()
	  * 
	  * If it can be assumed that the sensor moves SLOWLY through the environment (i.e. its pose can be approximated to be the same since the beginning to the end of one complete scan)
	  * then this observation can be converted / loaded into the following other classes:
	  *  - **TODO** mrpt::opengl::CPointCloud 
	  *  - **TODO** mrpt::opengl::CPointCloudColoured
	  *  - Maps of points: See mrpt::maps::CPointsMap::loadFromRangeScan() (available in all derived classes)
	  *
	  * Otherwise, the following API exists for accurate reconstruction of the sensor path in SE(3) over time: 
	  *  - **TODO** XXXX
	  *
	  * \note New in MRPT 1.3.3
	  * \sa CObservation, CPointsMap, CVelodyneScanner
	  * \ingroup mrpt_obs_grp
	  */
	class OBS_IMPEXP CObservationVelodyneScan : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationVelodyneScan )

	 public:
		CObservationVelodyneScan();
		virtual ~CObservationVelodyneScan( );

		/** @name Raw scan fixed parameters
		    @{ */
		static const int SIZE_BLOCK = 100;
		static const int RAW_SCAN_SIZE = 3;
		static const int SCANS_PER_BLOCK = 32;
		static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

		static const float ROTATION_RESOLUTION; // = 0.01f; /**< degrees */
		static const uint16_t ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

		static const float DISTANCE_MAX;        /**< meters */
		static const float DISTANCE_RESOLUTION;  /**< meters */
		static const float DISTANCE_MAX_UNITS;

		static const uint16_t UPPER_BANK = 0xeeff;  //!< Blocks 0-31
		static const uint16_t LOWER_BANK = 0xddff;  //!< Blocks 32-63

		/** Special Defines for VLP16 support **/
		static const int VLP16_FIRINGS_PER_BLOCK = 2;
		static const int VLP16_SCANS_PER_FIRING = 16;
		static const int VLP16_BLOCK_TDURATION = 110.592;
		static const int VLP16_DSR_TOFFSET = 2.304;
		static const int VLP16_FIRING_TOFFSET = 55.296;

		static const int PACKET_SIZE = 1206;
		static const int BLOCKS_PER_PACKET = 12;
		static const int PACKET_STATUS_SIZE = 4;
		static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
		/** @} */

		/** @name Scan data
		    @{ */

#pragma pack(push,1)
		struct OBS_IMPEXP laser_return_t {
			uint16_t distance;
			uint8_t  intensity;
		};
		/** Raw Velodyne data block.
		*  Each block contains data from either the upper or lower laser
		*  bank.  The device returns three times as many upper bank blocks. */
		struct OBS_IMPEXP raw_block_t
		{
			uint16_t        header;        ///< Block id: UPPER_BANK or LOWER_BANK
			uint16_t        rotation;      ///< 0-35999, divide by 100 to get degrees
			laser_return_t  laser_returns[SCANS_PER_BLOCK];
		} ;

		/** One unit of data from the scanner (the payload of one UDP packet) */
		struct OBS_IMPEXP TVelodyneRawPacket
		{
			raw_block_t blocks[BLOCKS_PER_PACKET];
			uint32_t gps_timestamp;
			uint8_t  laser_return_mode;  //!< 0x37: strongest, 0x38: last, 0x39: dual return
			uint8_t  velodyne_model_ID;  //!< 0x21: HDL-32E, 0x22: VLP-16
		};
#pragma pack(pop)

		double                       minRange,maxRange; //!< The maximum range allowed by the device, in meters (e.g. 100m). Stored here by the driver while capturing based on the sensor model.
		mrpt::poses::CPose3D         sensorPose; //!< The 6D pose of the sensor on the robot/vehicle frame of reference
		std::vector<TVelodyneRawPacket> scan_packets;  //!< The main content of this object: raw data packet from the LIDAR. \sa point_cloud

		/** See \a point_cloud and \a scan_packets */
		struct OBS_IMPEXP TPointCloud
		{
			std::vector<float>   x,y,z;
			std::vector<uint8_t> intensity; //!< Color [0,255]

			inline size_t size() const {
				return x.size();
			}
			inline void clear() {
				x.clear();
				y.clear();
				z.clear();
				intensity.clear();
			}
			inline void clear_deep() {
				x.swap(std::vector<float>());
				y.swap(std::vector<float>());
				z.swap(std::vector<float>());
				intensity.swap(std::vector<uint8_t>());
			}
		};

		/** Optionally, raw data can be converted into a 3D point cloud (local coordinates wrt the vehicle, not sensor, according to sensorPose) with intensity (graylevel) information. \sa generatePointCloud() */
		TPointCloud point_cloud;
		/** @} */

		/** @name Related to conversion to 3D point cloud 
		  * @{ */
		struct OBS_IMPEXP TGeneratePointCloudParameters
		{
		};

		/** Generates the point cloud into the point cloud data fields in \a CObservationVelodyneScan::point_cloud
		  * \note Points with ranges out of [minRange,maxRange] are discarded.
		  */
		void generatePointCloud(const TGeneratePointCloudParameters &params);

		/** @} */
		
		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const { out_sensorPose = sensorPose; } // See base class docs
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) { sensorPose = newSensorPose; } // See base class docs
		virtual void getDescriptionAsText(std::ostream &o) const; // See base class docs

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationVelodyneScan, CObservation, OBS_IMPEXP)


	} // End of namespace

	namespace utils { // Specialization must occur in the same namespace
		MRPT_DECLARE_TTYPENAME_PTR_NAMESPACE(CObservationVelodyneScan, ::mrpt::obs)
	}

} // End of namespace

#endif
