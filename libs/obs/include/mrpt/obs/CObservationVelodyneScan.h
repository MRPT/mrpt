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
#include <mrpt/poses/CPose3D.h>

namespace mrpt {
namespace obs 
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationVelodyneScan, CObservation, OBS_IMPEXP)

	/** A "CObservation"-derived class representing the RAW DATA of a complete 360deg scan from a Velodyne scanner. 
	  * A scan comprises one or more "velodyne packets" (refer to Velodyne user manual).
	  *
	  * Methods to convert this RAW data into a point cloud:
	  * - XXX
	  *
	  * \note New in MRPT 1.3.3
	  * \sa CObservation, CPointsMap
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

		static const uint16_t UPPER_BANK = 0xeeff;
		static const uint16_t LOWER_BANK = 0xddff;

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
		/** Raw Velodyne data block.
		*  Each block contains data from either the upper or lower laser
		*  bank.  The device returns three times as many upper bank blocks. */
		typedef struct raw_block
		{
			uint16_t header;        ///< UPPER_BANK or LOWER_BANK
			uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
			uint8_t  data[BLOCK_DATA_SIZE];
		} raw_block_t;

		/** One unit of data from the scanner (the payload of one UDP packet) */
		struct OBS_IMPEXP TVelodyneRawPacket
		{
			raw_block_t blocks[BLOCKS_PER_PACKET];
			uint16_t revolution;
			uint8_t status[PACKET_STATUS_SIZE]; 
		};
#pragma pack(pop)

		double                       minRange,maxRange; //!< The maximum range allowed by the device, in meters (e.g. 100m). Stored here by the driver while capturing based on the sensor model.
		mrpt::poses::CPose3D         sensorPose; //!< The 6D pose of the sensor on the robot/vehicle frame of reference
		std::vector<TVelodyneRawPacket> scan_packets;
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
