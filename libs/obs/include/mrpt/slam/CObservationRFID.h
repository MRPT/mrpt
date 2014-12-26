/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationRFID_H
#define CObservationRFID_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationRFID, CObservation, OBS_IMPEXP)

	/** This represents one or more RFID tags observed by a receiver.
	 *
	 * \sa CObservation, mrpt::hwdrivers::CImpinjRFID for a software sensor capable of reading this kind of observations.
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationRFID : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationRFID )

	 public:
		/** Constructor */
		CObservationRFID();

		 /** @name The data members
		  * @{ */
		mrpt::poses::CPose3D  sensorPoseOnRobot; //!< The location of the sensing antenna on the robot coordinate framework

        /** Each of the individual readings of a RFID tag */
        struct OBS_IMPEXP TTagReading
        {
            TTagReading() : power(-1000) {}

            double      power;  //!< The power or signal strength as sensed by the RFID receiver (in dBm)
            std::string epc; //!< EPC code of the observed tag
            std::string antennaPort; //!< Port of the antenna that did the reading
        };

        /** The vector of individual tag observations */
        std::vector<TTagReading>  tag_readings;

		inline uint32_t getNtags() const { return tag_readings.size(); }

		/** @} */

		void getSensorPose( CPose3D &out_sensorPose ) const;
		/** A general method to change the sensor pose on the robot.
		  *  It has no effects in this class
		  * \sa getSensorPose  */
		void setSensorPose( const CPose3D &newSensorPose );
		// See base class docs
		virtual void getDescriptionAsText(std::ostream &o) const;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationRFID, CObservation, OBS_IMPEXP)


	} // End of namespace
} // End of namespace

#endif
