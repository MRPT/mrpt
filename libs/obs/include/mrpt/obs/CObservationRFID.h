/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CObservationRFID_H
#define CObservationRFID_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace obs
{

	/** This represents one or more RFID tags observed by a receiver.
	 *
	 * \sa CObservation, mrpt::hwdrivers::CImpinjRFID for a software sensor capable of reading this kind of observations.
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationRFID : public CObservation
	{
		DEFINE_SERIALIZABLE( CObservationRFID )

	 public:
		/** Constructor */
		CObservationRFID();

		 /** @name The data members
		  * @{ */
		/** The location of the sensing antenna on the robot coordinate framework */
		mrpt::poses::CPose3D  sensorPoseOnRobot; 

        /** Each of the individual readings of a RFID tag */
        struct OBS_IMPEXP TTagReading
        {
            TTagReading() : power(-1000) {}

            /** The power or signal strength as sensed by the RFID receiver (in dBm) */
            double      power;  
            /** EPC code of the observed tag */
            std::string epc; 
            /** Port of the antenna that did the reading */
            std::string antennaPort; 
        };

        /** The vector of individual tag observations */
        std::vector<TTagReading>  tag_readings;

		inline uint32_t getNtags() const { return tag_readings.size(); }

		/** @} */

		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const override;
		/** A general method to change the sensor pose on the robot.
		  *  It has no effects in this class
		  * \sa getSensorPose  */
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) override;
		// See base class docs
		void getDescriptionAsText(std::ostream &o) const override;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationRFID, CObservation, OBS_IMPEXP)


	} // End of namespace
} // End of namespace

#endif
