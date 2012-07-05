/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
