/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef COBSERVATION_H
#define COBSERVATION_H


#include <mrpt/obs/link_pragmas.h>

#include <mrpt/utils/CSerializable.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/datetime.h>

/** The main namespace for all the Mobile Robot Programming Toolkit (MRPT) C++ libraries. */
namespace mrpt
{
	namespace poses
	{
		class CPosePDF;
		class CPose2D;
		class CPose3D;
	}

	namespace math { struct TPose3D; }

	/** This namespace contains algorithms for SLAM, localization, map building, representation of robot's actions and observations, and representation of many kinds of metric maps.
	  */
	namespace slam
	{
		using namespace poses;

		/** Used for CObservationBearingRange::TMeasurement::beaconID
		 * \ingroup mrpt_obs_grp
		  */
		#define INVALID_LANDMARK_ID 	(-1)

		/** Used for CObservationBeaconRange
		 * \ingroup mrpt_obs_grp
		  */
		#define INVALID_BEACON_ID  		(-1)

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservation, mrpt::utils::CSerializable, OBS_IMPEXP )

		/** Declares a class that represents any robot's observation.
			   This is a base class for many types of sensors
				 observations. Users can add a new observation type
				 creating a new class deriving from this one.<br>
			   <b>IMPORTANT</b>: Observations don't include any information about the
				robot pose beliefs, just the raw observation and, where
				aplicable, information about sensor position or
				orientation respect to robotic coordinates origin.
		 *
		 * \sa CSensoryFrame, CMetricMap
	 	 * \ingroup mrpt_obs_grp
		 */
		class OBS_IMPEXP CObservation : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_VIRTUAL_SERIALIZABLE(CObservation)

		protected:
			void swap(CObservation &o);  //!< Swap with another observation, ONLY the data defined here in the base class CObservation. It's protected since it'll be only called from child classes that should know what else to swap appart from these common data.

		 public:

		 /** @name Data common to any observation
		     @{ */

			/** The associated time-stamp.
			*/
			mrpt::system::TTimeStamp	timestamp;

			/** An arbitrary label that can be used to identify the sensor.
			*/
			std::string			sensorLabel;

		/** @} */

		/** Constructor: It sets the initial timestamp to current time
		*/
		CObservation();


		/** This method is equivalent to:
		* \code
		*		map->insertObservation(this, robotPose)
		* \endcode
		* \param theMap The map where this observation is to be inserted: the map will be updated.
		* \param robotPose The pose of the robot base for this observation, relative to the target metric map. Set to NULL (default) to use (0,0,0deg)
		*
		* \return Returns true if the map has been updated, or false if this observations
		*			has nothing to do with a metric map (for example, a sound observation).
		*
		* \sa CMetricMap, CMetricMap::insertObservation
		*/
		template <class METRICMAP>
		inline bool insertObservationInto( METRICMAP *theMap, const CPose3D *robotPose = NULL ) const
		{
			return theMap->insertObservation(this,robotPose);
		}

		/** A general method to retrieve the sensor pose on the robot.
		*  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		* \sa setSensorPose
		*/
		virtual void getSensorPose( CPose3D &out_sensorPose ) const = 0;

		/** A general method to retrieve the sensor pose on the robot.
		*  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		* \sa setSensorPose
		*/
		void getSensorPose( mrpt::math::TPose3D &out_sensorPose ) const;

		/** A general method to change the sensor pose on the robot.
		*  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		* \sa getSensorPose
		*/
		virtual void setSensorPose( const CPose3D &newSensorPose ) = 0;

		/** A general method to change the sensor pose on the robot.
		*  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		* \sa getSensorPose
		*/
		void setSensorPose( const mrpt::math::TPose3D &newSensorPose );

		/** @name Delayed-load manual control methods.
		    @{ */

		/** Makes sure all images and other fields which may be externally stored are loaded in memory.
		  *  Note that for all CImages, calling load() is not required since the images will be automatically loaded upon first access, so load() shouldn't be needed to be called in normal cases by the user.
		  *  If all the data were alredy loaded or this object has no externally stored data fields, calling this method has no effects.
		  * \sa unload
		  */
		virtual void load() const { /* Default implementation: do nothing */ }
		/** Unload all images, for the case they being delayed-load images stored in external files (othewise, has no effect).
		  * \sa load
		  */
		virtual void unload() { /* Default implementation: do nothing */ }

		/** @} */

		}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
