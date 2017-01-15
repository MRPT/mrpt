/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef COBSERVATION_H
#define COBSERVATION_H


#include <mrpt/obs/link_pragmas.h>

#include <mrpt/utils/CSerializable.h>
#include <mrpt/system/datetime.h>
#include <mrpt/math/math_frwds.h>

namespace mrpt
{
	/** This namespace contains representation of robot actions and observations */
	namespace obs
	{
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
		 *  This is a base class for many types of sensor observations.
		 *  Users can add new observation types creating a new class deriving from this one.
		 *
		 *  <b>IMPORTANT</b>: Observations don't include any information about the robot pose,
		 *   just  raw sensory data and, where aplicable, information about the sensor position and
		 *   orientation in the local frame of the robot.
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
			mrpt::system::TTimeStamp timestamp; //!< The associated UTC time-stamp. Where available, this should contain the accurate satellite-based timestamp of the sensor reading. \sa getOriginalReceivedTimeStamp(), getTimeStamp()
			std::string              sensorLabel;//!< An arbitrary label that can be used to identify the sensor.

			/** Returns CObservation::timestamp for all kind of observations \sa getOriginalReceivedTimeStamp() */
			mrpt::system::TTimeStamp getTimeStamp() const { return timestamp;  }
			/** By default, returns CObservation::timestamp but in sensors capable of satellite (or otherwise) accurate UTC timing of readings, this contains the computer-based timestamp of reception, which may be slightly different than \a timestamp \sa getTimeStamp()  */
			virtual mrpt::system::TTimeStamp getOriginalReceivedTimeStamp() const { return timestamp; }
		/** @} */

		CObservation(); //!< Constructor: It sets the initial timestamp to current time

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
		inline bool insertObservationInto( METRICMAP *theMap, const mrpt::poses::CPose3D *robotPose = NULL ) const
		{
			return theMap->insertObservation(this,robotPose);
		}

		/** A general method to retrieve the sensor pose on the robot.
		*  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		* \sa setSensorPose
		*/
		virtual void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const = 0;

		/** A general method to retrieve the sensor pose on the robot.
		*  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		* \sa setSensorPose
		*/
		void getSensorPose( mrpt::math::TPose3D &out_sensorPose ) const;

		/** A general method to change the sensor pose on the robot.
		*  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		* \sa getSensorPose
		*/
		virtual void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) = 0;

		/** A general method to change the sensor pose on the robot.
		*  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		* \sa getSensorPose
		*/
		void setSensorPose( const mrpt::math::TPose3D &newSensorPose );

		/** Build a detailed, multi-line textual description of the observation contents and dump it to the output stream. 
		  * \note If overried by derived classes, call base CObservation::getDescriptionAsText() first to show common information. 
		  * \note This is the text that appears in RawLogViewer when selecting an object in the dataset */
		virtual void getDescriptionAsText(std::ostream &o) const;

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
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservation, mrpt::utils::CSerializable, OBS_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
