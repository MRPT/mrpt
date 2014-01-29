/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
