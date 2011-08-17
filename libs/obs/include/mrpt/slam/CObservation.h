/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservation, mrpt::utils::CSerializable,OBS_IMPEXP )

		/** Declares a class that represents any robot's observation.
			   This is a base class for many types of sensors
				 observations. Users can add a new observation type
				 creating a new class deriving from this one.<br>
			   <b>IMPORTANT</b>: Observations doesn't include any information about the
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
