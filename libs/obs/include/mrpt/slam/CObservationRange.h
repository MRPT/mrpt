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
#ifndef CObservationRange_H
#define CObservationRange_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationRange , CObservation,OBS_IMPEXP )

	/** Declares a class derived from "CObservation" that
	       encapsules a single range measurement, and associated parameters. This can be used
	 *     for example to store measurements from infrared proximity sensors (IR) or ultrasonic sensors (sonars).
	 *
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationRange : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationRange )

	 public:
		/** Default constructor.
		 */
		CObservationRange( );

		 /** The data members
		  */
		float	minSensorDistance;
		float	maxSensorDistance;
		float	sensorConeApperture;  //!< Cone aperture of each ultrasonic beam, in radians.

		struct OBS_IMPEXP TMeasurement
		{
			TMeasurement() : sensorID(0), sensorPose(), sensedDistance(0)
			{}

			/** Some kind of sensor ID which identifies it on the bus (if applicable, 0 otherwise)
			  */
			uint16_t	sensorID;

			/** The 6D position of the sensor on the robot.
			  */
			math::TPose3D	sensorPose;

			/** The measured range, in meters (or a value of 0 if there was no detected echo).
			  */
			float		sensedDistance;
		};

		typedef std::deque<TMeasurement>                    TMeasurementList;
		typedef std::deque<TMeasurement>::const_iterator    const_iterator;
		typedef std::deque<TMeasurement>::iterator          iterator;

		TMeasurementList sensedData; //!< All the measurements


		iterator begin() { return sensedData.begin(); }
		iterator end() { return sensedData.end(); }
		const_iterator begin() const { return sensedData.begin(); }
		const_iterator end() const { return sensedData.end(); }

		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const;


		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose );


	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
