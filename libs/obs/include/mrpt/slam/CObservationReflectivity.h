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
#ifndef CObservationReflectivity_H
#define CObservationReflectivity_H

#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationReflectivity , CObservation,OBS_IMPEXP )

	/** Declares a class derived from "CObservation" that encapsules a single short-range reflectivity measurement. 
	 *    This can be used for example to store readings from IR sensors (Lego Mindstorm NXT, etc...).
	 *
	 * \sa mrpt::slam::CReflectivityGridMap2D, CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationReflectivity : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationReflectivity )

	 public:
		CObservationReflectivity( );	//!< Default constructor.
		virtual ~CObservationReflectivity(); 

		/** The read reflectivity level, in the range [0,1] (0=black, 1=white).
		  */
		float reflectivityLevel;

		/** The pose of this sensor in robot's local coordinates.
		  */
		CPose3D sensorPose;

		/** 1-sigma of the sensor Gaussian noise (in the same normalized units than \a reflectivityLevel)
		  */
		float  sensorStdNoise;
		
		/** A general method to retrieve the sensor pose on the robot.
		  *  Note that most sensors will return a full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa setSensorPose
		  */
		void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose = sensorPose; }

		/** A general method to change the sensor pose on the robot.
		  *  Note that most sensors will use the full (6D) CPose3D, but see the derived classes for more details or special cases.
		  * \sa getSensorPose
		  */
		void setSensorPose( const CPose3D &newSensorPose ) { sensorPose = newSensorPose; }

	}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
