/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
#ifndef CObservationGasSensors_H
#define CObservationGasSensors_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace slam
{

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationGasSensors , CObservation, OBS_IMPEXP)

	/** Declares a class derived from "CObservation" that represents a set of readings from gas sensors.
	 *
	 * \sa CObservation
	 */
	class OBS_IMPEXP CObservationGasSensors : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationGasSensors )

	 public:
		/** Constructor.
		 */
		CObservationGasSensors(  );

		 /** The structure for each e-nose
		  */
		struct OBS_IMPEXP TObservationENose
		{
			TObservationENose() :
				eNosePoseOnTheRobot(),
				readingsVoltage(),
				sensorTypes(),
				hasTemperature(false),
				temperature()
			{}

			/** The pose of the sensors on the robot
			  */
			math::TPose3D			eNosePoseOnTheRobot;

			/** The set of readings (in volts) from the array of sensors (size of "sensorTypes" is the same that the size of "readingsVoltage")
			  */
			vector_float	readingsVoltage;

			/** The kind of sensors in the array (size of "sensorTypes" is the same that the size of "readingsVoltage")
			  *  The meaning of values for types of sensors is as follows:
			  *	 0x0000 : No sensor installed in this slot
			  *  0x2600 : Figaro TGS 2600
			  *  0x2602 : Figaro TGS 2602
			  *  0x2620 : Figaro TGS 2620
			  *  0x4161 : Figaro TGS 4161
			  */
			vector_int		sensorTypes;

			/** Must be true for "temperature" to contain a valid measurement
			  */
			bool			hasTemperature;

			/** Sensed temperature in Celcius (valid if hasTemperature=true only)
			  */
			float			temperature;

			/** True if the input to this chamber/enose is poluted air, False if clean air
			  */
			bool			isActive;

		};

		/** One entry per e-nose on the robot.
		  */
		std::vector<TObservationENose>		m_readings;

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
