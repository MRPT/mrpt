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
#ifndef CObservationIMU_H
#define CObservationIMU_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace slam
{

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationIMU , CObservation,OBS_IMPEXP )

	/** Symbolic names for the indices of IMU data (refer to mrpt::slam::CObservationIMU)
	 * \ingroup mrpt_obs_grp
	  */
	enum TIMUDataIndex
	{
		IMU_X_ACC = 0,
		IMU_Y_ACC,
		IMU_Z_ACC,
		IMU_YAW_VEL,
		IMU_PITCH_VEL,
		IMU_ROLL_VEL,
		IMU_X_VEL,
		IMU_Y_VEL,
		IMU_Z_VEL,
		IMU_YAW,
		IMU_PITCH,
		IMU_ROLL,
		IMU_X,
		IMU_Y,
		IMU_Z
	};

	/** This class stores measurements from an Inertial Measurement Unit (IMU), and/or its attitude estimation (integration of raw measurements).
	 *
	 *  The order of the 15 raw values in each entry of mrpt::slam::CObservationIMU::rawMeasurements is (you can use the TIMUDataIndex "enum" symbolic names):
		<table>
		<tr> <td> 0 </td> <td>IMU_X_ACC</td> <td> x-axis acceleration (m/sec<sup>2</sup>)</td> </tr>
		<tr> <td> 1 </td> <td>IMU_Y_ACC</td> <td> y-axis acceleration (m/sec<sup>2</sup>)</td> </tr>
		<tr> <td> 2 </td> <td>IMU_Z_ACC</td> <td> z-axis acceleration (m/sec<sup>2</sup>)</td> </tr>
		<tr> <td> 3 </td> <td>IMU_YAW_VEL</td> <td> yaw angular velocity (rad/sec)</td> </tr>
		<tr> <td> 4 </td> <td>IMU_PITCH_VEL</td> <td> pitch angular velocity (rad/sec)</td> </tr>
		<tr> <td> 5 </td> <td>IMU_ROLL_VEL</td> <td> roll angular velocity (rad/sec)</td> </tr>
		<tr> <td> 6 </td> <td>IMU_X_VEL</td> <td> x-axis velocity (m/sec)</td> </tr>
		<tr> <td> 7 </td> <td>IMU_Y_VEL</td> <td> y-axis velocity (m/sec)</td> </tr>
		<tr> <td> 8 </td> <td>IMU_Z_VEL</td> <td> z-axis velocity (m/sec)</td> </tr>
		<tr> <td> 9 </td> <td>IMU_YAW</td> <td> yaw absolute value (rad)</td> </tr>
		<tr> <td> 10 </td> <td>IMU_PITCH</td> <td> pitch absolute value (rad)</td> </tr>
		<tr> <td> 11 </td> <td>IMU_ROLL</td> <td> roll absolute value (rad)</td> </tr>
		<tr> <td> 12 </td> <td>IMU_X</td> <td> x absolute value (meters)</td> </tr>
		<tr> <td> 13 </td> <td>IMU_Y</td> <td> y absolute value (meters)</td> </tr>
		<tr> <td> 14 </td> <td>IMU_Z</td> <td> z absolute value (meters)</td> </tr>
		</table>
	 *
	 *  The first 6 values are directly measured by accelerometers & gyroscopes. The rest, if present, are estimates from the IMU unit.
	 *
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationIMU : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationIMU )

	 public:
		/** Constructor.
		 */
		CObservationIMU(  ) :
			sensorPose(),
			dataIsPresent(15,false),
			rawMeasurements(15,0)
		{ }

		/** Destructor
		  */
		virtual ~CObservationIMU()
		{ }

		/** The pose of the sensor on the robot.
		  */
		CPose3D  sensorPose;

		/** Each of the 15 entries of this vector is true if the corresponding data index contains valid data (the IMU unit supplies that kind of data).
		  *  See the top of this page for the meaning of the indices.
		  */
		vector_bool dataIsPresent;

		/** The accelerometer and/or gyroscope measurements taken by the IMU at the given timestamp.
		  * \sa dataIsPresent, CObservation::timestamp
		  */
		std::vector<double>  rawMeasurements;


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
