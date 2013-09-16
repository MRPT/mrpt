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
		/// x-axis acceleration (m/sec<sup>2</sup>)
		IMU_X_ACC = 0 ,
		/// y-axis acceleration (m/sec<sup>2</sup>)
		IMU_Y_ACC,
		/// z-axis acceleration (m/sec<sup>2</sup>)
		IMU_Z_ACC,
		/// yaw angular velocity (rad/sec)
		IMU_YAW_VEL,
		/// pitch angular velocity (rad/sec)
		IMU_PITCH_VEL,
		/// roll angular velocity (rad/sec)
		IMU_ROLL_VEL,
		/// x-axis velocity (m/sec)
		IMU_X_VEL,
		/// y-axis velocity (m/sec)
		IMU_Y_VEL,
		/// z-axis velocity (m/sec)
		IMU_Z_VEL,
		/// yaw absolute value (rad)
		IMU_YAW,
		/// pitch absolute value (rad)
		IMU_PITCH,
		/// roll absolute value (rad)
		IMU_ROLL,
		/// x absolute value (meters)
		IMU_X,
		/// y absolute value (meters)
		IMU_Y,
		/// z absolute value (meters)
		IMU_Z, 
		/// x magnetic field value (gauss)
		IMU_MAG_X,
		/// y magnetic field value (gauss)
		IMU_MAG_Y,
		/// z magnetic field value (gauss)
		IMU_MAG_Z,
		/// air pressure (Pascals)
		IMU_PRESSURE,
		/// altitude from an altimeter (meters)
		IMU_ALTITUDE,
		/// temperature (degrees Celsius)
		IMU_TEMPERATURE,

		// Always leave this last value to reflect the number of enum values:
		COUNT_IMU_DATA_FIELDS
	};

	/** This class stores measurements from an Inertial Measurement Unit (IMU) (attitude estimation, raw gyroscope and accelerometer values), altimeters or magnetometers.
	 *
	 *  The order of the 21 raw values in each entry of mrpt::slam::CObservationIMU::rawMeasurements is (you can use the TIMUDataIndex "enum" symbolic names):
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
		<tr> <td> 15 </td> <td>IMU_MAG_X</td> <td> x magnetic field value (gauss)</td> </tr>
		<tr> <td> 16 </td> <td>IMU_MAG_X</td> <td> x magnetic field value (gauss)</td> </tr>
		<tr> <td> 17 </td> <td>IMU_MAG_X</td> <td> x magnetic field value (gauss)</td> </tr>
		<tr> <td> 18 </td> <td>IMU_PRESSURE</td> <td> air pressure (Pascals)</td> </tr>
		<tr> <td> 19 </td> <td>IMU_ALTITUDE</td> <td>altitude from an altimeter (meters)</td> </tr>
		<tr> <td> 20 </td> <td>IMU_TEMPERATURE</td> <td> temperature (degrees Celsius)</td> </tr>
		</table>
	 *
	 *  Values from 0 to 5 are direct measurements measured by accelerometers & gyroscopes. 
	 *  Values at indices from 6 to 14, if present, are estimates (dead reckoning) from the IMU unit.
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
			dataIsPresent(21,false),
			rawMeasurements(21,0)
		{ }

		/** Destructor
		  */
		virtual ~CObservationIMU()
		{ }

		/** The pose of the sensor on the robot.
		  */
		CPose3D  sensorPose;

		/** Each entry in this vector is true if the corresponding data index contains valid data (the IMU unit supplies that kind of data).
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
