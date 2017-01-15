/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationIMU_H
#define CObservationIMU_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace obs
{

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationIMU , CObservation,OBS_IMPEXP )

	/** Symbolic names for the indices of IMU data (refer to mrpt::obs::CObservationIMU)
	 * \ingroup mrpt_obs_grp
	  */
	enum TIMUDataIndex
	{
		IMU_X_ACC = 0 ,     //!< x-axis acceleration (local/vehicle frame) (m/sec<sup>2</sup>)
		IMU_Y_ACC,          //!< y-axis acceleration (local/vehicle frame) (m/sec<sup>2</sup>)
		IMU_Z_ACC,          //!< z-axis acceleration (local/vehicle frame) (m/sec<sup>2</sup>)
		IMU_YAW_VEL,        //!< yaw angular velocity (local/vehicle frame) (rad/sec)
		IMU_PITCH_VEL,      //!< pitch angular velocity (local/vehicle frame) (rad/sec)
		IMU_ROLL_VEL,       //!< roll angular velocity (local/vehicle frame) (rad/sec)
		IMU_X_VEL,          //!< x-axis velocity (global/navigation frame) (m/sec)
		IMU_Y_VEL,          //!< y-axis velocity (global/navigation  frame) (m/sec)
		IMU_Z_VEL,          //!< z-axis velocity (global/navigation  frame) (m/sec)
		IMU_YAW,            //!< orientation yaw absolute value (global/navigation frame) (rad)
		IMU_PITCH,          //!< orientation pitch absolute value (global/navigation frame) (rad)
		IMU_ROLL,           //!< orientation roll absolute value (global/navigation frame) (rad)
		IMU_X,              //!< x absolute value (global/navigation frame) (meters)
		IMU_Y,              //!< y absolute value (global/navigation frame) (meters)
		IMU_Z,              //!< z absolute value (global/navigation frame) (meters)
		IMU_MAG_X,          //!< x magnetic field value (local/vehicle frame) (gauss)
		IMU_MAG_Y,          //!< y magnetic field value (local/vehicle frame) (gauss)
		IMU_MAG_Z,          //!< z magnetic field value (local/vehicle frame) (gauss)
		IMU_PRESSURE,       //!< air pressure (Pascals)
		IMU_ALTITUDE,       //!< altitude from an altimeter (meters)
		IMU_TEMPERATURE,    //!< temperature (degrees Celsius)
		IMU_ORI_QUAT_X,     //!< Orientation Quaternion X (global/navigation frame)
		IMU_ORI_QUAT_Y,     //!< Orientation Quaternion Y (global/navigation frame)
		IMU_ORI_QUAT_Z,     //!< Orientation Quaternion Z (global/navigation frame)
		IMU_ORI_QUAT_W,     //!< Orientation Quaternion W (global/navigation frame)
		IMU_YAW_VEL_GLOBAL, //!< yaw angular velocity (global/navigation frame) (rad/sec)
		IMU_PITCH_VEL_GLOBAL,//!< pitch angular velocity (global/navigation frame) (rad/sec)
		IMU_ROLL_VEL_GLOBAL, //!< roll angular velocity (global/navigation frame) (rad/sec)
		IMU_X_ACC_GLOBAL,   //!< x-axis acceleration (global/navigation frame) (m/sec<sup>2</sup>)
		IMU_Y_ACC_GLOBAL,   //!< y-axis acceleration (global/navigation frame) (m/sec<sup>2</sup>)
		IMU_Z_ACC_GLOBAL,   //!< z-axis acceleration (global/navigation frame) (m/sec<sup>2</sup>)

		// Always leave this last value to reflect the number of enum values
		COUNT_IMU_DATA_FIELDS
	};

	/** This class stores measurements from an Inertial Measurement Unit (IMU) (attitude estimation, raw gyroscope and accelerometer values), altimeters or magnetometers.
	 *
	 *  The order of the values in each entry of mrpt::obs::CObservationIMU::rawMeasurements is defined as symbolic names in the enum mrpt::obs::TIMUDataIndex. 
	 *  Check it out also for reference on the unit and the coordinate frame used for each value.
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
			dataIsPresent(mrpt::obs::COUNT_IMU_DATA_FIELDS,false),
			rawMeasurements(mrpt::obs::COUNT_IMU_DATA_FIELDS,0)
		{ }

		/** Destructor
		  */
		virtual ~CObservationIMU()
		{ }

		/** The pose of the sensor on the robot. */
		mrpt::poses::CPose3D  sensorPose;

		/** Each entry in this vector is true if the corresponding data index contains valid data (the IMU unit supplies that kind of data).
		  *  See the top of this page for the meaning of the indices.
		  */
		vector_bool dataIsPresent;

		/** The accelerometer and/or gyroscope measurements taken by the IMU at the given timestamp.
		  * \sa dataIsPresent, CObservation::timestamp
		  */
		std::vector<double>  rawMeasurements;


		// See base class docs
		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE { out_sensorPose = sensorPose; }
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) MRPT_OVERRIDE { sensorPose = newSensorPose; }
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;


	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationIMU , CObservation,OBS_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
