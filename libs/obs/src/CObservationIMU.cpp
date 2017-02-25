/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/utils/CStream.h>
#include <mrpt/obs/CObservationIMU.h>
//#include <mrpt/math/CMatrixD.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationIMU, CObservation,mrpt::obs)

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationIMU::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 3;  // v1->v2 was only done to fix a bug in the ordering of YAW/PITCH/ROLL rates.
	else
	{
		out << sensorPose
		    << dataIsPresent
		    << timestamp;

		out << rawMeasurements;
		// Version 3: Added 6 new raw measurements (IMU_MAG_X=15 to IMU_TEMPERATURE=20)

		out << sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationIMU::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		in >> sensorPose;
		in >> dataIsPresent;

		in >> timestamp;

		// In version 0 it was a vector of floats:
		if (version<1)
		{
			mrpt::math::CVectorFloat	tmp;
			in >> tmp;
			rawMeasurements.resize(tmp.size());
			for (size_t i=0;i<rawMeasurements.size();i++)
				rawMeasurements[i] = tmp[i];
		}
		else
		{
			in >> rawMeasurements;
		}

		if (version<2)
		{
			// A bug in the grabbing from XSens IMU's made /ROLL rates to be stored in the wrong order:
			std::swap(rawMeasurements[IMU_YAW_VEL],rawMeasurements[IMU_ROLL_VEL]);
		}
		else
		{
			// v2: nothing to do, data is already in the right order.
		}

		in >> sensorLabel;

		// Fill new entries with default values:
		if (dataIsPresent.size()<COUNT_IMU_DATA_FIELDS)
		{
			const size_t nOld = dataIsPresent.size();
			ASSERT_(rawMeasurements.size()==dataIsPresent.size());

			dataIsPresent.resize(COUNT_IMU_DATA_FIELDS);
			rawMeasurements.resize(COUNT_IMU_DATA_FIELDS);
			for (size_t i=nOld;i<COUNT_IMU_DATA_FIELDS;i++) {
				dataIsPresent[i]=false;
				rawMeasurements[i]=0;
			}
		}
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationIMU::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;;
	CObservation::getDescriptionAsText(o);

	o << "Sensor pose on the robot: " << sensorPose << endl;

	o << format("Orientation (degrees): (yaw,pitch,roll)=(%.06f, %.06f, %.06f)\n\n",
		RAD2DEG( rawMeasurements[IMU_YAW] ),
		RAD2DEG( rawMeasurements[IMU_PITCH] ),
		RAD2DEG( rawMeasurements[IMU_ROLL] ) );

	// Units:
	// Use "COUNT_IMU_DATA_FIELDS" so a compile error happens if the sizes don't fit ;-)
	static const char * imu_units[ mrpt::obs::COUNT_IMU_DATA_FIELDS ] =
	{
		"m/s^2", //	IMU_X_ACC,
		"m/s^2", //	IMU_Y_ACC,
		"m/s^2", //	IMU_Z_ACC,
		"rad/s", //	IMU_YAW_VEL,
		"rad/s", //	IMU_PITCH_VEL,
		"rad/s", //	IMU_ROLL_VEL,
		"m/s", //	IMU_X_VEL,
		"m/s", //	IMU_Y_VEL,
		"m/s", //	IMU_Z_VEL,
		"rad", //	IMU_YAW,
		"rad", //	IMU_PITCH,
		"rad", //	IMU_ROLL,
		"m", //	IMU_X,
		"m", //	IMU_Y,
		"m",  //	IMU_Z
		"gauss", // IMU_MAG_X,
		"gauss", // IMU_MAG_Y,
		"gauss", // IMU_MAG_Z,
		"Pa", // IMU_PRESSURE,
		"m", // IMU_ALTITUDE,
		"deg.", // IMU_TEMPERATURE,
		"qx", // IMU_ORI_QUAT_X,
		"qy", // IMU_ORI_QUAT_Y,
		"qz", // IMU_ORI_QUAT_Z,
		"qw", // IMU_ORI_QUAT_W,
		"rad/s", //	IMU_YAW_VEL_GLOBAL
		"rad/s", //	IMU_PITCH_VEL_GLOBAL
		"rad/s", //	IMU_ROLL_VEL_GLOBAL
		"m/s^2", //	IMU_X_ACC_GLOBAL
		"m/s^2", //	IMU_Y_ACC_GLOBAL
		"m/s^2"  //	IMU_Z_ACC_GLOBAL
	};

#define DUMP_IMU_DATA(x)  \
	o << format("%15s = ",#x); \
	if (dataIsPresent[x]) \
	o << format("%10f %s\n", rawMeasurements[x], imu_units[x]); \
	else  	o << "(not present)\n";

	DUMP_IMU_DATA(IMU_X_ACC)
	DUMP_IMU_DATA(IMU_Y_ACC)
	DUMP_IMU_DATA(IMU_Z_ACC)
	DUMP_IMU_DATA(IMU_YAW_VEL)
	DUMP_IMU_DATA(IMU_PITCH_VEL)
	DUMP_IMU_DATA(IMU_ROLL_VEL)
	DUMP_IMU_DATA(IMU_X_VEL)
	DUMP_IMU_DATA(IMU_Y_VEL)
	DUMP_IMU_DATA(IMU_Z_VEL)
	DUMP_IMU_DATA(IMU_YAW)
	DUMP_IMU_DATA(IMU_PITCH)
	DUMP_IMU_DATA(IMU_ROLL)
	DUMP_IMU_DATA(IMU_X)
	DUMP_IMU_DATA(IMU_Y)
	DUMP_IMU_DATA(IMU_Z)
	DUMP_IMU_DATA(IMU_MAG_X)
	DUMP_IMU_DATA(IMU_MAG_Y)
	DUMP_IMU_DATA(IMU_MAG_Z)
	DUMP_IMU_DATA(IMU_PRESSURE)
	DUMP_IMU_DATA(IMU_ALTITUDE)
	DUMP_IMU_DATA(IMU_TEMPERATURE)
	DUMP_IMU_DATA(IMU_ORI_QUAT_X)
	DUMP_IMU_DATA(IMU_ORI_QUAT_Y)
	DUMP_IMU_DATA(IMU_ORI_QUAT_Z)
	DUMP_IMU_DATA(IMU_ORI_QUAT_W)
	DUMP_IMU_DATA(IMU_YAW_VEL_GLOBAL)
	DUMP_IMU_DATA(IMU_PITCH_VEL_GLOBAL)
	DUMP_IMU_DATA(IMU_ROLL_VEL_GLOBAL)
	DUMP_IMU_DATA(IMU_X_ACC_GLOBAL)
	DUMP_IMU_DATA(IMU_Y_ACC_GLOBAL)
	DUMP_IMU_DATA(IMU_Z_ACC_GLOBAL)


}
