/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
//
#include <mrpt/math/CVectorDynamic.h>  // CVectorFloat
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationIMU, CObservation, mrpt::obs)

CObservationIMU::CObservationIMU()
{
	dataIsPresent.fill(false);
	rawMeasurements.fill(0);
}

uint8_t CObservationIMU::serializeGetVersion() const { return 4; }
void CObservationIMU::serializeTo(mrpt::serialization::CArchive& out) const
{
	// v1->v2 was only done to fix a bug in the ordering of
	// YAW/PITCH/ROLL rates.
	// Version 3: Added 6 new raw measurements (IMU_MAG_X=15 to
	// IMU_TEMPERATURE=20)
	// v4: switch std::vector -> std::array

	out << sensorPose;
	out.WriteBuffer(dataIsPresent.data(), dataIsPresent.size());
	out << timestamp;
	out.WriteBufferFixEndianness(
		rawMeasurements.data(), rawMeasurements.size());
	out << sensorLabel;
}

void CObservationIMU::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 4:
			in >> sensorPose;
			in.ReadBuffer(dataIsPresent.data(), dataIsPresent.size());
			in >> timestamp;
			in.ReadBufferFixEndianness(
				rawMeasurements.data(), rawMeasurements.size());
			in >> sensorLabel;
			break;

		case 0:
		case 1:
		case 2:
		case 3:
		{
			// Fill new entries with default values:
			dataIsPresent.fill(false);
			rawMeasurements.fill(0);

			in >> sensorPose;
			{
				std::vector<bool> d;
				in >> d;
				for (size_t i = 0; i < d.size(); i++)
					dataIsPresent.at(i) = d.at(i);
			}

			in >> timestamp;

			// In version 0 it was a vector of floats:
			if (version < 1)
			{
				mrpt::math::CVectorFloat tmp;
				in >> tmp;
				for (int i = 0; i < tmp.size(); i++)
					rawMeasurements.at(i) = tmp[i];
			}
			else
			{
				std::vector<double> d;
				in >> d;
				for (size_t i = 0; i < d.size(); i++)
					rawMeasurements.at(i) = d.at(i);
			}

			if (version < 2)
			{
				// A bug in the grabbing from XSens IMU's made /ROLL rates to be
				// stored in the wrong order:
				std::swap(
					rawMeasurements[IMU_YAW_VEL],
					rawMeasurements[IMU_ROLL_VEL]);
			}
			else
			{
				// v2: nothing to do, data is already in the right order.
			}

			in >> sensorLabel;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CObservationIMU::getDescriptionAsText(std::ostream& o) const
{
	using namespace std;
	;
	CObservation::getDescriptionAsText(o);

	o << "Sensor pose on the robot: " << sensorPose << endl;

	o << format(
		"Orientation (degrees): (yaw,pitch,roll)=(%.06f, %.06f, %.06f)\n\n",
		RAD2DEG(rawMeasurements[IMU_YAW]), RAD2DEG(rawMeasurements[IMU_PITCH]),
		RAD2DEG(rawMeasurements[IMU_ROLL]));

	// Units:
	// Use "COUNT_IMU_DATA_FIELDS" so a compile error happens if the sizes don't
	// fit ;-)
	static const char* imu_units[mrpt::obs::COUNT_IMU_DATA_FIELDS] = {
		"m/s^2",  //	IMU_X_ACC,
		"m/s^2",  //	IMU_Y_ACC,
		"m/s^2",  //	IMU_Z_ACC,
		"rad/s",  //	IMU_YAW_VEL,
		"rad/s",  //	IMU_PITCH_VEL,
		"rad/s",  //	IMU_ROLL_VEL,
		"m/s",	//	IMU_X_VEL,
		"m/s",	//	IMU_Y_VEL,
		"m/s",	//	IMU_Z_VEL,
		"rad",	//	IMU_YAW,
		"rad",	//	IMU_PITCH,
		"rad",	//	IMU_ROLL,
		"m",  //	IMU_X,
		"m",  //	IMU_Y,
		"m",  //	IMU_Z
		"gauss",  // IMU_MAG_X,
		"gauss",  // IMU_MAG_Y,
		"gauss",  // IMU_MAG_Z,
		"Pa",  // IMU_PRESSURE,
		"m",  // IMU_ALTITUDE,
		"deg.",	 // IMU_TEMPERATURE,
		"qx",  // IMU_ORI_QUAT_X,
		"qy",  // IMU_ORI_QUAT_Y,
		"qz",  // IMU_ORI_QUAT_Z,
		"qw",  // IMU_ORI_QUAT_W,
		"rad/s",  //	IMU_YAW_VEL_GLOBAL
		"rad/s",  //	IMU_PITCH_VEL_GLOBAL
		"rad/s",  //	IMU_ROLL_VEL_GLOBAL
		"m/s^2",  //	IMU_X_ACC_GLOBAL
		"m/s^2",  //	IMU_Y_ACC_GLOBAL
		"m/s^2"	 //	IMU_Z_ACC_GLOBAL
	};

#define DUMP_IMU_DATA(x)                                                       \
	o << format("%20s = ", #x);                                                \
	if (dataIsPresent[x])                                                      \
		o << format("%10f %s\n", rawMeasurements[x], imu_units[x]);            \
	else                                                                       \
		o << "(not present)\n";

	DUMP_IMU_DATA(IMU_X_ACC)
	DUMP_IMU_DATA(IMU_Y_ACC)
	DUMP_IMU_DATA(IMU_Z_ACC)
	DUMP_IMU_DATA(IMU_WX)
	DUMP_IMU_DATA(IMU_WY)
	DUMP_IMU_DATA(IMU_WZ)
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

std::string CObservationIMU::exportTxtHeader() const
{
	return mrpt::format(
		"%16s %16s %16s "  // IMU_{X,Y,Z}_ACC
		"%16s %16s %16s "  // IMU_YAW_VEL...
		"%16s %16s %16s "  // IMU_X_VEL...
		"%16s %16s %16s "  // IMU_YAW...
		"%16s %16s %16s "  // IMU_X...
		"%16s %16s %16s "  // MAG_X MAG_Y MAG_Z
		"%16s %16s %16s "  // PRESS ALTIT TEMP
		"%16s %16s %16s %16s "	// ORI_QUAT
		"%16s %16s %16s "  // YAW_VEL_GLOBAL
		"%16s %16s %16s "  // X Y Z ACC GLOBAL
		,
		"IMU_X_ACC", "IMU_Y_ACC", "IMU_Z_ACC", "IMU_WZ", "IMU_WY", "IMU_WX",
		"IMU_X_VEL", "IMU_Y_VEL", "IMU_Z_VEL", "IMU_YAW", "IMU_PITCH",
		"IMU_ROLL", "IMU_X", "IMU_Y", "IMU_Z", "MAG_X", "MAG_Y", "MAG_Z",
		"PRESS", "ALTITUDE", "TEMPERATURE", "ORI_QUAT_X", "ORI_QUAT_Y",
		"ORI_QUAT_Z", "ORI_QUAT_W", "YAW_VEL_GLOBAL", "PITCH_VEL_GLOBAL",
		"ROLL_VEL_GLOBAL", "X_ACC_GLOBAL", "Y_ACC_GLOBAL", "Z_ACC_GLOBAL");
}
std::string CObservationIMU::exportTxtDataRow() const
{
	std::string s;
	for (size_t idx = 0; idx < rawMeasurements.size(); idx++)
		s += mrpt::format(
			"%16.8f ", dataIsPresent[idx] ? rawMeasurements[idx] : 0);
	return s;
}
