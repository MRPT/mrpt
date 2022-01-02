/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/io/CStream.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/serialization/CArchive.h>

/* STD */
#include <cstdint>

using namespace boost::python;
using namespace mrpt::io;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::serialization;

// CActionCollection
void CActionCollection_insert1(
	CActionCollection& me, CActionRobotMovement2D& action)
{
	me.insert(action);
}

void CActionCollection_insert2(
	CActionCollection& me, CActionRobotMovement3D& action)
{
	me.insert(action);
}
// end of CActionCollection

// CSensoryFrame
const CPointsMap* CSensoryFrame_getAuxPointsMap(CSensoryFrame& me)
{
	return me.getAuxPointsMap<CPointsMap>();
}

const CPointsMap* CSensoryFrame_buildAuxPointsMap(CSensoryFrame& me)
{
	return me.buildAuxPointsMap<CPointsMap>();
}
// end of CSensoryFrame

// CObservation
long_ CObservation_get_timestamp(CObservation& me)
{
	return long_(me.timestamp);
}

void CObservation_set_timestamp(CObservation& me, long_ timestamp)
{
	auto t = mrpt::Clock::fromDouble(extract<uint64_t>(timestamp));
	me.timestamp = t;
}

CPose3D CObservation_getSensorPose1(CObservation& me)
{
	CPose3D sensorPose;
	me.getSensorPose(sensorPose);
	return sensorPose;
}

void CObservation_getSensorPose2(CObservation& me, CPose3D& sensorPose)
{
	me.getSensorPose(sensorPose);
}

void CObservation_getSensorPose3(CObservation& me, TPose3D& sensorPose)
{
	me.getSensorPose(sensorPose);
}

void CObservation_setSensorPose1(CObservation& me, CPose3D& sensorPose)
{
	me.setSensorPose(sensorPose);
}

void CObservation_setSensorPose2(CObservation& me, TPose3D& sensorPose)
{
	me.setSensorPose(sensorPose);
}
// end of CObservation

// CObservationOdometry
#ifdef ROS_EXTENSIONS
object CObservationOdometry_to_ROS_RawOdometry_msg(
	CObservationOdometry& me, str frame_id)
{
	// import msg
	dict locals;
	exec(
		"from pymrpt.msg import RawOdometry\n"
		"raw_odometry_msg = RawOdometry()\n",
		object(), locals);
	object raw_odometry_msg = locals["raw_odometry_msg"];
	// set info
	raw_odometry_msg.attr("header").attr("frame_id") = frame_id;
	raw_odometry_msg.attr("header").attr("stamp") =
		TTimeStamp_to_ROS_Time(long_(me.timestamp));
	raw_odometry_msg.attr("has_encoders_info") = me.hasEncodersInfo;
	raw_odometry_msg.attr("has_velocities") = me.hasVelocities;
	// set data
	raw_odometry_msg.attr("encoder_left_ticks") = me.encoderLeftTicks;
	raw_odometry_msg.attr("encoder_right_ticks") = me.encoderRightTicks;
	raw_odometry_msg.attr("velocity_lin") = me.velocityLocal.vx;
	raw_odometry_msg.attr("velocity_ang") = me.velocityLocal.omega;
	return raw_odometry_msg;
}

void CObservationOdometry_from_ROS_RawOdometry_msg(
	CObservationOdometry& me, object raw_odometry_msg)
{
	// set info
	me.sensorLabel =
		extract<std::string>(raw_odometry_msg.attr("header").attr("frame_id"));
	auto t = mrpt::Clock::fromDouble(extract<uint64_t>(TTimeStamp_from_ROS_Time(
		raw_odometry_msg.attr("header").attr("stamp"))));
	me.timestamp = t;
	me.hasEncodersInfo =
		extract<bool>(raw_odometry_msg.attr("has_encoders_info"));
	me.hasVelocities = extract<bool>(raw_odometry_msg.attr("has_velocities"));
	// set data
	me.encoderLeftTicks =
		extract<int>(raw_odometry_msg.attr("encoder_left_ticks"));
	me.encoderRightTicks =
		extract<int>(raw_odometry_msg.attr("encoder_right_ticks"));
	me.velocityLocal.vx = extract<float>(raw_odometry_msg.attr("velocity_lin"));
	me.velocityLocal.vy = 0;
	me.velocityLocal.omega =
		extract<float>(raw_odometry_msg.attr("velocity_ang"));
}
#endif
// end of CObservationOdometry

// CObservationRange
#ifdef ROS_EXTENSIONS
object CObservationRange_to_ROS_Range_msg(CObservationRange& me, str frame_id)
{
	// import msg
	dict locals;
	exec(
		"from sensor_msgs.msg import Range\n"
		"range_msg = Range()\n",
		object(), locals);
	object range_msg = locals["range_msg"];
	// set info
	range_msg.attr("header").attr("frame_id") = frame_id;
	range_msg.attr("header").attr("stamp") =
		TTimeStamp_to_ROS_Time(long_(me.timestamp));
	range_msg.attr("min_range") = me.minSensorDistance;
	range_msg.attr("max_range") = me.maxSensorDistance;
	range_msg.attr("field_of_view") = me.sensorConeApperture;
	// set range
	range_msg.attr("range") = me.sensedData[0].sensedDistance;
	return range_msg;
}

void CObservationRange_from_ROS_Range_msg(
	CObservationRange& me, object range_msg)
{
	// set info
	me.sensorLabel =
		extract<std::string>(range_msg.attr("header").attr("frame_id"));
	auto t = mrpt::Clock::fromDouble(extract<uint64_t>(
		TTimeStamp_from_ROS_Time(range_msg.attr("header").attr("stamp"))));
	me.timestamp = t;
	me.minSensorDistance = extract<float>(range_msg.attr("min_range"));
	me.maxSensorDistance = extract<float>(range_msg.attr("max_range"));
	me.sensorConeApperture = extract<float>(range_msg.attr("field_of_view"));
	// set range
	CObservationRange::TMeasurement data;
	data.sensedDistance = extract<float>(range_msg.attr("range"));
	me.sensedData.clear();
	me.sensedData.push_back(data);
}
#endif
// end of CObservationRange

// CObservation2DRangeScan
#ifdef ROS_EXTENSIONS
object CObservation2DRangeScan_to_ROS_LaserScan_msg(
	CObservation2DRangeScan& me, str frame_id)
{
	// import msg
	dict locals;
	exec(
		"from sensor_msgs.msg import LaserScan\n"
		"scan_msg = LaserScan()\n",
		object(), locals);
	object scan_msg = locals["scan_msg"];
	// set info
	scan_msg.attr("header").attr("frame_id") = frame_id;
	scan_msg.attr("header").attr("stamp") =
		TTimeStamp_to_ROS_Time(long_(me.timestamp));
	scan_msg.attr("range_min") = 0.0;
	scan_msg.attr("range_max") = me.maxRange;
	scan_msg.attr("angle_min") = -me.aperture / 2.0;
	scan_msg.attr("angle_max") = me.aperture / 2.0;
	scan_msg.attr("angle_increment") = me.beamAperture;
	// set ranges (no intensities given in mrpt)
	list ranges;
	for (size_t i = 0; i < me.getScanSize(); i++)
		ranges.append(me.getScanRange(i));
	scan_msg.attr("ranges") = ranges;
	return scan_msg;
}

void CObservation2DRangeScan_from_ROS_LaserScan_msg(
	CObservation2DRangeScan& me, object scan_msg, CPose3D pose)
{
	// set info
	me.sensorLabel =
		extract<std::string>(scan_msg.attr("header").attr("frame_id"));
	auto t = mrpt::Clock::fromDouble(extract<uint64_t>(
		TTimeStamp_from_ROS_Time(scan_msg.attr("header").attr("stamp"))));
	me.timestamp = t;
	me.maxRange = extract<float>(scan_msg.attr("range_max"));
	me.aperture = extract<float>(scan_msg.attr("angle_max")) -
		extract<float>(scan_msg.attr("angle_min"));
	me.beamAperture = extract<float>(scan_msg.attr("angle_increment"));
	me.sensorPose = pose;
	// set ranges
	tuple ranges = extract<tuple>(scan_msg.attr("ranges"));
	const size_t N = len(ranges);
	me.resizeScan(N);
	for (int i = 0; i < len(ranges); ++i)
	{
		float range = extract<float>(ranges[i]);
		me.setScanRange(i, range);
		me.setScanRangeValidity(i, (range < me.maxRange - 0.01));
	}
}
#endif
// end of CObservation2DRangeScan

// CRawLog
tuple CRawlog_readActionObservationPair(CStream& inStream, size_t rawlogEntry)
{
	list ret_val;
	CActionCollection::Ptr action;
	CSensoryFrame::Ptr observations;

	auto arch = mrpt::serialization::archiveFrom(inStream);
	bool is_next = CRawlog::readActionObservationPair(
		arch, action, observations, rawlogEntry);

	ret_val.append(is_next);
	try
	{
		ret_val.append(*action);
	}
	catch (...)
	{
		ret_val.append(object());
		is_next = false;
	}
	try
	{
		ret_val.append(*observations);
	}
	catch (...)
	{
		ret_val.append(object());
		is_next = false;
	}
	ret_val.append(rawlogEntry);

	return tuple(ret_val);
}
// end of CRawLog

// smart pointer contents
MAKE_PTR_CTX(CAction)
MAKE_PTR_CTX(CActionCollection)
MAKE_PTR_CTX(CActionRobotMovement2D)
MAKE_PTR_CTX(CObservation)
MAKE_PTR_CTX(CObservationOdometry)
MAKE_PTR_CTX(CObservationRange)
MAKE_PTR_CTX(CObservation2DRangeScan)
MAKE_PTR_CTX(CObservationBearingRange)
MAKE_PTR_CTX(CSensoryFrame)

// exporter
void export_obs()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(obs)

	// CAction
	{
		MAKE_PTR(CAction)

		class_<CAction, boost::noncopyable, bases<CSerializable>>(
			"CAction", no_init)
			.def_readwrite("timestamp", &CAction::timestamp);
	}

	// CActionCollection
	{
		class_<CActionCollection>("CActionCollection", init<>())
			.def(
				"clear", &CActionCollection::clear,
				"Erase all actions from the list.")
			.def(
				"insert", &CActionCollection::insert,
				"Add a new object to the list.")
			.def(
				"size", &CActionCollection::size,
				"Returns the actions count in the collection.")
			.def(
				"eraseByIndex", &CActionCollection::eraseByIndex,
				"Remove an action from the list by its index.")
				MAKE_CREATE(CActionCollection);
	}

	// CActionRobotMovement2D
	{
		MAKE_PTR(CActionRobotMovement2D)

		scope s =
			class_<CActionRobotMovement2D, bases<CAction>>(
				"CActionRobotMovement2D", init<>())
				.def_readwrite(
					"rawOdometryIncrementReading",
					&CActionRobotMovement2D::rawOdometryIncrementReading)
				.def_readwrite(
					"estimationMethod",
					&CActionRobotMovement2D::estimationMethod)
				.def_readwrite(
					"hasEncodersInfo", &CActionRobotMovement2D::hasEncodersInfo)
				.def_readwrite(
					"encoderLeftTicks",
					&CActionRobotMovement2D::encoderLeftTicks)
				.def_readwrite(
					"encoderRightTicks",
					&CActionRobotMovement2D::encoderRightTicks)
				.def_readwrite(
					"hasVelocities", &CActionRobotMovement2D::hasVelocities)
				.def_readwrite(
					"velocityLin", &CActionRobotMovement2D::velocityLin)
				.def_readwrite(
					"velocityAng", &CActionRobotMovement2D::velocityAng)
				.def_readwrite(
					"motionModelConfiguration",
					&CActionRobotMovement2D::motionModelConfiguration)
				.def(
					"computeFromOdometry",
					&CActionRobotMovement2D::computeFromOdometry,
					"Computes the PDF of the pose increment from an odometry "
					"reading and according to the given motion model (speed "
					"and encoder ticks information is not modified).")
				.def(
					"computeFromEncoders",
					&CActionRobotMovement2D::computeFromEncoders,
					"If \"hasEncodersInfo\"=true, this method updates the pose "
					"estimation according to the ticks from both encoders and "
					"the passed parameters, which is passed internally to the "
					"method \"computeFromOdometry\" with the last used PDF "
					"options (or the defualt ones if not explicitly called by "
					"the user).") MAKE_CREATE(CActionRobotMovement2D);

		// TEstimationMethod
		enum_<CActionRobotMovement2D::TEstimationMethod>("TEstimationMethod")
			.value("emOdometry", CActionRobotMovement2D::emOdometry)
			.value(
				"emScan2DMatching", CActionRobotMovement2D::emScan2DMatching);

		// TDrawSampleMotionModel
		enum_<CActionRobotMovement2D::TDrawSampleMotionModel>(
			"TDrawSampleMotionModel")
			.value("mmGaussian", CActionRobotMovement2D::mmGaussian)
			.value("mmThrun", CActionRobotMovement2D::mmThrun);

		{
			// TMotionModelOptions
			scope s2 = class_<CActionRobotMovement2D::TMotionModelOptions>(
						   "TMotionModelOptions", init<>())
						   .def_readwrite(
							   "modelSelection",
							   &CActionRobotMovement2D::TMotionModelOptions::
								   modelSelection)
						   .def_readwrite(
							   "gaussianModel",
							   &CActionRobotMovement2D::TMotionModelOptions::
								   gaussianModel)
						   .def_readwrite(
							   "thrunModel",
							   &CActionRobotMovement2D::TMotionModelOptions::
								   thrunModel);

			// TOptions_GaussianModel
			class_<CActionRobotMovement2D::TMotionModelOptions::
					   TOptions_GaussianModel>(
				"TOptions_GaussianModel", init<>())
				.def_readwrite(
					"a1",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_GaussianModel::a1)
				.def_readwrite(
					"a2",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_GaussianModel::a2)
				.def_readwrite(
					"a3",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_GaussianModel::a3)
				.def_readwrite(
					"a4",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_GaussianModel::a4)
				.def_readwrite(
					"minStdXY",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_GaussianModel::minStdXY)
				.def_readwrite(
					"minStdPHI",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_GaussianModel::minStdPHI);

			// TOptions_ThrunModel
			class_<CActionRobotMovement2D::TMotionModelOptions::
					   TOptions_ThrunModel>("TOptions_ThrunModel", init<>())
				.def_readwrite(
					"nParticlesCount",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_ThrunModel::nParticlesCount)
				.def_readwrite(
					"alfa1_rot_rot",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_ThrunModel::alfa1_rot_rot)
				.def_readwrite(
					"alfa2_rot_trans",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_ThrunModel::alfa2_rot_trans)
				.def_readwrite(
					"alfa3_trans_trans",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_ThrunModel::alfa3_trans_trans)
				.def_readwrite(
					"alfa4_trans_rot",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_ThrunModel::alfa4_trans_rot)
				.def_readwrite(
					"additional_std_XY",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_ThrunModel::additional_std_XY)
				.def_readwrite(
					"additional_std_phi",
					&CActionRobotMovement2D::TMotionModelOptions::
						TOptions_ThrunModel::additional_std_phi);
		}
	}

	// CObservation
	{
		MAKE_PTR(CObservation)

		class_<CObservation, boost::noncopyable, bases<CSerializable>>(
			"CObservation", no_init)
			.add_property(
				"timestamp", &CObservation_get_timestamp,
				&CObservation_set_timestamp)
			.def_readwrite("sensorLabel", &CObservation::sensorLabel)
			.def(
				"getSensorPose", &CObservation_getSensorPose1,
				"A general method to retrieve the sensor pose on the robot.")
			.def(
				"getSensorPose", &CObservation_getSensorPose2,
				"A general method to retrieve the sensor pose on the robot.")
			.def(
				"getSensorPose", &CObservation_getSensorPose3,
				"A general method to retrieve the sensor pose on the robot.")
			.def(
				"setSensorPose", &CObservation_setSensorPose1,
				"A general method to change the sensor pose on the robot.")
			.def(
				"setSensorPose", &CObservation_setSensorPose2,
				"A general method to change the sensor pose on the robot.");
	}

	// CObservationOdometry
	{
		MAKE_PTR(CObservationOdometry)

		scope s =
			class_<CObservationOdometry, bases<CObservation>>(
				"CObservationOdometry", init<>())
				.def_readwrite("odometry", &CObservationOdometry::odometry)
				.def_readwrite(
					"hasEncodersInfo", &CObservationOdometry::hasEncodersInfo)
				.def_readwrite(
					"encoderLeftTicks", &CObservationOdometry::encoderLeftTicks)
				.def_readwrite(
					"encoderRightTicks",
					&CObservationOdometry::encoderRightTicks)
				.def_readwrite(
					"hasVelocities", &CObservationOdometry::hasVelocities)
				.def_readwrite(
					"velocityLocal", &CObservationOdometry::velocityLocal)
					MAKE_CREATE(CObservationOdometry)
#ifdef ROS_EXTENSIONS
				.def(
					"to_ROS_RawOdometry_msg",
					&CObservationOdometry_to_ROS_RawOdometry_msg,
					"Convert to ROS pymrpt_msgs/RawOdometry.")
				.def(
					"from_ROS_RawOdometry_msg",
					&CObservationOdometry_from_ROS_RawOdometry_msg,
					"Convert from ROS pymrpt_msgs/RawOdometry.")
#endif
			;
	}
	// CObservationRange
	{
		MAKE_PTR(CObservationRange)

		scope s =
			class_<CObservationRange, bases<CObservation>>(
				"CObservationRange", init<>())
				.def_readwrite(
					"minSensorDistance", &CObservationRange::minSensorDistance)
				.def_readwrite(
					"maxSensorDistance", &CObservationRange::maxSensorDistance)
				.def_readwrite(
					"sensorConeApperture",
					&CObservationRange::sensorConeApperture)
				.def_readwrite("sensedData", &CObservationRange::sensedData)
					MAKE_CREATE(CObservationRange)
#ifdef ROS_EXTENSIONS
				.def(
					"to_ROS_Range_msg", &CObservationRange_to_ROS_Range_msg,
					"Convert to ROS sensor_msgs/Range.")
				.def(
					"from_ROS_Range_msg", &CObservationRange_from_ROS_Range_msg,
					"Convert from ROS sensor_msgs/Range.")
#endif
			;

		// TMeasurement
		class_<CObservationRange::TMeasurement>("TMeasurement", init<>())
			.def_readwrite(
				"sensorID", &CObservationRange::TMeasurement::sensorID)
			.def_readwrite(
				"sensorPose", &CObservationRange::TMeasurement::sensorPose)
			.def_readwrite(
				"sensedDistance",
				&CObservationRange::TMeasurement::sensedDistance);

// TMeasurementList
#if 0
		class_<CObservationRange::TMeasurementList>(
			"TMeasurementList", init<>())
			.def("__len__", &CObservationRange::TMeasurementList::size)
			.def("clear", &CObservationRange::TMeasurementList::clear)
			.def(
				"append",
				&StlListLike<CObservationRange::TMeasurementList>::add,
				with_custodian_and_ward<1, 2>())  // to let container keep value
			.def(
				"__getitem__",
				&StlListLike<CObservationRange::TMeasurementList>::get,
				return_value_policy<copy_non_const_reference>())
			.def(
				"__setitem__",
				&StlListLike<CObservationRange::TMeasurementList>::set,
				with_custodian_and_ward<1, 2>())  // to let container keep value
			.def(
				"__delitem__",
				&StlListLike<CObservationRange::TMeasurementList>::del);
#endif
	}

	// CObservation2DRangeScan
	{
		MAKE_PTR(CObservation2DRangeScan)

		class_<CObservation2DRangeScan, bases<CObservation>>(
			"CObservation2DRangeScan", init<>())
			.def_readwrite("aperture", &CObservation2DRangeScan::aperture)
			.def_readwrite("sensorPose", &CObservation2DRangeScan::sensorPose)
			.def_readwrite("maxRange", &CObservation2DRangeScan::maxRange)
			.def_readwrite("stdError", &CObservation2DRangeScan::stdError)
			.def_readwrite(
				"beamAperture", &CObservation2DRangeScan::beamAperture)
			.def_readwrite("deltaPitch", &CObservation2DRangeScan::deltaPitch)
				MAKE_CREATE(CObservation2DRangeScan)
#ifdef ROS_EXTENSIONS
			.def(
				"to_ROS_LaserScan_msg",
				&CObservation2DRangeScan_to_ROS_LaserScan_msg,
				"Convert to ROS sensor_msgs/LaserScan.")
			.def(
				"from_ROS_LaserScan_msg",
				&CObservation2DRangeScan_from_ROS_LaserScan_msg,
				"Convert to ROS sensor_msgs/LaserScan.")
#endif
			;
	}

	// CObservationBearingRange
	{
		MAKE_PTR(CObservationBearingRange)

		scope s =
			class_<CObservationBearingRange, bases<CObservation>>(
				"CObservationBearingRange", init<>())
				.def_readwrite(
					"minSensorDistance",
					&CObservationBearingRange::minSensorDistance)
				.def_readwrite(
					"maxSensorDistance",
					&CObservationBearingRange::maxSensorDistance)
				.def_readwrite(
					"fieldOfView_yaw",
					&CObservationBearingRange::fieldOfView_yaw)
				.def_readwrite(
					"fieldOfView_pitch",
					&CObservationBearingRange::fieldOfView_pitch)
				.def_readwrite(
					"sensorLocationOnRobot",
					&CObservationBearingRange::sensorLocationOnRobot)
				.def_readwrite(
					"sensedData", &CObservationBearingRange::sensedData)
				.def_readwrite(
					"validCovariances",
					&CObservationBearingRange::validCovariances)
				.def_readwrite(
					"sensor_std_range",
					&CObservationBearingRange::sensor_std_range)
				.def_readwrite(
					"sensor_std_yaw", &CObservationBearingRange::sensor_std_yaw)
				.def_readwrite(
					"sensor_std_pitch",
					&CObservationBearingRange::sensor_std_pitch)
					MAKE_CREATE(CObservationBearingRange);

		// TMeasurement
		class_<CObservationBearingRange::TMeasurement>("TMeasurement", init<>())
			.def_readwrite(
				"range", &CObservationBearingRange::TMeasurement::range)
			.def_readwrite("yaw", &CObservationBearingRange::TMeasurement::yaw)
			.def_readwrite(
				"pitch", &CObservationBearingRange::TMeasurement::pitch)
			.def_readwrite(
				"landmarkID",
				&CObservationBearingRange::TMeasurement::landmarkID)
			.def_readwrite(
				"covariance",
				&CObservationBearingRange::TMeasurement::covariance);

// TMeasurementList
#if 0
		class_<CObservationBearingRange::TMeasurementList>(
			"TMeasurementList", init<>())
			.def("__len__", &CObservationBearingRange::TMeasurementList::size)
			.def("clear", &CObservationBearingRange::TMeasurementList::clear)
			.def(
				"append",
				&StlListLike<CObservationBearingRange::TMeasurementList>::add,
				with_custodian_and_ward<1, 2>())  // to let container keep value
			.def(
				"__getitem__",
				&StlListLike<CObservationBearingRange::TMeasurementList>::get,
				return_value_policy<copy_non_const_reference>())
			.def(
				"__setitem__",
				&StlListLike<CObservationBearingRange::TMeasurementList>::set,
				with_custodian_and_ward<1, 2>())  // to let container keep value
			.def(
				"__delitem__",
				&StlListLike<CObservationBearingRange::TMeasurementList>::del);
#endif
	}

	// CRawlog
	{
		scope s =
			class_<CRawlog>("CRawlog", init<>())
				.def(
					"clear", &CRawlog::clear,
					"Clear the sequence of actions/observations, freeing the "
					"memory of all the objects in the list.")
				.def(
					"size", &CRawlog::size,
					"Returns the number of actions / observations object in "
					"the sequence.")
				.def(
					"insert",
					static_cast<void (CRawlog::*)(const CSerializable::Ptr&)>(
						&CRawlog::insert),
					"Add an object to the sequence.")
				.def(
					"readActionObservationPair",
					&CRawlog_readActionObservationPair,
					"Reads a consecutive pair action / observation from the "
					"rawlog opened at some input stream.")
				.staticmethod("readActionObservationPair")
				.def(
					"loadFromRawLogFile", &CRawlog::loadFromRawLogFile,
					"Load the contents from a file containing either CRawLog "
					"objects or directly Action/Observation object pairs.")
				.def(
					"saveToRawLogFile", &CRawlog::saveToRawLogFile,
					"Saves the contents to a rawlog-file, compatible with "
					"RawlogViewer (As the sequence of internal objects).");
	}

	// CSensoryFrame
	{
		MAKE_PTR(CSensoryFrame)

		class_<CSensoryFrame>("CSensoryFrame", init<>())
			.def(
				"clear", &CSensoryFrame::clear,
				"Clear all current observations.")
			.def(
				"insert", &CSensoryFrame::insert,
				"Inserts a new observation to the list.")
			.def(
				"size", &CSensoryFrame::size,
				"Returns the number of observations in the list")
			.def(
				"eraseByIndex", &CSensoryFrame::eraseByIndex,
				"Removes the i'th observation in the list (0=first).")
			.def(
				"getAuxPointsMap", &CSensoryFrame_getAuxPointsMap,
				return_internal_reference<>(),
				"Returns the cached points map representation of the scan, if "
				"already build with buildAuxPointsMap(), or nullptr otherwise.")
			.def(
				"buildAuxPointsMap", &CSensoryFrame_buildAuxPointsMap,
				return_internal_reference<>(),
				"Returns a cached points map representing this laser scan, "
				"building it upon the first call.") MAKE_CREATE(CSensoryFrame);
	}
}
