#include <mrpt/obs/T3DPointsProjectionParams.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <sstream> // __str__

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_obs_T3DPointsProjectionParams(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::T3DPointsProjectionParams file:mrpt/obs/T3DPointsProjectionParams.h line:21
		pybind11::class_<mrpt::obs::T3DPointsProjectionParams, std::shared_ptr<mrpt::obs::T3DPointsProjectionParams>> cl(M("mrpt::obs"), "T3DPointsProjectionParams", "Used in CObservation3DRangeScan::unprojectInto()\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::T3DPointsProjectionParams(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::T3DPointsProjectionParams const &o){ return new mrpt::obs::T3DPointsProjectionParams(o); } ) );
		cl.def_readwrite("takeIntoAccountSensorPoseOnRobot", &mrpt::obs::T3DPointsProjectionParams::takeIntoAccountSensorPoseOnRobot);
		cl.def_readwrite("robotPoseInTheWorld", &mrpt::obs::T3DPointsProjectionParams::robotPoseInTheWorld);
		cl.def_readwrite("USE_SSE2", &mrpt::obs::T3DPointsProjectionParams::USE_SSE2);
		cl.def_readwrite("MAKE_ORGANIZED", &mrpt::obs::T3DPointsProjectionParams::MAKE_ORGANIZED);
		cl.def_readwrite("decimation", &mrpt::obs::T3DPointsProjectionParams::decimation);
		cl.def_readwrite("layer", &mrpt::obs::T3DPointsProjectionParams::layer);
		cl.def_readwrite("onlyPointsWithIntensityColor", &mrpt::obs::T3DPointsProjectionParams::onlyPointsWithIntensityColor);
		cl.def("assign", (struct mrpt::obs::T3DPointsProjectionParams & (mrpt::obs::T3DPointsProjectionParams::*)(const struct mrpt::obs::T3DPointsProjectionParams &)) &mrpt::obs::T3DPointsProjectionParams::operator=, "C++: mrpt::obs::T3DPointsProjectionParams::operator=(const struct mrpt::obs::T3DPointsProjectionParams &) --> struct mrpt::obs::T3DPointsProjectionParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::T3DPointsTo2DScanParams file:mrpt/obs/T3DPointsTo2DScanParams.h line:18
		pybind11::class_<mrpt::obs::T3DPointsTo2DScanParams, std::shared_ptr<mrpt::obs::T3DPointsTo2DScanParams>> cl(M("mrpt::obs"), "T3DPointsTo2DScanParams", "Used in CObservation3DRangeScan::convertTo2DScan()\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::T3DPointsTo2DScanParams(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::T3DPointsTo2DScanParams const &o){ return new mrpt::obs::T3DPointsTo2DScanParams(o); } ) );
		cl.def_readwrite("sensorLabel", &mrpt::obs::T3DPointsTo2DScanParams::sensorLabel);
		cl.def_readwrite("angle_sup", &mrpt::obs::T3DPointsTo2DScanParams::angle_sup);
		cl.def_readwrite("angle_inf", &mrpt::obs::T3DPointsTo2DScanParams::angle_inf);
		cl.def_readwrite("z_min", &mrpt::obs::T3DPointsTo2DScanParams::z_min);
		cl.def_readwrite("z_max", &mrpt::obs::T3DPointsTo2DScanParams::z_max);
		cl.def_readwrite("oversampling_ratio", &mrpt::obs::T3DPointsTo2DScanParams::oversampling_ratio);
		cl.def_readwrite("use_origin_sensor_pose", &mrpt::obs::T3DPointsTo2DScanParams::use_origin_sensor_pose);
		cl.def("assign", (struct mrpt::obs::T3DPointsTo2DScanParams & (mrpt::obs::T3DPointsTo2DScanParams::*)(const struct mrpt::obs::T3DPointsTo2DScanParams &)) &mrpt::obs::T3DPointsTo2DScanParams::operator=, "C++: mrpt::obs::T3DPointsTo2DScanParams::operator=(const struct mrpt::obs::T3DPointsTo2DScanParams &) --> struct mrpt::obs::T3DPointsTo2DScanParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
