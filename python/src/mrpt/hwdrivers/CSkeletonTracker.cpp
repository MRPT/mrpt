#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CSkeletonTracker.h>
#include <mrpt/hwdrivers/CTaoboticsIMU.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <utility>
#include <vector>

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

// mrpt::hwdrivers::CSkeletonTracker file:mrpt/hwdrivers/CSkeletonTracker.h line:47
struct PyCallBack_mrpt_hwdrivers_CSkeletonTracker : public mrpt::hwdrivers::CSkeletonTracker {
	using mrpt::hwdrivers::CSkeletonTracker::CSkeletonTracker;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CSkeletonTracker::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkeletonTracker::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkeletonTracker::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkeletonTracker::initialize();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "loadConfig");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::loadConfig(a0, a1);
	}
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> >;
	_binder_ret_0 getObservations() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "getObservations");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CGenericSensor::getObservations();
	}
	void setPathForExternalImages(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "setPathForExternalImages");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setPathForExternalImages(a0);
	}
	void setExternalImageFormat(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "setExternalImageFormat");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setExternalImageFormat(a0);
	}
	void setExternalImageJPEGQuality(const unsigned int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "setExternalImageJPEGQuality");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setExternalImageJPEGQuality(a0);
	}
	unsigned int getExternalImageJPEGQuality() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "getExternalImageJPEGQuality");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned int>::value) {
				static pybind11::detail::override_caster_t<unsigned int> caster;
				return pybind11::detail::cast_ref<unsigned int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned int>(std::move(o));
		}
		return CGenericSensor::getExternalImageJPEGQuality();
	}
};

// mrpt::hwdrivers::CTaoboticsIMU file:mrpt/hwdrivers/CTaoboticsIMU.h line:85
struct PyCallBack_mrpt_hwdrivers_CTaoboticsIMU : public mrpt::hwdrivers::CTaoboticsIMU {
	using mrpt::hwdrivers::CTaoboticsIMU::CTaoboticsIMU;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CTaoboticsIMU::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTaoboticsIMU::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTaoboticsIMU::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTaoboticsIMU::initialize();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "loadConfig");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::loadConfig(a0, a1);
	}
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> >;
	_binder_ret_0 getObservations() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "getObservations");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CGenericSensor::getObservations();
	}
	void setPathForExternalImages(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "setPathForExternalImages");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setPathForExternalImages(a0);
	}
	void setExternalImageFormat(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "setExternalImageFormat");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setExternalImageFormat(a0);
	}
	void setExternalImageJPEGQuality(const unsigned int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "setExternalImageJPEGQuality");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setExternalImageJPEGQuality(a0);
	}
	unsigned int getExternalImageJPEGQuality() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "getExternalImageJPEGQuality");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned int>::value) {
				static pybind11::detail::override_caster_t<unsigned int> caster;
				return pybind11::detail::cast_ref<unsigned int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned int>(std::move(o));
		}
		return CGenericSensor::getExternalImageJPEGQuality();
	}
};

void bind_mrpt_hwdrivers_CSkeletonTracker(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CSkeletonTracker file:mrpt/hwdrivers/CSkeletonTracker.h line:47
		pybind11::class_<mrpt::hwdrivers::CSkeletonTracker, std::shared_ptr<mrpt::hwdrivers::CSkeletonTracker>, PyCallBack_mrpt_hwdrivers_CSkeletonTracker, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CSkeletonTracker", "A class for grabbing mrpt::obs::CObservationSkeleton from a PrimeSense\ncamera.\n  It connects to a PrimeSense camera and tries to detect users while\nrecording the positions of their skeletons' joints along time.\n\n  See also the application \"rawlog-grabber\" for a ready-to-use application to\ngather data from this sensor.\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n\n  ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CSkeletonTracker(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CSkeletonTracker(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CSkeletonTracker::*)() const) &mrpt::hwdrivers::CSkeletonTracker::GetRuntimeClass, "C++: mrpt::hwdrivers::CSkeletonTracker::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CSkeletonTracker::CreateObject, "C++: mrpt::hwdrivers::CSkeletonTracker::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CSkeletonTracker::doRegister, "C++: mrpt::hwdrivers::CSkeletonTracker::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CSkeletonTracker::*)()) &mrpt::hwdrivers::CSkeletonTracker::doProcess, "This method will be invoked at a minimum rate of \"process_rate\" (Hz)\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CSkeletonTracker::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CSkeletonTracker::*)()) &mrpt::hwdrivers::CSkeletonTracker::initialize, "Connects to the PrimeSense camera and prepares it to get skeleton data\n\nC++: mrpt::hwdrivers::CSkeletonTracker::initialize() --> void");
		cl.def("setPreview", [](mrpt::hwdrivers::CSkeletonTracker &o) -> void { return o.setPreview(); }, "");
		cl.def("setPreview", (void (mrpt::hwdrivers::CSkeletonTracker::*)(const bool)) &mrpt::hwdrivers::CSkeletonTracker::setPreview, "Set/unset preview \n\nC++: mrpt::hwdrivers::CSkeletonTracker::setPreview(const bool) --> void", pybind11::arg("setPreview"));
	}
	{ // mrpt::hwdrivers::CTaoboticsIMU file:mrpt/hwdrivers/CTaoboticsIMU.h line:85
		pybind11::class_<mrpt::hwdrivers::CTaoboticsIMU, std::shared_ptr<mrpt::hwdrivers::CTaoboticsIMU>, PyCallBack_mrpt_hwdrivers_CTaoboticsIMU, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CTaoboticsIMU", "A driver for Taobotics IMU.\n\n Supported models: HFI-A9, HFI-B9, HFI-B6.\n\n ## Frame format\n From analysis the provided [python scripts]()\n we found that the sensor uses these data frames, documented here for\n reference:\n\n ### Model: \"hfi-a9\"\n\n  Two frames:\n  Frame field :  0xAA 0x55 0x2c | (9 * 4[float]) D0-D8  | ?? ??\n  Byte index  :   0     1    2  | 3 4 5 6 |  ...        | 48\n   Data:\n    - D{0,1,2}: wx,wy,wz\n    - D{3,4,5}: accx,accy,accz\n    - D{6,7,8}: magx,magy,magz\n\n  Frame field :  0xAA 0x55 0x14 | (5 * 4[float]) D0-D4   | ??\n  Byte index  :   0     1    2  |  3 ... |      ...      | 24\n\n ### Model: \"hfi-b6\"\n\n  Frame field :  0x55 | TYPE |  D0  |  D1  |  D2  |  D3  | CHKSUM\n  Byte index  :   0   |  1   | 2  3 | 4  5 | 6  7 | 8  9 |   10\n  Type:\n  0x51: acceleration, 0x52: angular velocity, 0x53: euler angles\n\n ### Configuration file block\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n If used programmatically, this class will be used as:\n\n \n\n\n\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CTaoboticsIMU(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CTaoboticsIMU(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CTaoboticsIMU::*)() const) &mrpt::hwdrivers::CTaoboticsIMU::GetRuntimeClass, "C++: mrpt::hwdrivers::CTaoboticsIMU::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CTaoboticsIMU::CreateObject, "C++: mrpt::hwdrivers::CTaoboticsIMU::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CTaoboticsIMU::doRegister, "C++: mrpt::hwdrivers::CTaoboticsIMU::doRegister() --> void");
		cl.def("loadConfig_sensorSpecific", (void (mrpt::hwdrivers::CTaoboticsIMU::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::hwdrivers::CTaoboticsIMU::loadConfig_sensorSpecific, "See the class documentation at the top for expected parameters \n\nC++: mrpt::hwdrivers::CTaoboticsIMU::loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("iniSection"));
		cl.def("doProcess", (void (mrpt::hwdrivers::CTaoboticsIMU::*)()) &mrpt::hwdrivers::CTaoboticsIMU::doProcess, "This method will be invoked at a minimum rate of \"process_rate\" (Hz)\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CTaoboticsIMU::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CTaoboticsIMU::*)()) &mrpt::hwdrivers::CTaoboticsIMU::initialize, "Opens the serial port and start streaming data.\n You must have called loadConfig_sensorSpecific before\n calling this function, or set the configuration via the provided methods,\n e.g. setSerialPort(), etc.\n\nC++: mrpt::hwdrivers::CTaoboticsIMU::initialize() --> void");
		cl.def("setSerialPort", (void (mrpt::hwdrivers::CTaoboticsIMU::*)(const std::string &)) &mrpt::hwdrivers::CTaoboticsIMU::setSerialPort, "Must be called before initialize(). If not set, the default is\n \"/dev/ttyUSB0\". Use \"COM1\", etc. for Windows.\n\nC++: mrpt::hwdrivers::CTaoboticsIMU::setSerialPort(const std::string &) --> void", pybind11::arg("serialPort"));
		cl.def("setSerialBaudRate", (void (mrpt::hwdrivers::CTaoboticsIMU::*)(int)) &mrpt::hwdrivers::CTaoboticsIMU::setSerialBaudRate, "Must be called before initialize(). If not called, the default 921600 is\n used.\n\nC++: mrpt::hwdrivers::CTaoboticsIMU::setSerialBaudRate(int) --> void", pybind11::arg("rate"));
	}
}
