#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CIMUXSens_MT4.h>
#include <mrpt/hwdrivers/CIbeoLuxETH.h>
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

// mrpt::hwdrivers::CIMUXSens_MT4 file:mrpt/hwdrivers/CIMUXSens_MT4.h line:55
struct PyCallBack_mrpt_hwdrivers_CIMUXSens_MT4 : public mrpt::hwdrivers::CIMUXSens_MT4 {
	using mrpt::hwdrivers::CIMUXSens_MT4::CIMUXSens_MT4;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CIMUXSens_MT4::GetRuntimeClass();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIMUXSens_MT4::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIMUXSens_MT4::initialize();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIMUXSens_MT4::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUXSens_MT4 *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CIbeoLuxETH file:mrpt/hwdrivers/CIbeoLuxETH.h line:42
struct PyCallBack_mrpt_hwdrivers_CIbeoLuxETH : public mrpt::hwdrivers::CIbeoLuxETH {
	using mrpt::hwdrivers::CIbeoLuxETH::CIbeoLuxETH;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CIbeoLuxETH::GetRuntimeClass();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIbeoLuxETH::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIbeoLuxETH::initialize();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIbeoLuxETH::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIbeoLuxETH *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CIMUXSens_MT4(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CIMUXSens_MT4 file:mrpt/hwdrivers/CIMUXSens_MT4.h line:55
		pybind11::class_<mrpt::hwdrivers::CIMUXSens_MT4, std::shared_ptr<mrpt::hwdrivers::CIMUXSens_MT4>, PyCallBack_mrpt_hwdrivers_CIMUXSens_MT4, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CIMUXSens_MT4", "A class for interfacing XSens 4th generation Inertial Measuring Units\n (IMUs): MTi 10-series, MTi 100-series.\n  Usage considerations:\n    - In Windows, you only need to install XSens drivers.\n    - In Linux, this class requires the system libraries: libusb-1.0 &\n libudev (dev packages). Accessing USB devices may require\n      running the program as super user (\"sudo\"). To avoid that, Or, install\n  MRPT/scripts/52-xsens.rules  in /etc/udev/rules.d/\n to allow access to all users.\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  \n Set the environment variable \"MRPT_HWDRIVERS_VERBOSE\" to \"1\" to\n enable diagnostic information while using this class.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CIMUXSens_MT4(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CIMUXSens_MT4(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CIMUXSens_MT4::*)() const) &mrpt::hwdrivers::CIMUXSens_MT4::GetRuntimeClass, "C++: mrpt::hwdrivers::CIMUXSens_MT4::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CIMUXSens_MT4::CreateObject, "C++: mrpt::hwdrivers::CIMUXSens_MT4::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CIMUXSens_MT4::doRegister, "C++: mrpt::hwdrivers::CIMUXSens_MT4::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CIMUXSens_MT4::*)()) &mrpt::hwdrivers::CIMUXSens_MT4::doProcess, "This method will be invoked at a minimum rate of \"process_rate\" (Hz)\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CIMUXSens_MT4::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CIMUXSens_MT4::*)()) &mrpt::hwdrivers::CIMUXSens_MT4::initialize, "Turns on the xSens device and configure it for getting orientation data\n\nC++: mrpt::hwdrivers::CIMUXSens_MT4::initialize() --> void");
		cl.def("close", (void (mrpt::hwdrivers::CIMUXSens_MT4::*)()) &mrpt::hwdrivers::CIMUXSens_MT4::close, "C++: mrpt::hwdrivers::CIMUXSens_MT4::close() --> void");
	}
	{ // mrpt::hwdrivers::CIbeoLuxETH file:mrpt/hwdrivers/CIbeoLuxETH.h line:42
		pybind11::class_<mrpt::hwdrivers::CIbeoLuxETH, std::shared_ptr<mrpt::hwdrivers::CIbeoLuxETH>, PyCallBack_mrpt_hwdrivers_CIbeoLuxETH, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CIbeoLuxETH", "This \"software driver\" implements the communication protocol for interfacing\n a Ibeo Lux laser scanners through an ethernet controller.\n   This class does not need to be bind, i.e. you do not need to call\n C2DRangeFinderAbstract::bindIO.\n   Connection is established when user call the turnOn() method. You can\n pass to the class's constructor the Lux's ip address and port.\n   Device will NOT be configured. Configuration has to be done seperately.\n\n To get a laser scan you must proceed like that :\n \n\n\n\n\n\n\n\n \n This class was contributed by Adrien Barral - Robopec (France)\n \n\n And modified by Jan Girlich - University of Hamburg\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CIbeoLuxETH(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CIbeoLuxETH(); } ), "doc");
		cl.def( pybind11::init( [](std::string const & a0){ return new mrpt::hwdrivers::CIbeoLuxETH(a0); }, [](std::string const & a0){ return new PyCallBack_mrpt_hwdrivers_CIbeoLuxETH(a0); } ), "doc");
		cl.def( pybind11::init<std::string, unsigned int>(), pybind11::arg("_ip"), pybind11::arg("_port") );

		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CIbeoLuxETH::*)() const) &mrpt::hwdrivers::CIbeoLuxETH::GetRuntimeClass, "C++: mrpt::hwdrivers::CIbeoLuxETH::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CIbeoLuxETH::CreateObject, "C++: mrpt::hwdrivers::CIbeoLuxETH::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CIbeoLuxETH::doRegister, "C++: mrpt::hwdrivers::CIbeoLuxETH::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CIbeoLuxETH::*)()) &mrpt::hwdrivers::CIbeoLuxETH::doProcess, "This function acquire a laser scan from the device. If an error occured,\n hardwareError will be set to true.\n The new laser scan will be stored in the outObservation argument.\n\n \n This method throw exception if the frame received from the\n LMS 100 contain the following bad parameters :\n  * Status is not OK\n  * Data in the scan aren't DIST1 (may be RSSIx or DIST2).\n\nC++: mrpt::hwdrivers::CIbeoLuxETH::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CIbeoLuxETH::*)()) &mrpt::hwdrivers::CIbeoLuxETH::initialize, "C++: mrpt::hwdrivers::CIbeoLuxETH::initialize() --> void");
		cl.def("makeCommandHeader", (void (mrpt::hwdrivers::CIbeoLuxETH::*)(unsigned char *)) &mrpt::hwdrivers::CIbeoLuxETH::makeCommandHeader, "C++: mrpt::hwdrivers::CIbeoLuxETH::makeCommandHeader(unsigned char *) --> void", pybind11::arg("buffer"));
		cl.def("makeStartCommand", (void (mrpt::hwdrivers::CIbeoLuxETH::*)(unsigned char *)) &mrpt::hwdrivers::CIbeoLuxETH::makeStartCommand, "C++: mrpt::hwdrivers::CIbeoLuxETH::makeStartCommand(unsigned char *) --> void", pybind11::arg("buffer"));
		cl.def("makeStopCommand", (void (mrpt::hwdrivers::CIbeoLuxETH::*)(unsigned char *)) &mrpt::hwdrivers::CIbeoLuxETH::makeStopCommand, "C++: mrpt::hwdrivers::CIbeoLuxETH::makeStopCommand(unsigned char *) --> void", pybind11::arg("buffer"));
		cl.def("makeTypeCommand", (void (mrpt::hwdrivers::CIbeoLuxETH::*)(unsigned char *)) &mrpt::hwdrivers::CIbeoLuxETH::makeTypeCommand, "C++: mrpt::hwdrivers::CIbeoLuxETH::makeTypeCommand(unsigned char *) --> void", pybind11::arg("buffer"));
	}
}
