#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CBoardENoses.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <utility>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::hwdrivers::CBoardENoses file:mrpt/hwdrivers/CBoardENoses.h line:54
struct PyCallBack_mrpt_hwdrivers_CBoardENoses : public mrpt::hwdrivers::CBoardENoses {
	using mrpt::hwdrivers::CBoardENoses::CBoardENoses;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CBoardENoses::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBoardENoses::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBoardENoses::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CBoardENoses::initialize();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CBoardENoses *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CBoardENoses(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CBoardENoses file:mrpt/hwdrivers/CBoardENoses.h line:54
		pybind11::class_<mrpt::hwdrivers::CBoardENoses, std::shared_ptr<mrpt::hwdrivers::CBoardENoses>, PyCallBack_mrpt_hwdrivers_CBoardENoses, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CBoardENoses", "A class for interfacing an e-Noses via a FTDI USB link.\n  Implemented for the board v1.0 designed by 2007 @ ISA (University of\n Malaga).\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CBoardENoses(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CBoardENoses(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CBoardENoses::*)() const) &mrpt::hwdrivers::CBoardENoses::GetRuntimeClass, "C++: mrpt::hwdrivers::CBoardENoses::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CBoardENoses::CreateObject, "C++: mrpt::hwdrivers::CBoardENoses::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CBoardENoses::doRegister, "C++: mrpt::hwdrivers::CBoardENoses::doRegister() --> void");
		cl.def("setActiveChamber", (bool (mrpt::hwdrivers::CBoardENoses::*)(unsigned char)) &mrpt::hwdrivers::CBoardENoses::setActiveChamber, "Set the active chamber (afected by poluted air) on the device\n \n\n true on success, false on communications errors or device not\n found.\n\nC++: mrpt::hwdrivers::CBoardENoses::setActiveChamber(unsigned char) --> bool", pybind11::arg("chamber"));
		cl.def("queryFirmwareVersion", (bool (mrpt::hwdrivers::CBoardENoses::*)(std::string &)) &mrpt::hwdrivers::CBoardENoses::queryFirmwareVersion, "Query the firmware version on the device (can be used to test\n communications).\n \n\n true on success, false on communications errors or device not\n found.\n\nC++: mrpt::hwdrivers::CBoardENoses::queryFirmwareVersion(std::string &) --> bool", pybind11::arg("out_firmwareVersion"));
		cl.def("getObservation", (bool (mrpt::hwdrivers::CBoardENoses::*)(class mrpt::obs::CObservationGasSensors &)) &mrpt::hwdrivers::CBoardENoses::getObservation, "Request the master eNose the latest readings from all the eNoses.\n  The output observation contains a valid timestamp and 3D positions if\n \"loadConfig\" has been called previously.\n \n\n true if OK, false if there were any error.\n\nC++: mrpt::hwdrivers::CBoardENoses::getObservation(class mrpt::obs::CObservationGasSensors &) --> bool", pybind11::arg("outObservation"));
		cl.def("doProcess", (void (mrpt::hwdrivers::CBoardENoses::*)()) &mrpt::hwdrivers::CBoardENoses::doProcess, "C++: mrpt::hwdrivers::CBoardENoses::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CBoardENoses::*)()) &mrpt::hwdrivers::CBoardENoses::initialize, "Tries to open the camera, after setting all the parameters with a call\n to loadConfig.\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CBoardENoses::initialize() --> void");
		cl.def("setSerialPort", (void (mrpt::hwdrivers::CBoardENoses::*)(const std::string &)) &mrpt::hwdrivers::CBoardENoses::setSerialPort, "If not an empty string, will open that serial port, otherwise will try\n to open USB FTDI device \"m_usbSerialNumber\"\n  The default is an empty string. Example strings: \"COM1\", \"ttyUSB0\", ...\n\nC++: mrpt::hwdrivers::CBoardENoses::setSerialPort(const std::string &) --> void", pybind11::arg("port"));
		cl.def("getSerialPort", (std::string (mrpt::hwdrivers::CBoardENoses::*)() const) &mrpt::hwdrivers::CBoardENoses::getSerialPort, "C++: mrpt::hwdrivers::CBoardENoses::getSerialPort() const --> std::string");
		cl.def("setSerialPortBaud", (void (mrpt::hwdrivers::CBoardENoses::*)(unsigned int)) &mrpt::hwdrivers::CBoardENoses::setSerialPortBaud, "Set the serial port baud rate (default: 115200) \n\nC++: mrpt::hwdrivers::CBoardENoses::setSerialPortBaud(unsigned int) --> void", pybind11::arg("baud"));
		cl.def("getSerialPortBaud", (unsigned int (mrpt::hwdrivers::CBoardENoses::*)() const) &mrpt::hwdrivers::CBoardENoses::getSerialPortBaud, "C++: mrpt::hwdrivers::CBoardENoses::getSerialPortBaud() const --> unsigned int");
	}
}
