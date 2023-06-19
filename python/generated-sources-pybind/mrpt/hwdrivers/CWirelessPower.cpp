#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CWirelessPower.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationWirelessPower.h>
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
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::hwdrivers::CWirelessPower file:mrpt/hwdrivers/CWirelessPower.h line:22
struct PyCallBack_mrpt_hwdrivers_CWirelessPower : public mrpt::hwdrivers::CWirelessPower {
	using mrpt::hwdrivers::CWirelessPower::CWirelessPower;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CWirelessPower::GetRuntimeClass();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWirelessPower::doProcess();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CWirelessPower::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "loadConfig");
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
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::initialize();
	}
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> >;
	_binder_ret_0 getObservations() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CWirelessPower *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CWirelessPower(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CWirelessPower file:mrpt/hwdrivers/CWirelessPower.h line:22
		pybind11::class_<mrpt::hwdrivers::CWirelessPower, std::shared_ptr<mrpt::hwdrivers::CWirelessPower>, PyCallBack_mrpt_hwdrivers_CWirelessPower, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CWirelessPower", "This class implements a wireless power probe.\n  \n\n mrpt::maps::CWirelessPowerGridMap2D,\n mrpt::obs::CObservationWirelessPower\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CWirelessPower(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CWirelessPower(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CWirelessPower::*)() const) &mrpt::hwdrivers::CWirelessPower::GetRuntimeClass, "C++: mrpt::hwdrivers::CWirelessPower::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CWirelessPower::CreateObject, "C++: mrpt::hwdrivers::CWirelessPower::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CWirelessPower::doRegister, "C++: mrpt::hwdrivers::CWirelessPower::doRegister() --> void");
		cl.def("setNet", (void (mrpt::hwdrivers::CWirelessPower::*)(std::string, std::string)) &mrpt::hwdrivers::CWirelessPower::setNet, "Set the SSID and GUID of the target network.\n \n\n std::exception In case there is a failure\n \n\n SSID of the target network\n \n\n GUID of the network interface\n\nC++: mrpt::hwdrivers::CWirelessPower::setNet(std::string, std::string) --> void", pybind11::arg("ssid"), pybind11::arg("guid"));
		cl.def("doProcess", (void (mrpt::hwdrivers::CWirelessPower::*)()) &mrpt::hwdrivers::CWirelessPower::doProcess, "C++: mrpt::hwdrivers::CWirelessPower::doProcess() --> void");
		cl.def("loadConfig_sensorSpecific", (void (mrpt::hwdrivers::CWirelessPower::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::hwdrivers::CWirelessPower::loadConfig_sensorSpecific, "C++: mrpt::hwdrivers::CWirelessPower::loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("section"));
		cl.def("ListInterfaces", (class std::vector<std::string > (mrpt::hwdrivers::CWirelessPower::*)()) &mrpt::hwdrivers::CWirelessPower::ListInterfaces, "Gets a list of the interfaces\n \n\n std::exception In case there is a failure\n \n\n std::vector returns the identifiers (GUID) of the available\n interfaces\n\nC++: mrpt::hwdrivers::CWirelessPower::ListInterfaces() --> class std::vector<std::string >");
		cl.def("GetPower", (int (mrpt::hwdrivers::CWirelessPower::*)()) &mrpt::hwdrivers::CWirelessPower::GetPower, "Gets the power of a given network\n \n\n std::exception In case there is a failure\n \n\n Returns the power (0-100).\n\nC++: mrpt::hwdrivers::CWirelessPower::GetPower() --> int");
		cl.def("getObservation", (bool (mrpt::hwdrivers::CWirelessPower::*)(class mrpt::obs::CObservationWirelessPower &)) &mrpt::hwdrivers::CWirelessPower::getObservation, "Gets the power of a given network as a timestamped observation\n NOTE: Deprecated, use getObservations instead. See CGenericSensor\n documentation. This function is kept for internal use of the module\n \n\n Returns true if the observation was correct, and false otherwise\n \n\n mrpt::hwdrivers::CGenericSensor\n\nC++: mrpt::hwdrivers::CWirelessPower::getObservation(class mrpt::obs::CObservationWirelessPower &) --> bool", pybind11::arg("outObservation"));
		cl.def("ListNetworks", (class std::vector<std::string > (mrpt::hwdrivers::CWirelessPower::*)()) &mrpt::hwdrivers::CWirelessPower::ListNetworks, "Gets a list of the networks available for an interface\n \n\n std::exception In case there is a failure\n \n\n std::vector returns handles to the available networks of a given\n interface\n\nC++: mrpt::hwdrivers::CWirelessPower::ListNetworks() --> class std::vector<std::string >");
	}
}
