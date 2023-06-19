#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
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

// mrpt::hwdrivers::CGenericSensor file:mrpt/hwdrivers/CGenericSensor.h line:71
struct PyCallBack_mrpt_hwdrivers_CGenericSensor : public mrpt::hwdrivers::CGenericSensor {
	using mrpt::hwdrivers::CGenericSensor::CGenericSensor;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CGenericSensor::GetRuntimeClass\"");
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CGenericSensor::loadConfig_sensorSpecific\"");
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "initialize");
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
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CGenericSensor::doProcess\"");
	}
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> >;
	_binder_ret_0 getObservations() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGenericSensor *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CGenericSensor(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::TSensorClassId file:mrpt/hwdrivers/CGenericSensor.h line:31
		pybind11::class_<mrpt::hwdrivers::TSensorClassId, std::shared_ptr<mrpt::hwdrivers::TSensorClassId>> cl(M("mrpt::hwdrivers"), "TSensorClassId", "A structure for runtime ID class type information in the context of\n hwdrivers::CGenericSensor.");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::TSensorClassId(); } ) );
	}
	{ // mrpt::hwdrivers::CGenericSensor file:mrpt/hwdrivers/CGenericSensor.h line:71
		pybind11::class_<mrpt::hwdrivers::CGenericSensor, std::shared_ptr<mrpt::hwdrivers::CGenericSensor>, PyCallBack_mrpt_hwdrivers_CGenericSensor> cl(M("mrpt::hwdrivers"), "CGenericSensor", "A generic interface for a wide-variety of sensors designed to be used in the\napplication RawLogGrabber.\n  Derived classes should be designed with the following execution flow in\nmind:\n		- Object constructor\n		- CGenericSensor::loadConfig: The following parameters are common to all\nsensors in rawlog-grabber (they are automatically loaded by rawlog-grabber) -\nsee each class documentation for additional parameters:\n			- \"process_rate\": (Mandatory) The rate in Hertz (Hz) at which the\nsensor\nthread should invoke \"doProcess\".\n			- \"max_queue_len\": (Optional) The maximum number of objects in the\nobservations queue (default is 200). If overflow occurs, an error message\nwill be issued at run-time.\n			- \"grab_decimation\": (Optional) Grab only 1 out of N observations\ncaptured\nby the sensor (default is 1, i.e. do not decimate).\n		- CGenericSensor::initialize\n		- CGenericSensor::doProcess\n		- CGenericSensor::getObservations\n\n  Notice that there are helper methods for managing the internal list of\nobjects (see CGenericSensor::appendObservation).\n\n  Class Factory: This is also a factory of derived classes, through\nthe static method CGenericSensor::createSensor\n\n  For more details on RawLogGrabber refer to the wiki page:\n    https://www.mrpt.org/Application:RawLogGrabber\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_hwdrivers_CGenericSensor(); } ) );

		pybind11::enum_<mrpt::hwdrivers::CGenericSensor::TSensorState>(cl, "TSensorState", pybind11::arithmetic(), "The current state of the sensor\n \n\n CGenericSensor::getState")
			.value("ssInitializing", mrpt::hwdrivers::CGenericSensor::ssInitializing)
			.value("ssWorking", mrpt::hwdrivers::CGenericSensor::ssWorking)
			.value("ssError", mrpt::hwdrivers::CGenericSensor::ssError)
			.value("ssUninitialized", mrpt::hwdrivers::CGenericSensor::ssUninitialized)
			.export_values();

		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CGenericSensor::*)() const) &mrpt::hwdrivers::CGenericSensor::GetRuntimeClass, "C++: mrpt::hwdrivers::CGenericSensor::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def("getState", (enum mrpt::hwdrivers::CGenericSensor::TSensorState (mrpt::hwdrivers::CGenericSensor::*)() const) &mrpt::hwdrivers::CGenericSensor::getState, "The current state of the sensor  \n\nC++: mrpt::hwdrivers::CGenericSensor::getState() const --> enum mrpt::hwdrivers::CGenericSensor::TSensorState");
		cl.def("getProcessRate", (double (mrpt::hwdrivers::CGenericSensor::*)() const) &mrpt::hwdrivers::CGenericSensor::getProcessRate, "C++: mrpt::hwdrivers::CGenericSensor::getProcessRate() const --> double");
		cl.def("getSensorLabel", (std::string (mrpt::hwdrivers::CGenericSensor::*)() const) &mrpt::hwdrivers::CGenericSensor::getSensorLabel, "C++: mrpt::hwdrivers::CGenericSensor::getSensorLabel() const --> std::string");
		cl.def("setSensorLabel", (void (mrpt::hwdrivers::CGenericSensor::*)(const std::string &)) &mrpt::hwdrivers::CGenericSensor::setSensorLabel, "C++: mrpt::hwdrivers::CGenericSensor::setSensorLabel(const std::string &) --> void", pybind11::arg("sensorLabel"));
		cl.def("enableVerbose", [](mrpt::hwdrivers::CGenericSensor &o) -> void { return o.enableVerbose(); }, "");
		cl.def("enableVerbose", (void (mrpt::hwdrivers::CGenericSensor::*)(bool)) &mrpt::hwdrivers::CGenericSensor::enableVerbose, "Enable or disable extra debug info dumped to std::cout during sensor\n operation.\n Default: disabled unless the environment variable\n \"MRPT_HWDRIVERS_VERBOSE\" is set to \"1\" during object creation.\n\nC++: mrpt::hwdrivers::CGenericSensor::enableVerbose(bool) --> void", pybind11::arg("enabled"));
		cl.def("isVerboseEnabled", (bool (mrpt::hwdrivers::CGenericSensor::*)() const) &mrpt::hwdrivers::CGenericSensor::isVerboseEnabled, "C++: mrpt::hwdrivers::CGenericSensor::isVerboseEnabled() const --> bool");
		cl.def_static("registerClass", (void (*)(const struct mrpt::hwdrivers::TSensorClassId *)) &mrpt::hwdrivers::CGenericSensor::registerClass, "Register a class into the internal list of \"CGenericSensor\" descendents.\n  Used internally in the macros DEFINE_GENERIC_SENSOR, etc...\n\n  Can be used as \"CGenericSensor::registerClass(\n SENSOR_CLASS_ID(CMySensor) );\" if\n    building custom sensors outside mrpt libraries in user code.\n\nC++: mrpt::hwdrivers::CGenericSensor::registerClass(const struct mrpt::hwdrivers::TSensorClassId *) --> void", pybind11::arg("pNewClass"));
		cl.def_static("createSensor", (class mrpt::hwdrivers::CGenericSensor * (*)(const std::string &)) &mrpt::hwdrivers::CGenericSensor::createSensor, "Creates a sensor by a name of the class.\n  Typically the user may want to create a smart pointer around the\n returned pointer, whis is made with:\n  \n\n\n\n\n \n A pointer to a new class, or nullptr if class name is unknown.\n\nC++: mrpt::hwdrivers::CGenericSensor::createSensor(const std::string &) --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic, pybind11::arg("className"));
		cl.def_static("createSensorPtr", (class std::shared_ptr<class mrpt::hwdrivers::CGenericSensor> (*)(const std::string &)) &mrpt::hwdrivers::CGenericSensor::createSensorPtr, "Just like createSensor, but returning a smart pointer to the newly\n created sensor object. \n\nC++: mrpt::hwdrivers::CGenericSensor::createSensorPtr(const std::string &) --> class std::shared_ptr<class mrpt::hwdrivers::CGenericSensor>", pybind11::arg("className"));
		cl.def("loadConfig", (void (mrpt::hwdrivers::CGenericSensor::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::hwdrivers::CGenericSensor::loadConfig, "Loads the generic settings common to any sensor (See CGenericSensor),\n then call to \"loadConfig_sensorSpecific\"\n  \n\n This method throws an exception with a descriptive message\n if some critical parameter is missing or has an invalid value.\n\nC++: mrpt::hwdrivers::CGenericSensor::loadConfig(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("section"));
		cl.def("initialize", (void (mrpt::hwdrivers::CGenericSensor::*)()) &mrpt::hwdrivers::CGenericSensor::initialize, "This method can or cannot be implemented in the derived class, depending\n on the need for it.\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CGenericSensor::initialize() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CGenericSensor::*)()) &mrpt::hwdrivers::CGenericSensor::doProcess, "This method will be invoked at a minimum rate of \"process_rate\" (Hz)\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CGenericSensor::doProcess() --> void");
		cl.def("getObservations", (class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > (mrpt::hwdrivers::CGenericSensor::*)()) &mrpt::hwdrivers::CGenericSensor::getObservations, "Returns a list of enqueued objects, emptying it (thread-safe).\n\nC++: mrpt::hwdrivers::CGenericSensor::getObservations() --> class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> >");
		cl.def("setPathForExternalImages", (void (mrpt::hwdrivers::CGenericSensor::*)(const std::string &)) &mrpt::hwdrivers::CGenericSensor::setPathForExternalImages, "Set the path where to save off-rawlog image files (will be ignored in\n those sensors where this is not applicable).\n  An  empty string (the default value at construction) means to save\n images embedded in the rawlog, instead of on separate files.\n \n\n std::exception If the directory doesn't exists and cannot be\n created.\n\nC++: mrpt::hwdrivers::CGenericSensor::setPathForExternalImages(const std::string &) --> void", pybind11::arg("directory"));
		cl.def("setExternalImageFormat", (void (mrpt::hwdrivers::CGenericSensor::*)(const std::string &)) &mrpt::hwdrivers::CGenericSensor::setExternalImageFormat, "Set the extension (\"jpg\",\"gif\",\"png\",...) that determines the format of\n images saved externally. Default: \"png\".\n \n\n setPathForExternalImages, setExternalImageJPEGQuality\n\nC++: mrpt::hwdrivers::CGenericSensor::setExternalImageFormat(const std::string &) --> void", pybind11::arg("ext"));
		cl.def("setExternalImageJPEGQuality", (void (mrpt::hwdrivers::CGenericSensor::*)(const unsigned int)) &mrpt::hwdrivers::CGenericSensor::setExternalImageJPEGQuality, "The quality of JPEG compression, when external images is enabled and the\n format is \"jpg\". \n\n setExternalImageFormat \n\nC++: mrpt::hwdrivers::CGenericSensor::setExternalImageJPEGQuality(const unsigned int) --> void", pybind11::arg("quality"));
		cl.def("getExternalImageJPEGQuality", (unsigned int (mrpt::hwdrivers::CGenericSensor::*)() const) &mrpt::hwdrivers::CGenericSensor::getExternalImageJPEGQuality, "C++: mrpt::hwdrivers::CGenericSensor::getExternalImageJPEGQuality() const --> unsigned int");
	}
}
