#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CCANBusReader.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationCANBusJ1939.h>
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

// mrpt::hwdrivers::CCANBusReader file:mrpt/hwdrivers/CCANBusReader.h line:57
struct PyCallBack_mrpt_hwdrivers_CCANBusReader : public mrpt::hwdrivers::CCANBusReader {
	using mrpt::hwdrivers::CCANBusReader::CCANBusReader;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CCANBusReader::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCANBusReader::loadConfig_sensorSpecific(a0, a1);
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCANBusReader::initialize();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCANBusReader::doProcess();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CCANBusReader(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CCANBusReader file:mrpt/hwdrivers/CCANBusReader.h line:57
		pybind11::class_<mrpt::hwdrivers::CCANBusReader, std::shared_ptr<mrpt::hwdrivers::CCANBusReader>, PyCallBack_mrpt_hwdrivers_CCANBusReader, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CCANBusReader", "This \"software driver\" implements the communication protocol for interfacing\n a SICK LMS 2XX laser scanners through a standard RS232 serial port (or a\n USB2SERIAL converter).\n   The serial port is opened upon the first call to \"doProcess\" or\n \"initialize\", so you must call \"loadConfig\" before\n   this, or manually call \"setSerialPort\". Another alternative is to call the\n base class method C2DRangeFinderAbstract::bindIO,\n   but the \"setSerialPort\" interface is probably much simpler to use.\n\n   For an example of usage see the example in\n \"samples/SICK_laser_serial_test\".\n   See also the example configuration file for rawlog-grabber in\n \"share/mrpt/config_files/rawlog-grabber\".\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n C2DRangeFinderAbstract\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CCANBusReader(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CCANBusReader(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CCANBusReader::*)() const) &mrpt::hwdrivers::CCANBusReader::GetRuntimeClass, "C++: mrpt::hwdrivers::CCANBusReader::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CCANBusReader::CreateObject, "C++: mrpt::hwdrivers::CCANBusReader::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CCANBusReader::doRegister, "C++: mrpt::hwdrivers::CCANBusReader::doRegister() --> void");
		cl.def("setSerialPort", (void (mrpt::hwdrivers::CCANBusReader::*)(const std::string &)) &mrpt::hwdrivers::CCANBusReader::setSerialPort, "Changes the serial port to connect to (call prior to 'doProcess'), for\n example \"COM1\" or \"ttyS0\".\n  This is not needed if the configuration is loaded with \"loadConfig\".\n\nC++: mrpt::hwdrivers::CCANBusReader::setSerialPort(const std::string &) --> void", pybind11::arg("port"));
		cl.def("getSerialPort", (std::string (mrpt::hwdrivers::CCANBusReader::*)() const) &mrpt::hwdrivers::CCANBusReader::getSerialPort, "setSerialPort \n\nC++: mrpt::hwdrivers::CCANBusReader::getSerialPort() const --> std::string");
		cl.def("setBaudRate", (void (mrpt::hwdrivers::CCANBusReader::*)(int)) &mrpt::hwdrivers::CCANBusReader::setBaudRate, "Changes the serial port baud rate (call prior to 'doProcess'); valid\n values are 9600,38400 and 500000.\n  This is not needed if the configuration is loaded with \"loadConfig\".\n  \n\n getBaudRate \n\nC++: mrpt::hwdrivers::CCANBusReader::setBaudRate(int) --> void", pybind11::arg("baud"));
		cl.def("getBaudRate", (int (mrpt::hwdrivers::CCANBusReader::*)() const) &mrpt::hwdrivers::CCANBusReader::getBaudRate, "setBaudRate \n\nC++: mrpt::hwdrivers::CCANBusReader::getBaudRate() const --> int");
		cl.def("setCANReaderTimeStamping", [](mrpt::hwdrivers::CCANBusReader &o) -> void { return o.setCANReaderTimeStamping(); }, "");
		cl.def("setCANReaderTimeStamping", (void (mrpt::hwdrivers::CCANBusReader::*)(bool)) &mrpt::hwdrivers::CCANBusReader::setCANReaderTimeStamping, "Enables/Disables the addition of a timestamp according to the arrival\n time to the converter (default=false)\n  (call prior to 'doProcess') This is not needed if the configuration is\n loaded with \"loadConfig\".\n\nC++: mrpt::hwdrivers::CCANBusReader::setCANReaderTimeStamping(bool) --> void", pybind11::arg("setTimestamp"));
		cl.def("getCANReaderTimeStamping", (bool (mrpt::hwdrivers::CCANBusReader::*)()) &mrpt::hwdrivers::CCANBusReader::getCANReaderTimeStamping, "C++: mrpt::hwdrivers::CCANBusReader::getCANReaderTimeStamping() --> bool");
		cl.def("setCANReaderSpeed", (void (mrpt::hwdrivers::CCANBusReader::*)(const unsigned int)) &mrpt::hwdrivers::CCANBusReader::setCANReaderSpeed, "Sets the CAN reader speed when connecting to the CAN Bus\n\nC++: mrpt::hwdrivers::CCANBusReader::setCANReaderSpeed(const unsigned int) --> void", pybind11::arg("speed"));
		cl.def("getCANReaderSpeed", (unsigned int (mrpt::hwdrivers::CCANBusReader::*)()) &mrpt::hwdrivers::CCANBusReader::getCANReaderSpeed, "C++: mrpt::hwdrivers::CCANBusReader::getCANReaderSpeed() --> unsigned int");
		cl.def("getCurrentConnectTry", (unsigned int (mrpt::hwdrivers::CCANBusReader::*)() const) &mrpt::hwdrivers::CCANBusReader::getCurrentConnectTry, "If performing several tries in ::initialize(), this is the current try\n loop number. \n\nC++: mrpt::hwdrivers::CCANBusReader::getCurrentConnectTry() const --> unsigned int");
		cl.def("doProcessSimple", (void (mrpt::hwdrivers::CCANBusReader::*)(bool &, class mrpt::obs::CObservationCANBusJ1939 &, bool &)) &mrpt::hwdrivers::CCANBusReader::doProcessSimple, "Specific laser scanner \"software drivers\" must process here new data\n from the I/O stream, and, if a whole scan has arrived, return it.\n  This method will be typically called in a different thread than other\n methods, and will be called in a timely fashion.\n\nC++: mrpt::hwdrivers::CCANBusReader::doProcessSimple(bool &, class mrpt::obs::CObservationCANBusJ1939 &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("initialize", (void (mrpt::hwdrivers::CCANBusReader::*)()) &mrpt::hwdrivers::CCANBusReader::initialize, "Set-up communication with the laser.\n  Called automatically by rawlog-grabber.\n  If used manually, call after \"loadConfig\" and before \"doProcess\".\n\n  In this class this method does nothing, since the communications are\n setup at the first try from \"doProcess\" or \"doProcessSimple\".\n\nC++: mrpt::hwdrivers::CCANBusReader::initialize() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CCANBusReader::*)()) &mrpt::hwdrivers::CCANBusReader::doProcess, "C++: mrpt::hwdrivers::CCANBusReader::doProcess() --> void");
	}
}
