#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/gnss_messages_type_list.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/typemeta/static_string.h>
#include <mutex>
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

// mrpt::hwdrivers::CGPSInterface file:mrpt/hwdrivers/CGPSInterface.h line:142
struct PyCallBack_mrpt_hwdrivers_CGPSInterface : public mrpt::hwdrivers::CGPSInterface {
	using mrpt::hwdrivers::CGPSInterface::CGPSInterface;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CGPSInterface::GetRuntimeClass();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGPSInterface::doProcess();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGPSInterface::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "initialize");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPSInterface *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CGPSInterface(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CGPSInterface file:mrpt/hwdrivers/CGPSInterface.h line:142
		pybind11::class_<mrpt::hwdrivers::CGPSInterface, std::shared_ptr<mrpt::hwdrivers::CGPSInterface>, PyCallBack_mrpt_hwdrivers_CGPSInterface, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CGPSInterface", "A class capable of reading GPS/GNSS/GNSS+IMU receiver data, from a serial\n port or from any input stream,\n  and  the ASCII/binary stream into indivual messages  in\n mrpt::obs::CObservationGPS objects.\n\n Typical input streams are serial ports or raw GPS log files. By default, the\n serial port selected by CGPSInterface::setSerialPortName()\n or as set in the configuration file will be open upon call to\n CGenericSensor::initialize().\n Alternatively, an external stream can be bound with\n CGPSInterface::bindStream() before calling CGenericSensor::initialize().\n This feature can be used to parse commands from a file, a TCP/IP stream, a\n memory block, etc.\n\n The parsers in the enum type CGPSInterface::PARSERS are supported as\n parameter `parser` in the\n configuration file below or in method CGPSInterface::setParser():\n  - `NONE`: Do not try to parse the messages into CObservation's. Only useful\n if combined with `raw_dump_file_prefix`\n  - `AUTO`: Try to automatically identify the format of incomming data.\n  - `NMEA` (NMEA 0183, ASCII messages): Default parser. Supported frames:\n GGA, RMC,... See full list of messages in children of\n mrpt::obs::gnss::gnss_message\n  - `NOVATEL_OEM6` (Novatel OEM6, binary frames): Supported frames:\n BESTPOS,... Note that receiving a correct IONUTC msg is required for a\n correct timestamping of subsequent frames. See full list of messages in\n children of mrpt::obs::gnss::gnss_message\n\n See available parameters below, and an example config file for\n rawlog-grabber\n [here](https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/rawlog-grabber/gps.ini)\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n Note that the `customInit` field, supported in MRPT <1.4.0 will be still\n parsed and obeyed, but since it has been superseded\n by the new mechanism to establish set-up commands, it is no further\n documented here.\n\n  The next picture summarizes existing MRPT classes related to GPS / GNSS\n devices (CGPSInterface, CNTRIPEmitter, CGPS_NTRIP):\n\n    \n\n VERSIONS HISTORY:\n - 09/JUN/2006: First version (JLBC)\n - 04/JUN/2008: Added virtual methods for device-specific initialization\n commands.\n - 10/JUN/2008: Converted into CGenericSensor class (there are no inhirited\n classes anymore).\n - 07/DEC/2012: Added public static method to parse NMEA strings.\n - 17/JUN/2014: Added GGA feedback.\n - 01/FEB/2016: API changed for MTPT 1.4.0\n\n  \n Verbose debug info will be dumped to cout if the environment variable\n \"MRPT_HWDRIVERS_VERBOSE\" is set to \"1\", or if you call\n CGenericSensor::enableVerbose(true)\n  \n\n\n  \n\n [API changed in MRPT 1.4.0] mrpt::hwdrivers::CGPSInterface API\n clean-up and made more generic so any stream can be used to parse GNSS\n messages, not only serial ports.\n\n \n CGPS_NTRIP, CNTRIPEmitter, mrpt::obs::CObservationGPS\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CGPSInterface(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CGPSInterface(); } ) );

		pybind11::enum_<mrpt::hwdrivers::CGPSInterface::PARSERS>(cl, "PARSERS", pybind11::arithmetic(), "Read about parser selection in the documentation for CGPSInterface ")
			.value("NONE", mrpt::hwdrivers::CGPSInterface::NONE)
			.value("AUTO", mrpt::hwdrivers::CGPSInterface::AUTO)
			.value("NMEA", mrpt::hwdrivers::CGPSInterface::NMEA)
			.value("NOVATEL_OEM6", mrpt::hwdrivers::CGPSInterface::NOVATEL_OEM6)
			.export_values();

		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CGPSInterface::*)() const) &mrpt::hwdrivers::CGPSInterface::GetRuntimeClass, "C++: mrpt::hwdrivers::CGPSInterface::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CGPSInterface::CreateObject, "C++: mrpt::hwdrivers::CGPSInterface::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CGPSInterface::doRegister, "C++: mrpt::hwdrivers::CGPSInterface::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CGPSInterface::*)()) &mrpt::hwdrivers::CGPSInterface::doProcess, "C++: mrpt::hwdrivers::CGPSInterface::doProcess() --> void");
		cl.def("isGPS_connected", (bool (mrpt::hwdrivers::CGPSInterface::*)()) &mrpt::hwdrivers::CGPSInterface::isGPS_connected, "Returns true if communications work, i.e. if some message has been\n received. \n\nC++: mrpt::hwdrivers::CGPSInterface::isGPS_connected() --> bool");
		cl.def("setSerialPortName", (void (mrpt::hwdrivers::CGPSInterface::*)(const std::string &)) &mrpt::hwdrivers::CGPSInterface::setSerialPortName, "@{ \n\n Set the serial port to use (COM1, ttyUSB0, etc). \n\nC++: mrpt::hwdrivers::CGPSInterface::setSerialPortName(const std::string &) --> void", pybind11::arg("COM_port"));
		cl.def("getSerialPortName", (std::string (mrpt::hwdrivers::CGPSInterface::*)() const) &mrpt::hwdrivers::CGPSInterface::getSerialPortName, "Get the serial port to use (COM1, ttyUSB0, etc). \n\nC++: mrpt::hwdrivers::CGPSInterface::getSerialPortName() const --> std::string");
		cl.def("setParser", (void (mrpt::hwdrivers::CGPSInterface::*)(enum mrpt::hwdrivers::CGPSInterface::PARSERS)) &mrpt::hwdrivers::CGPSInterface::setParser, "Select the parser for incomming data, among the options enumerated in \n \n\nC++: mrpt::hwdrivers::CGPSInterface::setParser(enum mrpt::hwdrivers::CGPSInterface::PARSERS) --> void", pybind11::arg("parser"));
		cl.def("getParser", (enum mrpt::hwdrivers::CGPSInterface::PARSERS (mrpt::hwdrivers::CGPSInterface::*)() const) &mrpt::hwdrivers::CGPSInterface::getParser, "C++: mrpt::hwdrivers::CGPSInterface::getParser() const --> enum mrpt::hwdrivers::CGPSInterface::PARSERS");
		cl.def("useExternalStream", (bool (mrpt::hwdrivers::CGPSInterface::*)() const) &mrpt::hwdrivers::CGPSInterface::useExternalStream, "C++: mrpt::hwdrivers::CGPSInterface::useExternalStream() const --> bool");
		cl.def("setSetupCommandsDelay", (void (mrpt::hwdrivers::CGPSInterface::*)(const double)) &mrpt::hwdrivers::CGPSInterface::setSetupCommandsDelay, "C++: mrpt::hwdrivers::CGPSInterface::setSetupCommandsDelay(const double) --> void", pybind11::arg("delay_secs"));
		cl.def("getSetupCommandsDelay", (double (mrpt::hwdrivers::CGPSInterface::*)() const) &mrpt::hwdrivers::CGPSInterface::getSetupCommandsDelay, "C++: mrpt::hwdrivers::CGPSInterface::getSetupCommandsDelay() const --> double");
		cl.def("setSetupCommands", (void (mrpt::hwdrivers::CGPSInterface::*)(const class std::vector<std::string > &)) &mrpt::hwdrivers::CGPSInterface::setSetupCommands, "C++: mrpt::hwdrivers::CGPSInterface::setSetupCommands(const class std::vector<std::string > &) --> void", pybind11::arg("cmds"));
		cl.def("getSetupCommands", (const class std::vector<std::string > & (mrpt::hwdrivers::CGPSInterface::*)() const) &mrpt::hwdrivers::CGPSInterface::getSetupCommands, "C++: mrpt::hwdrivers::CGPSInterface::getSetupCommands() const --> const class std::vector<std::string > &", pybind11::return_value_policy::automatic);
		cl.def("setShutdownCommands", (void (mrpt::hwdrivers::CGPSInterface::*)(const class std::vector<std::string > &)) &mrpt::hwdrivers::CGPSInterface::setShutdownCommands, "C++: mrpt::hwdrivers::CGPSInterface::setShutdownCommands(const class std::vector<std::string > &) --> void", pybind11::arg("cmds"));
		cl.def("getShutdownCommands", (const class std::vector<std::string > & (mrpt::hwdrivers::CGPSInterface::*)() const) &mrpt::hwdrivers::CGPSInterface::getShutdownCommands, "C++: mrpt::hwdrivers::CGPSInterface::getShutdownCommands() const --> const class std::vector<std::string > &", pybind11::return_value_policy::automatic);
		cl.def("enableSetupCommandsAppendCRLF", (void (mrpt::hwdrivers::CGPSInterface::*)(const bool)) &mrpt::hwdrivers::CGPSInterface::enableSetupCommandsAppendCRLF, "C++: mrpt::hwdrivers::CGPSInterface::enableSetupCommandsAppendCRLF(const bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledSetupCommandsAppendCRLF", (bool (mrpt::hwdrivers::CGPSInterface::*)() const) &mrpt::hwdrivers::CGPSInterface::isEnabledSetupCommandsAppendCRLF, "C++: mrpt::hwdrivers::CGPSInterface::isEnabledSetupCommandsAppendCRLF() const --> bool");
		cl.def("enableAppendMsgTypeToSensorLabel", (void (mrpt::hwdrivers::CGPSInterface::*)(bool)) &mrpt::hwdrivers::CGPSInterface::enableAppendMsgTypeToSensorLabel, "C++: mrpt::hwdrivers::CGPSInterface::enableAppendMsgTypeToSensorLabel(bool) --> void", pybind11::arg("enable"));
		cl.def("setRawDumpFilePrefix", (void (mrpt::hwdrivers::CGPSInterface::*)(const std::string &)) &mrpt::hwdrivers::CGPSInterface::setRawDumpFilePrefix, "If set to non-empty, RAW GPS serial data will be also dumped to a\n separate file. \n\nC++: mrpt::hwdrivers::CGPSInterface::setRawDumpFilePrefix(const std::string &) --> void", pybind11::arg("filePrefix"));
		cl.def("getRawDumpFilePrefix", (std::string (mrpt::hwdrivers::CGPSInterface::*)() const) &mrpt::hwdrivers::CGPSInterface::getRawDumpFilePrefix, "C++: mrpt::hwdrivers::CGPSInterface::getRawDumpFilePrefix() const --> std::string");
		cl.def("sendCustomCommand", (bool (mrpt::hwdrivers::CGPSInterface::*)(const void *, size_t)) &mrpt::hwdrivers::CGPSInterface::sendCustomCommand, "Send a custom data block to the GNSS device right now. Can be used to\n  change its behavior online as needed.\n  \n\n false on communication error \n\nC++: mrpt::hwdrivers::CGPSInterface::sendCustomCommand(const void *, size_t) --> bool", pybind11::arg("data"), pybind11::arg("datalen"));
		cl.def("isAIMConfigured", (bool (mrpt::hwdrivers::CGPSInterface::*)()) &mrpt::hwdrivers::CGPSInterface::isAIMConfigured, "@} \n\nC++: mrpt::hwdrivers::CGPSInterface::isAIMConfigured() --> bool");
		cl.def_static("parse_NMEA", [](const std::string & a0, class mrpt::obs::CObservationGPS & a1) -> bool { return mrpt::hwdrivers::CGPSInterface::parse_NMEA(a0, a1); }, "", pybind11::arg("cmd_line"), pybind11::arg("out_obs"));
		cl.def_static("parse_NMEA", (bool (*)(const std::string &, class mrpt::obs::CObservationGPS &, const bool)) &mrpt::hwdrivers::CGPSInterface::parse_NMEA, "Parses one line of NMEA data from a GPS receiver, and writes the\n recognized fields (if any) into an observation object.\n Recognized frame types are those listed for the `NMEA` parser in the\n documentation of CGPSInterface\n \n\n true if some new data field has been correctly parsed and\n inserted into out_obs\n\nC++: mrpt::hwdrivers::CGPSInterface::parse_NMEA(const std::string &, class mrpt::obs::CObservationGPS &, const bool) --> bool", pybind11::arg("cmd_line"), pybind11::arg("out_obs"), pybind11::arg("verbose"));
		cl.def("getLastGGA", [](mrpt::hwdrivers::CGPSInterface &o) -> std::string { return o.getLastGGA(); }, "");
		cl.def("getLastGGA", (std::string (mrpt::hwdrivers::CGPSInterface::*)(bool)) &mrpt::hwdrivers::CGPSInterface::getLastGGA, "Gets the latest GGA command or an empty string if no newer GGA command\n was received since the last call to this method.\n \n\n If set to true, will empty the GGA cache so next calls\n will return an empty string if no new frame is received.\n\nC++: mrpt::hwdrivers::CGPSInterface::getLastGGA(bool) --> std::string", pybind11::arg("reset"));
		cl.def("implement_parser_NMEA", (bool (mrpt::hwdrivers::CGPSInterface::*)(unsigned long &)) &mrpt::hwdrivers::CGPSInterface::implement_parser_NMEA, "bytes in the\n  incoming buffer, and return false if the available data does not match\n the expected format, so we must skip 1 byte and try again.\n @{ \n\nC++: mrpt::hwdrivers::CGPSInterface::implement_parser_NMEA(unsigned long &) --> bool", pybind11::arg("out_minimum_rx_buf_to_decide"));
		cl.def("implement_parser_NOVATEL_OEM6", (bool (mrpt::hwdrivers::CGPSInterface::*)(unsigned long &)) &mrpt::hwdrivers::CGPSInterface::implement_parser_NOVATEL_OEM6, "C++: mrpt::hwdrivers::CGPSInterface::implement_parser_NOVATEL_OEM6(unsigned long &) --> bool", pybind11::arg("out_minimum_rx_buf_to_decide"));
	}
}
