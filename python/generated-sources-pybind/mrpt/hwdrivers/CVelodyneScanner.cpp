#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CVelodyneScanner.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/VelodyneCalibration.h>
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

// mrpt::hwdrivers::CVelodyneScanner file:mrpt/hwdrivers/CVelodyneScanner.h line:165
struct PyCallBack_mrpt_hwdrivers_CVelodyneScanner : public mrpt::hwdrivers::CVelodyneScanner {
	using mrpt::hwdrivers::CVelodyneScanner::CVelodyneScanner;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CVelodyneScanner::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVelodyneScanner::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVelodyneScanner::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CVelodyneScanner::initialize();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CVelodyneScanner *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CVelodyneScanner(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CVelodyneScanner file:mrpt/hwdrivers/CVelodyneScanner.h line:165
		pybind11::class_<mrpt::hwdrivers::CVelodyneScanner, std::shared_ptr<mrpt::hwdrivers::CVelodyneScanner>, PyCallBack_mrpt_hwdrivers_CVelodyneScanner, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CVelodyneScanner", "A C++ interface to Velodyne laser scanners (HDL-64, HDL-32, VLP-16), working\n on Linux and Windows.\n (Using this class requires WinPCap as a run-time dependency in Windows).\n It can receive data from real devices via an Ethernet connection or parse a\n WireShark PCAP file for offline processing.\n The choice of online vs. offline operation is taken upon calling \n if a PCAP input file has been defined,\n offline operation takes place and network is not listened for incomming\n packets.\n\n Parsing dual return scans requires a VLP-16 with firmware version 3.0.23 or\n newer. While converting the scan into a\n point cloud in mrpt::obs::CObservationVelodyneScan you can select whether to\n keep the strongest, the last or both laser returns.\n\n XML calibration files are not mandatory for VLP-16 and HDL-32, but they are\n for HDL-64.\n\n Grabbing live data (as a user) \n  - Use the application\n [velodyne-view](http://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/)\n to visualize the LIDAR output in real-time (optionally saving to a PCAP file)\n or to playback a PCAP file.\n  - Use\n [rawlog-grabber](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/)\n to record a dataset in MRPT's format together with any other set of sensors.\n See example config file:\n [MRPT_files-grabber.ini](https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/rawlog-grabber/velodyne.ini)\n\n Grabbing live data (programmatically) \n  - See CGenericSensor for a general overview of the sequence of methods to\n be called: loadConfig(), initialize(), doProcess().\n  - Or use this class inside the application\n [rawlog-grabber](http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/).\n See example config files:\n [MRPT_files-grabber.ini](https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/rawlog-grabber/velodyne.ini)\n\n See the source code of the example application `[MRPT]/apps/velodyne-view`\n ([velodyne-view web\n page](http://www.mrpt.org/list-of-mrpt-apps/application-velodyne-view/)) for\n more details.\n\n Playing back a PCAP file:\n  It is common to save Velodyne datasets as Wireshark's PCAP files.\n  These files can be played back with tools like\n [bittwist](http://bittwist.sourceforge.net/), which emit all UDP packets in\n the PCAP log.\n  Then, use this class to receive the packets as if they come from the real\n sensor.\n\n  Alternatively, if MRPT is linked against libpcap, this class can directly\n parse a PCAP file to simulate reading from a device offline.\n  See method setPCAPInputFile() and config file parameter ``\n\n  To compile with PCAP support: In Debian/Ubuntu, install libpcap-dev. In\n Windows, install WinPCap developer packages + the regular WinPCap driver.\n\n  Configuration and usage: \n Data is returned as observations of type:\n  - mrpt::obs::CObservationVelodyneScan for one or more \"data packets\" (refer\n to Velodyne usage manual)\n  - mrpt::obs::CObservationGPS for GPS (GPRMC) packets, if available via the\n synchronization interface of the device.\n  See those classes for documentation on their fields.\n\n Configuration includes setting the device IP (optional) and sensor model\n (mandatory only if a calibration file is not provided).\n These parameters can be set programmatically (see methods of this class), or\n via a configuration file with CGenericSensor::loadConfig() (see example\n config file section below).\n\n About timestamps:\n  Each gathered observation of type mrpt::obs::CObservationVelodyneScan is\n populated with two timestamps, one for the local PC timestamp and,\n  if available, another one for the GPS-stamped timestamp. Refer to the\n observation docs for details.\n\n Format of parameters for loading from a .ini file\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n Copyright notice\n Portions of this class are based on code from velodyne ROS node in\n https://github.com/ros-drivers/velodyne\n  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson\n  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin\n  License: Modified BSD Software License Agreement\n\n \n New in MRPT 1.4.0\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CVelodyneScanner(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CVelodyneScanner(); } ) );

		pybind11::enum_<mrpt::hwdrivers::CVelodyneScanner::model_t>(cl, "model_t", pybind11::arithmetic(), "LIDAR model types ")
			.value("VLP16", mrpt::hwdrivers::CVelodyneScanner::VLP16)
			.value("HDL32", mrpt::hwdrivers::CVelodyneScanner::HDL32)
			.value("HDL64", mrpt::hwdrivers::CVelodyneScanner::HDL64)
			.export_values();


		pybind11::enum_<mrpt::hwdrivers::CVelodyneScanner::return_type_t>(cl, "return_type_t", pybind11::arithmetic(), "LIDAR return type ")
			.value("UNCHANGED", mrpt::hwdrivers::CVelodyneScanner::UNCHANGED)
			.value("STRONGEST", mrpt::hwdrivers::CVelodyneScanner::STRONGEST)
			.value("LAST", mrpt::hwdrivers::CVelodyneScanner::LAST)
			.value("DUAL", mrpt::hwdrivers::CVelodyneScanner::DUAL)
			.export_values();

		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CVelodyneScanner::*)() const) &mrpt::hwdrivers::CVelodyneScanner::GetRuntimeClass, "C++: mrpt::hwdrivers::CVelodyneScanner::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CVelodyneScanner::CreateObject, "C++: mrpt::hwdrivers::CVelodyneScanner::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CVelodyneScanner::doRegister, "C++: mrpt::hwdrivers::CVelodyneScanner::doRegister() --> void");
		cl.def("setModelName", (void (mrpt::hwdrivers::CVelodyneScanner::*)(const enum mrpt::hwdrivers::CVelodyneScanner::model_t)) &mrpt::hwdrivers::CVelodyneScanner::setModelName, "see above for the list of parameters and their meaning\n @{ \n\n See supported model names in the general discussion docs for\n mrpt::hwdrivers::CVelodyneScanner \n\nC++: mrpt::hwdrivers::CVelodyneScanner::setModelName(const enum mrpt::hwdrivers::CVelodyneScanner::model_t) --> void", pybind11::arg("model"));
		cl.def("getModelName", (enum mrpt::hwdrivers::CVelodyneScanner::model_t (mrpt::hwdrivers::CVelodyneScanner::*)() const) &mrpt::hwdrivers::CVelodyneScanner::getModelName, "C++: mrpt::hwdrivers::CVelodyneScanner::getModelName() const --> enum mrpt::hwdrivers::CVelodyneScanner::model_t");
		cl.def("setPosPacketsMinPeriod", (void (mrpt::hwdrivers::CVelodyneScanner::*)(double)) &mrpt::hwdrivers::CVelodyneScanner::setPosPacketsMinPeriod, "Set the minimum period between the generation of\n mrpt::obs::CObservationGPS observations from Velodyne Position RMC GPS\n packets \n\nC++: mrpt::hwdrivers::CVelodyneScanner::setPosPacketsMinPeriod(double) --> void", pybind11::arg("period_seconds"));
		cl.def("getPosPacketsMinPeriod", (double (mrpt::hwdrivers::CVelodyneScanner::*)() const) &mrpt::hwdrivers::CVelodyneScanner::getPosPacketsMinPeriod, "C++: mrpt::hwdrivers::CVelodyneScanner::getPosPacketsMinPeriod() const --> double");
		cl.def("setPosPacketsTimingTimeout", (void (mrpt::hwdrivers::CVelodyneScanner::*)(double)) &mrpt::hwdrivers::CVelodyneScanner::setPosPacketsTimingTimeout, "Set how long to wait, after loss of GPS signal, to report timestamps as\n \"not based on satellite time\". 30 secs, with typical velodyne clock\n drifts, means a ~1.7 ms typical drift. \n\nC++: mrpt::hwdrivers::CVelodyneScanner::setPosPacketsTimingTimeout(double) --> void", pybind11::arg("timeout"));
		cl.def("getPosPacketsTimingTimeout", (double (mrpt::hwdrivers::CVelodyneScanner::*)() const) &mrpt::hwdrivers::CVelodyneScanner::getPosPacketsTimingTimeout, "C++: mrpt::hwdrivers::CVelodyneScanner::getPosPacketsTimingTimeout() const --> double");
		cl.def("setDeviceIP", (void (mrpt::hwdrivers::CVelodyneScanner::*)(const std::string &)) &mrpt::hwdrivers::CVelodyneScanner::setDeviceIP, "UDP packets from other IPs will be ignored. Default: empty string, means\n do not filter by IP \n\nC++: mrpt::hwdrivers::CVelodyneScanner::setDeviceIP(const std::string &) --> void", pybind11::arg("ip"));
		cl.def("getDeviceIP", (const std::string & (mrpt::hwdrivers::CVelodyneScanner::*)() const) &mrpt::hwdrivers::CVelodyneScanner::getDeviceIP, "C++: mrpt::hwdrivers::CVelodyneScanner::getDeviceIP() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setPCAPVerbosity", (void (mrpt::hwdrivers::CVelodyneScanner::*)(const bool)) &mrpt::hwdrivers::CVelodyneScanner::setPCAPVerbosity, "Enables/disables PCAP info messages to console (default: true) \n\nC++: mrpt::hwdrivers::CVelodyneScanner::setPCAPVerbosity(const bool) --> void", pybind11::arg("verbose"));
		cl.def("setPCAPInputFile", (void (mrpt::hwdrivers::CVelodyneScanner::*)(const std::string &)) &mrpt::hwdrivers::CVelodyneScanner::setPCAPInputFile, "Enables reading from a PCAP file instead of live UDP packet listening \n\nC++: mrpt::hwdrivers::CVelodyneScanner::setPCAPInputFile(const std::string &) --> void", pybind11::arg("pcap_file"));
		cl.def("getPCAPInputFile", (const std::string & (mrpt::hwdrivers::CVelodyneScanner::*)() const) &mrpt::hwdrivers::CVelodyneScanner::getPCAPInputFile, "C++: mrpt::hwdrivers::CVelodyneScanner::getPCAPInputFile() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setPCAPOutputFile", (void (mrpt::hwdrivers::CVelodyneScanner::*)(const std::string &)) &mrpt::hwdrivers::CVelodyneScanner::setPCAPOutputFile, "Enables dumping to a PCAP file in parallel to returning regular MRPT\n objects. Default=\"\": no pcap log. \n\nC++: mrpt::hwdrivers::CVelodyneScanner::setPCAPOutputFile(const std::string &) --> void", pybind11::arg("out_pcap_file"));
		cl.def("getPCAPOutputFile", (const std::string & (mrpt::hwdrivers::CVelodyneScanner::*)() const) &mrpt::hwdrivers::CVelodyneScanner::getPCAPOutputFile, "C++: mrpt::hwdrivers::CVelodyneScanner::getPCAPOutputFile() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setPCAPInputFileReadOnce", (void (mrpt::hwdrivers::CVelodyneScanner::*)(bool)) &mrpt::hwdrivers::CVelodyneScanner::setPCAPInputFileReadOnce, "C++: mrpt::hwdrivers::CVelodyneScanner::setPCAPInputFileReadOnce(bool) --> void", pybind11::arg("read_once"));
		cl.def("getPCAPInputFileReadOnce", (bool (mrpt::hwdrivers::CVelodyneScanner::*)() const) &mrpt::hwdrivers::CVelodyneScanner::getPCAPInputFileReadOnce, "C++: mrpt::hwdrivers::CVelodyneScanner::getPCAPInputFileReadOnce() const --> bool");
		cl.def("getCalibration", (const struct mrpt::obs::VelodyneCalibration & (mrpt::hwdrivers::CVelodyneScanner::*)() const) &mrpt::hwdrivers::CVelodyneScanner::getCalibration, "C++: mrpt::hwdrivers::CVelodyneScanner::getCalibration() const --> const struct mrpt::obs::VelodyneCalibration &", pybind11::return_value_policy::automatic);
		cl.def("setCalibration", (void (mrpt::hwdrivers::CVelodyneScanner::*)(const struct mrpt::obs::VelodyneCalibration &)) &mrpt::hwdrivers::CVelodyneScanner::setCalibration, "C++: mrpt::hwdrivers::CVelodyneScanner::setCalibration(const struct mrpt::obs::VelodyneCalibration &) --> void", pybind11::arg("calib"));
		cl.def("loadCalibrationFile", (bool (mrpt::hwdrivers::CVelodyneScanner::*)(const std::string &)) &mrpt::hwdrivers::CVelodyneScanner::loadCalibrationFile, "Returns false on error. \n\n mrpt::obs::VelodyneCalibration::loadFromXMLFile() \n\nC++: mrpt::hwdrivers::CVelodyneScanner::loadCalibrationFile(const std::string &) --> bool", pybind11::arg("velodyne_xml_calib_file_path"));
		cl.def("setLidarReturnType", (bool (mrpt::hwdrivers::CVelodyneScanner::*)(enum mrpt::hwdrivers::CVelodyneScanner::return_type_t)) &mrpt::hwdrivers::CVelodyneScanner::setLidarReturnType, "Changes among STRONGEST, LAST, DUAL return types (via HTTP post\n interface).\n Can be called at any instant, before or after initialize().\n Requires setting a device IP address.\n \n\n false on error \n\nC++: mrpt::hwdrivers::CVelodyneScanner::setLidarReturnType(enum mrpt::hwdrivers::CVelodyneScanner::return_type_t) --> bool", pybind11::arg("ret_type"));
		cl.def("setLidarRPM", (bool (mrpt::hwdrivers::CVelodyneScanner::*)(int)) &mrpt::hwdrivers::CVelodyneScanner::setLidarRPM, "Changes Lidar RPM (valid range: 300-600) (via HTTP post interface).\n Can be called at any instant, before or after initialize().\n Requires setting a device IP address.\n \n\n false on error\n\nC++: mrpt::hwdrivers::CVelodyneScanner::setLidarRPM(int) --> bool", pybind11::arg("rpm"));
		cl.def("setLidarOnOff", (bool (mrpt::hwdrivers::CVelodyneScanner::*)(bool)) &mrpt::hwdrivers::CVelodyneScanner::setLidarOnOff, "Switches the LASER on/off (saves energy when not measuring) (via HTTP\n post interface).\n Can be called at any instant, before or after initialize().\n Requires setting a device IP address.\n \n\n false on error\n\nC++: mrpt::hwdrivers::CVelodyneScanner::setLidarOnOff(bool) --> bool", pybind11::arg("on"));
		cl.def("setFramePublishing", (void (mrpt::hwdrivers::CVelodyneScanner::*)(bool)) &mrpt::hwdrivers::CVelodyneScanner::setFramePublishing, "Switches whole frame (points in a single revolution) on/off publication\n to data packet publication. When on, getNextObservation() will return\n true whenever a frame is avaliable, when off, getNextObservation() will\n return true whenever a data packet is avaliable. The default is on. When\n listening to data packets on a PCAP, pcap_read_fast is enforced.\n\nC++: mrpt::hwdrivers::CVelodyneScanner::setFramePublishing(bool) --> void", pybind11::arg("on"));
		cl.def("getNextObservation", (bool (mrpt::hwdrivers::CVelodyneScanner::*)(class std::shared_ptr<class mrpt::obs::CObservationVelodyneScan> &, class std::shared_ptr<class mrpt::obs::CObservationGPS> &)) &mrpt::hwdrivers::CVelodyneScanner::getNextObservation, "Polls the UDP port for incoming data packets. The user *must* call this\n method in a timely fashion to grab data as it it generated by the device.\n  The minimum call rate should be the expected number of data\n packets/second (!=scans/second). Checkout Velodyne user manual if in\n doubt.\n\n \n Upon return, an empty smart pointer will be found\n here if no new data was available. Otherwise, a valid scan.\n \n\n  Upon return, an empty smart pointer will be found\n here if no new GPS data was available. Otherwise, a valid GPS reading.\n \n\n true if no error ocurred (even if there was no new observation).\n false if any communication error occurred.\n\nC++: mrpt::hwdrivers::CVelodyneScanner::getNextObservation(class std::shared_ptr<class mrpt::obs::CObservationVelodyneScan> &, class std::shared_ptr<class mrpt::obs::CObservationGPS> &) --> bool", pybind11::arg("outScan"), pybind11::arg("outGPS"));
		cl.def("doProcess", (void (mrpt::hwdrivers::CVelodyneScanner::*)()) &mrpt::hwdrivers::CVelodyneScanner::doProcess, "C++: mrpt::hwdrivers::CVelodyneScanner::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CVelodyneScanner::*)()) &mrpt::hwdrivers::CVelodyneScanner::initialize, "Tries to initialize the sensor driver, after setting all the parameters\n with a call to loadConfig.\n Velodyne specifics: this method sets up the UDP listening sockets, so\n all relevant params MUST BE SET BEFORE calling this.\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CVelodyneScanner::initialize() --> void");
		cl.def("close", (void (mrpt::hwdrivers::CVelodyneScanner::*)()) &mrpt::hwdrivers::CVelodyneScanner::close, "Close the UDP sockets set-up in  This is called\n automatically upon destruction \n\nC++: mrpt::hwdrivers::CVelodyneScanner::close() --> void");
		cl.def("receivePackets", (bool (mrpt::hwdrivers::CVelodyneScanner::*)(mrpt::Clock::time_point &, struct mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket &, mrpt::Clock::time_point &, struct mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket &)) &mrpt::hwdrivers::CVelodyneScanner::receivePackets, "Users normally would prefer calling  instead.\n This method polls the UDP data port and returns one Velodyne DATA packet\n (1206 bytes) and/or one POSITION packet. Refer to Velodyne users manual.\n Approximate timestamps (based on this computer clock) are returned for\n each kind of packets, or INVALID_TIMESTAMP if timeout ocurred waiting for\n a packet.\n \n\n true on all ok. false only for pcap reading EOF\n\nC++: mrpt::hwdrivers::CVelodyneScanner::receivePackets(mrpt::Clock::time_point &, struct mrpt::obs::CObservationVelodyneScan::TVelodyneRawPacket &, mrpt::Clock::time_point &, struct mrpt::obs::CObservationVelodyneScan::TVelodynePositionPacket &) --> bool", pybind11::arg("data_pkt_timestamp"), pybind11::arg("out_data_pkt"), pybind11::arg("pos_pkt_timestamp"), pybind11::arg("out_pos_pkt"));

		{ // mrpt::hwdrivers::CVelodyneScanner::TModelProperties file:mrpt/hwdrivers/CVelodyneScanner.h line:192
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::hwdrivers::CVelodyneScanner::TModelProperties, std::shared_ptr<mrpt::hwdrivers::CVelodyneScanner::TModelProperties>> cl(enclosing_class, "TModelProperties", "Hard-wired properties of LIDARs depending on the model ");
			cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CVelodyneScanner::TModelProperties(); } ) );
			cl.def_readwrite("maxRange", &mrpt::hwdrivers::CVelodyneScanner::TModelProperties::maxRange);
		}

		{ // mrpt::hwdrivers::CVelodyneScanner::TModelPropertiesFactory file:mrpt/hwdrivers/CVelodyneScanner.h line:198
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::hwdrivers::CVelodyneScanner::TModelPropertiesFactory, std::shared_ptr<mrpt::hwdrivers::CVelodyneScanner::TModelPropertiesFactory>> cl(enclosing_class, "TModelPropertiesFactory", "Access to default sets of parameters for Velodyne LIDARs ");
			cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CVelodyneScanner::TModelPropertiesFactory(); } ) );
			cl.def_static("getListKnownModels", (std::string (*)()) &mrpt::hwdrivers::CVelodyneScanner::TModelPropertiesFactory::getListKnownModels, "Return human-readable string: \"`VLP16`,`XXX`,...\" \n\nC++: mrpt::hwdrivers::CVelodyneScanner::TModelPropertiesFactory::getListKnownModels() --> std::string");
		}

	}
}
