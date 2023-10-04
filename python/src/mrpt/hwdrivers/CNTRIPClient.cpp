#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGPS_NTRIP.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CNTRIPClient.h>
#include <mrpt/hwdrivers/CNTRIPEmitter.h>
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

// mrpt::hwdrivers::CNTRIPEmitter file:mrpt/hwdrivers/CNTRIPEmitter.h line:62
struct PyCallBack_mrpt_hwdrivers_CNTRIPEmitter : public mrpt::hwdrivers::CNTRIPEmitter {
	using mrpt::hwdrivers::CNTRIPEmitter::CNTRIPEmitter;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CNTRIPEmitter::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNTRIPEmitter::loadConfig_sensorSpecific(a0, a1);
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNTRIPEmitter::initialize();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CNTRIPEmitter::doProcess();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CNTRIPEmitter *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CGPS_NTRIP file:mrpt/hwdrivers/CGPS_NTRIP.h line:74
struct PyCallBack_mrpt_hwdrivers_CGPS_NTRIP : public mrpt::hwdrivers::CGPS_NTRIP {
	using mrpt::hwdrivers::CGPS_NTRIP::CGPS_NTRIP;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CGPS_NTRIP::GetRuntimeClass();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGPS_NTRIP::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGPS_NTRIP::initialize();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGPS_NTRIP::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGPS_NTRIP *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CNTRIPClient(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CNTRIPClient file:mrpt/hwdrivers/CNTRIPClient.h line:36
		pybind11::class_<mrpt::hwdrivers::CNTRIPClient, std::shared_ptr<mrpt::hwdrivers::CNTRIPClient>> cl(M("mrpt::hwdrivers"), "CNTRIPClient", "A client for NTRIP (HTTP) sources of differential GPS corrections from\ninternet servers, or Global navigation satellite system (GNSS) internet\nradio.\n  Usage:\n		- To open the server, invoke \"open\" with the proper parameters. Then use\n\"stream_data\" to read the read data.\n		- To obtain a list of all the mountpoints available at a given NTRIP\nCaster, call \"retrieveListOfMountpoints\" (it's a static method).\n\n  It is not neccesary to call \"close\", the connection is ended at\ndestruction.\n\n \n For a good reference of the NTRIP protocol, see\nhttp://gnss.itacyl.es/opencms/opencms/system/modules/es.jcyl.ita.site.gnss/resources/documentos_gnss/NtripDocumentation.pdf\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNTRIPClient(); } ) );
		cl.def("open", (bool (mrpt::hwdrivers::CNTRIPClient::*)(const struct mrpt::hwdrivers::CNTRIPClient::NTRIPArgs &, std::string &)) &mrpt::hwdrivers::CNTRIPClient::open, "Tries to open a given NTRIP stream and, if successful, launches a thread\n for continuously reading from it.\n \n\n close, stream_data\n\n \n false On any kind of error, with a description of the error in\n errmsg, if provided.\n\nC++: mrpt::hwdrivers::CNTRIPClient::open(const struct mrpt::hwdrivers::CNTRIPClient::NTRIPArgs &, std::string &) --> bool", pybind11::arg("params"), pybind11::arg("out_errmsg"));
		cl.def("close", (void (mrpt::hwdrivers::CNTRIPClient::*)()) &mrpt::hwdrivers::CNTRIPClient::close, "Closes the connection.\n \n\n open\n\nC++: mrpt::hwdrivers::CNTRIPClient::close() --> void");
		cl.def("sendBackToServer", (void (mrpt::hwdrivers::CNTRIPClient::*)(const std::string &)) &mrpt::hwdrivers::CNTRIPClient::sendBackToServer, "Enqueues a string to be sent back to the NTRIP server (e.g. GGA frames)\n\nC++: mrpt::hwdrivers::CNTRIPClient::sendBackToServer(const std::string &) --> void", pybind11::arg("data"));

		{ // mrpt::hwdrivers::CNTRIPClient::TMountPoint file:mrpt/hwdrivers/CNTRIPClient.h line:42
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::hwdrivers::CNTRIPClient::TMountPoint, std::shared_ptr<mrpt::hwdrivers::CNTRIPClient::TMountPoint>> cl(enclosing_class, "TMountPoint", "A descriptor of one stream in an NTRIP Caster - See\n CNTRIPClient::retrieveListOfMountpoints");
			cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNTRIPClient::TMountPoint(); } ) );
			cl.def( pybind11::init( [](mrpt::hwdrivers::CNTRIPClient::TMountPoint const &o){ return new mrpt::hwdrivers::CNTRIPClient::TMountPoint(o); } ) );
			cl.def_readwrite("mountpoint_name", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::mountpoint_name);
			cl.def_readwrite("id", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::id);
			cl.def_readwrite("format", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::format);
			cl.def_readwrite("format_details", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::format_details);
			cl.def_readwrite("carrier", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::carrier);
			cl.def_readwrite("nav_system", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::nav_system);
			cl.def_readwrite("network", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::network);
			cl.def_readwrite("country_code", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::country_code);
			cl.def_readwrite("latitude", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::latitude);
			cl.def_readwrite("longitude", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::longitude);
			cl.def_readwrite("needs_nmea", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::needs_nmea);
			cl.def_readwrite("net_ref_stations", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::net_ref_stations);
			cl.def_readwrite("generator_model", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::generator_model);
			cl.def_readwrite("compr_encryp", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::compr_encryp);
			cl.def_readwrite("authentication", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::authentication);
			cl.def_readwrite("pay_service", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::pay_service);
			cl.def_readwrite("stream_bitspersec", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::stream_bitspersec);
			cl.def_readwrite("extra_info", &mrpt::hwdrivers::CNTRIPClient::TMountPoint::extra_info);
			cl.def("assign", (struct mrpt::hwdrivers::CNTRIPClient::TMountPoint & (mrpt::hwdrivers::CNTRIPClient::TMountPoint::*)(const struct mrpt::hwdrivers::CNTRIPClient::TMountPoint &)) &mrpt::hwdrivers::CNTRIPClient::TMountPoint::operator=, "C++: mrpt::hwdrivers::CNTRIPClient::TMountPoint::operator=(const struct mrpt::hwdrivers::CNTRIPClient::TMountPoint &) --> struct mrpt::hwdrivers::CNTRIPClient::TMountPoint &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::hwdrivers::CNTRIPClient::NTRIPArgs file:mrpt/hwdrivers/CNTRIPClient.h line:79
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::hwdrivers::CNTRIPClient::NTRIPArgs, std::shared_ptr<mrpt::hwdrivers::CNTRIPClient::NTRIPArgs>> cl(enclosing_class, "NTRIPArgs", "The arguments for connecting to a NTRIP stream, used in\n CNTRIPClient::open");
			cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNTRIPClient::NTRIPArgs(); } ) );
			cl.def( pybind11::init( [](mrpt::hwdrivers::CNTRIPClient::NTRIPArgs const &o){ return new mrpt::hwdrivers::CNTRIPClient::NTRIPArgs(o); } ) );
			cl.def_readwrite("server", &mrpt::hwdrivers::CNTRIPClient::NTRIPArgs::server);
			cl.def_readwrite("port", &mrpt::hwdrivers::CNTRIPClient::NTRIPArgs::port);
			cl.def_readwrite("user", &mrpt::hwdrivers::CNTRIPClient::NTRIPArgs::user);
			cl.def_readwrite("password", &mrpt::hwdrivers::CNTRIPClient::NTRIPArgs::password);
			cl.def_readwrite("mountpoint", &mrpt::hwdrivers::CNTRIPClient::NTRIPArgs::mountpoint);
			cl.def("assign", (struct mrpt::hwdrivers::CNTRIPClient::NTRIPArgs & (mrpt::hwdrivers::CNTRIPClient::NTRIPArgs::*)(const struct mrpt::hwdrivers::CNTRIPClient::NTRIPArgs &)) &mrpt::hwdrivers::CNTRIPClient::NTRIPArgs::operator=, "C++: mrpt::hwdrivers::CNTRIPClient::NTRIPArgs::operator=(const struct mrpt::hwdrivers::CNTRIPClient::NTRIPArgs &) --> struct mrpt::hwdrivers::CNTRIPClient::NTRIPArgs &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::hwdrivers::CNTRIPEmitter file:mrpt/hwdrivers/CNTRIPEmitter.h line:62
		pybind11::class_<mrpt::hwdrivers::CNTRIPEmitter, std::shared_ptr<mrpt::hwdrivers::CNTRIPEmitter>, PyCallBack_mrpt_hwdrivers_CNTRIPEmitter, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CNTRIPEmitter", "This \"virtual driver\" encapsulates a NTRIP client (see CNTRIPClient) but\n adds the functionality of dumping the received datastream to a given serial\n port.\n  Used within rawlog-grabber, along CGPSInterface, this class allows one to\n build a powerful & simple RTK-capable GPS receiver system.\n\n  Therefore, this sensor will never \"collect\" any observation via the\n CGenericSensor interface.\n\n   See also the example configuration file for rawlog-grabber in\n \"share/mrpt/config_files/rawlog-grabber\".\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  The next picture summarizes existing MRPT classes related to GPS / GNSS\n devices (CGPSInterface, CNTRIPEmitter, CGPS_NTRIP):\n\n    \n\n \n\n \n CGPSInterface, CGPS_NTRIP, CNTRIPClient");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CNTRIPEmitter(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CNTRIPEmitter(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CNTRIPEmitter::*)() const) &mrpt::hwdrivers::CNTRIPEmitter::GetRuntimeClass, "C++: mrpt::hwdrivers::CNTRIPEmitter::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CNTRIPEmitter::CreateObject, "C++: mrpt::hwdrivers::CNTRIPEmitter::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CNTRIPEmitter::doRegister, "C++: mrpt::hwdrivers::CNTRIPEmitter::doRegister() --> void");
		cl.def("setOutputSerialPort", (void (mrpt::hwdrivers::CNTRIPEmitter::*)(const std::string &)) &mrpt::hwdrivers::CNTRIPEmitter::setOutputSerialPort, "Changes the serial port to connect to (call prior to 'doProcess'), for\n example \"COM1\" or \"ttyS0\".\n  This is not needed if the configuration is loaded with \"loadConfig\".\n\nC++: mrpt::hwdrivers::CNTRIPEmitter::setOutputSerialPort(const std::string &) --> void", pybind11::arg("port"));
		cl.def("getOutputSerialPort", (std::string (mrpt::hwdrivers::CNTRIPEmitter::*)() const) &mrpt::hwdrivers::CNTRIPEmitter::getOutputSerialPort, "C++: mrpt::hwdrivers::CNTRIPEmitter::getOutputSerialPort() const --> std::string");
		cl.def("setRawOutputFilePrefix", (void (mrpt::hwdrivers::CNTRIPEmitter::*)(const std::string &)) &mrpt::hwdrivers::CNTRIPEmitter::setRawOutputFilePrefix, "C++: mrpt::hwdrivers::CNTRIPEmitter::setRawOutputFilePrefix(const std::string &) --> void", pybind11::arg("outfile"));
		cl.def("getRawOutputFilePrefix", (std::string (mrpt::hwdrivers::CNTRIPEmitter::*)() const) &mrpt::hwdrivers::CNTRIPEmitter::getRawOutputFilePrefix, "C++: mrpt::hwdrivers::CNTRIPEmitter::getRawOutputFilePrefix() const --> std::string");
		cl.def("initialize", (void (mrpt::hwdrivers::CNTRIPEmitter::*)()) &mrpt::hwdrivers::CNTRIPEmitter::initialize, "Set up the NTRIP communications, raising an exception on fatal errors.\n  Called automatically by rawlog-grabber.\n  If used manually, call after \"loadConfig\" and before \"doProcess\".\n\nC++: mrpt::hwdrivers::CNTRIPEmitter::initialize() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CNTRIPEmitter::*)()) &mrpt::hwdrivers::CNTRIPEmitter::doProcess, "The main loop, which must be called in a timely fashion in order to\n process the incomming NTRIP data stream and dump it to the serial port.\n  This method is called automatically when used within rawlog-grabber.\n\nC++: mrpt::hwdrivers::CNTRIPEmitter::doProcess() --> void");
		cl.def("getNTRIPClient", (class mrpt::hwdrivers::CNTRIPClient & (mrpt::hwdrivers::CNTRIPEmitter::*)()) &mrpt::hwdrivers::CNTRIPEmitter::getNTRIPClient, "Exposes the NTRIP client object \n\nC++: mrpt::hwdrivers::CNTRIPEmitter::getNTRIPClient() --> class mrpt::hwdrivers::CNTRIPClient &", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::hwdrivers::CGPS_NTRIP file:mrpt/hwdrivers/CGPS_NTRIP.h line:74
		pybind11::class_<mrpt::hwdrivers::CGPS_NTRIP, std::shared_ptr<mrpt::hwdrivers::CGPS_NTRIP>, PyCallBack_mrpt_hwdrivers_CGPS_NTRIP, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CGPS_NTRIP", "A combination of GPS receiver + NTRIP receiver capable of submitting GGA\n frames to enable RTCM 3.0.\n This class holds instances of two classes, publicly exposed as member\n variables:\n  - mrpt::hwdrivers::CGPSInterface  gps;\n  - mrpt::hwdrivers::CNTRIPEmitter  ntrip;\n\n and acts as a \"joint sensor\", calling both objects' doProcess() inside the\n doProcess() loop, etc.\n\n The goal of this class is automatically gather GGA frames from the gps\n sensor and upload them to the NTRIP server.\n\n Configuration file format is a combination of the original parameters for\n both classes, each with\n a prefix: `\"gps_` for CGPSInterface params and `ntrip_` for CNTRIPEmitter.\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  The next picture summarizes existing MRPT classes related to GPS / GNSS\n devices (CGPSInterface, CNTRIPEmitter, CGPS_NTRIP):\n\n    \n\n  \n Verbose debug info will be dumped to cout if the environment variable\n \"MRPT_HWDRIVERS_VERBOSE\" is set to \"1\", or if you call\n CGenericSensor::enableVerbose(true)\n\n \n\n \n CGPSInterface, CNTRIPEmitter");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CGPS_NTRIP(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CGPS_NTRIP(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CGPS_NTRIP::*)() const) &mrpt::hwdrivers::CGPS_NTRIP::GetRuntimeClass, "C++: mrpt::hwdrivers::CGPS_NTRIP::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CGPS_NTRIP::CreateObject, "C++: mrpt::hwdrivers::CGPS_NTRIP::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CGPS_NTRIP::doRegister, "C++: mrpt::hwdrivers::CGPS_NTRIP::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CGPS_NTRIP::*)()) &mrpt::hwdrivers::CGPS_NTRIP::doProcess, "C++: mrpt::hwdrivers::CGPS_NTRIP::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CGPS_NTRIP::*)()) &mrpt::hwdrivers::CGPS_NTRIP::initialize, "C++: mrpt::hwdrivers::CGPS_NTRIP::initialize() --> void");
	}
}
