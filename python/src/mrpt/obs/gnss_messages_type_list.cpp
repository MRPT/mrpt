#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/obs/gnss_messages.h>
#include <mrpt/obs/gnss_messages_type_list.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <variant>

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

// mrpt::obs::gnss::Message_NMEA_GGA file: line:19
struct PyCallBack_mrpt_obs_gnss_Message_NMEA_GGA : public mrpt::obs::gnss::Message_NMEA_GGA {
	using mrpt::obs::gnss::Message_NMEA_GGA::Message_NMEA_GGA;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_GGA *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_GGA::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_GGA *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_GGA::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_GGA *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return gnss_message::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NMEA_GLL file: line:101
struct PyCallBack_mrpt_obs_gnss_Message_NMEA_GLL : public mrpt::obs::gnss::Message_NMEA_GLL {
	using mrpt::obs::gnss::Message_NMEA_GLL::Message_NMEA_GLL;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_GLL *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_GLL::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_GLL *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_GLL::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_GLL *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return gnss_message::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NMEA_RMC file: line:130
struct PyCallBack_mrpt_obs_gnss_Message_NMEA_RMC : public mrpt::obs::gnss::Message_NMEA_RMC {
	using mrpt::obs::gnss::Message_NMEA_RMC::Message_NMEA_RMC;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_RMC *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_RMC::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_RMC *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_RMC::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_RMC *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return gnss_message::fixEndianness();
	}
};

void bind_mrpt_obs_gnss_messages_type_list(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::obs::gnss::gnss_message_type_t file:mrpt/obs/gnss_messages_type_list.h line:22
	pybind11::enum_<mrpt::obs::gnss::gnss_message_type_t>(M("mrpt::obs::gnss"), "gnss_message_type_t", pybind11::arithmetic(), "List of all known GNSS message types.\n Normally, each type here has a corresponding class, derived from\n mrpt::obs::gnss::gnss_message,\n that stores the message data, but some classes may be still in the \"TO-DO\"\n list or just not needed in practice.\n On the other hand,  message classes  be associated with one and\n only one value from this list.\n \n\n mrpt::obs::CObservationGPS, mrpt::obs::gnss::gnss_message")
		.value("NMEA_GGA", mrpt::obs::gnss::NMEA_GGA)
		.value("NMEA_GLL", mrpt::obs::gnss::NMEA_GLL)
		.value("NMEA_GSA", mrpt::obs::gnss::NMEA_GSA)
		.value("NMEA_GSV", mrpt::obs::gnss::NMEA_GSV)
		.value("NMEA_MSS", mrpt::obs::gnss::NMEA_MSS)
		.value("NMEA_RMC", mrpt::obs::gnss::NMEA_RMC)
		.value("NMEA_VTG", mrpt::obs::gnss::NMEA_VTG)
		.value("NMEA_ZDA", mrpt::obs::gnss::NMEA_ZDA)
		.value("TOPCON_PZS", mrpt::obs::gnss::TOPCON_PZS)
		.value("TOPCON_SATS", mrpt::obs::gnss::TOPCON_SATS)
		.value("NV_OEM6_MSG2ENUM", mrpt::obs::gnss::NV_OEM6_MSG2ENUM)
		.value("NV_OEM6_GENERIC_FRAME", mrpt::obs::gnss::NV_OEM6_GENERIC_FRAME)
		.value("NV_OEM6_GENERIC_SHORT_FRAME", mrpt::obs::gnss::NV_OEM6_GENERIC_SHORT_FRAME)
		.value("NV_OEM6_ALIGNBSLNENU", mrpt::obs::gnss::NV_OEM6_ALIGNBSLNENU)
		.value("NV_OEM6_ALIGNBSLNXYZ", mrpt::obs::gnss::NV_OEM6_ALIGNBSLNXYZ)
		.value("NV_OEM6_ALIGNDOP", mrpt::obs::gnss::NV_OEM6_ALIGNDOP)
		.value("NV_OEM6_BESTPOS", mrpt::obs::gnss::NV_OEM6_BESTPOS)
		.value("NV_OEM6_BESTSATS", mrpt::obs::gnss::NV_OEM6_BESTSATS)
		.value("NV_OEM6_BESTUTM", mrpt::obs::gnss::NV_OEM6_BESTUTM)
		.value("NV_OEM6_BESTVEL", mrpt::obs::gnss::NV_OEM6_BESTVEL)
		.value("NV_OEM6_BESTXYZ", mrpt::obs::gnss::NV_OEM6_BESTXYZ)
		.value("NV_OEM6_CLOCKSTEERING", mrpt::obs::gnss::NV_OEM6_CLOCKSTEERING)
		.value("NV_OEM6_GPGLL", mrpt::obs::gnss::NV_OEM6_GPGLL)
		.value("NV_OEM6_GPGGA", mrpt::obs::gnss::NV_OEM6_GPGGA)
		.value("NV_OEM6_GPGGARTK", mrpt::obs::gnss::NV_OEM6_GPGGARTK)
		.value("NV_OEM6_GPGSA", mrpt::obs::gnss::NV_OEM6_GPGSA)
		.value("NV_OEM6_GPGSV", mrpt::obs::gnss::NV_OEM6_GPGSV)
		.value("NV_OEM6_GPHDT", mrpt::obs::gnss::NV_OEM6_GPHDT)
		.value("NV_OEM6_GPRMC", mrpt::obs::gnss::NV_OEM6_GPRMC)
		.value("NV_OEM6_GPVTG", mrpt::obs::gnss::NV_OEM6_GPVTG)
		.value("NV_OEM6_GPZDA", mrpt::obs::gnss::NV_OEM6_GPZDA)
		.value("NV_OEM6_IONUTC", mrpt::obs::gnss::NV_OEM6_IONUTC)
		.value("NV_OEM6_MARKPOS", mrpt::obs::gnss::NV_OEM6_MARKPOS)
		.value("NV_OEM6_MARK2POS", mrpt::obs::gnss::NV_OEM6_MARK2POS)
		.value("NV_OEM6_MARKTIME", mrpt::obs::gnss::NV_OEM6_MARKTIME)
		.value("NV_OEM6_MARK2TIME", mrpt::obs::gnss::NV_OEM6_MARK2TIME)
		.value("NV_OEM6_PPPPOS", mrpt::obs::gnss::NV_OEM6_PPPPOS)
		.value("NV_OEM6_RANGECMP", mrpt::obs::gnss::NV_OEM6_RANGECMP)
		.value("NV_OEM6_RAWEPHEM", mrpt::obs::gnss::NV_OEM6_RAWEPHEM)
		.value("NV_OEM6_RXSTATUS", mrpt::obs::gnss::NV_OEM6_RXSTATUS)
		.value("NV_OEM6_VERSION", mrpt::obs::gnss::NV_OEM6_VERSION)
		.value("NV_OEM6_INSPVAS", mrpt::obs::gnss::NV_OEM6_INSPVAS)
		.value("NV_OEM6_INSATTS", mrpt::obs::gnss::NV_OEM6_INSATTS)
		.value("NV_OEM6_INSCOVS", mrpt::obs::gnss::NV_OEM6_INSCOVS)
		.value("NV_OEM6_INSVELS", mrpt::obs::gnss::NV_OEM6_INSVELS)
		.value("NV_OEM6_RAWIMUS", mrpt::obs::gnss::NV_OEM6_RAWIMUS)
		.export_values();

;

	{ // mrpt::obs::gnss::gnss_message file: line:27
		pybind11::class_<mrpt::obs::gnss::gnss_message, std::shared_ptr<mrpt::obs::gnss::gnss_message>> cl(M("mrpt::obs::gnss"), "gnss_message", "Pure virtual base for all message types. \n mrpt::obs::CObservationGPS  ");
		cl.def_readwrite("message_type", &mrpt::obs::gnss::gnss_message::message_type);
		cl.def("writeToStream", (void (mrpt::obs::gnss::gnss_message::*)(class mrpt::serialization::CArchive &) const) &mrpt::obs::gnss::gnss_message::writeToStream, "Save to binary stream. Launches an exception upon error \n\nC++: mrpt::obs::gnss::gnss_message::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
		cl.def("readFromStream", (void (mrpt::obs::gnss::gnss_message::*)(class mrpt::serialization::CArchive &)) &mrpt::obs::gnss::gnss_message::readFromStream, "Load from binary stream into this existing object. Launches an exception\n upon error. \n\nC++: mrpt::obs::gnss::gnss_message::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
		cl.def("isOfType", (bool (mrpt::obs::gnss::gnss_message::*)(const enum mrpt::obs::gnss::gnss_message_type_t) const) &mrpt::obs::gnss::gnss_message::isOfType, "C++: mrpt::obs::gnss::gnss_message::isOfType(const enum mrpt::obs::gnss::gnss_message_type_t) const --> bool", pybind11::arg("type_id"));
		cl.def_static("readAndBuildFromStream", (struct mrpt::obs::gnss::gnss_message * (*)(class mrpt::serialization::CArchive &)) &mrpt::obs::gnss::gnss_message::readAndBuildFromStream, "Load from binary stream and creates object detecting its type (class\n factory). Launches an exception upon error \n\nC++: mrpt::obs::gnss::gnss_message::readAndBuildFromStream(class mrpt::serialization::CArchive &) --> struct mrpt::obs::gnss::gnss_message *", pybind11::return_value_policy::automatic, pybind11::arg("in"));
		cl.def_static("Factory", (struct mrpt::obs::gnss::gnss_message * (*)(const enum mrpt::obs::gnss::gnss_message_type_t)) &mrpt::obs::gnss::gnss_message::Factory, "Creates message \n nullptr on unknown msg type \n\nC++: mrpt::obs::gnss::gnss_message::Factory(const enum mrpt::obs::gnss::gnss_message_type_t) --> struct mrpt::obs::gnss::gnss_message *", pybind11::return_value_policy::automatic, pybind11::arg("msg_id"));
		cl.def_static("FactoryKnowsMsgType", (bool (*)(const enum mrpt::obs::gnss::gnss_message_type_t)) &mrpt::obs::gnss::gnss_message::FactoryKnowsMsgType, "Returns true if Factory() has a registered constructor for this msg type\n\nC++: mrpt::obs::gnss::gnss_message::FactoryKnowsMsgType(const enum mrpt::obs::gnss::gnss_message_type_t) --> bool", pybind11::arg("msg_id"));
		cl.def("fixEndianness", (void (mrpt::obs::gnss::gnss_message::*)()) &mrpt::obs::gnss::gnss_message::fixEndianness, "If we are in a big endian system, reverse all fields >1 byte to fix its\n representation. Only in binary frames, text-based derived classes\n obviously do not need to reimplement this one. \n\nC++: mrpt::obs::gnss::gnss_message::fixEndianness() --> void");
		cl.def("getMessageTypeAsString", (const std::string & (mrpt::obs::gnss::gnss_message::*)() const) &mrpt::obs::gnss::gnss_message::getMessageTypeAsString, "Returns \"NMEA_GGA\", etc. \n\nC++: mrpt::obs::gnss::gnss_message::getMessageTypeAsString() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("assign", (struct mrpt::obs::gnss::gnss_message & (mrpt::obs::gnss::gnss_message::*)(const struct mrpt::obs::gnss::gnss_message &)) &mrpt::obs::gnss::gnss_message::operator=, "C++: mrpt::obs::gnss::gnss_message::operator=(const struct mrpt::obs::gnss::gnss_message &) --> struct mrpt::obs::gnss::gnss_message &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::gnss::UTC_time file: line:144
		pybind11::class_<mrpt::obs::gnss::UTC_time, std::shared_ptr<mrpt::obs::gnss::UTC_time>> cl(M("mrpt::obs::gnss"), "UTC_time", "UTC (Coordinated Universal Time) time-stamp structure for GPS messages. \n\n mrpt::obs::CObservationGPS ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::UTC_time(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::UTC_time const &o){ return new mrpt::obs::gnss::UTC_time(o); } ) );
		cl.def_readwrite("hour", &mrpt::obs::gnss::UTC_time::hour);
		cl.def_readwrite("minute", &mrpt::obs::gnss::UTC_time::minute);
		cl.def_readwrite("sec", &mrpt::obs::gnss::UTC_time::sec);
		cl.def("getAsTimestamp", (mrpt::Clock::time_point (mrpt::obs::gnss::UTC_time::*)(const mrpt::Clock::time_point &) const) &mrpt::obs::gnss::UTC_time::getAsTimestamp, "Build an MRPT timestamp with the hour/minute/sec of this structure and\n the date from the given timestamp. \n\nC++: mrpt::obs::gnss::UTC_time::getAsTimestamp(const mrpt::Clock::time_point &) const --> mrpt::Clock::time_point", pybind11::arg("date"));
		cl.def("__eq__", (bool (mrpt::obs::gnss::UTC_time::*)(const struct mrpt::obs::gnss::UTC_time &) const) &mrpt::obs::gnss::UTC_time::operator==, "C++: mrpt::obs::gnss::UTC_time::operator==(const struct mrpt::obs::gnss::UTC_time &) const --> bool", pybind11::arg("o"));
		cl.def("__ne__", (bool (mrpt::obs::gnss::UTC_time::*)(const struct mrpt::obs::gnss::UTC_time &) const) &mrpt::obs::gnss::UTC_time::operator!=, "C++: mrpt::obs::gnss::UTC_time::operator!=(const struct mrpt::obs::gnss::UTC_time &) const --> bool", pybind11::arg("o"));
		cl.def("writeToStream", (void (mrpt::obs::gnss::UTC_time::*)(class mrpt::serialization::CArchive &) const) &mrpt::obs::gnss::UTC_time::writeToStream, "Save to binary stream. Launches an exception upon error \n\nC++: mrpt::obs::gnss::UTC_time::writeToStream(class mrpt::serialization::CArchive &) const --> void", pybind11::arg("out"));
		cl.def("readFromStream", (void (mrpt::obs::gnss::UTC_time::*)(class mrpt::serialization::CArchive &)) &mrpt::obs::gnss::UTC_time::readFromStream, "Save to binary stream. Launches an exception upon error \n\nC++: mrpt::obs::gnss::UTC_time::readFromStream(class mrpt::serialization::CArchive &) --> void", pybind11::arg("in"));
		cl.def("assign", (struct mrpt::obs::gnss::UTC_time & (mrpt::obs::gnss::UTC_time::*)(const struct mrpt::obs::gnss::UTC_time &)) &mrpt::obs::gnss::UTC_time::operator=, "C++: mrpt::obs::gnss::UTC_time::operator=(const struct mrpt::obs::gnss::UTC_time &) --> struct mrpt::obs::gnss::UTC_time &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::gnss::Message_NMEA_GGA file: line:19
		pybind11::class_<mrpt::obs::gnss::Message_NMEA_GGA, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_GGA>, PyCallBack_mrpt_obs_gnss_Message_NMEA_GGA, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NMEA_GGA", "NMEA datum: GGA. \n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_GGA(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_GGA(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NMEA_GGA const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_GGA(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_GGA const &o){ return new mrpt::obs::gnss::Message_NMEA_GGA(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NMEA_GGA::fields);
		cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_GGA & (mrpt::obs::gnss::Message_NMEA_GGA::*)(const struct mrpt::obs::gnss::Message_NMEA_GGA &)) &mrpt::obs::gnss::Message_NMEA_GGA::operator=, "C++: mrpt::obs::gnss::Message_NMEA_GGA::operator=(const struct mrpt::obs::gnss::Message_NMEA_GGA &) --> struct mrpt::obs::gnss::Message_NMEA_GGA &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NMEA_GGA::content_t file: line:28
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NMEA_GGA::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_GGA::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_GGA::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_GGA::content_t const &o){ return new mrpt::obs::gnss::Message_NMEA_GGA::content_t(o); } ) );
			cl.def_readwrite("UTCTime", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::UTCTime);
			cl.def_readwrite("latitude_degrees", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::latitude_degrees);
			cl.def_readwrite("longitude_degrees", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::longitude_degrees);
			cl.def_readwrite("fix_quality", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::fix_quality);
			cl.def_readwrite("altitude_meters", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::altitude_meters);
			cl.def_readwrite("geoidal_distance", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::geoidal_distance);
			cl.def_readwrite("orthometric_altitude", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::orthometric_altitude);
			cl.def_readwrite("corrected_orthometric_altitude", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::corrected_orthometric_altitude);
			cl.def_readwrite("satellitesUsed", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::satellitesUsed);
			cl.def_readwrite("thereis_HDOP", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::thereis_HDOP);
			cl.def_readwrite("HDOP", &mrpt::obs::gnss::Message_NMEA_GGA::content_t::HDOP);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_GGA::content_t & (mrpt::obs::gnss::Message_NMEA_GGA::content_t::*)(const struct mrpt::obs::gnss::Message_NMEA_GGA::content_t &)) &mrpt::obs::gnss::Message_NMEA_GGA::content_t::operator=, "C++: mrpt::obs::gnss::Message_NMEA_GGA::content_t::operator=(const struct mrpt::obs::gnss::Message_NMEA_GGA::content_t &) --> struct mrpt::obs::gnss::Message_NMEA_GGA::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NMEA_GLL file: line:101
		pybind11::class_<mrpt::obs::gnss::Message_NMEA_GLL, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_GLL>, PyCallBack_mrpt_obs_gnss_Message_NMEA_GLL, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NMEA_GLL", "NMEA datum: GLL. \n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_GLL(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_GLL(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NMEA_GLL const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_GLL(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_GLL const &o){ return new mrpt::obs::gnss::Message_NMEA_GLL(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NMEA_GLL::fields);
		cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_GLL & (mrpt::obs::gnss::Message_NMEA_GLL::*)(const struct mrpt::obs::gnss::Message_NMEA_GLL &)) &mrpt::obs::gnss::Message_NMEA_GLL::operator=, "C++: mrpt::obs::gnss::Message_NMEA_GLL::operator=(const struct mrpt::obs::gnss::Message_NMEA_GLL &) --> struct mrpt::obs::gnss::Message_NMEA_GLL &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NMEA_GLL::content_t file: line:110
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NMEA_GLL::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_GLL::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_GLL::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_GLL::content_t const &o){ return new mrpt::obs::gnss::Message_NMEA_GLL::content_t(o); } ) );
			cl.def_readwrite("UTCTime", &mrpt::obs::gnss::Message_NMEA_GLL::content_t::UTCTime);
			cl.def_readwrite("latitude_degrees", &mrpt::obs::gnss::Message_NMEA_GLL::content_t::latitude_degrees);
			cl.def_readwrite("longitude_degrees", &mrpt::obs::gnss::Message_NMEA_GLL::content_t::longitude_degrees);
			cl.def_readwrite("validity_char", &mrpt::obs::gnss::Message_NMEA_GLL::content_t::validity_char);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_GLL::content_t & (mrpt::obs::gnss::Message_NMEA_GLL::content_t::*)(const struct mrpt::obs::gnss::Message_NMEA_GLL::content_t &)) &mrpt::obs::gnss::Message_NMEA_GLL::content_t::operator=, "C++: mrpt::obs::gnss::Message_NMEA_GLL::content_t::operator=(const struct mrpt::obs::gnss::Message_NMEA_GLL::content_t &) --> struct mrpt::obs::gnss::Message_NMEA_GLL::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NMEA_RMC file: line:130
		pybind11::class_<mrpt::obs::gnss::Message_NMEA_RMC, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_RMC>, PyCallBack_mrpt_obs_gnss_Message_NMEA_RMC, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NMEA_RMC", "NMEA datum: RMC. \n mrpt::obs::CObservationGPS   ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_RMC(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_RMC(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NMEA_RMC const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_RMC(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_RMC const &o){ return new mrpt::obs::gnss::Message_NMEA_RMC(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NMEA_RMC::fields);
		cl.def("getDateAsTimestamp", (mrpt::Clock::time_point (mrpt::obs::gnss::Message_NMEA_RMC::*)() const) &mrpt::obs::gnss::Message_NMEA_RMC::getDateAsTimestamp, "Build an MRPT timestamp with the year/month/day of this observation. \n\nC++: mrpt::obs::gnss::Message_NMEA_RMC::getDateAsTimestamp() const --> mrpt::Clock::time_point");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_RMC & (mrpt::obs::gnss::Message_NMEA_RMC::*)(const struct mrpt::obs::gnss::Message_NMEA_RMC &)) &mrpt::obs::gnss::Message_NMEA_RMC::operator=, "C++: mrpt::obs::gnss::Message_NMEA_RMC::operator=(const struct mrpt::obs::gnss::Message_NMEA_RMC &) --> struct mrpt::obs::gnss::Message_NMEA_RMC &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NMEA_RMC::content_t file: line:139
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NMEA_RMC::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_RMC::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_RMC::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_RMC::content_t const &o){ return new mrpt::obs::gnss::Message_NMEA_RMC::content_t(o); } ) );
			cl.def_readwrite("UTCTime", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::UTCTime);
			cl.def_readwrite("validity_char", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::validity_char);
			cl.def_readwrite("latitude_degrees", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::latitude_degrees);
			cl.def_readwrite("longitude_degrees", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::longitude_degrees);
			cl.def_readwrite("speed_knots", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::speed_knots);
			cl.def_readwrite("direction_degrees", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::direction_degrees);
			cl.def_readwrite("date_day", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::date_day);
			cl.def_readwrite("date_month", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::date_month);
			cl.def_readwrite("date_year", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::date_year);
			cl.def_readwrite("magnetic_dir", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::magnetic_dir);
			cl.def_readwrite("positioning_mode", &mrpt::obs::gnss::Message_NMEA_RMC::content_t::positioning_mode);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_RMC::content_t & (mrpt::obs::gnss::Message_NMEA_RMC::content_t::*)(const struct mrpt::obs::gnss::Message_NMEA_RMC::content_t &)) &mrpt::obs::gnss::Message_NMEA_RMC::content_t::operator=, "C++: mrpt::obs::gnss::Message_NMEA_RMC::content_t::operator=(const struct mrpt::obs::gnss::Message_NMEA_RMC::content_t &) --> struct mrpt::obs::gnss::Message_NMEA_RMC::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
