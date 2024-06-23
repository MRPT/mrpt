#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/obs/gnss_messages.h>
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

// mrpt::obs::gnss::Message_NMEA_VTG file: line:174
struct PyCallBack_mrpt_obs_gnss_Message_NMEA_VTG : public mrpt::obs::gnss::Message_NMEA_VTG {
	using mrpt::obs::gnss::Message_NMEA_VTG::Message_NMEA_VTG;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_VTG *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_VTG::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_VTG *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_VTG::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_VTG *>(this), "fixEndianness");
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

// mrpt::obs::gnss::Message_NMEA_GSA file: line:198
struct PyCallBack_mrpt_obs_gnss_Message_NMEA_GSA : public mrpt::obs::gnss::Message_NMEA_GSA {
	using mrpt::obs::gnss::Message_NMEA_GSA::Message_NMEA_GSA;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_GSA *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_GSA::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_GSA *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_GSA::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_GSA *>(this), "fixEndianness");
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

// mrpt::obs::gnss::Message_NMEA_ZDA file: line:224
struct PyCallBack_mrpt_obs_gnss_Message_NMEA_ZDA : public mrpt::obs::gnss::Message_NMEA_ZDA {
	using mrpt::obs::gnss::Message_NMEA_ZDA::Message_NMEA_ZDA;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_ZDA *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_ZDA::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_ZDA *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NMEA_ZDA::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NMEA_ZDA *>(this), "fixEndianness");
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

void bind_unknown_unknown_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::gnss::Message_NMEA_VTG file: line:174
		pybind11::class_<mrpt::obs::gnss::Message_NMEA_VTG, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_VTG>, PyCallBack_mrpt_obs_gnss_Message_NMEA_VTG, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NMEA_VTG", "NMEA datum: VTG. \n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_VTG(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_VTG(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NMEA_VTG const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_VTG(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_VTG const &o){ return new mrpt::obs::gnss::Message_NMEA_VTG(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NMEA_VTG::fields);
		cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_VTG & (mrpt::obs::gnss::Message_NMEA_VTG::*)(const struct mrpt::obs::gnss::Message_NMEA_VTG &)) &mrpt::obs::gnss::Message_NMEA_VTG::operator=, "C++: mrpt::obs::gnss::Message_NMEA_VTG::operator=(const struct mrpt::obs::gnss::Message_NMEA_VTG &) --> struct mrpt::obs::gnss::Message_NMEA_VTG &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NMEA_VTG::content_t file: line:183
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NMEA_VTG::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_VTG::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_VTG::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_VTG::content_t const &o){ return new mrpt::obs::gnss::Message_NMEA_VTG::content_t(o); } ) );
			cl.def_readwrite("true_track", &mrpt::obs::gnss::Message_NMEA_VTG::content_t::true_track);
			cl.def_readwrite("magnetic_track", &mrpt::obs::gnss::Message_NMEA_VTG::content_t::magnetic_track);
			cl.def_readwrite("ground_speed_knots", &mrpt::obs::gnss::Message_NMEA_VTG::content_t::ground_speed_knots);
			cl.def_readwrite("ground_speed_kmh", &mrpt::obs::gnss::Message_NMEA_VTG::content_t::ground_speed_kmh);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_VTG::content_t & (mrpt::obs::gnss::Message_NMEA_VTG::content_t::*)(const struct mrpt::obs::gnss::Message_NMEA_VTG::content_t &)) &mrpt::obs::gnss::Message_NMEA_VTG::content_t::operator=, "C++: mrpt::obs::gnss::Message_NMEA_VTG::content_t::operator=(const struct mrpt::obs::gnss::Message_NMEA_VTG::content_t &) --> struct mrpt::obs::gnss::Message_NMEA_VTG::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NMEA_GSA file: line:198
		pybind11::class_<mrpt::obs::gnss::Message_NMEA_GSA, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_GSA>, PyCallBack_mrpt_obs_gnss_Message_NMEA_GSA, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NMEA_GSA", "NMEA datum: GSA. \n mrpt::obs::CObservationGPS   ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_GSA(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_GSA(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NMEA_GSA const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_GSA(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_GSA const &o){ return new mrpt::obs::gnss::Message_NMEA_GSA(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NMEA_GSA::fields);
		cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_GSA & (mrpt::obs::gnss::Message_NMEA_GSA::*)(const struct mrpt::obs::gnss::Message_NMEA_GSA &)) &mrpt::obs::gnss::Message_NMEA_GSA::operator=, "C++: mrpt::obs::gnss::Message_NMEA_GSA::operator=(const struct mrpt::obs::gnss::Message_NMEA_GSA &) --> struct mrpt::obs::gnss::Message_NMEA_GSA &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NMEA_GSA::content_t file: line:207
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NMEA_GSA::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_GSA::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_GSA::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_GSA::content_t const &o){ return new mrpt::obs::gnss::Message_NMEA_GSA::content_t(o); } ) );
			cl.def_readwrite("auto_selection_fix", &mrpt::obs::gnss::Message_NMEA_GSA::content_t::auto_selection_fix);
			cl.def_readwrite("fix_2D_3D", &mrpt::obs::gnss::Message_NMEA_GSA::content_t::fix_2D_3D);
			cl.def_readwrite("PDOP", &mrpt::obs::gnss::Message_NMEA_GSA::content_t::PDOP);
			cl.def_readwrite("HDOP", &mrpt::obs::gnss::Message_NMEA_GSA::content_t::HDOP);
			cl.def_readwrite("VDOP", &mrpt::obs::gnss::Message_NMEA_GSA::content_t::VDOP);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_GSA::content_t & (mrpt::obs::gnss::Message_NMEA_GSA::content_t::*)(const struct mrpt::obs::gnss::Message_NMEA_GSA::content_t &)) &mrpt::obs::gnss::Message_NMEA_GSA::content_t::operator=, "C++: mrpt::obs::gnss::Message_NMEA_GSA::content_t::operator=(const struct mrpt::obs::gnss::Message_NMEA_GSA::content_t &) --> struct mrpt::obs::gnss::Message_NMEA_GSA::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NMEA_ZDA file: line:224
		pybind11::class_<mrpt::obs::gnss::Message_NMEA_ZDA, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_ZDA>, PyCallBack_mrpt_obs_gnss_Message_NMEA_ZDA, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NMEA_ZDA", "NMEA datum: ZDA. \n mrpt::obs::CObservationGPS   ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_ZDA(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_ZDA(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NMEA_ZDA const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NMEA_ZDA(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_ZDA const &o){ return new mrpt::obs::gnss::Message_NMEA_ZDA(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NMEA_ZDA::fields);
		cl.def("getDateTimeAsTimestamp", (mrpt::Clock::time_point (mrpt::obs::gnss::Message_NMEA_ZDA::*)() const) &mrpt::obs::gnss::Message_NMEA_ZDA::getDateTimeAsTimestamp, "Build an MRPT UTC timestamp with the year/month/day + hour/minute/sec of\n this observation. \n\nC++: mrpt::obs::gnss::Message_NMEA_ZDA::getDateTimeAsTimestamp() const --> mrpt::Clock::time_point");
		cl.def("getDateAsTimestamp", (mrpt::Clock::time_point (mrpt::obs::gnss::Message_NMEA_ZDA::*)() const) &mrpt::obs::gnss::Message_NMEA_ZDA::getDateAsTimestamp, "Build an MRPT timestamp with the year/month/day of this observation. \n\nC++: mrpt::obs::gnss::Message_NMEA_ZDA::getDateAsTimestamp() const --> mrpt::Clock::time_point");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_ZDA & (mrpt::obs::gnss::Message_NMEA_ZDA::*)(const struct mrpt::obs::gnss::Message_NMEA_ZDA &)) &mrpt::obs::gnss::Message_NMEA_ZDA::operator=, "C++: mrpt::obs::gnss::Message_NMEA_ZDA::operator=(const struct mrpt::obs::gnss::Message_NMEA_ZDA &) --> struct mrpt::obs::gnss::Message_NMEA_ZDA &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NMEA_ZDA::content_t file: line:233
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NMEA_ZDA::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NMEA_ZDA::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NMEA_ZDA::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NMEA_ZDA::content_t const &o){ return new mrpt::obs::gnss::Message_NMEA_ZDA::content_t(o); } ) );
			cl.def_readwrite("UTCTime", &mrpt::obs::gnss::Message_NMEA_ZDA::content_t::UTCTime);
			cl.def_readwrite("date_day", &mrpt::obs::gnss::Message_NMEA_ZDA::content_t::date_day);
			cl.def_readwrite("date_month", &mrpt::obs::gnss::Message_NMEA_ZDA::content_t::date_month);
			cl.def_readwrite("date_year", &mrpt::obs::gnss::Message_NMEA_ZDA::content_t::date_year);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NMEA_ZDA::content_t & (mrpt::obs::gnss::Message_NMEA_ZDA::content_t::*)(const struct mrpt::obs::gnss::Message_NMEA_ZDA::content_t &)) &mrpt::obs::gnss::Message_NMEA_ZDA::content_t::operator=, "C++: mrpt::obs::gnss::Message_NMEA_ZDA::content_t::operator=(const struct mrpt::obs::gnss::Message_NMEA_ZDA::content_t &) --> struct mrpt::obs::gnss::Message_NMEA_ZDA::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::nv_oem6_header_t file: line:19
		pybind11::class_<mrpt::obs::gnss::nv_oem6_header_t, std::shared_ptr<mrpt::obs::gnss::nv_oem6_header_t>> cl(M("mrpt::obs::gnss"), "nv_oem6_header_t", "Novatel OEM6 regular header structure \n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](mrpt::obs::gnss::nv_oem6_header_t const &o){ return new mrpt::obs::gnss::nv_oem6_header_t(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::nv_oem6_header_t(); } ) );
		cl.def_readwrite("hdr_len", &mrpt::obs::gnss::nv_oem6_header_t::hdr_len);
		cl.def_readwrite("msg_id", &mrpt::obs::gnss::nv_oem6_header_t::msg_id);
		cl.def_readwrite("msg_type", &mrpt::obs::gnss::nv_oem6_header_t::msg_type);
		cl.def_readwrite("port_addr", &mrpt::obs::gnss::nv_oem6_header_t::port_addr);
		cl.def_readwrite("msg_len", &mrpt::obs::gnss::nv_oem6_header_t::msg_len);
		cl.def_readwrite("seq_number", &mrpt::obs::gnss::nv_oem6_header_t::seq_number);
		cl.def_readwrite("idle_percent", &mrpt::obs::gnss::nv_oem6_header_t::idle_percent);
		cl.def_readwrite("time_status", &mrpt::obs::gnss::nv_oem6_header_t::time_status);
		cl.def_readwrite("week", &mrpt::obs::gnss::nv_oem6_header_t::week);
		cl.def_readwrite("ms_in_week", &mrpt::obs::gnss::nv_oem6_header_t::ms_in_week);
		cl.def_readwrite("receiver_status", &mrpt::obs::gnss::nv_oem6_header_t::receiver_status);
		cl.def_readwrite("reserved", &mrpt::obs::gnss::nv_oem6_header_t::reserved);
		cl.def_readwrite("receiver_sw_version", &mrpt::obs::gnss::nv_oem6_header_t::receiver_sw_version);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::nv_oem6_header_t::*)()) &mrpt::obs::gnss::nv_oem6_header_t::fixEndianness, "C++: mrpt::obs::gnss::nv_oem6_header_t::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::nv_oem6_header_t & (mrpt::obs::gnss::nv_oem6_header_t::*)(const struct mrpt::obs::gnss::nv_oem6_header_t &)) &mrpt::obs::gnss::nv_oem6_header_t::operator=, "C++: mrpt::obs::gnss::nv_oem6_header_t::operator=(const struct mrpt::obs::gnss::nv_oem6_header_t &) --> struct mrpt::obs::gnss::nv_oem6_header_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::gnss::nv_oem6_short_header_t file: line:59
		pybind11::class_<mrpt::obs::gnss::nv_oem6_short_header_t, std::shared_ptr<mrpt::obs::gnss::nv_oem6_short_header_t>> cl(M("mrpt::obs::gnss"), "nv_oem6_short_header_t", "Novatel OEM6 short header structure \n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](mrpt::obs::gnss::nv_oem6_short_header_t const &o){ return new mrpt::obs::gnss::nv_oem6_short_header_t(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::nv_oem6_short_header_t(); } ) );
		cl.def_readwrite("msg_len", &mrpt::obs::gnss::nv_oem6_short_header_t::msg_len);
		cl.def_readwrite("msg_id", &mrpt::obs::gnss::nv_oem6_short_header_t::msg_id);
		cl.def_readwrite("week", &mrpt::obs::gnss::nv_oem6_short_header_t::week);
		cl.def_readwrite("ms_in_week", &mrpt::obs::gnss::nv_oem6_short_header_t::ms_in_week);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::nv_oem6_short_header_t::*)()) &mrpt::obs::gnss::nv_oem6_short_header_t::fixEndianness, "C++: mrpt::obs::gnss::nv_oem6_short_header_t::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::nv_oem6_short_header_t & (mrpt::obs::gnss::nv_oem6_short_header_t::*)(const struct mrpt::obs::gnss::nv_oem6_short_header_t &)) &mrpt::obs::gnss::nv_oem6_short_header_t::operator=, "C++: mrpt::obs::gnss::nv_oem6_short_header_t::operator=(const struct mrpt::obs::gnss::nv_oem6_short_header_t &) --> struct mrpt::obs::gnss::nv_oem6_short_header_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
