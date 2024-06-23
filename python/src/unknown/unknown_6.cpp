#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/obs/gnss_messages.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <ostream>
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

// mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS file: line:96
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RXSTATUS : public mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS {
	using mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::Message_NV_OEM6_RXSTATUS;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RXSTATUS::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RXSTATUS::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RXSTATUS::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM file: line:102
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWEPHEM : public mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM {
	using mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::Message_NV_OEM6_RAWEPHEM;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RAWEPHEM::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RAWEPHEM::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RAWEPHEM::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_VERSION file: line:448
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_VERSION : public mrpt::obs::gnss::Message_NV_OEM6_VERSION {
	using mrpt::obs::gnss::Message_NV_OEM6_VERSION::Message_NV_OEM6_VERSION;

	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_VERSION *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_VERSION::fixEndianness();
	}
	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_VERSION *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_VERSION::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_VERSION *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_VERSION::internal_readFromStream(a0);
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS file: line:108
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWIMUS : public mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS {
	using mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::Message_NV_OEM6_RAWIMUS;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RAWIMUS::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RAWIMUS::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RAWIMUS::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_MARKPOS file: line:114
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKPOS : public mrpt::obs::gnss::Message_NV_OEM6_MARKPOS {
	using mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::Message_NV_OEM6_MARKPOS;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_MARKPOS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_MARKPOS::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_MARKPOS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_MARKPOS::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_MARKPOS *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_MARKPOS::fixEndianness();
	}
};

void bind_unknown_unknown_6(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS file: line:96
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RXSTATUS, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_RXSTATUS", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RXSTATUS(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RXSTATUS const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RXSTATUS(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::*)()) &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS & (mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS &)) &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS &) --> struct mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::header);
			cl.def_readwrite("error", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::error);
			cl.def_readwrite("num_stats", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::num_stats);
			cl.def_readwrite("rxstat", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::rxstat);
			cl.def_readwrite("rxstat_pri", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::rxstat_pri);
			cl.def_readwrite("rxstat_set", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::rxstat_set);
			cl.def_readwrite("rxstat_clear", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::rxstat_clear);
			cl.def_readwrite("aux1stat", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux1stat);
			cl.def_readwrite("aux1stat_pri", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux1stat_pri);
			cl.def_readwrite("aux1stat_set", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux1stat_set);
			cl.def_readwrite("aux1stat_clear", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux1stat_clear);
			cl.def_readwrite("aux2stat", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux2stat);
			cl.def_readwrite("aux2stat_pri", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux2stat_pri);
			cl.def_readwrite("aux2stat_set", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux2stat_set);
			cl.def_readwrite("aux2stat_clear", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux2stat_clear);
			cl.def_readwrite("aux3stat", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux3stat);
			cl.def_readwrite("aux3stat_pri", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux3stat_pri);
			cl.def_readwrite("aux3stat_set", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux3stat_set);
			cl.def_readwrite("aux3stat_clear", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::aux3stat_clear);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t & (mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM file: line:102
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWEPHEM, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_RAWEPHEM", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWEPHEM(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWEPHEM const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWEPHEM(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::*)()) &mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM & (mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM &)) &mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM &) --> struct mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t::header);
			cl.def_readwrite("sat_prn", &mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t::sat_prn);
			cl.def_readwrite("ref_week", &mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t::ref_week);
			cl.def_readwrite("ref_secs", &mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t::ref_secs);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t & (mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_VERSION file: line:448
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_VERSION, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_VERSION>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_VERSION, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_VERSION", "Novatel frame: NV_OEM6_VERSION. \n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_VERSION(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_VERSION(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_VERSION const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_VERSION(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_VERSION const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_VERSION(o); } ) );
		cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_VERSION::header);
		cl.def_readwrite("num_comps", &mrpt::obs::gnss::Message_NV_OEM6_VERSION::num_comps);
		cl.def_readwrite("components", &mrpt::obs::gnss::Message_NV_OEM6_VERSION::components);
		cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_VERSION::crc);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_VERSION::*)()) &mrpt::obs::gnss::Message_NV_OEM6_VERSION::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_VERSION::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_VERSION & (mrpt::obs::gnss::Message_NV_OEM6_VERSION::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_VERSION &)) &mrpt::obs::gnss::Message_NV_OEM6_VERSION::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_VERSION::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_VERSION &) --> struct mrpt::obs::gnss::Message_NV_OEM6_VERSION &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_VERSION::TComponentVersion file: line:451
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_VERSION::TComponentVersion, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_VERSION::TComponentVersion>> cl(enclosing_class, "TComponentVersion", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_VERSION::TComponentVersion(); } ) );
			cl.def_readwrite("type", &mrpt::obs::gnss::Message_NV_OEM6_VERSION::TComponentVersion::type);
		}

	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS file: line:108
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWIMUS, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_RAWIMUS", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWIMUS(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWIMUS const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RAWIMUS(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::*)()) &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS & (mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS &)) &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS &) --> struct mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::header);
			cl.def_readwrite("week", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::week);
			cl.def_readwrite("week_seconds", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::week_seconds);
			cl.def_readwrite("imu_status", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::imu_status);
			cl.def_readwrite("accel_z", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::accel_z);
			cl.def_readwrite("accel_y_neg", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::accel_y_neg);
			cl.def_readwrite("accel_x", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::accel_x);
			cl.def_readwrite("gyro_z", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::gyro_z);
			cl.def_readwrite("gyro_y_neg", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::gyro_y_neg);
			cl.def_readwrite("gyro_x", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::gyro_x);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t & (mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_MARKPOS file: line:114
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_MARKPOS, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_MARKPOS>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKPOS, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_MARKPOS", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_MARKPOS(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKPOS(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKPOS const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKPOS(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_MARKPOS const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_MARKPOS(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::*)()) &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_MARKPOS & (mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_MARKPOS &)) &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_MARKPOS &) --> struct mrpt::obs::gnss::Message_NV_OEM6_MARKPOS &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::header);
			cl.def_readwrite("solution_stat", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::solution_stat);
			cl.def_readwrite("position_type", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::position_type);
			cl.def_readwrite("lat", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::lat);
			cl.def_readwrite("lon", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::lon);
			cl.def_readwrite("hgt", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::hgt);
			cl.def_readwrite("undulation", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::undulation);
			cl.def_readwrite("datum_id", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::datum_id);
			cl.def_readwrite("lat_sigma", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::lat_sigma);
			cl.def_readwrite("lon_sigma", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::lon_sigma);
			cl.def_readwrite("hgt_sigma", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::hgt_sigma);
			cl.def_readwrite("diff_age", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::diff_age);
			cl.def_readwrite("sol_age", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::sol_age);
			cl.def_readwrite("num_sats_tracked", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::num_sats_tracked);
			cl.def_readwrite("num_sats_sol", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::num_sats_sol);
			cl.def_readwrite("num_sats_sol_L1", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::num_sats_sol_L1);
			cl.def_readwrite("num_sats_sol_multi", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::num_sats_sol_multi);
			cl.def_readwrite("reserved", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::reserved);
			cl.def_readwrite("ext_sol_stat", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::ext_sol_stat);
			cl.def_readwrite("galileo_beidou_mask", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::galileo_beidou_mask);
			cl.def_readwrite("gps_glonass_mask", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::gps_glonass_mask);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t & (mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_MARKPOS::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
