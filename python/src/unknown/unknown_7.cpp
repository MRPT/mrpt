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

// mrpt::obs::gnss::Message_NV_OEM6_MARKTIME file: line:120
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKTIME : public mrpt::obs::gnss::Message_NV_OEM6_MARKTIME {
	using mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::Message_NV_OEM6_MARKTIME;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_MARKTIME *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_MARKTIME::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_MARKTIME *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_MARKTIME::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_MARKTIME *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_MARKTIME::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME file: line:126
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARK2TIME : public mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME {
	using mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::Message_NV_OEM6_MARK2TIME;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_MARK2TIME::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_MARK2TIME::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_MARK2TIME::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_IONUTC file: line:132
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_IONUTC : public mrpt::obs::gnss::Message_NV_OEM6_IONUTC {
	using mrpt::obs::gnss::Message_NV_OEM6_IONUTC::Message_NV_OEM6_IONUTC;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_IONUTC *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_IONUTC::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_IONUTC *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_IONUTC::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_IONUTC *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_IONUTC::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_TOPCON_PZS file: line:18
struct PyCallBack_mrpt_obs_gnss_Message_TOPCON_PZS : public mrpt::obs::gnss::Message_TOPCON_PZS {
	using mrpt::obs::gnss::Message_TOPCON_PZS::Message_TOPCON_PZS;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_TOPCON_PZS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_TOPCON_PZS::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_TOPCON_PZS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_TOPCON_PZS::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_TOPCON_PZS *>(this), "fixEndianness");
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

// mrpt::obs::gnss::Message_TOPCON_SATS file: line:87
struct PyCallBack_mrpt_obs_gnss_Message_TOPCON_SATS : public mrpt::obs::gnss::Message_TOPCON_SATS {
	using mrpt::obs::gnss::Message_TOPCON_SATS::Message_TOPCON_SATS;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_TOPCON_SATS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_TOPCON_SATS::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_TOPCON_SATS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_TOPCON_SATS::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_TOPCON_SATS *>(this), "fixEndianness");
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

void bind_unknown_unknown_7(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::gnss::Message_NV_OEM6_MARKTIME file: line:120
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_MARKTIME, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_MARKTIME>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKTIME, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_MARKTIME", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_MARKTIME(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKTIME(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKTIME const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARKTIME(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_MARKTIME const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_MARKTIME(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::*)()) &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_MARKTIME & (mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_MARKTIME &)) &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_MARKTIME &) --> struct mrpt::obs::gnss::Message_NV_OEM6_MARKTIME &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::header);
			cl.def_readwrite("week", &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::week);
			cl.def_readwrite("week_seconds", &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::week_seconds);
			cl.def_readwrite("clock_offset", &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::clock_offset);
			cl.def_readwrite("clock_offset_std", &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::clock_offset_std);
			cl.def_readwrite("utc_offset", &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::utc_offset);
			cl.def_readwrite("clock_status", &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::clock_status);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t & (mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_MARKTIME::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME file: line:126
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARK2TIME, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_MARK2TIME", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARK2TIME(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARK2TIME const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_MARK2TIME(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::*)()) &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME & (mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME &)) &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME &) --> struct mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::header);
			cl.def_readwrite("week", &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::week);
			cl.def_readwrite("week_seconds", &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::week_seconds);
			cl.def_readwrite("clock_offset", &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::clock_offset);
			cl.def_readwrite("clock_offset_std", &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::clock_offset_std);
			cl.def_readwrite("utc_offset", &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::utc_offset);
			cl.def_readwrite("clock_status", &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::clock_status);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t & (mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_IONUTC file: line:132
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_IONUTC, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_IONUTC>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_IONUTC, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_IONUTC", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_IONUTC(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_IONUTC(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_IONUTC const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_IONUTC(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_IONUTC const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_IONUTC(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_IONUTC::*)()) &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_IONUTC::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_IONUTC & (mrpt::obs::gnss::Message_NV_OEM6_IONUTC::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_IONUTC &)) &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_IONUTC::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_IONUTC &) --> struct mrpt::obs::gnss::Message_NV_OEM6_IONUTC &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::header);
			cl.def_readwrite("a0", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::a0);
			cl.def_readwrite("a1", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::a1);
			cl.def_readwrite("a2", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::a2);
			cl.def_readwrite("a3", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::a3);
			cl.def_readwrite("b0", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::b0);
			cl.def_readwrite("b1", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::b1);
			cl.def_readwrite("b2", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::b2);
			cl.def_readwrite("b3", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::b3);
			cl.def_readwrite("utc_wn", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::utc_wn);
			cl.def_readwrite("tot", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::tot);
			cl.def_readwrite("A0", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::A0);
			cl.def_readwrite("A1", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::A1);
			cl.def_readwrite("wn_lsf", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::wn_lsf);
			cl.def_readwrite("dn", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::dn);
			cl.def_readwrite("deltat_ls", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::deltat_ls);
			cl.def_readwrite("deltat_lsf", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::deltat_lsf);
			cl.def_readwrite("reserved", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::reserved);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t & (mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_IONUTC::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_TOPCON_PZS file: line:18
		pybind11::class_<mrpt::obs::gnss::Message_TOPCON_PZS, std::shared_ptr<mrpt::obs::gnss::Message_TOPCON_PZS>, PyCallBack_mrpt_obs_gnss_Message_TOPCON_PZS, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_TOPCON_PZS", "GPS datum for TopCon's mmGPS devices: PZS. \n mrpt::obs::CObservationGPS ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_TOPCON_PZS(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_TOPCON_PZS(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_TOPCON_PZS const &o){ return new PyCallBack_mrpt_obs_gnss_Message_TOPCON_PZS(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_TOPCON_PZS const &o){ return new mrpt::obs::gnss::Message_TOPCON_PZS(o); } ) );
		cl.def_readwrite("latitude_degrees", &mrpt::obs::gnss::Message_TOPCON_PZS::latitude_degrees);
		cl.def_readwrite("longitude_degrees", &mrpt::obs::gnss::Message_TOPCON_PZS::longitude_degrees);
		cl.def_readwrite("height_meters", &mrpt::obs::gnss::Message_TOPCON_PZS::height_meters);
		cl.def_readwrite("RTK_height_meters", &mrpt::obs::gnss::Message_TOPCON_PZS::RTK_height_meters);
		cl.def_readwrite("PSigma", &mrpt::obs::gnss::Message_TOPCON_PZS::PSigma);
		cl.def_readwrite("angle_transmitter", &mrpt::obs::gnss::Message_TOPCON_PZS::angle_transmitter);
		cl.def_readwrite("nId", &mrpt::obs::gnss::Message_TOPCON_PZS::nId);
		cl.def_readwrite("Fix", &mrpt::obs::gnss::Message_TOPCON_PZS::Fix);
		cl.def_readwrite("TXBattery", &mrpt::obs::gnss::Message_TOPCON_PZS::TXBattery);
		cl.def_readwrite("RXBattery", &mrpt::obs::gnss::Message_TOPCON_PZS::RXBattery);
		cl.def_readwrite("error", &mrpt::obs::gnss::Message_TOPCON_PZS::error);
		cl.def_readwrite("hasCartesianPosVel", &mrpt::obs::gnss::Message_TOPCON_PZS::hasCartesianPosVel);
		cl.def_readwrite("cartesian_x", &mrpt::obs::gnss::Message_TOPCON_PZS::cartesian_x);
		cl.def_readwrite("cartesian_y", &mrpt::obs::gnss::Message_TOPCON_PZS::cartesian_y);
		cl.def_readwrite("cartesian_z", &mrpt::obs::gnss::Message_TOPCON_PZS::cartesian_z);
		cl.def_readwrite("cartesian_vx", &mrpt::obs::gnss::Message_TOPCON_PZS::cartesian_vx);
		cl.def_readwrite("cartesian_vy", &mrpt::obs::gnss::Message_TOPCON_PZS::cartesian_vy);
		cl.def_readwrite("cartesian_vz", &mrpt::obs::gnss::Message_TOPCON_PZS::cartesian_vz);
		cl.def_readwrite("hasPosCov", &mrpt::obs::gnss::Message_TOPCON_PZS::hasPosCov);
		cl.def_readwrite("pos_covariance", &mrpt::obs::gnss::Message_TOPCON_PZS::pos_covariance);
		cl.def_readwrite("hasVelCov", &mrpt::obs::gnss::Message_TOPCON_PZS::hasVelCov);
		cl.def_readwrite("vel_covariance", &mrpt::obs::gnss::Message_TOPCON_PZS::vel_covariance);
		cl.def_readwrite("hasStats", &mrpt::obs::gnss::Message_TOPCON_PZS::hasStats);
		cl.def_readwrite("stats_GPS_sats_used", &mrpt::obs::gnss::Message_TOPCON_PZS::stats_GPS_sats_used);
		cl.def_readwrite("stats_GLONASS_sats_used", &mrpt::obs::gnss::Message_TOPCON_PZS::stats_GLONASS_sats_used);
		cl.def_readwrite("stats_rtk_fix_progress", &mrpt::obs::gnss::Message_TOPCON_PZS::stats_rtk_fix_progress);
		cl.def("assign", (struct mrpt::obs::gnss::Message_TOPCON_PZS & (mrpt::obs::gnss::Message_TOPCON_PZS::*)(const struct mrpt::obs::gnss::Message_TOPCON_PZS &)) &mrpt::obs::gnss::Message_TOPCON_PZS::operator=, "C++: mrpt::obs::gnss::Message_TOPCON_PZS::operator=(const struct mrpt::obs::gnss::Message_TOPCON_PZS &) --> struct mrpt::obs::gnss::Message_TOPCON_PZS &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::gnss::Message_TOPCON_SATS file: line:87
		pybind11::class_<mrpt::obs::gnss::Message_TOPCON_SATS, std::shared_ptr<mrpt::obs::gnss::Message_TOPCON_SATS>, PyCallBack_mrpt_obs_gnss_Message_TOPCON_SATS, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_TOPCON_SATS", "TopCon mmGPS devices: SATS, a generic structure for statistics about tracked\n satelites and their positions. \n\n mrpt::obs::CObservationGPS   ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_TOPCON_SATS(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_TOPCON_SATS(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_TOPCON_SATS const &o){ return new PyCallBack_mrpt_obs_gnss_Message_TOPCON_SATS(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_TOPCON_SATS const &o){ return new mrpt::obs::gnss::Message_TOPCON_SATS(o); } ) );
		cl.def_readwrite("USIs", &mrpt::obs::gnss::Message_TOPCON_SATS::USIs);
		cl.def_readwrite("ELs", &mrpt::obs::gnss::Message_TOPCON_SATS::ELs);
		cl.def_readwrite("AZs", &mrpt::obs::gnss::Message_TOPCON_SATS::AZs);
		cl.def("assign", (struct mrpt::obs::gnss::Message_TOPCON_SATS & (mrpt::obs::gnss::Message_TOPCON_SATS::*)(const struct mrpt::obs::gnss::Message_TOPCON_SATS &)) &mrpt::obs::gnss::Message_TOPCON_SATS::operator=, "C++: mrpt::obs::gnss::Message_TOPCON_SATS::operator=(const struct mrpt::obs::gnss::Message_TOPCON_SATS &) --> struct mrpt::obs::gnss::Message_TOPCON_SATS &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
