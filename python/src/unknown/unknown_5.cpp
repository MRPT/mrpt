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

// mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME file: line:202
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_FRAME : public mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME {
	using mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME::Message_NV_OEM6_GENERIC_FRAME;

	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_GENERIC_FRAME::fixEndianness();
	}
	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_GENERIC_FRAME::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_GENERIC_FRAME::internal_readFromStream(a0);
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME file: line:219
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_SHORT_FRAME : public mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME {
	using mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME::Message_NV_OEM6_GENERIC_SHORT_FRAME;

	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_GENERIC_SHORT_FRAME::fixEndianness();
	}
	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_GENERIC_SHORT_FRAME::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_GENERIC_SHORT_FRAME::internal_readFromStream(a0);
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_BESTPOS file: line:78
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_BESTPOS : public mrpt::obs::gnss::Message_NV_OEM6_BESTPOS {
	using mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::Message_NV_OEM6_BESTPOS;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_BESTPOS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_BESTPOS::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_BESTPOS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_BESTPOS::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_BESTPOS *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_BESTPOS::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_INSPVAS file: line:84
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSPVAS : public mrpt::obs::gnss::Message_NV_OEM6_INSPVAS {
	using mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::Message_NV_OEM6_INSPVAS;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_INSPVAS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_INSPVAS::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_INSPVAS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_INSPVAS::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_INSPVAS *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_INSPVAS::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_INSCOVS file: line:90
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSCOVS : public mrpt::obs::gnss::Message_NV_OEM6_INSCOVS {
	using mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::Message_NV_OEM6_INSCOVS;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_INSCOVS *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_INSCOVS::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_INSCOVS *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_INSCOVS::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_INSCOVS *>(this), "fixEndianness");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_INSCOVS::fixEndianness();
	}
};

// mrpt::obs::gnss::Message_NV_OEM6_RANGECMP file: line:369
struct PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RANGECMP : public mrpt::obs::gnss::Message_NV_OEM6_RANGECMP {
	using mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::Message_NV_OEM6_RANGECMP;

	void internal_writeToStream(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RANGECMP *>(this), "internal_writeToStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RANGECMP::internal_writeToStream(a0);
	}
	void internal_readFromStream(class mrpt::serialization::CArchive & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RANGECMP *>(this), "internal_readFromStream");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return Message_NV_OEM6_RANGECMP::internal_readFromStream(a0);
	}
	void fixEndianness() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::gnss::Message_NV_OEM6_RANGECMP *>(this), "fixEndianness");
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

void bind_unknown_unknown_5(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME file: line:202
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_FRAME, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_GENERIC_FRAME", "Novatel generic frame (to store frames without a parser at the present\n time). \n\n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_FRAME(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_FRAME const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_FRAME(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME(o); } ) );
		cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME::header);
		cl.def_readwrite("msg_body", &mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME::msg_body);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME::*)()) &mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME & (mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME &)) &mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME &) --> struct mrpt::obs::gnss::Message_NV_OEM6_GENERIC_FRAME &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME file: line:219
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_SHORT_FRAME, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_GENERIC_SHORT_FRAME", "Novatel generic short-header frame (to store frames without a parser at the\n present time). \n\n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_SHORT_FRAME(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_SHORT_FRAME const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_GENERIC_SHORT_FRAME(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME(o); } ) );
		cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME::header);
		cl.def_readwrite("msg_body", &mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME::msg_body);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME::*)()) &mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME & (mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME &)) &mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME &) --> struct mrpt::obs::gnss::Message_NV_OEM6_GENERIC_SHORT_FRAME &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_BESTPOS file: line:78
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_BESTPOS, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_BESTPOS>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_BESTPOS, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_BESTPOS", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_BESTPOS(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_BESTPOS(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_BESTPOS const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_BESTPOS(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_BESTPOS const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_BESTPOS(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::*)()) &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_BESTPOS & (mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_BESTPOS &)) &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_BESTPOS &) --> struct mrpt::obs::gnss::Message_NV_OEM6_BESTPOS &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::header);
			cl.def_readwrite("solution_stat", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::solution_stat);
			cl.def_readwrite("position_type", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::position_type);
			cl.def_readwrite("lat", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::lat);
			cl.def_readwrite("lon", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::lon);
			cl.def_readwrite("hgt", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::hgt);
			cl.def_readwrite("undulation", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::undulation);
			cl.def_readwrite("datum_id", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::datum_id);
			cl.def_readwrite("lat_sigma", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::lat_sigma);
			cl.def_readwrite("lon_sigma", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::lon_sigma);
			cl.def_readwrite("hgt_sigma", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::hgt_sigma);
			cl.def_readwrite("diff_age", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::diff_age);
			cl.def_readwrite("sol_age", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::sol_age);
			cl.def_readwrite("num_sats_tracked", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::num_sats_tracked);
			cl.def_readwrite("num_sats_sol", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::num_sats_sol);
			cl.def_readwrite("num_sats_sol_L1", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::num_sats_sol_L1);
			cl.def_readwrite("num_sats_sol_multi", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::num_sats_sol_multi);
			cl.def_readwrite("reserved", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::reserved);
			cl.def_readwrite("ext_sol_stat", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::ext_sol_stat);
			cl.def_readwrite("galileo_beidou_mask", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::galileo_beidou_mask);
			cl.def_readwrite("gps_glonass_mask", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::gps_glonass_mask);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t & (mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_BESTPOS::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_INSPVAS file: line:84
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_INSPVAS, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_INSPVAS>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSPVAS, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_INSPVAS", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_INSPVAS(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSPVAS(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSPVAS const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSPVAS(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_INSPVAS const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_INSPVAS(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::*)()) &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_INSPVAS & (mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_INSPVAS &)) &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_INSPVAS &) --> struct mrpt::obs::gnss::Message_NV_OEM6_INSPVAS &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::header);
			cl.def_readwrite("week", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::week);
			cl.def_readwrite("seconds_in_week", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::seconds_in_week);
			cl.def_readwrite("lat", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::lat);
			cl.def_readwrite("lon", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::lon);
			cl.def_readwrite("hgt", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::hgt);
			cl.def_readwrite("vel_north", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::vel_north);
			cl.def_readwrite("vel_east", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::vel_east);
			cl.def_readwrite("vel_up", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::vel_up);
			cl.def_readwrite("roll", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::roll);
			cl.def_readwrite("pitch", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::pitch);
			cl.def_readwrite("azimuth", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::azimuth);
			cl.def_readwrite("ins_status", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::ins_status);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t & (mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_INSPVAS::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_INSCOVS file: line:90
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_INSCOVS, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_INSCOVS>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSCOVS, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_INSCOVS", "");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_INSCOVS(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSCOVS(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSCOVS const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_INSCOVS(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_INSCOVS const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_INSCOVS(o); } ) );
		cl.def_readwrite("fields", &mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::fields);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::*)()) &mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::fixEndianness, "C++: mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_INSCOVS & (mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_INSCOVS &)) &mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_INSCOVS &) --> struct mrpt::obs::gnss::Message_NV_OEM6_INSCOVS &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t file: line:122
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t>> cl(enclosing_class, "content_t", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t(o); } ) );
			cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t::header);
			cl.def_readwrite("week", &mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t::week);
			cl.def_readwrite("seconds_in_week", &mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t::seconds_in_week);
			cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t::crc);
			cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t & (mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t &)) &mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t &) --> struct mrpt::obs::gnss::Message_NV_OEM6_INSCOVS::content_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::obs::gnss::Message_NV_OEM6_RANGECMP file: line:369
		pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_RANGECMP, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_RANGECMP>, PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RANGECMP, mrpt::obs::gnss::gnss_message> cl(M("mrpt::obs::gnss"), "Message_NV_OEM6_RANGECMP", "Novatel frame: NV_OEM6_RANGECMP. \n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_RANGECMP(); }, [](){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RANGECMP(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RANGECMP const &o){ return new PyCallBack_mrpt_obs_gnss_Message_NV_OEM6_RANGECMP(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::gnss::Message_NV_OEM6_RANGECMP const &o){ return new mrpt::obs::gnss::Message_NV_OEM6_RANGECMP(o); } ) );
		cl.def_readwrite("header", &mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::header);
		cl.def_readwrite("num_obs", &mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::num_obs);
		cl.def_readwrite("obs_data", &mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::obs_data);
		cl.def_readwrite("crc", &mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::crc);
		cl.def("assign", (struct mrpt::obs::gnss::Message_NV_OEM6_RANGECMP & (mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::*)(const struct mrpt::obs::gnss::Message_NV_OEM6_RANGECMP &)) &mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::operator=, "C++: mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::operator=(const struct mrpt::obs::gnss::Message_NV_OEM6_RANGECMP &) --> struct mrpt::obs::gnss::Message_NV_OEM6_RANGECMP &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::TCompressedRangeLog file: line:372
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::TCompressedRangeLog, std::shared_ptr<mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::TCompressedRangeLog>> cl(enclosing_class, "TCompressedRangeLog", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::Message_NV_OEM6_RANGECMP::TCompressedRangeLog(); } ) );
		}

	}
}
