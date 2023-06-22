#include <iterator>
#include <memory>
#include <mrpt/core/Stringifyable.h>
#include <sstream> // __str__
#include <string>

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

// mrpt::Stringifyable file:mrpt/core/Stringifyable.h line:20
struct PyCallBack_mrpt_Stringifyable : public mrpt::Stringifyable {
	using mrpt::Stringifyable::Stringifyable;

	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::Stringifyable *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"Stringifyable::asString\"");
	}
};

void bind_mrpt_core_Stringifyable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::Stringifyable file:mrpt/core/Stringifyable.h line:20
		pybind11::class_<mrpt::Stringifyable, std::shared_ptr<mrpt::Stringifyable>, PyCallBack_mrpt_Stringifyable> cl(M("mrpt"), "Stringifyable", "Interface for classes whose state can be represented as a human-friendly\n text.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_Stringifyable(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_Stringifyable const &>());
		cl.def("asString", (std::string (mrpt::Stringifyable::*)() const) &mrpt::Stringifyable::asString, "Returns a human-friendly textual description of the object. For classes\n with a large/complex internal state, only a summary should be returned\n instead of the exhaustive enumeration of all data members.\n\nC++: mrpt::Stringifyable::asString() const --> std::string");
		cl.def("assign", (class mrpt::Stringifyable & (mrpt::Stringifyable::*)(const class mrpt::Stringifyable &)) &mrpt::Stringifyable::operator=, "C++: mrpt::Stringifyable::operator=(const class mrpt::Stringifyable &) --> class mrpt::Stringifyable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
