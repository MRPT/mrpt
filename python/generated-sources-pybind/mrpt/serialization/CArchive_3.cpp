#include <memory>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <sstream> // __str__
#include <variant>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_serialization_CArchive_3(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/serialization/CArchive.h line:655
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<std::monostate>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<std::monostate>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_std_monostate_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<std::monostate>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<std::monostate>::get, "C++: mrpt::rtti::CLASS_ID_impl<std::monostate>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
}
