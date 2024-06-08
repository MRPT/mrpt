#include <mrpt/rtti/CObject.h>
#include <sstream> // __str__

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

void bind_mrpt_rtti_CObject_2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::rtti::internal::CopyCtor file:mrpt/rtti/CObject.h line:123
		pybind11::class_<mrpt::rtti::internal::CopyCtor<true>, std::shared_ptr<mrpt::rtti::internal::CopyCtor<true>>> cl(M("mrpt::rtti::internal"), "CopyCtor_true_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::internal::CopyCtor<true>(); } ) );
	}
	{ // mrpt::rtti::internal::CopyCtor file:mrpt/rtti/CObject.h line:132
		pybind11::class_<mrpt::rtti::internal::CopyCtor<false>, std::shared_ptr<mrpt::rtti::internal::CopyCtor<false>>> cl(M("mrpt::rtti::internal"), "CopyCtor_false_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::internal::CopyCtor<false>(); } ) );
	}
}
