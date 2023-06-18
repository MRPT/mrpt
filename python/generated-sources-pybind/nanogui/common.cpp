#include <nanogui/common.h>
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

void bind_nanogui_common(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // NVGcontext file:nanogui/common.h line:106
		pybind11::class_<NVGcontext, std::shared_ptr<NVGcontext>> cl(M(""), "NVGcontext", "");
		cl.def( pybind11::init( [](){ return new NVGcontext(); } ) );
	}
	{ // GLFWwindow file:nanogui/common.h line:107
		pybind11::class_<GLFWwindow, std::shared_ptr<GLFWwindow>> cl(M(""), "GLFWwindow", "");
		cl.def( pybind11::init( [](){ return new GLFWwindow(); } ) );
	}
}
