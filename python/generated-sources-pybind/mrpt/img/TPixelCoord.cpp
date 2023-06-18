#include <ios>
#include <locale>
#include <mrpt/img/TPixelCoord.h>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
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

void bind_mrpt_img_TPixelCoord(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::img::TPixelCoordf file:mrpt/img/TPixelCoord.h line:18
		pybind11::class_<mrpt::img::TPixelCoordf, std::shared_ptr<mrpt::img::TPixelCoordf>> cl(M("mrpt::img"), "TPixelCoordf", "A pair (x,y) of pixel coordinates (subpixel resolution). \n\n mrpt_img_grp  ");
		cl.def( pybind11::init( [](){ return new mrpt::img::TPixelCoordf(); } ) );
		cl.def( pybind11::init<const float, const float>(), pybind11::arg("_x"), pybind11::arg("_y") );

		cl.def( pybind11::init( [](mrpt::img::TPixelCoordf const &o){ return new mrpt::img::TPixelCoordf(o); } ) );
		cl.def_readwrite("x", &mrpt::img::TPixelCoordf::x);
		cl.def_readwrite("y", &mrpt::img::TPixelCoordf::y);
		cl.def("assign", (struct mrpt::img::TPixelCoordf & (mrpt::img::TPixelCoordf::*)(const struct mrpt::img::TPixelCoordf &)) &mrpt::img::TPixelCoordf::operator=, "C++: mrpt::img::TPixelCoordf::operator=(const struct mrpt::img::TPixelCoordf &) --> struct mrpt::img::TPixelCoordf &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::img::TPixelCoordf const &o) -> std::string { std::ostringstream s; using namespace mrpt::img; s << o; return s.str(); } );
	}
	{ // mrpt::img::TPixelCoord file:mrpt/img/TPixelCoord.h line:40
		pybind11::class_<mrpt::img::TPixelCoord, std::shared_ptr<mrpt::img::TPixelCoord>> cl(M("mrpt::img"), "TPixelCoord", "A pair (x,y) of pixel coordinates (integer resolution). ");
		cl.def( pybind11::init( [](){ return new mrpt::img::TPixelCoord(); } ) );
		cl.def( pybind11::init<const int, const int>(), pybind11::arg("_x"), pybind11::arg("_y") );

		cl.def( pybind11::init( [](mrpt::img::TPixelCoord const &o){ return new mrpt::img::TPixelCoord(o); } ) );
		cl.def_readwrite("x", &mrpt::img::TPixelCoord::x);
		cl.def_readwrite("y", &mrpt::img::TPixelCoord::y);
		cl.def("__eq__", (bool (mrpt::img::TPixelCoord::*)(const struct mrpt::img::TPixelCoord &)) &mrpt::img::TPixelCoord::operator==, "C++: mrpt::img::TPixelCoord::operator==(const struct mrpt::img::TPixelCoord &) --> bool", pybind11::arg("o"));
		cl.def("assign", (struct mrpt::img::TPixelCoord & (mrpt::img::TPixelCoord::*)(const struct mrpt::img::TPixelCoord &)) &mrpt::img::TPixelCoord::operator=, "C++: mrpt::img::TPixelCoord::operator=(const struct mrpt::img::TPixelCoord &) --> struct mrpt::img::TPixelCoord &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::img::TPixelCoord const &o) -> std::string { std::ostringstream s; using namespace mrpt::img; s << o; return s.str(); } );
	}
}
