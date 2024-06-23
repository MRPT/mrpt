#include <ios>
#include <locale>
#include <mrpt/img/TColor.h>
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

void bind_mrpt_img_TColor(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::img::TColor file:mrpt/img/TColor.h line:26
		pybind11::class_<mrpt::img::TColor, std::shared_ptr<mrpt::img::TColor>> cl(M("mrpt::img"), "TColor", "A RGB color - 8bit. Struct pack=1 is ensured.\n \n");
		cl.def( pybind11::init( [](){ return new mrpt::img::TColor(); } ) );
		cl.def( pybind11::init( [](uint8_t const & a0, uint8_t const & a1, uint8_t const & a2){ return new mrpt::img::TColor(a0, a1, a2); } ), "doc" , pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def( pybind11::init<uint8_t, uint8_t, uint8_t, uint8_t>(), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("alpha") );

		cl.def( pybind11::init<const unsigned int>(), pybind11::arg("color_RGB_24bit") );

		cl.def( pybind11::init<const unsigned int, const unsigned char>(), pybind11::arg("color_RGB_24bit"), pybind11::arg("alpha") );

		cl.def( pybind11::init( [](mrpt::img::TColor const &o){ return new mrpt::img::TColor(o); } ) );
		cl.def_readwrite("R", &mrpt::img::TColor::R);
		cl.def_readwrite("G", &mrpt::img::TColor::G);
		cl.def_readwrite("B", &mrpt::img::TColor::B);
		cl.def_readwrite("A", &mrpt::img::TColor::A);
		cl.def("assign", (struct mrpt::img::TColor & (mrpt::img::TColor::*)(const struct mrpt::img::TColor &)) &mrpt::img::TColor::operator=, "C++: mrpt::img::TColor::operator=(const struct mrpt::img::TColor &) --> struct mrpt::img::TColor &", pybind11::return_value_policy::automatic, pybind11::arg("other"));
		cl.def("__iadd__", (struct mrpt::img::TColor & (mrpt::img::TColor::*)(const struct mrpt::img::TColor &)) &mrpt::img::TColor::operator+=, "C++: mrpt::img::TColor::operator+=(const struct mrpt::img::TColor &) --> struct mrpt::img::TColor &", pybind11::return_value_policy::automatic, pybind11::arg("other"));
		cl.def("__isub__", (struct mrpt::img::TColor & (mrpt::img::TColor::*)(const struct mrpt::img::TColor &)) &mrpt::img::TColor::operator-=, "C++: mrpt::img::TColor::operator-=(const struct mrpt::img::TColor &) --> struct mrpt::img::TColor &", pybind11::return_value_policy::automatic, pybind11::arg("other"));
		cl.def_static("red", (struct mrpt::img::TColor (*)()) &mrpt::img::TColor::red, "Predefined colors \n\nC++: mrpt::img::TColor::red() --> struct mrpt::img::TColor");
		cl.def_static("green", (struct mrpt::img::TColor (*)()) &mrpt::img::TColor::green, "C++: mrpt::img::TColor::green() --> struct mrpt::img::TColor");
		cl.def_static("blue", (struct mrpt::img::TColor (*)()) &mrpt::img::TColor::blue, "C++: mrpt::img::TColor::blue() --> struct mrpt::img::TColor");
		cl.def_static("black", (struct mrpt::img::TColor (*)()) &mrpt::img::TColor::black, "C++: mrpt::img::TColor::black() --> struct mrpt::img::TColor");
		cl.def_static("white", (struct mrpt::img::TColor (*)()) &mrpt::img::TColor::white, "C++: mrpt::img::TColor::white() --> struct mrpt::img::TColor");
		cl.def_static("gray", (struct mrpt::img::TColor (*)()) &mrpt::img::TColor::gray, "C++: mrpt::img::TColor::gray() --> struct mrpt::img::TColor");

		cl.def("__str__", [](mrpt::img::TColor const &o) -> std::string { std::ostringstream s; using namespace mrpt::img; s << o; return s.str(); } );
	}
	{ // mrpt::img::TColorf file:mrpt/img/TColor.h line:85
		pybind11::class_<mrpt::img::TColorf, std::shared_ptr<mrpt::img::TColorf>> cl(M("mrpt::img"), "TColorf", "An RGBA color - floats in the range [0,1]\n \n");
		cl.def( pybind11::init( [](){ return new mrpt::img::TColorf(); } ) );
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2){ return new mrpt::img::TColorf(a0, a1, a2); } ), "doc" , pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def( pybind11::init<float, float, float, float>(), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("alpha") );

		cl.def( pybind11::init<const struct mrpt::img::TColor &>(), pybind11::arg("col") );

		cl.def( pybind11::init( [](mrpt::img::TColorf const &o){ return new mrpt::img::TColorf(o); } ) );
		cl.def_readwrite("R", &mrpt::img::TColorf::R);
		cl.def_readwrite("G", &mrpt::img::TColorf::G);
		cl.def_readwrite("B", &mrpt::img::TColorf::B);
		cl.def_readwrite("A", &mrpt::img::TColorf::A);
		cl.def("asTColor", (struct mrpt::img::TColor (mrpt::img::TColorf::*)() const) &mrpt::img::TColorf::asTColor, "Returns the 0-255 integer version of this color: RGBA_u8  \n\nC++: mrpt::img::TColorf::asTColor() const --> struct mrpt::img::TColor");
		cl.def("assign", (struct mrpt::img::TColorf & (mrpt::img::TColorf::*)(const struct mrpt::img::TColorf &)) &mrpt::img::TColorf::operator=, "C++: mrpt::img::TColorf::operator=(const struct mrpt::img::TColorf &) --> struct mrpt::img::TColorf &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::img::TColorf const &o) -> std::string { std::ostringstream s; using namespace mrpt::img; s << o; return s.str(); } );
	}
}
