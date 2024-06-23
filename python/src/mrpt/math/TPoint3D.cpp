#include <mrpt/math/TPoint3D.h>
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

void bind_mrpt_math_TPoint3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TPoint3D_data file:mrpt/math/TPoint3D.h line:31
		pybind11::class_<mrpt::math::TPoint3D_data<double>, std::shared_ptr<mrpt::math::TPoint3D_data<double>>> cl(M("mrpt::math"), "TPoint3D_data_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPoint3D_data<double>(); } ) );
		cl.def( pybind11::init<double, double, double>(), pybind11::arg("X"), pybind11::arg("Y"), pybind11::arg("Z") );

		cl.def( pybind11::init( [](mrpt::math::TPoint3D_data<double> const &o){ return new mrpt::math::TPoint3D_data<double>(o); } ) );
		cl.def_readwrite("x", &mrpt::math::TPoint3D_data<double>::x);
		cl.def_readwrite("y", &mrpt::math::TPoint3D_data<double>::y);
		cl.def_readwrite("z", &mrpt::math::TPoint3D_data<double>::z);
		cl.def("assign", (struct mrpt::math::TPoint3D_data<double> & (mrpt::math::TPoint3D_data<double>::*)(const struct mrpt::math::TPoint3D_data<double> &)) &mrpt::math::TPoint3D_data<double>::operator=, "C++: mrpt::math::TPoint3D_data<double>::operator=(const struct mrpt::math::TPoint3D_data<double> &) --> struct mrpt::math::TPoint3D_data<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TPoint3D_data file:mrpt/math/TPoint3D.h line:31
		pybind11::class_<mrpt::math::TPoint3D_data<float>, std::shared_ptr<mrpt::math::TPoint3D_data<float>>> cl(M("mrpt::math"), "TPoint3D_data_float_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPoint3D_data<float>(); } ) );
		cl.def( pybind11::init<float, float, float>(), pybind11::arg("X"), pybind11::arg("Y"), pybind11::arg("Z") );

		cl.def( pybind11::init( [](mrpt::math::TPoint3D_data<float> const &o){ return new mrpt::math::TPoint3D_data<float>(o); } ) );
		cl.def_readwrite("x", &mrpt::math::TPoint3D_data<float>::x);
		cl.def_readwrite("y", &mrpt::math::TPoint3D_data<float>::y);
		cl.def_readwrite("z", &mrpt::math::TPoint3D_data<float>::z);
		cl.def("assign", (struct mrpt::math::TPoint3D_data<float> & (mrpt::math::TPoint3D_data<float>::*)(const struct mrpt::math::TPoint3D_data<float> &)) &mrpt::math::TPoint3D_data<float>::operator=, "C++: mrpt::math::TPoint3D_data<float>::operator=(const struct mrpt::math::TPoint3D_data<float> &) --> struct mrpt::math::TPoint3D_data<float> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TPointXYZIu8 file:mrpt/math/TPoint3D.h line:295
		pybind11::class_<mrpt::math::TPointXYZIu8, std::shared_ptr<mrpt::math::TPointXYZIu8>> cl(M("mrpt::math"), "TPointXYZIu8", "XYZ point (double) + Intensity(u8) \n mrpt::math::TPoint3D ");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPointXYZIu8(); } ) );
		cl.def( pybind11::init<double, double, double, uint8_t>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("intensity_val") );

		cl.def_readwrite("pt", &mrpt::math::TPointXYZIu8::pt);
		cl.def_readwrite("intensity", &mrpt::math::TPointXYZIu8::intensity);
	}
	{ // mrpt::math::TPointXYZRGBu8 file:mrpt/math/TPoint3D.h line:306
		pybind11::class_<mrpt::math::TPointXYZRGBu8, std::shared_ptr<mrpt::math::TPointXYZRGBu8>> cl(M("mrpt::math"), "TPointXYZRGBu8", "XYZ point (double) + RGB(u8) \n mrpt::math::TPoint3D ");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPointXYZRGBu8(); } ) );
		cl.def( pybind11::init<double, double, double, uint8_t, uint8_t, uint8_t>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R_val"), pybind11::arg("G_val"), pybind11::arg("B_val") );

		cl.def_readwrite("pt", &mrpt::math::TPointXYZRGBu8::pt);
		cl.def_readwrite("r", &mrpt::math::TPointXYZRGBu8::r);
		cl.def_readwrite("g", &mrpt::math::TPointXYZRGBu8::g);
		cl.def_readwrite("b", &mrpt::math::TPointXYZRGBu8::b);
	}
	{ // mrpt::math::TPointXYZfIu8 file:mrpt/math/TPoint3D.h line:318
		pybind11::class_<mrpt::math::TPointXYZfIu8, std::shared_ptr<mrpt::math::TPointXYZfIu8>> cl(M("mrpt::math"), "TPointXYZfIu8", "XYZ point (float) + Intensity(u8) \n mrpt::math::TPoint3D ");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPointXYZfIu8(); } ) );
		cl.def( pybind11::init<float, float, float, uint8_t>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("intensity_val") );

		cl.def_readwrite("pt", &mrpt::math::TPointXYZfIu8::pt);
		cl.def_readwrite("intensity", &mrpt::math::TPointXYZfIu8::intensity);
	}
	{ // mrpt::math::TPointXYZfRGBu8 file:mrpt/math/TPoint3D.h line:329
		pybind11::class_<mrpt::math::TPointXYZfRGBu8, std::shared_ptr<mrpt::math::TPointXYZfRGBu8>> cl(M("mrpt::math"), "TPointXYZfRGBu8", "XYZ point (float) + RGB(u8) \n mrpt::math::TPoint3D ");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPointXYZfRGBu8(); } ) );
		cl.def( pybind11::init<float, float, float, uint8_t, uint8_t, uint8_t>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R_val"), pybind11::arg("G_val"), pybind11::arg("B_val") );

		cl.def_readwrite("pt", &mrpt::math::TPointXYZfRGBu8::pt);
		cl.def_readwrite("r", &mrpt::math::TPointXYZfRGBu8::r);
		cl.def_readwrite("g", &mrpt::math::TPointXYZfRGBu8::g);
		cl.def_readwrite("b", &mrpt::math::TPointXYZfRGBu8::b);
	}
	{ // mrpt::math::TPointXYZfRGBAu8 file:mrpt/math/TPoint3D.h line:347
		pybind11::class_<mrpt::math::TPointXYZfRGBAu8, std::shared_ptr<mrpt::math::TPointXYZfRGBAu8>> cl(M("mrpt::math"), "TPointXYZfRGBAu8", "XYZ point (float) + RGBA(u8) \n mrpt::math::TPoint3D ");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPointXYZfRGBAu8(); } ) );
		cl.def( pybind11::init( [](float const & a0, float const & a1, float const & a2, uint8_t const & a3, uint8_t const & a4, uint8_t const & a5){ return new mrpt::math::TPointXYZfRGBAu8(a0, a1, a2, a3, a4, a5); } ), "doc" , pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R_val"), pybind11::arg("G_val"), pybind11::arg("B_val"));
		cl.def( pybind11::init<float, float, float, uint8_t, uint8_t, uint8_t, uint8_t>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R_val"), pybind11::arg("G_val"), pybind11::arg("B_val"), pybind11::arg("A_val") );

		cl.def( pybind11::init( [](mrpt::math::TPointXYZfRGBAu8 const &o){ return new mrpt::math::TPointXYZfRGBAu8(o); } ) );
		cl.def_readwrite("pt", &mrpt::math::TPointXYZfRGBAu8::pt);
		cl.def_readwrite("r", &mrpt::math::TPointXYZfRGBAu8::r);
		cl.def_readwrite("g", &mrpt::math::TPointXYZfRGBAu8::g);
		cl.def_readwrite("b", &mrpt::math::TPointXYZfRGBAu8::b);
		cl.def_readwrite("a", &mrpt::math::TPointXYZfRGBAu8::a);
		cl.def("assign", (struct mrpt::math::TPointXYZfRGBAu8 & (mrpt::math::TPointXYZfRGBAu8::*)(const struct mrpt::math::TPointXYZfRGBAu8 &)) &mrpt::math::TPointXYZfRGBAu8::operator=, "C++: mrpt::math::TPointXYZfRGBAu8::operator=(const struct mrpt::math::TPointXYZfRGBAu8 &) --> struct mrpt::math::TPointXYZfRGBAu8 &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TPointXYZRGBAf file:mrpt/math/TPoint3D.h line:372
		pybind11::class_<mrpt::math::TPointXYZRGBAf, std::shared_ptr<mrpt::math::TPointXYZRGBAf>> cl(M("mrpt::math"), "TPointXYZRGBAf", "XYZ point (float) + RGBA(float) [1-byte memory packed, no padding]\n \n\n mrpt::math::TPoint3D ");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPointXYZRGBAf(); } ) );
		cl.def( pybind11::init<float, float, float, float, float, float, float>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("R_val"), pybind11::arg("G_val"), pybind11::arg("B_val"), pybind11::arg("A_val") );

		cl.def_readwrite("pt", &mrpt::math::TPointXYZRGBAf::pt);
		cl.def_readwrite("R", &mrpt::math::TPointXYZRGBAf::R);
		cl.def_readwrite("G", &mrpt::math::TPointXYZRGBAf::G);
		cl.def_readwrite("B", &mrpt::math::TPointXYZRGBAf::B);
		cl.def_readwrite("A", &mrpt::math::TPointXYZRGBAf::A);
	}
}
