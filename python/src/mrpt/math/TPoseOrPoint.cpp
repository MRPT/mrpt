#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoseOrPoint.h>
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

void bind_mrpt_math_TPoseOrPoint(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TPoseOrPoint file:mrpt/math/TPoseOrPoint.h line:27
		pybind11::class_<mrpt::math::TPoseOrPoint, std::shared_ptr<mrpt::math::TPoseOrPoint>> cl(M("mrpt::math"), "TPoseOrPoint", "Base type of all TPoseXX and TPointXX classes in mrpt::math.\n Useful for type traits. No virtual methods at all.");
		cl.def( pybind11::init( [](mrpt::math::TPoseOrPoint const &o){ return new mrpt::math::TPoseOrPoint(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::math::TPoseOrPoint(); } ) );
		cl.def("assign", (struct mrpt::math::TPoseOrPoint & (mrpt::math::TPoseOrPoint::*)(const struct mrpt::math::TPoseOrPoint &)) &mrpt::math::TPoseOrPoint::operator=, "C++: mrpt::math::TPoseOrPoint::operator=(const struct mrpt::math::TPoseOrPoint &) --> struct mrpt::math::TPoseOrPoint &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TPoint2D_data file:mrpt/math/TPoint2D.h line:26
		pybind11::class_<mrpt::math::TPoint2D_data<double>, std::shared_ptr<mrpt::math::TPoint2D_data<double>>> cl(M("mrpt::math"), "TPoint2D_data_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPoint2D_data<double>(); } ) );
		cl.def( pybind11::init<double, double>(), pybind11::arg("X"), pybind11::arg("Y") );

		cl.def( pybind11::init( [](mrpt::math::TPoint2D_data<double> const &o){ return new mrpt::math::TPoint2D_data<double>(o); } ) );
		cl.def_readwrite("x", &mrpt::math::TPoint2D_data<double>::x);
		cl.def_readwrite("y", &mrpt::math::TPoint2D_data<double>::y);
		cl.def("assign", (struct mrpt::math::TPoint2D_data<double> & (mrpt::math::TPoint2D_data<double>::*)(const struct mrpt::math::TPoint2D_data<double> &)) &mrpt::math::TPoint2D_data<double>::operator=, "C++: mrpt::math::TPoint2D_data<double>::operator=(const struct mrpt::math::TPoint2D_data<double> &) --> struct mrpt::math::TPoint2D_data<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TPoint2D_data file:mrpt/math/TPoint2D.h line:26
		pybind11::class_<mrpt::math::TPoint2D_data<float>, std::shared_ptr<mrpt::math::TPoint2D_data<float>>> cl(M("mrpt::math"), "TPoint2D_data_float_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPoint2D_data<float>(); } ) );
		cl.def( pybind11::init<float, float>(), pybind11::arg("X"), pybind11::arg("Y") );

		cl.def( pybind11::init( [](mrpt::math::TPoint2D_data<float> const &o){ return new mrpt::math::TPoint2D_data<float>(o); } ) );
		cl.def_readwrite("x", &mrpt::math::TPoint2D_data<float>::x);
		cl.def_readwrite("y", &mrpt::math::TPoint2D_data<float>::y);
		cl.def("assign", (struct mrpt::math::TPoint2D_data<float> & (mrpt::math::TPoint2D_data<float>::*)(const struct mrpt::math::TPoint2D_data<float> &)) &mrpt::math::TPoint2D_data<float>::operator=, "C++: mrpt::math::TPoint2D_data<float>::operator=(const struct mrpt::math::TPoint2D_data<float> &) --> struct mrpt::math::TPoint2D_data<float> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
