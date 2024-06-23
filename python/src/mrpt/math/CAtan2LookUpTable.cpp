#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/math/CAtan2LookUpTable.h>
#include <mrpt/math/CHistogram.h>
#include <sstream> // __str__
#include <utility>
#include <vector>

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

void bind_mrpt_math_CAtan2LookUpTable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CAtan2LookUpTable file:mrpt/math/CAtan2LookUpTable.h line:24
		pybind11::class_<mrpt::math::CAtan2LookUpTable, std::shared_ptr<mrpt::math::CAtan2LookUpTable>> cl(M("mrpt::math"), "CAtan2LookUpTable", "A look-up-table (LUT) of atan values for any (x,y) value in a\n square/rectangular grid of predefined resolution\n\n \n mrpt::math::CAtan2LookUpTableMultiRes,\n mrpt::obs::CSinCosLookUpTableFor2DScans\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::CAtan2LookUpTable(); } ) );
		cl.def( pybind11::init<double, double, double, double, double>(), pybind11::arg("xmin"), pybind11::arg("xmax"), pybind11::arg("ymin"), pybind11::arg("ymax"), pybind11::arg("resolution") );

		cl.def( pybind11::init( [](mrpt::math::CAtan2LookUpTable const &o){ return new mrpt::math::CAtan2LookUpTable(o); } ) );
		cl.def("resize", (void (mrpt::math::CAtan2LookUpTable::*)(double, double, double, double, double)) &mrpt::math::CAtan2LookUpTable::resize, "C++: mrpt::math::CAtan2LookUpTable::resize(double, double, double, double, double) --> void", pybind11::arg("xmin"), pybind11::arg("xmax"), pybind11::arg("ymin"), pybind11::arg("ymax"), pybind11::arg("resolution"));
		cl.def("atan2", (bool (mrpt::math::CAtan2LookUpTable::*)(double, double, double &) const) &mrpt::math::CAtan2LookUpTable::atan2, "Returns the precomputed value for atan2(y,x). \n false if out of\n grid bounds. \n\nC++: mrpt::math::CAtan2LookUpTable::atan2(double, double, double &) const --> bool", pybind11::arg("y"), pybind11::arg("x"), pybind11::arg("out_atan2"));
		cl.def("atan2ByIndex", (bool (mrpt::math::CAtan2LookUpTable::*)(unsigned int, unsigned int, double &) const) &mrpt::math::CAtan2LookUpTable::atan2ByIndex, "Returns the precomputed value for atan2() of the corresponding cell\n with indices (ix,iy). \n\n false if out of grid bounds. \n\nC++: mrpt::math::CAtan2LookUpTable::atan2ByIndex(unsigned int, unsigned int, double &) const --> bool", pybind11::arg("iy"), pybind11::arg("ix"), pybind11::arg("out_atan2"));
		cl.def("getXMin", (double (mrpt::math::CAtan2LookUpTable::*)() const) &mrpt::math::CAtan2LookUpTable::getXMin, "C++: mrpt::math::CAtan2LookUpTable::getXMin() const --> double");
		cl.def("getXMax", (double (mrpt::math::CAtan2LookUpTable::*)() const) &mrpt::math::CAtan2LookUpTable::getXMax, "C++: mrpt::math::CAtan2LookUpTable::getXMax() const --> double");
		cl.def("getYMin", (double (mrpt::math::CAtan2LookUpTable::*)() const) &mrpt::math::CAtan2LookUpTable::getYMin, "C++: mrpt::math::CAtan2LookUpTable::getYMin() const --> double");
		cl.def("getYMax", (double (mrpt::math::CAtan2LookUpTable::*)() const) &mrpt::math::CAtan2LookUpTable::getYMax, "C++: mrpt::math::CAtan2LookUpTable::getYMax() const --> double");
		cl.def("getResolution", (double (mrpt::math::CAtan2LookUpTable::*)() const) &mrpt::math::CAtan2LookUpTable::getResolution, "C++: mrpt::math::CAtan2LookUpTable::getResolution() const --> double");
		cl.def("getSizeX", (size_t (mrpt::math::CAtan2LookUpTable::*)() const) &mrpt::math::CAtan2LookUpTable::getSizeX, "C++: mrpt::math::CAtan2LookUpTable::getSizeX() const --> size_t");
		cl.def("getSizeY", (size_t (mrpt::math::CAtan2LookUpTable::*)() const) &mrpt::math::CAtan2LookUpTable::getSizeY, "C++: mrpt::math::CAtan2LookUpTable::getSizeY() const --> size_t");
	}
	{ // mrpt::math::CAtan2LookUpTableMultiRes file:mrpt/math/CAtan2LookUpTable.h line:79
		pybind11::class_<mrpt::math::CAtan2LookUpTableMultiRes, std::shared_ptr<mrpt::math::CAtan2LookUpTableMultiRes>> cl(M("mrpt::math"), "CAtan2LookUpTableMultiRes", "Like CAtan2LookUpTable but with a multiresolution grid for increasingly\n better accuracy in points nearer to the origin.\n Example of usage:\n \n\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::CAtan2LookUpTableMultiRes(); } ) );
		cl.def( pybind11::init<const class std::map<double, double> &>(), pybind11::arg("lst_resolutions2extensions") );

		cl.def( pybind11::init( [](mrpt::math::CAtan2LookUpTableMultiRes const &o){ return new mrpt::math::CAtan2LookUpTableMultiRes(o); } ) );
		cl.def("resize", (void (mrpt::math::CAtan2LookUpTableMultiRes::*)(const class std::map<double, double> &)) &mrpt::math::CAtan2LookUpTableMultiRes::resize, "See CAtan2LookUpTableMultiRes for a discussion of the parameters \n\nC++: mrpt::math::CAtan2LookUpTableMultiRes::resize(const class std::map<double, double> &) --> void", pybind11::arg("lst_resolutions2extensions"));
		cl.def("atan2", (bool (mrpt::math::CAtan2LookUpTableMultiRes::*)(double, double, double &) const) &mrpt::math::CAtan2LookUpTableMultiRes::atan2, "Returns the precomputed value for atan2(y,x). \n false if out of\n grid bounds. \n\nC++: mrpt::math::CAtan2LookUpTableMultiRes::atan2(double, double, double &) const --> bool", pybind11::arg("y"), pybind11::arg("x"), pybind11::arg("out_atan2"));
		cl.def("assign", (class mrpt::math::CAtan2LookUpTableMultiRes & (mrpt::math::CAtan2LookUpTableMultiRes::*)(const class mrpt::math::CAtan2LookUpTableMultiRes &)) &mrpt::math::CAtan2LookUpTableMultiRes::operator=, "C++: mrpt::math::CAtan2LookUpTableMultiRes::operator=(const class mrpt::math::CAtan2LookUpTableMultiRes &) --> class mrpt::math::CAtan2LookUpTableMultiRes &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::CHistogram file:mrpt/math/CHistogram.h line:34
		pybind11::class_<mrpt::math::CHistogram, std::shared_ptr<mrpt::math::CHistogram>> cl(M("mrpt::math"), "CHistogram", "This class provides an easy way of computing histograms for unidimensional\nreal valued variables.\n   Call \"getHistogram\" or \"getHistogramNormalized\" to retrieve the full list\nof bin positions & hit counts.\n\n  Example:\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init<const double, const double, size_t>(), pybind11::arg("min"), pybind11::arg("max"), pybind11::arg("nBins") );

		cl.def( pybind11::init( [](mrpt::math::CHistogram const &o){ return new mrpt::math::CHistogram(o); } ) );
		cl.def("createWithFixedWidth", (class mrpt::math::CHistogram (mrpt::math::CHistogram::*)(double, double, double)) &mrpt::math::CHistogram::createWithFixedWidth, "Constructor with a fixed bin width.\n \n\n std::exception On max<=min or width<=0\n\nC++: mrpt::math::CHistogram::createWithFixedWidth(double, double, double) --> class mrpt::math::CHistogram", pybind11::arg("min"), pybind11::arg("max"), pybind11::arg("binWidth"));
		cl.def("clear", (void (mrpt::math::CHistogram::*)()) &mrpt::math::CHistogram::clear, "Clear the histogram:\n\nC++: mrpt::math::CHistogram::clear() --> void");
		cl.def("add", (void (mrpt::math::CHistogram::*)(const double)) &mrpt::math::CHistogram::add, "	Add an element to the histogram. If element is out of [min,max] it is\n ignored. \n\nC++: mrpt::math::CHistogram::add(const double) --> void", pybind11::arg("x"));
		cl.def("getBinCount", (size_t (mrpt::math::CHistogram::*)(size_t) const) &mrpt::math::CHistogram::getBinCount, "Retuns the elements count into the selected bin index, where first one\n is 0.\n \n\n std::exception On invalid index\n\nC++: mrpt::math::CHistogram::getBinCount(size_t) const --> size_t", pybind11::arg("index"));
		cl.def("getBinRatio", (double (mrpt::math::CHistogram::*)(size_t) const) &mrpt::math::CHistogram::getBinRatio, "Retuns the ratio in [0,1] range for the selected bin index, where first\n one is 0.\n  It returns 0 if no elements have been added.\n \n\n std::exception On invalid index.\n\nC++: mrpt::math::CHistogram::getBinRatio(size_t) const --> double", pybind11::arg("index"));
		cl.def("assign", (class mrpt::math::CHistogram & (mrpt::math::CHistogram::*)(const class mrpt::math::CHistogram &)) &mrpt::math::CHistogram::operator=, "C++: mrpt::math::CHistogram::operator=(const class mrpt::math::CHistogram &) --> class mrpt::math::CHistogram &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
