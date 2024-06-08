#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSinCosLookUpTableFor2DScans.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <sstream> // __str__
#include <string>
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

void bind_mrpt_obs_T2DScanProperties(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::T2DScanProperties file:mrpt/obs/T2DScanProperties.h line:20
		pybind11::class_<mrpt::obs::T2DScanProperties, std::shared_ptr<mrpt::obs::T2DScanProperties>> cl(M("mrpt::obs"), "T2DScanProperties", "Auxiliary struct that holds all the relevant *geometry* information about a\n 2D scan.\n This class is used in CSinCosLookUpTableFor2DScans\n \n\n\n \n CObservation2DRangeScan, CObservation2DRangeScan::getScanProperties,\n CSinCosLookUpTableFor2DScans");
		cl.def( pybind11::init( [](){ return new mrpt::obs::T2DScanProperties(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::T2DScanProperties const &o){ return new mrpt::obs::T2DScanProperties(o); } ) );
		cl.def_readwrite("nRays", &mrpt::obs::T2DScanProperties::nRays);
		cl.def_readwrite("aperture", &mrpt::obs::T2DScanProperties::aperture);
		cl.def_readwrite("rightToLeft", &mrpt::obs::T2DScanProperties::rightToLeft);
		cl.def("assign", (struct mrpt::obs::T2DScanProperties & (mrpt::obs::T2DScanProperties::*)(const struct mrpt::obs::T2DScanProperties &)) &mrpt::obs::T2DScanProperties::operator=, "C++: mrpt::obs::T2DScanProperties::operator=(const struct mrpt::obs::T2DScanProperties &) --> struct mrpt::obs::T2DScanProperties &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::CSinCosLookUpTableFor2DScans file:mrpt/obs/CSinCosLookUpTableFor2DScans.h line:28
		pybind11::class_<mrpt::obs::CSinCosLookUpTableFor2DScans, std::shared_ptr<mrpt::obs::CSinCosLookUpTableFor2DScans>> cl(M("mrpt::obs"), "CSinCosLookUpTableFor2DScans", "A smart look-up-table (LUT) of sin/cos values for 2D laser scans.\n  Refer to the main method CSinCosLookUpTableFor2DScans::getSinCosForScan()\n\n  This class is used in mrpt::maps::CPointsMap\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CSinCosLookUpTableFor2DScans(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CSinCosLookUpTableFor2DScans const &o){ return new mrpt::obs::CSinCosLookUpTableFor2DScans(o); } ) );
		cl.def("assign", (class mrpt::obs::CSinCosLookUpTableFor2DScans & (mrpt::obs::CSinCosLookUpTableFor2DScans::*)(const class mrpt::obs::CSinCosLookUpTableFor2DScans &)) &mrpt::obs::CSinCosLookUpTableFor2DScans::operator=, "C++: mrpt::obs::CSinCosLookUpTableFor2DScans::operator=(const class mrpt::obs::CSinCosLookUpTableFor2DScans &) --> class mrpt::obs::CSinCosLookUpTableFor2DScans &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("getSinCosForScan", (const struct mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues & (mrpt::obs::CSinCosLookUpTableFor2DScans::*)(const class mrpt::obs::CObservation2DRangeScan &) const) &mrpt::obs::CSinCosLookUpTableFor2DScans::getSinCosForScan, "Return two vectors with the cos and the sin of the angles for each of\n the\n rays in a scan, computing them only the first time and returning a\n cached copy the rest.\n  Usage:\n \n\n\n\n\n\n\n   \n\nC++: mrpt::obs::CSinCosLookUpTableFor2DScans::getSinCosForScan(const class mrpt::obs::CObservation2DRangeScan &) const --> const struct mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues &", pybind11::return_value_policy::automatic, pybind11::arg("scan"));
		cl.def("getSinCosForScan", (const struct mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues & (mrpt::obs::CSinCosLookUpTableFor2DScans::*)(const struct mrpt::obs::T2DScanProperties &) const) &mrpt::obs::CSinCosLookUpTableFor2DScans::getSinCosForScan, "C++: mrpt::obs::CSinCosLookUpTableFor2DScans::getSinCosForScan(const struct mrpt::obs::T2DScanProperties &) const --> const struct mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues &", pybind11::return_value_policy::automatic, pybind11::arg("scan_prop"));

		{ // mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues file:mrpt/obs/CSinCosLookUpTableFor2DScans.h line:37
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues, std::shared_ptr<mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues>> cl(enclosing_class, "TSinCosValues", "A pair of vectors with the cos and sin values. ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues const &o){ return new mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues(o); } ) );
			cl.def_readwrite("ccos", &mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues::ccos);
			cl.def_readwrite("csin", &mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues::csin);
		}

	}
}
