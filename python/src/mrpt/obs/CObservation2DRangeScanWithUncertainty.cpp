#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation2DRangeScanWithUncertainty.h>
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

void bind_mrpt_obs_CObservation2DRangeScanWithUncertainty(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservation2DRangeScanWithUncertainty file:mrpt/obs/CObservation2DRangeScanWithUncertainty.h line:19
		pybind11::class_<mrpt::obs::CObservation2DRangeScanWithUncertainty, std::shared_ptr<mrpt::obs::CObservation2DRangeScanWithUncertainty>> cl(M("mrpt::obs"), "CObservation2DRangeScanWithUncertainty", "A 2D range scan plus an uncertainty model for each range.\n \n\n mrpt::maps::COccupancyGridMap2D::laserScanSimulatorWithUncertainty()");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservation2DRangeScanWithUncertainty(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservation2DRangeScanWithUncertainty const &o){ return new mrpt::obs::CObservation2DRangeScanWithUncertainty(o); } ) );
		cl.def_readwrite("rangeScan", &mrpt::obs::CObservation2DRangeScanWithUncertainty::rangeScan);
		cl.def_readwrite("rangesMean", &mrpt::obs::CObservation2DRangeScanWithUncertainty::rangesMean);
		cl.def_readwrite("rangesCovar", &mrpt::obs::CObservation2DRangeScanWithUncertainty::rangesCovar);
		cl.def("evaluateScanLikelihood", (double (mrpt::obs::CObservation2DRangeScanWithUncertainty::*)(const class mrpt::obs::CObservation2DRangeScan &, const struct mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams &) const) &mrpt::obs::CObservation2DRangeScanWithUncertainty::evaluateScanLikelihood, "Returns a measure of the likelihood of a given scan, compared to this\n scan variances \n\nC++: mrpt::obs::CObservation2DRangeScanWithUncertainty::evaluateScanLikelihood(const class mrpt::obs::CObservation2DRangeScan &, const struct mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams &) const --> double", pybind11::arg("otherScan"), pybind11::arg("params"));
		cl.def("assign", (class mrpt::obs::CObservation2DRangeScanWithUncertainty & (mrpt::obs::CObservation2DRangeScanWithUncertainty::*)(const class mrpt::obs::CObservation2DRangeScanWithUncertainty &)) &mrpt::obs::CObservation2DRangeScanWithUncertainty::operator=, "C++: mrpt::obs::CObservation2DRangeScanWithUncertainty::operator=(const class mrpt::obs::CObservation2DRangeScanWithUncertainty &) --> class mrpt::obs::CObservation2DRangeScanWithUncertainty &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams file:mrpt/obs/CObservation2DRangeScanWithUncertainty.h line:30
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams, std::shared_ptr<mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams>> cl(enclosing_class, "TEvalParams", "");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams(); } ) );
			cl.def_readwrite("prob_outliers", &mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams::prob_outliers);
			cl.def_readwrite("prob_lost_ray", &mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams::prob_lost_ray);
			cl.def_readwrite("max_prediction_std_dev", &mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams::max_prediction_std_dev);
			cl.def_readwrite("min_ray_log_lik", &mrpt::obs::CObservation2DRangeScanWithUncertainty::TEvalParams::min_ray_log_lik);
		}

	}
}
