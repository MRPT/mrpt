#include <iterator>
#include <memory>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <optional>
#include <sstream> // __str__
#include <string>
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

void bind_mrpt_rtti_CObject_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::obs::CSensoryFrame>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::obs::CSensoryFrame>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_obs_CSensoryFrame_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::obs::CSensoryFrame>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::obs::CSensoryFrame>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::obs::CSensoryFrame>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
	{ // mrpt::rtti::CLASS_ID_impl file:mrpt/rtti/CObject.h line:91
		pybind11::class_<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CLandmarksMap>, std::shared_ptr<mrpt::rtti::CLASS_ID_impl<mrpt::maps::CLandmarksMap>>> cl(M("mrpt::rtti"), "CLASS_ID_impl_mrpt_maps_CLandmarksMap_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::rtti::CLASS_ID_impl<mrpt::maps::CLandmarksMap>(); } ) );
		cl.def_static("get", (const struct mrpt::rtti::TRuntimeClassId * (*)()) &mrpt::rtti::CLASS_ID_impl<mrpt::maps::CLandmarksMap>::get, "C++: mrpt::rtti::CLASS_ID_impl<mrpt::maps::CLandmarksMap>::get() --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
	}
}
