#include <memory>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CWeightedPointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
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

void bind_mrpt_maps_CWeightedPointsMap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::PointCloudAdapter file:mrpt/maps/CWeightedPointsMap.h line:160
		pybind11::class_<mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>, std::shared_ptr<mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>>> cl(M("mrpt::opengl"), "PointCloudAdapter_mrpt_maps_CWeightedPointsMap_t", "Specialization\n mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>\n \n");
		cl.def( pybind11::init<const class mrpt::maps::CWeightedPointsMap &>(), pybind11::arg("obj") );

		cl.def("size", (size_t (mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::*)() const) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::size, "Get number of points \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::size() const --> size_t");
		cl.def("resize", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::resize, "Set number of points (to uninitialized values) \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("setDimensions", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::*)(size_t, size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::setDimensions, "Does nothing as of now \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::setDimensions(size_t, size_t) --> void", pybind11::arg("height"), pybind11::arg("width"));
		cl.def("setPointXYZ", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::setPointXYZ, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CWeightedPointsMap>::setPointXYZ(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
	}
}
