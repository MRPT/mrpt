#include <istream>
#include <iterator>
#include <memory>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <optional>
#include <ostream>
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

void bind_mrpt_maps_CPointsMap_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::PointCloudAdapter file:mrpt/maps/CPointsMap.h line:1291
		pybind11::class_<mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>, std::shared_ptr<mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>>> cl(M("mrpt::opengl"), "PointCloudAdapter_mrpt_maps_CPointsMap_t", "Specialization mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>\n \n");
		cl.def( pybind11::init<const class mrpt::maps::CPointsMap &>(), pybind11::arg("obj") );

		cl.def("size", (size_t (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::*)() const) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::size, "Get number of points \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::size() const --> size_t");
		cl.def("resize", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::resize, "Set number of points (to uninitialized values) \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("setDimensions", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::*)(size_t, size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::setDimensions, "Does nothing as of now \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::setDimensions(size_t, size_t) --> void", pybind11::arg("height"), pybind11::arg("width"));
		cl.def("setPointXYZ", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::setPointXYZ, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::setPointXYZ(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setInvalidPoint", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::setInvalidPoint, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMap>::setInvalidPoint(size_t) --> void", pybind11::arg("idx"));
	}
}
