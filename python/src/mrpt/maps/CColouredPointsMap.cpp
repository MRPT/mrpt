#include <iterator>
#include <memory>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
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

void bind_mrpt_maps_CColouredPointsMap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::PointCloudAdapter file:mrpt/maps/CColouredPointsMap.h line:373
		pybind11::class_<mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>, std::shared_ptr<mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>>> cl(M("mrpt::opengl"), "PointCloudAdapter_mrpt_maps_CColouredPointsMap_t", "Specialization\n mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap> \n\n\n mrpt_adapters_grp ");
		cl.def( pybind11::init<const class mrpt::maps::CColouredPointsMap &>(), pybind11::arg("obj") );

		cl.def("size", (size_t (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)() const) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::size, "Get number of points \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::size() const --> size_t");
		cl.def("resize", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(const unsigned long)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::resize, "Set number of points (to uninitialized values) \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::resize(const unsigned long) --> void", pybind11::arg("N"));
		cl.def("setDimensions", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(size_t, size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setDimensions, "Does nothing as of now \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setDimensions(size_t, size_t) --> void", pybind11::arg("height"), pybind11::arg("width"));
		cl.def("setPointXYZ", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointXYZ, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointXYZ(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setPointXYZ_RGBAf", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(size_t, const float, const float, const float, const float, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointXYZ_RGBAf, "Set XYZ_RGBf coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointXYZ_RGBAf(size_t, const float, const float, const float, const float, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("setPointXYZ_RGBu8", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(size_t, const float, const float, const float, const unsigned char, const unsigned char, const unsigned char)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointXYZ_RGBu8, "Set XYZ_RGBu8 coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointXYZ_RGBu8(size_t, const float, const float, const float, const unsigned char, const unsigned char, const unsigned char) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("getPointRGBf", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(size_t, float &, float &, float &) const) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::getPointRGBf, "Get RGBf color of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::getPointRGBf(size_t, float &, float &, float &) const --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setPointRGBf", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointRGBf, "Set XYZ_RGBf coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointRGBf(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("getPointRGBu8", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(size_t, unsigned char &, unsigned char &, unsigned char &) const) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::getPointRGBu8, "Get RGBu8 color of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::getPointRGBu8(size_t, unsigned char &, unsigned char &, unsigned char &) const --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setPointRGBu8", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(size_t, const unsigned char, const unsigned char, const unsigned char)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointRGBu8, "Set RGBu8 coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setPointRGBu8(size_t, const unsigned char, const unsigned char, const unsigned char) --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setInvalidPoint", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setInvalidPoint, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CColouredPointsMap>::setInvalidPoint(size_t) --> void", pybind11::arg("idx"));
	}
}
