#include <iterator>
#include <memory>
#include <mrpt/core/aligned_allocator.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
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

void bind_mrpt_maps_CPointsMapXYZI(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::PointCloudAdapter file:mrpt/maps/CPointsMapXYZI.h line:304
		pybind11::class_<mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>, std::shared_ptr<mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>>> cl(M("mrpt::opengl"), "PointCloudAdapter_mrpt_maps_CPointsMapXYZI_t", "Specialization\n mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>\n \n");
		cl.def( pybind11::init<const class mrpt::maps::CPointsMapXYZI &>(), pybind11::arg("obj") );

		cl.def("size", (size_t (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)() const) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::size, "Get number of points \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::size() const --> size_t");
		cl.def("resize", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::resize, "Set number of points (to uninitialized values) \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("setDimensions", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)(size_t, size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setDimensions, "Does nothing as of now \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setDimensions(size_t, size_t) --> void", pybind11::arg(""), pybind11::arg(""));
		cl.def("setPointXYZ", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointXYZ, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointXYZ(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setPointXYZ_RGBAf", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)(size_t, const float, const float, const float, const float, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointXYZ_RGBAf, "Set XYZ_RGBf coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointXYZ_RGBAf(size_t, const float, const float, const float, const float, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"), pybind11::arg("a"));
		cl.def("setPointXYZ_RGBu8", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)(size_t, const float, const float, const float, const unsigned char, const unsigned char, const unsigned char)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointXYZ_RGBu8, "Set XYZ_RGBu8 coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointXYZ_RGBu8(size_t, const float, const float, const float, const unsigned char, const unsigned char, const unsigned char) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("getPointRGBf", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)(size_t, float &, float &, float &) const) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::getPointRGBf, "Get RGBf color of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::getPointRGBf(size_t, float &, float &, float &) const --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setPointRGBf", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointRGBf, "Set XYZ_RGBf coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointRGBf(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("getPointRGBu8", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)(size_t, unsigned char &, unsigned char &, unsigned char &) const) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::getPointRGBu8, "Get RGBu8 color of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::getPointRGBu8(size_t, unsigned char &, unsigned char &, unsigned char &) const --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
		cl.def("setPointRGBu8", (void (mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::*)(size_t, const unsigned char, const unsigned char, const unsigned char)) &mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointRGBu8, "Set RGBu8 coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::maps::CPointsMapXYZI>::setPointRGBu8(size_t, const unsigned char, const unsigned char, const unsigned char) --> void", pybind11::arg("idx"), pybind11::arg("r"), pybind11::arg("g"), pybind11::arg("b"));
	}
}
