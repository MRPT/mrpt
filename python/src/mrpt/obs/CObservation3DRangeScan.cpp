#include <iterator>
#include <memory>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <string>

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

void bind_mrpt_obs_CObservation3DRangeScan(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::opengl::PointCloudAdapter file:mrpt/obs/CObservation3DRangeScan.h line:602
		pybind11::class_<mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>, std::shared_ptr<mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>>> cl(M("mrpt::opengl"), "PointCloudAdapter_mrpt_obs_CObservation3DRangeScan_t", "Specialization mrpt::opengl::PointCloudAdapter<CObservation3DRangeScan>\n \n");
		cl.def( pybind11::init<const class mrpt::obs::CObservation3DRangeScan &>(), pybind11::arg("obj") );

		cl.def("size", (size_t (mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::*)() const) &mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::size, "Get number of points \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::size() const --> size_t");
		cl.def("resize", (void (mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::resize, "Set number of points (to uninitialized values) \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("setDimensions", (void (mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::*)(size_t, size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::setDimensions, "Does nothing as of now \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::setDimensions(size_t, size_t) --> void", pybind11::arg("height"), pybind11::arg("width"));
		cl.def("setPointXYZ", (void (mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::*)(size_t, const float, const float, const float)) &mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::setPointXYZ, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::setPointXYZ(size_t, const float, const float, const float) --> void", pybind11::arg("idx"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("setInvalidPoint", (void (mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::*)(size_t)) &mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::setInvalidPoint, "Set XYZ coordinates of i'th point \n\nC++: mrpt::opengl::PointCloudAdapter<mrpt::obs::CObservation3DRangeScan>::setInvalidPoint(size_t) --> void", pybind11::arg("idx"));
	}
}
