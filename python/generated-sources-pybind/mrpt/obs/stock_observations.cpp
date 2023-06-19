#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <streambuf>
#include <string>
#include <type_traits>
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

void bind_mrpt_obs_stock_observations(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::obs::stock_observations::example2DRangeScan(class mrpt::obs::CObservation2DRangeScan &, int) file:mrpt/obs/stock_observations.h line:23
	M("mrpt::obs::stock_observations").def("example2DRangeScan", [](class mrpt::obs::CObservation2DRangeScan & a0) -> void { return mrpt::obs::stock_observations::example2DRangeScan(a0); }, "", pybind11::arg("s"));
	M("mrpt::obs::stock_observations").def("example2DRangeScan", (void (*)(class mrpt::obs::CObservation2DRangeScan &, int)) &mrpt::obs::stock_observations::example2DRangeScan, "Example 2D lidar scans (form a venerable SICK LMS200).\n Implemented indices: 0,1.\n \n\n\n \n\nC++: mrpt::obs::stock_observations::example2DRangeScan(class mrpt::obs::CObservation2DRangeScan &, int) --> void", pybind11::arg("s"), pybind11::arg("i"));

	// mrpt::obs::stock_observations::exampleImage(class mrpt::img::CImage &, int) file:mrpt/obs/stock_observations.h line:29
	M("mrpt::obs::stock_observations").def("exampleImage", [](class mrpt::img::CImage & a0) -> void { return mrpt::obs::stock_observations::exampleImage(a0); }, "", pybind11::arg("im"));
	M("mrpt::obs::stock_observations").def("exampleImage", (void (*)(class mrpt::img::CImage &, int)) &mrpt::obs::stock_observations::exampleImage, "Example images (an 800x640 image pair from a Bumblebee 1)\n Implemented indices: 0,1.\n \n\n\n \n\nC++: mrpt::obs::stock_observations::exampleImage(class mrpt::img::CImage &, int) --> void", pybind11::arg("im"), pybind11::arg("i"));

}
