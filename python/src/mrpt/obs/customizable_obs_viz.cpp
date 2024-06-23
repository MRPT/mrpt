#include <any>
#include <chrono>
#include <functional>
#include <istream>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/CommentPosition.h>
#include <mrpt/containers/YamlEmitOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
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

void bind_mrpt_obs_customizable_obs_viz(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::VisualizationParameters file:mrpt/obs/customizable_obs_viz.h line:31
		pybind11::class_<mrpt::obs::VisualizationParameters, std::shared_ptr<mrpt::obs::VisualizationParameters>> cl(M("mrpt::obs"), "VisualizationParameters", "Here we can customize the way observations will be rendered as 3D objects\n  in obs_to_viz(), obs3Dscan_to_viz(), etc.\n  \n\n obs_to_viz(), obs3Dscan_to_viz()");
		cl.def( pybind11::init( [](){ return new mrpt::obs::VisualizationParameters(); } ) );
		cl.def( pybind11::init( [](mrpt::obs::VisualizationParameters const &o){ return new mrpt::obs::VisualizationParameters(o); } ) );
		cl.def_readwrite("showAxis", &mrpt::obs::VisualizationParameters::showAxis);
		cl.def_readwrite("axisTickFrequency", &mrpt::obs::VisualizationParameters::axisTickFrequency);
		cl.def_readwrite("axisLimits", &mrpt::obs::VisualizationParameters::axisLimits);
		cl.def_readwrite("axisTickTextSize", &mrpt::obs::VisualizationParameters::axisTickTextSize);
		cl.def_readwrite("colorFromRGBimage", &mrpt::obs::VisualizationParameters::colorFromRGBimage);
		cl.def_readwrite("colorizeByAxis", &mrpt::obs::VisualizationParameters::colorizeByAxis);
		cl.def_readwrite("invertColorMapping", &mrpt::obs::VisualizationParameters::invertColorMapping);
		cl.def_readwrite("colorMap", &mrpt::obs::VisualizationParameters::colorMap);
		cl.def_readwrite("pointSize", &mrpt::obs::VisualizationParameters::pointSize);
		cl.def_readwrite("drawSensorPose", &mrpt::obs::VisualizationParameters::drawSensorPose);
		cl.def_readwrite("sensorPoseScale", &mrpt::obs::VisualizationParameters::sensorPoseScale);
		cl.def_readwrite("onlyPointsWithColor", &mrpt::obs::VisualizationParameters::onlyPointsWithColor);
		cl.def_readwrite("showSurfaceIn2Dscans", &mrpt::obs::VisualizationParameters::showSurfaceIn2Dscans);
		cl.def_readwrite("showPointsIn2Dscans", &mrpt::obs::VisualizationParameters::showPointsIn2Dscans);
		cl.def_readwrite("surface2DscansColor", &mrpt::obs::VisualizationParameters::surface2DscansColor);
		cl.def_readwrite("points2DscansColor", &mrpt::obs::VisualizationParameters::points2DscansColor);
		cl.def("save_to_ini_file", [](mrpt::obs::VisualizationParameters const &o, class mrpt::config::CConfigFileBase & a0) -> void { return o.save_to_ini_file(a0); }, "", pybind11::arg("cfg"));
		cl.def("save_to_ini_file", (void (mrpt::obs::VisualizationParameters::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::obs::VisualizationParameters::save_to_ini_file, "C++: mrpt::obs::VisualizationParameters::save_to_ini_file(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("cfg"), pybind11::arg("section"));
		cl.def("load_from_ini_file", [](mrpt::obs::VisualizationParameters &o, const class mrpt::config::CConfigFileBase & a0) -> void { return o.load_from_ini_file(a0); }, "", pybind11::arg("cfg"));
		cl.def("load_from_ini_file", (void (mrpt::obs::VisualizationParameters::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::obs::VisualizationParameters::load_from_ini_file, "C++: mrpt::obs::VisualizationParameters::load_from_ini_file(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("cfg"), pybind11::arg("section"));
		cl.def("assign", (struct mrpt::obs::VisualizationParameters & (mrpt::obs::VisualizationParameters::*)(const struct mrpt::obs::VisualizationParameters &)) &mrpt::obs::VisualizationParameters::operator=, "C++: mrpt::obs::VisualizationParameters::operator=(const struct mrpt::obs::VisualizationParameters &) --> struct mrpt::obs::VisualizationParameters &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::obs::obs_to_viz(const class std::shared_ptr<class mrpt::obs::CObservation> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) file:mrpt/obs/customizable_obs_viz.h line:72
	M("mrpt::obs").def("obs_to_viz", (bool (*)(const class std::shared_ptr<class mrpt::obs::CObservation> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &)) &mrpt::obs::obs_to_viz, "Clears `out` and creates a visualization of the given observation,\n  dispatching the call according to the actual observation class.\n  \n\n true if type has known visualizer, false if it does not (then, `out`\n          will be empty)\n\n  \n This and the accompanying functions are defined in namespace\n        mrpt::obs, but you must link against mrpt::maps too to have their\n        definitions.\n\nC++: mrpt::obs::obs_to_viz(const class std::shared_ptr<class mrpt::obs::CObservation> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) --> bool", pybind11::arg("obs"), pybind11::arg("p"), pybind11::arg("out"));

	// mrpt::obs::obs_to_viz(const class mrpt::obs::CSensoryFrame &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) file:mrpt/obs/customizable_obs_viz.h line:88
	M("mrpt::obs").def("obs_to_viz", (bool (*)(const class mrpt::obs::CSensoryFrame &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &)) &mrpt::obs::obs_to_viz, "Clears `out` and creates a visualization of the given sensory-frame,\n  dispatching the call according to the actual observation classes inside the\n SF.\n\n  \n true if type has known visualizer, false if it does not (then, `out`\n          will be empty)\n\n  \n This and the accompanying functions are defined in namespace\n        mrpt::obs, but you must link against mrpt::maps too to have their\n        definitions.\n\nC++: mrpt::obs::obs_to_viz(const class mrpt::obs::CSensoryFrame &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) --> bool", pybind11::arg("sf"), pybind11::arg("p"), pybind11::arg("out"));

	// mrpt::obs::obs3Dscan_to_viz(const class std::shared_ptr<class mrpt::obs::CObservation3DRangeScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) file:mrpt/obs/customizable_obs_viz.h line:94
	M("mrpt::obs").def("obs3Dscan_to_viz", (void (*)(const class std::shared_ptr<class mrpt::obs::CObservation3DRangeScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &)) &mrpt::obs::obs3Dscan_to_viz, "Clears `out` and creates a visualization of the given observation.\n\nC++: mrpt::obs::obs3Dscan_to_viz(const class std::shared_ptr<class mrpt::obs::CObservation3DRangeScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) --> void", pybind11::arg("obs"), pybind11::arg("p"), pybind11::arg("out"));

	// mrpt::obs::obsVelodyne_to_viz(const class std::shared_ptr<class mrpt::obs::CObservationVelodyneScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) file:mrpt/obs/customizable_obs_viz.h line:100
	M("mrpt::obs").def("obsVelodyne_to_viz", (void (*)(const class std::shared_ptr<class mrpt::obs::CObservationVelodyneScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &)) &mrpt::obs::obsVelodyne_to_viz, "Clears `out` and creates a visualization of the given observation.\n\nC++: mrpt::obs::obsVelodyne_to_viz(const class std::shared_ptr<class mrpt::obs::CObservationVelodyneScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) --> void", pybind11::arg("obs"), pybind11::arg("p"), pybind11::arg("out"));

	// mrpt::obs::obsPointCloud_to_viz(const class std::shared_ptr<class mrpt::obs::CObservationPointCloud> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) file:mrpt/obs/customizable_obs_viz.h line:106
	M("mrpt::obs").def("obsPointCloud_to_viz", (void (*)(const class std::shared_ptr<class mrpt::obs::CObservationPointCloud> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &)) &mrpt::obs::obsPointCloud_to_viz, "Clears `out` and creates a visualization of the given observation.\n\nC++: mrpt::obs::obsPointCloud_to_viz(const class std::shared_ptr<class mrpt::obs::CObservationPointCloud> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) --> void", pybind11::arg("obs"), pybind11::arg("p"), pybind11::arg("out"));

	// mrpt::obs::obsRotatingScan_to_viz(const class std::shared_ptr<class mrpt::obs::CObservationRotatingScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) file:mrpt/obs/customizable_obs_viz.h line:111
	M("mrpt::obs").def("obsRotatingScan_to_viz", (void (*)(const class std::shared_ptr<class mrpt::obs::CObservationRotatingScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &)) &mrpt::obs::obsRotatingScan_to_viz, "C++: mrpt::obs::obsRotatingScan_to_viz(const class std::shared_ptr<class mrpt::obs::CObservationRotatingScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) --> void", pybind11::arg("obs"), pybind11::arg("p"), pybind11::arg("out"));

	// mrpt::obs::obs2Dscan_to_viz(const class std::shared_ptr<class mrpt::obs::CObservation2DRangeScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) file:mrpt/obs/customizable_obs_viz.h line:117
	M("mrpt::obs").def("obs2Dscan_to_viz", (void (*)(const class std::shared_ptr<class mrpt::obs::CObservation2DRangeScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &)) &mrpt::obs::obs2Dscan_to_viz, "Clears `out` and creates a visualization of the given observation.\n\nC++: mrpt::obs::obs2Dscan_to_viz(const class std::shared_ptr<class mrpt::obs::CObservation2DRangeScan> &, const struct mrpt::obs::VisualizationParameters &, class mrpt::opengl::CSetOfObjects &) --> void", pybind11::arg("obs"), pybind11::arg("p"), pybind11::arg("out"));

	// mrpt::obs::recolorize3Dpc(const class std::shared_ptr<class mrpt::opengl::CPointCloudColoured> &, const struct mrpt::obs::VisualizationParameters &) file:mrpt/obs/customizable_obs_viz.h line:123
	M("mrpt::obs").def("recolorize3Dpc", (void (*)(const class std::shared_ptr<class mrpt::opengl::CPointCloudColoured> &, const struct mrpt::obs::VisualizationParameters &)) &mrpt::obs::recolorize3Dpc, "Recolorize a pointcloud according to the given parameters\n\nC++: mrpt::obs::recolorize3Dpc(const class std::shared_ptr<class mrpt::opengl::CPointCloudColoured> &, const struct mrpt::obs::VisualizationParameters &) --> void", pybind11::arg("pnts"), pybind11::arg("p"));

}
