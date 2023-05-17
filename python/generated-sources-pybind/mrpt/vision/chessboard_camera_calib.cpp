#include <any>
#include <chrono>
#include <functional>
#include <ios>
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
#include <mrpt/core/safe_pointers.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/io/CStream.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/T2DScanProperties.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
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
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/chessboard_camera_calib.h>
#include <mrpt/vision/types.h>
#include <mrpt/vision/utils.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_vision_chessboard_camera_calib(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::vision::TImageCalibData file:mrpt/vision/chessboard_camera_calib.h line:28
		pybind11::class_<mrpt::vision::TImageCalibData, std::shared_ptr<mrpt::vision::TImageCalibData>> cl(M("mrpt::vision"), "TImageCalibData", "Data associated to each image in the calibration process\n mrpt::vision::checkerBoardCameraCalibration (All the information can be left\n empty and will be filled up in the calibration method).");
		cl.def( pybind11::init( [](mrpt::vision::TImageCalibData const &o){ return new mrpt::vision::TImageCalibData(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::vision::TImageCalibData(); } ) );
		cl.def_readwrite("img_original", &mrpt::vision::TImageCalibData::img_original);
		cl.def_readwrite("img_checkboard", &mrpt::vision::TImageCalibData::img_checkboard);
		cl.def_readwrite("img_rectified", &mrpt::vision::TImageCalibData::img_rectified);
		cl.def_readwrite("detected_corners", &mrpt::vision::TImageCalibData::detected_corners);
		cl.def_readwrite("reconstructed_camera_pose", &mrpt::vision::TImageCalibData::reconstructed_camera_pose);
		cl.def_readwrite("projectedPoints_distorted", &mrpt::vision::TImageCalibData::projectedPoints_distorted);
		cl.def_readwrite("projectedPoints_undistorted", &mrpt::vision::TImageCalibData::projectedPoints_undistorted);
		cl.def("clear", (void (mrpt::vision::TImageCalibData::*)()) &mrpt::vision::TImageCalibData::clear, "Empty all the data \n\nC++: mrpt::vision::TImageCalibData::clear() --> void");
		cl.def("assign", (struct mrpt::vision::TImageCalibData & (mrpt::vision::TImageCalibData::*)(const struct mrpt::vision::TImageCalibData &)) &mrpt::vision::TImageCalibData::operator=, "C++: mrpt::vision::TImageCalibData::operator=(const struct mrpt::vision::TImageCalibData &) --> struct mrpt::vision::TImageCalibData &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::vision::openCV_cross_correlation(const class mrpt::img::CImage &, const class mrpt::img::CImage &, unsigned long &, unsigned long &, double &, int, int, int, int) file:mrpt/vision/utils.h line:63
	M("mrpt::vision").def("openCV_cross_correlation", [](const class mrpt::img::CImage & a0, const class mrpt::img::CImage & a1, unsigned long & a2, unsigned long & a3, double & a4) -> void { return mrpt::vision::openCV_cross_correlation(a0, a1, a2, a3, a4); }, "", pybind11::arg("img"), pybind11::arg("patch_img"), pybind11::arg("x_max"), pybind11::arg("y_max"), pybind11::arg("max_val"));
	M("mrpt::vision").def("openCV_cross_correlation", [](const class mrpt::img::CImage & a0, const class mrpt::img::CImage & a1, unsigned long & a2, unsigned long & a3, double & a4, int const & a5) -> void { return mrpt::vision::openCV_cross_correlation(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("img"), pybind11::arg("patch_img"), pybind11::arg("x_max"), pybind11::arg("y_max"), pybind11::arg("max_val"), pybind11::arg("x_search_ini"));
	M("mrpt::vision").def("openCV_cross_correlation", [](const class mrpt::img::CImage & a0, const class mrpt::img::CImage & a1, unsigned long & a2, unsigned long & a3, double & a4, int const & a5, int const & a6) -> void { return mrpt::vision::openCV_cross_correlation(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("img"), pybind11::arg("patch_img"), pybind11::arg("x_max"), pybind11::arg("y_max"), pybind11::arg("max_val"), pybind11::arg("x_search_ini"), pybind11::arg("y_search_ini"));
	M("mrpt::vision").def("openCV_cross_correlation", [](const class mrpt::img::CImage & a0, const class mrpt::img::CImage & a1, unsigned long & a2, unsigned long & a3, double & a4, int const & a5, int const & a6, int const & a7) -> void { return mrpt::vision::openCV_cross_correlation(a0, a1, a2, a3, a4, a5, a6, a7); }, "", pybind11::arg("img"), pybind11::arg("patch_img"), pybind11::arg("x_max"), pybind11::arg("y_max"), pybind11::arg("max_val"), pybind11::arg("x_search_ini"), pybind11::arg("y_search_ini"), pybind11::arg("x_search_size"));
	M("mrpt::vision").def("openCV_cross_correlation", (void (*)(const class mrpt::img::CImage &, const class mrpt::img::CImage &, unsigned long &, unsigned long &, double &, int, int, int, int)) &mrpt::vision::openCV_cross_correlation, "	Computes the correlation between this image and another one, encapsulating\n the openCV function cvMatchTemplate\n   This implementation reduced computation time.\n \n\n            [IN]    The imput image. This function supports\n gray-scale (1 channel only) images.\n \n\n      [IN]    The \"patch\" image, which must be equal, or\n smaller than \"this\" image. This function supports gray-scale (1 channel only)\n images.\n \n\n          [OUT]   The x coordinate where it was found the maximun\n cross correlation value.\n \n\n          [OUT]   The y coordinate where it was found the maximun\n cross correlation value.\n \n\n        [OUT]   The maximun value of cross correlation which we\n can find\n \n\n   [IN]    The \"x\" coordinate of the search window.\n \n\n   [IN]    The \"y\" coordinate of the search window.\n \n\n  [IN]    The width of the search window.\n \n\n  [IN]    The height of the search window.\n  Note: By default, the search area is the whole (this) image.\n \n\n cross_correlation\n\nC++: mrpt::vision::openCV_cross_correlation(const class mrpt::img::CImage &, const class mrpt::img::CImage &, unsigned long &, unsigned long &, double &, int, int, int, int) --> void", pybind11::arg("img"), pybind11::arg("patch_img"), pybind11::arg("x_max"), pybind11::arg("y_max"), pybind11::arg("max_val"), pybind11::arg("x_search_ini"), pybind11::arg("y_search_ini"), pybind11::arg("x_search_size"), pybind11::arg("y_search_size"));

	// mrpt::vision::computeMsd(const class mrpt::tfest::TMatchingPairListTempl<float> &, const class mrpt::poses::CPose3D &) file:mrpt/vision/utils.h line:102
	M("mrpt::vision").def("computeMsd", (double (*)(const class mrpt::tfest::TMatchingPairListTempl<float> &, const class mrpt::poses::CPose3D &)) &mrpt::vision::computeMsd, "Computes the mean squared distance between a set of 3D correspondences\n ...\n\nC++: mrpt::vision::computeMsd(const class mrpt::tfest::TMatchingPairListTempl<float> &, const class mrpt::poses::CPose3D &) --> double", pybind11::arg("list"), pybind11::arg("Rt"));

	// mrpt::vision::computeMainOrientation(const class mrpt::img::CImage &, unsigned int, unsigned int) file:mrpt/vision/utils.h line:122
	M("mrpt::vision").def("computeMainOrientation", (float (*)(const class mrpt::img::CImage &, unsigned int, unsigned int)) &mrpt::vision::computeMainOrientation, "Computes the main orientation of a set of points with an image (for using in\n SIFT-based algorithms)\n \n\n    [IN] The input image.\n \n\n        [IN] A vector containing the 'x' coordinates of the image\n points.\n \n\n        [IN] A vector containing the 'y' coordinates of the image\n points.\n \n\n The main orientation of the image point.\n\nC++: mrpt::vision::computeMainOrientation(const class mrpt::img::CImage &, unsigned int, unsigned int) --> float", pybind11::arg("image"), pybind11::arg("x"), pybind11::arg("y"));

	// mrpt::vision::normalizeImage(const class mrpt::img::CImage &, class mrpt::img::CImage &) file:mrpt/vision/utils.h line:130
	M("mrpt::vision").def("normalizeImage", (void (*)(const class mrpt::img::CImage &, class mrpt::img::CImage &)) &mrpt::vision::normalizeImage, "Normalizes the brigthness and contrast of an image by setting its mean value\n to zero and its standard deviation to unit.\n \n\n        [IN]        The input image.\n \n\n       [OUTPUT]    The new normalized image.\n\nC++: mrpt::vision::normalizeImage(const class mrpt::img::CImage &, class mrpt::img::CImage &) --> void", pybind11::arg("image"), pybind11::arg("nimage"));

	// mrpt::vision::matchFeatures(const class mrpt::vision::CFeatureList &, const class mrpt::vision::CFeatureList &, class mrpt::vision::CMatchedFeatureList &, const struct mrpt::vision::TMatchingOptions &, const struct mrpt::vision::TStereoSystemParams &) file:mrpt/vision/utils.h line:140
	M("mrpt::vision").def("matchFeatures", [](const class mrpt::vision::CFeatureList & a0, const class mrpt::vision::CFeatureList & a1, class mrpt::vision::CMatchedFeatureList & a2) -> size_t { return mrpt::vision::matchFeatures(a0, a1, a2); }, "", pybind11::arg("list1"), pybind11::arg("list2"), pybind11::arg("matches"));
	M("mrpt::vision").def("matchFeatures", [](const class mrpt::vision::CFeatureList & a0, const class mrpt::vision::CFeatureList & a1, class mrpt::vision::CMatchedFeatureList & a2, const struct mrpt::vision::TMatchingOptions & a3) -> size_t { return mrpt::vision::matchFeatures(a0, a1, a2, a3); }, "", pybind11::arg("list1"), pybind11::arg("list2"), pybind11::arg("matches"), pybind11::arg("options"));
	M("mrpt::vision").def("matchFeatures", (size_t (*)(const class mrpt::vision::CFeatureList &, const class mrpt::vision::CFeatureList &, class mrpt::vision::CMatchedFeatureList &, const struct mrpt::vision::TMatchingOptions &, const struct mrpt::vision::TStereoSystemParams &)) &mrpt::vision::matchFeatures, "Find the matches between two lists of features which must be of the same\n type.\n \n\n    [IN]    One list.\n \n\n    [IN]    Other list.\n \n\n  [OUT]   A vector of pairs of correspondences.\n \n\n  [IN]    A struct containing matching options\n \n\n Returns the number of matched pairs of features.\n\nC++: mrpt::vision::matchFeatures(const class mrpt::vision::CFeatureList &, const class mrpt::vision::CFeatureList &, class mrpt::vision::CMatchedFeatureList &, const struct mrpt::vision::TMatchingOptions &, const struct mrpt::vision::TStereoSystemParams &) --> size_t", pybind11::arg("list1"), pybind11::arg("list2"), pybind11::arg("matches"), pybind11::arg("options"), pybind11::arg("params"));

	// mrpt::vision::computeSAD(const class mrpt::img::CImage &, const class mrpt::img::CImage &) file:mrpt/vision/utils.h line:164
	M("mrpt::vision").def("computeSAD", (double (*)(const class mrpt::img::CImage &, const class mrpt::img::CImage &)) &mrpt::vision::computeSAD, "Calculates the Sum of Absolutes Differences (range [0,1]) between two\n patches. Both patches must have the same size.\n \n\n [IN]  One patch.\n \n\n [IN]  The other patch.\n \n\n The value of computed SAD normalized to [0,1]\n\nC++: mrpt::vision::computeSAD(const class mrpt::img::CImage &, const class mrpt::img::CImage &) --> double", pybind11::arg("patch1"), pybind11::arg("patch2"));

	// mrpt::vision::addFeaturesToImage(const class mrpt::img::CImage &, const class mrpt::vision::CFeatureList &, class mrpt::img::CImage &) file:mrpt/vision/utils.h line:173
	M("mrpt::vision").def("addFeaturesToImage", (void (*)(const class mrpt::img::CImage &, const class mrpt::vision::CFeatureList &, class mrpt::img::CImage &)) &mrpt::vision::addFeaturesToImage, "Draw rectangles around each of the features on a copy of the input image.\n \n\n    [IN]    The input image where to draw the features.\n \n\n  [IN]    The list of features.\n \n\n   [OUT]   The copy of the input image with the marked\n features.\n\nC++: mrpt::vision::addFeaturesToImage(const class mrpt::img::CImage &, const class mrpt::vision::CFeatureList &, class mrpt::img::CImage &) --> void", pybind11::arg("inImg"), pybind11::arg("theList"), pybind11::arg("outImg"));

	// mrpt::vision::projectMatchedFeature(const class mrpt::vision::CFeature &, const class mrpt::vision::CFeature &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::vision::TStereoSystemParams &) file:mrpt/vision/utils.h line:204
	M("mrpt::vision").def("projectMatchedFeature", [](const class mrpt::vision::CFeature & a0, const class mrpt::vision::CFeature & a1, struct mrpt::math::TPoint3D_<double> & a2) -> void { return mrpt::vision::projectMatchedFeature(a0, a1, a2); }, "", pybind11::arg("leftFeat"), pybind11::arg("rightFeat"), pybind11::arg("p3D"));
	M("mrpt::vision").def("projectMatchedFeature", (void (*)(const class mrpt::vision::CFeature &, const class mrpt::vision::CFeature &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::vision::TStereoSystemParams &)) &mrpt::vision::projectMatchedFeature, "Computes the 3D position of a particular matched feature.\n \n\n     [IN]    The left feature.\n \n\n    [IN]    The right feature.\n \n\n         [OUT]   The 3D position of the projected point.\n \n\n       [IN]    The intrinsic and extrinsic parameters of the\n stereo pair.\n\nC++: mrpt::vision::projectMatchedFeature(const class mrpt::vision::CFeature &, const class mrpt::vision::CFeature &, struct mrpt::math::TPoint3D_<double> &, const struct mrpt::vision::TStereoSystemParams &) --> void", pybind11::arg("leftFeat"), pybind11::arg("rightFeat"), pybind11::arg("p3D"), pybind11::arg("params"));

	// mrpt::vision::projectMatchedFeatures(class mrpt::vision::CMatchedFeatureList &, const struct mrpt::vision::TStereoSystemParams &, class mrpt::maps::CLandmarksMap &) file:mrpt/vision/utils.h line:218
	M("mrpt::vision").def("projectMatchedFeatures", (void (*)(class mrpt::vision::CMatchedFeatureList &, const struct mrpt::vision::TStereoSystemParams &, class mrpt::maps::CLandmarksMap &)) &mrpt::vision::projectMatchedFeatures, "Project a list of matched features into the 3D space, using the provided\n parameters of the stereo system\n \n\n       [IN/OUT]    The list of matched features. Features which\n yields a 3D point outside the area defined in TStereoSystemParams are removed\n from the lists.\n \n\n        [IN]        The parameters of the stereo system.\n \n\n    [OUT]       A map containing the projected landmarks.\n \n\n TStereoSystemParams, CLandmarksMap\n\nC++: mrpt::vision::projectMatchedFeatures(class mrpt::vision::CMatchedFeatureList &, const struct mrpt::vision::TStereoSystemParams &, class mrpt::maps::CLandmarksMap &) --> void", pybind11::arg("mfList"), pybind11::arg("param"), pybind11::arg("landmarks"));

	// mrpt::vision::projectMatchedFeatures(class mrpt::vision::CFeatureList &, class mrpt::vision::CFeatureList &, const struct mrpt::vision::TStereoSystemParams &, class mrpt::maps::CLandmarksMap &) file:mrpt/vision/utils.h line:233
	M("mrpt::vision").def("projectMatchedFeatures", (void (*)(class mrpt::vision::CFeatureList &, class mrpt::vision::CFeatureList &, const struct mrpt::vision::TStereoSystemParams &, class mrpt::maps::CLandmarksMap &)) &mrpt::vision::projectMatchedFeatures, "Project a pair of feature lists into the 3D space, using the provided\noptions for the stereo system. The matches must be in order,\n	i.e. leftList[0] corresponds to rightList[0] and so on. Features which\nyields a 3D point outside the area defined in TStereoSystemParams are removed\nfrom the lists.\n \n\n     [IN/OUT]    The left list of matched features.\n \n\n    [IN/OUT]    The right list of matched features.\n \n\n        [IN]        The options of the stereo system.\n \n\n    (OUT]       A map containing the projected landmarks.\n \n\n TStereoSystemParams, CLandmarksMap\n\nC++: mrpt::vision::projectMatchedFeatures(class mrpt::vision::CFeatureList &, class mrpt::vision::CFeatureList &, const struct mrpt::vision::TStereoSystemParams &, class mrpt::maps::CLandmarksMap &) --> void", pybind11::arg("leftList"), pybind11::arg("rightList"), pybind11::arg("param"), pybind11::arg("landmarks"));

	// mrpt::vision::computeStereoRectificationMaps(const class mrpt::img::TCamera &, const class mrpt::img::TCamera &, const class mrpt::poses::CPose3D &, void *, void *, void *, void *) file:mrpt/vision/utils.h line:298
	M("mrpt::vision").def("computeStereoRectificationMaps", (void (*)(const class mrpt::img::TCamera &, const class mrpt::img::TCamera &, const class mrpt::poses::CPose3D &, void *, void *, void *, void *)) &mrpt::vision::computeStereoRectificationMaps, "Computes a pair of x-and-y maps for stereo rectification from a pair of\ncameras and the relative pose of the second one wrt the first one.\n	\n\n cam2           [IN]    The pair of involved cameras\n	\n\n      [IN]    The change in pose of the second camera\nwrt the first one\n	\n\n    [OUT]   The x-and-y maps corresponding to cam1\n(should be converted to *cv::Mat)\n	\n\n    [OUT]   The x-and-y maps corresponding to cam2\n(should be converted to *cv::Mat)\n \n\n An easier to use class for stereo rectification\nmrpt::vision::CStereoRectifyMap\n\nC++: mrpt::vision::computeStereoRectificationMaps(const class mrpt::img::TCamera &, const class mrpt::img::TCamera &, const class mrpt::poses::CPose3D &, void *, void *, void *, void *) --> void", pybind11::arg("cam1"), pybind11::arg("cam2"), pybind11::arg("rightCameraPose"), pybind11::arg("outMap1x"), pybind11::arg("outMap1y"), pybind11::arg("outMap2x"), pybind11::arg("outMap2y"));

}
