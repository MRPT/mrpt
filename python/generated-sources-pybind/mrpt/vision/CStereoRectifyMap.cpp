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
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <mrpt/vision/CStereoRectifyMap.h>
#include <ostream>
#include <sstream> // __str__
#include <string>
#include <type_traits>
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

void bind_mrpt_vision_CStereoRectifyMap(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::vision::CStereoRectifyMap file:mrpt/vision/CStereoRectifyMap.h line:74
		pybind11::class_<mrpt::vision::CStereoRectifyMap, std::shared_ptr<mrpt::vision::CStereoRectifyMap>> cl(M("mrpt::vision"), "CStereoRectifyMap", "Use this class to rectify stereo images if the same distortion maps are\n reused over and over again.\n  The rectify maps are cached internally and only computed once for the\n camera parameters.\n The stereo camera calibration must be supplied in a\n mrpt::util::TStereoCamera structure\n  (which provides method for loading from a plain text config file) or\n directly from the\n  parameters of a mrpt::obs::CObservationStereoImages object.\n\n Remember that the rectified images have a different set of intrinsic\n parameters than the\n  original images, which can be retrieved with \n\n  Works with grayscale or color images.\n\n  Refer to the program stereo-calib-gui for a tool that generates the\n required stereo camera parameters\n  from a set of stereo images of a checkerboard.\n\n  Example of usage with mrpt::obs::CObservationStereoImages:\n\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  Read also the tutorial page online:\n https://www.mrpt.org/Rectifying_stereo_images\n\n \n CUndistortMap, mrpt::obs::CObservationStereoImages,\n mrpt::img::TCamera, the application \n* href=\"http://www.mrpt.org/Application:camera-calib\" >camera-calib for\n calibrating a camera.\n\n \n This class provides a uniform wrap over different OpenCV versions. The\n \"alpha\" parameter is ignored if built against OpenCV 2.0.X\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::vision::CStereoRectifyMap(); } ) );
		cl.def( pybind11::init( [](mrpt::vision::CStereoRectifyMap const &o){ return new mrpt::vision::CStereoRectifyMap(o); } ) );
		cl.def("isSet", (bool (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::isSet, "		@{ \n\n Returns true if  has been already called, false\n otherwise.\n  Can be used within loops to determine the first usage of the object and\n when it needs to be initialized.\n\nC++: mrpt::vision::CStereoRectifyMap::isSet() const --> bool");
		cl.def("setFromCamParams", (void (mrpt::vision::CStereoRectifyMap::*)(const class mrpt::img::TStereoCamera &)) &mrpt::vision::CStereoRectifyMap::setFromCamParams, "Prepares the mapping from the intrinsic, distortion and relative pose\n parameters of a stereo camera.\n Must be called before invoking \n The  parameter can be changed with  before invoking\n this method; otherwise, the current rectification maps will be marked as\n invalid and should be prepared again.\n \n\n setAlpha()\n\nC++: mrpt::vision::CStereoRectifyMap::setFromCamParams(const class mrpt::img::TStereoCamera &) --> void", pybind11::arg("params"));
		cl.def("setFromCamParams", (void (mrpt::vision::CStereoRectifyMap::*)(const class mrpt::obs::CObservationStereoImages &)) &mrpt::vision::CStereoRectifyMap::setFromCamParams, "A wrapper to  which takes the parameters from an\n stereo observation object \n\nC++: mrpt::vision::CStereoRectifyMap::setFromCamParams(const class mrpt::obs::CObservationStereoImages &) --> void", pybind11::arg("stereo_obs"));
		cl.def("getCameraParams", (const class mrpt::img::TStereoCamera & (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::getCameraParams, "Returns the camera parameters which were used to generate the distortion\n map, as passed by the user to  \n\nC++: mrpt::vision::CStereoRectifyMap::getCameraParams() const --> const class mrpt::img::TStereoCamera &", pybind11::return_value_policy::automatic);
		cl.def("getRectifiedImageParams", (const class mrpt::img::TStereoCamera & (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::getRectifiedImageParams, "After computing the rectification maps, this method retrieves the\n calibration parameters of the rectified images\n  (which won't have any distortion).\n \n\n std::exception If the rectification maps have not been\n computed.\n\nC++: mrpt::vision::CStereoRectifyMap::getRectifiedImageParams() const --> const class mrpt::img::TStereoCamera &", pybind11::return_value_policy::automatic);
		cl.def("getRectifiedLeftImageParams", (const class mrpt::img::TCamera & (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::getRectifiedLeftImageParams, "Just like  but for the left camera only \n\nC++: mrpt::vision::CStereoRectifyMap::getRectifiedLeftImageParams() const --> const class mrpt::img::TCamera &", pybind11::return_value_policy::automatic);
		cl.def("getRectifiedRightImageParams", (const class mrpt::img::TCamera & (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::getRectifiedRightImageParams, "Just like  but for the right camera only \n\nC++: mrpt::vision::CStereoRectifyMap::getRectifiedRightImageParams() const --> const class mrpt::img::TCamera &", pybind11::return_value_policy::automatic);
		cl.def("setAlpha", (void (mrpt::vision::CStereoRectifyMap::*)(double)) &mrpt::vision::CStereoRectifyMap::setAlpha, "Sets the  parameter which controls the zoom in/out of the\n rectified images, such that:\n  - alpha=0 => rectified images are zoom in so that only valid pixels are\n visible\n  - alpha=1 => rectified images will contain large \"black areas\" but no\n pixel from the original image will be lost.\n Intermediary values leads to intermediary results.\n Its default value (-1) means auto guess by the OpenCV's algorithm.\n \n\n Call this method before building the rectification maps, otherwise\n they'll be marked as invalid.\n\nC++: mrpt::vision::CStereoRectifyMap::setAlpha(double) --> void", pybind11::arg("alpha"));
		cl.def("getAlpha", (double (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::getAlpha, "Return the  parameter \n setAlpha \n\nC++: mrpt::vision::CStereoRectifyMap::getAlpha() const --> double");
		cl.def("enableResizeOutput", [](mrpt::vision::CStereoRectifyMap &o, bool const & a0) -> void { return o.enableResizeOutput(a0); }, "", pybind11::arg("enable"));
		cl.def("enableResizeOutput", [](mrpt::vision::CStereoRectifyMap &o, bool const & a0, unsigned int const & a1) -> void { return o.enableResizeOutput(a0, a1); }, "", pybind11::arg("enable"), pybind11::arg("target_width"));
		cl.def("enableResizeOutput", (void (mrpt::vision::CStereoRectifyMap::*)(bool, unsigned int, unsigned int)) &mrpt::vision::CStereoRectifyMap::enableResizeOutput, "If enabled, the computed maps will rectify images to a size different\n than their original size.\n \n\n Call this method before building the rectification maps, otherwise\n they'll be marked as invalid.\n\nC++: mrpt::vision::CStereoRectifyMap::enableResizeOutput(bool, unsigned int, unsigned int) --> void", pybind11::arg("enable"), pybind11::arg("target_width"), pybind11::arg("target_height"));
		cl.def("isEnabledResizeOutput", (bool (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::isEnabledResizeOutput, "Returns whether resizing is enabled (default=false) \n\n enableResizeOutput \n\nC++: mrpt::vision::CStereoRectifyMap::isEnabledResizeOutput() const --> bool");
		cl.def("getResizeOutputSize", (struct mrpt::img::TPixelCoord (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::getResizeOutputSize, "Only when  returns true, this gets the target\n size  \n\n enableResizeOutput \n\nC++: mrpt::vision::CStereoRectifyMap::getResizeOutputSize() const --> struct mrpt::img::TPixelCoord");
		cl.def("setInterpolationMethod", (void (mrpt::vision::CStereoRectifyMap::*)(const enum mrpt::img::TInterpolationMethod)) &mrpt::vision::CStereoRectifyMap::setInterpolationMethod, "Change remap interpolation method (default=Lineal). This parameter can\n be safely changed at any instant without consequences. \n\nC++: mrpt::vision::CStereoRectifyMap::setInterpolationMethod(const enum mrpt::img::TInterpolationMethod) --> void", pybind11::arg("interp"));
		cl.def("getInterpolationMethod", (enum mrpt::img::TInterpolationMethod (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::getInterpolationMethod, "Get the currently selected interpolation method \n\n setInterpolationMethod \n\nC++: mrpt::vision::CStereoRectifyMap::getInterpolationMethod() const --> enum mrpt::img::TInterpolationMethod");
		cl.def("enableBothCentersCoincide", [](mrpt::vision::CStereoRectifyMap &o) -> void { return o.enableBothCentersCoincide(); }, "");
		cl.def("enableBothCentersCoincide", (void (mrpt::vision::CStereoRectifyMap::*)(bool)) &mrpt::vision::CStereoRectifyMap::enableBothCentersCoincide, "If enabled (default=false), the principal points in both output images\n will coincide.\n \n\n Call this method before building the rectification maps, otherwise\n they'll be marked as invalid.\n\nC++: mrpt::vision::CStereoRectifyMap::enableBothCentersCoincide(bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledBothCentersCoincide", (bool (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::isEnabledBothCentersCoincide, "enableBothCentersCoincide \n\nC++: mrpt::vision::CStereoRectifyMap::isEnabledBothCentersCoincide() const --> bool");
		cl.def("getLeftCameraRot", (const class mrpt::poses::CPose3DQuat & (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::getLeftCameraRot, "After computing the rectification maps, get the rotation applied to the\n left/right camera so their virtual image plane is the same after\n rectification \n\nC++: mrpt::vision::CStereoRectifyMap::getLeftCameraRot() const --> const class mrpt::poses::CPose3DQuat &", pybind11::return_value_policy::automatic);
		cl.def("getRightCameraRot", (const class mrpt::poses::CPose3DQuat & (mrpt::vision::CStereoRectifyMap::*)() const) &mrpt::vision::CStereoRectifyMap::getRightCameraRot, "See   \n\nC++: mrpt::vision::CStereoRectifyMap::getRightCameraRot() const --> const class mrpt::poses::CPose3DQuat &", pybind11::return_value_policy::automatic);
		cl.def("rectify", (void (mrpt::vision::CStereoRectifyMap::*)(const class mrpt::img::CImage &, const class mrpt::img::CImage &, class mrpt::img::CImage &, class mrpt::img::CImage &) const) &mrpt::vision::CStereoRectifyMap::rectify, "Rectify the input image pair and save the result in a different output\n images -  must have been set prior to calling this.\n The previous contents of the output images are completely ignored, but\n if they are already of the\n correct size and type, allocation time will be saved.\n Recall that  provides you the new intrinsic\n parameters of these images.\n \n\n std::exception If the rectification maps have not been\n computed.\n \n\n The same image CANNOT be at the same time input and output, in\n which case an exception will be raised (but see the overloaded version\n for in-place rectification)\n\nC++: mrpt::vision::CStereoRectifyMap::rectify(const class mrpt::img::CImage &, const class mrpt::img::CImage &, class mrpt::img::CImage &, class mrpt::img::CImage &) const --> void", pybind11::arg("in_left_image"), pybind11::arg("in_right_image"), pybind11::arg("out_left_image"), pybind11::arg("out_right_image"));
		cl.def("rectify", [](mrpt::vision::CStereoRectifyMap const &o, class mrpt::obs::CObservationStereoImages & a0) -> void { return o.rectify(a0); }, "", pybind11::arg("stereo_image_observation"));
		cl.def("rectify", (void (mrpt::vision::CStereoRectifyMap::*)(class mrpt::obs::CObservationStereoImages &, const bool) const) &mrpt::vision::CStereoRectifyMap::rectify, "Overloaded version for in-place rectification of image pairs stored in a\n mrpt::obs::CObservationStereoImages.\n  Upon return, the new camera intrinsic parameters will be already stored\n in the observation object.\n If  is set to  (recommended), will reuse\n over and over again the same\n auxiliary images (kept internally to this object) needed for in-place\n rectification.\n The only reason not to enable this cache is when multiple threads can\n invoke this method simultaneously.\n \n\n This method uses the left & right camera rotations computed by the\n rectification map to update\n         mrpt::obs::CObservationStereoImages::cameraPose (left camera wrt\n the robot frame) and\n         mrpt::obs::CObservationStereoImages::rightCameraPose (right wrt\n left camera).\n\nC++: mrpt::vision::CStereoRectifyMap::rectify(class mrpt::obs::CObservationStereoImages &, const bool) const --> void", pybind11::arg("stereo_image_observation"), pybind11::arg("use_internal_mem_cache"));
		cl.def("assign", (class mrpt::vision::CStereoRectifyMap & (mrpt::vision::CStereoRectifyMap::*)(const class mrpt::vision::CStereoRectifyMap &)) &mrpt::vision::CStereoRectifyMap::operator=, "C++: mrpt::vision::CStereoRectifyMap::operator=(const class mrpt::vision::CStereoRectifyMap &) --> class mrpt::vision::CStereoRectifyMap &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
