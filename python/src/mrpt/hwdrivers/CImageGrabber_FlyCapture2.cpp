#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/hwdrivers/CImageGrabber_FlyCapture2.h>
#include <mrpt/hwdrivers/CImageGrabber_OpenCV.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
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

void bind_mrpt_hwdrivers_CImageGrabber_FlyCapture2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::TCaptureOptions_FlyCapture2 file:mrpt/hwdrivers/CImageGrabber_FlyCapture2.h line:18
		pybind11::class_<mrpt::hwdrivers::TCaptureOptions_FlyCapture2, std::shared_ptr<mrpt::hwdrivers::TCaptureOptions_FlyCapture2>> cl(M("mrpt::hwdrivers"), "TCaptureOptions_FlyCapture2", "Options used when creating a camera capture object of type\n CImageGrabber_FlyCapture2   \n");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::TCaptureOptions_FlyCapture2(); } ) );
		cl.def( pybind11::init( [](mrpt::hwdrivers::TCaptureOptions_FlyCapture2 const &o){ return new mrpt::hwdrivers::TCaptureOptions_FlyCapture2(o); } ) );
		cl.def_readwrite("camera_index", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::camera_index);
		cl.def_readwrite("open_by_guid", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::open_by_guid);
		cl.def_readwrite("videomode", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::videomode);
		cl.def_readwrite("framerate", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::framerate);
		cl.def_readwrite("grabmode", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::grabmode);
		cl.def_readwrite("numBuffers", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::numBuffers);
		cl.def_readwrite("grabTimeout", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::grabTimeout);
		cl.def_readwrite("trigger_enabled", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::trigger_enabled);
		cl.def_readwrite("trigger_polarity", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::trigger_polarity);
		cl.def_readwrite("trigger_source", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::trigger_source);
		cl.def_readwrite("trigger_mode", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::trigger_mode);
		cl.def_readwrite("strobe_enabled", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::strobe_enabled);
		cl.def_readwrite("strobe_source", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::strobe_source);
		cl.def_readwrite("strobe_polarity", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::strobe_polarity);
		cl.def_readwrite("strobe_delay", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::strobe_delay);
		cl.def_readwrite("strobe_duration", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::strobe_duration);
		cl.def_readwrite("autoexposure_auto", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::autoexposure_auto);
		cl.def_readwrite("autoexposure_onOff", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::autoexposure_onOff);
		cl.def_readwrite("autoexposure_abs", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::autoexposure_abs);
		cl.def_readwrite("autoexposure_EV", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::autoexposure_EV);
		cl.def_readwrite("shutter_auto", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::shutter_auto);
		cl.def_readwrite("shutter_abs", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::shutter_abs);
		cl.def_readwrite("shutter_time_ms", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::shutter_time_ms);
		cl.def_readwrite("gain_auto", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::gain_auto);
		cl.def_readwrite("gain_abs", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::gain_abs);
		cl.def_readwrite("gain_dB", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::gain_dB);
		cl.def_readwrite("stereo_mode", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::stereo_mode);
		cl.def_readwrite("rect_width", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::rect_width);
		cl.def_readwrite("rect_height", &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::rect_height);
		cl.def("loadOptionsFrom", [](mrpt::hwdrivers::TCaptureOptions_FlyCapture2 &o, const class mrpt::config::CConfigFileBase & a0, const std::string & a1) -> void { return o.loadOptionsFrom(a0, a1); }, "", pybind11::arg("configSource"), pybind11::arg("sectionName"));
		cl.def("loadOptionsFrom", (void (mrpt::hwdrivers::TCaptureOptions_FlyCapture2::*)(const class mrpt::config::CConfigFileBase &, const std::string &, const std::string &)) &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::loadOptionsFrom, "Loads all the options from a config file.\n Expected format:\n\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n All parameter names may have an optional prefix, set with the\n \"prefix\" parameter.\n  For example, if prefix=\"LEFT_\", the expected variable name\n \"camera_index\" in the config section will be \"LEFT_camera_index\", and so\n on.\n\nC++: mrpt::hwdrivers::TCaptureOptions_FlyCapture2::loadOptionsFrom(const class mrpt::config::CConfigFileBase &, const std::string &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("sectionName"), pybind11::arg("prefix"));
		cl.def("assign", (struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 & (mrpt::hwdrivers::TCaptureOptions_FlyCapture2::*)(const struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 &)) &mrpt::hwdrivers::TCaptureOptions_FlyCapture2::operator=, "C++: mrpt::hwdrivers::TCaptureOptions_FlyCapture2::operator=(const struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 &) --> struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::hwdrivers::CImageGrabber_FlyCapture2 file:mrpt/hwdrivers/CImageGrabber_FlyCapture2.h line:181
		pybind11::class_<mrpt::hwdrivers::CImageGrabber_FlyCapture2, std::shared_ptr<mrpt::hwdrivers::CImageGrabber_FlyCapture2>> cl(M("mrpt::hwdrivers"), "CImageGrabber_FlyCapture2", "A wrapper for Point Gray Research (PGR) FlyCapture2 API for capturing images\n from Firewire, USB3 or GigaE cameras and stereo cameras.\n  This class is only available when compiling MRPT with\n \"MRPT_HAS_PGR_FLYCAPTURE2\".\n\n \n See the most generic camera grabber in MRPT:\n mrpt::hwdrivers::CCameraSensor\n \n\n See example code in [samples]/captureVideoFlyCapture2 and\n [samples]/captureVideoFlyCapture2_stereo.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CImageGrabber_FlyCapture2(); } ) );
		cl.def( pybind11::init<const struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 &>(), pybind11::arg("options") );

		cl.def("getCameraOptions", (const struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 & (mrpt::hwdrivers::CImageGrabber_FlyCapture2::*)() const) &mrpt::hwdrivers::CImageGrabber_FlyCapture2::getCameraOptions, "Returns the current settings of the camera \n\nC++: mrpt::hwdrivers::CImageGrabber_FlyCapture2::getCameraOptions() const --> const struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 &", pybind11::return_value_policy::automatic);
		cl.def("open", [](mrpt::hwdrivers::CImageGrabber_FlyCapture2 &o, const struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 & a0) -> void { return o.open(a0); }, "", pybind11::arg("options"));
		cl.def("open", (void (mrpt::hwdrivers::CImageGrabber_FlyCapture2::*)(const struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 &, const bool)) &mrpt::hwdrivers::CImageGrabber_FlyCapture2::open, "Tries to open the camera with the given options, and starts capture.\n Raises an exception on error.\n \n\n If set to false, the camera is only opened and\n configured, but a posterior call to startCapture() is required to start\n grabbing images.\n \n\n close(), startCapture()\n\nC++: mrpt::hwdrivers::CImageGrabber_FlyCapture2::open(const struct mrpt::hwdrivers::TCaptureOptions_FlyCapture2 &, const bool) --> void", pybind11::arg("options"), pybind11::arg("startCapture"));
		cl.def("startCapture", (void (mrpt::hwdrivers::CImageGrabber_FlyCapture2::*)()) &mrpt::hwdrivers::CImageGrabber_FlyCapture2::startCapture, "Start the actual image capture of the camera. Must be called after\n open(), only when \"startCapture\" was set to false.\n \n\n startSyncCapture\n\nC++: mrpt::hwdrivers::CImageGrabber_FlyCapture2::startCapture() --> void");
		cl.def("stopCapture", (void (mrpt::hwdrivers::CImageGrabber_FlyCapture2::*)()) &mrpt::hwdrivers::CImageGrabber_FlyCapture2::stopCapture, "Stop capture. \n\nC++: mrpt::hwdrivers::CImageGrabber_FlyCapture2::stopCapture() --> void");
		cl.def("close", (void (mrpt::hwdrivers::CImageGrabber_FlyCapture2::*)()) &mrpt::hwdrivers::CImageGrabber_FlyCapture2::close, "Stop capture and closes the opened camera, if any. Called automatically\n on object destruction. \n\nC++: mrpt::hwdrivers::CImageGrabber_FlyCapture2::close() --> void");
		cl.def_static("getFC2version", (std::string (*)()) &mrpt::hwdrivers::CImageGrabber_FlyCapture2::getFC2version, "Returns the PGR FlyCapture2 library version \n\nC++: mrpt::hwdrivers::CImageGrabber_FlyCapture2::getFC2version() --> std::string");
		cl.def("getObservation", (bool (mrpt::hwdrivers::CImageGrabber_FlyCapture2::*)(class mrpt::obs::CObservationImage &)) &mrpt::hwdrivers::CImageGrabber_FlyCapture2::getObservation, "Grab mono image from the camera. This method blocks until the next frame\n is captured.\n \n\n false on any error. \n\nC++: mrpt::hwdrivers::CImageGrabber_FlyCapture2::getObservation(class mrpt::obs::CObservationImage &) --> bool", pybind11::arg("out_observation"));
		cl.def("getObservation", (bool (mrpt::hwdrivers::CImageGrabber_FlyCapture2::*)(class mrpt::obs::CObservationStereoImages &)) &mrpt::hwdrivers::CImageGrabber_FlyCapture2::getObservation, "Grab stereo image from the camera. This method blocks until the next\n frame is captured.\n \n\n false on any error. \n\nC++: mrpt::hwdrivers::CImageGrabber_FlyCapture2::getObservation(class mrpt::obs::CObservationStereoImages &) --> bool", pybind11::arg("out_observation"));
		cl.def("isStereo", (bool (mrpt::hwdrivers::CImageGrabber_FlyCapture2::*)()) &mrpt::hwdrivers::CImageGrabber_FlyCapture2::isStereo, "Returns if current configuration is stereo or not \n\nC++: mrpt::hwdrivers::CImageGrabber_FlyCapture2::isStereo() --> bool");
	}
	// mrpt::hwdrivers::TCameraType file:mrpt/hwdrivers/CImageGrabber_OpenCV.h line:20
	pybind11::enum_<mrpt::hwdrivers::TCameraType>(M("mrpt::hwdrivers"), "TCameraType", pybind11::arithmetic(), "These capture types are like their OpenCV equivalents. ")
		.value("CAMERA_CV_AUTODETECT", mrpt::hwdrivers::CAMERA_CV_AUTODETECT)
		.value("CAMERA_CV_DC1394", mrpt::hwdrivers::CAMERA_CV_DC1394)
		.value("CAMERA_CV_VFL", mrpt::hwdrivers::CAMERA_CV_VFL)
		.value("CAMERA_CV_DSHOW", mrpt::hwdrivers::CAMERA_CV_DSHOW)
		.export_values();

;

	{ // mrpt::hwdrivers::TCaptureCVOptions file:mrpt/hwdrivers/CImageGrabber_OpenCV.h line:35
		pybind11::class_<mrpt::hwdrivers::TCaptureCVOptions, std::shared_ptr<mrpt::hwdrivers::TCaptureCVOptions>> cl(M("mrpt::hwdrivers"), "TCaptureCVOptions", "Options used when creating an OpenCV capture object\n  Some options apply to IEEE1394 cameras only.\n \n\n CImageGrabber_OpenCV\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::TCaptureCVOptions(); } ) );
		cl.def( pybind11::init( [](mrpt::hwdrivers::TCaptureCVOptions const &o){ return new mrpt::hwdrivers::TCaptureCVOptions(o); } ) );
		cl.def_readwrite("frame_width", &mrpt::hwdrivers::TCaptureCVOptions::frame_width);
		cl.def_readwrite("frame_height", &mrpt::hwdrivers::TCaptureCVOptions::frame_height);
		cl.def_readwrite("gain", &mrpt::hwdrivers::TCaptureCVOptions::gain);
		cl.def_readwrite("ieee1394_fps", &mrpt::hwdrivers::TCaptureCVOptions::ieee1394_fps);
		cl.def_readwrite("ieee1394_grayscale", &mrpt::hwdrivers::TCaptureCVOptions::ieee1394_grayscale);
	}
	{ // mrpt::hwdrivers::CImageGrabber_OpenCV file:mrpt/hwdrivers/CImageGrabber_OpenCV.h line:64
		pybind11::class_<mrpt::hwdrivers::CImageGrabber_OpenCV, std::shared_ptr<mrpt::hwdrivers::CImageGrabber_OpenCV>> cl(M("mrpt::hwdrivers"), "CImageGrabber_OpenCV", "A class for grabing images from a \"OpenCV\"-compatible camera, or from an AVI\n video file.\n   See the constructor for the options when opening the camera.\n\n  Unless input from AVI files is required, it is recommended to use the more\n generic class\n   mrpt::hwdrivers::CCameraSensor.\n\n \n This class is only available when compiling MRPT with the flag\n \"MRPT_HAS_OPENCV\" defined.\n \n\n Some code is based on the class CaptureCamera from the Orocos project.\n \n\n mrpt::hwdrivers::CCameraSensor, CImageGrabber_dc1394\n \n\n The most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CImageGrabber_OpenCV(); } ), "doc" );
		cl.def( pybind11::init( [](int const & a0){ return new mrpt::hwdrivers::CImageGrabber_OpenCV(a0); } ), "doc" , pybind11::arg("cameraIndex"));
		cl.def( pybind11::init( [](int const & a0, enum mrpt::hwdrivers::TCameraType const & a1){ return new mrpt::hwdrivers::CImageGrabber_OpenCV(a0, a1); } ), "doc" , pybind11::arg("cameraIndex"), pybind11::arg("cameraType"));
		cl.def( pybind11::init<int, enum mrpt::hwdrivers::TCameraType, const struct mrpt::hwdrivers::TCaptureCVOptions &>(), pybind11::arg("cameraIndex"), pybind11::arg("cameraType"), pybind11::arg("options") );

		cl.def( pybind11::init<const std::string &>(), pybind11::arg("AVI_fileName") );

		cl.def( pybind11::init( [](mrpt::hwdrivers::CImageGrabber_OpenCV const &o){ return new mrpt::hwdrivers::CImageGrabber_OpenCV(o); } ) );
		cl.def("isOpen", (bool (mrpt::hwdrivers::CImageGrabber_OpenCV::*)() const) &mrpt::hwdrivers::CImageGrabber_OpenCV::isOpen, "Check whether the camera has been open successfully. \n\nC++: mrpt::hwdrivers::CImageGrabber_OpenCV::isOpen() const --> bool");
		cl.def("getObservation", (bool (mrpt::hwdrivers::CImageGrabber_OpenCV::*)(class mrpt::obs::CObservationImage &)) &mrpt::hwdrivers::CImageGrabber_OpenCV::getObservation, "Grab an image from the opened camera.\n \n\n The object to be filled with sensed data.\n\n \n false on any error, true if all go fine.\n\nC++: mrpt::hwdrivers::CImageGrabber_OpenCV::getObservation(class mrpt::obs::CObservationImage &) --> bool", pybind11::arg("out_observation"));
		cl.def("assign", (class mrpt::hwdrivers::CImageGrabber_OpenCV & (mrpt::hwdrivers::CImageGrabber_OpenCV::*)(const class mrpt::hwdrivers::CImageGrabber_OpenCV &)) &mrpt::hwdrivers::CImageGrabber_OpenCV::operator=, "C++: mrpt::hwdrivers::CImageGrabber_OpenCV::operator=(const class mrpt::hwdrivers::CImageGrabber_OpenCV &) --> class mrpt::hwdrivers::CImageGrabber_OpenCV &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
