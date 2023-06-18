#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/hwdrivers/CDUO3DCamera.h>
#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
#include <string>
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

void bind_mrpt_hwdrivers_CDUO3DCamera(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::TCaptureOptions_DUO3D file:mrpt/hwdrivers/CDUO3DCamera.h line:20
		pybind11::class_<mrpt::hwdrivers::TCaptureOptions_DUO3D, std::shared_ptr<mrpt::hwdrivers::TCaptureOptions_DUO3D>> cl(M("mrpt::hwdrivers"), "TCaptureOptions_DUO3D", "Options used when creating a camera capture object of type\n CImageGrabber_FlyCapture2   \n");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::TCaptureOptions_DUO3D(); } ) );
		cl.def( pybind11::init( [](mrpt::hwdrivers::TCaptureOptions_DUO3D const &o){ return new mrpt::hwdrivers::TCaptureOptions_DUO3D(o); } ) );

		pybind11::enum_<mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult>(cl, "TYMLReadResult", pybind11::arithmetic(), "")
			.value("yrr_NAME_NON_CONSISTENT", mrpt::hwdrivers::TCaptureOptions_DUO3D::yrr_NAME_NON_CONSISTENT)
			.value("yrr_EMPTY", mrpt::hwdrivers::TCaptureOptions_DUO3D::yrr_EMPTY)
			.value("yrr_OK", mrpt::hwdrivers::TCaptureOptions_DUO3D::yrr_OK)
			.export_values();

		cl.def_readwrite("m_img_width", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_img_width);
		cl.def_readwrite("m_img_height", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_img_height);
		cl.def_readwrite("m_fps", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_fps);
		cl.def_readwrite("m_exposure", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_exposure);
		cl.def_readwrite("m_led", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_led);
		cl.def_readwrite("m_gain", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_gain);
		cl.def_readwrite("m_capture_imu", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_capture_imu);
		cl.def_readwrite("m_capture_rectified", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_capture_rectified);
		cl.def_readwrite("m_calibration_from_file", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_calibration_from_file);
		cl.def_readwrite("m_rectify_map_filename", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_rectify_map_filename);
		cl.def_readwrite("m_intrinsic_filename", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_intrinsic_filename);
		cl.def_readwrite("m_extrinsic_filename", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_extrinsic_filename);
		cl.def_readwrite("m_stereo_camera", &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_stereo_camera);
		cl.def("loadOptionsFrom", [](mrpt::hwdrivers::TCaptureOptions_DUO3D &o, const class mrpt::config::CConfigFileBase & a0, const std::string & a1) -> void { return o.loadOptionsFrom(a0, a1); }, "", pybind11::arg("configSource"), pybind11::arg("sectionName"));
		cl.def("loadOptionsFrom", (void (mrpt::hwdrivers::TCaptureOptions_DUO3D::*)(const class mrpt::config::CConfigFileBase &, const std::string &, const std::string &)) &mrpt::hwdrivers::TCaptureOptions_DUO3D::loadOptionsFrom, "Loads all the options from a config file.\n Expected format:\n\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n All parameter names may have an optional prefix, set with the\n\"prefix\" parameter.\n  For example, if prefix=\"LEFT_\", the expected variable name\n\"camera_index\" in the config section will be \"LEFT_camera_index\", and so\non.\n\nC++: mrpt::hwdrivers::TCaptureOptions_DUO3D::loadOptionsFrom(const class mrpt::config::CConfigFileBase &, const std::string &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("sectionName"), pybind11::arg("prefix"));
		cl.def("m_camera_int_params_from_yml", [](mrpt::hwdrivers::TCaptureOptions_DUO3D &o) -> mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult { return o.m_camera_int_params_from_yml(); }, "");
		cl.def("m_camera_int_params_from_yml", (enum mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult (mrpt::hwdrivers::TCaptureOptions_DUO3D::*)(const std::string &)) &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_camera_int_params_from_yml, "C++: mrpt::hwdrivers::TCaptureOptions_DUO3D::m_camera_int_params_from_yml(const std::string &) --> enum mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult", pybind11::arg("_file_name"));
		cl.def("m_camera_ext_params_from_yml", [](mrpt::hwdrivers::TCaptureOptions_DUO3D &o) -> mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult { return o.m_camera_ext_params_from_yml(); }, "");
		cl.def("m_camera_ext_params_from_yml", (enum mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult (mrpt::hwdrivers::TCaptureOptions_DUO3D::*)(const std::string &)) &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_camera_ext_params_from_yml, "C++: mrpt::hwdrivers::TCaptureOptions_DUO3D::m_camera_ext_params_from_yml(const std::string &) --> enum mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult", pybind11::arg("_file_name"));
		cl.def("m_rectify_map_from_yml", [](mrpt::hwdrivers::TCaptureOptions_DUO3D &o) -> mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult { return o.m_rectify_map_from_yml(); }, "");
		cl.def("m_rectify_map_from_yml", (enum mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult (mrpt::hwdrivers::TCaptureOptions_DUO3D::*)(const std::string &)) &mrpt::hwdrivers::TCaptureOptions_DUO3D::m_rectify_map_from_yml, "C++: mrpt::hwdrivers::TCaptureOptions_DUO3D::m_rectify_map_from_yml(const std::string &) --> enum mrpt::hwdrivers::TCaptureOptions_DUO3D::TYMLReadResult", pybind11::arg("_file_name"));
		cl.def("assign", (struct mrpt::hwdrivers::TCaptureOptions_DUO3D & (mrpt::hwdrivers::TCaptureOptions_DUO3D::*)(const struct mrpt::hwdrivers::TCaptureOptions_DUO3D &)) &mrpt::hwdrivers::TCaptureOptions_DUO3D::operator=, "C++: mrpt::hwdrivers::TCaptureOptions_DUO3D::operator=(const struct mrpt::hwdrivers::TCaptureOptions_DUO3D &) --> struct mrpt::hwdrivers::TCaptureOptions_DUO3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::hwdrivers::CFFMPEG_InputStream file:mrpt/hwdrivers/CFFMPEG_InputStream.h line:42
		pybind11::class_<mrpt::hwdrivers::CFFMPEG_InputStream, std::shared_ptr<mrpt::hwdrivers::CFFMPEG_InputStream>> cl(M("mrpt::hwdrivers"), "CFFMPEG_InputStream", "A generic class which process a video file or other kind of input stream\n (http, rtsp) and allows the extraction of images frame by frame.\n  Video sources can be open with \"openURL\", which can manage both video files\n and \"rtsp://\" sources (IP cameras).\n\n  Frames are retrieved by calling CFFMPEG_InputStream::retrieveFrame\n\n   For an example of usage, see the file \"samples/grab_camera_ffmpeg\"\n\n \n This class is an easy to use C++ wrapper for ffmpeg libraries\n (libavcodec). In Unix systems these libraries must be installed in the system\n as explained in \n* href=\"http://www.mrpt.org/Building_and_Installing_Instructions\" > MRPT's\n wiki. In Win32, a precompiled version for Visual Studio must be also\n downloaded as explained in \n* href=\"http://www.mrpt.org/Building_and_Installing_Instructions\" >the\n wiki.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CFFMPEG_InputStream(); } ) );
		cl.def( pybind11::init( [](mrpt::hwdrivers::CFFMPEG_InputStream const &o){ return new mrpt::hwdrivers::CFFMPEG_InputStream(o); } ) );
		cl.def("openURL", [](mrpt::hwdrivers::CFFMPEG_InputStream &o, const std::string & a0) -> bool { return o.openURL(a0); }, "", pybind11::arg("url"));
		cl.def("openURL", [](mrpt::hwdrivers::CFFMPEG_InputStream &o, const std::string & a0, bool const & a1) -> bool { return o.openURL(a0, a1); }, "", pybind11::arg("url"), pybind11::arg("grab_as_grayscale"));
		cl.def("openURL", [](mrpt::hwdrivers::CFFMPEG_InputStream &o, const std::string & a0, bool const & a1, bool const & a2) -> bool { return o.openURL(a0, a1, a2); }, "", pybind11::arg("url"), pybind11::arg("grab_as_grayscale"), pybind11::arg("verbose"));
		cl.def("openURL", (bool (mrpt::hwdrivers::CFFMPEG_InputStream::*)(const std::string &, bool, bool, const class std::map<std::string, std::string > &)) &mrpt::hwdrivers::CFFMPEG_InputStream::openURL, "Open a video file or a video stream (rtsp://)\n  This can be used to open local video files (eg. \"myVideo.avi\",\n \"c:\\a.mpeg\") and also IP cameras (e.g `rtsp://a.b.c.d/live.sdp`).\n  User/password can be used like `rtsp://USER:PASSWORD/PATH`.\n\n [ffmpeg options](https://www.ffmpeg.org/ffmpeg-protocols.html)\n can be added via the  argument.\n\n If  is set to true, more information about the video will be\n dumped to cout.\n\n \n close, retrieveFrame\n \n\n false on any error (and error info dumped to cerr), true on\n success.\n\nC++: mrpt::hwdrivers::CFFMPEG_InputStream::openURL(const std::string &, bool, bool, const class std::map<std::string, std::string > &) --> bool", pybind11::arg("url"), pybind11::arg("grab_as_grayscale"), pybind11::arg("verbose"), pybind11::arg("options"));
		cl.def("isOpen", (bool (mrpt::hwdrivers::CFFMPEG_InputStream::*)() const) &mrpt::hwdrivers::CFFMPEG_InputStream::isOpen, "Return whether the video source was open correctly \n\nC++: mrpt::hwdrivers::CFFMPEG_InputStream::isOpen() const --> bool");
		cl.def("close", (void (mrpt::hwdrivers::CFFMPEG_InputStream::*)()) &mrpt::hwdrivers::CFFMPEG_InputStream::close, "Close the video stream (this is called automatically at destruction).\n \n\n openURL\n\nC++: mrpt::hwdrivers::CFFMPEG_InputStream::close() --> void");
		cl.def("getVideoFPS", (double (mrpt::hwdrivers::CFFMPEG_InputStream::*)() const) &mrpt::hwdrivers::CFFMPEG_InputStream::getVideoFPS, "Get the frame-per-second (FPS) of the video source, or \"-1\" if the video\n is not open. \n\nC++: mrpt::hwdrivers::CFFMPEG_InputStream::getVideoFPS() const --> double");
		cl.def("retrieveFrame", (bool (mrpt::hwdrivers::CFFMPEG_InputStream::*)(class mrpt::img::CImage &)) &mrpt::hwdrivers::CFFMPEG_InputStream::retrieveFrame, "Get the next frame from the video stream.\n  Note that for remote streams (IP cameras) this method may block until\n enough information is read to generate a new frame.\n  Images are returned as 8-bit depth grayscale if \"grab_as_grayscale\" is\n true.\n  \n\n false on any error, true on success.\n  \n\n openURL, close, isOpen\n\nC++: mrpt::hwdrivers::CFFMPEG_InputStream::retrieveFrame(class mrpt::img::CImage &) --> bool", pybind11::arg("out_img"));
		cl.def("assign", (class mrpt::hwdrivers::CFFMPEG_InputStream & (mrpt::hwdrivers::CFFMPEG_InputStream::*)(const class mrpt::hwdrivers::CFFMPEG_InputStream &)) &mrpt::hwdrivers::CFFMPEG_InputStream::operator=, "C++: mrpt::hwdrivers::CFFMPEG_InputStream::operator=(const class mrpt::hwdrivers::CFFMPEG_InputStream &) --> class mrpt::hwdrivers::CFFMPEG_InputStream &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
