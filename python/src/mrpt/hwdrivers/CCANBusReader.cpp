#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CCANBusReader.h>
#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationCANBusJ1939.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <string>
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

// mrpt::hwdrivers::CCANBusReader file:mrpt/hwdrivers/CCANBusReader.h line:57
struct PyCallBack_mrpt_hwdrivers_CCANBusReader : public mrpt::hwdrivers::CCANBusReader {
	using mrpt::hwdrivers::CCANBusReader::CCANBusReader;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CCANBusReader::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCANBusReader::loadConfig_sensorSpecific(a0, a1);
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCANBusReader::initialize();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCANBusReader::doProcess();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "loadConfig");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::loadConfig(a0, a1);
	}
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> >;
	_binder_ret_0 getObservations() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "getObservations");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CGenericSensor::getObservations();
	}
	void setPathForExternalImages(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "setPathForExternalImages");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setPathForExternalImages(a0);
	}
	void setExternalImageFormat(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "setExternalImageFormat");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setExternalImageFormat(a0);
	}
	void setExternalImageJPEGQuality(const unsigned int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "setExternalImageJPEGQuality");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::setExternalImageJPEGQuality(a0);
	}
	unsigned int getExternalImageJPEGQuality() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCANBusReader *>(this), "getExternalImageJPEGQuality");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned int>::value) {
				static pybind11::detail::override_caster_t<unsigned int> caster;
				return pybind11::detail::cast_ref<unsigned int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned int>(std::move(o));
		}
		return CGenericSensor::getExternalImageJPEGQuality();
	}
};

void bind_mrpt_hwdrivers_CCANBusReader(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CCANBusReader file:mrpt/hwdrivers/CCANBusReader.h line:57
		pybind11::class_<mrpt::hwdrivers::CCANBusReader, std::shared_ptr<mrpt::hwdrivers::CCANBusReader>, PyCallBack_mrpt_hwdrivers_CCANBusReader, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CCANBusReader", "This \"software driver\" implements the communication protocol for interfacing\n a SICK LMS 2XX laser scanners through a standard RS232 serial port (or a\n USB2SERIAL converter).\n   The serial port is opened upon the first call to \"doProcess\" or\n \"initialize\", so you must call \"loadConfig\" before\n   this, or manually call \"setSerialPort\". Another alternative is to call the\n base class method C2DRangeFinderAbstract::bindIO,\n   but the \"setSerialPort\" interface is probably much simpler to use.\n\n   For an example of usage see the example in\n \"samples/SICK_laser_serial_test\".\n   See also the example configuration file for rawlog-grabber in\n \"share/mrpt/config_files/rawlog-grabber\".\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n C2DRangeFinderAbstract\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CCANBusReader(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CCANBusReader(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CCANBusReader::*)() const) &mrpt::hwdrivers::CCANBusReader::GetRuntimeClass, "C++: mrpt::hwdrivers::CCANBusReader::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CCANBusReader::CreateObject, "C++: mrpt::hwdrivers::CCANBusReader::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CCANBusReader::doRegister, "C++: mrpt::hwdrivers::CCANBusReader::doRegister() --> void");
		cl.def("setSerialPort", (void (mrpt::hwdrivers::CCANBusReader::*)(const std::string &)) &mrpt::hwdrivers::CCANBusReader::setSerialPort, "Changes the serial port to connect to (call prior to 'doProcess'), for\n example \"COM1\" or \"ttyS0\".\n  This is not needed if the configuration is loaded with \"loadConfig\".\n\nC++: mrpt::hwdrivers::CCANBusReader::setSerialPort(const std::string &) --> void", pybind11::arg("port"));
		cl.def("getSerialPort", (std::string (mrpt::hwdrivers::CCANBusReader::*)() const) &mrpt::hwdrivers::CCANBusReader::getSerialPort, "setSerialPort \n\nC++: mrpt::hwdrivers::CCANBusReader::getSerialPort() const --> std::string");
		cl.def("setBaudRate", (void (mrpt::hwdrivers::CCANBusReader::*)(int)) &mrpt::hwdrivers::CCANBusReader::setBaudRate, "Changes the serial port baud rate (call prior to 'doProcess'); valid\n values are 9600,38400 and 500000.\n  This is not needed if the configuration is loaded with \"loadConfig\".\n  \n\n getBaudRate \n\nC++: mrpt::hwdrivers::CCANBusReader::setBaudRate(int) --> void", pybind11::arg("baud"));
		cl.def("getBaudRate", (int (mrpt::hwdrivers::CCANBusReader::*)() const) &mrpt::hwdrivers::CCANBusReader::getBaudRate, "setBaudRate \n\nC++: mrpt::hwdrivers::CCANBusReader::getBaudRate() const --> int");
		cl.def("setCANReaderTimeStamping", [](mrpt::hwdrivers::CCANBusReader &o) -> void { return o.setCANReaderTimeStamping(); }, "");
		cl.def("setCANReaderTimeStamping", (void (mrpt::hwdrivers::CCANBusReader::*)(bool)) &mrpt::hwdrivers::CCANBusReader::setCANReaderTimeStamping, "Enables/Disables the addition of a timestamp according to the arrival\n time to the converter (default=false)\n  (call prior to 'doProcess') This is not needed if the configuration is\n loaded with \"loadConfig\".\n\nC++: mrpt::hwdrivers::CCANBusReader::setCANReaderTimeStamping(bool) --> void", pybind11::arg("setTimestamp"));
		cl.def("getCANReaderTimeStamping", (bool (mrpt::hwdrivers::CCANBusReader::*)()) &mrpt::hwdrivers::CCANBusReader::getCANReaderTimeStamping, "C++: mrpt::hwdrivers::CCANBusReader::getCANReaderTimeStamping() --> bool");
		cl.def("setCANReaderSpeed", (void (mrpt::hwdrivers::CCANBusReader::*)(const unsigned int)) &mrpt::hwdrivers::CCANBusReader::setCANReaderSpeed, "Sets the CAN reader speed when connecting to the CAN Bus\n\nC++: mrpt::hwdrivers::CCANBusReader::setCANReaderSpeed(const unsigned int) --> void", pybind11::arg("speed"));
		cl.def("getCANReaderSpeed", (unsigned int (mrpt::hwdrivers::CCANBusReader::*)()) &mrpt::hwdrivers::CCANBusReader::getCANReaderSpeed, "C++: mrpt::hwdrivers::CCANBusReader::getCANReaderSpeed() --> unsigned int");
		cl.def("getCurrentConnectTry", (unsigned int (mrpt::hwdrivers::CCANBusReader::*)() const) &mrpt::hwdrivers::CCANBusReader::getCurrentConnectTry, "If performing several tries in ::initialize(), this is the current try\n loop number. \n\nC++: mrpt::hwdrivers::CCANBusReader::getCurrentConnectTry() const --> unsigned int");
		cl.def("doProcessSimple", (void (mrpt::hwdrivers::CCANBusReader::*)(bool &, class mrpt::obs::CObservationCANBusJ1939 &, bool &)) &mrpt::hwdrivers::CCANBusReader::doProcessSimple, "Specific laser scanner \"software drivers\" must process here new data\n from the I/O stream, and, if a whole scan has arrived, return it.\n  This method will be typically called in a different thread than other\n methods, and will be called in a timely fashion.\n\nC++: mrpt::hwdrivers::CCANBusReader::doProcessSimple(bool &, class mrpt::obs::CObservationCANBusJ1939 &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("initialize", (void (mrpt::hwdrivers::CCANBusReader::*)()) &mrpt::hwdrivers::CCANBusReader::initialize, "Set-up communication with the laser.\n  Called automatically by rawlog-grabber.\n  If used manually, call after \"loadConfig\" and before \"doProcess\".\n\n  In this class this method does nothing, since the communications are\n setup at the first try from \"doProcess\" or \"doProcessSimple\".\n\nC++: mrpt::hwdrivers::CCANBusReader::initialize() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CCANBusReader::*)()) &mrpt::hwdrivers::CCANBusReader::doProcess, "C++: mrpt::hwdrivers::CCANBusReader::doProcess() --> void");
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
		cl.def("retrieveFrame", (bool (mrpt::hwdrivers::CFFMPEG_InputStream::*)(class mrpt::img::CImage &, int64_t &)) &mrpt::hwdrivers::CFFMPEG_InputStream::retrieveFrame, "Refer to docs for ffmpeg AVFrame::pts\n\nC++: mrpt::hwdrivers::CFFMPEG_InputStream::retrieveFrame(class mrpt::img::CImage &, int64_t &) --> bool", pybind11::arg("out_img"), pybind11::arg("outPTS"));
		cl.def("assign", (class mrpt::hwdrivers::CFFMPEG_InputStream & (mrpt::hwdrivers::CFFMPEG_InputStream::*)(const class mrpt::hwdrivers::CFFMPEG_InputStream &)) &mrpt::hwdrivers::CFFMPEG_InputStream::operator=, "C++: mrpt::hwdrivers::CFFMPEG_InputStream::operator=(const class mrpt::hwdrivers::CFFMPEG_InputStream &) --> class mrpt::hwdrivers::CFFMPEG_InputStream &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
