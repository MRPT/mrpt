#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee_libdc1394.h>
#include <mrpt/hwdrivers/CStereoGrabber_SVS.h>
#include <mrpt/hwdrivers/CSwissRanger3DCamera.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
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

// mrpt::hwdrivers::CSwissRanger3DCamera file:mrpt/hwdrivers/CSwissRanger3DCamera.h line:108
struct PyCallBack_mrpt_hwdrivers_CSwissRanger3DCamera : public mrpt::hwdrivers::CSwissRanger3DCamera {
	using mrpt::hwdrivers::CSwissRanger3DCamera::CSwissRanger3DCamera;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CSwissRanger3DCamera::GetRuntimeClass();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSwissRanger3DCamera::initialize();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSwissRanger3DCamera::doProcess();
	}
	void setPathForExternalImages(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "setPathForExternalImages");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSwissRanger3DCamera::setPathForExternalImages(a0);
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSwissRanger3DCamera::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "getObservations");
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
	void setExternalImageFormat(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSwissRanger3DCamera *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CStereoGrabber_Bumblebee_libdc1394(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CStereoGrabber_Bumblebee_libdc1394 file:mrpt/hwdrivers/CStereoGrabber_Bumblebee_libdc1394.h line:34
		pybind11::class_<mrpt::hwdrivers::CStereoGrabber_Bumblebee_libdc1394, std::shared_ptr<mrpt::hwdrivers::CStereoGrabber_Bumblebee_libdc1394>> cl(M("mrpt::hwdrivers"), "CStereoGrabber_Bumblebee_libdc1394", "Grabs from a \"Bumblebee\" or \"Bumblebee2\" stereo camera using raw access to\n the libdc1394 library.\n Only raw, unrectified images can be captured with this class, which can be\n manually rectified given\n correct calibration parameters.\n\n See mrpt::hwdrivers::CStereoGrabber_Bumblebee for another class capable of\n live capture of rectified images using\n the vendor (PointGreyResearch) Triclops API.\n\n Once connected to a camera, you can call `getStereoObservation()` to\n retrieve the stereo images.\n\n \n You'll probably want to use instead the most generic camera grabber in\n MRPT: mrpt::hwdrivers::CCameraSensor\n \n\n\n ");
		cl.def( pybind11::init<uint64_t, uint16_t, double>(), pybind11::arg("cameraGUID"), pybind11::arg("cameraUnit"), pybind11::arg("frameRate") );

		cl.def("getStereoObservation", (bool (mrpt::hwdrivers::CStereoGrabber_Bumblebee_libdc1394::*)(class mrpt::obs::CObservationStereoImages &)) &mrpt::hwdrivers::CStereoGrabber_Bumblebee_libdc1394::getStereoObservation, "Grab stereo images, and return the pair of rectified images.\n \n\n The object to be filled with sensed data.\n\n \n The member \"CObservationStereoImages::refCameraPose\" must be set on\n the return of\n  this method by the user, since we don't know here the robot physical\n structure.\n\n \n false on any error, true if all go fine.\n\nC++: mrpt::hwdrivers::CStereoGrabber_Bumblebee_libdc1394::getStereoObservation(class mrpt::obs::CObservationStereoImages &) --> bool", pybind11::arg("out_observation"));
	}
	{ // mrpt::hwdrivers::TCaptureOptions_SVS file:mrpt/hwdrivers/CStereoGrabber_SVS.h line:18
		pybind11::class_<mrpt::hwdrivers::TCaptureOptions_SVS, std::shared_ptr<mrpt::hwdrivers::TCaptureOptions_SVS>> cl(M("mrpt::hwdrivers"), "TCaptureOptions_SVS", "Options used when creating a STOC Videre Design camera capture object\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::TCaptureOptions_SVS(); } ), "doc" );
		cl.def( pybind11::init( [](int const & a0){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0); } ), "doc" , pybind11::arg("_frame_width"));
		cl.def( pybind11::init( [](int const & a0, int const & a1){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, double const & a2){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1, a2); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, double const & a2, int const & a3){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1, a2, a3); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"), pybind11::arg("_NDisp"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, double const & a2, int const & a3, int const & a4){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1, a2, a3, a4); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"), pybind11::arg("_NDisp"), pybind11::arg("_Corrsize"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, double const & a2, int const & a3, int const & a4, int const & a5){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1, a2, a3, a4, a5); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"), pybind11::arg("_NDisp"), pybind11::arg("_Corrsize"), pybind11::arg("_LR"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, double const & a2, int const & a3, int const & a4, int const & a5, int const & a6){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1, a2, a3, a4, a5, a6); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"), pybind11::arg("_NDisp"), pybind11::arg("_Corrsize"), pybind11::arg("_LR"), pybind11::arg("_Thresh"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, double const & a2, int const & a3, int const & a4, int const & a5, int const & a6, int const & a7){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1, a2, a3, a4, a5, a6, a7); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"), pybind11::arg("_NDisp"), pybind11::arg("_Corrsize"), pybind11::arg("_LR"), pybind11::arg("_Thresh"), pybind11::arg("_Unique"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, double const & a2, int const & a3, int const & a4, int const & a5, int const & a6, int const & a7, int const & a8){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1, a2, a3, a4, a5, a6, a7, a8); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"), pybind11::arg("_NDisp"), pybind11::arg("_Corrsize"), pybind11::arg("_LR"), pybind11::arg("_Thresh"), pybind11::arg("_Unique"), pybind11::arg("_Horopter"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, double const & a2, int const & a3, int const & a4, int const & a5, int const & a6, int const & a7, int const & a8, int const & a9){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"), pybind11::arg("_NDisp"), pybind11::arg("_Corrsize"), pybind11::arg("_LR"), pybind11::arg("_Thresh"), pybind11::arg("_Unique"), pybind11::arg("_Horopter"), pybind11::arg("_SpeckleSize"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, double const & a2, int const & a3, int const & a4, int const & a5, int const & a6, int const & a7, int const & a8, int const & a9, bool const & a10){ return new mrpt::hwdrivers::TCaptureOptions_SVS(a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10); } ), "doc" , pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"), pybind11::arg("_NDisp"), pybind11::arg("_Corrsize"), pybind11::arg("_LR"), pybind11::arg("_Thresh"), pybind11::arg("_Unique"), pybind11::arg("_Horopter"), pybind11::arg("_SpeckleSize"), pybind11::arg("_procesOnChip"));
		cl.def( pybind11::init<int, int, double, int, int, int, int, int, int, int, bool, bool>(), pybind11::arg("_frame_width"), pybind11::arg("_frame_height"), pybind11::arg("_framerate"), pybind11::arg("_NDisp"), pybind11::arg("_Corrsize"), pybind11::arg("_LR"), pybind11::arg("_Thresh"), pybind11::arg("_Unique"), pybind11::arg("_Horopter"), pybind11::arg("_SpeckleSize"), pybind11::arg("_procesOnChip"), pybind11::arg("_calDisparity") );

		cl.def( pybind11::init( [](mrpt::hwdrivers::TCaptureOptions_SVS const &o){ return new mrpt::hwdrivers::TCaptureOptions_SVS(o); } ) );
		cl.def_readwrite("frame_width", &mrpt::hwdrivers::TCaptureOptions_SVS::frame_width);
		cl.def_readwrite("frame_height", &mrpt::hwdrivers::TCaptureOptions_SVS::frame_height);
		cl.def_readwrite("getRectified", &mrpt::hwdrivers::TCaptureOptions_SVS::getRectified);
		cl.def_readwrite("framerate", &mrpt::hwdrivers::TCaptureOptions_SVS::framerate);
		cl.def_readwrite("m_NDisp", &mrpt::hwdrivers::TCaptureOptions_SVS::m_NDisp);
		cl.def_readwrite("m_Corrsize", &mrpt::hwdrivers::TCaptureOptions_SVS::m_Corrsize);
		cl.def_readwrite("m_LR", &mrpt::hwdrivers::TCaptureOptions_SVS::m_LR);
		cl.def_readwrite("m_Thresh", &mrpt::hwdrivers::TCaptureOptions_SVS::m_Thresh);
		cl.def_readwrite("m_Unique", &mrpt::hwdrivers::TCaptureOptions_SVS::m_Unique);
		cl.def_readwrite("m_Horopter", &mrpt::hwdrivers::TCaptureOptions_SVS::m_Horopter);
		cl.def_readwrite("m_SpeckleSize", &mrpt::hwdrivers::TCaptureOptions_SVS::m_SpeckleSize);
		cl.def_readwrite("m_procesOnChip", &mrpt::hwdrivers::TCaptureOptions_SVS::m_procesOnChip);
		cl.def_readwrite("m_calDisparity", &mrpt::hwdrivers::TCaptureOptions_SVS::m_calDisparity);
		cl.def("assign", (struct mrpt::hwdrivers::TCaptureOptions_SVS & (mrpt::hwdrivers::TCaptureOptions_SVS::*)(const struct mrpt::hwdrivers::TCaptureOptions_SVS &)) &mrpt::hwdrivers::TCaptureOptions_SVS::operator=, "C++: mrpt::hwdrivers::TCaptureOptions_SVS::operator=(const struct mrpt::hwdrivers::TCaptureOptions_SVS &) --> struct mrpt::hwdrivers::TCaptureOptions_SVS &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::hwdrivers::CStereoGrabber_SVS file:mrpt/hwdrivers/CStereoGrabber_SVS.h line:73
		pybind11::class_<mrpt::hwdrivers::CStereoGrabber_SVS, std::shared_ptr<mrpt::hwdrivers::CStereoGrabber_SVS>> cl(M("mrpt::hwdrivers"), "CStereoGrabber_SVS", "A class for grabing stereo images from a STOC camera of Videre Design\n NOTE:\n		- Windows:\n			- This class is not available.\n\n		- Linux:\n			- This class is only available when compiling MRPT with\n\"MRPT_HAS_SVS\".\n			- You must have the videre design's library.\n			- Capture will be made in grayscale.\n 			- The grabber must be launch in root.\n\n Once connected to a camera, you can call \"getStereoObservation\" to retrieve\nthe Disparity images.\n\n \n You'll probably want to use instead the most generic camera grabber in\nMRPT: mrpt::hwdrivers::CCameraSensor\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CStereoGrabber_SVS(); } ), "doc" );
		cl.def( pybind11::init( [](int const & a0){ return new mrpt::hwdrivers::CStereoGrabber_SVS(a0); } ), "doc" , pybind11::arg("cameraIndex"));
		cl.def( pybind11::init<int, const struct mrpt::hwdrivers::TCaptureOptions_SVS &>(), pybind11::arg("cameraIndex"), pybind11::arg("options") );

		cl.def_readwrite("m_options", &mrpt::hwdrivers::CStereoGrabber_SVS::m_options);
		cl.def("getStereoObservation", (bool (mrpt::hwdrivers::CStereoGrabber_SVS::*)(class mrpt::obs::CObservationStereoImages &)) &mrpt::hwdrivers::CStereoGrabber_SVS::getStereoObservation, "Grab stereo images, and return the pair of rectified images.\n \n\n The object to be filled with sensed data.\n\n  NOTICE: (1) That the member \"CObservationStereoImages::refCameraPose\"\n must be\n                set on the return of this method, since we don't know here\n the robot physical structure.\n          (2) The images are already rectified.\n\n \n false on any error, true if all go fine.\n\nC++: mrpt::hwdrivers::CStereoGrabber_SVS::getStereoObservation(class mrpt::obs::CObservationStereoImages &) --> bool", pybind11::arg("out_observation"));
	}
	{ // mrpt::hwdrivers::CSwissRanger3DCamera file:mrpt/hwdrivers/CSwissRanger3DCamera.h line:108
		pybind11::class_<mrpt::hwdrivers::CSwissRanger3DCamera, std::shared_ptr<mrpt::hwdrivers::CSwissRanger3DCamera>, PyCallBack_mrpt_hwdrivers_CSwissRanger3DCamera, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CSwissRanger3DCamera", "A class for grabing \"range images\" from a MESA imaging SwissRanger 3D\ncameras (SR-2, SR-3000, SR-4k).\n\n NOTES:\n		- This class requires a vendor specific driver installed in the system\nin\norder to build MRPT with support for this sensor. Download and install the\ndriver from: http://www.mesa-imaging.ch/drivers.php\n		- The intensity channel (grayscale image) is converted from 16bit to\nstandard 8bit-per-pixel using a logarithmic, modified A-law compression. This\nallows exploiting the full dynamic range of the sensor and provides quite\ngood results.\n\n As with any other CGenericSensor class, the normal sequence of methods to be\ncalled is:\n   - loadConfig() - Or calls to the individual setXXX() to configure the\ncamera parameters.\n   - initialize() - to init the comms with the camera\n   - call getNextObservation() for getting the frames.\n\n  This sensor can be also used from within rawlog-grabber.\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CSwissRanger3DCamera(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CSwissRanger3DCamera(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::GetRuntimeClass, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CSwissRanger3DCamera::CreateObject, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CSwissRanger3DCamera::doRegister, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::doRegister() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)()) &mrpt::hwdrivers::CSwissRanger3DCamera::initialize, "Initializes the 3D camera - should be invoked after calling loadConfig()\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::initialize() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)()) &mrpt::hwdrivers::CSwissRanger3DCamera::doProcess, "To be called  at a high rate (>XX Hz), this method populates the\n internal buffer of received observations.\n  This method is mainly intended for usage within rawlog-grabber or\n similar programs.\n  For an alternative, see getNextObservation()\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n \n\n getNextObservation\n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::doProcess() --> void");
		cl.def("getNextObservation", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(class mrpt::obs::CObservation3DRangeScan &, bool &, bool &)) &mrpt::hwdrivers::CSwissRanger3DCamera::getNextObservation, "The main data retrieving function, to be called after calling\n loadConfig() and initialize().\n  \n\n The output retrieved observation (only if\n there_is_obs=true).\n  \n\n If set to false, there was no new observation.\n  \n\n True on hardware/comms error.\n\n \n doProcess\n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::getNextObservation(class mrpt::obs::CObservation3DRangeScan &, bool &, bool &) --> void", pybind11::arg("out_obs"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"));
		cl.def("open", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)()) &mrpt::hwdrivers::CSwissRanger3DCamera::open, "return false on error - Called automatically from initialize(), no need\n normally for the user to call this. \n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::open() --> bool");
		cl.def("close", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)()) &mrpt::hwdrivers::CSwissRanger3DCamera::close, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::close() --> void");
		cl.def("isOpen", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::isOpen, "whether the camera is open and comms work ok. To be called after\n initialize() \n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::isOpen() const --> bool");
		cl.def("rows", (size_t (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::rows, "Get the row count in the camera images, loaded automatically upon camera\n open(). \n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::rows() const --> size_t");
		cl.def("cols", (size_t (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::cols, "Get the col count in the camera images, loaded automatically upon camera\n open(). \n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::cols() const --> size_t");
		cl.def("getCameraSerialNumber", (unsigned int (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::getCameraSerialNumber, "Get the camera serial number, loaded automatically upon camera open().\n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::getCameraSerialNumber() const --> unsigned int");
		cl.def("getMaxRange", (double (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::getMaxRange, "Returns the maximum camera range, as deduced from its operating\n frequency. \n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::getMaxRange() const --> double");
		cl.def("setPathForExternalImages", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(const std::string &)) &mrpt::hwdrivers::CSwissRanger3DCamera::setPathForExternalImages, "Set the path where to save off-rawlog image files (this class DOES take\n into account this path).\n  An  empty string (the default value at construction) means to save\n images embedded in the rawlog, instead of on separate files.\n \n\n std::exception If the directory doesn't exists and cannot be\n created.\n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::setPathForExternalImages(const std::string &) --> void", pybind11::arg("directory"));
		cl.def("setOpenFromUSB", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::setOpenFromUSB, "true: open from USB, false: open from ethernet. \n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::setOpenFromUSB(bool) --> void", pybind11::arg("USB"));
		cl.def("getOpenFromUSBMode", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::getOpenFromUSBMode, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::getOpenFromUSBMode() const --> bool");
		cl.def("setOpenIPAddress", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(const std::string &)) &mrpt::hwdrivers::CSwissRanger3DCamera::setOpenIPAddress, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::setOpenIPAddress(const std::string &) --> void", pybind11::arg("IP"));
		cl.def("getOpenIPAddress", (std::string (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::getOpenIPAddress, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::getOpenIPAddress() const --> std::string");
		cl.def("setSave3D", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::setSave3D, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::setSave3D(bool) --> void", pybind11::arg("save"));
		cl.def("setSaveRangeImage", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::setSaveRangeImage, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::setSaveRangeImage(bool) --> void", pybind11::arg("save"));
		cl.def("setSaveIntensityImage", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::setSaveIntensityImage, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::setSaveIntensityImage(bool) --> void", pybind11::arg("save"));
		cl.def("setSaveConfidenceImage", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::setSaveConfidenceImage, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::setSaveConfidenceImage(bool) --> void", pybind11::arg("save"));
		cl.def("enableImageHistEqualization", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::enableImageHistEqualization, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::enableImageHistEqualization(bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledImageHistEqualization", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledImageHistEqualization, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledImageHistEqualization() const --> bool");
		cl.def("enableMedianFilter", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::enableMedianFilter, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::enableMedianFilter(bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledMedianFilter", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledMedianFilter, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledMedianFilter() const --> bool");
		cl.def("enableMedianCrossFilter", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::enableMedianCrossFilter, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::enableMedianCrossFilter(bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledMedianCrossFilter", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledMedianCrossFilter, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledMedianCrossFilter() const --> bool");
		cl.def("enableConvGray", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::enableConvGray, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::enableConvGray(bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledConvGray", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledConvGray, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledConvGray() const --> bool");
		cl.def("enableDenoiseANF", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::enableDenoiseANF, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::enableDenoiseANF(bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledDenoiseANF", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledDenoiseANF, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledDenoiseANF() const --> bool");
		cl.def("enablePreviewWindow", [](mrpt::hwdrivers::CSwissRanger3DCamera &o) -> void { return o.enablePreviewWindow(); }, "");
		cl.def("enablePreviewWindow", (void (mrpt::hwdrivers::CSwissRanger3DCamera::*)(bool)) &mrpt::hwdrivers::CSwissRanger3DCamera::enablePreviewWindow, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::enablePreviewWindow(bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledPreviewWindow", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)() const) &mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledPreviewWindow, "C++: mrpt::hwdrivers::CSwissRanger3DCamera::isEnabledPreviewWindow() const --> bool");
		cl.def("getMesaLibVersion", (bool (mrpt::hwdrivers::CSwissRanger3DCamera::*)(std::string &) const) &mrpt::hwdrivers::CSwissRanger3DCamera::getMesaLibVersion, "Get the version of the MESA library.\n \n\n false on error\n\nC++: mrpt::hwdrivers::CSwissRanger3DCamera::getMesaLibVersion(std::string &) const --> bool", pybind11::arg("out_version"));
	}
}
