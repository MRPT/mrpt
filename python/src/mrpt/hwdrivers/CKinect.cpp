#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/T3DPointsTo2DScanParams.h>
#include <mrpt/obs/TRangeImageFilter.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <mutex>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
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

// mrpt::hwdrivers::CKinect file:mrpt/hwdrivers/CKinect.h line:265
struct PyCallBack_mrpt_hwdrivers_CKinect : public mrpt::hwdrivers::CKinect {
	using mrpt::hwdrivers::CKinect::CKinect;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CKinect::GetRuntimeClass();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKinect::initialize();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKinect::doProcess();
	}
	void setPathForExternalImages(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "setPathForExternalImages");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKinect::setPathForExternalImages(a0);
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CKinect::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CKinect *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CKinect(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CKinect file:mrpt/hwdrivers/CKinect.h line:265
		pybind11::class_<mrpt::hwdrivers::CKinect, std::shared_ptr<mrpt::hwdrivers::CKinect>, PyCallBack_mrpt_hwdrivers_CKinect, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CKinect", "A class for grabing \"range images\", intensity images (either RGB or IR) and\nother information from an Xbox Kinect sensor.\n To use Kinect for Windows or ASUS/Primesense RGBD cameras, use the class\nCOpenNI2.\n\n  Configuration and usage: \n Data is returned as observations of type mrpt::obs::CObservation3DRangeScan\n(and mrpt::obs::CObservationIMU for accelerometers data).\n  See those classes for documentation on their fields.\n\n As with any other CGenericSensor class, the normal sequence of methods to be\ncalled is:\n   - CGenericSensor::loadConfig() - Or calls to the individual setXXX() to\nconfigure the sensor parameters.\n   - CKinect::initialize() - to start the communication with the sensor.\n   - call CKinect::getNextObservation() for getting the data.\n\n Calibration parameters\n  For an accurate transformation of depth images to 3D points, you'll have to\ncalibrate your Kinect, and supply\n   the following threee pieces of information (default calibration\ndata will be used otherwise, but they'll be not optimal for all sensors!):\n    - Camera parameters for the RGB camera. See\nCKinect::setCameraParamsIntensity()\n    - Camera parameters for the depth camera. See\nCKinect::setCameraParamsDepth()\n    - The 3D relative pose of the two cameras. See\nCKinect::setRelativePoseIntensityWrtDepth()\n\n   See https://www.mrpt.org/Kinect_calibration for a procedure to calibrate\nKinect sensors with an interactive GUI program.\n\n Coordinates convention\n   The origin of coordinates is the focal point of the depth camera, with the\naxes oriented as in the\n   diagram shown in mrpt::obs::CObservation3DRangeScan. Notice in that\npicture that the RGB camera is\n   assumed to have axes as usual in computer vision, which differ from those\nfor the depth camera.\n\n   The X,Y,Z axes used to report the data from accelerometers coincide with\nthose of the depth camera\n    (e.g. the camera standing on a table would have an ACC_Z=-9.8m/s2).\n\n   Notice however that, for consistency with stereo cameras, when loading the\ncalibration parameters from\n    a configuration file, the left-to-right pose increment is expected as if\nboth RGB & IR cameras had\n    their +Z axes pointing forward, +X to the right, +Y downwards (just like\nit's the standard in stereo cameras\n    and in computer vision literature). In other words: the pose stored in\nthis class uses a different\n    axes convention for the depth camera than in a stereo camera, so when a\npose L2R is loaded from a calibration file\n    it's actually converted like:\n\n      L2R(this class convention) = CPose3D(0,0,0,-90deg,0deg,-90deg) (+)\nL2R(in the config file)\n\n Some general comments\n		- Depth is grabbed in 10bit depth, and a range N it's converted to\nmeters\nas: range(m) = 0.1236 * tan(N/2842.5 + 1.1863)\n		- This sensor can be also used from within rawlog-grabber to grab\ndatasets\nwithin a robot with more sensors.\n		- There is no built-in threading support, so if you use this class\nmanually\n(not with-in rawlog-grabber),\n			the ideal would be to create a thread and continuously request data\nfrom\nthat thread (see mrpt::system::createThread ).\n		- The intensity channel default to the RGB images, but it can be changed\nwith setVideoChannel() to read the IR camera images (useful for calibrating).\n		- There is a built-in support for an optional preview of the data on a\nwindow, so you don't need to even worry on creating a window to show them.\n		- This class relies on an embedded version of libfreenect (you do NOT\nneed\nto install it in your system). Thanks guys for the great job!\n\n Converting to 3D point cloud \n   You can convert the 3D observation into a 3D point cloud with this piece\nof code:\n\n \n\n\n\n\n\n   Then the point cloud mrpt::maps::CColouredPointsMap can be converted into\nan OpenGL object for\n    rendering with mrpt::maps::CMetricMap::getAs3DObject() or alternatively\nwith:\n\n  \n\n\n\n\n Raw depth to range conversion\n  At construction, this class builds an internal array for converting raw 10\nor 11bit depths into ranges in meters.\n   Users can read that array or modify it (if you have a better calibration,\nfor example) by calling CKinect::getRawDepth2RangeConversion().\n   If you replace it, remember to set the first and last entries (index 0 and\nKINECT_RANGES_TABLE_LEN-1) to zero, to indicate that those are invalid\nranges.\n\n  \n  \n  \n    \n    R(d) = k3 * tan(d/k2 + k1); \n    k1 = 1.1863,  k2 = 2842.5, k3 = 0.1236 \n  \n  \n  \n  \n  \n\n Platform-specific comments\n   For more details, refer to \n*>libfreenect documentation:\n		- Linux: You'll need root privileges to access Kinect. Or, install\n\nMRPT/scripts/51-kinect.rules  in /etc/udev/rules.d/ to\nallow access to all users.\n		- Windows:\n			- Since MRPT 0.9.4 you'll only need to install \n*href=\"http://sourceforge.net/projects/libusb-win32/files/libusb-win32-releases/\"\n>libusb-win32: download and extract the latest\nlibusb-win32-bin-x.x.x.x.zip\n			- To install the drivers, read this:\nhttp://openkinect.org/wiki/Getting_Started#Windows\n		- MacOS: (write me!)\n\n Format of parameters for loading from a .ini file\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  More references to read:\n		- http://openkinect.org/wiki/Imaging_Information\n		- http://nicolas.burrus.name/index.php/Research/KinectCalibration\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CKinect(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CKinect(); } ) );

		pybind11::enum_<mrpt::hwdrivers::CKinect::TVideoChannel>(cl, "TVideoChannel", pybind11::arithmetic(), "RGB or IR video channel identifiers \n setVideoChannel ")
			.value("VIDEO_CHANNEL_RGB", mrpt::hwdrivers::CKinect::VIDEO_CHANNEL_RGB)
			.value("VIDEO_CHANNEL_IR", mrpt::hwdrivers::CKinect::VIDEO_CHANNEL_IR)
			.export_values();

		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::GetRuntimeClass, "C++: mrpt::hwdrivers::CKinect::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CKinect::CreateObject, "C++: mrpt::hwdrivers::CKinect::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CKinect::doRegister, "C++: mrpt::hwdrivers::CKinect::doRegister() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CKinect::*)()) &mrpt::hwdrivers::CKinect::initialize, "Initializes the 3D camera - should be invoked after calling loadConfig()\n or setting the different parameters with the set*() methods.\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CKinect::initialize() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CKinect::*)()) &mrpt::hwdrivers::CKinect::doProcess, "To be called  at a high rate (>XX Hz), this method populates the\n internal buffer of received observations.\n  This method is mainly intended for usage within rawlog-grabber or\n similar programs.\n  For an alternative, see getNextObservation()\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n \n\n getNextObservation\n\nC++: mrpt::hwdrivers::CKinect::doProcess() --> void");
		cl.def("getNextObservation", (void (mrpt::hwdrivers::CKinect::*)(class mrpt::obs::CObservation3DRangeScan &, bool &, bool &)) &mrpt::hwdrivers::CKinect::getNextObservation, "The main data retrieving function, to be called after calling\n loadConfig() and initialize().\n  \n\n The output retrieved observation (only if\n there_is_obs=true).\n  \n\n If set to false, there was no new observation.\n  \n\n True on hardware/comms error.\n\n \n doProcess\n\nC++: mrpt::hwdrivers::CKinect::getNextObservation(class mrpt::obs::CObservation3DRangeScan &, bool &, bool &) --> void", pybind11::arg("out_obs"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"));
		cl.def("getNextObservation", (void (mrpt::hwdrivers::CKinect::*)(class mrpt::obs::CObservation3DRangeScan &, class mrpt::obs::CObservationIMU &, bool &, bool &)) &mrpt::hwdrivers::CKinect::getNextObservation, "This method also grabs data from the accelerometers, returning\n them in out_obs_imu\n\nC++: mrpt::hwdrivers::CKinect::getNextObservation(class mrpt::obs::CObservation3DRangeScan &, class mrpt::obs::CObservationIMU &, bool &, bool &) --> void", pybind11::arg("out_obs"), pybind11::arg("out_obs_imu"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"));
		cl.def("setPathForExternalImages", (void (mrpt::hwdrivers::CKinect::*)(const std::string &)) &mrpt::hwdrivers::CKinect::setPathForExternalImages, "Set the path where to save off-rawlog image files (this class DOES take\n into account this path).\n  An  empty string (the default value at construction) means to save\n images embedded in the rawlog, instead of on separate files.\n \n\n std::exception If the directory doesn't exists and cannot be\n created.\n\nC++: mrpt::hwdrivers::CKinect::setPathForExternalImages(const std::string &) --> void", pybind11::arg("directory"));
		cl.def("open", (void (mrpt::hwdrivers::CKinect::*)()) &mrpt::hwdrivers::CKinect::open, "Try to open the camera (set all the parameters before calling this) -\n users may also call initialize(), which in turn calls this method.\n  Raises an exception upon error.\n \n\n std::exception A textual description of the error.\n\nC++: mrpt::hwdrivers::CKinect::open() --> void");
		cl.def("isOpen", (bool (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::isOpen, "Whether there is a working connection to the sensor \n\nC++: mrpt::hwdrivers::CKinect::isOpen() const --> bool");
		cl.def("close", (void (mrpt::hwdrivers::CKinect::*)()) &mrpt::hwdrivers::CKinect::close, "Close the Connection to the sensor (not need to call it manually unless\n desired for some reason,\n since it's called at destructor) \n\nC++: mrpt::hwdrivers::CKinect::close() --> void");
		cl.def("setVideoChannel", (void (mrpt::hwdrivers::CKinect::*)(const enum mrpt::hwdrivers::CKinect::TVideoChannel)) &mrpt::hwdrivers::CKinect::setVideoChannel, "Changes the video channel to open (RGB or IR) - you can call this method\n   before start grabbing or in the middle of streaming and the video source\n   will change on the fly.\n    Default is RGB channel. \n\nC++: mrpt::hwdrivers::CKinect::setVideoChannel(const enum mrpt::hwdrivers::CKinect::TVideoChannel) --> void", pybind11::arg("vch"));
		cl.def("getVideoChannel", (enum mrpt::hwdrivers::CKinect::TVideoChannel (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::getVideoChannel, "Return the current video channel (RGB or IR) \n setVideoChannel \n\nC++: mrpt::hwdrivers::CKinect::getVideoChannel() const --> enum mrpt::hwdrivers::CKinect::TVideoChannel");
		cl.def("setDeviceIndexToOpen", (void (mrpt::hwdrivers::CKinect::*)(int)) &mrpt::hwdrivers::CKinect::setDeviceIndexToOpen, "Set the sensor index to open (if there're several sensors attached to\n the computer); default=0 -> the first one. \n\nC++: mrpt::hwdrivers::CKinect::setDeviceIndexToOpen(int) --> void", pybind11::arg("index"));
		cl.def("getDeviceIndexToOpen", (int (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::getDeviceIndexToOpen, "C++: mrpt::hwdrivers::CKinect::getDeviceIndexToOpen() const --> int");
		cl.def("setTiltAngleDegrees", (void (mrpt::hwdrivers::CKinect::*)(double)) &mrpt::hwdrivers::CKinect::setTiltAngleDegrees, "Change tilt angle \n Sensor must be open first. \n\nC++: mrpt::hwdrivers::CKinect::setTiltAngleDegrees(double) --> void", pybind11::arg("angle"));
		cl.def("getTiltAngleDegrees", (double (mrpt::hwdrivers::CKinect::*)()) &mrpt::hwdrivers::CKinect::getTiltAngleDegrees, "C++: mrpt::hwdrivers::CKinect::getTiltAngleDegrees() --> double");
		cl.def("enablePreviewRGB", [](mrpt::hwdrivers::CKinect &o) -> void { return o.enablePreviewRGB(); }, "");
		cl.def("enablePreviewRGB", (void (mrpt::hwdrivers::CKinect::*)(bool)) &mrpt::hwdrivers::CKinect::enablePreviewRGB, "Default: disabled \n\nC++: mrpt::hwdrivers::CKinect::enablePreviewRGB(bool) --> void", pybind11::arg("enable"));
		cl.def("disablePreviewRGB", (void (mrpt::hwdrivers::CKinect::*)()) &mrpt::hwdrivers::CKinect::disablePreviewRGB, "C++: mrpt::hwdrivers::CKinect::disablePreviewRGB() --> void");
		cl.def("isPreviewRGBEnabled", (bool (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::isPreviewRGBEnabled, "C++: mrpt::hwdrivers::CKinect::isPreviewRGBEnabled() const --> bool");
		cl.def("setPreviewDecimation", (void (mrpt::hwdrivers::CKinect::*)(size_t)) &mrpt::hwdrivers::CKinect::setPreviewDecimation, "If preview is enabled, show only one image out of N (default: 1=show\n all) \n\nC++: mrpt::hwdrivers::CKinect::setPreviewDecimation(size_t) --> void", pybind11::arg("decimation_factor"));
		cl.def("getPreviewDecimation", (size_t (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::getPreviewDecimation, "C++: mrpt::hwdrivers::CKinect::getPreviewDecimation() const --> size_t");
		cl.def("getMaxRange", (double (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::getMaxRange, "Get the maximum range (meters) that can be read in the observation field\n \"rangeImage\" \n\nC++: mrpt::hwdrivers::CKinect::getMaxRange() const --> double");
		cl.def("rows", (size_t (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::rows, "Get the row count in the camera images, loaded automatically upon camera\n open(). \n\nC++: mrpt::hwdrivers::CKinect::rows() const --> size_t");
		cl.def("cols", (size_t (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::cols, "Get the col count in the camera images, loaded automatically upon camera\n open(). \n\nC++: mrpt::hwdrivers::CKinect::cols() const --> size_t");
		cl.def("getCameraParamsIntensity", (const class mrpt::img::TCamera & (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::getCameraParamsIntensity, "Get a const reference to the depth camera calibration parameters \n\nC++: mrpt::hwdrivers::CKinect::getCameraParamsIntensity() const --> const class mrpt::img::TCamera &", pybind11::return_value_policy::automatic);
		cl.def("setCameraParamsIntensity", (void (mrpt::hwdrivers::CKinect::*)(const class mrpt::img::TCamera &)) &mrpt::hwdrivers::CKinect::setCameraParamsIntensity, "C++: mrpt::hwdrivers::CKinect::setCameraParamsIntensity(const class mrpt::img::TCamera &) --> void", pybind11::arg("p"));
		cl.def("getCameraParamsDepth", (const class mrpt::img::TCamera & (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::getCameraParamsDepth, "Get a const reference to the depth camera calibration parameters \n\nC++: mrpt::hwdrivers::CKinect::getCameraParamsDepth() const --> const class mrpt::img::TCamera &", pybind11::return_value_policy::automatic);
		cl.def("setCameraParamsDepth", (void (mrpt::hwdrivers::CKinect::*)(const class mrpt::img::TCamera &)) &mrpt::hwdrivers::CKinect::setCameraParamsDepth, "C++: mrpt::hwdrivers::CKinect::setCameraParamsDepth(const class mrpt::img::TCamera &) --> void", pybind11::arg("p"));
		cl.def("setRelativePoseIntensityWrtDepth", (void (mrpt::hwdrivers::CKinect::*)(const class mrpt::poses::CPose3D &)) &mrpt::hwdrivers::CKinect::setRelativePoseIntensityWrtDepth, "Set the pose of the intensity camera wrt the depth camera \n See\n mrpt::obs::CObservation3DRangeScan for a 3D diagram of this pose \n\nC++: mrpt::hwdrivers::CKinect::setRelativePoseIntensityWrtDepth(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("p"));
		cl.def("getRelativePoseIntensityWrtDepth", (const class mrpt::poses::CPose3D & (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::getRelativePoseIntensityWrtDepth, "C++: mrpt::hwdrivers::CKinect::getRelativePoseIntensityWrtDepth() const --> const class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic);
		cl.def("enableGrabRGB", [](mrpt::hwdrivers::CKinect &o) -> void { return o.enableGrabRGB(); }, "");
		cl.def("enableGrabRGB", (void (mrpt::hwdrivers::CKinect::*)(bool)) &mrpt::hwdrivers::CKinect::enableGrabRGB, "Enable/disable the grabbing of the RGB channel \n\nC++: mrpt::hwdrivers::CKinect::enableGrabRGB(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrabRGBEnabled", (bool (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::isGrabRGBEnabled, "C++: mrpt::hwdrivers::CKinect::isGrabRGBEnabled() const --> bool");
		cl.def("enableGrabDepth", [](mrpt::hwdrivers::CKinect &o) -> void { return o.enableGrabDepth(); }, "");
		cl.def("enableGrabDepth", (void (mrpt::hwdrivers::CKinect::*)(bool)) &mrpt::hwdrivers::CKinect::enableGrabDepth, "Enable/disable the grabbing of the depth channel \n\nC++: mrpt::hwdrivers::CKinect::enableGrabDepth(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrabDepthEnabled", (bool (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::isGrabDepthEnabled, "C++: mrpt::hwdrivers::CKinect::isGrabDepthEnabled() const --> bool");
		cl.def("enableGrabAccelerometers", [](mrpt::hwdrivers::CKinect &o) -> void { return o.enableGrabAccelerometers(); }, "");
		cl.def("enableGrabAccelerometers", (void (mrpt::hwdrivers::CKinect::*)(bool)) &mrpt::hwdrivers::CKinect::enableGrabAccelerometers, "Enable/disable the grabbing of the inertial data \n\nC++: mrpt::hwdrivers::CKinect::enableGrabAccelerometers(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrabAccelerometersEnabled", (bool (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::isGrabAccelerometersEnabled, "C++: mrpt::hwdrivers::CKinect::isGrabAccelerometersEnabled() const --> bool");
		cl.def("enableGrab3DPoints", [](mrpt::hwdrivers::CKinect &o) -> void { return o.enableGrab3DPoints(); }, "");
		cl.def("enableGrab3DPoints", (void (mrpt::hwdrivers::CKinect::*)(bool)) &mrpt::hwdrivers::CKinect::enableGrab3DPoints, "Enable/disable the grabbing of the 3D point clouds \n\nC++: mrpt::hwdrivers::CKinect::enableGrab3DPoints(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrab3DPointsEnabled", (bool (mrpt::hwdrivers::CKinect::*)() const) &mrpt::hwdrivers::CKinect::isGrab3DPointsEnabled, "C++: mrpt::hwdrivers::CKinect::isGrab3DPointsEnabled() const --> bool");
	}
}
