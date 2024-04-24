#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/COpenNI2_RGBD360.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationRGBD360.h>
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

// mrpt::hwdrivers::COpenNI2_RGBD360 file:mrpt/hwdrivers/COpenNI2_RGBD360.h line:207
struct PyCallBack_mrpt_hwdrivers_COpenNI2_RGBD360 : public mrpt::hwdrivers::COpenNI2_RGBD360 {
	using mrpt::hwdrivers::COpenNI2_RGBD360::COpenNI2_RGBD360;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return COpenNI2_RGBD360::GetRuntimeClass();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COpenNI2_RGBD360::initialize();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COpenNI2_RGBD360::doProcess();
	}
	void setPathForExternalImages(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "setPathForExternalImages");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COpenNI2_RGBD360::setPathForExternalImages(a0);
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COpenNI2_RGBD360::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2_RGBD360 *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_COpenNI2_RGBD360(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::COpenNI2_RGBD360 file:mrpt/hwdrivers/COpenNI2_RGBD360.h line:207
		pybind11::class_<mrpt::hwdrivers::COpenNI2_RGBD360, std::shared_ptr<mrpt::hwdrivers::COpenNI2_RGBD360>, PyCallBack_mrpt_hwdrivers_COpenNI2_RGBD360, mrpt::hwdrivers::CGenericSensor, mrpt::hwdrivers::COpenNI2Generic> cl(M("mrpt::hwdrivers"), "COpenNI2_RGBD360", "A class for grabing RGBD images from several OpenNI2 sensors. This is used\nto obtain larger fields of view using a radial configuration of the sensors.\n The same options (resolution, fps, etc.) are used for every sensor.\n\n  Configuration and usage: \n Data is returned as observations of type mrpt::obs::CObservationRGBD360.\n  See those classes for documentation on their fields.\n\n As with any other CGenericSensor class, the normal sequence of methods to be\ncalled is:\n   - CGenericSensor::loadConfig() - Or calls to the individual setXXX() to\nconfigure the sensor parameters.\n   - COpenNI2_RGBD360::initialize() - to start the communication with the\nsensor.\n   - call COpenNI2_RGBD360::getNextObservation() for getting the data.\n\n Calibration parameters\n   The reference system for both depth and RGB images provided by each\nindividual OpenNI2 sensors are referred to the\n   RGB Camera.\n   The extrinsic parameters of each RGBD sensor are provided from a\nconfiguration file. This calibration was obtained\n   using the method reported in [].\n\n Coordinates convention\n   The origin of coordinates is the focal point of the RGB camera of the\nfirst indexed sensor, with the axes oriented\n   as in the diagram shown in mrpt::obs::CObservation3DRangeScan. Notice in\nthat picture that the RGB camera is\n   assumed to have axes as usual in computer vision, which differ from those\nfor the depth camera.\n\n   The X,Y,Z axes used to report the data from accelerometers coincide with\nthose of the depth camera\n    (e.g. the camera standing on a table would have an ACC_Z=-9.8m/s2).\n\n   Notice however that, for consistency with stereo cameras, when loading the\ncalibration parameters from\n    a configuration file, the left-to-right pose increment is expected as if\nboth RGB & IR cameras had\n    their +Z axes pointing forward, +X to the right, +Y downwards (just like\nit's the standard in stereo cameras\n    and in computer vision literature). In other words: the pose stored in\nthis class uses a different\n    axes convention for the depth camera than in a stereo camera, so when a\npose L2R is loaded from a calibration file\n    it's actually converted like:\n\n      L2R(this class convention) = CPose3D(0,0,0,-90deg,0deg,-90deg) (+)\nL2R(in the config file)\n\n Some general comments\n		- Depth is grabbed in 10bit depth, and a range N it's converted to\nmeters\nas: range(m) = 0.1236 * tan(N/2842.5 + 1.1863)\n		- This sensor can be also used from within rawlog-grabber to grab\ndatasets\nwithin a robot with more sensors.\n		- There is no built-in threading support, so if you use this class\nmanually\n(not with-in rawlog-grabber),\n			the ideal would be to create a thread and continuously request data\nfrom\nthat thread (see mrpt::system::createThread ).\n		- The intensity channel default to the RGB images, but it can be changed\nwith setVideoChannel() to read the IR camera images (useful for calibrating).\n		- There is a built-in support for an optional preview of the data on a\nwindow, so you don't need to even worry on creating a window to show them.\n		- This class relies on an embedded version of libfreenect (you do NOT\nneed\nto install it in your system). Thanks guys for the great job!\n\n Converting to 3D point cloud \n   You can convert the 3D observation into a 3D point cloud with this piece\nof code:\n\n \n\n\n\n\n\n   Then the point cloud mrpt::maps::CColouredPointsMap can be converted into\nan OpenGL object for\n    rendering with mrpt::maps::CMetricMap::getAs3DObject() or alternatively\nwith:\n\n  \n\n\n\n\n Platform-specific comments\n   For more details, refer to \n*>libfreenect documentation:\n		- Linux: You'll need root privileges to access Kinect. Or, install\n\nMRPT/scripts/51-kinect.rules  in /etc/udev/rules.d/ to\nallow access to all users.\n		- Windows:\n			- Since MRPT 0.9.4 you'll only need to install \n*href=\"http://sourceforge.net/projects/libusb-win32/files/libusb-win32-releases/\"\n>libusb-win32: download and extract the latest\nlibusb-win32-bin-x.x.x.x.zip\n			- To install the drivers, read this:\nhttp://openkinect.org/wiki/Getting_Started#Windows\n		- MacOS: (write me!)\n\n Format of parameters for loading from a .ini file\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  More references to read:\n		- http://RGBD360\n		- http://http://www.openni.org/\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::COpenNI2_RGBD360(); }, [](){ return new PyCallBack_mrpt_hwdrivers_COpenNI2_RGBD360(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::COpenNI2_RGBD360::*)() const) &mrpt::hwdrivers::COpenNI2_RGBD360::GetRuntimeClass, "C++: mrpt::hwdrivers::COpenNI2_RGBD360::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::COpenNI2_RGBD360::CreateObject, "C++: mrpt::hwdrivers::COpenNI2_RGBD360::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::COpenNI2_RGBD360::doRegister, "C++: mrpt::hwdrivers::COpenNI2_RGBD360::doRegister() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::COpenNI2_RGBD360::*)()) &mrpt::hwdrivers::COpenNI2_RGBD360::initialize, "Initializes the 3D camera - should be invoked after calling loadConfig()\n or setting the different parameters with the set*() methods.\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::COpenNI2_RGBD360::initialize() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::COpenNI2_RGBD360::*)()) &mrpt::hwdrivers::COpenNI2_RGBD360::doProcess, "To be called  at a high rate (>XX Hz), this method populates the\n internal buffer of received observations.\n  This method is mainly intended for usage within rawlog-grabber or\n similar programs.\n  For an alternative, see getNextObservation()\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n \n\n getNextObservation\n\nC++: mrpt::hwdrivers::COpenNI2_RGBD360::doProcess() --> void");
		cl.def("getNextObservation", (void (mrpt::hwdrivers::COpenNI2_RGBD360::*)(class mrpt::obs::CObservationRGBD360 &, bool &, bool &)) &mrpt::hwdrivers::COpenNI2_RGBD360::getNextObservation, "The main data retrieving function, to be called after calling\n loadConfig() and initialize().\n  \n\n The output retrieved observation (only if\n there_is_obs=true).\n  \n\n If set to false, there was no new observation.\n  \n\n True on hardware/comms error.\n\n \n doProcess\n\nC++: mrpt::hwdrivers::COpenNI2_RGBD360::getNextObservation(class mrpt::obs::CObservationRGBD360 &, bool &, bool &) --> void", pybind11::arg("out_obs"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"));
		cl.def("setPathForExternalImages", (void (mrpt::hwdrivers::COpenNI2_RGBD360::*)(const std::string &)) &mrpt::hwdrivers::COpenNI2_RGBD360::setPathForExternalImages, "Set the path where to save off-rawlog image files (this class DOES take\n into account this path).\n  An  empty string (the default value at construction) means to save\n images embedded in the rawlog, instead of on separate files.\n \n\n std::exception If the directory doesn't exists and cannot be\n created.\n\nC++: mrpt::hwdrivers::COpenNI2_RGBD360::setPathForExternalImages(const std::string &) --> void", pybind11::arg("directory"));
		cl.def("getMaxRange", (double (mrpt::hwdrivers::COpenNI2_RGBD360::*)() const) &mrpt::hwdrivers::COpenNI2_RGBD360::getMaxRange, "Get the maximum range (meters) that can be read in the observation field\n \"rangeImage\" \n\nC++: mrpt::hwdrivers::COpenNI2_RGBD360::getMaxRange() const --> double");
		cl.def("enableGrabRGB", [](mrpt::hwdrivers::COpenNI2_RGBD360 &o) -> void { return o.enableGrabRGB(); }, "");
		cl.def("enableGrabRGB", (void (mrpt::hwdrivers::COpenNI2_RGBD360::*)(bool)) &mrpt::hwdrivers::COpenNI2_RGBD360::enableGrabRGB, "Enable/disable the grabbing of the RGB channel \n\nC++: mrpt::hwdrivers::COpenNI2_RGBD360::enableGrabRGB(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrabRGBEnabled", (bool (mrpt::hwdrivers::COpenNI2_RGBD360::*)() const) &mrpt::hwdrivers::COpenNI2_RGBD360::isGrabRGBEnabled, "C++: mrpt::hwdrivers::COpenNI2_RGBD360::isGrabRGBEnabled() const --> bool");
		cl.def("enableGrabDepth", [](mrpt::hwdrivers::COpenNI2_RGBD360 &o) -> void { return o.enableGrabDepth(); }, "");
		cl.def("enableGrabDepth", (void (mrpt::hwdrivers::COpenNI2_RGBD360::*)(bool)) &mrpt::hwdrivers::COpenNI2_RGBD360::enableGrabDepth, "Enable/disable the grabbing of the depth channel \n\nC++: mrpt::hwdrivers::COpenNI2_RGBD360::enableGrabDepth(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrabDepthEnabled", (bool (mrpt::hwdrivers::COpenNI2_RGBD360::*)() const) &mrpt::hwdrivers::COpenNI2_RGBD360::isGrabDepthEnabled, "C++: mrpt::hwdrivers::COpenNI2_RGBD360::isGrabDepthEnabled() const --> bool");
		cl.def("enableGrab3DPoints", [](mrpt::hwdrivers::COpenNI2_RGBD360 &o) -> void { return o.enableGrab3DPoints(); }, "");
		cl.def("enableGrab3DPoints", (void (mrpt::hwdrivers::COpenNI2_RGBD360::*)(bool)) &mrpt::hwdrivers::COpenNI2_RGBD360::enableGrab3DPoints, "Enable/disable the grabbing of the 3D point clouds \n\nC++: mrpt::hwdrivers::COpenNI2_RGBD360::enableGrab3DPoints(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrab3DPointsEnabled", (bool (mrpt::hwdrivers::COpenNI2_RGBD360::*)() const) &mrpt::hwdrivers::COpenNI2_RGBD360::isGrab3DPointsEnabled, "C++: mrpt::hwdrivers::COpenNI2_RGBD360::isGrab3DPointsEnabled() const --> bool");
	}
}
