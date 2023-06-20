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
#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/hwdrivers/CPtuBase.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationRGBD360.h>
#include <mrpt/obs/CObservationRange.h>
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

// mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors file:mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h line:101
struct PyCallBack_mrpt_hwdrivers_CPhidgetInterfaceKitProximitySensors : public mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors {
	using mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::CPhidgetInterfaceKitProximitySensors;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CPhidgetInterfaceKitProximitySensors::GetRuntimeClass();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPhidgetInterfaceKitProximitySensors::initialize();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPhidgetInterfaceKitProximitySensors::doProcess();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CGenericSensor::loadConfig_sensorSpecific\"");
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors *>(this), "getExternalImageJPEGQuality");
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
	// mrpt::hwdrivers::SensorType file:mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h line:93
	pybind11::enum_<mrpt::hwdrivers::SensorType>(M("mrpt::hwdrivers"), "SensorType", pybind11::arithmetic(), ": An interface for the phidget Interface kit board (1018).\n  \n\n\n  \n Adrien BARRAL - Robopec (aba.com).\n\n An interface for the Phidgets Interface kit board (part number 1018) on wich\n it could be plugged either an Sharp IR adaptater board\n (phidget's part number : 1101),or a MaxBotix EZ-1 sonar (phidget's part\n number : 1118).\n The configuration file describe what is plugged to this board, and the\n geometry of the sensors on the robots. See the exemple below.\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n The maximum number of sensors on this board is 8. Sensor 1 is the first\n sensor. If you haven't plugged any sensor on an entry of the board, you\n haven't to specify\n anyithing about this sensor in the configuration file.\n The following table enumerate the different sensors supported by this class.\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n{The Phidget library use udev. By default, udev require to be root to\n be launched, if you want to be able to run a program wich use a phidget board\n without be root, you must modify files in /etc/udev/rules.d .}\n \n\n\n ")
		.value("SHARP_30cm", mrpt::hwdrivers::SHARP_30cm)
		.value("SHARP_80cm", mrpt::hwdrivers::SHARP_80cm)
		.value("EZ1", mrpt::hwdrivers::EZ1)
		.value("UNPLUGGED", mrpt::hwdrivers::UNPLUGGED)
		.export_values();

;

	{ // mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors file:mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h line:101
		pybind11::class_<mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors, std::shared_ptr<mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors>, PyCallBack_mrpt_hwdrivers_CPhidgetInterfaceKitProximitySensors, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CPhidgetInterfaceKitProximitySensors", "");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CPhidgetInterfaceKitProximitySensors(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::*)() const) &mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::GetRuntimeClass, "C++: mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::CreateObject, "C++: mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::doRegister, "C++: mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::doRegister() --> void");
		cl.def("getObservation", (void (mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::*)(class mrpt::obs::CObservationRange &)) &mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::getObservation, "This method tries to get a set of range measurements from the IR\n sensors.\n \n\n Will be true if an observation was\n sucessfully received.\n\nC++: mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::getObservation(class mrpt::obs::CObservationRange &) --> void", pybind11::arg("outObservation"));
		cl.def("initialize", (void (mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::*)()) &mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::initialize, "Initialize the sensor according to the parameters previously read in the\n configuration file.\n \n\n throw an exception if the board could not be found.\n \n\n throw an exception if the process rate can't be set on one of\n the board channel.\n\nC++: mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::initialize() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::*)()) &mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::doProcess, "This method should be called periodically. Period depend on the\n process_rate in the configuration file.\n\nC++: mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors::doProcess() --> void");
	}
	{ // mrpt::hwdrivers::CPtuBase file:mrpt/hwdrivers/CPtuBase.h line:20
		pybind11::class_<mrpt::hwdrivers::CPtuBase, std::shared_ptr<mrpt::hwdrivers::CPtuBase>> cl(M("mrpt::hwdrivers"), "CPtuBase", "This class implements initialization and communication methods to\n control a generic Pan and Tilt Unit, working in radians.\n \n\n\n ");
		cl.def_readwrite("tiltResolution", &mrpt::hwdrivers::CPtuBase::tiltResolution);
		cl.def_readwrite("panResolution", &mrpt::hwdrivers::CPtuBase::panResolution);
		cl.def("rangeMeasure", (bool (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::rangeMeasure, "Search limit forward \n\nC++: mrpt::hwdrivers::CPtuBase::rangeMeasure() --> bool");
		cl.def("moveToAbsPos", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double)) &mrpt::hwdrivers::CPtuBase::moveToAbsPos, "Specification of positions in absolute terms \n\nC++: mrpt::hwdrivers::CPtuBase::moveToAbsPos(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("absPosQ", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &)) &mrpt::hwdrivers::CPtuBase::absPosQ, "Query position in absolute terms \n\nC++: mrpt::hwdrivers::CPtuBase::absPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("moveToOffPos", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double)) &mrpt::hwdrivers::CPtuBase::moveToOffPos, "Specify desired axis position as an offset from the current position. \n	This method recives the number of radians to move.\n	\n\n\n\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CPtuBase::moveToOffPos(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("offPosQ", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &)) &mrpt::hwdrivers::CPtuBase::offPosQ, "Query position in relative terms \n\nC++: mrpt::hwdrivers::CPtuBase::offPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("maxPosQ", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &)) &mrpt::hwdrivers::CPtuBase::maxPosQ, "Query max movement limit of a axis in absolute terms \n\nC++: mrpt::hwdrivers::CPtuBase::maxPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("minPosQ", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &)) &mrpt::hwdrivers::CPtuBase::minPosQ, "Query min movement limit of a axis in absolute terms \n\nC++: mrpt::hwdrivers::CPtuBase::minPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("enableLimitsQ", (bool (mrpt::hwdrivers::CPtuBase::*)(bool &)) &mrpt::hwdrivers::CPtuBase::enableLimitsQ, "Query if exist movement limits \n\nC++: mrpt::hwdrivers::CPtuBase::enableLimitsQ(bool &) --> bool", pybind11::arg("enable"));
		cl.def("enableLimits", (bool (mrpt::hwdrivers::CPtuBase::*)(bool)) &mrpt::hwdrivers::CPtuBase::enableLimits, "Enable/Disable movement limits \n\nC++: mrpt::hwdrivers::CPtuBase::enableLimits(bool) --> bool", pybind11::arg("set"));
		cl.def("inmediateExecution", (bool (mrpt::hwdrivers::CPtuBase::*)(bool)) &mrpt::hwdrivers::CPtuBase::inmediateExecution, "With I mode (default) instructs pan-tilt unit to immediately\n	execute positional commands. \n	In S mode instructs pan-tilt unit to execute positional commands\n	only when an Await Position Command Completion command is executed\n	or when put into Immediate Execution Mode. \n	\n\n\n\n\n\n\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CPtuBase::inmediateExecution(bool) --> bool", pybind11::arg("set"));
		cl.def("aWait", (bool (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::aWait, "Wait the finish of the last position command to\n	continue accept commands\n\nC++: mrpt::hwdrivers::CPtuBase::aWait() --> bool");
		cl.def("haltAll", (bool (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::haltAll, "Inmediately stop all \n\nC++: mrpt::hwdrivers::CPtuBase::haltAll() --> bool");
		cl.def("halt", (bool (mrpt::hwdrivers::CPtuBase::*)(char)) &mrpt::hwdrivers::CPtuBase::halt, "Inmediately stop \n\nC++: mrpt::hwdrivers::CPtuBase::halt(char) --> bool", pybind11::arg("axis"));
		cl.def("speed", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double)) &mrpt::hwdrivers::CPtuBase::speed, "Specification of turn speed \n\nC++: mrpt::hwdrivers::CPtuBase::speed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec"));
		cl.def("speedQ", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &)) &mrpt::hwdrivers::CPtuBase::speedQ, "Query turn speed \n\nC++: mrpt::hwdrivers::CPtuBase::speedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec"));
		cl.def("aceleration", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double)) &mrpt::hwdrivers::CPtuBase::aceleration, "Specification (de/a)celeration in turn \n\nC++: mrpt::hwdrivers::CPtuBase::aceleration(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec2"));
		cl.def("acelerationQ", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &)) &mrpt::hwdrivers::CPtuBase::acelerationQ, "Query (de/a)celeration in turn \n\nC++: mrpt::hwdrivers::CPtuBase::acelerationQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec2"));
		cl.def("baseSpeed", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double)) &mrpt::hwdrivers::CPtuBase::baseSpeed, "Specification of velocity to which start and finish\n	the (de/a)celeration\n\nC++: mrpt::hwdrivers::CPtuBase::baseSpeed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec"));
		cl.def("baseSpeedQ", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &)) &mrpt::hwdrivers::CPtuBase::baseSpeedQ, "Query velocity to which start and finish\n	the (de/a)celeration\n\nC++: mrpt::hwdrivers::CPtuBase::baseSpeedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec"));
		cl.def("upperSpeed", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double)) &mrpt::hwdrivers::CPtuBase::upperSpeed, "Specification of velocity upper limit \n\nC++: mrpt::hwdrivers::CPtuBase::upperSpeed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec"));
		cl.def("upperSpeedQ", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &)) &mrpt::hwdrivers::CPtuBase::upperSpeedQ, "Query velocity upper limit \n\nC++: mrpt::hwdrivers::CPtuBase::upperSpeedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec"));
		cl.def("lowerSpeed", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double)) &mrpt::hwdrivers::CPtuBase::lowerSpeed, "Specification of velocity lower limit \n\nC++: mrpt::hwdrivers::CPtuBase::lowerSpeed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec"));
		cl.def("lowerSpeedQ", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &)) &mrpt::hwdrivers::CPtuBase::lowerSpeedQ, "Query velocity lower limit \n\nC++: mrpt::hwdrivers::CPtuBase::lowerSpeedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("RadSec"));
		cl.def("reset", (bool (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::reset, "Reset PTU to initial state \n\nC++: mrpt::hwdrivers::CPtuBase::reset() --> bool");
		cl.def("save", (bool (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::save, "Save or restart default values \n\nC++: mrpt::hwdrivers::CPtuBase::save() --> bool");
		cl.def("restoreDefaults", (bool (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::restoreDefaults, "Restore default values \n\nC++: mrpt::hwdrivers::CPtuBase::restoreDefaults() --> bool");
		cl.def("restoreFactoryDefaults", (bool (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::restoreFactoryDefaults, "Restore factory default values \n\nC++: mrpt::hwdrivers::CPtuBase::restoreFactoryDefaults() --> bool");
		cl.def("version", (bool (mrpt::hwdrivers::CPtuBase::*)(char *)) &mrpt::hwdrivers::CPtuBase::version, "Version and CopyRights \n\nC++: mrpt::hwdrivers::CPtuBase::version(char *) --> bool", pybind11::arg("nVersion"));
		cl.def("nversion", (void (mrpt::hwdrivers::CPtuBase::*)(double &)) &mrpt::hwdrivers::CPtuBase::nversion, "Number of version \n\nC++: mrpt::hwdrivers::CPtuBase::nversion(double &) --> void", pybind11::arg("nVersion"));
		cl.def("powerModeQ", (bool (mrpt::hwdrivers::CPtuBase::*)(bool, char &)) &mrpt::hwdrivers::CPtuBase::powerModeQ, "Query power mode \n\nC++: mrpt::hwdrivers::CPtuBase::powerModeQ(bool, char &) --> bool", pybind11::arg("transit"), pybind11::arg("mode"));
		cl.def("powerMode", (bool (mrpt::hwdrivers::CPtuBase::*)(bool, char)) &mrpt::hwdrivers::CPtuBase::powerMode, "Specification of power mode \n\nC++: mrpt::hwdrivers::CPtuBase::powerMode(bool, char) --> bool", pybind11::arg("transit"), pybind11::arg("mode"));
		cl.def("status", (double (mrpt::hwdrivers::CPtuBase::*)(double &)) &mrpt::hwdrivers::CPtuBase::status, "Check if ptu is moving \n\nC++: mrpt::hwdrivers::CPtuBase::status(double &) --> double", pybind11::arg("rad"));
		cl.def("setLimits", (bool (mrpt::hwdrivers::CPtuBase::*)(char, double &, double &)) &mrpt::hwdrivers::CPtuBase::setLimits, "Set limits of movement \n\nC++: mrpt::hwdrivers::CPtuBase::setLimits(char, double &, double &) --> bool", pybind11::arg("axis"), pybind11::arg("l"), pybind11::arg("u"));
		cl.def("changeMotionDir", (bool (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::changeMotionDir, "C++: mrpt::hwdrivers::CPtuBase::changeMotionDir() --> bool");
		cl.def("checkErrors", (int (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::checkErrors, "Check errors, returns 0 if there are not errors or error code otherwise\n *\n\nC++: mrpt::hwdrivers::CPtuBase::checkErrors() --> int");
		cl.def("clearErrors", (void (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::clearErrors, "Clear errors *\n\nC++: mrpt::hwdrivers::CPtuBase::clearErrors() --> void");
		cl.def("init", (bool (mrpt::hwdrivers::CPtuBase::*)(const std::string &)) &mrpt::hwdrivers::CPtuBase::init, "PTU and serial port initialization \n\nC++: mrpt::hwdrivers::CPtuBase::init(const std::string &) --> bool", pybind11::arg("port"));
		cl.def("close", (void (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::close, "Close Connection with serial port \n\nC++: mrpt::hwdrivers::CPtuBase::close() --> void");
		cl.def("radError", (double (mrpt::hwdrivers::CPtuBase::*)(char, double)) &mrpt::hwdrivers::CPtuBase::radError, "To obtains the mistake for use discrete values when the movement\n	is expressed in radians. Parameters are the absolute position in\n	radians and the axis desired\n\nC++: mrpt::hwdrivers::CPtuBase::radError(char, double) --> double", pybind11::arg("axis"), pybind11::arg("nRadMoved"));
		cl.def("radToPos", (long (mrpt::hwdrivers::CPtuBase::*)(char, double)) &mrpt::hwdrivers::CPtuBase::radToPos, "To obtain the discrete value for a number of radians \n\nC++: mrpt::hwdrivers::CPtuBase::radToPos(char, double) --> long", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("posToRad", (double (mrpt::hwdrivers::CPtuBase::*)(char, long)) &mrpt::hwdrivers::CPtuBase::posToRad, "To obtain the number of radians for a discrete value \n\nC++: mrpt::hwdrivers::CPtuBase::posToRad(char, long) --> double", pybind11::arg("axis"), pybind11::arg("nPos"));
		cl.def("scan", (bool (mrpt::hwdrivers::CPtuBase::*)(char, int, float, float, double)) &mrpt::hwdrivers::CPtuBase::scan, "Performs a scan in the axis indicated and whit the precision desired.\n		\n\n {Pan or Till}\n		\n\n {Wait time betwen commands}\n		\n\n {initial position}\n		\n\n {final position}\n		\n\n {radians precision for the scan}\n\nC++: mrpt::hwdrivers::CPtuBase::scan(char, int, float, float, double) --> bool", pybind11::arg("axis"), pybind11::arg("wait"), pybind11::arg("initial"), pybind11::arg("final"), pybind11::arg("RadPre"));
		cl.def("verboseQ", (bool (mrpt::hwdrivers::CPtuBase::*)(bool &)) &mrpt::hwdrivers::CPtuBase::verboseQ, "Query verbose mode \n\nC++: mrpt::hwdrivers::CPtuBase::verboseQ(bool &) --> bool", pybind11::arg("modo"));
		cl.def("verbose", (bool (mrpt::hwdrivers::CPtuBase::*)(bool)) &mrpt::hwdrivers::CPtuBase::verbose, "Set verbose. \n	\n	Example of response with FV (verbose) active:\n		FV *\n		PP * Current pan position is 0\n		Example of response with FT (terse) active:\n		FT *\n		PP * 0\n	\n\nC++: mrpt::hwdrivers::CPtuBase::verbose(bool) --> bool", pybind11::arg("set"));
		cl.def("echoModeQ", (bool (mrpt::hwdrivers::CPtuBase::*)(bool &)) &mrpt::hwdrivers::CPtuBase::echoModeQ, "Query echo mode \n\nC++: mrpt::hwdrivers::CPtuBase::echoModeQ(bool &) --> bool", pybind11::arg("mode"));
		cl.def("echoMode", (bool (mrpt::hwdrivers::CPtuBase::*)(bool)) &mrpt::hwdrivers::CPtuBase::echoMode, "Enable/Disable echo response with command. \n	\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CPtuBase::echoMode(bool) --> bool", pybind11::arg("mode"));
		cl.def("resolution", (bool (mrpt::hwdrivers::CPtuBase::*)()) &mrpt::hwdrivers::CPtuBase::resolution, "Query the pan and tilt resolution per position moved\n	and initialize local atributes\n\nC++: mrpt::hwdrivers::CPtuBase::resolution() --> bool");
		cl.def("assign", (class mrpt::hwdrivers::CPtuBase & (mrpt::hwdrivers::CPtuBase::*)(const class mrpt::hwdrivers::CPtuBase &)) &mrpt::hwdrivers::CPtuBase::operator=, "C++: mrpt::hwdrivers::CPtuBase::operator=(const class mrpt::hwdrivers::CPtuBase &) --> class mrpt::hwdrivers::CPtuBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
