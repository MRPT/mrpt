#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CMyntEyeCamera.h>
#include <mrpt/hwdrivers/COpenNI2Generic.h>
#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/img/CCanvas.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/img/TColor.h>
#include <mrpt/img/TPixelCoord.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/io/CStream.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
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

// mrpt::hwdrivers::TMyntEyeCameraParameters file:mrpt/hwdrivers/CMyntEyeCamera.h line:23
struct PyCallBack_mrpt_hwdrivers_TMyntEyeCameraParameters : public mrpt::hwdrivers::TMyntEyeCameraParameters {
	using mrpt::hwdrivers::TMyntEyeCameraParameters::TMyntEyeCameraParameters;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::TMyntEyeCameraParameters *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TMyntEyeCameraParameters::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::TMyntEyeCameraParameters *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::hwdrivers::COpenNI2Sensor file:mrpt/hwdrivers/COpenNI2Sensor.h line:216
struct PyCallBack_mrpt_hwdrivers_COpenNI2Sensor : public mrpt::hwdrivers::COpenNI2Sensor {
	using mrpt::hwdrivers::COpenNI2Sensor::COpenNI2Sensor;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return COpenNI2Sensor::GetRuntimeClass();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COpenNI2Sensor::initialize();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COpenNI2Sensor::doProcess();
	}
	void setPathForExternalImages(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "setPathForExternalImages");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COpenNI2Sensor::setPathForExternalImages(a0);
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return COpenNI2Sensor::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::COpenNI2Sensor *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CMyntEyeCamera(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::TMyntEyeCameraParameters file:mrpt/hwdrivers/CMyntEyeCamera.h line:23
		pybind11::class_<mrpt::hwdrivers::TMyntEyeCameraParameters, std::shared_ptr<mrpt::hwdrivers::TMyntEyeCameraParameters>, PyCallBack_mrpt_hwdrivers_TMyntEyeCameraParameters, mrpt::config::CLoadableOptions> cl(M("mrpt::hwdrivers"), "TMyntEyeCameraParameters", "Open parameters for CMyntEyeCamera\n \n\n CMyntEyeCamera\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::TMyntEyeCameraParameters(); }, [](){ return new PyCallBack_mrpt_hwdrivers_TMyntEyeCameraParameters(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_hwdrivers_TMyntEyeCameraParameters const &o){ return new PyCallBack_mrpt_hwdrivers_TMyntEyeCameraParameters(o); } ) );
		cl.def( pybind11::init( [](mrpt::hwdrivers::TMyntEyeCameraParameters const &o){ return new mrpt::hwdrivers::TMyntEyeCameraParameters(o); } ) );
		cl.def_readwrite("ir_intensity", &mrpt::hwdrivers::TMyntEyeCameraParameters::ir_intensity);
		cl.def("loadFromConfigFile", (void (mrpt::hwdrivers::TMyntEyeCameraParameters::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::hwdrivers::TMyntEyeCameraParameters::loadFromConfigFile, "C++: mrpt::hwdrivers::TMyntEyeCameraParameters::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
		cl.def("assign", (struct mrpt::hwdrivers::TMyntEyeCameraParameters & (mrpt::hwdrivers::TMyntEyeCameraParameters::*)(const struct mrpt::hwdrivers::TMyntEyeCameraParameters &)) &mrpt::hwdrivers::TMyntEyeCameraParameters::operator=, "C++: mrpt::hwdrivers::TMyntEyeCameraParameters::operator=(const struct mrpt::hwdrivers::TMyntEyeCameraParameters &) --> struct mrpt::hwdrivers::TMyntEyeCameraParameters &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::hwdrivers::CMyntEyeCamera file:mrpt/hwdrivers/CMyntEyeCamera.h line:37
		pybind11::class_<mrpt::hwdrivers::CMyntEyeCamera, std::shared_ptr<mrpt::hwdrivers::CMyntEyeCamera>> cl(M("mrpt::hwdrivers"), "CMyntEyeCamera", "Wrapper on MYNT-EYE-D cameras. Requires MYNT-EYE SDK.\n\n \n mrpt::hwdrivers::CCameraSensor\n \n\n The most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor\n \n\n\n ");
		cl.def( pybind11::init<const struct mrpt::hwdrivers::TMyntEyeCameraParameters &>(), pybind11::arg("params") );

		cl.def( pybind11::init( [](mrpt::hwdrivers::CMyntEyeCamera const &o){ return new mrpt::hwdrivers::CMyntEyeCamera(o); } ) );
		cl.def("isOpen", (bool (mrpt::hwdrivers::CMyntEyeCamera::*)() const) &mrpt::hwdrivers::CMyntEyeCamera::isOpen, "Check whether the camera has been open successfully. \n\nC++: mrpt::hwdrivers::CMyntEyeCamera::isOpen() const --> bool");
		cl.def("getObservation", (bool (mrpt::hwdrivers::CMyntEyeCamera::*)(class mrpt::obs::CObservation3DRangeScan &)) &mrpt::hwdrivers::CMyntEyeCamera::getObservation, "Grab an image from the opened camera.\n \n\n The object to be filled with sensed data.\n\n \n false on any error, true if all go fine.\n\nC++: mrpt::hwdrivers::CMyntEyeCamera::getObservation(class mrpt::obs::CObservation3DRangeScan &) --> bool", pybind11::arg("out"));
		cl.def("assign", (class mrpt::hwdrivers::CMyntEyeCamera & (mrpt::hwdrivers::CMyntEyeCamera::*)(const class mrpt::hwdrivers::CMyntEyeCamera &)) &mrpt::hwdrivers::CMyntEyeCamera::operator=, "C++: mrpt::hwdrivers::CMyntEyeCamera::operator=(const class mrpt::hwdrivers::CMyntEyeCamera &) --> class mrpt::hwdrivers::CMyntEyeCamera &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::hwdrivers::COpenNI2Generic file:mrpt/hwdrivers/COpenNI2Generic.h line:23
		pybind11::class_<mrpt::hwdrivers::COpenNI2Generic, std::shared_ptr<mrpt::hwdrivers::COpenNI2Generic>> cl(M("mrpt::hwdrivers"), "COpenNI2Generic", "An abstract class for accessing OpenNI2 compatible sensors.\n This class permits one to access several sensors simultaneously. The same\noptions (resolution, fps, etc.) are used for every sensor.\n\n  More references to read:\n		- http://http://www.openni.org/\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::COpenNI2Generic(); } ) );
		cl.def( pybind11::init( [](int const & a0, int const & a1){ return new mrpt::hwdrivers::COpenNI2Generic(a0, a1); } ), "doc" , pybind11::arg("width"), pybind11::arg("height"));
		cl.def( pybind11::init( [](int const & a0, int const & a1, float const & a2){ return new mrpt::hwdrivers::COpenNI2Generic(a0, a1, a2); } ), "doc" , pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("fps"));
		cl.def( pybind11::init<int, int, float, bool>(), pybind11::arg("width"), pybind11::arg("height"), pybind11::arg("fps"), pybind11::arg("open_streams_now") );

		cl.def( pybind11::init( [](mrpt::hwdrivers::COpenNI2Generic const &o){ return new mrpt::hwdrivers::COpenNI2Generic(o); } ) );
		cl.def_static("getNumInstances", (int (*)()) &mrpt::hwdrivers::COpenNI2Generic::getNumInstances, "Get the number of OpenNI2 cameras currently open via COpenNI2Generic\n\nC++: mrpt::hwdrivers::COpenNI2Generic::getNumInstances() --> int");
		cl.def("getNextFrameRGB", [](mrpt::hwdrivers::COpenNI2Generic &o, class mrpt::img::CImage & a0, mrpt::Clock::time_point & a1, bool & a2, bool & a3) -> void { return o.getNextFrameRGB(a0, a1, a2, a3); }, "", pybind11::arg("rgb_img"), pybind11::arg("timestamp"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"));
		cl.def("getNextFrameRGB", (void (mrpt::hwdrivers::COpenNI2Generic::*)(class mrpt::img::CImage &, mrpt::Clock::time_point &, bool &, bool &, unsigned int)) &mrpt::hwdrivers::COpenNI2Generic::getNextFrameRGB, "The main data retrieving function, to be called after calling\n loadConfig() and initialize().\n  \n\n The output retrieved RGB image (only if\n there_is_obs=true).\n  \n\n The timestamp of the capture (only if\n there_is_obs=true).\n  \n\n If set to false, there was no new observation.\n  \n\n True on hardware/comms error.\n  \n\n The index of the sensor accessed.  \n\nC++: mrpt::hwdrivers::COpenNI2Generic::getNextFrameRGB(class mrpt::img::CImage &, mrpt::Clock::time_point &, bool &, bool &, unsigned int) --> void", pybind11::arg("rgb_img"), pybind11::arg("timestamp"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"), pybind11::arg("sensor_id"));
		cl.def("getNextFrameD", [](mrpt::hwdrivers::COpenNI2Generic &o, class mrpt::math::CMatrixDynamic<unsigned short> & a0, mrpt::Clock::time_point & a1, bool & a2, bool & a3) -> void { return o.getNextFrameD(a0, a1, a2, a3); }, "", pybind11::arg("depth_img_mm"), pybind11::arg("timestamp"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"));
		cl.def("getNextFrameD", (void (mrpt::hwdrivers::COpenNI2Generic::*)(class mrpt::math::CMatrixDynamic<unsigned short> &, mrpt::Clock::time_point &, bool &, bool &, unsigned int)) &mrpt::hwdrivers::COpenNI2Generic::getNextFrameD, "The main data retrieving function, to be called after calling\n loadConfig() and initialize().\n  \n\n The output retrieved depth image (only if\n there_is_obs=true).\n  \n\n The timestamp of the capture (only if\n there_is_obs=true).\n  \n\n If set to false, there was no new observation.\n  \n\n True on hardware/comms error.\n  \n\n The index of the sensor accessed. \n\nC++: mrpt::hwdrivers::COpenNI2Generic::getNextFrameD(class mrpt::math::CMatrixDynamic<unsigned short> &, mrpt::Clock::time_point &, bool &, bool &, unsigned int) --> void", pybind11::arg("depth_img_mm"), pybind11::arg("timestamp"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"), pybind11::arg("sensor_id"));
		cl.def("getNextFrameRGBD", [](mrpt::hwdrivers::COpenNI2Generic &o, class mrpt::obs::CObservation3DRangeScan & a0, bool & a1, bool & a2) -> void { return o.getNextFrameRGBD(a0, a1, a2); }, "", pybind11::arg("out_obs"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"));
		cl.def("getNextFrameRGBD", (void (mrpt::hwdrivers::COpenNI2Generic::*)(class mrpt::obs::CObservation3DRangeScan &, bool &, bool &, unsigned int)) &mrpt::hwdrivers::COpenNI2Generic::getNextFrameRGBD, "The main data retrieving function, to be called after calling\n loadConfig() and initialize().\n  \n\n The output retrieved observation (only if\n there_is_obs=true).\n  \n\n If set to false, there was no new observation.\n  \n\n True on hardware/comms error.\n  \n\n The index of the sensor accessed.\n\n \n doProcess\n\nC++: mrpt::hwdrivers::COpenNI2Generic::getNextFrameRGBD(class mrpt::obs::CObservation3DRangeScan &, bool &, bool &, unsigned int) --> void", pybind11::arg("out_obs"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"), pybind11::arg("sensor_id"));
		cl.def("open", [](mrpt::hwdrivers::COpenNI2Generic &o) -> void { return o.open(); }, "");
		cl.def("open", (void (mrpt::hwdrivers::COpenNI2Generic::*)(unsigned int)) &mrpt::hwdrivers::COpenNI2Generic::open, "@{ \n\n Try to open the camera (all the parameters [resolution,fps,...] must be\n set before calling this) - users may also call initialize(), which in\n turn calls this method.\n  Raises an exception upon error.\n \n\n std::exception A textual description of the error.\n\nC++: mrpt::hwdrivers::COpenNI2Generic::open(unsigned int) --> void", pybind11::arg("sensor_id"));
		cl.def("openDeviceBySerial", (unsigned int (mrpt::hwdrivers::COpenNI2Generic::*)(const unsigned int)) &mrpt::hwdrivers::COpenNI2Generic::openDeviceBySerial, "Open a RGBD device specified by its serial number. This method is a\n wrapper for\n  openDevicesBySerialNum(const std::set<unsigned>& vSerialRequired)\n  This method requires to open the sensors which are still closed to read\n their serial.\n\nC++: mrpt::hwdrivers::COpenNI2Generic::openDeviceBySerial(const unsigned int) --> unsigned int", pybind11::arg("SerialRequired"));
		cl.def("getDeviceIDFromSerialNum", (bool (mrpt::hwdrivers::COpenNI2Generic::*)(const unsigned int, int &) const) &mrpt::hwdrivers::COpenNI2Generic::getDeviceIDFromSerialNum, "Get the ID of the device corresponding to 'SerialRequired'.\n\nC++: mrpt::hwdrivers::COpenNI2Generic::getDeviceIDFromSerialNum(const unsigned int, int &) const --> bool", pybind11::arg("SerialRequired"), pybind11::arg("sensor_id"));
		cl.def("start", (bool (mrpt::hwdrivers::COpenNI2Generic::*)()) &mrpt::hwdrivers::COpenNI2Generic::start, "Open all sensor streams (normally called automatically at constructor,\n no need to call it manually). \n\n false on error \n kill() to close\n\nC++: mrpt::hwdrivers::COpenNI2Generic::start() --> bool");
		cl.def("kill", (void (mrpt::hwdrivers::COpenNI2Generic::*)()) &mrpt::hwdrivers::COpenNI2Generic::kill, "Kill the OpenNI2 driver \n start()\n\nC++: mrpt::hwdrivers::COpenNI2Generic::kill() --> void");
		cl.def("isOpen", (bool (mrpt::hwdrivers::COpenNI2Generic::*)(const unsigned int) const) &mrpt::hwdrivers::COpenNI2Generic::isOpen, "Whether there is a working connection to the sensor\n\nC++: mrpt::hwdrivers::COpenNI2Generic::isOpen(const unsigned int) const --> bool", pybind11::arg("sensor_id"));
		cl.def("close", [](mrpt::hwdrivers::COpenNI2Generic &o) -> void { return o.close(); }, "");
		cl.def("close", (void (mrpt::hwdrivers::COpenNI2Generic::*)(unsigned int)) &mrpt::hwdrivers::COpenNI2Generic::close, "Close the connection to the sensor (no need to call it manually unless\n desired for some reason, since it's called at destructor\n\nC++: mrpt::hwdrivers::COpenNI2Generic::close(unsigned int) --> void", pybind11::arg("sensor_id"));
		cl.def("getNumDevices", (int (mrpt::hwdrivers::COpenNI2Generic::*)() const) &mrpt::hwdrivers::COpenNI2Generic::getNumDevices, "The number of available devices at initialization\n\nC++: mrpt::hwdrivers::COpenNI2Generic::getNumDevices() const --> int");
		cl.def("getConnectedDevices", (int (mrpt::hwdrivers::COpenNI2Generic::*)()) &mrpt::hwdrivers::COpenNI2Generic::getConnectedDevices, "Get a list of the connected OpenNI2 sensors.\n\nC++: mrpt::hwdrivers::COpenNI2Generic::getConnectedDevices() --> int");
		cl.def("setVerbose", (void (mrpt::hwdrivers::COpenNI2Generic::*)(bool)) &mrpt::hwdrivers::COpenNI2Generic::setVerbose, "@} \n\nC++: mrpt::hwdrivers::COpenNI2Generic::setVerbose(bool) --> void", pybind11::arg("verbose"));
		cl.def("isVerbose", (bool (mrpt::hwdrivers::COpenNI2Generic::*)() const) &mrpt::hwdrivers::COpenNI2Generic::isVerbose, "C++: mrpt::hwdrivers::COpenNI2Generic::isVerbose() const --> bool");
		cl.def("getColorSensorParam", [](mrpt::hwdrivers::COpenNI2Generic const &o, class mrpt::img::TCamera & a0) -> bool { return o.getColorSensorParam(a0); }, "", pybind11::arg("param"));
		cl.def("getColorSensorParam", (bool (mrpt::hwdrivers::COpenNI2Generic::*)(class mrpt::img::TCamera &, unsigned int) const) &mrpt::hwdrivers::COpenNI2Generic::getColorSensorParam, "C++: mrpt::hwdrivers::COpenNI2Generic::getColorSensorParam(class mrpt::img::TCamera &, unsigned int) const --> bool", pybind11::arg("param"), pybind11::arg("sensor_id"));
		cl.def("getDepthSensorParam", [](mrpt::hwdrivers::COpenNI2Generic const &o, class mrpt::img::TCamera & a0) -> bool { return o.getDepthSensorParam(a0); }, "", pybind11::arg("param"));
		cl.def("getDepthSensorParam", (bool (mrpt::hwdrivers::COpenNI2Generic::*)(class mrpt::img::TCamera &, unsigned int) const) &mrpt::hwdrivers::COpenNI2Generic::getDepthSensorParam, "C++: mrpt::hwdrivers::COpenNI2Generic::getDepthSensorParam(class mrpt::img::TCamera &, unsigned int) const --> bool", pybind11::arg("param"), pybind11::arg("sensor_id"));
		cl.def("assign", (class mrpt::hwdrivers::COpenNI2Generic & (mrpt::hwdrivers::COpenNI2Generic::*)(const class mrpt::hwdrivers::COpenNI2Generic &)) &mrpt::hwdrivers::COpenNI2Generic::operator=, "C++: mrpt::hwdrivers::COpenNI2Generic::operator=(const class mrpt::hwdrivers::COpenNI2Generic &) --> class mrpt::hwdrivers::COpenNI2Generic &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::hwdrivers::COpenNI2Sensor file:mrpt/hwdrivers/COpenNI2Sensor.h line:216
		pybind11::class_<mrpt::hwdrivers::COpenNI2Sensor, std::shared_ptr<mrpt::hwdrivers::COpenNI2Sensor>, PyCallBack_mrpt_hwdrivers_COpenNI2Sensor, mrpt::hwdrivers::CGenericSensor, mrpt::hwdrivers::COpenNI2Generic> cl(M("mrpt::hwdrivers"), "COpenNI2Sensor", "A class for grabing \"range images\", intensity images (either RGB or IR) and\nother information from an OpenNI2 sensor.\n This class permits one to access several sensors simultaneously. The same\noptions (resolution, fps, etc.) are used for every sensor.\n\n  Configuration and usage: \n Data is returned as observations of type mrpt::obs::CObservation3DRangeScan.\n  See those classes for documentation on their fields.\n\n As with any other CGenericSensor class, the normal sequence of methods to be\ncalled is:\n   - CGenericSensor::loadConfig() - Or calls to the individual setXXX() to\nconfigure the sensor parameters.\n   - COpenNI2Sensor::initialize() - to start the communication with the\nsensor.\n   - call COpenNI2Sensor::getNextObservation() for getting the data.\n\n Calibration parameters\n  In this class we employ the OpenNI2 method to return depth images refered\nto the RGB camera. Otherwise we could specify\n   an accurate transformation of depth images to 3D points, you'll have to\ncalibrate your RGBD sensor for that, and supply\n   the following threee pieces of information (default calibration\ndata will be used otherwise, but they'll be not optimal for all sensors!):\n    - Camera parameters for the RGB camera. See\nCOpenNI2Sensor::setCameraParamsIntensity()\n    - Camera parameters for the depth camera. See\nCOpenNI2Sensor::setCameraParamsDepth()\n    - The 3D relative pose of the two cameras. See\nCOpenNI2Sensor::setRelativePoseIntensityWrtDepth()\n\n   See https://www.mrpt.org/Kinect_calibration for a procedure to calibrate\nRGBD sensors with an interactive GUI program.\n\n Coordinates convention\n   The origin of coordinates is the focal point of the depth camera, with the\naxes oriented as in the\n   diagram shown in mrpt::obs::CObservation3DRangeScan. Notice in that\npicture that the RGB camera is\n   assumed to have axes as usual in computer vision, which differ from those\nfor the depth camera.\n\n   The X,Y,Z axes used to report the data from accelerometers coincide with\nthose of the depth camera\n    (e.g. the camera standing on a table would have an ACC_Z=-9.8m/s2).\n\n   Notice however that, for consistency with stereo cameras, when loading the\ncalibration parameters from\n    a configuration file, the left-to-right pose increment is expected as if\nboth RGB & IR cameras had\n    their +Z axes pointing forward, +X to the right, +Y downwards (just like\nit's the standard in stereo cameras\n    and in computer vision literature). In other words: the pose stored in\nthis class uses a different\n    axes convention for the depth camera than in a stereo camera, so when a\npose L2R is loaded from a calibration file\n    it's actually converted like:\n\n      L2R(this class convention) = CPose3D(0,0,0,-90deg,0deg,-90deg) (+)\nL2R(in the config file)\n\n Some general comments\n		- Depth is grabbed in millimeters\n		- This sensor can be also used from within rawlog-grabber to grab\ndatasets\nwithin a robot with more sensors.\n		- There is no built-in threading support, so if you use this class\nmanually\n(not with-in rawlog-grabber),\n			the ideal would be to create a thread and continuously request data\nfrom\nthat thread (see mrpt::system::createThread ).\n		- The intensity channel default to the RGB images, but it can be changed\nwith setVideoChannel() to read the IR camera images (useful for calibrating).\n		- There is a built-in support for an optional preview of the data on a\nwindow, so you don't need to even worry on creating a window to show them.\n		- This class relies on an embedded version of libfreenect (you do NOT\nneed\nto install it in your system). Thanks guys for the great job!\n\n Converting to 3D point cloud \n   You can convert the 3D observation into a 3D point cloud with this piece\nof code:\n\n \n\n\n\n\n\n   Then the point cloud mrpt::maps::CColouredPointsMap can be converted into\nan OpenGL object for\n    rendering with mrpt::maps::CMetricMap::getAs3DObject() or alternatively\nwith:\n\n  \n\n\n\n\n Platform-specific comments\n   For more details, refer to \n*>libfreenect documentation:\n		- Linux: You'll need root privileges to access Kinect. Or, install\n\nMRPT/scripts/51-kinect.rules  in /etc/udev/rules.d/ to\nallow access to all users.\n		- Windows:\n			- Since MRPT 0.9.4 you'll only need to install \n*href=\"http://sourceforge.net/projects/libusb-win32/files/libusb-win32-releases/\"\n>libusb-win32: download and extract the latest\nlibusb-win32-bin-x.x.x.x.zip\n			- To install the drivers, read this:\nhttp://openkinect.org/wiki/Getting_Started#Windows\n		- MacOS: (write me!)\n\n Format of parameters for loading from a .ini file\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  More references to read:IMPEXP mrpt\n		- http://http://www.openni.org/\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::COpenNI2Sensor(); }, [](){ return new PyCallBack_mrpt_hwdrivers_COpenNI2Sensor(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::GetRuntimeClass, "C++: mrpt::hwdrivers::COpenNI2Sensor::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::COpenNI2Sensor::CreateObject, "C++: mrpt::hwdrivers::COpenNI2Sensor::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::COpenNI2Sensor::doRegister, "C++: mrpt::hwdrivers::COpenNI2Sensor::doRegister() --> void");
		cl.def("setSerialToOpen", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(const unsigned int)) &mrpt::hwdrivers::COpenNI2Sensor::setSerialToOpen, "Set the serial number of the device to open.\n  \n\n This method must throw an exception when such serial number\n is not found among the connected devices.\n\nC++: mrpt::hwdrivers::COpenNI2Sensor::setSerialToOpen(const unsigned int) --> void", pybind11::arg("serial"));
		cl.def("setSensorIDToOpen", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(const unsigned int)) &mrpt::hwdrivers::COpenNI2Sensor::setSensorIDToOpen, "Set the sensor_id of the device to open.\n  \n\n This method must throw an exception when such serial number\n is not found among the connected devices.\n\nC++: mrpt::hwdrivers::COpenNI2Sensor::setSensorIDToOpen(const unsigned int) --> void", pybind11::arg("sensor_id"));
		cl.def("initialize", (void (mrpt::hwdrivers::COpenNI2Sensor::*)()) &mrpt::hwdrivers::COpenNI2Sensor::initialize, "Initializes the 3D camera - should be invoked after calling loadConfig()\n or setting the different parameters with the set*() methods.\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::COpenNI2Sensor::initialize() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::COpenNI2Sensor::*)()) &mrpt::hwdrivers::COpenNI2Sensor::doProcess, "To be called  at a high rate (>XX Hz), this method populates the\n internal buffer of received observations.\n  This method is mainly intended for usage within rawlog-grabber or\n similar programs.\n  For an alternative, see getNextObservation()\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n \n\n getNextObservation\n\nC++: mrpt::hwdrivers::COpenNI2Sensor::doProcess() --> void");
		cl.def("getNextObservation", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(class mrpt::obs::CObservation3DRangeScan &, bool &, bool &)) &mrpt::hwdrivers::COpenNI2Sensor::getNextObservation, "The main data retrieving function, to be called after calling\n loadConfig() and initialize().\n  \n\n The output retrieved observation (only if\n there_is_obs=true).\n  \n\n If set to false, there was no new observation.\n  \n\n True on hardware/comms error.\n\n \n doProcess\n\nC++: mrpt::hwdrivers::COpenNI2Sensor::getNextObservation(class mrpt::obs::CObservation3DRangeScan &, bool &, bool &) --> void", pybind11::arg("out_obs"), pybind11::arg("there_is_obs"), pybind11::arg("hardware_error"));
		cl.def("setPathForExternalImages", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(const std::string &)) &mrpt::hwdrivers::COpenNI2Sensor::setPathForExternalImages, "Set the path where to save off-rawlog image files (this class DOES take\n into account this path).\n  An  empty string (the default value at construction) means to save\n images embedded in the rawlog, instead of on separate files.\n \n\n std::exception If the directory doesn't exists and cannot be\n created.\n\nC++: mrpt::hwdrivers::COpenNI2Sensor::setPathForExternalImages(const std::string &) --> void", pybind11::arg("directory"));
		cl.def("getMaxRange", (double (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::getMaxRange, "Get the maximum range (meters) that can be read in the observation field\n \"rangeImage\" \n\nC++: mrpt::hwdrivers::COpenNI2Sensor::getMaxRange() const --> double");
		cl.def("rows", (size_t (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::rows, "Get the row count in the camera images, loaded automatically upon camera\n open(). \n\nC++: mrpt::hwdrivers::COpenNI2Sensor::rows() const --> size_t");
		cl.def("cols", (size_t (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::cols, "Get the col count in the camera images, loaded automatically upon camera\n open(). \n\nC++: mrpt::hwdrivers::COpenNI2Sensor::cols() const --> size_t");
		cl.def("getCameraParamsIntensity", (const class mrpt::img::TCamera & (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::getCameraParamsIntensity, "Get a const reference to the depth camera calibration parameters \n\nC++: mrpt::hwdrivers::COpenNI2Sensor::getCameraParamsIntensity() const --> const class mrpt::img::TCamera &", pybind11::return_value_policy::automatic);
		cl.def("setCameraParamsIntensity", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(const class mrpt::img::TCamera &)) &mrpt::hwdrivers::COpenNI2Sensor::setCameraParamsIntensity, "C++: mrpt::hwdrivers::COpenNI2Sensor::setCameraParamsIntensity(const class mrpt::img::TCamera &) --> void", pybind11::arg("p"));
		cl.def("getCameraParamsDepth", (const class mrpt::img::TCamera & (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::getCameraParamsDepth, "Get a const reference to the depth camera calibration parameters \n\nC++: mrpt::hwdrivers::COpenNI2Sensor::getCameraParamsDepth() const --> const class mrpt::img::TCamera &", pybind11::return_value_policy::automatic);
		cl.def("setCameraParamsDepth", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(const class mrpt::img::TCamera &)) &mrpt::hwdrivers::COpenNI2Sensor::setCameraParamsDepth, "C++: mrpt::hwdrivers::COpenNI2Sensor::setCameraParamsDepth(const class mrpt::img::TCamera &) --> void", pybind11::arg("p"));
		cl.def("setRelativePoseIntensityWrtDepth", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(const class mrpt::poses::CPose3D &)) &mrpt::hwdrivers::COpenNI2Sensor::setRelativePoseIntensityWrtDepth, "Set the pose of the intensity camera wrt the depth camera \n See\n mrpt::obs::CObservation3DRangeScan for a 3D diagram of this pose \n\nC++: mrpt::hwdrivers::COpenNI2Sensor::setRelativePoseIntensityWrtDepth(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("p"));
		cl.def("getRelativePoseIntensityWrtDepth", (const class mrpt::poses::CPose3D & (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::getRelativePoseIntensityWrtDepth, "C++: mrpt::hwdrivers::COpenNI2Sensor::getRelativePoseIntensityWrtDepth() const --> const class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic);
		cl.def("enableGrabRGB", [](mrpt::hwdrivers::COpenNI2Sensor &o) -> void { return o.enableGrabRGB(); }, "");
		cl.def("enableGrabRGB", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(bool)) &mrpt::hwdrivers::COpenNI2Sensor::enableGrabRGB, "Enable/disable the grabbing of the RGB channel \n\nC++: mrpt::hwdrivers::COpenNI2Sensor::enableGrabRGB(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrabRGBEnabled", (bool (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::isGrabRGBEnabled, "C++: mrpt::hwdrivers::COpenNI2Sensor::isGrabRGBEnabled() const --> bool");
		cl.def("enableGrabDepth", [](mrpt::hwdrivers::COpenNI2Sensor &o) -> void { return o.enableGrabDepth(); }, "");
		cl.def("enableGrabDepth", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(bool)) &mrpt::hwdrivers::COpenNI2Sensor::enableGrabDepth, "Enable/disable the grabbing of the depth channel \n\nC++: mrpt::hwdrivers::COpenNI2Sensor::enableGrabDepth(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrabDepthEnabled", (bool (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::isGrabDepthEnabled, "C++: mrpt::hwdrivers::COpenNI2Sensor::isGrabDepthEnabled() const --> bool");
		cl.def("enableGrab3DPoints", [](mrpt::hwdrivers::COpenNI2Sensor &o) -> void { return o.enableGrab3DPoints(); }, "");
		cl.def("enableGrab3DPoints", (void (mrpt::hwdrivers::COpenNI2Sensor::*)(bool)) &mrpt::hwdrivers::COpenNI2Sensor::enableGrab3DPoints, "Enable/disable the grabbing of the 3D point clouds \n\nC++: mrpt::hwdrivers::COpenNI2Sensor::enableGrab3DPoints(bool) --> void", pybind11::arg("enable"));
		cl.def("isGrab3DPointsEnabled", (bool (mrpt::hwdrivers::COpenNI2Sensor::*)() const) &mrpt::hwdrivers::COpenNI2Sensor::isGrab3DPointsEnabled, "C++: mrpt::hwdrivers::COpenNI2Sensor::isGrab3DPointsEnabled() const --> bool");
	}
}
