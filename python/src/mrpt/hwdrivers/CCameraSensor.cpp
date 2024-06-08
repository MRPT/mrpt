#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
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

// mrpt::hwdrivers::CCameraSensor file:mrpt/hwdrivers/CCameraSensor.h line:283
struct PyCallBack_mrpt_hwdrivers_CCameraSensor : public mrpt::hwdrivers::CCameraSensor {
	using mrpt::hwdrivers::CCameraSensor::CCameraSensor;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CCameraSensor::GetRuntimeClass();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCameraSensor::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCameraSensor::initialize();
	}
	void setPathForExternalImages(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "setPathForExternalImages");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCameraSensor::setPathForExternalImages(a0);
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CCameraSensor::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CCameraSensor *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CCameraSensor(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CCameraSensor file:mrpt/hwdrivers/CCameraSensor.h line:283
		pybind11::class_<mrpt::hwdrivers::CCameraSensor, std::shared_ptr<mrpt::hwdrivers::CCameraSensor>, PyCallBack_mrpt_hwdrivers_CCameraSensor, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CCameraSensor", "The central class for camera grabbers in MRPT, implementing the \"generic\n sensor\" interface.\n   This class provides the user with a uniform interface to a variety of\n other classes which manage only one specific camera \"driver\" (opencv, ffmpeg,\n PGR FlyCapture,...)\n\n   Following the \"generic sensor\" interface, all the parameters must be\n passed int the form of a configuration file,\n   which may be also formed on the fly (without being a real config file) as\n in this example:\n\n  \n\n\n\n\n\n\n\n\n\n\n  Images can be retrieved through the normal \"doProcess()\" interface, or the\n specific method \"getNextFrame()\".\n\n Some notes:\n  - \"grabber_type\" determines the class to use internally for image capturing\n (see below).\n  - For the meaning of cv_camera_type and other parameters, refer to\n mrpt::hwdrivers::CImageGrabber_OpenCV\n  - For the parameters of dc1394 parameters, refer to generic IEEE1394\n documentation, and to mrpt::hwdrivers::TCaptureOptions_dc1394.\n  - If the high number of existing parameters annoy you, try the function\n prepareVideoSourceFromUserSelection(),\n     which displays a GUI dialog to the user so he/she can choose the desired\n camera & its parameters.\n\n  Images can be saved in the \"external storage\" mode. Detached threads are\n created for this task. See  and \n  These methods are called automatically from the app rawlog-grabber.\n\n  These is the list of all accepted parameters:\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n  \n The execution rate, in rawlog-grabber or the user code calling\n doProcess(), should be greater than the required capture FPS.\n  \n\n In Linux you may need to execute \"chmod 666 /dev/video1394/ * \" and\n \"chmod 666 /dev/raw1394\" for allowing any user R/W access to firewire\n cameras.\n \n\n [New in MRPT 1.4.0] The `bumblebee` driver has been deleted, use the\n `flycap` driver in stereo mode.\n  \n\n mrpt::hwdrivers::CImageGrabber_OpenCV,\n mrpt::hwdrivers::CImageGrabber_dc1394, CGenericSensor,\n prepareVideoSourceFromUserSelection()\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CCameraSensor(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CCameraSensor(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CCameraSensor::*)() const) &mrpt::hwdrivers::CCameraSensor::GetRuntimeClass, "C++: mrpt::hwdrivers::CCameraSensor::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CCameraSensor::CreateObject, "C++: mrpt::hwdrivers::CCameraSensor::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CCameraSensor::doRegister, "C++: mrpt::hwdrivers::CCameraSensor::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CCameraSensor::*)()) &mrpt::hwdrivers::CCameraSensor::doProcess, "C++: mrpt::hwdrivers::CCameraSensor::doProcess() --> void");
		cl.def("getNextFrame", (class std::shared_ptr<class mrpt::obs::CObservation> (mrpt::hwdrivers::CCameraSensor::*)()) &mrpt::hwdrivers::CCameraSensor::getNextFrame, "Retrieves the next frame from the video source, raising an exception on\nany error.\n Note: The returned observations can be of one of these classes (you can\nuse IS_CLASS(obs,CObservationXXX) to determine it):\n		- mrpt::obs::CObservationImage (For normal cameras or video sources)\n		- mrpt::obs::CObservationStereoImages (For stereo cameras)\n		- mrpt::obs::CObservation3DRangeScan (For 3D cameras)\n\nC++: mrpt::hwdrivers::CCameraSensor::getNextFrame() --> class std::shared_ptr<class mrpt::obs::CObservation>");
		cl.def("initialize", (void (mrpt::hwdrivers::CCameraSensor::*)()) &mrpt::hwdrivers::CCameraSensor::initialize, "Tries to open the camera, after setting all the parameters with a call\n to loadConfig.\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CCameraSensor::initialize() --> void");
		cl.def("close", (void (mrpt::hwdrivers::CCameraSensor::*)()) &mrpt::hwdrivers::CCameraSensor::close, "Close the camera (if open).\n   This method is called automatically on destruction.\n\nC++: mrpt::hwdrivers::CCameraSensor::close() --> void");
		cl.def("setSoftwareTriggerLevel", (void (mrpt::hwdrivers::CCameraSensor::*)(bool)) &mrpt::hwdrivers::CCameraSensor::setSoftwareTriggerLevel, "Set Software trigger level value (ON or OFF) for cameras with this\n function available.\n\nC++: mrpt::hwdrivers::CCameraSensor::setSoftwareTriggerLevel(bool) --> void", pybind11::arg("level"));
		cl.def("setPathForExternalImages", (void (mrpt::hwdrivers::CCameraSensor::*)(const std::string &)) &mrpt::hwdrivers::CCameraSensor::setPathForExternalImages, "Set the path where to save off-rawlog image files (this class DOES take\n into account this path).\n  An  empty string (the default value at construction) means to save\n images embedded in the rawlog, instead of on separate files.\n \n\n std::exception If the directory doesn't exists and cannot be\n created.\n\nC++: mrpt::hwdrivers::CCameraSensor::setPathForExternalImages(const std::string &) --> void", pybind11::arg("directory"));
		cl.def("enableLaunchOwnThreadForSavingImages", [](mrpt::hwdrivers::CCameraSensor &o) -> void { return o.enableLaunchOwnThreadForSavingImages(); }, "");
		cl.def("enableLaunchOwnThreadForSavingImages", (void (mrpt::hwdrivers::CCameraSensor::*)(bool)) &mrpt::hwdrivers::CCameraSensor::enableLaunchOwnThreadForSavingImages, "This must be called before initialize() \n\nC++: mrpt::hwdrivers::CCameraSensor::enableLaunchOwnThreadForSavingImages(bool) --> void", pybind11::arg("enable"));
		cl.def("addPreSaveHook", (void (mrpt::hwdrivers::CCameraSensor::*)(class std::function<void (const class std::shared_ptr<class mrpt::obs::CObservation> &, void *)>, void *)) &mrpt::hwdrivers::CCameraSensor::addPreSaveHook, "Provides a \"hook\" for user-code to be run BEFORE an image is going to be\n saved to disk if external storage is enabled (e.g. to rectify images,\n preprocess them, etc.)\n Notice that this code may be called from detached threads, so it must be\n thread safe.\n If used, call this before initialize() \n\nC++: mrpt::hwdrivers::CCameraSensor::addPreSaveHook(class std::function<void (const class std::shared_ptr<class mrpt::obs::CObservation> &, void *)>, void *) --> void", pybind11::arg("user_function"), pybind11::arg("user_ptr"));
	}
	// mrpt::hwdrivers::prepareVideoSourceFromPanel(void *) file:mrpt/hwdrivers/CCameraSensor.h line:512
	M("mrpt::hwdrivers").def("prepareVideoSourceFromPanel", (class std::shared_ptr<class mrpt::hwdrivers::CCameraSensor> (*)(void *)) &mrpt::hwdrivers::prepareVideoSourceFromPanel, "Used only from MRPT apps: Use with caution since \"panel\" MUST be a\n \"mrpt::gui::CPanelCameraSelection *\"\n\nC++: mrpt::hwdrivers::prepareVideoSourceFromPanel(void *) --> class std::shared_ptr<class mrpt::hwdrivers::CCameraSensor>", pybind11::arg("panel"));

	// mrpt::hwdrivers::writeConfigFromVideoSourcePanel(void *, const std::string &, class mrpt::config::CConfigFileBase *) file:mrpt/hwdrivers/CCameraSensor.h line:521
	M("mrpt::hwdrivers").def("writeConfigFromVideoSourcePanel", (void (*)(void *, const std::string &, class mrpt::config::CConfigFileBase *)) &mrpt::hwdrivers::writeConfigFromVideoSourcePanel, "Parse the user options in the wxWidgets \"panel\" and write the configuration\n into the given section of the given configuration file.\n Use with caution since \"panel\" MUST be a \"mrpt::gui::CPanelCameraSelection\n *\"\n \n\n prepareVideoSourceFromUserSelection, prepareVideoSourceFromPanel,\n readConfigIntoVideoSourcePanel\n\nC++: mrpt::hwdrivers::writeConfigFromVideoSourcePanel(void *, const std::string &, class mrpt::config::CConfigFileBase *) --> void", pybind11::arg("panel"), pybind11::arg("in_cfgfile_section_name"), pybind11::arg("out_cfgfile"));

	// mrpt::hwdrivers::readConfigIntoVideoSourcePanel(void *, const std::string &, const class mrpt::config::CConfigFileBase *) file:mrpt/hwdrivers/CCameraSensor.h line:533
	M("mrpt::hwdrivers").def("readConfigIntoVideoSourcePanel", (void (*)(void *, const std::string &, const class mrpt::config::CConfigFileBase *)) &mrpt::hwdrivers::readConfigIntoVideoSourcePanel, "Parse the given section of the given configuration file and set accordingly\n the controls of the wxWidgets \"panel\".\n Use with caution since \"panel\" MUST be a \"mrpt::gui::CPanelCameraSelection\n *\"\n \n\n prepareVideoSourceFromUserSelection, prepareVideoSourceFromPanel,\n writeConfigFromVideoSourcePanel\n\nC++: mrpt::hwdrivers::readConfigIntoVideoSourcePanel(void *, const std::string &, const class mrpt::config::CConfigFileBase *) --> void", pybind11::arg("panel"), pybind11::arg("in_cfgfile_section_name"), pybind11::arg("in_cfgfile"));

	// mrpt::hwdrivers::prepareVideoSourceFromUserSelection() file:mrpt/hwdrivers/CCameraSensor.h line:541
	M("mrpt::hwdrivers").def("prepareVideoSourceFromUserSelection", (class std::shared_ptr<class mrpt::hwdrivers::CCameraSensor> (*)()) &mrpt::hwdrivers::prepareVideoSourceFromUserSelection, "Show to the user a list of possible camera drivers and creates and open the\n selected camera.\n\nC++: mrpt::hwdrivers::prepareVideoSourceFromUserSelection() --> class std::shared_ptr<class mrpt::hwdrivers::CCameraSensor>");

}
