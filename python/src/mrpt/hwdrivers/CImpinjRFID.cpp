#include <chrono>
#include <functional>
#include <ios>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CImpinjRFID.h>
#include <mrpt/hwdrivers/CJoystick.h>
#include <mrpt/hwdrivers/CLMS100eth.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationRFID.h>
#include <mrpt/obs/T2DScanProperties.h>
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
#include <streambuf>
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

// mrpt::hwdrivers::CImpinjRFID file:mrpt/hwdrivers/CImpinjRFID.h line:27
struct PyCallBack_mrpt_hwdrivers_CImpinjRFID : public mrpt::hwdrivers::CImpinjRFID {
	using mrpt::hwdrivers::CImpinjRFID::CImpinjRFID;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CImpinjRFID::GetRuntimeClass();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImpinjRFID::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImpinjRFID::initialize();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CImpinjRFID::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CImpinjRFID *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CLMS100Eth file:mrpt/hwdrivers/CLMS100eth.h line:72
struct PyCallBack_mrpt_hwdrivers_CLMS100Eth : public mrpt::hwdrivers::CLMS100Eth {
	using mrpt::hwdrivers::CLMS100Eth::CLMS100Eth;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CLMS100Eth::GetRuntimeClass();
	}
	void doProcessSimple(bool & a0, class mrpt::obs::CObservation2DRangeScan & a1, bool & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "doProcessSimple");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLMS100Eth::doProcessSimple(a0, a1, a2);
	}
	bool turnOn() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "turnOn");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CLMS100Eth::turnOn();
	}
	bool turnOff() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "turnOff");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CLMS100Eth::turnOff();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLMS100Eth::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLMS100Eth::initialize();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLMS100Eth::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CLMS100Eth *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CImpinjRFID(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CImpinjRFID file:mrpt/hwdrivers/CImpinjRFID.h line:27
		pybind11::class_<mrpt::hwdrivers::CImpinjRFID, std::shared_ptr<mrpt::hwdrivers::CImpinjRFID>, PyCallBack_mrpt_hwdrivers_CImpinjRFID, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CImpinjRFID", "This class implements an interface to an Impinj RFID reader. This object\n connects to a program that does the actual communication with the receiver.\n This is done because the manufacturer only provides libraries for C# and\n Java. The program that runs the device must be started after this object\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CImpinjRFID(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CImpinjRFID(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CImpinjRFID::*)() const) &mrpt::hwdrivers::CImpinjRFID::GetRuntimeClass, "C++: mrpt::hwdrivers::CImpinjRFID::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CImpinjRFID::CreateObject, "C++: mrpt::hwdrivers::CImpinjRFID::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CImpinjRFID::doRegister, "C++: mrpt::hwdrivers::CImpinjRFID::doRegister() --> void");
		cl.def("connect", (void (mrpt::hwdrivers::CImpinjRFID::*)()) &mrpt::hwdrivers::CImpinjRFID::connect, "Connect to the reader.\n\nC++: mrpt::hwdrivers::CImpinjRFID::connect() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CImpinjRFID::*)()) &mrpt::hwdrivers::CImpinjRFID::doProcess, "C++: mrpt::hwdrivers::CImpinjRFID::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CImpinjRFID::*)()) &mrpt::hwdrivers::CImpinjRFID::initialize, "C++: mrpt::hwdrivers::CImpinjRFID::initialize() --> void");
		cl.def("loadConfig_sensorSpecific", (void (mrpt::hwdrivers::CImpinjRFID::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::hwdrivers::CImpinjRFID::loadConfig_sensorSpecific, "C++: mrpt::hwdrivers::CImpinjRFID::loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("section"));
		cl.def("getObservation", (bool (mrpt::hwdrivers::CImpinjRFID::*)(class mrpt::obs::CObservationRFID &)) &mrpt::hwdrivers::CImpinjRFID::getObservation, "Gets the information of the tags as a timestamped observation\n NOTE: Deprecated, use getObservations instead. See CGenericSensor\n documentation. This function is kept for internal use of the module\n \n\n Returns true if the observation was correct, and false otherwise\n \n\n mrpt::hwdrivers::CGenericSensor\n\nC++: mrpt::hwdrivers::CImpinjRFID::getObservation(class mrpt::obs::CObservationRFID &) --> bool", pybind11::arg("obs"));
		cl.def("closeReader", (void (mrpt::hwdrivers::CImpinjRFID::*)()) &mrpt::hwdrivers::CImpinjRFID::closeReader, "Close the connection to the reader.\n\nC++: mrpt::hwdrivers::CImpinjRFID::closeReader() --> void");
	}
	{ // mrpt::hwdrivers::CJoystick file:mrpt/hwdrivers/CJoystick.h line:28
		pybind11::class_<mrpt::hwdrivers::CJoystick, std::shared_ptr<mrpt::hwdrivers::CJoystick>> cl(M("mrpt::hwdrivers"), "CJoystick", "Access to joysticks and gamepads (read buttons and position), and request\n number of joysticks in the system.\n\n \n New in MRPT 2.13.0: the API changed to support an arbitrary number of\n       input axes.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CJoystick(); } ) );
		cl.def( pybind11::init( [](mrpt::hwdrivers::CJoystick const &o){ return new mrpt::hwdrivers::CJoystick(o); } ) );
		cl.def_static("getJoysticksCount", (int (*)()) &mrpt::hwdrivers::CJoystick::getJoysticksCount, "Returns the number of Joysticks in the computer.\n\nC++: mrpt::hwdrivers::CJoystick::getJoysticksCount() --> int");
		cl.def("getJoystickPosition", (bool (mrpt::hwdrivers::CJoystick::*)(int, struct mrpt::hwdrivers::CJoystick::State &)) &mrpt::hwdrivers::CJoystick::getJoystickPosition, "Gets joystick information.\n\n   This method will try first to open the joystick, so you can safely call\n it while the joystick is plugged and removed arbitrarly.\n\n \n The index of the joystick to query: The first one is 0, the\n second 1, etc... See CJoystick::getJoysticksCount to discover the number\n of joysticks in the system.\n \n\n The x axis position, range [-1,1]\n \n\n The y axis position, range [-1,1]\n \n\n The z axis position, range [-1,1]\n \n\n Each element will hold true if buttons are pressed. The\n size of the vector will be set automatically to the number of buttons.\n \n\n If it is desired the raw integer measurement from\n JoyStick, set this pointer to a desired placeholder.\n \n\n If it is desired the raw integer measurement from\n JoyStick, set this pointer to a desired placeholder.\n \n\n If it is desired the raw integer measurement from\n JoyStick, set this pointer to a desired placeholder.\n\n \n Returns true if successfull, false on error, for example, if\n joystick is not present.\n\n \n setLimits\n\nC++: mrpt::hwdrivers::CJoystick::getJoystickPosition(int, struct mrpt::hwdrivers::CJoystick::State &) --> bool", pybind11::arg("nJoy"), pybind11::arg("output"));
		cl.def("assign", (class mrpt::hwdrivers::CJoystick & (mrpt::hwdrivers::CJoystick::*)(const class mrpt::hwdrivers::CJoystick &)) &mrpt::hwdrivers::CJoystick::operator=, "C++: mrpt::hwdrivers::CJoystick::operator=(const class mrpt::hwdrivers::CJoystick &) --> class mrpt::hwdrivers::CJoystick &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::hwdrivers::CJoystick::State file:mrpt/hwdrivers/CJoystick.h line:60
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::hwdrivers::CJoystick::State, std::shared_ptr<mrpt::hwdrivers::CJoystick::State>> cl(enclosing_class, "State", "");
			cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CJoystick::State(); } ) );
			cl.def( pybind11::init( [](mrpt::hwdrivers::CJoystick::State const &o){ return new mrpt::hwdrivers::CJoystick::State(o); } ) );
			cl.def_readwrite("buttons", &mrpt::hwdrivers::CJoystick::State::buttons);
			cl.def_readwrite("axes", &mrpt::hwdrivers::CJoystick::State::axes);
			cl.def_readwrite("axes_raw", &mrpt::hwdrivers::CJoystick::State::axes_raw);
			cl.def("assign", (struct mrpt::hwdrivers::CJoystick::State & (mrpt::hwdrivers::CJoystick::State::*)(const struct mrpt::hwdrivers::CJoystick::State &)) &mrpt::hwdrivers::CJoystick::State::operator=, "C++: mrpt::hwdrivers::CJoystick::State::operator=(const struct mrpt::hwdrivers::CJoystick::State &) --> struct mrpt::hwdrivers::CJoystick::State &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::hwdrivers::CLMS100Eth file:mrpt/hwdrivers/CLMS100eth.h line:72
		pybind11::class_<mrpt::hwdrivers::CLMS100Eth, std::shared_ptr<mrpt::hwdrivers::CLMS100Eth>, PyCallBack_mrpt_hwdrivers_CLMS100Eth, mrpt::hwdrivers::C2DRangeFinderAbstract> cl(M("mrpt::hwdrivers"), "CLMS100Eth", "This \"software driver\" implements the communication protocol for interfacing\na SICK LMS100 laser scanners through an ethernet controller.\n   This class does not need to be bind, i.e. you do not need to call\nC2DRangeFinderAbstract::bindIO.\n   Connection is established when user call the turnOn() method. You can\npass to the class's constructor the LMS100 's ip address and port.\n   Device will be configured with the following parameters :\n - Start Angle : -45 deg (imposed by hardware)\n - Stop Angle : +225 deg (imposed by hardware)\n - Apperture : 270 deg (imposed by hardware)\n - Angular resolution : 0.25 deg\n - Scan frequency : 25 Hz\n - Max Range : 20m (imposed by hardware).\n\n Important note: SICK LMS 1xx devices have two levels of\nconfiguration. In its present implementation, this class only handles one of\nthem, so\n    before using this class, you must \"pre-configure\" your scanner\nwith the SICK's software \"SOAP\" (this software ships with the device),\n    and set the framerate with this software. Of course, you have to\npre-configure the device just once, then save that configuration in its flash\nmemory.\n\n To get a laser scan you must proceed like that :\n \n\n\n\n\n\n\n\n The sensor pose on the vehicle could be loaded from an ini configuration\nfile with :\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n This class doesn't configure the SICK LMS sensor, it is recomended to\nconfigure the sensor via the\n the SICK software : SOPAS.\n \n\n This class was contributed by Adrien Barral - Robopec (France)\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CLMS100Eth(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CLMS100Eth(); } ), "doc");
		cl.def( pybind11::init( [](std::string const & a0){ return new mrpt::hwdrivers::CLMS100Eth(a0); }, [](std::string const & a0){ return new PyCallBack_mrpt_hwdrivers_CLMS100Eth(a0); } ), "doc");
		cl.def( pybind11::init<std::string, unsigned int>(), pybind11::arg("_ip"), pybind11::arg("_port") );

		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CLMS100Eth::*)() const) &mrpt::hwdrivers::CLMS100Eth::GetRuntimeClass, "C++: mrpt::hwdrivers::CLMS100Eth::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CLMS100Eth::CreateObject, "C++: mrpt::hwdrivers::CLMS100Eth::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CLMS100Eth::doRegister, "C++: mrpt::hwdrivers::CLMS100Eth::doRegister() --> void");
		cl.def("doProcessSimple", (void (mrpt::hwdrivers::CLMS100Eth::*)(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &)) &mrpt::hwdrivers::CLMS100Eth::doProcessSimple, "This function acquire a laser scan from the device. If an error occured,\n hardwareError will be set to true.\n The new laser scan will be stored in the outObservation argument.\n\n \n This method throw exception if the frame received from the\n LMS 100 contain the following bad parameters :\n  * Status is not OK\n  * Data in the scan aren't DIST1 (may be RSSIx or DIST2).\n\nC++: mrpt::hwdrivers::CLMS100Eth::doProcessSimple(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("turnOn", (bool (mrpt::hwdrivers::CLMS100Eth::*)()) &mrpt::hwdrivers::CLMS100Eth::turnOn, "This method must be called before trying to get a laser scan.\n\nC++: mrpt::hwdrivers::CLMS100Eth::turnOn() --> bool");
		cl.def("turnOff", (bool (mrpt::hwdrivers::CLMS100Eth::*)()) &mrpt::hwdrivers::CLMS100Eth::turnOff, "This method could be called manually to stop communication with the\n device. Method is also called by destructor.\n\nC++: mrpt::hwdrivers::CLMS100Eth::turnOff() --> bool");
		cl.def("setSensorPose", (void (mrpt::hwdrivers::CLMS100Eth::*)(const class mrpt::poses::CPose3D &)) &mrpt::hwdrivers::CLMS100Eth::setSensorPose, "A method to set the sensor pose on the robot.\n Equivalent to setting the sensor pose via loading it from a config\n file.\n\nC++: mrpt::hwdrivers::CLMS100Eth::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("_pose"));
		cl.def("doProcess", (void (mrpt::hwdrivers::CLMS100Eth::*)()) &mrpt::hwdrivers::CLMS100Eth::doProcess, "This method should be called periodically. Period depend on the\n process_rate in the configuration file.\n\nC++: mrpt::hwdrivers::CLMS100Eth::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CLMS100Eth::*)()) &mrpt::hwdrivers::CLMS100Eth::initialize, "Initialize the sensor according to the parameters previously read in the\n configuration file.\n\nC++: mrpt::hwdrivers::CLMS100Eth::initialize() --> void");
	}
}
