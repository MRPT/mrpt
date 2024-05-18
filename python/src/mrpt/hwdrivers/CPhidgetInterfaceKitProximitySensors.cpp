#include <chrono>
#include <functional>
#include <ios>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/hwdrivers/CRaePID.h>
#include <mrpt/hwdrivers/CRoboPeakLidar.h>
#include <mrpt/hwdrivers/CSICKTim561Eth_2050101.h>
#include <mrpt/hwdrivers/CSickLaserSerial.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationRange.h>
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

// mrpt::hwdrivers::CRaePID file:mrpt/hwdrivers/CRaePID.h line:28
struct PyCallBack_mrpt_hwdrivers_CRaePID : public mrpt::hwdrivers::CRaePID {
	using mrpt::hwdrivers::CRaePID::CRaePID;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CRaePID::GetRuntimeClass();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRaePID::doProcess();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRaePID::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "loadConfig");
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
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGenericSensor::initialize();
	}
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> >;
	_binder_ret_0 getObservations() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRaePID *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CRoboPeakLidar file:mrpt/hwdrivers/CRoboPeakLidar.h line:50
struct PyCallBack_mrpt_hwdrivers_CRoboPeakLidar : public mrpt::hwdrivers::CRoboPeakLidar {
	using mrpt::hwdrivers::CRoboPeakLidar::CRoboPeakLidar;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CRoboPeakLidar::GetRuntimeClass();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRoboPeakLidar::initialize();
	}
	void doProcessSimple(bool & a0, class mrpt::obs::CObservation2DRangeScan & a1, bool & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "doProcessSimple");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRoboPeakLidar::doProcessSimple(a0, a1, a2);
	}
	bool turnOn() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "turnOn");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRoboPeakLidar::turnOn();
	}
	bool turnOff() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "turnOff");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CRoboPeakLidar::turnOff();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRoboPeakLidar::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return C2DRangeFinderAbstract::doProcess();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CRoboPeakLidar *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CSICKTim561Eth file:mrpt/hwdrivers/CSICKTim561Eth_2050101.h line:17
struct PyCallBack_mrpt_hwdrivers_CSICKTim561Eth : public mrpt::hwdrivers::CSICKTim561Eth {
	using mrpt::hwdrivers::CSICKTim561Eth::CSICKTim561Eth;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CSICKTim561Eth::GetRuntimeClass();
	}
	void doProcessSimple(bool & a0, class mrpt::obs::CObservation2DRangeScan & a1, bool & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "doProcessSimple");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSICKTim561Eth::doProcessSimple(a0, a1, a2);
	}
	bool turnOn() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "turnOn");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSICKTim561Eth::turnOn();
	}
	bool turnOff() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "turnOff");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSICKTim561Eth::turnOff();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSICKTim561Eth::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSICKTim561Eth::initialize();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSICKTim561Eth::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSICKTim561Eth *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CSickLaserSerial file:mrpt/hwdrivers/CSickLaserSerial.h line:67
struct PyCallBack_mrpt_hwdrivers_CSickLaserSerial : public mrpt::hwdrivers::CSickLaserSerial {
	using mrpt::hwdrivers::CSickLaserSerial::CSickLaserSerial;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CSickLaserSerial::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSickLaserSerial::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcessSimple(bool & a0, class mrpt::obs::CObservation2DRangeScan & a1, bool & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "doProcessSimple");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSickLaserSerial::doProcessSimple(a0, a1, a2);
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSickLaserSerial::initialize();
	}
	bool turnOn() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "turnOn");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSickLaserSerial::turnOn();
	}
	bool turnOff() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "turnOff");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSickLaserSerial::turnOff();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return C2DRangeFinderAbstract::doProcess();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserSerial *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CPhidgetInterfaceKitProximitySensors(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
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
	{ // mrpt::hwdrivers::CRaePID file:mrpt/hwdrivers/CRaePID.h line:28
		pybind11::class_<mrpt::hwdrivers::CRaePID, std::shared_ptr<mrpt::hwdrivers::CRaePID>, PyCallBack_mrpt_hwdrivers_CRaePID, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CRaePID", "This class implements a driver for the RAE Systems gas PhotoIonization\n Detector (PID) (Tested on a MiniRAE Lite)\n   The sensor is accessed via a standard (or USB) serial port.\n\n   Refer to the manufacturer website for details on this sensor:\n http://www.raesystems.com/products/minirae-lite\n\n  \n mrpt::obs::CObservationGasSensors\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CRaePID(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CRaePID(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CRaePID::*)() const) &mrpt::hwdrivers::CRaePID::GetRuntimeClass, "C++: mrpt::hwdrivers::CRaePID::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CRaePID::CreateObject, "C++: mrpt::hwdrivers::CRaePID::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CRaePID::doRegister, "C++: mrpt::hwdrivers::CRaePID::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CRaePID::*)()) &mrpt::hwdrivers::CRaePID::doProcess, "C++: mrpt::hwdrivers::CRaePID::doProcess() --> void");
		cl.def("loadConfig_sensorSpecific", (void (mrpt::hwdrivers::CRaePID::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::hwdrivers::CRaePID::loadConfig_sensorSpecific, "C++: mrpt::hwdrivers::CRaePID::loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("section"));
		cl.def("getFirmware", (std::string (mrpt::hwdrivers::CRaePID::*)()) &mrpt::hwdrivers::CRaePID::getFirmware, "Get firmware version string.\n\nC++: mrpt::hwdrivers::CRaePID::getFirmware() --> std::string");
		cl.def("getModel", (std::string (mrpt::hwdrivers::CRaePID::*)()) &mrpt::hwdrivers::CRaePID::getModel, "Get model string.\n\nC++: mrpt::hwdrivers::CRaePID::getModel() --> std::string");
		cl.def("getSerialNumber", (std::string (mrpt::hwdrivers::CRaePID::*)()) &mrpt::hwdrivers::CRaePID::getSerialNumber, "Get serial number as a string.\n\nC++: mrpt::hwdrivers::CRaePID::getSerialNumber() --> std::string");
		cl.def("getName", (std::string (mrpt::hwdrivers::CRaePID::*)()) &mrpt::hwdrivers::CRaePID::getName, "Get name string.\n\nC++: mrpt::hwdrivers::CRaePID::getName() --> std::string");
		cl.def("switchPower", (bool (mrpt::hwdrivers::CRaePID::*)()) &mrpt::hwdrivers::CRaePID::switchPower, "Switch power on or off (returns true if turned on).\n\nC++: mrpt::hwdrivers::CRaePID::switchPower() --> bool");
		cl.def("getFullInfo", (class mrpt::obs::CObservationGasSensors (mrpt::hwdrivers::CRaePID::*)()) &mrpt::hwdrivers::CRaePID::getFullInfo, "Get full reading (see PID documentation). In the returned observation,\n each reding is saved as a separate e-nose\n\nC++: mrpt::hwdrivers::CRaePID::getFullInfo() --> class mrpt::obs::CObservationGasSensors");
		cl.def("errorStatus", (bool (mrpt::hwdrivers::CRaePID::*)(std::string &)) &mrpt::hwdrivers::CRaePID::errorStatus, "Get error status (true if an error was found). errorString shows the\n error code (see PID documentation)\n\nC++: mrpt::hwdrivers::CRaePID::errorStatus(std::string &) --> bool", pybind11::arg("errorString"));
		cl.def("getLimits", (void (mrpt::hwdrivers::CRaePID::*)(float &, float &)) &mrpt::hwdrivers::CRaePID::getLimits, "Get alarm limits\n\nC++: mrpt::hwdrivers::CRaePID::getLimits(float &, float &) --> void", pybind11::arg("min"), pybind11::arg("max"));
	}
	{ // mrpt::hwdrivers::CRoboPeakLidar file:mrpt/hwdrivers/CRoboPeakLidar.h line:50
		pybind11::class_<mrpt::hwdrivers::CRoboPeakLidar, std::shared_ptr<mrpt::hwdrivers::CRoboPeakLidar>, PyCallBack_mrpt_hwdrivers_CRoboPeakLidar, mrpt::hwdrivers::C2DRangeFinderAbstract> cl(M("mrpt::hwdrivers"), "CRoboPeakLidar", "Interfaces a Robo Peak LIDAR laser scanner.\n\n  See the example \"samples/RoboPeakLidar_laser_test\" and the application\n \"rawlog-grabber\" for a ready-to-use application to gather data from the\n scanner.\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n Class introduced in MRPT 1.2.2\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CRoboPeakLidar(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CRoboPeakLidar(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CRoboPeakLidar::*)() const) &mrpt::hwdrivers::CRoboPeakLidar::GetRuntimeClass, "C++: mrpt::hwdrivers::CRoboPeakLidar::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CRoboPeakLidar::CreateObject, "C++: mrpt::hwdrivers::CRoboPeakLidar::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CRoboPeakLidar::doRegister, "C++: mrpt::hwdrivers::CRoboPeakLidar::doRegister() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CRoboPeakLidar::*)()) &mrpt::hwdrivers::CRoboPeakLidar::initialize, "Attempts to connect and turns the laser on. Raises an exception on\n error. \n\nC++: mrpt::hwdrivers::CRoboPeakLidar::initialize() --> void");
		cl.def("doProcessSimple", (void (mrpt::hwdrivers::CRoboPeakLidar::*)(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &)) &mrpt::hwdrivers::CRoboPeakLidar::doProcessSimple, "C++: mrpt::hwdrivers::CRoboPeakLidar::doProcessSimple(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("setSerialPort", (void (mrpt::hwdrivers::CRoboPeakLidar::*)(const std::string &)) &mrpt::hwdrivers::CRoboPeakLidar::setSerialPort, "If set to non-empty, the serial port will be attempted to be opened\n automatically when this class is first used to request data from the\n laser.  \n\nC++: mrpt::hwdrivers::CRoboPeakLidar::setSerialPort(const std::string &) --> void", pybind11::arg("port_name"));
		cl.def("getSerialPort", (const std::string (mrpt::hwdrivers::CRoboPeakLidar::*)()) &mrpt::hwdrivers::CRoboPeakLidar::getSerialPort, "Returns the currently set serial port \n setSerialPort \n\nC++: mrpt::hwdrivers::CRoboPeakLidar::getSerialPort() --> const std::string");
		cl.def("turnOn", (bool (mrpt::hwdrivers::CRoboPeakLidar::*)()) &mrpt::hwdrivers::CRoboPeakLidar::turnOn, "See base class docs \n\nC++: mrpt::hwdrivers::CRoboPeakLidar::turnOn() --> bool");
		cl.def("turnOff", (bool (mrpt::hwdrivers::CRoboPeakLidar::*)()) &mrpt::hwdrivers::CRoboPeakLidar::turnOff, "See base class docs \n\nC++: mrpt::hwdrivers::CRoboPeakLidar::turnOff() --> bool");
		cl.def("getDeviceHealth", (bool (mrpt::hwdrivers::CRoboPeakLidar::*)() const) &mrpt::hwdrivers::CRoboPeakLidar::getDeviceHealth, "Returns true if the device is connected & operative \n\nC++: mrpt::hwdrivers::CRoboPeakLidar::getDeviceHealth() const --> bool");
		cl.def("disconnect", (void (mrpt::hwdrivers::CRoboPeakLidar::*)()) &mrpt::hwdrivers::CRoboPeakLidar::disconnect, "Closes the comms with the laser. Shouldn't have to be directly needed by\n the user \n\nC++: mrpt::hwdrivers::CRoboPeakLidar::disconnect() --> void");
	}
	{ // mrpt::hwdrivers::CSICKTim561Eth file:mrpt/hwdrivers/CSICKTim561Eth_2050101.h line:17
		pybind11::class_<mrpt::hwdrivers::CSICKTim561Eth, std::shared_ptr<mrpt::hwdrivers::CSICKTim561Eth>, PyCallBack_mrpt_hwdrivers_CSICKTim561Eth, mrpt::hwdrivers::C2DRangeFinderAbstract> cl(M("mrpt::hwdrivers"), "CSICKTim561Eth", "");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CSICKTim561Eth(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CSICKTim561Eth(); } ), "doc");
		cl.def( pybind11::init( [](std::string const & a0){ return new mrpt::hwdrivers::CSICKTim561Eth(a0); }, [](std::string const & a0){ return new PyCallBack_mrpt_hwdrivers_CSICKTim561Eth(a0); } ), "doc");
		cl.def( pybind11::init<std::string, unsigned int>(), pybind11::arg("_ip"), pybind11::arg("_port") );

		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CSICKTim561Eth::*)() const) &mrpt::hwdrivers::CSICKTim561Eth::GetRuntimeClass, "C++: mrpt::hwdrivers::CSICKTim561Eth::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CSICKTim561Eth::CreateObject, "C++: mrpt::hwdrivers::CSICKTim561Eth::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CSICKTim561Eth::doRegister, "C++: mrpt::hwdrivers::CSICKTim561Eth::doRegister() --> void");
		cl.def("doProcessSimple", (void (mrpt::hwdrivers::CSICKTim561Eth::*)(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &)) &mrpt::hwdrivers::CSICKTim561Eth::doProcessSimple, "This function acquire a laser scan from the device. If an error occured,\n hardwareError will be set to true.\n The new laser scan will be stored in the outObservation argument.\n\n \n This method throw exception if the frame received from the\n TIM 561 contain the following bad parameters:\n * Status is not OK\n * Data in the scan aren't DIST1(may be RSSIx or DIST2).\n\nC++: mrpt::hwdrivers::CSICKTim561Eth::doProcessSimple(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("turnOn", (bool (mrpt::hwdrivers::CSICKTim561Eth::*)()) &mrpt::hwdrivers::CSICKTim561Eth::turnOn, "This method must be called before trying to get a laser scan.\n\nC++: mrpt::hwdrivers::CSICKTim561Eth::turnOn() --> bool");
		cl.def("turnOff", (bool (mrpt::hwdrivers::CSICKTim561Eth::*)()) &mrpt::hwdrivers::CSICKTim561Eth::turnOff, "This method could be called manually to stop communication with the\n device. Method is also called by destructor.\n\nC++: mrpt::hwdrivers::CSICKTim561Eth::turnOff() --> bool");
		cl.def("rebootDev", (bool (mrpt::hwdrivers::CSICKTim561Eth::*)()) &mrpt::hwdrivers::CSICKTim561Eth::rebootDev, "This method could be called manually to reboot the device.\n\nC++: mrpt::hwdrivers::CSICKTim561Eth::rebootDev() --> bool");
		cl.def("setSensorPose", (void (mrpt::hwdrivers::CSICKTim561Eth::*)(const class mrpt::poses::CPose3D &)) &mrpt::hwdrivers::CSICKTim561Eth::setSensorPose, "A method to set the sensor pose on the robot.\n Equivalent to setting the sensor pose via loading it from a config file.\n\nC++: mrpt::hwdrivers::CSICKTim561Eth::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("_pose"));
		cl.def("doProcess", (void (mrpt::hwdrivers::CSICKTim561Eth::*)()) &mrpt::hwdrivers::CSICKTim561Eth::doProcess, "This method must be called periodically. Period depend on the\n process_rate in the configuration file.\n\nC++: mrpt::hwdrivers::CSICKTim561Eth::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CSICKTim561Eth::*)()) &mrpt::hwdrivers::CSICKTim561Eth::initialize, "Initialize the sensor according to the parameters previously read\n in the configuration file.\n\nC++: mrpt::hwdrivers::CSICKTim561Eth::initialize() --> void");
	}
	{ // mrpt::hwdrivers::CSickLaserSerial file:mrpt/hwdrivers/CSickLaserSerial.h line:67
		pybind11::class_<mrpt::hwdrivers::CSickLaserSerial, std::shared_ptr<mrpt::hwdrivers::CSickLaserSerial>, PyCallBack_mrpt_hwdrivers_CSickLaserSerial, mrpt::hwdrivers::C2DRangeFinderAbstract> cl(M("mrpt::hwdrivers"), "CSickLaserSerial", "This \"software driver\" implements the communication protocol for interfacing\n a SICK LMS 2XX laser scanners through a standard RS232 serial port (or a\n USB2SERIAL converter).\n   The serial port is opened upon the first call to \"doProcess\" or\n \"initialize\", so you must call \"loadConfig\" before\n   this, or manually call \"setSerialPort\". Another alternative is to call the\n base class method C2DRangeFinderAbstract::bindIO,\n   but the \"setSerialPort\" interface is probably much simpler to use.\n\n   For an example of usage see the example in\n \"samples/SICK_laser_serial_test\".\n   See also the example configuration file for rawlog-grabber in\n \"share/mrpt/config_files/rawlog-grabber\".\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n C2DRangeFinderAbstract\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CSickLaserSerial(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CSickLaserSerial(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CSickLaserSerial::*)() const) &mrpt::hwdrivers::CSickLaserSerial::GetRuntimeClass, "C++: mrpt::hwdrivers::CSickLaserSerial::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CSickLaserSerial::CreateObject, "C++: mrpt::hwdrivers::CSickLaserSerial::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CSickLaserSerial::doRegister, "C++: mrpt::hwdrivers::CSickLaserSerial::doRegister() --> void");
		cl.def("setSerialPort", (void (mrpt::hwdrivers::CSickLaserSerial::*)(const std::string &)) &mrpt::hwdrivers::CSickLaserSerial::setSerialPort, "Changes the serial port to connect to (call prior to 'doProcess'), for\n example \"COM1\" or \"ttyS0\".\n  This is not needed if the configuration is loaded with \"loadConfig\".\n\nC++: mrpt::hwdrivers::CSickLaserSerial::setSerialPort(const std::string &) --> void", pybind11::arg("port"));
		cl.def("getSerialPort", (std::string (mrpt::hwdrivers::CSickLaserSerial::*)() const) &mrpt::hwdrivers::CSickLaserSerial::getSerialPort, "setSerialPort \n\nC++: mrpt::hwdrivers::CSickLaserSerial::getSerialPort() const --> std::string");
		cl.def("setBaudRate", (void (mrpt::hwdrivers::CSickLaserSerial::*)(int)) &mrpt::hwdrivers::CSickLaserSerial::setBaudRate, "Changes the serial port baud rate (call prior to 'doProcess'); valid\n values are 9600,38400 and 500000.\n  This is not needed if the configuration is loaded with \"loadConfig\".\n  \n\n getBaudRate \n\nC++: mrpt::hwdrivers::CSickLaserSerial::setBaudRate(int) --> void", pybind11::arg("baud"));
		cl.def("getBaudRate", (int (mrpt::hwdrivers::CSickLaserSerial::*)() const) &mrpt::hwdrivers::CSickLaserSerial::getBaudRate, "setBaudRate \n\nC++: mrpt::hwdrivers::CSickLaserSerial::getBaudRate() const --> int");
		cl.def("setMillimeterMode", [](mrpt::hwdrivers::CSickLaserSerial &o) -> void { return o.setMillimeterMode(); }, "");
		cl.def("setMillimeterMode", (void (mrpt::hwdrivers::CSickLaserSerial::*)(bool)) &mrpt::hwdrivers::CSickLaserSerial::setMillimeterMode, "Enables/Disables the millimeter mode, with a greater accuracy but a\n shorter range (default=false)\n  (call prior to 'doProcess') This is not needed if the configuration is\n loaded with \"loadConfig\".\n\nC++: mrpt::hwdrivers::CSickLaserSerial::setMillimeterMode(bool) --> void", pybind11::arg("mm_mode"));
		cl.def("setScanFOV", (void (mrpt::hwdrivers::CSickLaserSerial::*)(int)) &mrpt::hwdrivers::CSickLaserSerial::setScanFOV, "Set the scanning field of view - possible values are 100 or 180\n (default)\n  (call prior to 'doProcess') This is not needed if the configuration is\n loaded with \"loadConfig\".\n\nC++: mrpt::hwdrivers::CSickLaserSerial::setScanFOV(int) --> void", pybind11::arg("fov_degrees"));
		cl.def("getScanFOV", (int (mrpt::hwdrivers::CSickLaserSerial::*)() const) &mrpt::hwdrivers::CSickLaserSerial::getScanFOV, "C++: mrpt::hwdrivers::CSickLaserSerial::getScanFOV() const --> int");
		cl.def("setScanResolution", (void (mrpt::hwdrivers::CSickLaserSerial::*)(int)) &mrpt::hwdrivers::CSickLaserSerial::setScanResolution, "Set the scanning resolution, in units of 1/100 degree - Possible values\n are 25, 50 and 100, for 0.25, 0.5 (default) and 1 deg.\n  (call prior to 'doProcess') This is not needed if the configuration is\n loaded with \"loadConfig\".\n\nC++: mrpt::hwdrivers::CSickLaserSerial::setScanResolution(int) --> void", pybind11::arg("res_1_100th_degree"));
		cl.def("getScanResolution", (int (mrpt::hwdrivers::CSickLaserSerial::*)() const) &mrpt::hwdrivers::CSickLaserSerial::getScanResolution, "C++: mrpt::hwdrivers::CSickLaserSerial::getScanResolution() const --> int");
		cl.def("getCurrentConnectTry", (unsigned int (mrpt::hwdrivers::CSickLaserSerial::*)() const) &mrpt::hwdrivers::CSickLaserSerial::getCurrentConnectTry, "If performing several tries in ::initialize(), this is the current try\n loop number. \n\nC++: mrpt::hwdrivers::CSickLaserSerial::getCurrentConnectTry() const --> unsigned int");
		cl.def("doProcessSimple", (void (mrpt::hwdrivers::CSickLaserSerial::*)(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &)) &mrpt::hwdrivers::CSickLaserSerial::doProcessSimple, "Specific laser scanner \"software drivers\" must process here new data\n from the I/O stream, and, if a whole scan has arrived, return it.\n  This method will be typically called in a different thread than other\n methods, and will be called in a timely fashion.\n\nC++: mrpt::hwdrivers::CSickLaserSerial::doProcessSimple(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("initialize", (void (mrpt::hwdrivers::CSickLaserSerial::*)()) &mrpt::hwdrivers::CSickLaserSerial::initialize, "Set-up communication with the laser.\n  Called automatically by rawlog-grabber.\n  If used manually, call after \"loadConfig\" and before \"doProcess\".\n\n  In this class this method does nothing, since the communications are\n setup at the first try from \"doProcess\" or \"doProcessSimple\".\n\nC++: mrpt::hwdrivers::CSickLaserSerial::initialize() --> void");
		cl.def("turnOn", (bool (mrpt::hwdrivers::CSickLaserSerial::*)()) &mrpt::hwdrivers::CSickLaserSerial::turnOn, "Enables the scanning mode (in this class this has no effect).\n \n\n If everything works \"true\", or \"false\" if there is any error.\n\nC++: mrpt::hwdrivers::CSickLaserSerial::turnOn() --> bool");
		cl.def("turnOff", (bool (mrpt::hwdrivers::CSickLaserSerial::*)()) &mrpt::hwdrivers::CSickLaserSerial::turnOff, "Disables the scanning mode (in this class this has no effect).\n \n\n If everything works \"true\", or \"false\" if there is any error.\n\nC++: mrpt::hwdrivers::CSickLaserSerial::turnOff() --> bool");
	}
}
