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
#include <mrpt/hwdrivers/CSICKTim561Eth_2050101.h>
#include <mrpt/hwdrivers/CSickLaserSerial.h>
#include <mrpt/hwdrivers/CSickLaserUSB.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
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
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

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
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable>, struct std::less<mrpt::Clock::time_point >, class std::allocator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > > >;
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
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable>, struct std::less<mrpt::Clock::time_point >, class std::allocator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > > >;
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

// mrpt::hwdrivers::CSickLaserUSB file:mrpt/hwdrivers/CSickLaserUSB.h line:62
struct PyCallBack_mrpt_hwdrivers_CSickLaserUSB : public mrpt::hwdrivers::CSickLaserUSB {
	using mrpt::hwdrivers::CSickLaserUSB::CSickLaserUSB;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CSickLaserUSB::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSickLaserUSB::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcessSimple(bool & a0, class mrpt::obs::CObservation2DRangeScan & a1, bool & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "doProcessSimple");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSickLaserUSB::doProcessSimple(a0, a1, a2);
	}
	bool turnOn() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "turnOn");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSickLaserUSB::turnOn();
	}
	bool turnOff() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "turnOff");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CSickLaserUSB::turnOff();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "doProcess");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "initialize");
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
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable>, struct std::less<mrpt::Clock::time_point >, class std::allocator<struct std::pair<const mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> > > >;
	_binder_ret_0 getObservations() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSickLaserUSB *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CSICKTim561Eth_2050101(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
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
	{ // mrpt::hwdrivers::CSickLaserUSB file:mrpt/hwdrivers/CSickLaserUSB.h line:62
		pybind11::class_<mrpt::hwdrivers::CSickLaserUSB, std::shared_ptr<mrpt::hwdrivers::CSickLaserUSB>, PyCallBack_mrpt_hwdrivers_CSickLaserUSB, mrpt::hwdrivers::C2DRangeFinderAbstract> cl(M("mrpt::hwdrivers"), "CSickLaserUSB", "This \"software driver\" implements the communication protocol for interfacing\n a SICK LMS2XX laser scanners through a custom USB RS-422 interface board.\n\n   NOTE that this class is for a custom hardware built at our lab (MAPIR,\n University of Malaga).\n   For a generic serial interface, see the class CSickLaserSerial.\n\n   This class does not need to be bind, i.e. you do not need to call\n C2DRangeFinderAbstract::bindIO. However, calling it will have not effect.\n   In this class the \"bind\" is ignored since it is designed for USB\n connections only, thus it internally generate the required object for\n simplicity of use.\n   The serial number of the USB device is used to open it on the first call\n to \"doProcess\", thus you must call \"loadConfig\" before this, or manually\n     call \"setDeviceSerialNumber\". The default serial number is \"LASER001\"\n\n Warning: Avoid defining an object of this class in a global scope if you\n want to catch all potential\n      exceptions during the constructors (like USB interface DLL not found,\n etc...)\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CSickLaserUSB(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CSickLaserUSB(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CSickLaserUSB::*)() const) &mrpt::hwdrivers::CSickLaserUSB::GetRuntimeClass, "C++: mrpt::hwdrivers::CSickLaserUSB::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CSickLaserUSB::CreateObject, "C++: mrpt::hwdrivers::CSickLaserUSB::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CSickLaserUSB::doRegister, "C++: mrpt::hwdrivers::CSickLaserUSB::doRegister() --> void");
		cl.def("setDeviceSerialNumber", (void (mrpt::hwdrivers::CSickLaserUSB::*)(const std::string &)) &mrpt::hwdrivers::CSickLaserUSB::setDeviceSerialNumber, "Changes the serial number of the device to open (call prior to\n 'doProcess')\n\nC++: mrpt::hwdrivers::CSickLaserUSB::setDeviceSerialNumber(const std::string &) --> void", pybind11::arg("deviceSerialNumber"));
		cl.def("doProcessSimple", (void (mrpt::hwdrivers::CSickLaserUSB::*)(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &)) &mrpt::hwdrivers::CSickLaserUSB::doProcessSimple, "Specific laser scanner \"software drivers\" must process here new data\n from the I/O stream, and, if a whole scan has arrived, return it.\n  This method will be typically called in a different thread than other\n methods, and will be called in a timely fashion.\n\nC++: mrpt::hwdrivers::CSickLaserUSB::doProcessSimple(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("turnOn", (bool (mrpt::hwdrivers::CSickLaserUSB::*)()) &mrpt::hwdrivers::CSickLaserUSB::turnOn, "Enables the scanning mode (in this class this has no effect).\n \n\n If everything works \"true\", or \"false\" if there is any error.\n\nC++: mrpt::hwdrivers::CSickLaserUSB::turnOn() --> bool");
		cl.def("turnOff", (bool (mrpt::hwdrivers::CSickLaserUSB::*)()) &mrpt::hwdrivers::CSickLaserUSB::turnOff, "Disables the scanning mode (in this class this has no effect).\n \n\n If everything works \"true\", or \"false\" if there is any error.\n\nC++: mrpt::hwdrivers::CSickLaserUSB::turnOff() --> bool");
	}
}
