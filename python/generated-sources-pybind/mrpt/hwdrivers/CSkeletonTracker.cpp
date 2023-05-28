#include <chrono>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/core/Clock.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/CSkeletonTracker.h>
#include <mrpt/hwdrivers/CTaoboticsIMU.h>
#include <mrpt/hwdrivers/CTuMicos.h>
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
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::hwdrivers::CSkeletonTracker file:mrpt/hwdrivers/CSkeletonTracker.h line:47
struct PyCallBack_mrpt_hwdrivers_CSkeletonTracker : public mrpt::hwdrivers::CSkeletonTracker {
	using mrpt::hwdrivers::CSkeletonTracker::CSkeletonTracker;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CSkeletonTracker::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkeletonTracker::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkeletonTracker::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CSkeletonTracker::initialize();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CSkeletonTracker *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CTaoboticsIMU file:mrpt/hwdrivers/CTaoboticsIMU.h line:85
struct PyCallBack_mrpt_hwdrivers_CTaoboticsIMU : public mrpt::hwdrivers::CTaoboticsIMU {
	using mrpt::hwdrivers::CTaoboticsIMU::CTaoboticsIMU;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CTaoboticsIMU::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTaoboticsIMU::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTaoboticsIMU::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTaoboticsIMU::initialize();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTaoboticsIMU *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CTuMicos file:mrpt/hwdrivers/CTuMicos.h line:22
struct PyCallBack_mrpt_hwdrivers_CTuMicos : public mrpt::hwdrivers::CTuMicos {
	using mrpt::hwdrivers::CTuMicos::CTuMicos;

	bool rangeMeasure() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "rangeMeasure");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::rangeMeasure();
	}
	bool moveToAbsPos(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "moveToAbsPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::moveToAbsPos(a0, a1);
	}
	bool absPosQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "absPosQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::absPosQ(a0, a1);
	}
	bool moveToOffPos(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "moveToOffPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::moveToOffPos(a0, a1);
	}
	bool offPosQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "offPosQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::offPosQ(a0, a1);
	}
	bool maxPosQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "maxPosQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::maxPosQ(a0, a1);
	}
	bool minPosQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "minPosQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::minPosQ(a0, a1);
	}
	bool enableLimitsQ(bool & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "enableLimitsQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::enableLimitsQ(a0);
	}
	bool enableLimits(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "enableLimits");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::enableLimits(a0);
	}
	bool inmediateExecution(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "inmediateExecution");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::inmediateExecution(a0);
	}
	bool aWait() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "aWait");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::aWait();
	}
	bool haltAll() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "haltAll");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::haltAll();
	}
	bool halt(char a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "halt");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::halt(a0);
	}
	bool speed(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "speed");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::speed(a0, a1);
	}
	bool speedQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "speedQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::speedQ(a0, a1);
	}
	bool aceleration(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "aceleration");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::aceleration(a0, a1);
	}
	bool acelerationQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "acelerationQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::acelerationQ(a0, a1);
	}
	bool baseSpeed(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "baseSpeed");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::baseSpeed(a0, a1);
	}
	bool baseSpeedQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "baseSpeedQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::baseSpeedQ(a0, a1);
	}
	bool upperSpeed(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "upperSpeed");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::upperSpeed(a0, a1);
	}
	bool upperSpeedQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "upperSpeedQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::upperSpeedQ(a0, a1);
	}
	bool lowerSpeed(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "lowerSpeed");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::lowerSpeed(a0, a1);
	}
	bool lowerSpeedQ(char a0, double & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "lowerSpeedQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::lowerSpeedQ(a0, a1);
	}
	bool reset() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "reset");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::reset();
	}
	bool save() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "save");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::save();
	}
	bool restoreDefaults() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "restoreDefaults");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::restoreDefaults();
	}
	bool restoreFactoryDefaults() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "restoreFactoryDefaults");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::restoreFactoryDefaults();
	}
	bool version(char * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "version");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::version(a0);
	}
	void nversion(double & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "nversion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTuMicos::nversion(a0);
	}
	bool powerModeQ(bool a0, char & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "powerModeQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::powerModeQ(a0, a1);
	}
	bool powerMode(bool a0, char a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "powerMode");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::powerMode(a0, a1);
	}
	bool setLimits(char a0, double & a1, double & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "setLimits");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::setLimits(a0, a1, a2);
	}
	bool changeMotionDir() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "changeMotionDir");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::changeMotionDir();
	}
	int checkErrors() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "checkErrors");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<int>::value) {
				static pybind11::detail::override_caster_t<int> caster;
				return pybind11::detail::cast_ref<int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<int>(std::move(o));
		}
		return CTuMicos::checkErrors();
	}
	void clearErrors() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "clearErrors");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTuMicos::clearErrors();
	}
	bool init(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "init");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::init(a0);
	}
	void close() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "close");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CTuMicos::close();
	}
	double radError(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "radError");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CTuMicos::radError(a0, a1);
	}
	long radToPos(char a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "radToPos");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<long>::value) {
				static pybind11::detail::override_caster_t<long> caster;
				return pybind11::detail::cast_ref<long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<long>(std::move(o));
		}
		return CTuMicos::radToPos(a0, a1);
	}
	double posToRad(char a0, long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "posToRad");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CTuMicos::posToRad(a0, a1);
	}
	bool scan(char a0, int a1, float a2, float a3, double a4) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "scan");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3, a4);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::scan(a0, a1, a2, a3, a4);
	}
	bool verboseQ(bool & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "verboseQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::verboseQ(a0);
	}
	bool verbose(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "verbose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::verbose(a0);
	}
	bool echoModeQ(bool & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "echoModeQ");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::echoModeQ(a0);
	}
	bool echoMode(bool a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "echoMode");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::echoMode(a0);
	}
	bool resolution() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "resolution");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CTuMicos::resolution();
	}
	double status(double & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CTuMicos *>(this), "status");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CTuMicos::status(a0);
	}
};

void bind_mrpt_hwdrivers_CSkeletonTracker(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CSkeletonTracker file:mrpt/hwdrivers/CSkeletonTracker.h line:47
		pybind11::class_<mrpt::hwdrivers::CSkeletonTracker, std::shared_ptr<mrpt::hwdrivers::CSkeletonTracker>, PyCallBack_mrpt_hwdrivers_CSkeletonTracker, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CSkeletonTracker", "A class for grabbing mrpt::obs::CObservationSkeleton from a PrimeSense\ncamera.\n  It connects to a PrimeSense camera and tries to detect users while\nrecording the positions of their skeletons' joints along time.\n\n  See also the application \"rawlog-grabber\" for a ready-to-use application to\ngather data from this sensor.\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n\n  ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CSkeletonTracker(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CSkeletonTracker(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CSkeletonTracker::*)() const) &mrpt::hwdrivers::CSkeletonTracker::GetRuntimeClass, "C++: mrpt::hwdrivers::CSkeletonTracker::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CSkeletonTracker::CreateObject, "C++: mrpt::hwdrivers::CSkeletonTracker::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CSkeletonTracker::doRegister, "C++: mrpt::hwdrivers::CSkeletonTracker::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CSkeletonTracker::*)()) &mrpt::hwdrivers::CSkeletonTracker::doProcess, "This method will be invoked at a minimum rate of \"process_rate\" (Hz)\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CSkeletonTracker::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CSkeletonTracker::*)()) &mrpt::hwdrivers::CSkeletonTracker::initialize, "Connects to the PrimeSense camera and prepares it to get skeleton data\n\nC++: mrpt::hwdrivers::CSkeletonTracker::initialize() --> void");
		cl.def("setPreview", [](mrpt::hwdrivers::CSkeletonTracker &o) -> void { return o.setPreview(); }, "");
		cl.def("setPreview", (void (mrpt::hwdrivers::CSkeletonTracker::*)(const bool)) &mrpt::hwdrivers::CSkeletonTracker::setPreview, "Set/unset preview \n\nC++: mrpt::hwdrivers::CSkeletonTracker::setPreview(const bool) --> void", pybind11::arg("setPreview"));
	}
	{ // mrpt::hwdrivers::CTaoboticsIMU file:mrpt/hwdrivers/CTaoboticsIMU.h line:85
		pybind11::class_<mrpt::hwdrivers::CTaoboticsIMU, std::shared_ptr<mrpt::hwdrivers::CTaoboticsIMU>, PyCallBack_mrpt_hwdrivers_CTaoboticsIMU, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CTaoboticsIMU", "A driver for Taobotics IMU.\n\n Supported models: HFI-A9, HFI-B9, HFI-B6.\n\n ## Frame format\n From analysis the provided [python scripts]()\n we found that the sensor uses these data frames, documented here for\n reference:\n\n ### Model: \"hfi-a9\"\n\n  Two frames:\n  Frame field :  0xAA 0x55 0x2c | (9 * 4[float]) D0-D8  | ?? ??\n  Byte index  :   0     1    2  | 3 4 5 6 |  ...        | 48\n   Data:\n    - D{0,1,2}: wx,wy,wz\n    - D{3,4,5}: accx,accy,accz\n    - D{6,7,8}: magx,magy,magz\n\n  Frame field :  0xAA 0x55 0x14 | (5 * 4[float]) D0-D4   | ??\n  Byte index  :   0     1    2  |  3 ... |      ...      | 24\n\n ### Model: \"hfi-b6\"\n\n  Frame field :  0x55 | TYPE |  D0  |  D1  |  D2  |  D3  | CHKSUM\n  Byte index  :   0   |  1   | 2  3 | 4  5 | 6  7 | 8  9 |   10\n  Type:\n  0x51: acceleration, 0x52: angular velocity, 0x53: euler angles\n\n ### Configuration file block\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n If used programmatically, this class will be used as:\n\n \n\n\n\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CTaoboticsIMU(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CTaoboticsIMU(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CTaoboticsIMU::*)() const) &mrpt::hwdrivers::CTaoboticsIMU::GetRuntimeClass, "C++: mrpt::hwdrivers::CTaoboticsIMU::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CTaoboticsIMU::CreateObject, "C++: mrpt::hwdrivers::CTaoboticsIMU::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CTaoboticsIMU::doRegister, "C++: mrpt::hwdrivers::CTaoboticsIMU::doRegister() --> void");
		cl.def("loadConfig_sensorSpecific", (void (mrpt::hwdrivers::CTaoboticsIMU::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::hwdrivers::CTaoboticsIMU::loadConfig_sensorSpecific, "See the class documentation at the top for expected parameters \n\nC++: mrpt::hwdrivers::CTaoboticsIMU::loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("iniSection"));
		cl.def("doProcess", (void (mrpt::hwdrivers::CTaoboticsIMU::*)()) &mrpt::hwdrivers::CTaoboticsIMU::doProcess, "This method will be invoked at a minimum rate of \"process_rate\" (Hz)\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CTaoboticsIMU::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CTaoboticsIMU::*)()) &mrpt::hwdrivers::CTaoboticsIMU::initialize, "Opens the serial port and start streaming data.\n You must have called loadConfig_sensorSpecific before\n calling this function, or set the configuration via the provided methods,\n e.g. setSerialPort(), etc.\n\nC++: mrpt::hwdrivers::CTaoboticsIMU::initialize() --> void");
		cl.def("setSerialPort", (void (mrpt::hwdrivers::CTaoboticsIMU::*)(const std::string &)) &mrpt::hwdrivers::CTaoboticsIMU::setSerialPort, "Must be called before initialize(). If not set, the default is\n \"/dev/ttyUSB0\". Use \"COM1\", etc. for Windows.\n\nC++: mrpt::hwdrivers::CTaoboticsIMU::setSerialPort(const std::string &) --> void", pybind11::arg("serialPort"));
		cl.def("setSerialBaudRate", (void (mrpt::hwdrivers::CTaoboticsIMU::*)(int)) &mrpt::hwdrivers::CTaoboticsIMU::setSerialBaudRate, "Must be called before initialize(). If not called, the default 921600 is\n used.\n\nC++: mrpt::hwdrivers::CTaoboticsIMU::setSerialBaudRate(int) --> void", pybind11::arg("rate"));
	}
	{ // mrpt::hwdrivers::CTuMicos file:mrpt/hwdrivers/CTuMicos.h line:22
		pybind11::class_<mrpt::hwdrivers::CTuMicos, std::shared_ptr<mrpt::hwdrivers::CTuMicos>, PyCallBack_mrpt_hwdrivers_CTuMicos, mrpt::hwdrivers::CPtuBase> cl(M("mrpt::hwdrivers"), "CTuMicos", "This class implements initialization and communication methods to\n control a Tilt Unit model DT-80, working in radians .\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CTuMicos(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CTuMicos(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_hwdrivers_CTuMicos const &o){ return new PyCallBack_mrpt_hwdrivers_CTuMicos(o); } ) );
		cl.def( pybind11::init( [](mrpt::hwdrivers::CTuMicos const &o){ return new mrpt::hwdrivers::CTuMicos(o); } ) );
		cl.def_readwrite("axis_index", &mrpt::hwdrivers::CTuMicos::axis_index);
		cl.def("rangeMeasure", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::rangeMeasure, "Search limit forward \n\nC++: mrpt::hwdrivers::CTuMicos::rangeMeasure() --> bool");
		cl.def("moveToAbsPos", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double)) &mrpt::hwdrivers::CTuMicos::moveToAbsPos, "Specification of positions in absolute terms \n\nC++: mrpt::hwdrivers::CTuMicos::moveToAbsPos(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("absPosQ", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &)) &mrpt::hwdrivers::CTuMicos::absPosQ, "Query position in absolute terms \n\nC++: mrpt::hwdrivers::CTuMicos::absPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("moveToOffPos", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double)) &mrpt::hwdrivers::CTuMicos::moveToOffPos, "Specify desired axis position as an offset from the current position. \n	This method recives the number of radians to move.\n	\n\n\n\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CTuMicos::moveToOffPos(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("offPosQ", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &)) &mrpt::hwdrivers::CTuMicos::offPosQ, "Query position in relative terms \n\nC++: mrpt::hwdrivers::CTuMicos::offPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("maxPosQ", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &)) &mrpt::hwdrivers::CTuMicos::maxPosQ, "Query max movement limit of a axis in absolute terms \n\nC++: mrpt::hwdrivers::CTuMicos::maxPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("minPosQ", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &)) &mrpt::hwdrivers::CTuMicos::minPosQ, "Query min movement limit of a axis in absolute terms \n\nC++: mrpt::hwdrivers::CTuMicos::minPosQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("enableLimitsQ", (bool (mrpt::hwdrivers::CTuMicos::*)(bool &)) &mrpt::hwdrivers::CTuMicos::enableLimitsQ, "Query if exist movement limits \n\nC++: mrpt::hwdrivers::CTuMicos::enableLimitsQ(bool &) --> bool", pybind11::arg("enable"));
		cl.def("enableLimits", (bool (mrpt::hwdrivers::CTuMicos::*)(bool)) &mrpt::hwdrivers::CTuMicos::enableLimits, "Enable/Disable movement limits \n\nC++: mrpt::hwdrivers::CTuMicos::enableLimits(bool) --> bool", pybind11::arg("set"));
		cl.def("inmediateExecution", (bool (mrpt::hwdrivers::CTuMicos::*)(bool)) &mrpt::hwdrivers::CTuMicos::inmediateExecution, "With I mode (default) instructs pan-tilt unit to immediately\n	execute positional commands. \n	In S mode instructs pan-tilt unit to execute positional commands\n	only when an Await Position Command Completion command is executed\n	or when put into Immediate Execution Mode. \n	\n\n\n\n\n\n\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CTuMicos::inmediateExecution(bool) --> bool", pybind11::arg("set"));
		cl.def("aWait", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::aWait, "Wait the finish of the last position command to\n	continue accept commands\n\nC++: mrpt::hwdrivers::CTuMicos::aWait() --> bool");
		cl.def("haltAll", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::haltAll, "Inmediately stop all \n\nC++: mrpt::hwdrivers::CTuMicos::haltAll() --> bool");
		cl.def("halt", (bool (mrpt::hwdrivers::CTuMicos::*)(char)) &mrpt::hwdrivers::CTuMicos::halt, "Inmediately stop \n\nC++: mrpt::hwdrivers::CTuMicos::halt(char) --> bool", pybind11::arg("axis"));
		cl.def("speed", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double)) &mrpt::hwdrivers::CTuMicos::speed, "Specification of turn speed \n\nC++: mrpt::hwdrivers::CTuMicos::speed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("speedQ", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &)) &mrpt::hwdrivers::CTuMicos::speedQ, "Query turn speed \n\nC++: mrpt::hwdrivers::CTuMicos::speedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("aceleration", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double)) &mrpt::hwdrivers::CTuMicos::aceleration, "Specification (de/a)celeration in turn \n\nC++: mrpt::hwdrivers::CTuMicos::aceleration(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec2"));
		cl.def("acelerationQ", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &)) &mrpt::hwdrivers::CTuMicos::acelerationQ, "Query (de/a)celeration in turn \n\nC++: mrpt::hwdrivers::CTuMicos::acelerationQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec2"));
		cl.def("baseSpeed", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double)) &mrpt::hwdrivers::CTuMicos::baseSpeed, "Specification of velocity to which start and finish\n	the (de/a)celeration\n\nC++: mrpt::hwdrivers::CTuMicos::baseSpeed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("baseSpeedQ", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &)) &mrpt::hwdrivers::CTuMicos::baseSpeedQ, "Query velocity to which start and finish\n	the (de/a)celeration\n\nC++: mrpt::hwdrivers::CTuMicos::baseSpeedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("upperSpeed", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double)) &mrpt::hwdrivers::CTuMicos::upperSpeed, "Specification of velocity upper limit \n\nC++: mrpt::hwdrivers::CTuMicos::upperSpeed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("upperSpeedQ", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &)) &mrpt::hwdrivers::CTuMicos::upperSpeedQ, "Query velocity upper limit \n\nC++: mrpt::hwdrivers::CTuMicos::upperSpeedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("lowerSpeed", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double)) &mrpt::hwdrivers::CTuMicos::lowerSpeed, "Specification of velocity lower limit \n\nC++: mrpt::hwdrivers::CTuMicos::lowerSpeed(char, double) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("lowerSpeedQ", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &)) &mrpt::hwdrivers::CTuMicos::lowerSpeedQ, "Query velocity lower limit \n\nC++: mrpt::hwdrivers::CTuMicos::lowerSpeedQ(char, double &) --> bool", pybind11::arg("axis"), pybind11::arg("radSec"));
		cl.def("reset", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::reset, "Reset PTU to initial state \n\nC++: mrpt::hwdrivers::CTuMicos::reset() --> bool");
		cl.def("save", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::save, "Save or restart default values \n\nC++: mrpt::hwdrivers::CTuMicos::save() --> bool");
		cl.def("restoreDefaults", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::restoreDefaults, "Restore default values \n\nC++: mrpt::hwdrivers::CTuMicos::restoreDefaults() --> bool");
		cl.def("restoreFactoryDefaults", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::restoreFactoryDefaults, "Restore factory default values \n\nC++: mrpt::hwdrivers::CTuMicos::restoreFactoryDefaults() --> bool");
		cl.def("version", (bool (mrpt::hwdrivers::CTuMicos::*)(char *)) &mrpt::hwdrivers::CTuMicos::version, "Version and CopyRights \n\nC++: mrpt::hwdrivers::CTuMicos::version(char *) --> bool", pybind11::arg("nVersion"));
		cl.def("nversion", (void (mrpt::hwdrivers::CTuMicos::*)(double &)) &mrpt::hwdrivers::CTuMicos::nversion, "Number of version \n\nC++: mrpt::hwdrivers::CTuMicos::nversion(double &) --> void", pybind11::arg("nVersion"));
		cl.def("powerModeQ", (bool (mrpt::hwdrivers::CTuMicos::*)(bool, char &)) &mrpt::hwdrivers::CTuMicos::powerModeQ, "Query power mode \n\nC++: mrpt::hwdrivers::CTuMicos::powerModeQ(bool, char &) --> bool", pybind11::arg("transit"), pybind11::arg("mode"));
		cl.def("powerMode", (bool (mrpt::hwdrivers::CTuMicos::*)(bool, char)) &mrpt::hwdrivers::CTuMicos::powerMode, "Specification of power mode \n\nC++: mrpt::hwdrivers::CTuMicos::powerMode(bool, char) --> bool", pybind11::arg("transit"), pybind11::arg("mode"));
		cl.def("clear", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::clear, "Clear controller internal stack \n\nC++: mrpt::hwdrivers::CTuMicos::clear() --> bool");
		cl.def("setLimits", (bool (mrpt::hwdrivers::CTuMicos::*)(char, double &, double &)) &mrpt::hwdrivers::CTuMicos::setLimits, "Set limits of movement \n\nC++: mrpt::hwdrivers::CTuMicos::setLimits(char, double &, double &) --> bool", pybind11::arg("axis"), pybind11::arg("l"), pybind11::arg("u"));
		cl.def("changeMotionDir", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::changeMotionDir, "C++: mrpt::hwdrivers::CTuMicos::changeMotionDir() --> bool");
		cl.def("checkErrors", (int (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::checkErrors, "************************** State Queries *******************\n\nC++: mrpt::hwdrivers::CTuMicos::checkErrors() --> int");
		cl.def("clearErrors", (void (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::clearErrors, "Clear errors *\n\nC++: mrpt::hwdrivers::CTuMicos::clearErrors() --> void");
		cl.def("init", (bool (mrpt::hwdrivers::CTuMicos::*)(const std::string &)) &mrpt::hwdrivers::CTuMicos::init, "PTU and serial port initialization \n\nC++: mrpt::hwdrivers::CTuMicos::init(const std::string &) --> bool", pybind11::arg("port"));
		cl.def("close", (void (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::close, "Close Connection with serial port \n\nC++: mrpt::hwdrivers::CTuMicos::close() --> void");
		cl.def("radError", (double (mrpt::hwdrivers::CTuMicos::*)(char, double)) &mrpt::hwdrivers::CTuMicos::radError, "To obtains the mistake for use discrete values when the movement\n	is expressed in radians. Parameters are the absolute position in\n	radians and the axis desired\n\nC++: mrpt::hwdrivers::CTuMicos::radError(char, double) --> double", pybind11::arg("axis"), pybind11::arg("nRadMoved"));
		cl.def("radToPos", (long (mrpt::hwdrivers::CTuMicos::*)(char, double)) &mrpt::hwdrivers::CTuMicos::radToPos, "To obtain the discrete value for a number of radians \n\nC++: mrpt::hwdrivers::CTuMicos::radToPos(char, double) --> long", pybind11::arg("axis"), pybind11::arg("nRad"));
		cl.def("posToRad", (double (mrpt::hwdrivers::CTuMicos::*)(char, long)) &mrpt::hwdrivers::CTuMicos::posToRad, "To obtain the number of radians for a discrete value \n\nC++: mrpt::hwdrivers::CTuMicos::posToRad(char, long) --> double", pybind11::arg("axis"), pybind11::arg("nPos"));
		cl.def("scan", (bool (mrpt::hwdrivers::CTuMicos::*)(char, int, float, float, double)) &mrpt::hwdrivers::CTuMicos::scan, "Performs a scan in the axis indicated and whit the precision desired. \n		\n\n {Pan or Till} \n		\n\n {Wait time betwen commands} \n		\n\n {initial position}\n		\n\n {final position}\n		\n\n {radians precision for the scan}\n\nC++: mrpt::hwdrivers::CTuMicos::scan(char, int, float, float, double) --> bool", pybind11::arg("axis"), pybind11::arg("wait"), pybind11::arg("initial"), pybind11::arg("final"), pybind11::arg("radPre"));
		cl.def("verboseQ", (bool (mrpt::hwdrivers::CTuMicos::*)(bool &)) &mrpt::hwdrivers::CTuMicos::verboseQ, "Query verbose mode \n\nC++: mrpt::hwdrivers::CTuMicos::verboseQ(bool &) --> bool", pybind11::arg("modo"));
		cl.def("verbose", (bool (mrpt::hwdrivers::CTuMicos::*)(bool)) &mrpt::hwdrivers::CTuMicos::verbose, "Set verbose. \n	\n\n\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CTuMicos::verbose(bool) --> bool", pybind11::arg("set"));
		cl.def("echoModeQ", (bool (mrpt::hwdrivers::CTuMicos::*)(bool &)) &mrpt::hwdrivers::CTuMicos::echoModeQ, "Query echo mode \n\nC++: mrpt::hwdrivers::CTuMicos::echoModeQ(bool &) --> bool", pybind11::arg("mode"));
		cl.def("echoMode", (bool (mrpt::hwdrivers::CTuMicos::*)(bool)) &mrpt::hwdrivers::CTuMicos::echoMode, "Enable/Disable echo response with command. \n	\n\n\n\n\n\n\n	 \n\nC++: mrpt::hwdrivers::CTuMicos::echoMode(bool) --> bool", pybind11::arg("mode"));
		cl.def("resolution", (bool (mrpt::hwdrivers::CTuMicos::*)()) &mrpt::hwdrivers::CTuMicos::resolution, "Query the pan and tilt resolution per position moved\n	and initialize local atributes\n\nC++: mrpt::hwdrivers::CTuMicos::resolution() --> bool");
		cl.def("status", (double (mrpt::hwdrivers::CTuMicos::*)(double &)) &mrpt::hwdrivers::CTuMicos::status, "Check if ptu is moving \n\nC++: mrpt::hwdrivers::CTuMicos::status(double &) --> double", pybind11::arg("rad"));
		cl.def("assign", (class mrpt::hwdrivers::CTuMicos & (mrpt::hwdrivers::CTuMicos::*)(const class mrpt::hwdrivers::CTuMicos &)) &mrpt::hwdrivers::CTuMicos::operator=, "C++: mrpt::hwdrivers::CTuMicos::operator=(const class mrpt::hwdrivers::CTuMicos &) --> class mrpt::hwdrivers::CTuMicos &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
