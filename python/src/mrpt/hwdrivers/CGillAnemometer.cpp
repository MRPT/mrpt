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
#include <mrpt/hwdrivers/CGillAnemometer.h>
#include <mrpt/hwdrivers/CGyroKVHDSP3000.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CIMUIntersense.h>
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
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::hwdrivers::CGillAnemometer file:mrpt/hwdrivers/CGillAnemometer.h line:27
struct PyCallBack_mrpt_hwdrivers_CGillAnemometer : public mrpt::hwdrivers::CGillAnemometer {
	using mrpt::hwdrivers::CGillAnemometer::CGillAnemometer;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CGillAnemometer::GetRuntimeClass();
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGillAnemometer::doProcess();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGillAnemometer::loadConfig_sensorSpecific(a0, a1);
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "initialize");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGillAnemometer *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CGyroKVHDSP3000 file:mrpt/hwdrivers/CGyroKVHDSP3000.h line:68
struct PyCallBack_mrpt_hwdrivers_CGyroKVHDSP3000 : public mrpt::hwdrivers::CGyroKVHDSP3000 {
	using mrpt::hwdrivers::CGyroKVHDSP3000::CGyroKVHDSP3000;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CGyroKVHDSP3000::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGyroKVHDSP3000::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGyroKVHDSP3000::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CGyroKVHDSP3000::initialize();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CGyroKVHDSP3000 *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CHokuyoURG file:mrpt/hwdrivers/CHokuyoURG.h line:77
struct PyCallBack_mrpt_hwdrivers_CHokuyoURG : public mrpt::hwdrivers::CHokuyoURG {
	using mrpt::hwdrivers::CHokuyoURG::CHokuyoURG;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CHokuyoURG::GetRuntimeClass();
	}
	void doProcessSimple(bool & a0, class mrpt::obs::CObservation2DRangeScan & a1, bool & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "doProcessSimple");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHokuyoURG::doProcessSimple(a0, a1, a2);
	}
	bool turnOn() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "turnOn");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CHokuyoURG::turnOn();
	}
	bool turnOff() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "turnOff");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CHokuyoURG::turnOff();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CHokuyoURG::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "doProcess");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "initialize");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CHokuyoURG *>(this), "getExternalImageJPEGQuality");
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

// mrpt::hwdrivers::CIMUIntersense file:mrpt/hwdrivers/CIMUIntersense.h line:72
struct PyCallBack_mrpt_hwdrivers_CIMUIntersense : public mrpt::hwdrivers::CIMUIntersense {
	using mrpt::hwdrivers::CIMUIntersense::CIMUIntersense;

	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		return CIMUIntersense::GetRuntimeClass();
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "loadConfig_sensorSpecific");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIMUIntersense::loadConfig_sensorSpecific(a0, a1);
	}
	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "doProcess");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIMUIntersense::doProcess();
	}
	void initialize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CIMUIntersense::initialize();
	}
	void loadConfig(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::CIMUIntersense *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_CGillAnemometer(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::CGillAnemometer file:mrpt/hwdrivers/CGillAnemometer.h line:27
		pybind11::class_<mrpt::hwdrivers::CGillAnemometer, std::shared_ptr<mrpt::hwdrivers::CGillAnemometer>, PyCallBack_mrpt_hwdrivers_CGillAnemometer, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CGillAnemometer", "This class implements a driver for the Gill Windsonic Option 1 Anemometer\n   The sensor is accessed via a standard serial port.\n\n   Refer to the manufacturer website for details on this sensor:\nhttp://gillinstruments.com/data/manuals/WindSonic-Web-Manual.pdf\n	  Configure for single <CR> return, at 2Hz\n  \n\n mrpt::obs::CObservationWindSensor\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CGillAnemometer(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CGillAnemometer(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CGillAnemometer::*)() const) &mrpt::hwdrivers::CGillAnemometer::GetRuntimeClass, "C++: mrpt::hwdrivers::CGillAnemometer::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CGillAnemometer::CreateObject, "C++: mrpt::hwdrivers::CGillAnemometer::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CGillAnemometer::doRegister, "C++: mrpt::hwdrivers::CGillAnemometer::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CGillAnemometer::*)()) &mrpt::hwdrivers::CGillAnemometer::doProcess, "C++: mrpt::hwdrivers::CGillAnemometer::doProcess() --> void");
		cl.def("loadConfig_sensorSpecific", (void (mrpt::hwdrivers::CGillAnemometer::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::hwdrivers::CGillAnemometer::loadConfig_sensorSpecific, "C++: mrpt::hwdrivers::CGillAnemometer::loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("section"));
	}
	// mrpt::hwdrivers::GYRO_MODE file:mrpt/hwdrivers/CGyroKVHDSP3000.h line:19
	pybind11::enum_<mrpt::hwdrivers::GYRO_MODE>(M("mrpt::hwdrivers"), "GYRO_MODE", pybind11::arithmetic(), "")
		.value("RATE", mrpt::hwdrivers::RATE)
		.value("INCREMENTAL_ANGLE", mrpt::hwdrivers::INCREMENTAL_ANGLE)
		.value("INTEGRATED_ANGLE", mrpt::hwdrivers::INTEGRATED_ANGLE)
		.export_values();

;

	{ // mrpt::hwdrivers::CGyroKVHDSP3000 file:mrpt/hwdrivers/CGyroKVHDSP3000.h line:68
		pybind11::class_<mrpt::hwdrivers::CGyroKVHDSP3000, std::shared_ptr<mrpt::hwdrivers::CGyroKVHDSP3000>, PyCallBack_mrpt_hwdrivers_CGyroKVHDSP3000, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CGyroKVHDSP3000", "A class for interfacing KVH DSP 3000 gyroscope with an assynchronous serial\ncommunication (product SN : 02-1222-01).\n  It uses a serial port connection to the device. The class implements the\ngeneric sensor class.\n  See also the application \"rawlog-grabber\" for a ready-to-use application to\ngather data from the scanner.\n	 The generated observation is a CObservationIMU, but only the yaw angular\nvelocity and the absolute yaw position are\n  are set in the vector CObservationIMU::rawMeasurements.\n  The sensor process rate is imposed by hardware at 100Hz.\n  For now, this sensor is only supported on posix system.\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n In most of the communs applications, this class will be used as :\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CGyroKVHDSP3000(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CGyroKVHDSP3000(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CGyroKVHDSP3000::*)() const) &mrpt::hwdrivers::CGyroKVHDSP3000::GetRuntimeClass, "C++: mrpt::hwdrivers::CGyroKVHDSP3000::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CGyroKVHDSP3000::CreateObject, "C++: mrpt::hwdrivers::CGyroKVHDSP3000::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CGyroKVHDSP3000::doRegister, "C++: mrpt::hwdrivers::CGyroKVHDSP3000::doRegister() --> void");
		cl.def("loadConfig_sensorSpecific", (void (mrpt::hwdrivers::CGyroKVHDSP3000::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::hwdrivers::CGyroKVHDSP3000::loadConfig_sensorSpecific, "See the class documentation at the top for expected parameters \n\nC++: mrpt::hwdrivers::CGyroKVHDSP3000::loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("configSource"), pybind11::arg("iniSection"));
		cl.def("doProcess", (void (mrpt::hwdrivers::CGyroKVHDSP3000::*)()) &mrpt::hwdrivers::CGyroKVHDSP3000::doProcess, "This method will be invoked at a minimum rate of \"process_rate\" (Hz)\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CGyroKVHDSP3000::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CGyroKVHDSP3000::*)()) &mrpt::hwdrivers::CGyroKVHDSP3000::initialize, "Turns on the KVH DSP 3000 device and configure it for getting\n orientation data. you must have called loadConfig_sensorSpecific before\n calling this function.\n\nC++: mrpt::hwdrivers::CGyroKVHDSP3000::initialize() --> void");
		cl.def("resetIncrementalAngle", (void (mrpt::hwdrivers::CGyroKVHDSP3000::*)()) &mrpt::hwdrivers::CGyroKVHDSP3000::resetIncrementalAngle, "Send to the sensor the command 'Z' wich reset the integrated angle. (in\n both rate mode and incremental, this function has no effect) \n\nC++: mrpt::hwdrivers::CGyroKVHDSP3000::resetIncrementalAngle() --> void");
		cl.def("changeMode", (void (mrpt::hwdrivers::CGyroKVHDSP3000::*)(enum mrpt::hwdrivers::GYRO_MODE)) &mrpt::hwdrivers::CGyroKVHDSP3000::changeMode, "C++: mrpt::hwdrivers::CGyroKVHDSP3000::changeMode(enum mrpt::hwdrivers::GYRO_MODE) --> void", pybind11::arg("_newMode"));
	}
	{ // mrpt::hwdrivers::CHokuyoURG file:mrpt/hwdrivers/CHokuyoURG.h line:77
		pybind11::class_<mrpt::hwdrivers::CHokuyoURG, std::shared_ptr<mrpt::hwdrivers::CHokuyoURG>, PyCallBack_mrpt_hwdrivers_CHokuyoURG, mrpt::hwdrivers::C2DRangeFinderAbstract> cl(M("mrpt::hwdrivers"), "CHokuyoURG", "This software driver implements the protocol SCIP-2.0 for interfacing HOKUYO\n URG/UTM/UXM/UST laser scanners (USB or Ethernet).\n  Refer to the example code\n [HOKUYO_laser_test](http://www.mrpt.org/tutorials/mrpt-examples/example_hokuyo_urgutm_laser_scanner/)\n  and to example rawlog-grabber [config\n files](https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files/rawlog-grabber)\n\n  See also the application \"rawlog-grabber\" for a ready-to-use application to\n gather data from the scanner.\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CHokuyoURG(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CHokuyoURG(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CHokuyoURG::*)() const) &mrpt::hwdrivers::CHokuyoURG::GetRuntimeClass, "C++: mrpt::hwdrivers::CHokuyoURG::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CHokuyoURG::CreateObject, "C++: mrpt::hwdrivers::CHokuyoURG::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CHokuyoURG::doRegister, "C++: mrpt::hwdrivers::CHokuyoURG::doRegister() --> void");
		cl.def("doProcessSimple", (void (mrpt::hwdrivers::CHokuyoURG::*)(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &)) &mrpt::hwdrivers::CHokuyoURG::doProcessSimple, "Specific laser scanner \"software drivers\" must process here new data\n from the I/O stream, and, if a whole scan has arrived, return it.\n  This method will be typically called in a different thread than other\n methods, and will be called in a timely fashion.\n\nC++: mrpt::hwdrivers::CHokuyoURG::doProcessSimple(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("turnOn", (bool (mrpt::hwdrivers::CHokuyoURG::*)()) &mrpt::hwdrivers::CHokuyoURG::turnOn, "Enables the scanning mode (which may depend on the specific laser\n device); this must be called before asking for observations to assure\n that the protocol has been initializated.\n \n\n If everything works \"true\", or \"false\" if there is any error.\n\nC++: mrpt::hwdrivers::CHokuyoURG::turnOn() --> bool");
		cl.def("turnOff", (bool (mrpt::hwdrivers::CHokuyoURG::*)()) &mrpt::hwdrivers::CHokuyoURG::turnOff, "Disables the scanning mode (this can be used to turn the device in low\n energy mode, if available)\n \n\n If everything works \"true\", or \"false\" if there is any error.\n\nC++: mrpt::hwdrivers::CHokuyoURG::turnOff() --> bool");
		cl.def("purgeBuffers", (void (mrpt::hwdrivers::CHokuyoURG::*)()) &mrpt::hwdrivers::CHokuyoURG::purgeBuffers, "Empties the RX buffers of the serial port \n\nC++: mrpt::hwdrivers::CHokuyoURG::purgeBuffers() --> void");
		cl.def("setSerialPort", (void (mrpt::hwdrivers::CHokuyoURG::*)(const std::string &)) &mrpt::hwdrivers::CHokuyoURG::setSerialPort, "If set to non-empty, the serial port will be attempted to be opened\n automatically when this class is first used to request data from the\n laser.  \n\nC++: mrpt::hwdrivers::CHokuyoURG::setSerialPort(const std::string &) --> void", pybind11::arg("port_name"));
		cl.def("setIPandPort", (void (mrpt::hwdrivers::CHokuyoURG::*)(const std::string &, const unsigned int &)) &mrpt::hwdrivers::CHokuyoURG::setIPandPort, "Set the ip direction and port to connect using Ethernet communication \n\nC++: mrpt::hwdrivers::CHokuyoURG::setIPandPort(const std::string &, const unsigned int &) --> void", pybind11::arg("ip"), pybind11::arg("port"));
		cl.def("getSerialPort", (const std::string (mrpt::hwdrivers::CHokuyoURG::*)()) &mrpt::hwdrivers::CHokuyoURG::getSerialPort, "Returns the currently set serial port \n setSerialPort \n\nC++: mrpt::hwdrivers::CHokuyoURG::getSerialPort() --> const std::string");
		cl.def("setReducedFOV", (void (mrpt::hwdrivers::CHokuyoURG::*)(const double)) &mrpt::hwdrivers::CHokuyoURG::setReducedFOV, "If called (before calling \"turnOn\"), the field of view of the laser is\n reduced to the given range (in radians), discarding the rest of measures.\n  Call with \"0\" to disable this reduction again (the default).\n\nC++: mrpt::hwdrivers::CHokuyoURG::setReducedFOV(const double) --> void", pybind11::arg("fov"));
		cl.def("setHighSensitivityMode", (bool (mrpt::hwdrivers::CHokuyoURG::*)(bool)) &mrpt::hwdrivers::CHokuyoURG::setHighSensitivityMode, "Changes the high sensitivity mode (HS) (default: false)\n \n\n false on any error\n\nC++: mrpt::hwdrivers::CHokuyoURG::setHighSensitivityMode(bool) --> bool", pybind11::arg("enabled"));
		cl.def("setIntensityMode", (bool (mrpt::hwdrivers::CHokuyoURG::*)(bool)) &mrpt::hwdrivers::CHokuyoURG::setIntensityMode, "If true scans will capture intensity. (default: false)\n Should not be called while scanning.\n \n\n false on any error\n\nC++: mrpt::hwdrivers::CHokuyoURG::setIntensityMode(bool) --> bool", pybind11::arg("enabled"));
		cl.def("setScanInterval", (void (mrpt::hwdrivers::CHokuyoURG::*)(unsigned int)) &mrpt::hwdrivers::CHokuyoURG::setScanInterval, "Set the skip scan count (0 means send all scans).\n Must be set before initialize()\n\nC++: mrpt::hwdrivers::CHokuyoURG::setScanInterval(unsigned int) --> void", pybind11::arg("skipScanCount"));
		cl.def("getScanInterval", (unsigned int (mrpt::hwdrivers::CHokuyoURG::*)() const) &mrpt::hwdrivers::CHokuyoURG::getScanInterval, "C++: mrpt::hwdrivers::CHokuyoURG::getScanInterval() const --> unsigned int");
		cl.def("sendCmd", (void (mrpt::hwdrivers::CHokuyoURG::*)(const char *)) &mrpt::hwdrivers::CHokuyoURG::sendCmd, "C++: mrpt::hwdrivers::CHokuyoURG::sendCmd(const char *) --> void", pybind11::arg("str"));

		{ // mrpt::hwdrivers::CHokuyoURG::TSensorInfo file:mrpt/hwdrivers/CHokuyoURG.h line:82
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::hwdrivers::CHokuyoURG::TSensorInfo, std::shared_ptr<mrpt::hwdrivers::CHokuyoURG::TSensorInfo>> cl(enclosing_class, "TSensorInfo", "Used in CHokuyoURG::displayVersionInfo ");
			cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CHokuyoURG::TSensorInfo(); } ) );
			cl.def( pybind11::init( [](mrpt::hwdrivers::CHokuyoURG::TSensorInfo const &o){ return new mrpt::hwdrivers::CHokuyoURG::TSensorInfo(o); } ) );
			cl.def_readwrite("model", &mrpt::hwdrivers::CHokuyoURG::TSensorInfo::model);
			cl.def_readwrite("d_min", &mrpt::hwdrivers::CHokuyoURG::TSensorInfo::d_min);
			cl.def_readwrite("d_max", &mrpt::hwdrivers::CHokuyoURG::TSensorInfo::d_max);
			cl.def_readwrite("scans_per_360deg", &mrpt::hwdrivers::CHokuyoURG::TSensorInfo::scans_per_360deg);
			cl.def_readwrite("scan_first", &mrpt::hwdrivers::CHokuyoURG::TSensorInfo::scan_first);
			cl.def_readwrite("scan_last", &mrpt::hwdrivers::CHokuyoURG::TSensorInfo::scan_last);
			cl.def_readwrite("scan_front", &mrpt::hwdrivers::CHokuyoURG::TSensorInfo::scan_front);
			cl.def_readwrite("motor_speed_rpm", &mrpt::hwdrivers::CHokuyoURG::TSensorInfo::motor_speed_rpm);
			cl.def("assign", (struct mrpt::hwdrivers::CHokuyoURG::TSensorInfo & (mrpt::hwdrivers::CHokuyoURG::TSensorInfo::*)(const struct mrpt::hwdrivers::CHokuyoURG::TSensorInfo &)) &mrpt::hwdrivers::CHokuyoURG::TSensorInfo::operator=, "C++: mrpt::hwdrivers::CHokuyoURG::TSensorInfo::operator=(const struct mrpt::hwdrivers::CHokuyoURG::TSensorInfo &) --> struct mrpt::hwdrivers::CHokuyoURG::TSensorInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::hwdrivers::CIMUIntersense file:mrpt/hwdrivers/CIMUIntersense.h line:72
		pybind11::class_<mrpt::hwdrivers::CIMUIntersense, std::shared_ptr<mrpt::hwdrivers::CIMUIntersense>, PyCallBack_mrpt_hwdrivers_CIMUIntersense, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "CIMUIntersense", "A class for interfacing Intersense Inertial Measuring Units (IMUs).\n  It connects to a InterSense inertiaCube 3 sensor and records inertial data.\n  NOTE: This device provides:\n		- Euler angles,\n		- 2 angular velocties (body-frame and navigation-frame)\n		- X,Y,Z velocity\n		- 2 accelerations (body-frame and navigation-frame)\n\n		In order to record all this information within the 'rawMeasurements'\nvector\nin mrpt::obs::CObservationIMU, some of it had to be stored in positions which\nweren't intended for the stored data (marked with *):\n		- Euler angles --> rawMeasurements[IMU_YAW], rawMeasurements[IMU_PITCH],\nrawMeasurements[IMU_ROLL]\n		- Body-frame angular velocity --> rawMeasurements[IMU_YAW_VEL],\nrawMeasurements[IMU_PITCH_VEL], rawMeasurements[IMU_ROLL_VEL]\n		- * Nav-frame angular velocity --> rawMeasurements[IMU_MAG_X],\nrawMeasurements[IMU_MAG_Y], rawMeasurements[IMU_MAG_Z]\n		- XYZ velocity --> rawMeasurements[IMU_X_VEL],\nrawMeasurements[IMU_Y_VEL],\nrawMeasurements[IMU_Z_VEL]\n		- Body-frame acceleration --> rawMeasurements[IMU_X_ACC],\nrawMeasurements[IMU_Y_ACC], rawMeasurements[IMU_Z_ACC]\n		- * Nav-frame acceleration --> rawMeasurements[IMU_X],\nrawMeasurements[IMU_Y], rawMeasurements[IMU_Z]\n		Be careful with this when using the grabbed mrpt::obs::CObservationIMU\ndata.\n\n  See also the application \"rawlog-grabber\" for a ready-to-use application to\ngather data from this sensor.\n\n  \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n \n Class introduced in MRPT 1.3.1\n \n\n\n  ");
		cl.def( pybind11::init( [](){ return new mrpt::hwdrivers::CIMUIntersense(); }, [](){ return new PyCallBack_mrpt_hwdrivers_CIMUIntersense(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::hwdrivers::TSensorClassId * (mrpt::hwdrivers::CIMUIntersense::*)() const) &mrpt::hwdrivers::CIMUIntersense::GetRuntimeClass, "C++: mrpt::hwdrivers::CIMUIntersense::GetRuntimeClass() const --> const struct mrpt::hwdrivers::TSensorClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class mrpt::hwdrivers::CGenericSensor * (*)()) &mrpt::hwdrivers::CIMUIntersense::CreateObject, "C++: mrpt::hwdrivers::CIMUIntersense::CreateObject() --> class mrpt::hwdrivers::CGenericSensor *", pybind11::return_value_policy::automatic);
		cl.def_static("doRegister", (void (*)()) &mrpt::hwdrivers::CIMUIntersense::doRegister, "C++: mrpt::hwdrivers::CIMUIntersense::doRegister() --> void");
		cl.def("doProcess", (void (mrpt::hwdrivers::CIMUIntersense::*)()) &mrpt::hwdrivers::CIMUIntersense::doProcess, "This method will be invoked at a minimum rate of \"process_rate\" (Hz)\n  \n\n This method must throw an exception with a descriptive\n message if some critical error is found.\n\nC++: mrpt::hwdrivers::CIMUIntersense::doProcess() --> void");
		cl.def("initialize", (void (mrpt::hwdrivers::CIMUIntersense::*)()) &mrpt::hwdrivers::CIMUIntersense::initialize, "Turns on the iSense device and configure it for getting orientation data\n\nC++: mrpt::hwdrivers::CIMUIntersense::initialize() --> void");
	}
}
