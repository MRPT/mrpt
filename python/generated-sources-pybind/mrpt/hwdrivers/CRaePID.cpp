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
#include <mrpt/hwdrivers/CRaePID.h>
#include <mrpt/hwdrivers/CRoboPeakLidar.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationGasSensors.h>
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
	using _binder_ret_0 = class std::multimap<struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >, class std::shared_ptr<class mrpt::serialization::CSerializable>, struct std::less<struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > >, class std::allocator<struct std::pair<const struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >, class std::shared_ptr<class mrpt::serialization::CSerializable> > > >;
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
	using _binder_ret_0 = class std::multimap<struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >, class std::shared_ptr<class mrpt::serialization::CSerializable>, struct std::less<struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > >, class std::allocator<struct std::pair<const struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >, class std::shared_ptr<class mrpt::serialization::CSerializable> > > >;
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

void bind_mrpt_hwdrivers_CRaePID(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
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
}
