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
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

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
	using _binder_ret_0 = class std::multimap<mrpt::Clock::time_point, class std::shared_ptr<class mrpt::serialization::CSerializable> >;
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

void bind_mrpt_hwdrivers_CSickLaserUSB(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
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
