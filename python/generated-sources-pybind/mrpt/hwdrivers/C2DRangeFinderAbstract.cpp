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
#include <mrpt/io/CStream.h>
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

// mrpt::hwdrivers::C2DRangeFinderAbstract file:mrpt/hwdrivers/C2DRangeFinderAbstract.h line:40
struct PyCallBack_mrpt_hwdrivers_C2DRangeFinderAbstract : public mrpt::hwdrivers::C2DRangeFinderAbstract {
	using mrpt::hwdrivers::C2DRangeFinderAbstract::C2DRangeFinderAbstract;

	void doProcess() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "doProcess");
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
	void doProcessSimple(bool & a0, class mrpt::obs::CObservation2DRangeScan & a1, bool & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "doProcessSimple");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"C2DRangeFinderAbstract::doProcessSimple\"");
	}
	bool turnOn() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "turnOn");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"C2DRangeFinderAbstract::turnOn\"");
	}
	bool turnOff() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "turnOff");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"C2DRangeFinderAbstract::turnOff\"");
	}
	const struct mrpt::hwdrivers::TSensorClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::hwdrivers::TSensorClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::hwdrivers::TSensorClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::hwdrivers::TSensorClassId *>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CGenericSensor::GetRuntimeClass\"");
	}
	void loadConfig_sensorSpecific(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "loadConfig_sensorSpecific");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "loadConfig");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "initialize");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "getObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "setPathForExternalImages");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "setExternalImageFormat");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "setExternalImageJPEGQuality");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::hwdrivers::C2DRangeFinderAbstract *>(this), "getExternalImageJPEGQuality");
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

void bind_mrpt_hwdrivers_C2DRangeFinderAbstract(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::hwdrivers::C2DRangeFinderAbstract file:mrpt/hwdrivers/C2DRangeFinderAbstract.h line:40
		pybind11::class_<mrpt::hwdrivers::C2DRangeFinderAbstract, std::shared_ptr<mrpt::hwdrivers::C2DRangeFinderAbstract>, PyCallBack_mrpt_hwdrivers_C2DRangeFinderAbstract, mrpt::hwdrivers::CGenericSensor> cl(M("mrpt::hwdrivers"), "C2DRangeFinderAbstract", "This is the base, abstract class for \"software drivers\" interfaces to 2D\n scanners (laser range finders).\n  Physical devices may be interfaced through a serial port, a USB\n connection,etc. but this class\n   abstract those details throught the \"binding\" of the specific scanner\n driver to a given I/O channel,\n   which must be set by calling \"hwdrivers::C2DRangeFinderAbstract::bindIO\".\n See also the derived classes.\n\n  There is support for \"exclusion polygons\", areas where points, if detected,\n should be marked as invalid.\n  Those areas are useful in cases where the scanner always detects part of\n the vehicle itself, and those\n   points want to be ignored (see\n C2DRangeFinderAbstract::loadExclusionAreas).\n\n \n comms::CSerialPort\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_hwdrivers_C2DRangeFinderAbstract(); } ) );
		cl.def("showPreview", [](mrpt::hwdrivers::C2DRangeFinderAbstract &o) -> void { return o.showPreview(); }, "");
		cl.def("showPreview", (void (mrpt::hwdrivers::C2DRangeFinderAbstract::*)(bool)) &mrpt::hwdrivers::C2DRangeFinderAbstract::showPreview, "Enables GUI visualization in real-time \n\nC++: mrpt::hwdrivers::C2DRangeFinderAbstract::showPreview(bool) --> void", pybind11::arg("enable"));
		cl.def("bindIO", (void (mrpt::hwdrivers::C2DRangeFinderAbstract::*)(const class std::shared_ptr<class mrpt::io::CStream> &)) &mrpt::hwdrivers::C2DRangeFinderAbstract::bindIO, "Binds the object to a given I/O channel.\n  The stream object must not be deleted before the destruction of this\n class.\n \n\n comms::CSerialPort\n\nC++: mrpt::hwdrivers::C2DRangeFinderAbstract::bindIO(const class std::shared_ptr<class mrpt::io::CStream> &) --> void", pybind11::arg("streamIO"));
		cl.def("getObservation", (void (mrpt::hwdrivers::C2DRangeFinderAbstract::*)(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &)) &mrpt::hwdrivers::C2DRangeFinderAbstract::getObservation, "Get the last observation from the sensor, if available, and unmarks it\n as being \"the last one\" (thus a new scan must arrive or subsequent calls\n will find no new observations).\n\nC++: mrpt::hwdrivers::C2DRangeFinderAbstract::getObservation(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("doProcess", (void (mrpt::hwdrivers::C2DRangeFinderAbstract::*)()) &mrpt::hwdrivers::C2DRangeFinderAbstract::doProcess, "Main method for a CGenericSensor \n\nC++: mrpt::hwdrivers::C2DRangeFinderAbstract::doProcess() --> void");
		cl.def("doProcessSimple", (void (mrpt::hwdrivers::C2DRangeFinderAbstract::*)(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &)) &mrpt::hwdrivers::C2DRangeFinderAbstract::doProcessSimple, "Specific laser scanner \"software drivers\" must process here new data\n from the I/O stream, and, if a whole scan has arrived, return it.\n  This method MUST BE CALLED in a timely fashion by the user to allow the\n proccessing of incoming data. It can be run in a different thread safely.\n\nC++: mrpt::hwdrivers::C2DRangeFinderAbstract::doProcessSimple(bool &, class mrpt::obs::CObservation2DRangeScan &, bool &) --> void", pybind11::arg("outThereIsObservation"), pybind11::arg("outObservation"), pybind11::arg("hardwareError"));
		cl.def("turnOn", (bool (mrpt::hwdrivers::C2DRangeFinderAbstract::*)()) &mrpt::hwdrivers::C2DRangeFinderAbstract::turnOn, "Enables the scanning mode (which may depend on the specific laser\n device); this must be called before asking for observations to assure\n that the protocol has been initializated.\n \n\n If everything works \"true\", or \"false\" if there is any error.\n\nC++: mrpt::hwdrivers::C2DRangeFinderAbstract::turnOn() --> bool");
		cl.def("turnOff", (bool (mrpt::hwdrivers::C2DRangeFinderAbstract::*)()) &mrpt::hwdrivers::C2DRangeFinderAbstract::turnOff, "Disables the scanning mode (this can be used to turn the device in low\n energy mode, if available)\n \n\n If everything works \"true\", or \"false\" if there is any error.\n\nC++: mrpt::hwdrivers::C2DRangeFinderAbstract::turnOff() --> bool");
		cl.def("getEstimatedScanPeriod", (double (mrpt::hwdrivers::C2DRangeFinderAbstract::*)() const) &mrpt::hwdrivers::C2DRangeFinderAbstract::getEstimatedScanPeriod, "Returns the empirical, filtered estimation for the period at which whole\n scans are being returned from calls to doProcessSimple()\n \n\n:  Units: seconds \n\nC++: mrpt::hwdrivers::C2DRangeFinderAbstract::getEstimatedScanPeriod() const --> double");
	}
}
