#include <chrono>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <ratio>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <variant>

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

// mrpt::obs::CObservationGasSensors file:mrpt/obs/CObservationGasSensors.h line:24
struct PyCallBack_mrpt_obs_CObservationGasSensors : public mrpt::obs::CObservationGasSensors {
	using mrpt::obs::CObservationGasSensors::CObservationGasSensors;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationGasSensors::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationGasSensors::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationGasSensors::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationGasSensors::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationGasSensors::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationGasSensors::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationGasSensors::setSensorPose(a0);
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "getOriginalReceivedTimeStamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CObservation::getOriginalReceivedTimeStamp();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::asString();
	}
	bool exportTxtSupported() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "exportTxtSupported");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CObservation::exportTxtSupported();
	}
	std::string exportTxtHeader() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "exportTxtHeader");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtHeader();
	}
	std::string exportTxtDataRow() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "exportTxtDataRow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservation::exportTxtDataRow();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "unload");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::unload();
	}
	void load_impl() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGasSensors *>(this), "load_impl");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::load_impl();
	}
};

void bind_mrpt_obs_CObservationGasSensors(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservationGasSensors file:mrpt/obs/CObservationGasSensors.h line:24
		pybind11::class_<mrpt::obs::CObservationGasSensors, std::shared_ptr<mrpt::obs::CObservationGasSensors>, PyCallBack_mrpt_obs_CObservationGasSensors, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationGasSensors", "Declares a class derived from \"CObservation\" that represents a set of\n readings from gas sensors.\n\n \n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationGasSensors(); }, [](){ return new PyCallBack_mrpt_obs_CObservationGasSensors(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationGasSensors const &o){ return new PyCallBack_mrpt_obs_CObservationGasSensors(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationGasSensors const &o){ return new mrpt::obs::CObservationGasSensors(o); } ) );
		cl.def_readwrite("m_readings", &mrpt::obs::CObservationGasSensors::m_readings);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationGasSensors::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationGasSensors::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationGasSensors::*)() const) &mrpt::obs::CObservationGasSensors::GetRuntimeClass, "C++: mrpt::obs::CObservationGasSensors::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationGasSensors::*)() const) &mrpt::obs::CObservationGasSensors::clone, "C++: mrpt::obs::CObservationGasSensors::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationGasSensors::CreateObject, "C++: mrpt::obs::CObservationGasSensors::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationGasSensors::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationGasSensors::getSensorPose, "C++: mrpt::obs::CObservationGasSensors::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationGasSensors::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationGasSensors::setSensorPose, "C++: mrpt::obs::CObservationGasSensors::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("assign", (class mrpt::obs::CObservationGasSensors & (mrpt::obs::CObservationGasSensors::*)(const class mrpt::obs::CObservationGasSensors &)) &mrpt::obs::CObservationGasSensors::operator=, "C++: mrpt::obs::CObservationGasSensors::operator=(const class mrpt::obs::CObservationGasSensors &) --> class mrpt::obs::CObservationGasSensors &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CObservationGasSensors::TObservationENose file:mrpt/obs/CObservationGasSensors.h line:35
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationGasSensors::TObservationENose, std::shared_ptr<mrpt::obs::CObservationGasSensors::TObservationENose>> cl(enclosing_class, "TObservationENose", "The structure for each e-nose");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationGasSensors::TObservationENose(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CObservationGasSensors::TObservationENose const &o){ return new mrpt::obs::CObservationGasSensors::TObservationENose(o); } ) );
			cl.def_readwrite("eNosePoseOnTheRobot", &mrpt::obs::CObservationGasSensors::TObservationENose::eNosePoseOnTheRobot);
			cl.def_readwrite("readingsVoltage", &mrpt::obs::CObservationGasSensors::TObservationENose::readingsVoltage);
			cl.def_readwrite("sensorTypes", &mrpt::obs::CObservationGasSensors::TObservationENose::sensorTypes);
			cl.def_readwrite("hasTemperature", &mrpt::obs::CObservationGasSensors::TObservationENose::hasTemperature);
			cl.def_readwrite("temperature", &mrpt::obs::CObservationGasSensors::TObservationENose::temperature);
			cl.def_readwrite("isActive", &mrpt::obs::CObservationGasSensors::TObservationENose::isActive);
			cl.def("assign", (struct mrpt::obs::CObservationGasSensors::TObservationENose & (mrpt::obs::CObservationGasSensors::TObservationENose::*)(const struct mrpt::obs::CObservationGasSensors::TObservationENose &)) &mrpt::obs::CObservationGasSensors::TObservationENose::operator=, "C++: mrpt::obs::CObservationGasSensors::TObservationENose::operator=(const struct mrpt::obs::CObservationGasSensors::TObservationENose &) --> struct mrpt::obs::CObservationGasSensors::TObservationENose &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::obs::CObservationGasSensors::CMOSmodel file:mrpt/obs/CObservationGasSensors.h line:80
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationGasSensors::CMOSmodel, std::shared_ptr<mrpt::obs::CObservationGasSensors::CMOSmodel>> cl(enclosing_class, "CMOSmodel", "Declares a class within \"CObservationGasSensors\" that represents a set\n of gas concentration readings from the modelation of a MOS gas sensor\n readings.\n This class provides the parameters and functions to simulate the inverse\n model of a MOS gas sensor.\n\n \n CObservationGasSensors");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationGasSensors::CMOSmodel(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CObservationGasSensors::CMOSmodel const &o){ return new mrpt::obs::CObservationGasSensors::CMOSmodel(o); } ) );
			cl.def_readwrite("winNoise_size", &mrpt::obs::CObservationGasSensors::CMOSmodel::winNoise_size);
			cl.def_readwrite("decimate_value", &mrpt::obs::CObservationGasSensors::CMOSmodel::decimate_value);
			cl.def_readwrite("a_rise", &mrpt::obs::CObservationGasSensors::CMOSmodel::a_rise);
			cl.def_readwrite("b_rise", &mrpt::obs::CObservationGasSensors::CMOSmodel::b_rise);
			cl.def_readwrite("a_decay", &mrpt::obs::CObservationGasSensors::CMOSmodel::a_decay);
			cl.def_readwrite("b_decay", &mrpt::obs::CObservationGasSensors::CMOSmodel::b_decay);
			cl.def_readwrite("save_maplog", &mrpt::obs::CObservationGasSensors::CMOSmodel::save_maplog);
			cl.def("get_GasDistribution_estimation", (bool (mrpt::obs::CObservationGasSensors::CMOSmodel::*)(float &, mrpt::Clock::time_point &)) &mrpt::obs::CObservationGasSensors::CMOSmodel::get_GasDistribution_estimation, "Obtain an estimation of the gas distribution based on raw sensor\n readings  \n\nC++: mrpt::obs::CObservationGasSensors::CMOSmodel::get_GasDistribution_estimation(float &, mrpt::Clock::time_point &) --> bool", pybind11::arg("reading"), pybind11::arg("timestamp"));
			cl.def("assign", (class mrpt::obs::CObservationGasSensors::CMOSmodel & (mrpt::obs::CObservationGasSensors::CMOSmodel::*)(const class mrpt::obs::CObservationGasSensors::CMOSmodel &)) &mrpt::obs::CObservationGasSensors::CMOSmodel::operator=, "C++: mrpt::obs::CObservationGasSensors::CMOSmodel::operator=(const class mrpt::obs::CObservationGasSensors::CMOSmodel &) --> class mrpt::obs::CObservationGasSensors::CMOSmodel &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
