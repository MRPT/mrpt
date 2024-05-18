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
#include <mrpt/obs/CObservationRFID.h>
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

// mrpt::obs::CObservationRFID file:mrpt/obs/CObservationRFID.h line:24
struct PyCallBack_mrpt_obs_CObservationRFID : public mrpt::obs::CObservationRFID {
	using mrpt::obs::CObservationRFID::CObservationRFID;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationRFID::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationRFID::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationRFID::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRFID::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRFID::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRFID::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationRFID::setSensorPose(a0);
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "getOriginalReceivedTimeStamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "exportTxtDataRow");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "unload");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationRFID *>(this), "load_impl");
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

void bind_mrpt_obs_CObservationRFID(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservationRFID file:mrpt/obs/CObservationRFID.h line:24
		pybind11::class_<mrpt::obs::CObservationRFID, std::shared_ptr<mrpt::obs::CObservationRFID>, PyCallBack_mrpt_obs_CObservationRFID, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationRFID", "This represents one or more RFID tags observed by a receiver.\n\n \n CObservation, mrpt::hwdrivers::CImpinjRFID for a software sensor capable\n of reading this kind of observations.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationRFID(); }, [](){ return new PyCallBack_mrpt_obs_CObservationRFID(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationRFID const &o){ return new PyCallBack_mrpt_obs_CObservationRFID(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationRFID const &o){ return new mrpt::obs::CObservationRFID(o); } ) );
		cl.def_readwrite("sensorPoseOnRobot", &mrpt::obs::CObservationRFID::sensorPoseOnRobot);
		cl.def_readwrite("tag_readings", &mrpt::obs::CObservationRFID::tag_readings);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationRFID::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationRFID::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationRFID::*)() const) &mrpt::obs::CObservationRFID::GetRuntimeClass, "C++: mrpt::obs::CObservationRFID::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationRFID::*)() const) &mrpt::obs::CObservationRFID::clone, "C++: mrpt::obs::CObservationRFID::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationRFID::CreateObject, "C++: mrpt::obs::CObservationRFID::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getNtags", (uint32_t (mrpt::obs::CObservationRFID::*)() const) &mrpt::obs::CObservationRFID::getNtags, "C++: mrpt::obs::CObservationRFID::getNtags() const --> uint32_t");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationRFID::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationRFID::getSensorPose, "@} \n\nC++: mrpt::obs::CObservationRFID::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationRFID::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationRFID::setSensorPose, "A general method to change the sensor pose on the robot.\n  It has no effects in this class\n \n\n getSensorPose  \n\nC++: mrpt::obs::CObservationRFID::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("assign", (class mrpt::obs::CObservationRFID & (mrpt::obs::CObservationRFID::*)(const class mrpt::obs::CObservationRFID &)) &mrpt::obs::CObservationRFID::operator=, "C++: mrpt::obs::CObservationRFID::operator=(const class mrpt::obs::CObservationRFID &) --> class mrpt::obs::CObservationRFID &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CObservationRFID::TTagReading file:mrpt/obs/CObservationRFID.h line:38
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationRFID::TTagReading, std::shared_ptr<mrpt::obs::CObservationRFID::TTagReading>> cl(enclosing_class, "TTagReading", "Each of the individual readings of a RFID tag ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationRFID::TTagReading(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CObservationRFID::TTagReading const &o){ return new mrpt::obs::CObservationRFID::TTagReading(o); } ) );
			cl.def_readwrite("power", &mrpt::obs::CObservationRFID::TTagReading::power);
			cl.def_readwrite("epc", &mrpt::obs::CObservationRFID::TTagReading::epc);
			cl.def_readwrite("antennaPort", &mrpt::obs::CObservationRFID::TTagReading::antennaPort);
			cl.def("assign", (struct mrpt::obs::CObservationRFID::TTagReading & (mrpt::obs::CObservationRFID::TTagReading::*)(const struct mrpt::obs::CObservationRFID::TTagReading &)) &mrpt::obs::CObservationRFID::TTagReading::operator=, "C++: mrpt::obs::CObservationRFID::TTagReading::operator=(const struct mrpt::obs::CObservationRFID::TTagReading &) --> struct mrpt::obs::CObservationRFID::TTagReading &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
