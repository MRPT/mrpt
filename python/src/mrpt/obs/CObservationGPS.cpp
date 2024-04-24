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
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/gnss_messages_type_list.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/datetime.h>
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

// mrpt::obs::CObservationGPS file:mrpt/obs/CObservationGPS.h line:68
struct PyCallBack_mrpt_obs_CObservationGPS : public mrpt::obs::CObservationGPS {
	using mrpt::obs::CObservationGPS::CObservationGPS;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationGPS::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationGPS::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationGPS::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationGPS::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationGPS::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationGPS::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationGPS::setSensorPose(a0);
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "getOriginalReceivedTimeStamp");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CObservationGPS::getOriginalReceivedTimeStamp();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "exportTxtDataRow");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "unload");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationGPS *>(this), "load_impl");
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

void bind_mrpt_obs_CObservationGPS(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservationGPS file:mrpt/obs/CObservationGPS.h line:68
		pybind11::class_<mrpt::obs::CObservationGPS, std::shared_ptr<mrpt::obs::CObservationGPS>, PyCallBack_mrpt_obs_CObservationGPS, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationGPS", "This class stores messages from GNSS or GNSS+IMU devices, from\n consumer-grade inexpensive GPS receivers to Novatel/Topcon/... advanced RTK\n solutions.\n\n  See mrpt::hwdrivers::CGPSInterface for a class capable of reading from a\n serial port or any input stream and  the ASCII/binary stream into\n  indivual messages  in mrpt::obs::CObservationGPS objects.\n\n  Supported message types are:\n  - NMEA 0183 (ASCII): GGA, RMC, etc.\n  - Topcon GRIL (Binary): PZS, SATS\n  - Novatel GNSS/SPAN OEM6 (Binary): See list of log packets under namespace\n mrpt::obs::gnss and in enum type mrpt::obs::gnss::gnss_message_type_t\n\n  Note that this object has  timestamp fields:\n  - The standard CObservation::timestamp field in the base class, which should\n contain the accurate satellite-based UTC timestamp, and\n  - the field CObservationGPS::originalReceivedTimestamp, with the local\n computer-based timestamp based on the reception of the message in the\n computer.\n\n Normally, users read and write messages by means of these methods:\n  - CObservationGPS::getMsgByClass()\n  - CObservationGPS::setMsg()\n  - CObservationGPS::hasMsgType()\n  - CObservationGPS::hasMsgClass()\n\n Example access to GPS datum:\n \n\n\n\n\n\n\n\n\n\n \n Since MRPT 2.11.12 there is an optional field for ENU covariance for\n easier compatibility with ROS messages.\n\n \n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationGPS const &o){ return new PyCallBack_mrpt_obs_CObservationGPS(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationGPS const &o){ return new mrpt::obs::CObservationGPS(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationGPS(); }, [](){ return new PyCallBack_mrpt_obs_CObservationGPS(); } ) );
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservationGPS::sensorPose);
		cl.def_readwrite("originalReceivedTimestamp", &mrpt::obs::CObservationGPS::originalReceivedTimestamp);
		cl.def_readwrite("has_satellite_timestamp", &mrpt::obs::CObservationGPS::has_satellite_timestamp);
		cl.def_readwrite("messages", &mrpt::obs::CObservationGPS::messages);
		cl.def_readwrite("covariance_enu", &mrpt::obs::CObservationGPS::covariance_enu);
		cl.def_static("Create", (class std::shared_ptr<class mrpt::obs::CObservationGPS> (*)()) &mrpt::obs::CObservationGPS::Create, "C++: mrpt::obs::CObservationGPS::Create() --> class std::shared_ptr<class mrpt::obs::CObservationGPS>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationGPS::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationGPS::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationGPS::*)() const) &mrpt::obs::CObservationGPS::GetRuntimeClass, "C++: mrpt::obs::CObservationGPS::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationGPS::*)() const) &mrpt::obs::CObservationGPS::clone, "C++: mrpt::obs::CObservationGPS::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationGPS::CreateObject, "C++: mrpt::obs::CObservationGPS::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("hasMsgType", (bool (mrpt::obs::CObservationGPS::*)(const enum mrpt::obs::gnss::gnss_message_type_t) const) &mrpt::obs::CObservationGPS::hasMsgType, "Returns true if the list  contains one of\n the requested type. \n\n mrpt::obs::gnss::gnss_message_type_t,\n CObservationGPS::getMsgByType() \n\nC++: mrpt::obs::CObservationGPS::hasMsgType(const enum mrpt::obs::gnss::gnss_message_type_t) const --> bool", pybind11::arg("type_id"));
		cl.def("getMsgByType", (struct mrpt::obs::gnss::gnss_message * (mrpt::obs::CObservationGPS::*)(const enum mrpt::obs::gnss::gnss_message_type_t)) &mrpt::obs::CObservationGPS::getMsgByType, "Returns a pointer to the message in the list CObservationGPS::messages\n of the requested type. Users normally would prefer using\n CObservationGPS::getMsgByClass()\n to avoid having to perform a dynamic_cast<>() on the returned pointer.\n \n\n std::runtime_error If there is no such a message in the list.\n Please, check existence before calling this method with\n CObservationGPS::hasMsgType()\n \n\n mrpt::obs::gnss::gnss_message_type_t,\n CObservationGPS::getMsgByClass(), CObservationGPS::hasMsgType() \n\nC++: mrpt::obs::CObservationGPS::getMsgByType(const enum mrpt::obs::gnss::gnss_message_type_t) --> struct mrpt::obs::gnss::gnss_message *", pybind11::return_value_policy::automatic, pybind11::arg("type_id"));
		cl.def("clear", (void (mrpt::obs::CObservationGPS::*)()) &mrpt::obs::CObservationGPS::clear, "Empties this observation, clearing the container  \n\nC++: mrpt::obs::CObservationGPS::clear() --> void");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationGPS::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationGPS::getSensorPose, "C++: mrpt::obs::CObservationGPS::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationGPS::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationGPS::setSensorPose, "C++: mrpt::obs::CObservationGPS::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("getOriginalReceivedTimeStamp", (mrpt::Clock::time_point (mrpt::obs::CObservationGPS::*)() const) &mrpt::obs::CObservationGPS::getOriginalReceivedTimeStamp, "C++: mrpt::obs::CObservationGPS::getOriginalReceivedTimeStamp() const --> mrpt::Clock::time_point");
		cl.def("has_GGA_datum", (bool (mrpt::obs::CObservationGPS::*)() const) &mrpt::obs::CObservationGPS::has_GGA_datum, "true if the corresponding field exists in  \n\nC++: mrpt::obs::CObservationGPS::has_GGA_datum() const --> bool");
		cl.def("has_RMC_datum", (bool (mrpt::obs::CObservationGPS::*)() const) &mrpt::obs::CObservationGPS::has_RMC_datum, "true if the corresponding field exists in  \n\nC++: mrpt::obs::CObservationGPS::has_RMC_datum() const --> bool");
		cl.def_static("GPS_time_to_UTC", (bool (*)(uint16_t, double, const int, mrpt::Clock::time_point &)) &mrpt::obs::CObservationGPS::GPS_time_to_UTC, "@{ \n\nC++: mrpt::obs::CObservationGPS::GPS_time_to_UTC(uint16_t, double, const int, mrpt::Clock::time_point &) --> bool", pybind11::arg("gps_week"), pybind11::arg("gps_sec"), pybind11::arg("leap_seconds_count"), pybind11::arg("utc_out"));
		cl.def_static("GPS_time_to_UTC", (bool (*)(uint16_t, double, const int, struct mrpt::system::TTimeParts &)) &mrpt::obs::CObservationGPS::GPS_time_to_UTC, "C++: mrpt::obs::CObservationGPS::GPS_time_to_UTC(uint16_t, double, const int, struct mrpt::system::TTimeParts &) --> bool", pybind11::arg("gps_week"), pybind11::arg("gps_sec"), pybind11::arg("leap_seconds_count"), pybind11::arg("utc_out"));
		cl.def("assign", (class mrpt::obs::CObservationGPS & (mrpt::obs::CObservationGPS::*)(const class mrpt::obs::CObservationGPS &)) &mrpt::obs::CObservationGPS::operator=, "C++: mrpt::obs::CObservationGPS::operator=(const class mrpt::obs::CObservationGPS &) --> class mrpt::obs::CObservationGPS &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
