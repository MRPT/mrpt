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
#include <mrpt/obs/CObservationIMU.h>
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

// mrpt::obs::CObservationIMU file:mrpt/obs/CObservationIMU.h line:110
struct PyCallBack_mrpt_obs_CObservationIMU : public mrpt::obs::CObservationIMU {
	using mrpt::obs::CObservationIMU::CObservationIMU;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationIMU::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationIMU::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationIMU::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationIMU::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationIMU::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationIMU::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationIMU::setSensorPose(a0);
	}
	bool exportTxtSupported() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "exportTxtSupported");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CObservationIMU::exportTxtSupported();
	}
	std::string exportTxtHeader() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "exportTxtHeader");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservationIMU::exportTxtHeader();
	}
	std::string exportTxtDataRow() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "exportTxtDataRow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservationIMU::exportTxtDataRow();
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "getOriginalReceivedTimeStamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "asString");
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
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "unload");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationIMU *>(this), "load_impl");
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

void bind_mrpt_obs_CObservationIMU(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::obs::TIMUDataIndex file:mrpt/obs/CObservationIMU.h line:22
	pybind11::enum_<mrpt::obs::TIMUDataIndex>(M("mrpt::obs"), "TIMUDataIndex", pybind11::arithmetic(), "Symbolic names for the indices of IMU data (refer to\n mrpt::obs::CObservationIMU)\n \n\n\n ")
		.value("IMU_X_ACC", mrpt::obs::IMU_X_ACC)
		.value("IMU_Y_ACC", mrpt::obs::IMU_Y_ACC)
		.value("IMU_Z_ACC", mrpt::obs::IMU_Z_ACC)
		.value("IMU_YAW_VEL", mrpt::obs::IMU_YAW_VEL)
		.value("IMU_WZ", mrpt::obs::IMU_WZ)
		.value("IMU_PITCH_VEL", mrpt::obs::IMU_PITCH_VEL)
		.value("IMU_WY", mrpt::obs::IMU_WY)
		.value("IMU_ROLL_VEL", mrpt::obs::IMU_ROLL_VEL)
		.value("IMU_WX", mrpt::obs::IMU_WX)
		.value("IMU_X_VEL", mrpt::obs::IMU_X_VEL)
		.value("IMU_Y_VEL", mrpt::obs::IMU_Y_VEL)
		.value("IMU_Z_VEL", mrpt::obs::IMU_Z_VEL)
		.value("IMU_YAW", mrpt::obs::IMU_YAW)
		.value("IMU_PITCH", mrpt::obs::IMU_PITCH)
		.value("IMU_ROLL", mrpt::obs::IMU_ROLL)
		.value("IMU_X", mrpt::obs::IMU_X)
		.value("IMU_Y", mrpt::obs::IMU_Y)
		.value("IMU_Z", mrpt::obs::IMU_Z)
		.value("IMU_MAG_X", mrpt::obs::IMU_MAG_X)
		.value("IMU_MAG_Y", mrpt::obs::IMU_MAG_Y)
		.value("IMU_MAG_Z", mrpt::obs::IMU_MAG_Z)
		.value("IMU_PRESSURE", mrpt::obs::IMU_PRESSURE)
		.value("IMU_ALTITUDE", mrpt::obs::IMU_ALTITUDE)
		.value("IMU_TEMPERATURE", mrpt::obs::IMU_TEMPERATURE)
		.value("IMU_ORI_QUAT_X", mrpt::obs::IMU_ORI_QUAT_X)
		.value("IMU_ORI_QUAT_Y", mrpt::obs::IMU_ORI_QUAT_Y)
		.value("IMU_ORI_QUAT_Z", mrpt::obs::IMU_ORI_QUAT_Z)
		.value("IMU_ORI_QUAT_W", mrpt::obs::IMU_ORI_QUAT_W)
		.value("IMU_YAW_VEL_GLOBAL", mrpt::obs::IMU_YAW_VEL_GLOBAL)
		.value("IMU_PITCH_VEL_GLOBAL", mrpt::obs::IMU_PITCH_VEL_GLOBAL)
		.value("IMU_ROLL_VEL_GLOBAL", mrpt::obs::IMU_ROLL_VEL_GLOBAL)
		.value("IMU_X_ACC_GLOBAL", mrpt::obs::IMU_X_ACC_GLOBAL)
		.value("IMU_Y_ACC_GLOBAL", mrpt::obs::IMU_Y_ACC_GLOBAL)
		.value("IMU_Z_ACC_GLOBAL", mrpt::obs::IMU_Z_ACC_GLOBAL)
		.value("COUNT_IMU_DATA_FIELDS", mrpt::obs::COUNT_IMU_DATA_FIELDS)
		.export_values();

;

	{ // mrpt::obs::CObservationIMU file:mrpt/obs/CObservationIMU.h line:110
		pybind11::class_<mrpt::obs::CObservationIMU, std::shared_ptr<mrpt::obs::CObservationIMU>, PyCallBack_mrpt_obs_CObservationIMU, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationIMU", "This class stores measurements from an Inertial Measurement Unit (IMU)\n (attitude estimation, raw gyroscope and accelerometer values), altimeters or\n magnetometers.\n\n  The order of the values in each entry of\n mrpt::obs::CObservationIMU::rawMeasurements is defined as symbolic names in\n the enum mrpt::obs::TIMUDataIndex.\n  Check it out also for reference on the unit and the coordinate frame used\n for each value.\n\n \n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationIMU(); }, [](){ return new PyCallBack_mrpt_obs_CObservationIMU(); } ) );
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservationIMU::sensorPose);
		cl.def_readwrite("dataIsPresent", &mrpt::obs::CObservationIMU::dataIsPresent);
		cl.def_readwrite("rawMeasurements", &mrpt::obs::CObservationIMU::rawMeasurements);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationIMU::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationIMU::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationIMU::*)() const) &mrpt::obs::CObservationIMU::GetRuntimeClass, "C++: mrpt::obs::CObservationIMU::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationIMU::*)() const) &mrpt::obs::CObservationIMU::clone, "C++: mrpt::obs::CObservationIMU::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationIMU::CreateObject, "C++: mrpt::obs::CObservationIMU::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("set", (void (mrpt::obs::CObservationIMU::*)(enum mrpt::obs::TIMUDataIndex, double)) &mrpt::obs::CObservationIMU::set, "Sets a given data type, and mark it as present. \n has(), set() \n\nC++: mrpt::obs::CObservationIMU::set(enum mrpt::obs::TIMUDataIndex, double) --> void", pybind11::arg("idx"), pybind11::arg("value"));
		cl.def("get", (double (mrpt::obs::CObservationIMU::*)(enum mrpt::obs::TIMUDataIndex) const) &mrpt::obs::CObservationIMU::get, "Gets a given data type, throws if not set. \n has(), get() \n\nC++: mrpt::obs::CObservationIMU::get(enum mrpt::obs::TIMUDataIndex) const --> double", pybind11::arg("idx"));
		cl.def("has", (bool (mrpt::obs::CObservationIMU::*)(enum mrpt::obs::TIMUDataIndex) const) &mrpt::obs::CObservationIMU::has, "Returns true if the given data type is set. \n set(), get() \n\nC++: mrpt::obs::CObservationIMU::has(enum mrpt::obs::TIMUDataIndex) const --> bool", pybind11::arg("idx"));
		cl.def("getSensorPose", (void (mrpt::obs::CObservationIMU::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationIMU::getSensorPose, "C++: mrpt::obs::CObservationIMU::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationIMU::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationIMU::setSensorPose, "C++: mrpt::obs::CObservationIMU::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("exportTxtSupported", (bool (mrpt::obs::CObservationIMU::*)() const) &mrpt::obs::CObservationIMU::exportTxtSupported, "C++: mrpt::obs::CObservationIMU::exportTxtSupported() const --> bool");
		cl.def("exportTxtHeader", (std::string (mrpt::obs::CObservationIMU::*)() const) &mrpt::obs::CObservationIMU::exportTxtHeader, "C++: mrpt::obs::CObservationIMU::exportTxtHeader() const --> std::string");
		cl.def("exportTxtDataRow", (std::string (mrpt::obs::CObservationIMU::*)() const) &mrpt::obs::CObservationIMU::exportTxtDataRow, "C++: mrpt::obs::CObservationIMU::exportTxtDataRow() const --> std::string");
		cl.def("assign", (class mrpt::obs::CObservationIMU & (mrpt::obs::CObservationIMU::*)(const class mrpt::obs::CObservationIMU &)) &mrpt::obs::CObservationIMU::operator=, "C++: mrpt::obs::CObservationIMU::operator=(const class mrpt::obs::CObservationIMU &) --> class mrpt::obs::CObservationIMU &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
