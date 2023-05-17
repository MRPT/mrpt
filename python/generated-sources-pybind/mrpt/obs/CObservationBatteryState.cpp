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
#include <mrpt/obs/CObservationBatteryState.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
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
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::obs::CObservationBatteryState file:mrpt/obs/CObservationBatteryState.h line:31
struct PyCallBack_mrpt_obs_CObservationBatteryState : public mrpt::obs::CObservationBatteryState {
	using mrpt::obs::CObservationBatteryState::CObservationBatteryState;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationBatteryState::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationBatteryState::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationBatteryState::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBatteryState::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBatteryState::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBatteryState::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBatteryState::setSensorPose(a0);
	}
	bool exportTxtSupported() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "exportTxtSupported");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CObservationBatteryState::exportTxtSupported();
	}
	std::string exportTxtHeader() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "exportTxtHeader");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservationBatteryState::exportTxtHeader();
	}
	std::string exportTxtDataRow() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "exportTxtDataRow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservationBatteryState::exportTxtDataRow();
	}
	using _binder_ret_0 = struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "getOriginalReceivedTimeStamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "asString");
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
	void load() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "load");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::load();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBatteryState *>(this), "unload");
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
};

// mrpt::obs::CObservationBeaconRanges file:mrpt/obs/CObservationBeaconRanges.h line:24
struct PyCallBack_mrpt_obs_CObservationBeaconRanges : public mrpt::obs::CObservationBeaconRanges {
	using mrpt::obs::CObservationBeaconRanges::CObservationBeaconRanges;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationBeaconRanges::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationBeaconRanges::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationBeaconRanges::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBeaconRanges::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBeaconRanges::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBeaconRanges::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBeaconRanges::setSensorPose(a0);
	}
	bool exportTxtSupported() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "exportTxtSupported");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CObservationBeaconRanges::exportTxtSupported();
	}
	std::string exportTxtHeader() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "exportTxtHeader");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservationBeaconRanges::exportTxtHeader();
	}
	std::string exportTxtDataRow() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "exportTxtDataRow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CObservationBeaconRanges::exportTxtDataRow();
	}
	using _binder_ret_0 = struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "getOriginalReceivedTimeStamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "asString");
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
	void load() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "load");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::load();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBeaconRanges *>(this), "unload");
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
};

// mrpt::obs::CObservationBearingRange file:mrpt/obs/CObservationBearingRange.h line:28
struct PyCallBack_mrpt_obs_CObservationBearingRange : public mrpt::obs::CObservationBearingRange {
	using mrpt::obs::CObservationBearingRange::CObservationBearingRange;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationBearingRange::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationBearingRange::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationBearingRange::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBearingRange::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBearingRange::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBearingRange::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationBearingRange::setSensorPose(a0);
	}
	using _binder_ret_0 = struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "getOriginalReceivedTimeStamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "exportTxtDataRow");
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
	void load() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "load");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservation::load();
	}
	void unload() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationBearingRange *>(this), "unload");
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
};

void bind_mrpt_obs_CObservationBatteryState(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservationBatteryState file:mrpt/obs/CObservationBatteryState.h line:31
		pybind11::class_<mrpt::obs::CObservationBatteryState, std::shared_ptr<mrpt::obs::CObservationBatteryState>, PyCallBack_mrpt_obs_CObservationBatteryState, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationBatteryState", "This represents a measurement of the batteries on the robot.\n  The battery levels are in volts in the form of the public members:\n	- voltageMainRobotBattery\n	- voltageMainRobotComputer\n  - voltageOtherBatteries\n\n  There are boolean flags for signaling when the corresponding values have\nbeen filled out or not.\n\n \n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationBatteryState(); }, [](){ return new PyCallBack_mrpt_obs_CObservationBatteryState(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationBatteryState const &o){ return new PyCallBack_mrpt_obs_CObservationBatteryState(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationBatteryState const &o){ return new mrpt::obs::CObservationBatteryState(o); } ) );
		cl.def_readwrite("voltageMainRobotBattery", &mrpt::obs::CObservationBatteryState::voltageMainRobotBattery);
		cl.def_readwrite("voltageMainRobotComputer", &mrpt::obs::CObservationBatteryState::voltageMainRobotComputer);
		cl.def_readwrite("voltageMainRobotBatteryIsValid", &mrpt::obs::CObservationBatteryState::voltageMainRobotBatteryIsValid);
		cl.def_readwrite("voltageMainRobotComputerIsValid", &mrpt::obs::CObservationBatteryState::voltageMainRobotComputerIsValid);
		cl.def_readwrite("voltageOtherBatteries", &mrpt::obs::CObservationBatteryState::voltageOtherBatteries);
		cl.def_readwrite("voltageOtherBatteriesValid", &mrpt::obs::CObservationBatteryState::voltageOtherBatteriesValid);
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<35> (*)()) &mrpt::obs::CObservationBatteryState::getClassName, "C++: mrpt::obs::CObservationBatteryState::getClassName() --> class mrpt::typemeta::string_literal<35>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationBatteryState::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationBatteryState::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationBatteryState::*)() const) &mrpt::obs::CObservationBatteryState::GetRuntimeClass, "C++: mrpt::obs::CObservationBatteryState::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationBatteryState::*)() const) &mrpt::obs::CObservationBatteryState::clone, "C++: mrpt::obs::CObservationBatteryState::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationBatteryState::CreateObject, "C++: mrpt::obs::CObservationBatteryState::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationBatteryState::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationBatteryState::getSensorPose, "C++: mrpt::obs::CObservationBatteryState::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationBatteryState::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationBatteryState::setSensorPose, "C++: mrpt::obs::CObservationBatteryState::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("exportTxtSupported", (bool (mrpt::obs::CObservationBatteryState::*)() const) &mrpt::obs::CObservationBatteryState::exportTxtSupported, "C++: mrpt::obs::CObservationBatteryState::exportTxtSupported() const --> bool");
		cl.def("exportTxtHeader", (std::string (mrpt::obs::CObservationBatteryState::*)() const) &mrpt::obs::CObservationBatteryState::exportTxtHeader, "C++: mrpt::obs::CObservationBatteryState::exportTxtHeader() const --> std::string");
		cl.def("exportTxtDataRow", (std::string (mrpt::obs::CObservationBatteryState::*)() const) &mrpt::obs::CObservationBatteryState::exportTxtDataRow, "C++: mrpt::obs::CObservationBatteryState::exportTxtDataRow() const --> std::string");
		cl.def("assign", (class mrpt::obs::CObservationBatteryState & (mrpt::obs::CObservationBatteryState::*)(const class mrpt::obs::CObservationBatteryState &)) &mrpt::obs::CObservationBatteryState::operator=, "C++: mrpt::obs::CObservationBatteryState::operator=(const class mrpt::obs::CObservationBatteryState &) --> class mrpt::obs::CObservationBatteryState &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::obs::CObservationBeaconRanges file:mrpt/obs/CObservationBeaconRanges.h line:24
		pybind11::class_<mrpt::obs::CObservationBeaconRanges, std::shared_ptr<mrpt::obs::CObservationBeaconRanges>, PyCallBack_mrpt_obs_CObservationBeaconRanges, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationBeaconRanges", "Declares a class derived from \"CObservation\" that represents one (or more)\n range measurements to labeled beacons.\n \n\n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationBeaconRanges(); }, [](){ return new PyCallBack_mrpt_obs_CObservationBeaconRanges(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationBeaconRanges const &o){ return new PyCallBack_mrpt_obs_CObservationBeaconRanges(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationBeaconRanges const &o){ return new mrpt::obs::CObservationBeaconRanges(o); } ) );
		cl.def_readwrite("minSensorDistance", &mrpt::obs::CObservationBeaconRanges::minSensorDistance);
		cl.def_readwrite("maxSensorDistance", &mrpt::obs::CObservationBeaconRanges::maxSensorDistance);
		cl.def_readwrite("stdError", &mrpt::obs::CObservationBeaconRanges::stdError);
		cl.def_readwrite("sensedData", &mrpt::obs::CObservationBeaconRanges::sensedData);
		cl.def_readwrite("auxEstimatePose", &mrpt::obs::CObservationBeaconRanges::auxEstimatePose);
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<35> (*)()) &mrpt::obs::CObservationBeaconRanges::getClassName, "C++: mrpt::obs::CObservationBeaconRanges::getClassName() --> class mrpt::typemeta::string_literal<35>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationBeaconRanges::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationBeaconRanges::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationBeaconRanges::*)() const) &mrpt::obs::CObservationBeaconRanges::GetRuntimeClass, "C++: mrpt::obs::CObservationBeaconRanges::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationBeaconRanges::*)() const) &mrpt::obs::CObservationBeaconRanges::clone, "C++: mrpt::obs::CObservationBeaconRanges::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationBeaconRanges::CreateObject, "C++: mrpt::obs::CObservationBeaconRanges::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("debugPrintOut", (void (mrpt::obs::CObservationBeaconRanges::*)()) &mrpt::obs::CObservationBeaconRanges::debugPrintOut, "Prints out the contents of the object  \n\nC++: mrpt::obs::CObservationBeaconRanges::debugPrintOut() --> void");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationBeaconRanges::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationBeaconRanges::getSensorPose, "C++: mrpt::obs::CObservationBeaconRanges::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationBeaconRanges::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationBeaconRanges::setSensorPose, "C++: mrpt::obs::CObservationBeaconRanges::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("getSensedRangeByBeaconID", (float (mrpt::obs::CObservationBeaconRanges::*)(int32_t)) &mrpt::obs::CObservationBeaconRanges::getSensedRangeByBeaconID, "Easy look-up into the vector sensedData, returns the range for a given\n beacon, or 0 if the beacon is not observed \n\nC++: mrpt::obs::CObservationBeaconRanges::getSensedRangeByBeaconID(int32_t) --> float", pybind11::arg("beaconID"));
		cl.def("exportTxtSupported", (bool (mrpt::obs::CObservationBeaconRanges::*)() const) &mrpt::obs::CObservationBeaconRanges::exportTxtSupported, "C++: mrpt::obs::CObservationBeaconRanges::exportTxtSupported() const --> bool");
		cl.def("exportTxtHeader", (std::string (mrpt::obs::CObservationBeaconRanges::*)() const) &mrpt::obs::CObservationBeaconRanges::exportTxtHeader, "C++: mrpt::obs::CObservationBeaconRanges::exportTxtHeader() const --> std::string");
		cl.def("exportTxtDataRow", (std::string (mrpt::obs::CObservationBeaconRanges::*)() const) &mrpt::obs::CObservationBeaconRanges::exportTxtDataRow, "C++: mrpt::obs::CObservationBeaconRanges::exportTxtDataRow() const --> std::string");
		cl.def("assign", (class mrpt::obs::CObservationBeaconRanges & (mrpt::obs::CObservationBeaconRanges::*)(const class mrpt::obs::CObservationBeaconRanges &)) &mrpt::obs::CObservationBeaconRanges::operator=, "C++: mrpt::obs::CObservationBeaconRanges::operator=(const class mrpt::obs::CObservationBeaconRanges &) --> class mrpt::obs::CObservationBeaconRanges &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CObservationBeaconRanges::TMeasurement file:mrpt/obs/CObservationBeaconRanges.h line:38
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationBeaconRanges::TMeasurement, std::shared_ptr<mrpt::obs::CObservationBeaconRanges::TMeasurement>> cl(enclosing_class, "TMeasurement", "Each one of the measurements ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationBeaconRanges::TMeasurement(); } ) );
			cl.def_readwrite("sensorLocationOnRobot", &mrpt::obs::CObservationBeaconRanges::TMeasurement::sensorLocationOnRobot);
			cl.def_readwrite("sensedDistance", &mrpt::obs::CObservationBeaconRanges::TMeasurement::sensedDistance);
			cl.def_readwrite("beaconID", &mrpt::obs::CObservationBeaconRanges::TMeasurement::beaconID);
		}

	}
	{ // mrpt::obs::CObservationBearingRange file:mrpt/obs/CObservationBearingRange.h line:28
		pybind11::class_<mrpt::obs::CObservationBearingRange, std::shared_ptr<mrpt::obs::CObservationBearingRange>, PyCallBack_mrpt_obs_CObservationBearingRange, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationBearingRange", "This observation represents a number of range-bearing value pairs, each one\n for a detected landmark, which optionally can have identification IDs.\n  This class can manage sensors that detect landmarks in a 2D plane (e.g. a\n laser scanner) or in the 3D space (e.g. a camera). There are\n  two direction angles: yaw (azimuth) and pitch (negative elevation). For 2D\n sensors, the pitch must be always set to 0.\n See CObservationBearingRange::validCovariances for the instructions to fill\n the uncertainty covariances.\n \n\n CObservation\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationBearingRange(); }, [](){ return new PyCallBack_mrpt_obs_CObservationBearingRange(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_obs_CObservationBearingRange const &o){ return new PyCallBack_mrpt_obs_CObservationBearingRange(o); } ) );
		cl.def( pybind11::init( [](mrpt::obs::CObservationBearingRange const &o){ return new mrpt::obs::CObservationBearingRange(o); } ) );
		cl.def_readwrite("minSensorDistance", &mrpt::obs::CObservationBearingRange::minSensorDistance);
		cl.def_readwrite("maxSensorDistance", &mrpt::obs::CObservationBearingRange::maxSensorDistance);
		cl.def_readwrite("fieldOfView_yaw", &mrpt::obs::CObservationBearingRange::fieldOfView_yaw);
		cl.def_readwrite("fieldOfView_pitch", &mrpt::obs::CObservationBearingRange::fieldOfView_pitch);
		cl.def_readwrite("sensorLocationOnRobot", &mrpt::obs::CObservationBearingRange::sensorLocationOnRobot);
		cl.def_readwrite("sensedData", &mrpt::obs::CObservationBearingRange::sensedData);
		cl.def_readwrite("validCovariances", &mrpt::obs::CObservationBearingRange::validCovariances);
		cl.def_readwrite("sensor_std_range", &mrpt::obs::CObservationBearingRange::sensor_std_range);
		cl.def_readwrite("sensor_std_yaw", &mrpt::obs::CObservationBearingRange::sensor_std_yaw);
		cl.def_readwrite("sensor_std_pitch", &mrpt::obs::CObservationBearingRange::sensor_std_pitch);
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<35> (*)()) &mrpt::obs::CObservationBearingRange::getClassName, "C++: mrpt::obs::CObservationBearingRange::getClassName() --> class mrpt::typemeta::string_literal<35>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationBearingRange::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationBearingRange::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationBearingRange::*)() const) &mrpt::obs::CObservationBearingRange::GetRuntimeClass, "C++: mrpt::obs::CObservationBearingRange::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationBearingRange::*)() const) &mrpt::obs::CObservationBearingRange::clone, "C++: mrpt::obs::CObservationBearingRange::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationBearingRange::CreateObject, "C++: mrpt::obs::CObservationBearingRange::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("debugPrintOut", (void (mrpt::obs::CObservationBearingRange::*)()) &mrpt::obs::CObservationBearingRange::debugPrintOut, "Prints out the contents of the object.\n\nC++: mrpt::obs::CObservationBearingRange::debugPrintOut() --> void");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationBearingRange::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationBearingRange::getSensorPose, "C++: mrpt::obs::CObservationBearingRange::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationBearingRange::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationBearingRange::setSensorPose, "C++: mrpt::obs::CObservationBearingRange::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("assign", (class mrpt::obs::CObservationBearingRange & (mrpt::obs::CObservationBearingRange::*)(const class mrpt::obs::CObservationBearingRange &)) &mrpt::obs::CObservationBearingRange::operator=, "C++: mrpt::obs::CObservationBearingRange::operator=(const class mrpt::obs::CObservationBearingRange &) --> class mrpt::obs::CObservationBearingRange &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CObservationBearingRange::TMeasurement file:mrpt/obs/CObservationBearingRange.h line:51
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationBearingRange::TMeasurement, std::shared_ptr<mrpt::obs::CObservationBearingRange::TMeasurement>> cl(enclosing_class, "TMeasurement", "Each one of the measurements:");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationBearingRange::TMeasurement(); } ) );
			cl.def_readwrite("range", &mrpt::obs::CObservationBearingRange::TMeasurement::range);
			cl.def_readwrite("yaw", &mrpt::obs::CObservationBearingRange::TMeasurement::yaw);
			cl.def_readwrite("pitch", &mrpt::obs::CObservationBearingRange::TMeasurement::pitch);
			cl.def_readwrite("landmarkID", &mrpt::obs::CObservationBearingRange::TMeasurement::landmarkID);
			cl.def_readwrite("covariance", &mrpt::obs::CObservationBearingRange::TMeasurement::covariance);
		}

	}
}
