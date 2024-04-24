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
#include <mrpt/obs/CObservationSkeleton.h>
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

// mrpt::obs::CObservationSkeleton file:mrpt/obs/CObservationSkeleton.h line:25
struct PyCallBack_mrpt_obs_CObservationSkeleton : public mrpt::obs::CObservationSkeleton {
	using mrpt::obs::CObservationSkeleton::CObservationSkeleton;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CObservationSkeleton::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CObservationSkeleton::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CObservationSkeleton::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationSkeleton::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationSkeleton::serializeFrom(a0, a1);
	}
	void getSensorPose(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "getSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationSkeleton::getSensorPose(a0);
	}
	void setSensorPose(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "setSensorPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CObservationSkeleton::setSensorPose(a0);
	}
	using _binder_ret_0 = mrpt::Clock::time_point;
	_binder_ret_0 getOriginalReceivedTimeStamp() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "getOriginalReceivedTimeStamp");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "exportTxtSupported");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "exportTxtHeader");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "exportTxtDataRow");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "unload");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::obs::CObservationSkeleton *>(this), "load_impl");
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

void bind_mrpt_obs_CObservationSkeleton(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::CObservationSkeleton file:mrpt/obs/CObservationSkeleton.h line:25
		pybind11::class_<mrpt::obs::CObservationSkeleton, std::shared_ptr<mrpt::obs::CObservationSkeleton>, PyCallBack_mrpt_obs_CObservationSkeleton, mrpt::obs::CObservation> cl(M("mrpt::obs"), "CObservationSkeleton", "This class stores a skeleton as tracked by OPENNI2 & NITE2 libraries from\n PrimeSense sensors\n\n \n CObservation\n \n\n Class introduced in MRPT 1.3.1\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationSkeleton(); }, [](){ return new PyCallBack_mrpt_obs_CObservationSkeleton(); } ) );
		cl.def_readwrite("sensorPose", &mrpt::obs::CObservationSkeleton::sensorPose);
		cl.def_readwrite("head", &mrpt::obs::CObservationSkeleton::head);
		cl.def_readwrite("neck", &mrpt::obs::CObservationSkeleton::neck);
		cl.def_readwrite("torso", &mrpt::obs::CObservationSkeleton::torso);
		cl.def_readwrite("left_shoulder", &mrpt::obs::CObservationSkeleton::left_shoulder);
		cl.def_readwrite("left_elbow", &mrpt::obs::CObservationSkeleton::left_elbow);
		cl.def_readwrite("left_hand", &mrpt::obs::CObservationSkeleton::left_hand);
		cl.def_readwrite("left_hip", &mrpt::obs::CObservationSkeleton::left_hip);
		cl.def_readwrite("left_knee", &mrpt::obs::CObservationSkeleton::left_knee);
		cl.def_readwrite("left_foot", &mrpt::obs::CObservationSkeleton::left_foot);
		cl.def_readwrite("right_shoulder", &mrpt::obs::CObservationSkeleton::right_shoulder);
		cl.def_readwrite("right_elbow", &mrpt::obs::CObservationSkeleton::right_elbow);
		cl.def_readwrite("right_hand", &mrpt::obs::CObservationSkeleton::right_hand);
		cl.def_readwrite("right_hip", &mrpt::obs::CObservationSkeleton::right_hip);
		cl.def_readwrite("right_knee", &mrpt::obs::CObservationSkeleton::right_knee);
		cl.def_readwrite("right_foot", &mrpt::obs::CObservationSkeleton::right_foot);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::obs::CObservationSkeleton::GetRuntimeClassIdStatic, "C++: mrpt::obs::CObservationSkeleton::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::obs::CObservationSkeleton::*)() const) &mrpt::obs::CObservationSkeleton::GetRuntimeClass, "C++: mrpt::obs::CObservationSkeleton::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::obs::CObservationSkeleton::*)() const) &mrpt::obs::CObservationSkeleton::clone, "C++: mrpt::obs::CObservationSkeleton::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::obs::CObservationSkeleton::CreateObject, "C++: mrpt::obs::CObservationSkeleton::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getSensorPose", (void (mrpt::obs::CObservationSkeleton::*)(class mrpt::poses::CPose3D &) const) &mrpt::obs::CObservationSkeleton::getSensorPose, "C++: mrpt::obs::CObservationSkeleton::getSensorPose(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("out_sensorPose"));
		cl.def("setSensorPose", (void (mrpt::obs::CObservationSkeleton::*)(const class mrpt::poses::CPose3D &)) &mrpt::obs::CObservationSkeleton::setSensorPose, "C++: mrpt::obs::CObservationSkeleton::setSensorPose(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newSensorPose"));
		cl.def("assign", (class mrpt::obs::CObservationSkeleton & (mrpt::obs::CObservationSkeleton::*)(const class mrpt::obs::CObservationSkeleton &)) &mrpt::obs::CObservationSkeleton::operator=, "C++: mrpt::obs::CObservationSkeleton::operator=(const class mrpt::obs::CObservationSkeleton &) --> class mrpt::obs::CObservationSkeleton &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::obs::CObservationSkeleton::TSkeletonJoint file:mrpt/obs/CObservationSkeleton.h line:59
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::obs::CObservationSkeleton::TSkeletonJoint, std::shared_ptr<mrpt::obs::CObservationSkeleton::TSkeletonJoint>> cl(enclosing_class, "TSkeletonJoint", "A generic joint for the skeleton observation ");
			cl.def( pybind11::init( [](){ return new mrpt::obs::CObservationSkeleton::TSkeletonJoint(); } ) );
			cl.def( pybind11::init( [](mrpt::obs::CObservationSkeleton::TSkeletonJoint const &o){ return new mrpt::obs::CObservationSkeleton::TSkeletonJoint(o); } ) );
			cl.def_readwrite("x", &mrpt::obs::CObservationSkeleton::TSkeletonJoint::x);
			cl.def_readwrite("y", &mrpt::obs::CObservationSkeleton::TSkeletonJoint::y);
			cl.def_readwrite("z", &mrpt::obs::CObservationSkeleton::TSkeletonJoint::z);
			cl.def_readwrite("conf", &mrpt::obs::CObservationSkeleton::TSkeletonJoint::conf);
			cl.def("assign", (struct mrpt::obs::CObservationSkeleton::TSkeletonJoint & (mrpt::obs::CObservationSkeleton::TSkeletonJoint::*)(const struct mrpt::obs::CObservationSkeleton::TSkeletonJoint &)) &mrpt::obs::CObservationSkeleton::TSkeletonJoint::operator=, "C++: mrpt::obs::CObservationSkeleton::TSkeletonJoint::operator=(const struct mrpt::obs::CObservationSkeleton::TSkeletonJoint &) --> struct mrpt::obs::CObservationSkeleton::TSkeletonJoint &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
