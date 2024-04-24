#include <iterator>
#include <memory>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <sstream> // __str__
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

// mrpt::poses::CPose2DInterpolator file:mrpt/poses/CPose2DInterpolator.h line:48
struct PyCallBack_mrpt_poses_CPose2DInterpolator : public mrpt::poses::CPose2DInterpolator {
	using mrpt::poses::CPose2DInterpolator::CPose2DInterpolator;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2DInterpolator *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose2DInterpolator::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2DInterpolator *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose2DInterpolator::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2DInterpolator *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose2DInterpolator::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2DInterpolator *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose2DInterpolator::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2DInterpolator *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose2DInterpolator::serializeFrom(a0, a1);
	}
};

// mrpt::poses::CPose3DInterpolator file:mrpt/poses/CPose3DInterpolator.h line:47
struct PyCallBack_mrpt_poses_CPose3DInterpolator : public mrpt::poses::CPose3DInterpolator {
	using mrpt::poses::CPose3DInterpolator::CPose3DInterpolator;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DInterpolator *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DInterpolator::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DInterpolator *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3DInterpolator::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DInterpolator *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3DInterpolator::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DInterpolator *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DInterpolator::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DInterpolator *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DInterpolator::serializeFrom(a0, a1);
	}
};

void bind_mrpt_poses_CPose2DInterpolator(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose2DInterpolator file:mrpt/poses/CPose2DInterpolator.h line:48
		pybind11::class_<mrpt::poses::CPose2DInterpolator, std::shared_ptr<mrpt::poses::CPose2DInterpolator>, PyCallBack_mrpt_poses_CPose2DInterpolator, mrpt::serialization::CSerializable, mrpt::poses::CPoseInterpolatorBase<2>> cl(M("mrpt::poses"), "CPose2DInterpolator", "This class stores a time-stamped trajectory in SE(2) (mrpt::math::TPose2D\n poses).\n  It can also interpolate SE(2) poses over time using linear, splines or\n SLERP interpolation, as set in CPose2DInterpolator::setInterpolationMethod()\n  Usage:\n   - Insert new poses into the sequence with CPose2DInterpolator::insert()\n   - Query an exact/interpolated pose with\n CPose2DInterpolator::interpolate().\n Example:\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n  Time is represented with mrpt::Clock::time_point.\n  See mrpt::system for methods and utilities to manage these time references.\n\n  See TInterpolatorMethod for the list of interpolation methods. The default\n method at constructor is \"imLinearSlerp\".\n\n \n CPoseOrPoint\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose2DInterpolator(); }, [](){ return new PyCallBack_mrpt_poses_CPose2DInterpolator(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose2DInterpolator const &o){ return new PyCallBack_mrpt_poses_CPose2DInterpolator(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose2DInterpolator const &o){ return new mrpt::poses::CPose2DInterpolator(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose2DInterpolator::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose2DInterpolator::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose2DInterpolator::*)() const) &mrpt::poses::CPose2DInterpolator::GetRuntimeClass, "C++: mrpt::poses::CPose2DInterpolator::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose2DInterpolator::*)() const) &mrpt::poses::CPose2DInterpolator::clone, "C++: mrpt::poses::CPose2DInterpolator::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose2DInterpolator::CreateObject, "C++: mrpt::poses::CPose2DInterpolator::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::poses::CPose2DInterpolator & (mrpt::poses::CPose2DInterpolator::*)(const class mrpt::poses::CPose2DInterpolator &)) &mrpt::poses::CPose2DInterpolator::operator=, "C++: mrpt::poses::CPose2DInterpolator::operator=(const class mrpt::poses::CPose2DInterpolator &) --> class mrpt::poses::CPose2DInterpolator &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPose3DInterpolator file:mrpt/poses/CPose3DInterpolator.h line:47
		pybind11::class_<mrpt::poses::CPose3DInterpolator, std::shared_ptr<mrpt::poses::CPose3DInterpolator>, PyCallBack_mrpt_poses_CPose3DInterpolator, mrpt::serialization::CSerializable, mrpt::poses::CPoseInterpolatorBase<3>> cl(M("mrpt::poses"), "CPose3DInterpolator", "This class stores a time-stamped trajectory in SE(3) (CPose3D poses).\n  It can also interpolate SE(3) poses over time using linear, splines or\n SLERP interpolation, as set in CPose3DInterpolator::setInterpolationMethod()\n  Usage:\n   - Insert new poses into the sequence with CPose3DInterpolator::insert()\n   - Query an exact/interpolated pose with\n CPose3DInterpolator::interpolate().\n Example:\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n  Time is represented with mrpt::Clock::time_point. See mrpt::system for\n methods and utilities to manage these time references.\n\n  See TInterpolatorMethod for the list of interpolation methods. The default\n method at constructor is \"imLinearSlerp\".\n\n \n CPoseOrPoint\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DInterpolator(); }, [](){ return new PyCallBack_mrpt_poses_CPose3DInterpolator(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3DInterpolator const &o){ return new PyCallBack_mrpt_poses_CPose3DInterpolator(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3DInterpolator const &o){ return new mrpt::poses::CPose3DInterpolator(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DInterpolator::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DInterpolator::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DInterpolator::*)() const) &mrpt::poses::CPose3DInterpolator::GetRuntimeClass, "C++: mrpt::poses::CPose3DInterpolator::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3DInterpolator::*)() const) &mrpt::poses::CPose3DInterpolator::clone, "C++: mrpt::poses::CPose3DInterpolator::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3DInterpolator::CreateObject, "C++: mrpt::poses::CPose3DInterpolator::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::poses::CPose3DInterpolator & (mrpt::poses::CPose3DInterpolator::*)(const class mrpt::poses::CPose3DInterpolator &)) &mrpt::poses::CPose3DInterpolator::operator=, "C++: mrpt::poses::CPose3DInterpolator::operator=(const class mrpt::poses::CPose3DInterpolator &) --> class mrpt::poses::CPose3DInterpolator &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
