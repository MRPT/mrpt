#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/CSerializable.h>
#include <sstream> // __str__
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

// mrpt::math::CMatrixD file:mrpt/math/CMatrixD.h line:23
struct PyCallBack_mrpt_math_CMatrixD : public mrpt::math::CMatrixD {
	using mrpt::math::CMatrixD::CMatrixD;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixD *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CMatrixD::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixD *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CMatrixD::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixD *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CMatrixD::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixD *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMatrixD::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixD *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMatrixD::serializeFrom(a0, a1);
	}
};

void bind_mrpt_math_CMatrixD(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixD file:mrpt/math/CMatrixD.h line:23
		pybind11::class_<mrpt::math::CMatrixD, std::shared_ptr<mrpt::math::CMatrixD>, PyCallBack_mrpt_math_CMatrixD, mrpt::serialization::CSerializable> cl(M("mrpt::math"), "CMatrixD", "This class is a \"CSerializable\" wrapper for\n \"CMatrixDynamic<double>\".\n \n\n For a complete introduction to Matrices and vectors in MRPT, see:\n https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixD(); }, [](){ return new PyCallBack_mrpt_math_CMatrixD(); } ) );
		cl.def( pybind11::init<size_t, size_t>(), pybind11::arg("row"), pybind11::arg("col") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_math_CMatrixD const &o){ return new PyCallBack_mrpt_math_CMatrixD(o); } ) );
		cl.def( pybind11::init( [](mrpt::math::CMatrixD const &o){ return new mrpt::math::CMatrixD(o); } ) );
		cl.def_static("getClassName", (auto (*)()) &mrpt::math::CMatrixD::getClassName, "C++: mrpt::math::CMatrixD::getClassName() --> auto");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::math::CMatrixD::GetRuntimeClassIdStatic, "C++: mrpt::math::CMatrixD::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::math::CMatrixD::*)() const) &mrpt::math::CMatrixD::GetRuntimeClass, "C++: mrpt::math::CMatrixD::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::math::CMatrixD::*)() const) &mrpt::math::CMatrixD::clone, "C++: mrpt::math::CMatrixD::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::math::CMatrixD::CreateObject, "C++: mrpt::math::CMatrixD::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::math::CMatrixD & (mrpt::math::CMatrixD::*)(const class mrpt::math::CMatrixD &)) &mrpt::math::CMatrixD::operator=, "C++: mrpt::math::CMatrixD::operator=(const class mrpt::math::CMatrixD &) --> class mrpt::math::CMatrixD &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
