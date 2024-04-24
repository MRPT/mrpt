#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixF.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/matrix_size_t.h>
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

// mrpt::math::CMatrixF file:mrpt/math/CMatrixF.h line:22
struct PyCallBack_mrpt_math_CMatrixF : public mrpt::math::CMatrixF {
	using mrpt::math::CMatrixF::CMatrixF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CMatrixF::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixF *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CMatrixF::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixF *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CMatrixF::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixF *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMatrixF::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::math::CMatrixF *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMatrixF::serializeFrom(a0, a1);
	}
};

void bind_mrpt_math_CMatrixF(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixF file:mrpt/math/CMatrixF.h line:22
		pybind11::class_<mrpt::math::CMatrixF, std::shared_ptr<mrpt::math::CMatrixF>, PyCallBack_mrpt_math_CMatrixF, mrpt::serialization::CSerializable, mrpt::math::CMatrixDynamic<float>> cl(M("mrpt::math"), "CMatrixF", "This class is a \"CSerializable\" wrapper for \"CMatrixFloat\".\n \n\n For a complete introduction to Matrices and vectors in MRPT, see:\n https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixF(); }, [](){ return new PyCallBack_mrpt_math_CMatrixF(); } ) );
		cl.def( pybind11::init<size_t, size_t>(), pybind11::arg("row"), pybind11::arg("col") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<float> &>(), pybind11::arg("m") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<double> &>(), pybind11::arg("m") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_math_CMatrixF const &o){ return new PyCallBack_mrpt_math_CMatrixF(o); } ) );
		cl.def( pybind11::init( [](mrpt::math::CMatrixF const &o){ return new mrpt::math::CMatrixF(o); } ) );
		cl.def("assign", (class mrpt::math::CMatrixF & (mrpt::math::CMatrixF::*)(const class mrpt::math::CMatrixF &)) &mrpt::math::CMatrixF::operator=<mrpt::math::CMatrixF>, "C++: mrpt::math::CMatrixF::operator=(const class mrpt::math::CMatrixF &) --> class mrpt::math::CMatrixF &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("assign", (class mrpt::math::CMatrixF & (mrpt::math::CMatrixF::*)(const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::CMatrixF::operator=<mrpt::math::CMatrixDynamic<float>>, "C++: mrpt::math::CMatrixF::operator=(const class mrpt::math::CMatrixDynamic<float> &) --> class mrpt::math::CMatrixF &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::math::CMatrixF::GetRuntimeClassIdStatic, "C++: mrpt::math::CMatrixF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::math::CMatrixF::*)() const) &mrpt::math::CMatrixF::GetRuntimeClass, "C++: mrpt::math::CMatrixF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::math::CMatrixF::*)() const) &mrpt::math::CMatrixF::clone, "C++: mrpt::math::CMatrixF::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::math::CMatrixF::CreateObject, "C++: mrpt::math::CMatrixF::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::math::CMatrixF & (mrpt::math::CMatrixF::*)(const class mrpt::math::CMatrixF &)) &mrpt::math::CMatrixF::operator=, "C++: mrpt::math::CMatrixF::operator=(const class mrpt::math::CMatrixF &) --> class mrpt::math::CMatrixF &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
