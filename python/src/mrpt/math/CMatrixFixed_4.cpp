#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
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

void bind_mrpt_math_CMatrixFixed_4(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,12UL,1UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,12UL,1UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_12UL_1UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,12UL,1UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,12UL,1UL> const &o){ return new mrpt::math::CMatrixFixed<double,12UL,1UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 12, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,12UL,1UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 12, 1>::setSize, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(class mrpt::math::CMatrixFixed<double, 12, 1> &)) &mrpt::math::CMatrixFixed<double, 12, 1>::swap, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::swap(class mrpt::math::CMatrixFixed<double, 12, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 12, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 12, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 12, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 12, 1>::rows, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 12, 1>::cols, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)()) &mrpt::math::CMatrixFixed<double, 12, 1>::data, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 12, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 12, 1>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 12, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 12, 1>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 12, 1>::operator[], "C++: mrpt::math::CMatrixFixed<double, 12, 1>::operator[](int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 12, 1> &)) &mrpt::math::CMatrixFixed<double, 12, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::sum_At(const class mrpt::math::CMatrixFixed<double, 12, 1> &) --> void", pybind11::arg("A"));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<float,4UL,1UL>, std::shared_ptr<mrpt::math::CMatrixFixed<float,4UL,1UL>>> cl(M("mrpt::math"), "CMatrixFixed_float_4UL_1UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<float,4UL,1UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const float *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<float,4UL,1UL> const &o){ return new mrpt::math::CMatrixFixed<float,4UL,1UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(const float *)) &mrpt::math::CMatrixFixed<float, 4, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::loadFromRawPointer(const float *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<float,4UL,1UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<float, 4, 1>::setSize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(class mrpt::math::CMatrixFixed<float, 4, 1> &)) &mrpt::math::CMatrixFixed<float, 4, 1>::swap, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::swap(class mrpt::math::CMatrixFixed<float, 4, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 4, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(size_t)) &mrpt::math::CMatrixFixed<float, 4, 1>::resize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 4, 1>::resize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::rows, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::cols, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::cols() const --> int");
		cl.def("data", (float * (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)()) &mrpt::math::CMatrixFixed<float, 4, 1>::data, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::data() --> float *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(int, int)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator(), "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator()(int, int) --> float &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator(), "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator()(int) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 4, 1> (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::cast_float, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 4, 1>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(const class mrpt::math::CMatrixFixed<float, 4, 1> &)) &mrpt::math::CMatrixFixed<float, 4, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::sum_At(const class mrpt::math::CMatrixFixed<float, 4, 1> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<float, 4, 1> & (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(const class mrpt::math::CMatrixFixed<float, 4, 1> &)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator=, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator=(const class mrpt::math::CMatrixFixed<float, 4, 1> &) --> class mrpt::math::CMatrixFixed<float, 4, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		// Manually-added matrix methods:
		using dat_t = float;
		using mat_t = mrpt::math::CMatrixFixed<dat_t,4,1>;
		cl.def("__getitem__", [](const mat_t&self, pybind11::tuple coord) -> dat_t { if (coord.size()==2) return self.coeff(coord[0].cast<size_t>(), coord[1].cast<size_t>()); else if (coord.size()==1) return self[coord[0].cast<size_t>()]; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__setitem__", [](mat_t&self, pybind11::tuple coord, dat_t val) { if (coord.size()==2) self.coeffRef(coord[0].cast<size_t>(), coord[1].cast<size_t>())=val; else if (coord.size()==1) self[coord[0].cast<size_t>()]=val; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__str__", [](const mat_t& o) -> std::string { return o.asString(); } );
		cl.def("inMatlabFormat", [](const mat_t& o) -> std::string { return o.inMatlabFormat(); } );
		cl.def("size", [](const mat_t&self) -> pybind11::tuple { return pybind11::make_tuple(self.cols(),self.rows()); });
		cl.def_static("Zero", []() -> mat_t { return mat_t::Zero(); }, "Returns a matrix with zeroes");
		cl.def(pybind11::init( [](pybind11::list vals){ auto m = new mat_t(); const auto nR = vals.size(); if (!nR) return m; const auto nC = vals[0].cast<pybind11::list>().size(); m->setSize(nR,nC); for (size_t r=0;r<nR;r++) { const auto row = vals[r].cast<pybind11::list>(); for (size_t c=0;c<nC;c++) m->coeffRef(r,c) = row[c].cast<dat_t>(); } return m; }));
		cl.def("to_list", [](const mat_t&self) -> pybind11::list { auto l = pybind11::list(); const auto nR = self.rows(), nC = self.cols(); for (size_t r=0;r<nR;r++) { auto row = pybind11::list(); l.append(row); for (size_t c=0;c<nC;c++) row.append(self.coeff(r,c)); } return l; });
	}
	// mrpt::math::TMatrixTextFileFormat file:mrpt/math/MatrixVectorBase.h line:35
	pybind11::enum_<mrpt::math::TMatrixTextFileFormat>(M("mrpt::math"), "TMatrixTextFileFormat", pybind11::arithmetic(), "Selection of the number format in MatrixVectorBase::saveToTextFile()\n \n")
		.value("MATRIX_FORMAT_ENG", mrpt::math::MATRIX_FORMAT_ENG)
		.value("MATRIX_FORMAT_FIXED", mrpt::math::MATRIX_FORMAT_FIXED)
		.value("MATRIX_FORMAT_INT", mrpt::math::MATRIX_FORMAT_INT)
		.export_values();

;

	{ // mrpt::math::CMatrixD file:mrpt/math/CMatrixD.h line:23
		pybind11::class_<mrpt::math::CMatrixD, std::shared_ptr<mrpt::math::CMatrixD>, PyCallBack_mrpt_math_CMatrixD, mrpt::serialization::CSerializable, mrpt::math::CMatrixDynamic<double>> cl(M("mrpt::math"), "CMatrixD", "This class is a \"CSerializable\" wrapper for\n \"CMatrixDynamic<double>\".\n \n\n For a complete introduction to Matrices and vectors in MRPT, see:\n https://www.mrpt.org/Matrices_vectors_arrays_and_Linear_Algebra_MRPT_and_Eigen_classes\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixD(); }, [](){ return new PyCallBack_mrpt_math_CMatrixD(); } ) );
		cl.def( pybind11::init<size_t, size_t>(), pybind11::arg("row"), pybind11::arg("col") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<double> &>(), pybind11::arg("m") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<float> &>(), pybind11::arg("m") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_math_CMatrixD const &o){ return new PyCallBack_mrpt_math_CMatrixD(o); } ) );
		cl.def( pybind11::init( [](mrpt::math::CMatrixD const &o){ return new mrpt::math::CMatrixD(o); } ) );
		cl.def("assign", (class mrpt::math::CMatrixD & (mrpt::math::CMatrixD::*)(const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::CMatrixD::operator=<mrpt::math::CMatrixDynamic<float>>, "C++: mrpt::math::CMatrixD::operator=(const class mrpt::math::CMatrixDynamic<float> &) --> class mrpt::math::CMatrixD &", pybind11::return_value_policy::automatic, pybind11::arg("other"));
		cl.def("assign", (class mrpt::math::CMatrixD & (mrpt::math::CMatrixD::*)(const class mrpt::math::CMatrixD &)) &mrpt::math::CMatrixD::operator=<mrpt::math::CMatrixD>, "C++: mrpt::math::CMatrixD::operator=(const class mrpt::math::CMatrixD &) --> class mrpt::math::CMatrixD &", pybind11::return_value_policy::automatic, pybind11::arg("other"));
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::math::CMatrixD::GetRuntimeClassIdStatic, "C++: mrpt::math::CMatrixD::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::math::CMatrixD::*)() const) &mrpt::math::CMatrixD::GetRuntimeClass, "C++: mrpt::math::CMatrixD::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::math::CMatrixD::*)() const) &mrpt::math::CMatrixD::clone, "C++: mrpt::math::CMatrixD::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::math::CMatrixD::CreateObject, "C++: mrpt::math::CMatrixD::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::math::CMatrixD & (mrpt::math::CMatrixD::*)(const class mrpt::math::CMatrixD &)) &mrpt::math::CMatrixD::operator=, "C++: mrpt::math::CMatrixD::operator=(const class mrpt::math::CMatrixD &) --> class mrpt::math::CMatrixD &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
