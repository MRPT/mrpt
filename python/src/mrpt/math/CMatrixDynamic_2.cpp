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

void bind_mrpt_math_CMatrixDynamic_2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixDynamic file:mrpt/math/CMatrixDynamic.h line:41
		pybind11::class_<mrpt::math::CMatrixDynamic<double>, std::shared_ptr<mrpt::math::CMatrixDynamic<double>>> cl(M("mrpt::math"), "CMatrixDynamic_double_t", "");
		cl.def( pybind11::init( [](mrpt::math::CMatrixDynamic<double> const &o){ return new mrpt::math::CMatrixDynamic<double>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixDynamic<double>(); } ), "doc" );
		cl.def( pybind11::init( [](size_t const & a0){ return new mrpt::math::CMatrixDynamic<double>(a0); } ), "doc" , pybind11::arg("row"));
		cl.def( pybind11::init<size_t, size_t>(), pybind11::arg("row"), pybind11::arg("col") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<double> &, size_t, size_t>(), pybind11::arg("m"), pybind11::arg("cropRowCount"), pybind11::arg("cropColCount") );

		cl.def("setFromMatrixLike", (void (mrpt::math::CMatrixDynamic<double>::*)(const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::CMatrixDynamic<double>::setFromMatrixLike<mrpt::math::CMatrixDynamic<float>>, "C++: mrpt::math::CMatrixDynamic<double>::setFromMatrixLike(const class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("m"));
		cl.def("setFromMatrixLike", (void (mrpt::math::CMatrixDynamic<double>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::math::CMatrixDynamic<double>::setFromMatrixLike<mrpt::math::CMatrixFixed<double, 3, 3>>, "C++: mrpt::math::CMatrixDynamic<double>::setFromMatrixLike(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("m"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<double> & (mrpt::math::CMatrixDynamic<double>::*)(const class mrpt::math::CMatrixDynamic<double> &)) &mrpt::math::CMatrixDynamic<double>::operator=<double>, "C++: mrpt::math::CMatrixDynamic<double>::operator=(const class mrpt::math::CMatrixDynamic<double> &) --> class mrpt::math::CMatrixDynamic<double> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<double> & (mrpt::math::CMatrixDynamic<double>::*)(const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::CMatrixDynamic<double>::operator=<float>, "C++: mrpt::math::CMatrixDynamic<double>::operator=(const class mrpt::math::CMatrixDynamic<float> &) --> class mrpt::math::CMatrixDynamic<double> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<double> & (mrpt::math::CMatrixDynamic<double>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::math::CMatrixDynamic<double>::operator=<3UL,3UL>, "C++: mrpt::math::CMatrixDynamic<double>::operator=(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> class mrpt::math::CMatrixDynamic<double> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("swap", (void (mrpt::math::CMatrixDynamic<double>::*)(class mrpt::math::CMatrixDynamic<double> &)) &mrpt::math::CMatrixDynamic<double>::swap, "C++: mrpt::math::CMatrixDynamic<double>::swap(class mrpt::math::CMatrixDynamic<double> &) --> void", pybind11::arg("o"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<double> & (mrpt::math::CMatrixDynamic<double>::*)(const class mrpt::math::CMatrixDynamic<double> &)) &mrpt::math::CMatrixDynamic<double>::operator=, "C++: mrpt::math::CMatrixDynamic<double>::operator=(const class mrpt::math::CMatrixDynamic<double> &) --> class mrpt::math::CMatrixDynamic<double> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("rows", (int (mrpt::math::CMatrixDynamic<double>::*)() const) &mrpt::math::CMatrixDynamic<double>::rows, "C++: mrpt::math::CMatrixDynamic<double>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixDynamic<double>::*)() const) &mrpt::math::CMatrixDynamic<double>::cols, "C++: mrpt::math::CMatrixDynamic<double>::cols() const --> int");
		cl.def("setSize", [](mrpt::math::CMatrixDynamic<double> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixDynamic<double>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixDynamic<double>::setSize, "C++: mrpt::math::CMatrixDynamic<double>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<double>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<double>::resize, "C++: mrpt::math::CMatrixDynamic<double>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<double>::*)(size_t)) &mrpt::math::CMatrixDynamic<double>::resize, "C++: mrpt::math::CMatrixDynamic<double>::resize(size_t) --> void", pybind11::arg("vectorLen"));
		cl.def("derived", (class mrpt::math::CMatrixDynamic<double> & (mrpt::math::CMatrixDynamic<double>::*)()) &mrpt::math::CMatrixDynamic<double>::derived, "C++: mrpt::math::CMatrixDynamic<double>::derived() --> class mrpt::math::CMatrixDynamic<double> &", pybind11::return_value_policy::automatic);
		cl.def("conservativeResize", (void (mrpt::math::CMatrixDynamic<double>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<double>::conservativeResize, "C++: mrpt::math::CMatrixDynamic<double>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("data", (double * (mrpt::math::CMatrixDynamic<double>::*)()) &mrpt::math::CMatrixDynamic<double>::data, "C++: mrpt::math::CMatrixDynamic<double>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixDynamic<double>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<double>::operator(), "C++: mrpt::math::CMatrixDynamic<double>::operator()(size_t, size_t) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("cast_float", (class mrpt::math::CMatrixDynamic<float> (mrpt::math::CMatrixDynamic<double>::*)() const) &mrpt::math::CMatrixDynamic<double>::cast_float, "C++: mrpt::math::CMatrixDynamic<double>::cast_float() const --> class mrpt::math::CMatrixDynamic<float>");
		cl.def("cast_double", (class mrpt::math::CMatrixDynamic<double> (mrpt::math::CMatrixDynamic<double>::*)() const) &mrpt::math::CMatrixDynamic<double>::cast_double, "C++: mrpt::math::CMatrixDynamic<double>::cast_double() const --> class mrpt::math::CMatrixDynamic<double>");
		cl.def("llt_solve", (class mrpt::math::CVectorDynamic<double> (mrpt::math::CMatrixDynamic<double>::*)(const class mrpt::math::CVectorDynamic<double> &) const) &mrpt::math::CMatrixDynamic<double>::llt_solve, "C++: mrpt::math::CMatrixDynamic<double>::llt_solve(const class mrpt::math::CVectorDynamic<double> &) const --> class mrpt::math::CVectorDynamic<double>", pybind11::arg("b"));
		cl.def("lu_solve", (class mrpt::math::CVectorDynamic<double> (mrpt::math::CMatrixDynamic<double>::*)(const class mrpt::math::CVectorDynamic<double> &) const) &mrpt::math::CMatrixDynamic<double>::lu_solve, "C++: mrpt::math::CMatrixDynamic<double>::lu_solve(const class mrpt::math::CVectorDynamic<double> &) const --> class mrpt::math::CVectorDynamic<double>", pybind11::arg("b"));

		// Manually-added matrix methods:
		using dat_t = double;
		using mat_t = mrpt::math::CMatrixDynamic<dat_t>;
		cl.def("__getitem__", [](const mat_t&self, pybind11::tuple coord) -> dat_t { if (coord.size()==2) return self.coeff(coord[0].cast<size_t>(), coord[1].cast<size_t>()); else if (coord.size()==1) return self[coord[0].cast<size_t>()]; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__setitem__", [](mat_t&self, pybind11::tuple coord, dat_t val) { if (coord.size()==2) self.coeffRef(coord[0].cast<size_t>(), coord[1].cast<size_t>())=val; else if (coord.size()==1) self[coord[0].cast<size_t>()]=val; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__str__", [](const mat_t& o) -> std::string { return o.asString(); } );
		cl.def("inMatlabFormat", [](const mat_t& o) -> std::string { return o.inMatlabFormat(); } );
		cl.def("size", [](const mat_t&self) -> pybind11::tuple { return pybind11::make_tuple(self.cols(),self.rows()); });
		cl.def_static("Identity", [](const size_t N) -> mat_t { return mat_t::Identity(N); }, "Returns the NxN identity matrix");
		cl.def_static("Zero", [](const size_t nRows, const size_t nCols) -> mat_t { return mat_t::Zero(nRows,nCols); }, "Returns a matrix with zeroes");
		cl.def(pybind11::init( [](pybind11::list vals){ auto m = new mat_t(); const auto nR = vals.size(); if (!nR) return m; const auto nC = vals[0].cast<pybind11::list>().size(); m->setSize(nR,nC); for (size_t r=0;r<nR;r++) { const auto row = vals[r].cast<pybind11::list>(); for (size_t c=0;c<nC;c++) m->coeffRef(r,c) = row[c].cast<dat_t>(); } return m; }));
		cl.def("to_list", [](const mat_t&self) -> pybind11::list { auto l = pybind11::list(); const auto nR = self.rows(), nC = self.cols(); for (size_t r=0;r<nR;r++) { auto row = pybind11::list(); l.append(row); for (size_t c=0;c<nC;c++) row.append(self.coeff(r,c)); } return l; });
	}
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
