#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CProbabilityDensityFunction.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <tuple>
#include <variant>
#include <vector>

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

// mrpt::poses::CPosePDF file:mrpt/poses/CPosePDF.h line:36
struct PyCallBack_mrpt_poses_CPosePDF : public mrpt::poses::CPosePDF {
	using mrpt::poses::CPosePDF::CPosePDF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPosePDF::GetRuntimeClass();
	}
	void copyFrom(const class mrpt::poses::CPosePDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPosePDF::copyFrom\"");
	}
	void bayesianFusion(const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPosePDF::bayesianFusion\"");
	}
	void inverse(class mrpt::poses::CPosePDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPosePDF::inverse\"");
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPosePDF::changeCoordinatesReference\"");
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeGetVersion\"");
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeTo\"");
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CSerializable::serializeFrom\"");
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CObject::clone\"");
	}
	void getMean(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::getMean\"");
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::getCovarianceAndMean\"");
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "isInfType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CProbabilityDensityFunction::isInfType();
	}
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "getInformationMatrix");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CProbabilityDensityFunction::getInformationMatrix(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::saveToTextFile\"");
	}
	void drawSingleSample(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDF *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CProbabilityDensityFunction::drawSingleSample\"");
	}
};

void bind_mrpt_poses_CPosePDF(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPosePDF file:mrpt/poses/CPosePDF.h line:36
		pybind11::class_<mrpt::poses::CPosePDF, std::shared_ptr<mrpt::poses::CPosePDF>, PyCallBack_mrpt_poses_CPosePDF, mrpt::serialization::CSerializable, mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose2D,3UL>> cl(M("mrpt::poses"), "CPosePDF", "Declares a class that represents a probability density function (pdf) of a\n 2D pose (x,y,phi).\n   This class is just the base class for unifying many diferent ways this pdf\n can be implemented.\n\n  For convenience, a pose composition is also defined for any pdf derived\n class,\n   changeCoordinatesReference, in the form of a method rather than an\n operator.\n\n  See also:\n  [probabilistic spatial representations](tutorial-pdf-over-poses.html)\n\n \n CPose2D, CPose3DPDF, CPoseRandomSampler\n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPosePDF const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPosePDF(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPosePDF::*)() const) &mrpt::poses::CPosePDF::GetRuntimeClass, "C++: mrpt::poses::CPosePDF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPosePDF::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPosePDF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("copyFrom", (void (mrpt::poses::CPosePDF::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPosePDF::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations)\n\nC++: mrpt::poses::CPosePDF::copyFrom(const class mrpt::poses::CPosePDF &) --> void", pybind11::arg("o"));
		cl.def("bayesianFusion", [](mrpt::poses::CPosePDF &o, const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPosePDF::*)(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double)) &mrpt::poses::CPosePDF::bayesianFusion, "Bayesian fusion of two pose distributions (product of two\n distributions->new distribution), then save the result in this object\n (WARNING: See implementing classes to see classes that can and cannot be\n mixtured!)\n \n\n The first distribution to fuse\n \n\n The second distribution to fuse\n \n\n If set to different of 0, the result of\n very separate Gaussian modes (that will result in negligible components)\n in SOGs will be dropped to reduce the number of modes in the output.\n\nC++: mrpt::poses::CPosePDF::bayesianFusion(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("inverse", (void (mrpt::poses::CPosePDF::*)(class mrpt::poses::CPosePDF &) const) &mrpt::poses::CPosePDF::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF\n\nC++: mrpt::poses::CPosePDF::inverse(class mrpt::poses::CPosePDF &) const --> void", pybind11::arg("o"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPosePDF::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPosePDF::changeCoordinatesReference, "C++: mrpt::poses::CPosePDF::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def_static("jacobiansPoseComposition", [](const class mrpt::poses::CPose2D & a0, const class mrpt::poses::CPose2D & a1, class mrpt::math::CMatrixFixed<double, 3, 3> & a2, class mrpt::math::CMatrixFixed<double, 3, 3> & a3) -> void { return mrpt::poses::CPosePDF::jacobiansPoseComposition(a0, a1, a2, a3); }, "", pybind11::arg("x"), pybind11::arg("u"), pybind11::arg("df_dx"), pybind11::arg("df_du"));
		cl.def_static("jacobiansPoseComposition", [](const class mrpt::poses::CPose2D & a0, const class mrpt::poses::CPose2D & a1, class mrpt::math::CMatrixFixed<double, 3, 3> & a2, class mrpt::math::CMatrixFixed<double, 3, 3> & a3, const bool & a4) -> void { return mrpt::poses::CPosePDF::jacobiansPoseComposition(a0, a1, a2, a3, a4); }, "", pybind11::arg("x"), pybind11::arg("u"), pybind11::arg("df_dx"), pybind11::arg("df_du"), pybind11::arg("compute_df_dx"));
		cl.def_static("jacobiansPoseComposition", (void (*)(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &, class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::math::CMatrixFixed<double, 3, 3> &, const bool, const bool)) &mrpt::poses::CPosePDF::jacobiansPoseComposition, "This static method computes the pose composition Jacobians, with these\n   formulas:\n    \n\n\n\n\n\n\n\n\n\n\n\n  \n\nC++: mrpt::poses::CPosePDF::jacobiansPoseComposition(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &, class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::math::CMatrixFixed<double, 3, 3> &, const bool, const bool) --> void", pybind11::arg("x"), pybind11::arg("u"), pybind11::arg("df_dx"), pybind11::arg("df_du"), pybind11::arg("compute_df_dx"), pybind11::arg("compute_df_du"));
		cl.def_static("jacobiansPoseComposition", (void (*)(const class mrpt::poses::CPosePDFGaussian &, const class mrpt::poses::CPosePDFGaussian &, class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::poses::CPosePDF::jacobiansPoseComposition, "C++: mrpt::poses::CPosePDF::jacobiansPoseComposition(const class mrpt::poses::CPosePDFGaussian &, const class mrpt::poses::CPosePDFGaussian &, class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("x"), pybind11::arg("u"), pybind11::arg("df_dx"), pybind11::arg("df_du"));
		cl.def_static("is_3D", (bool (*)()) &mrpt::poses::CPosePDF::is_3D, "C++: mrpt::poses::CPosePDF::is_3D() --> bool");
		cl.def_static("is_PDF", (bool (*)()) &mrpt::poses::CPosePDF::is_PDF, "C++: mrpt::poses::CPosePDF::is_PDF() --> bool");
		cl.def("assign", (class mrpt::poses::CPosePDF & (mrpt::poses::CPosePDF::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPosePDF::operator=, "C++: mrpt::poses::CPosePDF::operator=(const class mrpt::poses::CPosePDF &) --> class mrpt::poses::CPosePDF &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
