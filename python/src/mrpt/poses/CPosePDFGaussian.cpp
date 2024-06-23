#include <ios>
#include <iterator>
#include <locale>
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
#include <mrpt/poses/CPoint2DPDF.h>
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
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
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

// mrpt::poses::CPosePDFGaussian file:mrpt/poses/CPosePDFGaussian.h line:28
struct PyCallBack_mrpt_poses_CPosePDFGaussian : public mrpt::poses::CPosePDFGaussian {
	using mrpt::poses::CPosePDFGaussian::CPosePDFGaussian;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPosePDFGaussian::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPosePDFGaussian::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPosePDFGaussian::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussian::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussian::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussian::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPosePDFGaussian::getCovarianceAndMean();
	}
	void copyFrom(const class mrpt::poses::CPosePDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussian::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPosePDFGaussian::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussian::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussian::drawSingleSample(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussian::bayesianFusion(a0, a1, a2);
	}
	void inverse(class mrpt::poses::CPosePDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussian::inverse(a0);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussian *>(this), "getInformationMatrix");
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
};

void bind_mrpt_poses_CPosePDFGaussian(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPosePDFGaussian file:mrpt/poses/CPosePDFGaussian.h line:28
		pybind11::class_<mrpt::poses::CPosePDFGaussian, std::shared_ptr<mrpt::poses::CPosePDFGaussian>, PyCallBack_mrpt_poses_CPosePDFGaussian, mrpt::poses::CPosePDF> cl(M("mrpt::poses"), "CPosePDFGaussian", "Declares a class that represents a Probability Density  function (PDF) of a\n 2D pose \n\n.\n\n   This class implements that PDF using a mono-modal Gaussian distribution.\n See mrpt::poses::CPosePDF for more details.\n\n \n CPose2D, CPosePDF, CPosePDFParticles\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPosePDFGaussian(); }, [](){ return new PyCallBack_mrpt_poses_CPosePDFGaussian(); } ) );
		cl.def( pybind11::init<const class mrpt::poses::CPose2D &>(), pybind11::arg("init_Mean") );

		cl.def( pybind11::init<const class mrpt::poses::CPose2D &, const class mrpt::math::CMatrixFixed<double, 3, 3> &>(), pybind11::arg("init_Mean"), pybind11::arg("init_Cov") );

		cl.def( pybind11::init<const class mrpt::poses::CPosePDF &>(), pybind11::arg("o") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DPDF &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPosePDFGaussian const &o){ return new PyCallBack_mrpt_poses_CPosePDFGaussian(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPosePDFGaussian const &o){ return new mrpt::poses::CPosePDFGaussian(o); } ) );
		cl.def_readwrite("mean", &mrpt::poses::CPosePDFGaussian::mean);
		cl.def_readwrite("cov", &mrpt::poses::CPosePDFGaussian::cov);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPosePDFGaussian::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPosePDFGaussian::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPosePDFGaussian::*)() const) &mrpt::poses::CPosePDFGaussian::GetRuntimeClass, "C++: mrpt::poses::CPosePDFGaussian::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPosePDFGaussian::*)() const) &mrpt::poses::CPosePDFGaussian::clone, "C++: mrpt::poses::CPosePDFGaussian::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPosePDFGaussian::CreateObject, "C++: mrpt::poses::CPosePDFGaussian::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getPoseMean", (class mrpt::poses::CPose2D & (mrpt::poses::CPosePDFGaussian::*)()) &mrpt::poses::CPosePDFGaussian::getPoseMean, "C++: mrpt::poses::CPosePDFGaussian::getPoseMean() --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic);
		cl.def("getMean", (void (mrpt::poses::CPosePDFGaussian::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGaussian::getMean, "C++: mrpt::poses::CPosePDFGaussian::getMean(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("mean_pose"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D> (mrpt::poses::CPosePDFGaussian::*)() const) &mrpt::poses::CPosePDFGaussian::getCovarianceAndMean, "C++: mrpt::poses::CPosePDFGaussian::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>");
		cl.def("copyFrom", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPosePDFGaussian::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPosePDFGaussian::copyFrom(const class mrpt::poses::CPosePDF &) --> void", pybind11::arg("o"));
		cl.def("copyFrom", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPosePDFGaussian::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPosePDFGaussian::copyFrom(const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPosePDFGaussian::*)(const std::string &) const) &mrpt::poses::CPosePDFGaussian::saveToTextFile, "Save PDF's particles to a text file, containing the 2D pose in the first\n line, then the covariance matrix in next 3 lines. \n\nC++: mrpt::poses::CPosePDFGaussian::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPosePDFGaussian::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object.\n\nC++: mrpt::poses::CPosePDFGaussian::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::CPosePDFGaussian::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object.\n\nC++: mrpt::poses::CPosePDFGaussian::changeCoordinatesReference(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("rotateCov", (void (mrpt::poses::CPosePDFGaussian::*)(const double)) &mrpt::poses::CPosePDFGaussian::rotateCov, "Rotate the covariance matrix by replacing it by \n\n\n, where \n\n\n\n.\n\nC++: mrpt::poses::CPosePDFGaussian::rotateCov(const double) --> void", pybind11::arg("ang"));
		cl.def("inverseComposition", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPosePDFGaussian &, const class mrpt::poses::CPosePDFGaussian &)) &mrpt::poses::CPosePDFGaussian::inverseComposition, "Set \n , computing the mean using the \"-\"\n operator and the covariances through the corresponding Jacobians (For\n 'x0' and 'x1' being independent variables!). \n\nC++: mrpt::poses::CPosePDFGaussian::inverseComposition(const class mrpt::poses::CPosePDFGaussian &, const class mrpt::poses::CPosePDFGaussian &) --> void", pybind11::arg("x"), pybind11::arg("ref"));
		cl.def("inverseComposition", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPosePDFGaussian &, const class mrpt::poses::CPosePDFGaussian &, const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::poses::CPosePDFGaussian::inverseComposition, "Set \n , computing the mean using the \"-\"\n operator and the covariances through the corresponding Jacobians (Given\n the 3x3 cross-covariance matrix of variables x0 and x1). \n\nC++: mrpt::poses::CPosePDFGaussian::inverseComposition(const class mrpt::poses::CPosePDFGaussian &, const class mrpt::poses::CPosePDFGaussian &, const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("x1"), pybind11::arg("x0"), pybind11::arg("COV_01"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPosePDFGaussian::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGaussian::drawSingleSample, "Draws a single sample from the distribution\n\nC++: mrpt::poses::CPosePDFGaussian::drawSingleSample(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("outPart"));
		cl.def("bayesianFusion", [](mrpt::poses::CPosePDFGaussian &o, const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double)) &mrpt::poses::CPosePDFGaussian::bayesianFusion, "Bayesian fusion of two points gauss. distributions, then save the result\nin this object.\n  The process is as follows:\n		- (x1,S1): Mean and variance of the p1 distribution.\n		- (x2,S2): Mean and variance of the p2 distribution.\n		- (x,S): Mean and variance of the resulting distribution.\n\n    \n\n    \n\n   \n\nC++: mrpt::poses::CPosePDFGaussian::bayesianFusion(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("inverse", (void (mrpt::poses::CPosePDFGaussian::*)(class mrpt::poses::CPosePDF &) const) &mrpt::poses::CPosePDFGaussian::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF\n\nC++: mrpt::poses::CPosePDFGaussian::inverse(class mrpt::poses::CPosePDF &) const --> void", pybind11::arg("o"));
		cl.def("__iadd__", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::CPosePDFGaussian::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated). \n\nC++: mrpt::poses::CPosePDFGaussian::operator+=(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("Ap"));
		cl.def("evaluatePDF", (double (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGaussian::evaluatePDF, "Evaluates the PDF at a given point. \n\nC++: mrpt::poses::CPosePDFGaussian::evaluatePDF(const class mrpt::poses::CPose2D &) const --> double", pybind11::arg("x"));
		cl.def("evaluateNormalizedPDF", (double (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGaussian::evaluateNormalizedPDF, "Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in\n the range [0,1]. \n\nC++: mrpt::poses::CPosePDFGaussian::evaluateNormalizedPDF(const class mrpt::poses::CPose2D &) const --> double", pybind11::arg("x"));
		cl.def("mahalanobisDistanceTo", (double (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPosePDFGaussian &)) &mrpt::poses::CPosePDFGaussian::mahalanobisDistanceTo, "Computes the Mahalanobis distance between the centers of two Gaussians.\n\nC++: mrpt::poses::CPosePDFGaussian::mahalanobisDistanceTo(const class mrpt::poses::CPosePDFGaussian &) --> double", pybind11::arg("theOther"));
		cl.def("assureMinCovariance", (void (mrpt::poses::CPosePDFGaussian::*)(double, double)) &mrpt::poses::CPosePDFGaussian::assureMinCovariance, "Substitutes the diagonal elements if (square) they are below some given\n minimum values (Use this before bayesianFusion, for example, to avoid\n inversion of singular matrixes, etc...)  \n\nC++: mrpt::poses::CPosePDFGaussian::assureMinCovariance(double, double) --> void", pybind11::arg("minStdXY"), pybind11::arg("minStdPhi"));
		cl.def("__iadd__", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPosePDFGaussian &)) &mrpt::poses::CPosePDFGaussian::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated) (see formulas in\n jacobiansPoseComposition ). \n\nC++: mrpt::poses::CPosePDFGaussian::operator+=(const class mrpt::poses::CPosePDFGaussian &) --> void", pybind11::arg("Ap"));
		cl.def("__isub__", (void (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPosePDFGaussian &)) &mrpt::poses::CPosePDFGaussian::operator-=, "Makes: thisPDF = thisPDF - Ap, where \"-\" is pose inverse composition\n (both the mean, and the covariance matrix are updated) \n\nC++: mrpt::poses::CPosePDFGaussian::operator-=(const class mrpt::poses::CPosePDFGaussian &) --> void", pybind11::arg("ref"));
		cl.def("composePoint", (void (mrpt::poses::CPosePDFGaussian::*)(const struct mrpt::math::TPoint2D_<double> &, class mrpt::poses::CPoint2DPDFGaussian &) const) &mrpt::poses::CPosePDFGaussian::composePoint, "Returns the PDF of the 2D point \n with \"q\"=this pose\n and \"l\" a point without uncertainty \n\nC++: mrpt::poses::CPosePDFGaussian::composePoint(const struct mrpt::math::TPoint2D_<double> &, class mrpt::poses::CPoint2DPDFGaussian &) const --> void", pybind11::arg("l"), pybind11::arg("g"));
		cl.def("assign", (class mrpt::poses::CPosePDFGaussian & (mrpt::poses::CPosePDFGaussian::*)(const class mrpt::poses::CPosePDFGaussian &)) &mrpt::poses::CPosePDFGaussian::operator=, "C++: mrpt::poses::CPosePDFGaussian::operator=(const class mrpt::poses::CPosePDFGaussian &) --> class mrpt::poses::CPosePDFGaussian &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPosePDFGaussian const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );

		cl.def("__add__", [](const mrpt::poses::CPosePDFGaussian&a, const mrpt::poses::CPosePDFGaussian& b) -> mrpt::poses::CPosePDFGaussian { return a+b; });
		cl.def("__sub__", [](const mrpt::poses::CPosePDFGaussian&a, const mrpt::poses::CPosePDFGaussian& b) -> mrpt::poses::CPosePDFGaussian { return a-b; });
	}
}
