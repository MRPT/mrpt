#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixD.h>
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
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFSOG.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>
#include <tuple>
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

// mrpt::poses::CPointPDFSOG file:mrpt/poses/CPointPDFSOG.h line:35
struct PyCallBack_mrpt_poses_CPointPDFSOG : public mrpt::poses::CPointPDFSOG {
	using mrpt::poses::CPointPDFSOG::CPointPDFSOG;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPointPDFSOG::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPointPDFSOG::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPointPDFSOG::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFSOG::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFSOG::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFSOG::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPointPDFSOG::getCovarianceAndMean();
	}
	void copyFrom(const class mrpt::poses::CPointPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFSOG::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPointPDFSOG::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFSOG::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFSOG::drawSingleSample(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFSOG::bayesianFusion(a0, a1, a2);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFSOG *>(this), "getInformationMatrix");
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

void bind_mrpt_poses_CPointPDFSOG(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPointPDFSOG file:mrpt/poses/CPointPDFSOG.h line:35
		pybind11::class_<mrpt::poses::CPointPDFSOG, std::shared_ptr<mrpt::poses::CPointPDFSOG>, PyCallBack_mrpt_poses_CPointPDFSOG, mrpt::poses::CPointPDF> cl(M("mrpt::poses"), "CPointPDFSOG", "Declares a class that represents a Probability Density function (PDF) of a\n 3D point \n\n.\n   This class implements that PDF as the following multi-modal Gaussian\n distribution:\n\n \n\n\n  Where the number of modes N is the size of CPointPDFSOG::m_modes\n\n  See mrpt::poses::CPointPDF for more details.\n\n \n CPointPDF, CPosePDF,\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPointPDFSOG(); }, [](){ return new PyCallBack_mrpt_poses_CPointPDFSOG(); } ), "doc");
		cl.def( pybind11::init<size_t>(), pybind11::arg("nModes") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPointPDFSOG const &o){ return new PyCallBack_mrpt_poses_CPointPDFSOG(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPointPDFSOG const &o){ return new mrpt::poses::CPointPDFSOG(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPointPDFSOG::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPointPDFSOG::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPointPDFSOG::*)() const) &mrpt::poses::CPointPDFSOG::GetRuntimeClass, "C++: mrpt::poses::CPointPDFSOG::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPointPDFSOG::*)() const) &mrpt::poses::CPointPDFSOG::clone, "C++: mrpt::poses::CPointPDFSOG::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPointPDFSOG::CreateObject, "C++: mrpt::poses::CPointPDFSOG::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clear", (void (mrpt::poses::CPointPDFSOG::*)()) &mrpt::poses::CPointPDFSOG::clear, "Clear all the gaussian modes \n\nC++: mrpt::poses::CPointPDFSOG::clear() --> void");
		cl.def("__getitem__", (struct mrpt::poses::CPointPDFSOG::TGaussianMode & (mrpt::poses::CPointPDFSOG::*)(size_t)) &mrpt::poses::CPointPDFSOG::operator[], "Access to individual beacons \n\nC++: mrpt::poses::CPointPDFSOG::operator[](size_t) --> struct mrpt::poses::CPointPDFSOG::TGaussianMode &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("get", (struct mrpt::poses::CPointPDFSOG::TGaussianMode & (mrpt::poses::CPointPDFSOG::*)(size_t)) &mrpt::poses::CPointPDFSOG::get, "Access to individual beacons \n\nC++: mrpt::poses::CPointPDFSOG::get(size_t) --> struct mrpt::poses::CPointPDFSOG::TGaussianMode &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("push_back", (void (mrpt::poses::CPointPDFSOG::*)(const struct mrpt::poses::CPointPDFSOG::TGaussianMode &)) &mrpt::poses::CPointPDFSOG::push_back, "Inserts a copy of the given mode into the SOG \n\nC++: mrpt::poses::CPointPDFSOG::push_back(const struct mrpt::poses::CPointPDFSOG::TGaussianMode &) --> void", pybind11::arg("m"));
		cl.def("resize", (void (mrpt::poses::CPointPDFSOG::*)(size_t)) &mrpt::poses::CPointPDFSOG::resize, "Resize the number of SOG modes \n\nC++: mrpt::poses::CPointPDFSOG::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("size", (size_t (mrpt::poses::CPointPDFSOG::*)() const) &mrpt::poses::CPointPDFSOG::size, "Return the number of Gaussian modes. \n\nC++: mrpt::poses::CPointPDFSOG::size() const --> size_t");
		cl.def("empty", (bool (mrpt::poses::CPointPDFSOG::*)() const) &mrpt::poses::CPointPDFSOG::empty, "Return whether there is any Gaussian mode. \n\nC++: mrpt::poses::CPointPDFSOG::empty() const --> bool");
		cl.def("getMean", (void (mrpt::poses::CPointPDFSOG::*)(class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPointPDFSOG::getMean, "C++: mrpt::poses::CPointPDFSOG::getMean(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("mean_point"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D> (mrpt::poses::CPointPDFSOG::*)() const) &mrpt::poses::CPointPDFSOG::getCovarianceAndMean, "C++: mrpt::poses::CPointPDFSOG::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>");
		cl.def("normalizeWeights", (void (mrpt::poses::CPointPDFSOG::*)()) &mrpt::poses::CPointPDFSOG::normalizeWeights, "Normalize the weights in m_modes such as the maximum log-weight is 0 \n\nC++: mrpt::poses::CPointPDFSOG::normalizeWeights() --> void");
		cl.def("getMostLikelyMode", (void (mrpt::poses::CPointPDFSOG::*)(class mrpt::poses::CPointPDFGaussian &) const) &mrpt::poses::CPointPDFSOG::getMostLikelyMode, "Return the Gaussian mode with the highest likelihood (or an empty\n Gaussian if there are no modes in this SOG) \n\nC++: mrpt::poses::CPointPDFSOG::getMostLikelyMode(class mrpt::poses::CPointPDFGaussian &) const --> void", pybind11::arg("outVal"));
		cl.def("ESS", (double (mrpt::poses::CPointPDFSOG::*)() const) &mrpt::poses::CPointPDFSOG::ESS, "Computes the \"Effective sample size\" (typical measure for Particle\n Filters), applied to the weights of the individual Gaussian modes, as a\n measure of the equality of the modes (in the range [0,total # of modes]).\n\nC++: mrpt::poses::CPointPDFSOG::ESS() const --> double");
		cl.def("copyFrom", (void (mrpt::poses::CPointPDFSOG::*)(const class mrpt::poses::CPointPDF &)) &mrpt::poses::CPointPDFSOG::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPointPDFSOG::copyFrom(const class mrpt::poses::CPointPDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPointPDFSOG::*)(const std::string &) const) &mrpt::poses::CPointPDFSOG::saveToTextFile, "Save the density to a text file, with the following format:\n  There is one row per Gaussian \"mode\", and each row contains 10\n elements:\n   - w (The weight)\n   - x_mean (gaussian mean value)\n   - y_mean (gaussian mean value)\n   - x_mean (gaussian mean value)\n   - C11 (Covariance elements)\n   - C22 (Covariance elements)\n   - C33 (Covariance elements)\n   - C12 (Covariance elements)\n   - C13 (Covariance elements)\n   - C23 (Covariance elements)\n\n   \n\nC++: mrpt::poses::CPointPDFSOG::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPointPDFSOG::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPointPDFSOG::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPointPDFSOG::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPointPDFSOG::*)(class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPointPDFSOG::drawSingleSample, "Draw a sample from the pdf. \n\nC++: mrpt::poses::CPointPDFSOG::drawSingleSample(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("outSample"));
		cl.def("bayesianFusion", [](mrpt::poses::CPointPDFSOG &o, const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPointPDFSOG::*)(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double)) &mrpt::poses::CPointPDFSOG::bayesianFusion, "Bayesian fusion of two point distributions (product of two\n distributions->new distribution), then save the result in this object\n (WARNING: See implementing classes to see classes that can and cannot be\n mixtured!)\n \n\n The first distribution to fuse\n \n\n The second distribution to fuse\n \n\n If set to different of 0, the result of\n very separate Gaussian modes (that will result in negligible components)\n in SOGs will be dropped to reduce the number of modes in the output.\n\nC++: mrpt::poses::CPointPDFSOG::bayesianFusion(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("evaluatePDFInArea", [](mrpt::poses::CPointPDFSOG &o, float const & a0, float const & a1, float const & a2, float const & a3, float const & a4, float const & a5, class mrpt::math::CMatrixD & a6) -> void { return o.evaluatePDFInArea(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolutionXY"), pybind11::arg("z"), pybind11::arg("outMatrix"));
		cl.def("evaluatePDFInArea", (void (mrpt::poses::CPointPDFSOG::*)(float, float, float, float, float, float, class mrpt::math::CMatrixD &, bool)) &mrpt::poses::CPointPDFSOG::evaluatePDFInArea, "Evaluates the PDF within a rectangular grid and saves the result in a\n matrix (each row contains values for a fixed y-coordinate value).\n\nC++: mrpt::poses::CPointPDFSOG::evaluatePDFInArea(float, float, float, float, float, float, class mrpt::math::CMatrixD &, bool) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolutionXY"), pybind11::arg("z"), pybind11::arg("outMatrix"), pybind11::arg("sumOverAllZs"));
		cl.def("evaluatePDF", (double (mrpt::poses::CPointPDFSOG::*)(const class mrpt::poses::CPoint3D &, bool) const) &mrpt::poses::CPointPDFSOG::evaluatePDF, "Evaluates the PDF at a given point \n\nC++: mrpt::poses::CPointPDFSOG::evaluatePDF(const class mrpt::poses::CPoint3D &, bool) const --> double", pybind11::arg("x"), pybind11::arg("sumOverAllZs"));
		cl.def("assign", (class mrpt::poses::CPointPDFSOG & (mrpt::poses::CPointPDFSOG::*)(const class mrpt::poses::CPointPDFSOG &)) &mrpt::poses::CPointPDFSOG::operator=, "C++: mrpt::poses::CPointPDFSOG::operator=(const class mrpt::poses::CPointPDFSOG &) --> class mrpt::poses::CPointPDFSOG &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::poses::CPointPDFSOG::TGaussianMode file:mrpt/poses/CPointPDFSOG.h line:42
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::poses::CPointPDFSOG::TGaussianMode, std::shared_ptr<mrpt::poses::CPointPDFSOG::TGaussianMode>> cl(enclosing_class, "TGaussianMode", "The struct for each mode:");
			cl.def( pybind11::init( [](){ return new mrpt::poses::CPointPDFSOG::TGaussianMode(); } ) );
			cl.def( pybind11::init( [](mrpt::poses::CPointPDFSOG::TGaussianMode const &o){ return new mrpt::poses::CPointPDFSOG::TGaussianMode(o); } ) );
			cl.def_readwrite("val", &mrpt::poses::CPointPDFSOG::TGaussianMode::val);
			cl.def_readwrite("log_w", &mrpt::poses::CPointPDFSOG::TGaussianMode::log_w);
			cl.def("assign", (struct mrpt::poses::CPointPDFSOG::TGaussianMode & (mrpt::poses::CPointPDFSOG::TGaussianMode::*)(const struct mrpt::poses::CPointPDFSOG::TGaussianMode &)) &mrpt::poses::CPointPDFSOG::TGaussianMode::operator=, "C++: mrpt::poses::CPointPDFSOG::TGaussianMode::operator=(const struct mrpt::poses::CPointPDFSOG::TGaussianMode &) --> struct mrpt::poses::CPointPDFSOG::TGaussianMode &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
