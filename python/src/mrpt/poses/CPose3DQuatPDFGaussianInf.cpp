#include <ios>
#include <iterator>
#include <locale>
#include <memory>
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
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DQuatPDFGaussianInf.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
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

// mrpt::poses::CPose3DQuatPDFGaussianInf file:mrpt/poses/CPose3DQuatPDFGaussianInf.h line:40
struct PyCallBack_mrpt_poses_CPose3DQuatPDFGaussianInf : public mrpt::poses::CPose3DQuatPDFGaussianInf {
	using mrpt::poses::CPose3DQuatPDFGaussianInf::CPose3DQuatPDFGaussianInf;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPose3DQuat & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::getMean(a0);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "isInfType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::isInfType();
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 7, 7>, class mrpt::poses::CPose3DQuat>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::getCovarianceAndMean();
	}
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 7, 7> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "getInformationMatrix");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::getInformationMatrix(a0);
	}
	void copyFrom(const class mrpt::poses::CPose3DQuatPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose3DQuat & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::drawSingleSample(a0);
	}
	void inverse(class mrpt::poses::CPose3DQuatPDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussianInf *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussianInf::inverse(a0);
	}
};

// mrpt::poses::CPosePDFGaussianInf file:mrpt/poses/CPosePDFGaussianInf.h line:33
struct PyCallBack_mrpt_poses_CPosePDFGaussianInf : public mrpt::poses::CPosePDFGaussianInf {
	using mrpt::poses::CPosePDFGaussianInf::CPosePDFGaussianInf;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPosePDFGaussianInf::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPosePDFGaussianInf::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPosePDFGaussianInf::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussianInf::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussianInf::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussianInf::getMean(a0);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "isInfType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPosePDFGaussianInf::isInfType();
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPosePDFGaussianInf::getCovarianceAndMean();
	}
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "getInformationMatrix");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussianInf::getInformationMatrix(a0);
	}
	void copyFrom(const class mrpt::poses::CPosePDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussianInf::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPosePDFGaussianInf::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussianInf::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussianInf::drawSingleSample(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussianInf::bayesianFusion(a0, a1, a2);
	}
	void inverse(class mrpt::poses::CPosePDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGaussianInf *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGaussianInf::inverse(a0);
	}
};

void bind_mrpt_poses_CPose3DQuatPDFGaussianInf(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose3DQuatPDFGaussianInf file:mrpt/poses/CPose3DQuatPDFGaussianInf.h line:40
		pybind11::class_<mrpt::poses::CPose3DQuatPDFGaussianInf, std::shared_ptr<mrpt::poses::CPose3DQuatPDFGaussianInf>, PyCallBack_mrpt_poses_CPose3DQuatPDFGaussianInf, mrpt::poses::CPose3DQuatPDF> cl(M("mrpt::poses"), "CPose3DQuatPDFGaussianInf", "Declares a class that represents a Probability Density function (PDF) of a\n 3D pose using a quaternion \n\n\n\n.\n\n   This class implements that PDF using a mono-modal Gaussian distribution\n storing the information matrix instead of its inverse, the covariance matrix.\n     See mrpt::poses::CPose3DQuatPDF for more details, or\n     mrpt::poses::CPose3DPDF for classes based on Euler angles instead.\n\n  Uncertainty of pose composition operations (\n) is\n implemented in the methods \"CPose3DQuatPDFGaussianInf::operator+=\" and\n \"CPose3DQuatPDF::jacobiansPoseComposition\".\n\n \n Read also: \"A tutorial on SE(3) transformation parameterizations and\n on-manifold optimization\", in \n\n \n CPose3DQuat, CPose3DQuatPDF, CPose3DPDF, CPose3DQuatPDFGaussian\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DQuatPDFGaussianInf(); }, [](){ return new PyCallBack_mrpt_poses_CPose3DQuatPDFGaussianInf(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Quaternions>(), pybind11::arg("constructor_dummy_param") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DQuat &>(), pybind11::arg("init_Mean") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DQuat &, const class mrpt::math::CMatrixFixed<double, 7, 7> &>(), pybind11::arg("init_Mean"), pybind11::arg("init_CovInv") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3DQuatPDFGaussianInf const &o){ return new PyCallBack_mrpt_poses_CPose3DQuatPDFGaussianInf(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3DQuatPDFGaussianInf const &o){ return new mrpt::poses::CPose3DQuatPDFGaussianInf(o); } ) );
		cl.def_readwrite("mean", &mrpt::poses::CPose3DQuatPDFGaussianInf::mean);
		cl.def_readwrite("cov_inv", &mrpt::poses::CPose3DQuatPDFGaussianInf::cov_inv);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DQuatPDFGaussianInf::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DQuatPDFGaussianInf::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DQuatPDFGaussianInf::*)() const) &mrpt::poses::CPose3DQuatPDFGaussianInf::GetRuntimeClass, "C++: mrpt::poses::CPose3DQuatPDFGaussianInf::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3DQuatPDFGaussianInf::*)() const) &mrpt::poses::CPose3DQuatPDFGaussianInf::clone, "C++: mrpt::poses::CPose3DQuatPDFGaussianInf::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3DQuatPDFGaussianInf::CreateObject, "C++: mrpt::poses::CPose3DQuatPDFGaussianInf::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getPoseMean", (class mrpt::poses::CPose3DQuat & (mrpt::poses::CPose3DQuatPDFGaussianInf::*)()) &mrpt::poses::CPose3DQuatPDFGaussianInf::getPoseMean, "C++: mrpt::poses::CPose3DQuatPDFGaussianInf::getPoseMean() --> class mrpt::poses::CPose3DQuat &", pybind11::return_value_policy::automatic);
		cl.def("getMean", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuatPDFGaussianInf::getMean, "C++: mrpt::poses::CPose3DQuatPDFGaussianInf::getMean(class mrpt::poses::CPose3DQuat &) const --> void", pybind11::arg("mean_pose"));
		cl.def("isInfType", (bool (mrpt::poses::CPose3DQuatPDFGaussianInf::*)() const) &mrpt::poses::CPose3DQuatPDFGaussianInf::isInfType, "C++: mrpt::poses::CPose3DQuatPDFGaussianInf::isInfType() const --> bool");
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 7, 7>, class mrpt::poses::CPose3DQuat> (mrpt::poses::CPose3DQuatPDFGaussianInf::*)() const) &mrpt::poses::CPose3DQuatPDFGaussianInf::getCovarianceAndMean, "C++: mrpt::poses::CPose3DQuatPDFGaussianInf::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 7, 7>, class mrpt::poses::CPose3DQuat>");
		cl.def("getInformationMatrix", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(class mrpt::math::CMatrixFixed<double, 7, 7> &) const) &mrpt::poses::CPose3DQuatPDFGaussianInf::getInformationMatrix, "Returns the information (inverse covariance) matrix (a STATE_LEN x\n STATE_LEN matrix) \n\n getMean, getCovarianceAndMean \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::getInformationMatrix(class mrpt::math::CMatrixFixed<double, 7, 7> &) const --> void", pybind11::arg("inf"));
		cl.def("copyFrom", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const class mrpt::poses::CPose3DQuatPDF &)) &mrpt::poses::CPose3DQuatPDFGaussianInf::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::copyFrom(const class mrpt::poses::CPose3DQuatPDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const std::string &) const) &mrpt::poses::CPose3DQuatPDFGaussianInf::saveToTextFile, "Save the PDF to a text file, containing the 3D pose in the first line (x\n y z qr qx qy qz), then the information matrix in the next 7 lines. \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuatPDFGaussianInf::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::changeCoordinatesReference(const class mrpt::poses::CPose3DQuat &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DQuatPDFGaussianInf::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuatPDFGaussianInf::drawSingleSample, "Draws a single sample from the distribution \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::drawSingleSample(class mrpt::poses::CPose3DQuat &) const --> void", pybind11::arg("outPart"));
		cl.def("inverse", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(class mrpt::poses::CPose3DQuatPDF &) const) &mrpt::poses::CPose3DQuatPDFGaussianInf::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::inverse(class mrpt::poses::CPose3DQuatPDF &) const --> void", pybind11::arg("o"));
		cl.def("__neg__", (class mrpt::poses::CPose3DQuatPDFGaussianInf (mrpt::poses::CPose3DQuatPDFGaussianInf::*)() const) &mrpt::poses::CPose3DQuatPDFGaussianInf::operator-, "Unary - operator, returns the PDF of the inverse pose.  \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::operator-() const --> class mrpt::poses::CPose3DQuatPDFGaussianInf");
		cl.def("__iadd__", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuatPDFGaussianInf::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated). \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::operator+=(const class mrpt::poses::CPose3DQuat &) --> void", pybind11::arg("Ap"));
		cl.def("__iadd__", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const class mrpt::poses::CPose3DQuatPDFGaussianInf &)) &mrpt::poses::CPose3DQuatPDFGaussianInf::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated) (see formulas in\n jacobiansPoseComposition ). \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::operator+=(const class mrpt::poses::CPose3DQuatPDFGaussianInf &) --> void", pybind11::arg("Ap"));
		cl.def("__isub__", (void (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const class mrpt::poses::CPose3DQuatPDFGaussianInf &)) &mrpt::poses::CPose3DQuatPDFGaussianInf::operator-=, "Makes: thisPDF = thisPDF - Ap, where \"-\" is pose inverse composition\n (both the mean, and the covariance matrix are updated). \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::operator-=(const class mrpt::poses::CPose3DQuatPDFGaussianInf &) --> void", pybind11::arg("Ap"));
		cl.def("evaluatePDF", (double (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuatPDFGaussianInf::evaluatePDF, "Evaluates the PDF at a given point \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::evaluatePDF(const class mrpt::poses::CPose3DQuat &) const --> double", pybind11::arg("x"));
		cl.def("evaluateNormalizedPDF", (double (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuatPDFGaussianInf::evaluateNormalizedPDF, "Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in\n the range [0,1] \n\nC++: mrpt::poses::CPose3DQuatPDFGaussianInf::evaluateNormalizedPDF(const class mrpt::poses::CPose3DQuat &) const --> double", pybind11::arg("x"));
		cl.def("assign", (class mrpt::poses::CPose3DQuatPDFGaussianInf & (mrpt::poses::CPose3DQuatPDFGaussianInf::*)(const class mrpt::poses::CPose3DQuatPDFGaussianInf &)) &mrpt::poses::CPose3DQuatPDFGaussianInf::operator=, "C++: mrpt::poses::CPose3DQuatPDFGaussianInf::operator=(const class mrpt::poses::CPose3DQuatPDFGaussianInf &) --> class mrpt::poses::CPose3DQuatPDFGaussianInf &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPose3DQuatPDFGaussianInf const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );

		cl.def("__add__", [](const mrpt::poses::CPose3DQuatPDFGaussianInf&a, const mrpt::poses::CPose3DQuatPDFGaussianInf& b) -> mrpt::poses::CPose3DQuatPDFGaussianInf { return a+b; });
		cl.def("__sub__", [](const mrpt::poses::CPose3DQuatPDFGaussianInf&a, const mrpt::poses::CPose3DQuatPDFGaussianInf& b) -> mrpt::poses::CPose3DQuatPDFGaussianInf { return a-b; });
	}
	{ // mrpt::poses::CPosePDFGaussianInf file:mrpt/poses/CPosePDFGaussianInf.h line:33
		pybind11::class_<mrpt::poses::CPosePDFGaussianInf, std::shared_ptr<mrpt::poses::CPosePDFGaussianInf>, PyCallBack_mrpt_poses_CPosePDFGaussianInf, mrpt::poses::CPosePDF> cl(M("mrpt::poses"), "CPosePDFGaussianInf", "A Probability Density  function (PDF) of a 2D pose \n\n\n as a Gaussian with a mean and the inverse of the covariance.\n\n   This class implements a PDF as a mono-modal Gaussian distribution in its\n information form, that is,\n     keeping the inverse of the covariance matrix instead of the covariance\n matrix itself.\n\n  This class is the dual of CPosePDFGaussian.\n\n \n CPose2D, CPosePDF, CPosePDFParticles\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPosePDFGaussianInf(); }, [](){ return new PyCallBack_mrpt_poses_CPosePDFGaussianInf(); } ) );
		cl.def( pybind11::init<const class mrpt::poses::CPose2D &>(), pybind11::arg("init_Mean") );

		cl.def( pybind11::init<const class mrpt::poses::CPose2D &, const class mrpt::math::CMatrixFixed<double, 3, 3> &>(), pybind11::arg("init_Mean"), pybind11::arg("init_CovInv") );

		cl.def( pybind11::init<const class mrpt::poses::CPosePDF &>(), pybind11::arg("o") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DPDF &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPosePDFGaussianInf const &o){ return new PyCallBack_mrpt_poses_CPosePDFGaussianInf(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPosePDFGaussianInf const &o){ return new mrpt::poses::CPosePDFGaussianInf(o); } ) );
		cl.def_readwrite("mean", &mrpt::poses::CPosePDFGaussianInf::mean);
		cl.def_readwrite("cov_inv", &mrpt::poses::CPosePDFGaussianInf::cov_inv);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPosePDFGaussianInf::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPosePDFGaussianInf::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPosePDFGaussianInf::*)() const) &mrpt::poses::CPosePDFGaussianInf::GetRuntimeClass, "C++: mrpt::poses::CPosePDFGaussianInf::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPosePDFGaussianInf::*)() const) &mrpt::poses::CPosePDFGaussianInf::clone, "C++: mrpt::poses::CPosePDFGaussianInf::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPosePDFGaussianInf::CreateObject, "C++: mrpt::poses::CPosePDFGaussianInf::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getPoseMean", (class mrpt::poses::CPose2D & (mrpt::poses::CPosePDFGaussianInf::*)()) &mrpt::poses::CPosePDFGaussianInf::getPoseMean, "C++: mrpt::poses::CPosePDFGaussianInf::getPoseMean() --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic);
		cl.def("getMean", (void (mrpt::poses::CPosePDFGaussianInf::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGaussianInf::getMean, "Returns an estimate of the pose, (the mean, or mathematical expectation\n of the PDF).\n \n\n getCovariance \n\nC++: mrpt::poses::CPosePDFGaussianInf::getMean(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("mean_pose"));
		cl.def("isInfType", (bool (mrpt::poses::CPosePDFGaussianInf::*)() const) &mrpt::poses::CPosePDFGaussianInf::isInfType, "C++: mrpt::poses::CPosePDFGaussianInf::isInfType() const --> bool");
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D> (mrpt::poses::CPosePDFGaussianInf::*)() const) &mrpt::poses::CPosePDFGaussianInf::getCovarianceAndMean, "C++: mrpt::poses::CPosePDFGaussianInf::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>");
		cl.def("getInformationMatrix", (void (mrpt::poses::CPosePDFGaussianInf::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::poses::CPosePDFGaussianInf::getInformationMatrix, "Returns the information (inverse covariance) matrix (a STATE_LEN x\n STATE_LEN matrix) \n\n getMean, getCovarianceAndMean \n\nC++: mrpt::poses::CPosePDFGaussianInf::getInformationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> void", pybind11::arg("inf"));
		cl.def("copyFrom", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPosePDFGaussianInf::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPosePDFGaussianInf::copyFrom(const class mrpt::poses::CPosePDF &) --> void", pybind11::arg("o"));
		cl.def("copyFrom", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPosePDFGaussianInf::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPosePDFGaussianInf::copyFrom(const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPosePDFGaussianInf::*)(const std::string &) const) &mrpt::poses::CPosePDFGaussianInf::saveToTextFile, "Save PDF's particles to a text file, containing the 2D pose in the first\n line, then the covariance matrix in next 3 lines. \n\nC++: mrpt::poses::CPosePDFGaussianInf::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPosePDFGaussianInf::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object \n\nC++: mrpt::poses::CPosePDFGaussianInf::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::CPosePDFGaussianInf::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPosePDFGaussianInf::changeCoordinatesReference(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("rotateCov", (void (mrpt::poses::CPosePDFGaussianInf::*)(const double)) &mrpt::poses::CPosePDFGaussianInf::rotateCov, "Rotate the covariance matrix by replacing it by \n\n\n, where \n\n\n\n. \n\nC++: mrpt::poses::CPosePDFGaussianInf::rotateCov(const double) --> void", pybind11::arg("ang"));
		cl.def("inverseComposition", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPosePDFGaussianInf &, const class mrpt::poses::CPosePDFGaussianInf &)) &mrpt::poses::CPosePDFGaussianInf::inverseComposition, "Set \n , computing the mean using the \"-\"\n operator and the covariances through the corresponding Jacobians (For\n 'x0' and 'x1' being independent variables!).  \n\nC++: mrpt::poses::CPosePDFGaussianInf::inverseComposition(const class mrpt::poses::CPosePDFGaussianInf &, const class mrpt::poses::CPosePDFGaussianInf &) --> void", pybind11::arg("x"), pybind11::arg("ref"));
		cl.def("inverseComposition", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPosePDFGaussianInf &, const class mrpt::poses::CPosePDFGaussianInf &, const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::poses::CPosePDFGaussianInf::inverseComposition, "Set \n , computing the mean using the \"-\"\n operator and the covariances through the corresponding Jacobians (Given\n the 3x3 cross-covariance matrix of variables x0 and x1). \n\nC++: mrpt::poses::CPosePDFGaussianInf::inverseComposition(const class mrpt::poses::CPosePDFGaussianInf &, const class mrpt::poses::CPosePDFGaussianInf &, const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("x1"), pybind11::arg("x0"), pybind11::arg("COV_01"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPosePDFGaussianInf::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGaussianInf::drawSingleSample, "Draws a single sample from the distribution \n\nC++: mrpt::poses::CPosePDFGaussianInf::drawSingleSample(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("outPart"));
		cl.def("bayesianFusion", [](mrpt::poses::CPosePDFGaussianInf &o, const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double)) &mrpt::poses::CPosePDFGaussianInf::bayesianFusion, "Bayesian fusion of two points gauss. distributions, then save the result\nin this object.\n  The process is as follows:\n		- (x1,S1): Mean and variance of the p1 distribution.\n		- (x2,S2): Mean and variance of the p2 distribution.\n		- (x,S): Mean and variance of the resulting distribution.\n\n    \n\n    \n\n   \n\nC++: mrpt::poses::CPosePDFGaussianInf::bayesianFusion(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("inverse", (void (mrpt::poses::CPosePDFGaussianInf::*)(class mrpt::poses::CPosePDF &) const) &mrpt::poses::CPosePDFGaussianInf::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPosePDFGaussianInf::inverse(class mrpt::poses::CPosePDF &) const --> void", pybind11::arg("o"));
		cl.def("__iadd__", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::CPosePDFGaussianInf::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated). \n\nC++: mrpt::poses::CPosePDFGaussianInf::operator+=(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("Ap"));
		cl.def("evaluatePDF", (double (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGaussianInf::evaluatePDF, "Evaluates the PDF at a given point \n\nC++: mrpt::poses::CPosePDFGaussianInf::evaluatePDF(const class mrpt::poses::CPose2D &) const --> double", pybind11::arg("x"));
		cl.def("evaluateNormalizedPDF", (double (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGaussianInf::evaluateNormalizedPDF, "Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in\n the range [0,1]. \n\nC++: mrpt::poses::CPosePDFGaussianInf::evaluateNormalizedPDF(const class mrpt::poses::CPose2D &) const --> double", pybind11::arg("x"));
		cl.def("mahalanobisDistanceTo", (double (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPosePDFGaussianInf &)) &mrpt::poses::CPosePDFGaussianInf::mahalanobisDistanceTo, "Computes the Mahalanobis distance between the centers of two Gaussians.\n\nC++: mrpt::poses::CPosePDFGaussianInf::mahalanobisDistanceTo(const class mrpt::poses::CPosePDFGaussianInf &) --> double", pybind11::arg("theOther"));
		cl.def("__iadd__", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPosePDFGaussianInf &)) &mrpt::poses::CPosePDFGaussianInf::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated) (see formulas in\n jacobiansPoseComposition ). \n\nC++: mrpt::poses::CPosePDFGaussianInf::operator+=(const class mrpt::poses::CPosePDFGaussianInf &) --> void", pybind11::arg("Ap"));
		cl.def("__isub__", (void (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPosePDFGaussianInf &)) &mrpt::poses::CPosePDFGaussianInf::operator-=, "Makes: thisPDF = thisPDF - Ap, where \"-\" is pose inverse composition\n (both the mean, and the covariance matrix are updated) \n\nC++: mrpt::poses::CPosePDFGaussianInf::operator-=(const class mrpt::poses::CPosePDFGaussianInf &) --> void", pybind11::arg("ref"));
		cl.def("assign", (class mrpt::poses::CPosePDFGaussianInf & (mrpt::poses::CPosePDFGaussianInf::*)(const class mrpt::poses::CPosePDFGaussianInf &)) &mrpt::poses::CPosePDFGaussianInf::operator=, "C++: mrpt::poses::CPosePDFGaussianInf::operator=(const class mrpt::poses::CPosePDFGaussianInf &) --> class mrpt::poses::CPosePDFGaussianInf &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPosePDFGaussianInf const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );

		cl.def("__add__", [](const mrpt::poses::CPosePDFGaussianInf&a, const mrpt::poses::CPosePDFGaussianInf& b) -> mrpt::poses::CPosePDFGaussianInf { return a+b; });
		cl.def("__sub__", [](const mrpt::poses::CPosePDFGaussianInf&a, const mrpt::poses::CPosePDFGaussianInf& b) -> mrpt::poses::CPosePDFGaussianInf { return a-b; });
	}
}
