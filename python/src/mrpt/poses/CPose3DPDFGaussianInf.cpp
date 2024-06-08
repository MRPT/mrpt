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
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGrid.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
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

// mrpt::poses::CPose3DPDFGaussianInf file:mrpt/poses/CPose3DPDFGaussianInf.h line:37
struct PyCallBack_mrpt_poses_CPose3DPDFGaussianInf : public mrpt::poses::CPose3DPDFGaussianInf {
	using mrpt::poses::CPose3DPDFGaussianInf::CPose3DPDFGaussianInf;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DPDFGaussianInf::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3DPDFGaussianInf::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3DPDFGaussianInf::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussianInf::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussianInf::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussianInf::getMean(a0);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "isInfType");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPose3DPDFGaussianInf::isInfType();
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPose3DPDFGaussianInf::getCovarianceAndMean();
	}
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 6, 6> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "getInformationMatrix");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussianInf::getInformationMatrix(a0);
	}
	void copyFrom(const class mrpt::poses::CPose3DPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussianInf::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPose3DPDFGaussianInf::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussianInf::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussianInf::drawSingleSample(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPose3DPDF & a0, const class mrpt::poses::CPose3DPDF & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussianInf::bayesianFusion(a0, a1);
	}
	void inverse(class mrpt::poses::CPose3DPDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussianInf *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussianInf::inverse(a0);
	}
};

// mrpt::poses::CPose3DPDFGrid file:mrpt/poses/CPose3DPDFGrid.h line:25
struct PyCallBack_mrpt_poses_CPose3DPDFGrid : public mrpt::poses::CPose3DPDFGrid {
	using mrpt::poses::CPose3DPDFGrid::CPose3DPDFGrid;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DPDFGrid::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3DPDFGrid::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3DPDFGrid::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGrid::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGrid::serializeFrom(a0, a1);
	}
	void copyFrom(const class mrpt::poses::CPose3DPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGrid::copyFrom(a0);
	}
	void getMean(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGrid::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPose3DPDFGrid::getCovarianceAndMean();
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPose3DPDFGrid::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGrid::changeCoordinatesReference(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPose3DPDF & a0, const class mrpt::poses::CPose3DPDF & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGrid::bayesianFusion(a0, a1);
	}
	void inverse(class mrpt::poses::CPose3DPDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGrid::inverse(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGrid::drawSingleSample(a0);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "isInfType");
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
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 6, 6> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGrid *>(this), "getInformationMatrix");
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

void bind_mrpt_poses_CPose3DPDFGaussianInf(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose3DPDFGaussianInf file:mrpt/poses/CPose3DPDFGaussianInf.h line:37
		pybind11::class_<mrpt::poses::CPose3DPDFGaussianInf, std::shared_ptr<mrpt::poses::CPose3DPDFGaussianInf>, PyCallBack_mrpt_poses_CPose3DPDFGaussianInf, mrpt::poses::CPose3DPDF> cl(M("mrpt::poses"), "CPose3DPDFGaussianInf", "Declares a class that represents a Probability Density function (PDF) of a\n 3D pose \n\n as a\n Gaussian described by its mean and its inverse covariance matrix.\n\n   This class implements that PDF using a mono-modal Gaussian distribution in\n \"information\" form (inverse covariance matrix).\n\n  Uncertainty of pose composition operations (\n) is\n implemented in the method \"CPose3DPDFGaussianInf::operator+=\".\n\n \n Read also: \"A tutorial on SE(3) transformation parameterizations and\n on-manifold optimization\", in \n\n \n CPose3D, CPose3DPDF, CPose3DPDFParticles, CPose3DPDFGaussian\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DPDFGaussianInf(); }, [](){ return new PyCallBack_mrpt_poses_CPose3DPDFGaussianInf(); } ) );
		cl.def( pybind11::init<const class mrpt::poses::CPose3D &>(), pybind11::arg("init_Mean") );

		cl.def( pybind11::init<enum mrpt::poses::TConstructorFlags_Poses>(), pybind11::arg("constructor_dummy_param") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3D &, const class mrpt::math::CMatrixFixed<double, 6, 6> &>(), pybind11::arg("init_Mean"), pybind11::arg("init_CovInv") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DQuatPDFGaussian &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3DPDFGaussianInf const &o){ return new PyCallBack_mrpt_poses_CPose3DPDFGaussianInf(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3DPDFGaussianInf const &o){ return new mrpt::poses::CPose3DPDFGaussianInf(o); } ) );
		cl.def_readwrite("mean", &mrpt::poses::CPose3DPDFGaussianInf::mean);
		cl.def_readwrite("cov_inv", &mrpt::poses::CPose3DPDFGaussianInf::cov_inv);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DPDFGaussianInf::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DPDFGaussianInf::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DPDFGaussianInf::*)() const) &mrpt::poses::CPose3DPDFGaussianInf::GetRuntimeClass, "C++: mrpt::poses::CPose3DPDFGaussianInf::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3DPDFGaussianInf::*)() const) &mrpt::poses::CPose3DPDFGaussianInf::clone, "C++: mrpt::poses::CPose3DPDFGaussianInf::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3DPDFGaussianInf::CreateObject, "C++: mrpt::poses::CPose3DPDFGaussianInf::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getPoseMean", (class mrpt::poses::CPose3D & (mrpt::poses::CPose3DPDFGaussianInf::*)()) &mrpt::poses::CPose3DPDFGaussianInf::getPoseMean, "C++: mrpt::poses::CPose3DPDFGaussianInf::getPoseMean() --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic);
		cl.def("getMean", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGaussianInf::getMean, "C++: mrpt::poses::CPose3DPDFGaussianInf::getMean(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("mean_pose"));
		cl.def("isInfType", (bool (mrpt::poses::CPose3DPDFGaussianInf::*)() const) &mrpt::poses::CPose3DPDFGaussianInf::isInfType, "C++: mrpt::poses::CPose3DPDFGaussianInf::isInfType() const --> bool");
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D> (mrpt::poses::CPose3DPDFGaussianInf::*)() const) &mrpt::poses::CPose3DPDFGaussianInf::getCovarianceAndMean, "C++: mrpt::poses::CPose3DPDFGaussianInf::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>");
		cl.def("getInformationMatrix", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(class mrpt::math::CMatrixFixed<double, 6, 6> &) const) &mrpt::poses::CPose3DPDFGaussianInf::getInformationMatrix, "Returns the information (inverse covariance) matrix (a STATE_LEN x\n STATE_LEN matrix) \n\n getMean, getCovarianceAndMean \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::getInformationMatrix(class mrpt::math::CMatrixFixed<double, 6, 6> &) const --> void", pybind11::arg("inf"));
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFGaussianInf::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::copyFrom(const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("o"));
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPose3DPDFGaussianInf::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::copyFrom(const class mrpt::poses::CPosePDF &) --> void", pybind11::arg("o"));
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3DQuatPDFGaussian &)) &mrpt::poses::CPose3DPDFGaussianInf::copyFrom, "Copy from a 6D pose PDF described as a Quaternion\n\nC++: mrpt::poses::CPose3DPDFGaussianInf::copyFrom(const class mrpt::poses::CPose3DQuatPDFGaussian &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPose3DPDFGaussianInf::*)(const std::string &) const) &mrpt::poses::CPose3DPDFGaussianInf::saveToTextFile, "Save the PDF to a text file, containing the 3D pose in the first line,\n then the covariance matrix in next 3 lines. \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DPDFGaussianInf::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGaussianInf::drawSingleSample, "Draws a single sample from the distribution \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::drawSingleSample(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("outPart"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFGaussianInf::bayesianFusion, "Bayesian fusion of two points gauss. distributions, then save the result\nin this object.\n  The process is as follows:\n		- (x1,S1): Mean and variance of the p1 distribution.\n		- (x2,S2): Mean and variance of the p2 distribution.\n		- (x,S): Mean and variance of the resulting distribution.\n\n    \n\n    \n\n   \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::bayesianFusion(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("inverse", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(class mrpt::poses::CPose3DPDF &) const) &mrpt::poses::CPose3DPDFGaussianInf::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::inverse(class mrpt::poses::CPose3DPDF &) const --> void", pybind11::arg("o"));
		cl.def("__neg__", (class mrpt::poses::CPose3DPDFGaussianInf (mrpt::poses::CPose3DPDFGaussianInf::*)() const) &mrpt::poses::CPose3DPDFGaussianInf::operator-, "Unary - operator, returns the PDF of the inverse pose.  \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::operator-() const --> class mrpt::poses::CPose3DPDFGaussianInf");
		cl.def("__iadd__", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DPDFGaussianInf::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated) \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::operator+=(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("Ap"));
		cl.def("__iadd__", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3DPDFGaussianInf &)) &mrpt::poses::CPose3DPDFGaussianInf::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated) \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::operator+=(const class mrpt::poses::CPose3DPDFGaussianInf &) --> void", pybind11::arg("Ap"));
		cl.def("__isub__", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3DPDFGaussianInf &)) &mrpt::poses::CPose3DPDFGaussianInf::operator-=, "Makes: thisPDF = thisPDF - Ap, where \"-\" is pose inverse composition\n (both the mean, and the covariance matrix are updated) \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::operator-=(const class mrpt::poses::CPose3DPDFGaussianInf &) --> void", pybind11::arg("Ap"));
		cl.def("evaluatePDF", (double (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGaussianInf::evaluatePDF, "Evaluates the PDF at a given point \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::evaluatePDF(const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("x"));
		cl.def("evaluateNormalizedPDF", (double (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGaussianInf::evaluateNormalizedPDF, "Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in\n the range [0,1] \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::evaluateNormalizedPDF(const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("x"));
		cl.def("getInvCovSubmatrix2D", (void (mrpt::poses::CPose3DPDFGaussianInf::*)(class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::poses::CPose3DPDFGaussianInf::getInvCovSubmatrix2D, "Returns a 3x3 matrix with submatrix of the inverse covariance for the\n variables (x,y,yaw) only \n\nC++: mrpt::poses::CPose3DPDFGaussianInf::getInvCovSubmatrix2D(class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("out_cov"));
		cl.def("mahalanobisDistanceTo", (double (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3DPDFGaussianInf &)) &mrpt::poses::CPose3DPDFGaussianInf::mahalanobisDistanceTo, "Computes the Mahalanobis distance between the centers of two Gaussians.\n  The variables with a variance exactly equal to 0 are not taken into\n account in the process, but\n   \"infinity\" is returned if the corresponding elements are not exactly\n equal.\n\nC++: mrpt::poses::CPose3DPDFGaussianInf::mahalanobisDistanceTo(const class mrpt::poses::CPose3DPDFGaussianInf &) --> double", pybind11::arg("theOther"));
		cl.def("assign", (class mrpt::poses::CPose3DPDFGaussianInf & (mrpt::poses::CPose3DPDFGaussianInf::*)(const class mrpt::poses::CPose3DPDFGaussianInf &)) &mrpt::poses::CPose3DPDFGaussianInf::operator=, "C++: mrpt::poses::CPose3DPDFGaussianInf::operator=(const class mrpt::poses::CPose3DPDFGaussianInf &) --> class mrpt::poses::CPose3DPDFGaussianInf &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPose3DPDFGaussianInf const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );

		cl.def("__add__", [](const mrpt::poses::CPose3DPDFGaussianInf&a, const mrpt::poses::CPose3DPDFGaussianInf& b) -> mrpt::poses::CPose3DPDFGaussianInf { return a+b; });
		cl.def("__sub__", [](const mrpt::poses::CPose3DPDFGaussianInf&a, const mrpt::poses::CPose3DPDFGaussianInf& b) -> mrpt::poses::CPose3DPDFGaussianInf { return a-b; });
	}
	{ // mrpt::poses::CPose3DPDFGrid file:mrpt/poses/CPose3DPDFGrid.h line:25
		pybind11::class_<mrpt::poses::CPose3DPDFGrid, std::shared_ptr<mrpt::poses::CPose3DPDFGrid>, PyCallBack_mrpt_poses_CPose3DPDFGrid, mrpt::poses::CPose3DPDF, mrpt::poses::CPose3DGridTemplate<double>> cl(M("mrpt::poses"), "CPose3DPDFGrid", "Declares a class that represents a Probability Distribution\n  function (PDF) of a SE(3) pose (x,y,z, yaw, pitch, roll), in\n the form of a 6-dimensional grid of \"voxels\".\n\n \n CPose3D, CPose3DPDF, CPose3DGridTemplate\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DPDFGrid(); }, [](){ return new PyCallBack_mrpt_poses_CPose3DPDFGrid(); } ), "doc");
		cl.def( pybind11::init( [](const struct mrpt::math::TPose3D & a0){ return new mrpt::poses::CPose3DPDFGrid(a0); }, [](const struct mrpt::math::TPose3D & a0){ return new PyCallBack_mrpt_poses_CPose3DPDFGrid(a0); } ), "doc");
		cl.def( pybind11::init( [](const struct mrpt::math::TPose3D & a0, const struct mrpt::math::TPose3D & a1){ return new mrpt::poses::CPose3DPDFGrid(a0, a1); }, [](const struct mrpt::math::TPose3D & a0, const struct mrpt::math::TPose3D & a1){ return new PyCallBack_mrpt_poses_CPose3DPDFGrid(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](const struct mrpt::math::TPose3D & a0, const struct mrpt::math::TPose3D & a1, double const & a2){ return new mrpt::poses::CPose3DPDFGrid(a0, a1, a2); }, [](const struct mrpt::math::TPose3D & a0, const struct mrpt::math::TPose3D & a1, double const & a2){ return new PyCallBack_mrpt_poses_CPose3DPDFGrid(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init<const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, double, double>(), pybind11::arg("bb_min"), pybind11::arg("bb_max"), pybind11::arg("resolution_XYZ"), pybind11::arg("resolution_YPR") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3DPDFGrid const &o){ return new PyCallBack_mrpt_poses_CPose3DPDFGrid(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3DPDFGrid const &o){ return new mrpt::poses::CPose3DPDFGrid(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DPDFGrid::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DPDFGrid::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DPDFGrid::*)() const) &mrpt::poses::CPose3DPDFGrid::GetRuntimeClass, "C++: mrpt::poses::CPose3DPDFGrid::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3DPDFGrid::*)() const) &mrpt::poses::CPose3DPDFGrid::clone, "C++: mrpt::poses::CPose3DPDFGrid::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3DPDFGrid::CreateObject, "C++: mrpt::poses::CPose3DPDFGrid::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDFGrid::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFGrid::copyFrom, "Copy operator, translating if necesary (for example, between\n particles and gaussian representations) \n\nC++: mrpt::poses::CPose3DPDFGrid::copyFrom(const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("o"));
		cl.def("normalize", (void (mrpt::poses::CPose3DPDFGrid::*)()) &mrpt::poses::CPose3DPDFGrid::normalize, "Normalizes the PDF, such as all voxels sum the unity. \n\nC++: mrpt::poses::CPose3DPDFGrid::normalize() --> void");
		cl.def("uniformDistribution", (void (mrpt::poses::CPose3DPDFGrid::*)()) &mrpt::poses::CPose3DPDFGrid::uniformDistribution, "Assigns the same value to all the cells in the grid, so the sum 1 \n\nC++: mrpt::poses::CPose3DPDFGrid::uniformDistribution() --> void");
		cl.def("getMean", (void (mrpt::poses::CPose3DPDFGrid::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGrid::getMean, "C++: mrpt::poses::CPose3DPDFGrid::getMean(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("mean_pose"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D> (mrpt::poses::CPose3DPDFGrid::*)() const) &mrpt::poses::CPose3DPDFGrid::getCovarianceAndMean, "C++: mrpt::poses::CPose3DPDFGrid::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>");
		cl.def("saveToTextFile", (bool (mrpt::poses::CPose3DPDFGrid::*)(const std::string &) const) &mrpt::poses::CPose3DPDFGrid::saveToTextFile, "Save the contents of the 3D grid in one file, as a concatenation of\n (X,Y) slices. The size in X,Y,and the values for Z,yaw, pitch, roll, PHI\n are stored in another file named `<filename>_dims.txt`\n \n\n false on error \n\nC++: mrpt::poses::CPose3DPDFGrid::saveToTextFile(const std::string &) const --> bool", pybind11::arg("dataFile"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DPDFGrid::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DPDFGrid::changeCoordinatesReference, "C++: mrpt::poses::CPose3DPDFGrid::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPose3DPDFGrid::*)(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFGrid::bayesianFusion, "C++: mrpt::poses::CPose3DPDFGrid::bayesianFusion(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("inverse", (void (mrpt::poses::CPose3DPDFGrid::*)(class mrpt::poses::CPose3DPDF &) const) &mrpt::poses::CPose3DPDFGrid::inverse, "C++: mrpt::poses::CPose3DPDFGrid::inverse(class mrpt::poses::CPose3DPDF &) const --> void", pybind11::arg("o"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPose3DPDFGrid::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGrid::drawSingleSample, "Draws a single sample from the distribution.\n \n\n Precondition: voxel weights are assumed to be normalized. \n\nC++: mrpt::poses::CPose3DPDFGrid::drawSingleSample(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("outPart"));
		cl.def("assign", (class mrpt::poses::CPose3DPDFGrid & (mrpt::poses::CPose3DPDFGrid::*)(const class mrpt::poses::CPose3DPDFGrid &)) &mrpt::poses::CPose3DPDFGrid::operator=, "C++: mrpt::poses::CPose3DPDFGrid::operator=(const class mrpt::poses::CPose3DPDFGrid &) --> class mrpt::poses::CPose3DPDFGrid &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
