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
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/poses/CPosePDFSOG.h>
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

// mrpt::poses::CPosePDFGrid file:mrpt/poses/CPosePDFGrid.h line:24
struct PyCallBack_mrpt_poses_CPosePDFGrid : public mrpt::poses::CPosePDFGrid {
	using mrpt::poses::CPosePDFGrid::CPosePDFGrid;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPosePDFGrid::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPosePDFGrid::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPosePDFGrid::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGrid::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGrid::serializeFrom(a0, a1);
	}
	void copyFrom(const class mrpt::poses::CPosePDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGrid::copyFrom(a0);
	}
	void getMean(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGrid::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPosePDFGrid::getCovarianceAndMean();
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPosePDFGrid::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGrid::changeCoordinatesReference(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGrid::bayesianFusion(a0, a1, a2);
	}
	void inverse(class mrpt::poses::CPosePDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGrid::inverse(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFGrid::drawSingleSample(a0);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFGrid *>(this), "getInformationMatrix");
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

// mrpt::poses::CPosePDFSOG file:mrpt/poses/CPosePDFSOG.h line:35
struct PyCallBack_mrpt_poses_CPosePDFSOG : public mrpt::poses::CPosePDFSOG {
	using mrpt::poses::CPosePDFSOG::CPosePDFSOG;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPosePDFSOG::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPosePDFSOG::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPosePDFSOG::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFSOG::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFSOG::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFSOG::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPosePDFSOG::getCovarianceAndMean();
	}
	void copyFrom(const class mrpt::poses::CPosePDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFSOG::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPosePDFSOG::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFSOG::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFSOG::drawSingleSample(a0);
	}
	void inverse(class mrpt::poses::CPosePDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFSOG::inverse(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFSOG::bayesianFusion(a0, a1, a2);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPosePDFSOG *>(this), "getInformationMatrix");
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

void bind_mrpt_poses_CPosePDFGrid(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPosePDFGrid file:mrpt/poses/CPosePDFGrid.h line:24
		pybind11::class_<mrpt::poses::CPosePDFGrid, std::shared_ptr<mrpt::poses::CPosePDFGrid>, PyCallBack_mrpt_poses_CPosePDFGrid, mrpt::poses::CPosePDF, mrpt::poses::CPose2DGridTemplate<double>> cl(M("mrpt::poses"), "CPosePDFGrid", "Declares a class that represents a Probability Distribution\n    function (PDF) of a 2D pose (x,y,phi).\n   This class implements that PDF using a 3D grid.\n\n \n CPose2D, CPosePDF, CPose2DGridTemplate\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPosePDFGrid(); }, [](){ return new PyCallBack_mrpt_poses_CPosePDFGrid(); } ), "doc");
		cl.def( pybind11::init( [](double const & a0){ return new mrpt::poses::CPosePDFGrid(a0); }, [](double const & a0){ return new PyCallBack_mrpt_poses_CPosePDFGrid(a0); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1){ return new mrpt::poses::CPosePDFGrid(a0, a1); }, [](double const & a0, double const & a1){ return new PyCallBack_mrpt_poses_CPosePDFGrid(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2){ return new mrpt::poses::CPosePDFGrid(a0, a1, a2); }, [](double const & a0, double const & a1, double const & a2){ return new PyCallBack_mrpt_poses_CPosePDFGrid(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3){ return new mrpt::poses::CPosePDFGrid(a0, a1, a2, a3); }, [](double const & a0, double const & a1, double const & a2, double const & a3){ return new PyCallBack_mrpt_poses_CPosePDFGrid(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new mrpt::poses::CPosePDFGrid(a0, a1, a2, a3, a4); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4){ return new PyCallBack_mrpt_poses_CPosePDFGrid(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new mrpt::poses::CPosePDFGrid(a0, a1, a2, a3, a4, a5); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5){ return new PyCallBack_mrpt_poses_CPosePDFGrid(a0, a1, a2, a3, a4, a5); } ), "doc");
		cl.def( pybind11::init( [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6){ return new mrpt::poses::CPosePDFGrid(a0, a1, a2, a3, a4, a5, a6); }, [](double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, double const & a6){ return new PyCallBack_mrpt_poses_CPosePDFGrid(a0, a1, a2, a3, a4, a5, a6); } ), "doc");
		cl.def( pybind11::init<double, double, double, double, double, double, double, double>(), pybind11::arg("xMin"), pybind11::arg("xMax"), pybind11::arg("yMin"), pybind11::arg("yMax"), pybind11::arg("resolutionXY"), pybind11::arg("resolutionPhi"), pybind11::arg("phiMin"), pybind11::arg("phiMax") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPosePDFGrid const &o){ return new PyCallBack_mrpt_poses_CPosePDFGrid(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPosePDFGrid const &o){ return new mrpt::poses::CPosePDFGrid(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPosePDFGrid::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPosePDFGrid::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPosePDFGrid::*)() const) &mrpt::poses::CPosePDFGrid::GetRuntimeClass, "C++: mrpt::poses::CPosePDFGrid::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPosePDFGrid::*)() const) &mrpt::poses::CPosePDFGrid::clone, "C++: mrpt::poses::CPosePDFGrid::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPosePDFGrid::CreateObject, "C++: mrpt::poses::CPosePDFGrid::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("copyFrom", (void (mrpt::poses::CPosePDFGrid::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPosePDFGrid::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPosePDFGrid::copyFrom(const class mrpt::poses::CPosePDF &) --> void", pybind11::arg("o"));
		cl.def("normalize", (void (mrpt::poses::CPosePDFGrid::*)()) &mrpt::poses::CPosePDFGrid::normalize, "Normalizes the PDF, such as all cells sum the unity. \n\nC++: mrpt::poses::CPosePDFGrid::normalize() --> void");
		cl.def("uniformDistribution", (void (mrpt::poses::CPosePDFGrid::*)()) &mrpt::poses::CPosePDFGrid::uniformDistribution, "Assigns the same value to all the cells in the grid, so the sum 1. \n\nC++: mrpt::poses::CPosePDFGrid::uniformDistribution() --> void");
		cl.def("getMean", (void (mrpt::poses::CPosePDFGrid::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGrid::getMean, "C++: mrpt::poses::CPosePDFGrid::getMean(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("mean_pose"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D> (mrpt::poses::CPosePDFGrid::*)() const) &mrpt::poses::CPosePDFGrid::getCovarianceAndMean, "C++: mrpt::poses::CPosePDFGrid::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>");
		cl.def("saveToTextFile", (bool (mrpt::poses::CPosePDFGrid::*)(const std::string &) const) &mrpt::poses::CPosePDFGrid::saveToTextFile, "Save the contents of the 3D grid in one file, as a vertical\n concatenation of rectangular matrix for the different \"PHI\" discrete\n levels, and the size in X,Y,and PHI in another file named\n \"<filename>_dims.txt\". \n\n false on error \n\nC++: mrpt::poses::CPosePDFGrid::saveToTextFile(const std::string &) const --> bool", pybind11::arg("dataFile"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPosePDFGrid::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPosePDFGrid::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPosePDFGrid::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("bayesianFusion", [](mrpt::poses::CPosePDFGrid &o, const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPosePDFGrid::*)(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double)) &mrpt::poses::CPosePDFGrid::bayesianFusion, "Bayesian fusion of 2 densities (In the grid representation this becomes\n a pointwise multiplication) \n\nC++: mrpt::poses::CPosePDFGrid::bayesianFusion(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("inverse", (void (mrpt::poses::CPosePDFGrid::*)(class mrpt::poses::CPosePDF &) const) &mrpt::poses::CPosePDFGrid::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPosePDFGrid::inverse(class mrpt::poses::CPosePDF &) const --> void", pybind11::arg("o"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPosePDFGrid::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFGrid::drawSingleSample, "Draws a single sample from the distribution (WARNING: weights are\n assumed to be normalized!) \n\nC++: mrpt::poses::CPosePDFGrid::drawSingleSample(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("outPart"));
		cl.def("assign", (class mrpt::poses::CPosePDFGrid & (mrpt::poses::CPosePDFGrid::*)(const class mrpt::poses::CPosePDFGrid &)) &mrpt::poses::CPosePDFGrid::operator=, "C++: mrpt::poses::CPosePDFGrid::operator=(const class mrpt::poses::CPosePDFGrid &) --> class mrpt::poses::CPosePDFGrid &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPosePDFSOG file:mrpt/poses/CPosePDFSOG.h line:35
		pybind11::class_<mrpt::poses::CPosePDFSOG, std::shared_ptr<mrpt::poses::CPosePDFSOG>, PyCallBack_mrpt_poses_CPosePDFSOG, mrpt::poses::CPosePDF> cl(M("mrpt::poses"), "CPosePDFSOG", "Declares a class that represents a Probability Density  function (PDF) of a\n 2D pose \n\n.\n   This class implements that PDF as the following multi-modal Gaussian\n distribution:\n\n \n\n\n  Where the number of modes N is the size of CPosePDFSOG::m_modes\n\n  See mrpt::poses::CPosePDF for more details.\n\n \n CPose2D, CPosePDF, CPosePDFParticles\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPosePDFSOG(); }, [](){ return new PyCallBack_mrpt_poses_CPosePDFSOG(); } ), "doc");
		cl.def( pybind11::init<size_t>(), pybind11::arg("nModes") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPosePDFSOG const &o){ return new PyCallBack_mrpt_poses_CPosePDFSOG(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPosePDFSOG const &o){ return new mrpt::poses::CPosePDFSOG(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPosePDFSOG::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPosePDFSOG::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPosePDFSOG::*)() const) &mrpt::poses::CPosePDFSOG::GetRuntimeClass, "C++: mrpt::poses::CPosePDFSOG::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPosePDFSOG::*)() const) &mrpt::poses::CPosePDFSOG::clone, "C++: mrpt::poses::CPosePDFSOG::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPosePDFSOG::CreateObject, "C++: mrpt::poses::CPosePDFSOG::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("size", (size_t (mrpt::poses::CPosePDFSOG::*)() const) &mrpt::poses::CPosePDFSOG::size, "Return the number of Gaussian modes. \n\nC++: mrpt::poses::CPosePDFSOG::size() const --> size_t");
		cl.def("empty", (bool (mrpt::poses::CPosePDFSOG::*)() const) &mrpt::poses::CPosePDFSOG::empty, "Return whether there is any Gaussian mode. \n\nC++: mrpt::poses::CPosePDFSOG::empty() const --> bool");
		cl.def("clear", (void (mrpt::poses::CPosePDFSOG::*)()) &mrpt::poses::CPosePDFSOG::clear, "Clear the list of modes \n\nC++: mrpt::poses::CPosePDFSOG::clear() --> void");
		cl.def("__getitem__", (struct mrpt::poses::CPosePDFSOG::TGaussianMode & (mrpt::poses::CPosePDFSOG::*)(size_t)) &mrpt::poses::CPosePDFSOG::operator[], "Access to individual beacons \n\nC++: mrpt::poses::CPosePDFSOG::operator[](size_t) --> struct mrpt::poses::CPosePDFSOG::TGaussianMode &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("get", (struct mrpt::poses::CPosePDFSOG::TGaussianMode & (mrpt::poses::CPosePDFSOG::*)(size_t)) &mrpt::poses::CPosePDFSOG::get, "Access to individual beacons \n\nC++: mrpt::poses::CPosePDFSOG::get(size_t) --> struct mrpt::poses::CPosePDFSOG::TGaussianMode &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("push_back", (void (mrpt::poses::CPosePDFSOG::*)(const struct mrpt::poses::CPosePDFSOG::TGaussianMode &)) &mrpt::poses::CPosePDFSOG::push_back, "Inserts a copy of the given mode into the SOG \n\nC++: mrpt::poses::CPosePDFSOG::push_back(const struct mrpt::poses::CPosePDFSOG::TGaussianMode &) --> void", pybind11::arg("m"));
		cl.def("resize", (void (mrpt::poses::CPosePDFSOG::*)(size_t)) &mrpt::poses::CPosePDFSOG::resize, "Resize the number of SOG modes \n\nC++: mrpt::poses::CPosePDFSOG::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("mergeModes", [](mrpt::poses::CPosePDFSOG &o) -> void { return o.mergeModes(); }, "");
		cl.def("mergeModes", [](mrpt::poses::CPosePDFSOG &o, double const & a0) -> void { return o.mergeModes(a0); }, "", pybind11::arg("max_KLd"));
		cl.def("mergeModes", (void (mrpt::poses::CPosePDFSOG::*)(double, bool)) &mrpt::poses::CPosePDFSOG::mergeModes, "Merge very close modes so the overall number of modes is reduced while\n preserving the total distribution.\n  This method uses the approach described in the paper:\n  - \"Kullback-Leibler Approach to Gaussian Mixture Reduction\" AR\n Runnalls. IEEE Transactions on Aerospace and Electronic Systems, 2007.\n\n  \n The maximum KL-divergence to consider the merge of two\n nodes (and then stops the process).\n\nC++: mrpt::poses::CPosePDFSOG::mergeModes(double, bool) --> void", pybind11::arg("max_KLd"), pybind11::arg("verbose"));
		cl.def("getMean", (void (mrpt::poses::CPosePDFSOG::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFSOG::getMean, "C++: mrpt::poses::CPosePDFSOG::getMean(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("mean_pose"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D> (mrpt::poses::CPosePDFSOG::*)() const) &mrpt::poses::CPosePDFSOG::getCovarianceAndMean, "C++: mrpt::poses::CPosePDFSOG::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>");
		cl.def("getMostLikelyCovarianceAndMean", (void (mrpt::poses::CPosePDFSOG::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFSOG::getMostLikelyCovarianceAndMean, "For the most likely Gaussian mode in the SOG, returns the pose\n covariance matrix (3x3 cov matrix) and the mean. \n\n getMean \n\nC++: mrpt::poses::CPosePDFSOG::getMostLikelyCovarianceAndMean(class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::poses::CPose2D &) const --> void", pybind11::arg("cov"), pybind11::arg("mean_point"));
		cl.def("normalizeWeights", (void (mrpt::poses::CPosePDFSOG::*)()) &mrpt::poses::CPosePDFSOG::normalizeWeights, "Normalize the weights in m_modes such as the maximum log-weight is 0 \n\nC++: mrpt::poses::CPosePDFSOG::normalizeWeights() --> void");
		cl.def("copyFrom", (void (mrpt::poses::CPosePDFSOG::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPosePDFSOG::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPosePDFSOG::copyFrom(const class mrpt::poses::CPosePDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPosePDFSOG::*)(const std::string &) const) &mrpt::poses::CPosePDFSOG::saveToTextFile, "Save the density to a text file, with the following format:\n  There is one row per Gaussian \"mode\", and each row contains 10\n elements:\n   - w (The weight)\n   - x_mean (gaussian mean value)\n   - y_mean (gaussian mean value)\n   - phi_mean (gaussian mean value)\n   - C11 (Covariance elements)\n   - C22 (Covariance elements)\n   - C33 (Covariance elements)\n   - C12 (Covariance elements)\n   - C13 (Covariance elements)\n   - C23 (Covariance elements)\n\nC++: mrpt::poses::CPosePDFSOG::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPosePDFSOG::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPosePDFSOG::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPosePDFSOG::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("rotateAllCovariances", (void (mrpt::poses::CPosePDFSOG::*)(double)) &mrpt::poses::CPosePDFSOG::rotateAllCovariances, "Rotate all the covariance matrixes by replacing them by \n\n\n, where \n\n\n\n \n\nC++: mrpt::poses::CPosePDFSOG::rotateAllCovariances(double) --> void", pybind11::arg("ang"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPosePDFSOG::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFSOG::drawSingleSample, "Draws a single sample from the distribution \n\nC++: mrpt::poses::CPosePDFSOG::drawSingleSample(class mrpt::poses::CPose2D &) const --> void", pybind11::arg("outPart"));
		cl.def("inverse", (void (mrpt::poses::CPosePDFSOG::*)(class mrpt::poses::CPosePDF &) const) &mrpt::poses::CPosePDFSOG::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPosePDFSOG::inverse(class mrpt::poses::CPosePDF &) const --> void", pybind11::arg("o"));
		cl.def("__iadd__", (void (mrpt::poses::CPosePDFSOG::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::CPosePDFSOG::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated). \n\nC++: mrpt::poses::CPosePDFSOG::operator+=(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("Ap"));
		cl.def("evaluatePDF", [](mrpt::poses::CPosePDFSOG const &o, const class mrpt::poses::CPose2D & a0) -> double { return o.evaluatePDF(a0); }, "", pybind11::arg("x"));
		cl.def("evaluatePDF", (double (mrpt::poses::CPosePDFSOG::*)(const class mrpt::poses::CPose2D &, bool) const) &mrpt::poses::CPosePDFSOG::evaluatePDF, "Evaluates the PDF at a given point. \n\nC++: mrpt::poses::CPosePDFSOG::evaluatePDF(const class mrpt::poses::CPose2D &, bool) const --> double", pybind11::arg("x"), pybind11::arg("sumOverAllPhis"));
		cl.def("evaluateNormalizedPDF", (double (mrpt::poses::CPosePDFSOG::*)(const class mrpt::poses::CPose2D &) const) &mrpt::poses::CPosePDFSOG::evaluateNormalizedPDF, "Evaluates the ratio PDF(x) / max_PDF(x*), that is, the normalized PDF in\n the range [0,1]. \n\nC++: mrpt::poses::CPosePDFSOG::evaluateNormalizedPDF(const class mrpt::poses::CPose2D &) const --> double", pybind11::arg("x"));
		cl.def("evaluatePDFInArea", [](mrpt::poses::CPosePDFSOG &o, double const & a0, double const & a1, double const & a2, double const & a3, double const & a4, double const & a5, class mrpt::math::CMatrixDynamic<double> & a6) -> void { return o.evaluatePDFInArea(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolutionXY"), pybind11::arg("phi"), pybind11::arg("outMatrix"));
		cl.def("evaluatePDFInArea", (void (mrpt::poses::CPosePDFSOG::*)(double, double, double, double, double, double, class mrpt::math::CMatrixDynamic<double> &, bool)) &mrpt::poses::CPosePDFSOG::evaluatePDFInArea, "Evaluates the PDF within a rectangular grid (and a fixed orientation)\n and saves the result in a matrix (each row contains values for a fixed\n y-coordinate value). \n\nC++: mrpt::poses::CPosePDFSOG::evaluatePDFInArea(double, double, double, double, double, double, class mrpt::math::CMatrixDynamic<double> &, bool) --> void", pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("resolutionXY"), pybind11::arg("phi"), pybind11::arg("outMatrix"), pybind11::arg("sumOverAllPhis"));
		cl.def("bayesianFusion", [](mrpt::poses::CPosePDFSOG &o, const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPosePDFSOG::*)(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double)) &mrpt::poses::CPosePDFSOG::bayesianFusion, "Bayesian fusion of two pose distributions, then save the result in this\n object (WARNING: Currently p1 must be a mrpt::poses::CPosePDFSOG object\n and p2 a mrpt::poses::CPosePDFGaussian object) \n\nC++: mrpt::poses::CPosePDFSOG::bayesianFusion(const class mrpt::poses::CPosePDF &, const class mrpt::poses::CPosePDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("assign", (class mrpt::poses::CPosePDFSOG & (mrpt::poses::CPosePDFSOG::*)(const class mrpt::poses::CPosePDFSOG &)) &mrpt::poses::CPosePDFSOG::operator=, "C++: mrpt::poses::CPosePDFSOG::operator=(const class mrpt::poses::CPosePDFSOG &) --> class mrpt::poses::CPosePDFSOG &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::poses::CPosePDFSOG::TGaussianMode file:mrpt/poses/CPosePDFSOG.h line:42
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::poses::CPosePDFSOG::TGaussianMode, std::shared_ptr<mrpt::poses::CPosePDFSOG::TGaussianMode>> cl(enclosing_class, "TGaussianMode", "The struct for each mode:");
			cl.def( pybind11::init( [](){ return new mrpt::poses::CPosePDFSOG::TGaussianMode(); } ) );
			cl.def( pybind11::init( [](mrpt::poses::CPosePDFSOG::TGaussianMode const &o){ return new mrpt::poses::CPosePDFSOG::TGaussianMode(o); } ) );
			cl.def_readwrite("mean", &mrpt::poses::CPosePDFSOG::TGaussianMode::mean);
			cl.def_readwrite("cov", &mrpt::poses::CPosePDFSOG::TGaussianMode::cov);
			cl.def_readwrite("log_w", &mrpt::poses::CPosePDFSOG::TGaussianMode::log_w);
			cl.def("assign", (struct mrpt::poses::CPosePDFSOG::TGaussianMode & (mrpt::poses::CPosePDFSOG::TGaussianMode::*)(const struct mrpt::poses::CPosePDFSOG::TGaussianMode &)) &mrpt::poses::CPosePDFSOG::TGaussianMode::operator=, "C++: mrpt::poses::CPosePDFSOG::TGaussianMode::operator=(const struct mrpt::poses::CPosePDFSOG::TGaussianMode &) --> struct mrpt::poses::CPosePDFSOG::TGaussianMode &", pybind11::return_value_policy::automatic, pybind11::arg(""));

			cl.def("__str__", [](mrpt::poses::CPosePDFSOG::TGaussianMode const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );
		}

	}
}
