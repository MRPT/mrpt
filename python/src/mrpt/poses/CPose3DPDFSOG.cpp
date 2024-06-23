#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/core/aligned_allocator.h>
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
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
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

// mrpt::poses::CPose3DPDFSOG file:mrpt/poses/CPose3DPDFSOG.h line:32
struct PyCallBack_mrpt_poses_CPose3DPDFSOG : public mrpt::poses::CPose3DPDFSOG {
	using mrpt::poses::CPose3DPDFSOG::CPose3DPDFSOG;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DPDFSOG::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3DPDFSOG::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3DPDFSOG::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFSOG::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFSOG::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFSOG::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPose3DPDFSOG::getCovarianceAndMean();
	}
	void copyFrom(const class mrpt::poses::CPose3DPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFSOG::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPose3DPDFSOG::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFSOG::changeCoordinatesReference(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPose3DPDF & a0, const class mrpt::poses::CPose3DPDF & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFSOG::bayesianFusion(a0, a1);
	}
	void drawSingleSample(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFSOG::drawSingleSample(a0);
	}
	void inverse(class mrpt::poses::CPose3DPDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFSOG::inverse(a0);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFSOG *>(this), "getInformationMatrix");
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

// mrpt::poses::CPose3DQuatPDF file:mrpt/poses/CPose3DQuatPDF.h line:41
struct PyCallBack_mrpt_poses_CPose3DQuatPDF : public mrpt::poses::CPose3DQuatPDF {
	using mrpt::poses::CPose3DQuatPDF::CPose3DQuatPDF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DQuatPDF::GetRuntimeClass();
	}
	void copyFrom(const class mrpt::poses::CPose3DQuatPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPose3DQuatPDF::copyFrom\"");
	}
	void inverse(class mrpt::poses::CPose3DQuatPDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPose3DQuatPDF::inverse\"");
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPose3DQuatPDF::changeCoordinatesReference\"");
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "serializeGetVersion");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "serializeTo");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "serializeFrom");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "clone");
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
	void getMean(class mrpt::poses::CPose3DQuat & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "getMean");
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
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 7, 7>, class mrpt::poses::CPose3DQuat>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "getCovarianceAndMean");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "isInfType");
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
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 7, 7> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "getInformationMatrix");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "saveToTextFile");
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
	void drawSingleSample(class mrpt::poses::CPose3DQuat & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDF *>(this), "drawSingleSample");
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

// mrpt::poses::CPose3DQuatPDFGaussian file:mrpt/poses/CPose3DQuatPDFGaussian.h line:41
struct PyCallBack_mrpt_poses_CPose3DQuatPDFGaussian : public mrpt::poses::CPose3DQuatPDFGaussian {
	using mrpt::poses::CPose3DQuatPDFGaussian::CPose3DQuatPDFGaussian;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPose3DQuat & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 7, 7>, class mrpt::poses::CPose3DQuat>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::getCovarianceAndMean();
	}
	void copyFrom(const class mrpt::poses::CPose3DQuatPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose3DQuat & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::drawSingleSample(a0);
	}
	void inverse(class mrpt::poses::CPose3DQuatPDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuatPDFGaussian::inverse(a0);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "isInfType");
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
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 7, 7> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuatPDFGaussian *>(this), "getInformationMatrix");
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

void bind_mrpt_poses_CPose3DPDFSOG(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose3DPDFSOG file:mrpt/poses/CPose3DPDFSOG.h line:32
		pybind11::class_<mrpt::poses::CPose3DPDFSOG, std::shared_ptr<mrpt::poses::CPose3DPDFSOG>, PyCallBack_mrpt_poses_CPose3DPDFSOG, mrpt::poses::CPose3DPDF> cl(M("mrpt::poses"), "CPose3DPDFSOG", "Declares a class that represents a Probability Density function (PDF) of a\n 3D(6D) pose \n\n.\n   This class implements that PDF as the following multi-modal Gaussian\n distribution:\n\n \n\n\n  Where the number of modes N is the size of CPose3DPDFSOG::m_modes. Angles\n are always in radians.\n\n  See mrpt::poses::CPose3DPDF for more details.\n \n\n\n \n CPose3DPDF");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DPDFSOG(); }, [](){ return new PyCallBack_mrpt_poses_CPose3DPDFSOG(); } ), "doc");
		cl.def( pybind11::init<size_t>(), pybind11::arg("nModes") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3DPDFSOG const &o){ return new PyCallBack_mrpt_poses_CPose3DPDFSOG(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3DPDFSOG const &o){ return new mrpt::poses::CPose3DPDFSOG(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DPDFSOG::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DPDFSOG::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DPDFSOG::*)() const) &mrpt::poses::CPose3DPDFSOG::GetRuntimeClass, "C++: mrpt::poses::CPose3DPDFSOG::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3DPDFSOG::*)() const) &mrpt::poses::CPose3DPDFSOG::clone, "C++: mrpt::poses::CPose3DPDFSOG::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3DPDFSOG::CreateObject, "C++: mrpt::poses::CPose3DPDFSOG::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clear", (void (mrpt::poses::CPose3DPDFSOG::*)()) &mrpt::poses::CPose3DPDFSOG::clear, "Clear all the gaussian modes \n\nC++: mrpt::poses::CPose3DPDFSOG::clear() --> void");
		cl.def("resize", (void (mrpt::poses::CPose3DPDFSOG::*)(size_t)) &mrpt::poses::CPose3DPDFSOG::resize, "Set the number of SOG modes \n\nC++: mrpt::poses::CPose3DPDFSOG::resize(size_t) --> void", pybind11::arg("N"));
		cl.def("size", (size_t (mrpt::poses::CPose3DPDFSOG::*)() const) &mrpt::poses::CPose3DPDFSOG::size, "Return the number of Gaussian modes. \n\nC++: mrpt::poses::CPose3DPDFSOG::size() const --> size_t");
		cl.def("empty", (bool (mrpt::poses::CPose3DPDFSOG::*)() const) &mrpt::poses::CPose3DPDFSOG::empty, "Return whether there is any Gaussian mode. \n\nC++: mrpt::poses::CPose3DPDFSOG::empty() const --> bool");
		cl.def("getMean", (void (mrpt::poses::CPose3DPDFSOG::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFSOG::getMean, "C++: mrpt::poses::CPose3DPDFSOG::getMean(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("mean_pose"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D> (mrpt::poses::CPose3DPDFSOG::*)() const) &mrpt::poses::CPose3DPDFSOG::getCovarianceAndMean, "C++: mrpt::poses::CPose3DPDFSOG::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>");
		cl.def("normalizeWeights", (void (mrpt::poses::CPose3DPDFSOG::*)()) &mrpt::poses::CPose3DPDFSOG::normalizeWeights, "Normalize the weights in m_modes such as the maximum log-weight is 0. \n\nC++: mrpt::poses::CPose3DPDFSOG::normalizeWeights() --> void");
		cl.def("getMostLikelyMode", (void (mrpt::poses::CPose3DPDFSOG::*)(class mrpt::poses::CPose3DPDFGaussian &) const) &mrpt::poses::CPose3DPDFSOG::getMostLikelyMode, "Return the Gaussian mode with the highest likelihood (or an empty\n Gaussian if there are no modes in this SOG) \n\nC++: mrpt::poses::CPose3DPDFSOG::getMostLikelyMode(class mrpt::poses::CPose3DPDFGaussian &) const --> void", pybind11::arg("outVal"));
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDFSOG::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFSOG::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPose3DPDFSOG::copyFrom(const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPose3DPDFSOG::*)(const std::string &) const) &mrpt::poses::CPose3DPDFSOG::saveToTextFile, "Save the density to a text file, with the following format:\n  There is one row per Gaussian \"mode\", and each row contains 10\n elements:\n   - w (The linear weight)\n   - x_mean (gaussian mean value)\n   - y_mean (gaussian mean value)\n   - x_mean (gaussian mean value)\n   - yaw_mean (gaussian mean value, in radians)\n   - pitch_mean (gaussian mean value, in radians)\n   - roll_mean (gaussian mean value, in radians)\n   - C11,C22,C33,C44,C55,C66 (Covariance elements)\n   - C12,C13,C14,C15,C16 (Covariance elements)\n   - C23,C24,C25,C25 (Covariance elements)\n   - C34,C35,C36 (Covariance elements)\n   - C45,C46 (Covariance elements)\n   - C56 (Covariance elements)\n\n   \n\nC++: mrpt::poses::CPose3DPDFSOG::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DPDFSOG::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DPDFSOG::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPose3DPDFSOG::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPose3DPDFSOG::*)(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFSOG::bayesianFusion, "Bayesian fusion of two pose distributions, then save the result in this\n object (WARNING: Currently p1 must be a mrpt::poses::CPose3DPDFSOG object\n and p2 a mrpt::poses::CPose3DPDFSOG object) \n\nC++: mrpt::poses::CPose3DPDFSOG::bayesianFusion(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPose3DPDFSOG::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFSOG::drawSingleSample, "Draws a single sample from the distribution \n\nC++: mrpt::poses::CPose3DPDFSOG::drawSingleSample(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("outPart"));
		cl.def("inverse", (void (mrpt::poses::CPose3DPDFSOG::*)(class mrpt::poses::CPose3DPDF &) const) &mrpt::poses::CPose3DPDFSOG::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPose3DPDFSOG::inverse(class mrpt::poses::CPose3DPDF &) const --> void", pybind11::arg("o"));
		cl.def("appendFrom", (void (mrpt::poses::CPose3DPDFSOG::*)(const class mrpt::poses::CPose3DPDFSOG &)) &mrpt::poses::CPose3DPDFSOG::appendFrom, "Append the Gaussian modes from \"o\" to the current set of modes of \"this\"\n density \n\nC++: mrpt::poses::CPose3DPDFSOG::appendFrom(const class mrpt::poses::CPose3DPDFSOG &) --> void", pybind11::arg("o"));
		cl.def("assign", (class mrpt::poses::CPose3DPDFSOG & (mrpt::poses::CPose3DPDFSOG::*)(const class mrpt::poses::CPose3DPDFSOG &)) &mrpt::poses::CPose3DPDFSOG::operator=, "C++: mrpt::poses::CPose3DPDFSOG::operator=(const class mrpt::poses::CPose3DPDFSOG &) --> class mrpt::poses::CPose3DPDFSOG &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::poses::CPose3DPDFSOG::TGaussianMode file:mrpt/poses/CPose3DPDFSOG.h line:39
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::poses::CPose3DPDFSOG::TGaussianMode, std::shared_ptr<mrpt::poses::CPose3DPDFSOG::TGaussianMode>> cl(enclosing_class, "TGaussianMode", "The struct for each mode:");
			cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DPDFSOG::TGaussianMode(); } ) );
			cl.def_readwrite("val", &mrpt::poses::CPose3DPDFSOG::TGaussianMode::val);
			cl.def_readwrite("log_w", &mrpt::poses::CPose3DPDFSOG::TGaussianMode::log_w);
		}

	}
	{ // mrpt::poses::CPose3DQuatPDF file:mrpt/poses/CPose3DQuatPDF.h line:41
		pybind11::class_<mrpt::poses::CPose3DQuatPDF, std::shared_ptr<mrpt::poses::CPose3DQuatPDF>, PyCallBack_mrpt_poses_CPose3DQuatPDF, mrpt::serialization::CSerializable, mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3DQuat,7UL>> cl(M("mrpt::poses"), "CPose3DQuatPDF", "Declares a class that represents a Probability Density Function (PDF) of a\n 3D pose (6D actually), by means of a 7-vector with a translation [x y z] and\n a quaternion [qr qx qy qz].\n   This class is just the base class for unifying many diferent ways this PDF\n can be implemented.\n\n  For convenience, a pose composition is also defined for any\n    PDF derived class, changeCoordinatesReference, in the form of a method\n rather than an operator.\n\n  - For a similar class for 3D points (without attitude), see CPointPDF.\n  - For a similar class for 3D poses  (with Euler angles instead of\n quaternions), see CPose3DPDF.\n\n  See also:\n  [probabilistic spatial representations](tutorial-pdf-over-poses.html)\n\n \n CPose3DQuatPDF, CPose3DPDF\n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPose3DQuatPDF const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPose3DQuatPDF(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DQuatPDF::*)() const) &mrpt::poses::CPose3DQuatPDF::GetRuntimeClass, "C++: mrpt::poses::CPose3DQuatPDF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DQuatPDF::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DQuatPDF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("copyFrom", (void (mrpt::poses::CPose3DQuatPDF::*)(const class mrpt::poses::CPose3DQuatPDF &)) &mrpt::poses::CPose3DQuatPDF::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations)\n \n\n createFrom2D\n\nC++: mrpt::poses::CPose3DQuatPDF::copyFrom(const class mrpt::poses::CPose3DQuatPDF &) --> void", pybind11::arg("o"));
		cl.def_static("createFrom2D", (class std::shared_ptr<class mrpt::poses::CPose3DQuatPDF> (*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPose3DQuatPDF::createFrom2D, "This is a static transformation method from 2D poses to 3D PDFs,\n preserving the representation type (particles->particles,\n Gaussians->Gaussians,etc)\n  It returns a new object of any of the derived classes of\n CPose3DQuatPDF. This object must be deleted by the user when not required\n anymore.\n  \n\n copyFrom\n\nC++: mrpt::poses::CPose3DQuatPDF::createFrom2D(const class mrpt::poses::CPosePDF &) --> class std::shared_ptr<class mrpt::poses::CPose3DQuatPDF>", pybind11::arg("o"));
		cl.def("inverse", (void (mrpt::poses::CPose3DQuatPDF::*)(class mrpt::poses::CPose3DQuatPDF &) const) &mrpt::poses::CPose3DQuatPDF::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPose3DQuatPDF::inverse(class mrpt::poses::CPose3DQuatPDF &) const --> void", pybind11::arg("o"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DQuatPDF::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DQuatPDF::changeCoordinatesReference, "C++: mrpt::poses::CPose3DQuatPDF::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def_static("jacobiansPoseComposition", [](const class mrpt::poses::CPose3DQuat & a0, const class mrpt::poses::CPose3DQuat & a1, class mrpt::math::CMatrixFixed<double, 7, 7> & a2, class mrpt::math::CMatrixFixed<double, 7, 7> & a3) -> void { return mrpt::poses::CPose3DQuatPDF::jacobiansPoseComposition(a0, a1, a2, a3); }, "", pybind11::arg("x"), pybind11::arg("u"), pybind11::arg("df_dx"), pybind11::arg("df_du"));
		cl.def_static("jacobiansPoseComposition", (void (*)(const class mrpt::poses::CPose3DQuat &, const class mrpt::poses::CPose3DQuat &, class mrpt::math::CMatrixFixed<double, 7, 7> &, class mrpt::math::CMatrixFixed<double, 7, 7> &, class mrpt::poses::CPose3DQuat *)) &mrpt::poses::CPose3DQuatPDF::jacobiansPoseComposition, "This static method computes the two Jacobians of a pose composition\n operation \n\n\n  \n If set to !=nullptr, the result of \"x+u\" will be\n stored here (it will be computed internally anyway).\n  To see the mathematical derivation of the formulas, refer to the\n technical report here:\n   -\n https://www.mrpt.org/Probability_Density_Distributions_Over_Spatial_Representations\n\nC++: mrpt::poses::CPose3DQuatPDF::jacobiansPoseComposition(const class mrpt::poses::CPose3DQuat &, const class mrpt::poses::CPose3DQuat &, class mrpt::math::CMatrixFixed<double, 7, 7> &, class mrpt::math::CMatrixFixed<double, 7, 7> &, class mrpt::poses::CPose3DQuat *) --> void", pybind11::arg("x"), pybind11::arg("u"), pybind11::arg("df_dx"), pybind11::arg("df_du"), pybind11::arg("out_x_oplus_u"));
		cl.def("assign", (class mrpt::poses::CPose3DQuatPDF & (mrpt::poses::CPose3DQuatPDF::*)(const class mrpt::poses::CPose3DQuatPDF &)) &mrpt::poses::CPose3DQuatPDF::operator=, "C++: mrpt::poses::CPose3DQuatPDF::operator=(const class mrpt::poses::CPose3DQuatPDF &) --> class mrpt::poses::CPose3DQuatPDF &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPose3DQuatPDFGaussian file:mrpt/poses/CPose3DQuatPDFGaussian.h line:41
		pybind11::class_<mrpt::poses::CPose3DQuatPDFGaussian, std::shared_ptr<mrpt::poses::CPose3DQuatPDFGaussian>, PyCallBack_mrpt_poses_CPose3DQuatPDFGaussian, mrpt::poses::CPose3DQuatPDF> cl(M("mrpt::poses"), "CPose3DQuatPDFGaussian", "Declares a class that represents a Probability Density function (PDF) of a\n 3D pose using a quaternion \n\n\n\n.\n\n   This class implements that PDF using a mono-modal Gaussian distribution.\n See mrpt::poses::CPose3DQuatPDF for more details, or\n    mrpt::poses::CPose3DPDF for classes based on Euler angles instead.\n\n  Uncertainty of pose composition operations (\n) is\n implemented in the methods \"CPose3DQuatPDFGaussian::operator+=\" and\n \"CPose3DQuatPDF::jacobiansPoseComposition\".\n\n \n Read also: \"A tutorial on SE(3) transformation parameterizations and\n on-manifold optimization\", in \n\n \n CPose3DQuat, CPose3DQuatPDF, CPose3DPDF, CPose3DQuatPDFGaussianInf\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DQuatPDFGaussian(); }, [](){ return new PyCallBack_mrpt_poses_CPose3DQuatPDFGaussian(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Quaternions>(), pybind11::arg("constructor_dummy_param") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DQuat &>(), pybind11::arg("init_Mean") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DQuat &, const class mrpt::math::CMatrixFixed<double, 7, 7> &>(), pybind11::arg("init_Mean"), pybind11::arg("init_Cov") );

		cl.def( pybind11::init<const class mrpt::poses::CPosePDFGaussian &>(), pybind11::arg("o") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DPDFGaussian &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3DQuatPDFGaussian const &o){ return new PyCallBack_mrpt_poses_CPose3DQuatPDFGaussian(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3DQuatPDFGaussian const &o){ return new mrpt::poses::CPose3DQuatPDFGaussian(o); } ) );
		cl.def_readwrite("mean", &mrpt::poses::CPose3DQuatPDFGaussian::mean);
		cl.def_readwrite("cov", &mrpt::poses::CPose3DQuatPDFGaussian::cov);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DQuatPDFGaussian::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DQuatPDFGaussian::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DQuatPDFGaussian::*)() const) &mrpt::poses::CPose3DQuatPDFGaussian::GetRuntimeClass, "C++: mrpt::poses::CPose3DQuatPDFGaussian::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3DQuatPDFGaussian::*)() const) &mrpt::poses::CPose3DQuatPDFGaussian::clone, "C++: mrpt::poses::CPose3DQuatPDFGaussian::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3DQuatPDFGaussian::CreateObject, "C++: mrpt::poses::CPose3DQuatPDFGaussian::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getPoseMean", (class mrpt::poses::CPose3DQuat & (mrpt::poses::CPose3DQuatPDFGaussian::*)()) &mrpt::poses::CPose3DQuatPDFGaussian::getPoseMean, "C++: mrpt::poses::CPose3DQuatPDFGaussian::getPoseMean() --> class mrpt::poses::CPose3DQuat &", pybind11::return_value_policy::automatic);
		cl.def("getMean", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuatPDFGaussian::getMean, "C++: mrpt::poses::CPose3DQuatPDFGaussian::getMean(class mrpt::poses::CPose3DQuat &) const --> void", pybind11::arg("mean_pose"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 7, 7>, class mrpt::poses::CPose3DQuat> (mrpt::poses::CPose3DQuatPDFGaussian::*)() const) &mrpt::poses::CPose3DQuatPDFGaussian::getCovarianceAndMean, "C++: mrpt::poses::CPose3DQuatPDFGaussian::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 7, 7>, class mrpt::poses::CPose3DQuat>");
		cl.def("copyFrom", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuatPDF &)) &mrpt::poses::CPose3DQuatPDFGaussian::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::copyFrom(const class mrpt::poses::CPose3DQuatPDF &) --> void", pybind11::arg("o"));
		cl.def("copyFrom", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPose3DQuatPDFGaussian::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::copyFrom(const class mrpt::poses::CPosePDF &) --> void", pybind11::arg("o"));
		cl.def("copyFrom", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DPDFGaussian &)) &mrpt::poses::CPose3DQuatPDFGaussian::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::copyFrom(const class mrpt::poses::CPose3DPDFGaussian &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPose3DQuatPDFGaussian::*)(const std::string &) const) &mrpt::poses::CPose3DQuatPDFGaussian::saveToTextFile, "Save the PDF to a text file, containing the 3D pose in the first line (x\n y z qr qx qy qz), then the covariance matrix in the next 7 lines. \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuatPDFGaussian::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::changeCoordinatesReference(const class mrpt::poses::CPose3DQuat &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DQuatPDFGaussian::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object.\n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuatPDFGaussian::drawSingleSample, "Draws a single sample from the distribution \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::drawSingleSample(class mrpt::poses::CPose3DQuat &) const --> void", pybind11::arg("outPart"));
		cl.def("inverseJacobian", (class mrpt::math::CMatrixFixed<double, 7, 7> (mrpt::poses::CPose3DQuatPDFGaussian::*)() const) &mrpt::poses::CPose3DQuatPDFGaussian::inverseJacobian, "Compute the inverse Jacobian \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::inverseJacobian() const --> class mrpt::math::CMatrixFixed<double, 7, 7>");
		cl.def("inverse", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(class mrpt::poses::CPose3DQuatPDF &) const) &mrpt::poses::CPose3DQuatPDFGaussian::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::inverse(class mrpt::poses::CPose3DQuatPDF &) const --> void", pybind11::arg("o"));
		cl.def("inverseCompositionCrossCorrelation", (class mrpt::poses::CPose3DQuatPDFGaussian (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuatPDFGaussian &) const) &mrpt::poses::CPose3DQuatPDFGaussian::inverseCompositionCrossCorrelation, "Returns the displacement from the current pose (pose_from) to pose_to:\n displacement = - pose_from + pose_to\n It assumes that both poses are correlated via\n the direct generative model:\n pose_to = pose_from + displacement\n For a deeper explanation, check https://github.com/MRPT/mrpt/pull/1243\n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::inverseCompositionCrossCorrelation(const class mrpt::poses::CPose3DQuatPDFGaussian &) const --> class mrpt::poses::CPose3DQuatPDFGaussian", pybind11::arg("pose_to"));
		cl.def("__neg__", (class mrpt::poses::CPose3DQuatPDFGaussian (mrpt::poses::CPose3DQuatPDFGaussian::*)() const) &mrpt::poses::CPose3DQuatPDFGaussian::operator-, "Unary - operator, returns the PDF of the inverse pose.  \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::operator-() const --> class mrpt::poses::CPose3DQuatPDFGaussian");
		cl.def("__iadd__", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuatPDFGaussian::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated). \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::operator+=(const class mrpt::poses::CPose3DQuat &) --> void", pybind11::arg("Ap"));
		cl.def("__iadd__", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuatPDFGaussian &)) &mrpt::poses::CPose3DQuatPDFGaussian::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated) (see formulas in\n jacobiansPoseComposition ). \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::operator+=(const class mrpt::poses::CPose3DQuatPDFGaussian &) --> void", pybind11::arg("Ap"));
		cl.def("__isub__", (void (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuatPDFGaussian &)) &mrpt::poses::CPose3DQuatPDFGaussian::operator-=, "Makes: thisPDF = thisPDF - Ap, where \"-\" is pose inverse composition\n (both the mean, and the covariance matrix are updated).\n This operation assumes statistical independence between the two\n variables. If you want to take into account the cross-correlation between\n pose_1 and pose_2, use:\n displacement = pose_1.inverseCompositionCrossCorrelation(pose_2)\n Note: Both poses are correlated if the direct generative model is\n pose_2 = pose_1 + displacement\n \n\n inverseCompositionCrossCorrelation\n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::operator-=(const class mrpt::poses::CPose3DQuatPDFGaussian &) --> void", pybind11::arg("Ap"));
		cl.def("evaluatePDF", (double (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuatPDFGaussian::evaluatePDF, "Evaluates the PDF at a given point. \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::evaluatePDF(const class mrpt::poses::CPose3DQuat &) const --> double", pybind11::arg("x"));
		cl.def("evaluateNormalizedPDF", (double (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuatPDFGaussian::evaluateNormalizedPDF, "Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in\n the range [0,1]. \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::evaluateNormalizedPDF(const class mrpt::poses::CPose3DQuat &) const --> double", pybind11::arg("x"));
		cl.def("mahalanobisDistanceTo", (double (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuatPDFGaussian &)) &mrpt::poses::CPose3DQuatPDFGaussian::mahalanobisDistanceTo, "Computes the Mahalanobis distance between the centers of two Gaussians.\n  The variables with a variance exactly equal to 0 are not taken into\n account in the process, but\n   \"infinity\" is returned if the corresponding elements are not exactly\n equal. \n\nC++: mrpt::poses::CPose3DQuatPDFGaussian::mahalanobisDistanceTo(const class mrpt::poses::CPose3DQuatPDFGaussian &) --> double", pybind11::arg("theOther"));
		cl.def("assign", (class mrpt::poses::CPose3DQuatPDFGaussian & (mrpt::poses::CPose3DQuatPDFGaussian::*)(const class mrpt::poses::CPose3DQuatPDFGaussian &)) &mrpt::poses::CPose3DQuatPDFGaussian::operator=, "C++: mrpt::poses::CPose3DQuatPDFGaussian::operator=(const class mrpt::poses::CPose3DQuatPDFGaussian &) --> class mrpt::poses::CPose3DQuatPDFGaussian &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPose3DQuatPDFGaussian const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );
	}
}
