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
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
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

// mrpt::poses::CPose3DPDF file:mrpt/poses/CPose3DPDF.h line:36
struct PyCallBack_mrpt_poses_CPose3DPDF : public mrpt::poses::CPose3DPDF {
	using mrpt::poses::CPose3DPDF::CPose3DPDF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DPDF::GetRuntimeClass();
	}
	void copyFrom(const class mrpt::poses::CPose3DPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPose3DPDF::copyFrom\"");
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPose3DPDF::changeCoordinatesReference\"");
	}
	void bayesianFusion(const class mrpt::poses::CPose3DPDF & a0, const class mrpt::poses::CPose3DPDF & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPose3DPDF::bayesianFusion\"");
	}
	void inverse(class mrpt::poses::CPose3DPDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPose3DPDF::inverse\"");
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "serializeGetVersion");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "serializeTo");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "serializeFrom");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "clone");
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
	void getMean(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "getMean");
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
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "getCovarianceAndMean");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "getInformationMatrix");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "saveToTextFile");
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
	void drawSingleSample(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDF *>(this), "drawSingleSample");
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

// mrpt::poses::CPose3DPDFGaussian file:mrpt/poses/CPose3DPDFGaussian.h line:38
struct PyCallBack_mrpt_poses_CPose3DPDFGaussian : public mrpt::poses::CPose3DPDFGaussian {
	using mrpt::poses::CPose3DPDFGaussian::CPose3DPDFGaussian;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DPDFGaussian::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3DPDFGaussian::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3DPDFGaussian::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussian::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussian::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussian::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPose3DPDFGaussian::getCovarianceAndMean();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPose3DPDFGaussian::asString();
	}
	void copyFrom(const class mrpt::poses::CPose3DPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussian::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPose3DPDFGaussian::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussian::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussian::drawSingleSample(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPose3DPDF & a0, const class mrpt::poses::CPose3DPDF & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussian::bayesianFusion(a0, a1);
	}
	void inverse(class mrpt::poses::CPose3DPDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFGaussian::inverse(a0);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFGaussian *>(this), "getInformationMatrix");
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

void bind_mrpt_poses_CPose3DPDF(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose3DPDF file:mrpt/poses/CPose3DPDF.h line:36
		pybind11::class_<mrpt::poses::CPose3DPDF, std::shared_ptr<mrpt::poses::CPose3DPDF>, PyCallBack_mrpt_poses_CPose3DPDF, mrpt::serialization::CSerializable, mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPose3D,6UL>> cl(M("mrpt::poses"), "CPose3DPDF", "A Probability Density Function (PDF) of a SE(3) pose.\n This class is just the base class for unifying many diferent\n    ways this PDF can be implemented.\n\n  For convenience, a pose composition is also defined for any\n    PDF derived class, changeCoordinatesReference, in the form of a method\n rather than an operator.\n\n  For a similar class for 3D points (without attitude), see CPointPDF\n\n  See also:\n  [probabilistic spatial representations](tutorial-pdf-over-poses.html)\n\n \n CPose3D, CPosePDF, CPointPDF\n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPose3DPDF const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPose3DPDF(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DPDF::*)() const) &mrpt::poses::CPose3DPDF::GetRuntimeClass, "C++: mrpt::poses::CPose3DPDF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DPDF::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DPDF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDF::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDF::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations)\n \n\n createFrom2D\n\nC++: mrpt::poses::CPose3DPDF::copyFrom(const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("o"));
		cl.def_static("createFrom2D", (class mrpt::poses::CPose3DPDF * (*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPose3DPDF::createFrom2D, "This is a static transformation method from 2D poses to 3D PDFs,\n preserving the representation type (particles->particles,\n Gaussians->Gaussians,etc)\n\n \n It returns a new object of any of the derived classes of\n CPose3DPDF. This object must be deleted by the user when not required\n anymore.\n\n \n copyFrom\n\nC++: mrpt::poses::CPose3DPDF::createFrom2D(const class mrpt::poses::CPosePDF &) --> class mrpt::poses::CPose3DPDF *", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DPDF::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DPDF::changeCoordinatesReference, "C++: mrpt::poses::CPose3DPDF::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPose3DPDF::*)(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDF::bayesianFusion, "Bayesian fusion of two pose distributions, then save the result in this\n object (WARNING: Currently only distributions of the same class can be\n fused! eg, gaussian with gaussian,etc) \n\nC++: mrpt::poses::CPose3DPDF::bayesianFusion(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("inverse", (void (mrpt::poses::CPose3DPDF::*)(class mrpt::poses::CPose3DPDF &) const) &mrpt::poses::CPose3DPDF::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPose3DPDF::inverse(class mrpt::poses::CPose3DPDF &) const --> void", pybind11::arg("o"));
		cl.def_static("jacobiansPoseComposition", (void (*)(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPose3D &, class mrpt::math::CMatrixFixed<double, 6, 6> &, class mrpt::math::CMatrixFixed<double, 6, 6> &)) &mrpt::poses::CPose3DPDF::jacobiansPoseComposition, "This static method computes the pose composition Jacobians.\n\n See this technical report:\n http:///www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty\n\n Direct equations (for the covariances) in yaw-pitch-roll are too complex.\n  Make a way around them and consider instead this path:\n \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n   \n\nC++: mrpt::poses::CPose3DPDF::jacobiansPoseComposition(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPose3D &, class mrpt::math::CMatrixFixed<double, 6, 6> &, class mrpt::math::CMatrixFixed<double, 6, 6> &) --> void", pybind11::arg("x"), pybind11::arg("u"), pybind11::arg("df_dx"), pybind11::arg("df_du"));
		cl.def_static("is_3D", (bool (*)()) &mrpt::poses::CPose3DPDF::is_3D, "C++: mrpt::poses::CPose3DPDF::is_3D() --> bool");
		cl.def_static("is_PDF", (bool (*)()) &mrpt::poses::CPose3DPDF::is_PDF, "C++: mrpt::poses::CPose3DPDF::is_PDF() --> bool");
		cl.def("assign", (class mrpt::poses::CPose3DPDF & (mrpt::poses::CPose3DPDF::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDF::operator=, "C++: mrpt::poses::CPose3DPDF::operator=(const class mrpt::poses::CPose3DPDF &) --> class mrpt::poses::CPose3DPDF &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPose3DPDFGaussian file:mrpt/poses/CPose3DPDFGaussian.h line:38
		pybind11::class_<mrpt::poses::CPose3DPDFGaussian, std::shared_ptr<mrpt::poses::CPose3DPDFGaussian>, PyCallBack_mrpt_poses_CPose3DPDFGaussian, mrpt::poses::CPose3DPDF, mrpt::Stringifyable> cl(M("mrpt::poses"), "CPose3DPDFGaussian", "Declares a class that represents a Probability Density function (PDF) of a\n 3D pose \n\n.\n\n   This class implements that PDF using a mono-modal Gaussian distribution.\n See mrpt::poses::CPose3DPDF for more details.\n\n  Uncertainty of pose composition operations (\n) is\n implemented in the method \"CPose3DPDFGaussian::operator+=\".\n\n \n Read also: \"A tutorial on SE(3) transformation parameterizations and\n on-manifold optimization\", in \n\n \n CPose3D, CPose3DPDF, CPose3DPDFParticles\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DPDFGaussian(); }, [](){ return new PyCallBack_mrpt_poses_CPose3DPDFGaussian(); } ) );
		cl.def( pybind11::init<const class mrpt::poses::CPose3D &>(), pybind11::arg("init_Mean") );

		cl.def( pybind11::init<enum mrpt::poses::TConstructorFlags_Poses>(), pybind11::arg("constructor_dummy_param") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3D &, const class mrpt::math::CMatrixFixed<double, 6, 6> &>(), pybind11::arg("init_Mean"), pybind11::arg("init_Cov") );

		cl.def( pybind11::init<const class mrpt::poses::CPosePDFGaussian &>(), pybind11::arg("o") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DQuatPDFGaussian &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3DPDFGaussian const &o){ return new PyCallBack_mrpt_poses_CPose3DPDFGaussian(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3DPDFGaussian const &o){ return new mrpt::poses::CPose3DPDFGaussian(o); } ) );
		cl.def_readwrite("mean", &mrpt::poses::CPose3DPDFGaussian::mean);
		cl.def_readwrite("cov", &mrpt::poses::CPose3DPDFGaussian::cov);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DPDFGaussian::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DPDFGaussian::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DPDFGaussian::*)() const) &mrpt::poses::CPose3DPDFGaussian::GetRuntimeClass, "C++: mrpt::poses::CPose3DPDFGaussian::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3DPDFGaussian::*)() const) &mrpt::poses::CPose3DPDFGaussian::clone, "C++: mrpt::poses::CPose3DPDFGaussian::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3DPDFGaussian::CreateObject, "C++: mrpt::poses::CPose3DPDFGaussian::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getPoseMean", (class mrpt::poses::CPose3D & (mrpt::poses::CPose3DPDFGaussian::*)()) &mrpt::poses::CPose3DPDFGaussian::getPoseMean, "C++: mrpt::poses::CPose3DPDFGaussian::getPoseMean() --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic);
		cl.def("getMean", (void (mrpt::poses::CPose3DPDFGaussian::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGaussian::getMean, "Returns an estimate of the pose, (the mean, or mathematical expectation\n of the PDF).\n \n\n getCovariance\n\nC++: mrpt::poses::CPose3DPDFGaussian::getMean(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("mean_pose"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D> (mrpt::poses::CPose3DPDFGaussian::*)() const) &mrpt::poses::CPose3DPDFGaussian::getCovarianceAndMean, "Returns an estimate of the pose covariance matrix (6x6 cov matrix) and\n the mean, both at once.\n \n\n getMean\n\nC++: mrpt::poses::CPose3DPDFGaussian::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>");
		cl.def("asString", (std::string (mrpt::poses::CPose3DPDFGaussian::*)() const) &mrpt::poses::CPose3DPDFGaussian::asString, "C++: mrpt::poses::CPose3DPDFGaussian::asString() const --> std::string");
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFGaussian::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations)\n\nC++: mrpt::poses::CPose3DPDFGaussian::copyFrom(const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("o"));
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPose3DPDFGaussian::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations)\n\nC++: mrpt::poses::CPose3DPDFGaussian::copyFrom(const class mrpt::poses::CPosePDF &) --> void", pybind11::arg("o"));
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3DQuatPDFGaussian &)) &mrpt::poses::CPose3DPDFGaussian::copyFrom, "Copy from a 6D pose PDF described as a Quaternion\n\nC++: mrpt::poses::CPose3DPDFGaussian::copyFrom(const class mrpt::poses::CPose3DQuatPDFGaussian &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPose3DPDFGaussian::*)(const std::string &) const) &mrpt::poses::CPose3DPDFGaussian::saveToTextFile, "Save the PDF to a text file, containing the 3D pose in the first line,\n then the covariance matrix in next 3 lines.\n\nC++: mrpt::poses::CPose3DPDFGaussian::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DPDFGaussian::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object.\n\nC++: mrpt::poses::CPose3DPDFGaussian::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPose3DPDFGaussian::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGaussian::drawSingleSample, "Draws a single sample from the distribution\n\nC++: mrpt::poses::CPose3DPDFGaussian::drawSingleSample(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("outPart"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFGaussian::bayesianFusion, "Bayesian fusion of two points gauss. distributions, then save the result\nin this object.\n  The process is as follows:\n		- (x1,S1): Mean and variance of the p1 distribution.\n		- (x2,S2): Mean and variance of the p2 distribution.\n		- (x,S): Mean and variance of the resulting distribution.\n\n    \n\n    \n\n   \n\nC++: mrpt::poses::CPose3DPDFGaussian::bayesianFusion(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("inverse", (void (mrpt::poses::CPose3DPDFGaussian::*)(class mrpt::poses::CPose3DPDF &) const) &mrpt::poses::CPose3DPDFGaussian::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF\n\nC++: mrpt::poses::CPose3DPDFGaussian::inverse(class mrpt::poses::CPose3DPDF &) const --> void", pybind11::arg("o"));
		cl.def("__neg__", (class mrpt::poses::CPose3DPDFGaussian (mrpt::poses::CPose3DPDFGaussian::*)() const) &mrpt::poses::CPose3DPDFGaussian::operator-, "Unary - operator, returns the PDF of the inverse pose.  \n\nC++: mrpt::poses::CPose3DPDFGaussian::operator-() const --> class mrpt::poses::CPose3DPDFGaussian");
		cl.def("__iadd__", (void (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DPDFGaussian::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated).\n\nC++: mrpt::poses::CPose3DPDFGaussian::operator+=(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("Ap"));
		cl.def("__iadd__", (void (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3DPDFGaussian &)) &mrpt::poses::CPose3DPDFGaussian::operator+=, "Makes: thisPDF = thisPDF + Ap, where \"+\" is pose composition (both the\n mean, and the covariance matrix are updated).\n\nC++: mrpt::poses::CPose3DPDFGaussian::operator+=(const class mrpt::poses::CPose3DPDFGaussian &) --> void", pybind11::arg("Ap"));
		cl.def("__isub__", (void (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3DPDFGaussian &)) &mrpt::poses::CPose3DPDFGaussian::operator-=, "Makes: thisPDF = thisPDF - Ap, where \"-\" is pose inverse composition\n (both the mean, and the covariance matrix are updated).\n\nC++: mrpt::poses::CPose3DPDFGaussian::operator-=(const class mrpt::poses::CPose3DPDFGaussian &) --> void", pybind11::arg("Ap"));
		cl.def("evaluatePDF", (double (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGaussian::evaluatePDF, "Evaluates the PDF at a given point.\n\nC++: mrpt::poses::CPose3DPDFGaussian::evaluatePDF(const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("x"));
		cl.def("evaluateNormalizedPDF", (double (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFGaussian::evaluateNormalizedPDF, "Evaluates the ratio PDF(x) / PDF(MEAN), that is, the normalized PDF in\n the range [0,1].\n\nC++: mrpt::poses::CPose3DPDFGaussian::evaluateNormalizedPDF(const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("x"));
		cl.def("mahalanobisDistanceTo", (double (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3DPDFGaussian &)) &mrpt::poses::CPose3DPDFGaussian::mahalanobisDistanceTo, "Computes the Mahalanobis distance between the centers of two Gaussians.\n  The variables with a variance exactly equal to 0 are not taken into\n account in the process, but\n   \"infinity\" is returned if the corresponding elements are not exactly\n equal.\n\nC++: mrpt::poses::CPose3DPDFGaussian::mahalanobisDistanceTo(const class mrpt::poses::CPose3DPDFGaussian &) --> double", pybind11::arg("theOther"));
		cl.def("getCovSubmatrix2D", (void (mrpt::poses::CPose3DPDFGaussian::*)(class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::poses::CPose3DPDFGaussian::getCovSubmatrix2D, "Returns a 3x3 matrix with submatrix of the covariance for the variables\n (x,y,yaw) only.\n\nC++: mrpt::poses::CPose3DPDFGaussian::getCovSubmatrix2D(class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("out_cov"));
		cl.def("assign", (class mrpt::poses::CPose3DPDFGaussian & (mrpt::poses::CPose3DPDFGaussian::*)(const class mrpt::poses::CPose3DPDFGaussian &)) &mrpt::poses::CPose3DPDFGaussian::operator=, "C++: mrpt::poses::CPose3DPDFGaussian::operator=(const class mrpt::poses::CPose3DPDFGaussian &) --> class mrpt::poses::CPose3DPDFGaussian &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPose3DPDFGaussian const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );

		cl.def("__add__", [](const mrpt::poses::CPose3DPDFGaussian&a, const mrpt::poses::CPose3DPDFGaussian& b) -> mrpt::poses::CPose3DPDFGaussian { return a+b; });
		cl.def("__sub__", [](const mrpt::poses::CPose3DPDFGaussian&a, const mrpt::poses::CPose3DPDFGaussian& b) -> mrpt::poses::CPose3DPDFGaussian { return a-b; });
	}
}
