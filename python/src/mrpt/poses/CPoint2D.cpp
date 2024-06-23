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
#include <mrpt/poses/CPoint.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint2DPDF.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
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

// mrpt::poses::CPoint2D file:mrpt/poses/CPoint2D.h line:32
struct PyCallBack_mrpt_poses_CPoint2D : public mrpt::poses::CPoint2D {
	using mrpt::poses::CPoint2D::CPoint2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPoint2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPoint2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPoint2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2D::serializeFrom(a0, a1);
	}
	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2D *>(this), "setToNaN");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2D::setToNaN();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPoint::asString();
	}
};

// mrpt::poses::CPoint2DPDF file:mrpt/poses/CPoint2DPDF.h line:33
struct PyCallBack_mrpt_poses_CPoint2DPDF : public mrpt::poses::CPoint2DPDF {
	using mrpt::poses::CPoint2DPDF::CPoint2DPDF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPoint2DPDF::GetRuntimeClass();
	}
	void copyFrom(const class mrpt::poses::CPoint2DPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPoint2DPDF::copyFrom\"");
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPoint2DPDF::changeCoordinatesReference\"");
	}
	void bayesianFusion(const class mrpt::poses::CPoint2DPDF & a0, const class mrpt::poses::CPoint2DPDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPoint2DPDF::bayesianFusion\"");
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "serializeGetVersion");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "serializeTo");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "serializeFrom");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "clone");
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
	void getMean(class mrpt::poses::CPoint2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "getMean");
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
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 2, 2>, class mrpt::poses::CPoint2D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "getCovarianceAndMean");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "isInfType");
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
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 2, 2> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "getInformationMatrix");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "saveToTextFile");
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
	void drawSingleSample(class mrpt::poses::CPoint2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDF *>(this), "drawSingleSample");
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

// mrpt::poses::CPoint2DPDFGaussian file:mrpt/poses/CPoint2DPDFGaussian.h line:20
struct PyCallBack_mrpt_poses_CPoint2DPDFGaussian : public mrpt::poses::CPoint2DPDFGaussian {
	using mrpt::poses::CPoint2DPDFGaussian::CPoint2DPDFGaussian;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPoint2DPDFGaussian::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPoint2DPDFGaussian::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPoint2DPDFGaussian::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2DPDFGaussian::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2DPDFGaussian::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPoint2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2DPDFGaussian::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 2, 2>, class mrpt::poses::CPoint2D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPoint2DPDFGaussian::getCovarianceAndMean();
	}
	void copyFrom(const class mrpt::poses::CPoint2DPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2DPDFGaussian::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPoint2DPDFGaussian::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2DPDFGaussian::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPoint2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2DPDFGaussian::drawSingleSample(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPoint2DPDF & a0, const class mrpt::poses::CPoint2DPDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint2DPDFGaussian::bayesianFusion(a0, a1, a2);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "isInfType");
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
	void getInformationMatrix(class mrpt::math::CMatrixFixed<double, 2, 2> & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint2DPDFGaussian *>(this), "getInformationMatrix");
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

void bind_mrpt_poses_CPoint2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPoint2D file:mrpt/poses/CPoint2D.h line:32
		pybind11::class_<mrpt::poses::CPoint2D, std::shared_ptr<mrpt::poses::CPoint2D>, PyCallBack_mrpt_poses_CPoint2D, mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL>, mrpt::serialization::CSerializable> cl(M("mrpt::poses"), "CPoint2D", "A class used to store a 2D point.\n\n  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint,\n or refer\n    to the 2D/3D Geometry\n tutorial in the wiki.\n\n  \n   \n  \n\n \n CPoseOrPoint,CPose, CPoint\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPoint2D(); }, [](){ return new PyCallBack_mrpt_poses_CPoint2D(); } ) );
		cl.def( pybind11::init<double, double>(), pybind11::arg("x"), pybind11::arg("y") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint2D_<double> &>(), pybind11::arg("o") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("o") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPoint2D const &o){ return new PyCallBack_mrpt_poses_CPoint2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPoint2D const &o){ return new mrpt::poses::CPoint2D(o); } ) );
		cl.def_readwrite("m_coords", &mrpt::poses::CPoint2D::m_coords);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPoint2D::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPoint2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPoint2D::*)() const) &mrpt::poses::CPoint2D::GetRuntimeClass, "C++: mrpt::poses::CPoint2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPoint2D::*)() const) &mrpt::poses::CPoint2D::clone, "C++: mrpt::poses::CPoint2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPoint2D::CreateObject, "C++: mrpt::poses::CPoint2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("asTPoint", (struct mrpt::math::TPoint2D_<double> (mrpt::poses::CPoint2D::*)() const) &mrpt::poses::CPoint2D::asTPoint, "C++: mrpt::poses::CPoint2D::asTPoint() const --> struct mrpt::math::TPoint2D_<double>");
		cl.def("asVector", (void (mrpt::poses::CPoint2D::*)(class mrpt::math::CMatrixFixed<double, 2, 1> &) const) &mrpt::poses::CPoint2D::asVector, "Return the pose or point as a 2x1 vector [x, y]' \n\nC++: mrpt::poses::CPoint2D::asVector(class mrpt::math::CMatrixFixed<double, 2, 1> &) const --> void", pybind11::arg("v"));
		cl.def("__sub__", (class mrpt::poses::CPoint2D (mrpt::poses::CPoint2D::*)(const class mrpt::poses::CPose2D &) const) &mrpt::poses::CPoint2D::operator-, "The operator D=\"this\"-b is the pose inverse compounding operator,\n   the resulting points \"D\" fulfils: \"this\" = b + D, so that: b == a +\n (b-a)\n\nC++: mrpt::poses::CPoint2D::operator-(const class mrpt::poses::CPose2D &) const --> class mrpt::poses::CPoint2D", pybind11::arg("b"));
		cl.def_static("is_3D", (bool (*)()) &mrpt::poses::CPoint2D::is_3D, "C++: mrpt::poses::CPoint2D::is_3D() --> bool");
		cl.def_static("is_PDF", (bool (*)()) &mrpt::poses::CPoint2D::is_PDF, "C++: mrpt::poses::CPoint2D::is_PDF() --> bool");
		cl.def_static("size", (unsigned long (*)()) &mrpt::poses::CPoint2D::size, "C++: mrpt::poses::CPoint2D::size() --> unsigned long");
		cl.def_static("empty", (bool (*)()) &mrpt::poses::CPoint2D::empty, "C++: mrpt::poses::CPoint2D::empty() --> bool");
		cl.def_static("max_size", (unsigned long (*)()) &mrpt::poses::CPoint2D::max_size, "C++: mrpt::poses::CPoint2D::max_size() --> unsigned long");
		cl.def_static("resize", (void (*)(size_t)) &mrpt::poses::CPoint2D::resize, "C++: mrpt::poses::CPoint2D::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("setToNaN", (void (mrpt::poses::CPoint2D::*)()) &mrpt::poses::CPoint2D::setToNaN, "@} \n\nC++: mrpt::poses::CPoint2D::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoint2D & (mrpt::poses::CPoint2D::*)(const class mrpt::poses::CPoint2D &)) &mrpt::poses::CPoint2D::operator=, "C++: mrpt::poses::CPoint2D::operator=(const class mrpt::poses::CPoint2D &) --> class mrpt::poses::CPoint2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPoint2D const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );
	}
	{ // mrpt::poses::CPoint2DPDF file:mrpt/poses/CPoint2DPDF.h line:33
		pybind11::class_<mrpt::poses::CPoint2DPDF, std::shared_ptr<mrpt::poses::CPoint2DPDF>, PyCallBack_mrpt_poses_CPoint2DPDF, mrpt::serialization::CSerializable, mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint2D,2UL>> cl(M("mrpt::poses"), "CPoint2DPDF", "Declares a class that represents a Probability Distribution function (PDF)\n of a 2D point (x,y).\n   This class is just the base class for unifying many diferent\n    ways this PDF can be implemented.\n\n  For convenience, a pose composition is also defined for any\n    PDF derived class, changeCoordinatesReference, in the form of a method\n rather than an operator.\n\n  For a similar class for 6D poses (a 3D point with attitude), see CPose3DPDF\n\n  See also:\n  [probabilistic spatial representations](tutorial-pdf-over-poses.html)\n\n \n\n \n CPoint2D, CPointPDF");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPoint2DPDF(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPoint2DPDF const &>());
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPoint2DPDF::*)() const) &mrpt::poses::CPoint2DPDF::GetRuntimeClass, "C++: mrpt::poses::CPoint2DPDF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPoint2DPDF::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPoint2DPDF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("copyFrom", (void (mrpt::poses::CPoint2DPDF::*)(const class mrpt::poses::CPoint2DPDF &)) &mrpt::poses::CPoint2DPDF::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations)\n\nC++: mrpt::poses::CPoint2DPDF::copyFrom(const class mrpt::poses::CPoint2DPDF &) --> void", pybind11::arg("o"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPoint2DPDF::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPoint2DPDF::changeCoordinatesReference, "C++: mrpt::poses::CPoint2DPDF::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("bayesianFusion", [](mrpt::poses::CPoint2DPDF &o, const class mrpt::poses::CPoint2DPDF & a0, const class mrpt::poses::CPoint2DPDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPoint2DPDF::*)(const class mrpt::poses::CPoint2DPDF &, const class mrpt::poses::CPoint2DPDF &, const double)) &mrpt::poses::CPoint2DPDF::bayesianFusion, "Bayesian fusion of two point distributions (product of two\n distributions->new distribution), then save the result in this object\n (WARNING: See implementing classes to see classes that can and cannot be\n mixtured!)\n \n\n The first distribution to fuse\n \n\n The second distribution to fuse\n \n\n If set to different of 0, the result of\n very separate Gaussian modes (that will result in negligible components)\n in SOGs will be dropped to reduce the number of modes in the output.\n\nC++: mrpt::poses::CPoint2DPDF::bayesianFusion(const class mrpt::poses::CPoint2DPDF &, const class mrpt::poses::CPoint2DPDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def_static("is_3D", (bool (*)()) &mrpt::poses::CPoint2DPDF::is_3D, "C++: mrpt::poses::CPoint2DPDF::is_3D() --> bool");
		cl.def_static("is_PDF", (bool (*)()) &mrpt::poses::CPoint2DPDF::is_PDF, "C++: mrpt::poses::CPoint2DPDF::is_PDF() --> bool");
		cl.def("assign", (class mrpt::poses::CPoint2DPDF & (mrpt::poses::CPoint2DPDF::*)(const class mrpt::poses::CPoint2DPDF &)) &mrpt::poses::CPoint2DPDF::operator=, "C++: mrpt::poses::CPoint2DPDF::operator=(const class mrpt::poses::CPoint2DPDF &) --> class mrpt::poses::CPoint2DPDF &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPoint2DPDFGaussian file:mrpt/poses/CPoint2DPDFGaussian.h line:20
		pybind11::class_<mrpt::poses::CPoint2DPDFGaussian, std::shared_ptr<mrpt::poses::CPoint2DPDFGaussian>, PyCallBack_mrpt_poses_CPoint2DPDFGaussian, mrpt::poses::CPoint2DPDF> cl(M("mrpt::poses"), "CPoint2DPDFGaussian", "A gaussian distribution for 2D points. Also a method for bayesian fusion is\n provided.\n \n\n\n \n CPoint2DPDF");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPoint2DPDFGaussian(); }, [](){ return new PyCallBack_mrpt_poses_CPoint2DPDFGaussian(); } ) );
		cl.def( pybind11::init<const class mrpt::poses::CPoint2D &>(), pybind11::arg("init_Mean") );

		cl.def( pybind11::init<const class mrpt::poses::CPoint2D &, const class mrpt::math::CMatrixFixed<double, 2, 2> &>(), pybind11::arg("init_Mean"), pybind11::arg("init_Cov") );

		cl.def_readwrite("mean", &mrpt::poses::CPoint2DPDFGaussian::mean);
		cl.def_readwrite("cov", &mrpt::poses::CPoint2DPDFGaussian::cov);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPoint2DPDFGaussian::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPoint2DPDFGaussian::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPoint2DPDFGaussian::*)() const) &mrpt::poses::CPoint2DPDFGaussian::GetRuntimeClass, "C++: mrpt::poses::CPoint2DPDFGaussian::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPoint2DPDFGaussian::*)() const) &mrpt::poses::CPoint2DPDFGaussian::clone, "C++: mrpt::poses::CPoint2DPDFGaussian::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPoint2DPDFGaussian::CreateObject, "C++: mrpt::poses::CPoint2DPDFGaussian::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getMean", (void (mrpt::poses::CPoint2DPDFGaussian::*)(class mrpt::poses::CPoint2D &) const) &mrpt::poses::CPoint2DPDFGaussian::getMean, "Returns an estimate of the point, (the mean, or mathematical expectation\n of the PDF) \n\nC++: mrpt::poses::CPoint2DPDFGaussian::getMean(class mrpt::poses::CPoint2D &) const --> void", pybind11::arg("p"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 2, 2>, class mrpt::poses::CPoint2D> (mrpt::poses::CPoint2DPDFGaussian::*)() const) &mrpt::poses::CPoint2DPDFGaussian::getCovarianceAndMean, "Returns an estimate of the point covariance matrix (2x2 cov matrix) and\n the mean, both at once. \n\n getMean \n\nC++: mrpt::poses::CPoint2DPDFGaussian::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 2, 2>, class mrpt::poses::CPoint2D>");
		cl.def("copyFrom", (void (mrpt::poses::CPoint2DPDFGaussian::*)(const class mrpt::poses::CPoint2DPDF &)) &mrpt::poses::CPoint2DPDFGaussian::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPoint2DPDFGaussian::copyFrom(const class mrpt::poses::CPoint2DPDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPoint2DPDFGaussian::*)(const std::string &) const) &mrpt::poses::CPoint2DPDFGaussian::saveToTextFile, "Save PDF's particles to a text file, containing the 2D pose in the first\n line, then the covariance matrix in next 3 lines \n\nC++: mrpt::poses::CPoint2DPDFGaussian::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPoint2DPDFGaussian::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPoint2DPDFGaussian::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. Both the mean value and the covariance matrix\n are updated correctly. \n\nC++: mrpt::poses::CPoint2DPDFGaussian::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPoint2DPDFGaussian::*)(const class mrpt::poses::CPoint2DPDFGaussian &, const class mrpt::poses::CPoint2DPDFGaussian &)) &mrpt::poses::CPoint2DPDFGaussian::bayesianFusion, "Bayesian fusion of two points gauss. distributions, then save the result\nin this object.\n  The process is as follows:\n		- (x1,S1): Mean and variance of the p1 distribution.\n		- (x2,S2): Mean and variance of the p2 distribution.\n		- (x,S): Mean and variance of the resulting distribution.\n\n    \n\n    \n\n   \n\nC++: mrpt::poses::CPoint2DPDFGaussian::bayesianFusion(const class mrpt::poses::CPoint2DPDFGaussian &, const class mrpt::poses::CPoint2DPDFGaussian &) --> void", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("productIntegralWith", (double (mrpt::poses::CPoint2DPDFGaussian::*)(const class mrpt::poses::CPoint2DPDFGaussian &) const) &mrpt::poses::CPoint2DPDFGaussian::productIntegralWith, "Computes the \"correspondence likelihood\" of this PDF with another one:\n This is implemented as the integral from -inf to +inf of the product of\n both PDF.\n The resulting number is >=0.\n \n\n productIntegralNormalizedWith\n \n\n std::exception On errors like covariance matrix with null\n determinant, etc...\n\nC++: mrpt::poses::CPoint2DPDFGaussian::productIntegralWith(const class mrpt::poses::CPoint2DPDFGaussian &) const --> double", pybind11::arg("p"));
		cl.def("productIntegralNormalizedWith", (double (mrpt::poses::CPoint2DPDFGaussian::*)(const class mrpt::poses::CPoint2DPDFGaussian &) const) &mrpt::poses::CPoint2DPDFGaussian::productIntegralNormalizedWith, "Computes the \"correspondence likelihood\" of this PDF with another one:\n This is implemented as the integral from -inf to +inf of the product of\n both PDF.\n The resulting number is in the range [0,1].\n  Note that the resulting value is in fact\n  \n\n\n  , with \n being the square Mahalanobis distance between the\n two pdfs.\n \n\n productIntegralWith\n \n\n std::exception On errors like covariance matrix with null\n determinant, etc...\n\nC++: mrpt::poses::CPoint2DPDFGaussian::productIntegralNormalizedWith(const class mrpt::poses::CPoint2DPDFGaussian &) const --> double", pybind11::arg("p"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPoint2DPDFGaussian::*)(class mrpt::poses::CPoint2D &) const) &mrpt::poses::CPoint2DPDFGaussian::drawSingleSample, "Draw a sample from the pdf \n\nC++: mrpt::poses::CPoint2DPDFGaussian::drawSingleSample(class mrpt::poses::CPoint2D &) const --> void", pybind11::arg("outSample"));
		cl.def("bayesianFusion", [](mrpt::poses::CPoint2DPDFGaussian &o, const class mrpt::poses::CPoint2DPDF & a0, const class mrpt::poses::CPoint2DPDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPoint2DPDFGaussian::*)(const class mrpt::poses::CPoint2DPDF &, const class mrpt::poses::CPoint2DPDF &, const double)) &mrpt::poses::CPoint2DPDFGaussian::bayesianFusion, "Bayesian fusion of two point distributions (product of two\n distributions->new distribution), then save the result in this object\n (WARNING: See implementing classes to see classes that can and cannot be\n mixtured!)\n \n\n The first distribution to fuse\n \n\n The second distribution to fuse\n \n\n If set to different of 0, the result of\n very separate Gaussian modes (that will result in negligible components)\n in SOGs will be dropped to reduce the number of modes in the output.\n\nC++: mrpt::poses::CPoint2DPDFGaussian::bayesianFusion(const class mrpt::poses::CPoint2DPDF &, const class mrpt::poses::CPoint2DPDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("mahalanobisDistanceTo", (double (mrpt::poses::CPoint2DPDFGaussian::*)(const class mrpt::poses::CPoint2DPDFGaussian &) const) &mrpt::poses::CPoint2DPDFGaussian::mahalanobisDistanceTo, "Returns the Mahalanobis distance from this PDF to another PDF, that is,\n it's evaluation at (0,0,0) \n\nC++: mrpt::poses::CPoint2DPDFGaussian::mahalanobisDistanceTo(const class mrpt::poses::CPoint2DPDFGaussian &) const --> double", pybind11::arg("other"));
		cl.def("mahalanobisDistanceToPoint", (double (mrpt::poses::CPoint2DPDFGaussian::*)(const double, const double) const) &mrpt::poses::CPoint2DPDFGaussian::mahalanobisDistanceToPoint, "Returns the Mahalanobis distance from this PDF to some point \n\nC++: mrpt::poses::CPoint2DPDFGaussian::mahalanobisDistanceToPoint(const double, const double) const --> double", pybind11::arg("x"), pybind11::arg("y"));
		cl.def("assign", (class mrpt::poses::CPoint2DPDFGaussian & (mrpt::poses::CPoint2DPDFGaussian::*)(const class mrpt::poses::CPoint2DPDFGaussian &)) &mrpt::poses::CPoint2DPDFGaussian::operator=, "C++: mrpt::poses::CPoint2DPDFGaussian::operator=(const class mrpt::poses::CPoint2DPDFGaussian &) --> class mrpt::poses::CPoint2DPDFGaussian &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
