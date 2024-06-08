#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CProbabilityDensityFunction.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
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

// mrpt::poses::CPointPDF file:mrpt/poses/CPointPDF.h line:35
struct PyCallBack_mrpt_poses_CPointPDF : public mrpt::poses::CPointPDF {
	using mrpt::poses::CPointPDF::CPointPDF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPointPDF::GetRuntimeClass();
	}
	void copyFrom(const class mrpt::poses::CPointPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPointPDF::copyFrom\"");
	}
	void bayesianFusion(const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPointPDF::bayesianFusion\"");
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPointPDF::changeCoordinatesReference\"");
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "serializeGetVersion");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "serializeTo");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "serializeFrom");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "clone");
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
	void getMean(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "getMean");
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
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "getCovarianceAndMean");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "getInformationMatrix");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "saveToTextFile");
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
	void drawSingleSample(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDF *>(this), "drawSingleSample");
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

// mrpt::poses::CPointPDFGaussian file:mrpt/poses/CPointPDFGaussian.h line:22
struct PyCallBack_mrpt_poses_CPointPDFGaussian : public mrpt::poses::CPointPDFGaussian {
	using mrpt::poses::CPointPDFGaussian::CPointPDFGaussian;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPointPDFGaussian::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPointPDFGaussian::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPointPDFGaussian::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFGaussian::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFGaussian::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFGaussian::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPointPDFGaussian::getCovarianceAndMean();
	}
	void copyFrom(const class mrpt::poses::CPointPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFGaussian::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPointPDFGaussian::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFGaussian::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFGaussian::drawSingleSample(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFGaussian::bayesianFusion(a0, a1, a2);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFGaussian *>(this), "getInformationMatrix");
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

// mrpt::poses::CPointPDFParticles file:mrpt/poses/CPointPDFParticles.h line:24
struct PyCallBack_mrpt_poses_CPointPDFParticles : public mrpt::poses::CPointPDFParticles {
	using mrpt::poses::CPointPDFParticles::CPointPDFParticles;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPointPDFParticles::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPointPDFParticles::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPointPDFParticles::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFParticles::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFParticles::serializeFrom(a0, a1);
	}
	void getMean(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFParticles::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPointPDFParticles::getCovarianceAndMean();
	}
	void copyFrom(const class mrpt::poses::CPointPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFParticles::copyFrom(a0);
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPointPDFParticles::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFParticles::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPoint3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFParticles::drawSingleSample(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPointPDFParticles::bayesianFusion(a0, a1, a2);
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "getInformationMatrix");
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
	double getW(size_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "getW");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParticleFilterDataImpl::getW(a0);
	}
	void setW(size_t a0, double a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "setW");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParticleFilterDataImpl::setW(a0, a1);
	}
	size_t particlesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "particlesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CParticleFilterDataImpl::particlesCount();
	}
	double normalizeWeights(double * a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "normalizeWeights");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParticleFilterDataImpl::normalizeWeights(a0);
	}
	double ESS() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "ESS");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CParticleFilterDataImpl::ESS();
	}
	void prediction_and_update_pfStandardProposal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "prediction_and_update_pfStandardProposal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParticleFilterCapable::prediction_and_update_pfStandardProposal(a0, a1, a2);
	}
	void prediction_and_update_pfAuxiliaryPFStandard(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "prediction_and_update_pfAuxiliaryPFStandard");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParticleFilterCapable::prediction_and_update_pfAuxiliaryPFStandard(a0, a1, a2);
	}
	void prediction_and_update_pfOptimalProposal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "prediction_and_update_pfOptimalProposal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParticleFilterCapable::prediction_and_update_pfOptimalProposal(a0, a1, a2);
	}
	void prediction_and_update_pfAuxiliaryPFOptimal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPointPDFParticles *>(this), "prediction_and_update_pfAuxiliaryPFOptimal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CParticleFilterCapable::prediction_and_update_pfAuxiliaryPFOptimal(a0, a1, a2);
	}
};

void bind_mrpt_poses_CPointPDF(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPointPDF file:mrpt/poses/CPointPDF.h line:35
		pybind11::class_<mrpt::poses::CPointPDF, std::shared_ptr<mrpt::poses::CPointPDF>, PyCallBack_mrpt_poses_CPointPDF, mrpt::serialization::CSerializable, mrpt::math::CProbabilityDensityFunction<mrpt::poses::CPoint3D,3UL>> cl(M("mrpt::poses"), "CPointPDF", "Declares a class that represents a Probability Distribution\n    function (PDF) of a 3D point (x,y,z).\n   This class is just the base class for unifying many diferent\n    ways this PDF can be implemented.\n\n  For convenience, a pose composition is also defined for any\n    PDF derived class, changeCoordinatesReference, in the form of a method\n rather than an operator.\n\n  For a similar class for 6D poses (a 3D point with attitude), see CPose3DPDF\n\n  See also:\n  [probabilistic spatial representations](tutorial-pdf-over-poses.html)\n\n \n CPoint3D\n \n\n\n ");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPointPDF const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPointPDF(); } ) );
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPointPDF::*)() const) &mrpt::poses::CPointPDF::GetRuntimeClass, "C++: mrpt::poses::CPointPDF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPointPDF::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPointPDF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("copyFrom", (void (mrpt::poses::CPointPDF::*)(const class mrpt::poses::CPointPDF &)) &mrpt::poses::CPointPDF::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations)\n\nC++: mrpt::poses::CPointPDF::copyFrom(const class mrpt::poses::CPointPDF &) --> void", pybind11::arg("o"));
		cl.def("bayesianFusion", [](mrpt::poses::CPointPDF &o, const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPointPDF::*)(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double)) &mrpt::poses::CPointPDF::bayesianFusion, "Bayesian fusion of two point distributions (product of two\n distributions->new distribution), then save the result in this object\n (WARNING: See implementing classes to see classes that can and cannot be\n mixtured!)\n \n\n The first distribution to fuse\n \n\n The second distribution to fuse\n \n\n If set to different of 0, the result of\n very separate Gaussian modes (that will result in negligible components)\n in SOGs will be dropped to reduce the number of modes in the output.\n\nC++: mrpt::poses::CPointPDF::bayesianFusion(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPointPDF::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPointPDF::changeCoordinatesReference, "C++: mrpt::poses::CPointPDF::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def_static("is_3D", (bool (*)()) &mrpt::poses::CPointPDF::is_3D, "C++: mrpt::poses::CPointPDF::is_3D() --> bool");
		cl.def_static("is_PDF", (bool (*)()) &mrpt::poses::CPointPDF::is_PDF, "C++: mrpt::poses::CPointPDF::is_PDF() --> bool");
		cl.def("assign", (class mrpt::poses::CPointPDF & (mrpt::poses::CPointPDF::*)(const class mrpt::poses::CPointPDF &)) &mrpt::poses::CPointPDF::operator=, "C++: mrpt::poses::CPointPDF::operator=(const class mrpt::poses::CPointPDF &) --> class mrpt::poses::CPointPDF &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPointPDFGaussian file:mrpt/poses/CPointPDFGaussian.h line:22
		pybind11::class_<mrpt::poses::CPointPDFGaussian, std::shared_ptr<mrpt::poses::CPointPDFGaussian>, PyCallBack_mrpt_poses_CPointPDFGaussian, mrpt::poses::CPointPDF> cl(M("mrpt::poses"), "CPointPDFGaussian", "A gaussian distribution for 3D points. Also a method for bayesian fusion is\n provided.\n\n \n CPointPDF\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPointPDFGaussian(); }, [](){ return new PyCallBack_mrpt_poses_CPointPDFGaussian(); } ) );
		cl.def( pybind11::init<const class mrpt::poses::CPoint3D &>(), pybind11::arg("init_Mean") );

		cl.def( pybind11::init<const class mrpt::poses::CPoint3D &, const class mrpt::math::CMatrixFixed<double, 3, 3> &>(), pybind11::arg("init_Mean"), pybind11::arg("init_Cov") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPointPDFGaussian const &o){ return new PyCallBack_mrpt_poses_CPointPDFGaussian(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPointPDFGaussian const &o){ return new mrpt::poses::CPointPDFGaussian(o); } ) );
		cl.def_readwrite("mean", &mrpt::poses::CPointPDFGaussian::mean);
		cl.def_readwrite("cov", &mrpt::poses::CPointPDFGaussian::cov);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPointPDFGaussian::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPointPDFGaussian::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPointPDFGaussian::*)() const) &mrpt::poses::CPointPDFGaussian::GetRuntimeClass, "C++: mrpt::poses::CPointPDFGaussian::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPointPDFGaussian::*)() const) &mrpt::poses::CPointPDFGaussian::clone, "C++: mrpt::poses::CPointPDFGaussian::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPointPDFGaussian::CreateObject, "C++: mrpt::poses::CPointPDFGaussian::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("getMean", (void (mrpt::poses::CPointPDFGaussian::*)(class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPointPDFGaussian::getMean, "C++: mrpt::poses::CPointPDFGaussian::getMean(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("p"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D> (mrpt::poses::CPointPDFGaussian::*)() const) &mrpt::poses::CPointPDFGaussian::getCovarianceAndMean, "C++: mrpt::poses::CPointPDFGaussian::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>");
		cl.def("copyFrom", (void (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPointPDF &)) &mrpt::poses::CPointPDFGaussian::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPointPDFGaussian::copyFrom(const class mrpt::poses::CPointPDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPointPDFGaussian::*)(const std::string &) const) &mrpt::poses::CPointPDFGaussian::saveToTextFile, "Save PDF's particles to a text file, containing the 2D pose in the first\n line, then the covariance matrix in next 3 lines. \n\nC++: mrpt::poses::CPointPDFGaussian::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPointPDFGaussian::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. Both the mean value and the covariance matrix\n are updated correctly. \n\nC++: mrpt::poses::CPointPDFGaussian::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPointPDFGaussian &, const class mrpt::poses::CPointPDFGaussian &)) &mrpt::poses::CPointPDFGaussian::bayesianFusion, "Bayesian fusion of two points gauss. distributions, then save the result\nin this object.\n  The process is as follows:\n		- (x1,S1): Mean and variance of the p1 distribution.\n		- (x2,S2): Mean and variance of the p2 distribution.\n		- (x,S): Mean and variance of the resulting distribution.\n\n    \n\n    \n\n   \n\nC++: mrpt::poses::CPointPDFGaussian::bayesianFusion(const class mrpt::poses::CPointPDFGaussian &, const class mrpt::poses::CPointPDFGaussian &) --> void", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("productIntegralWith", (double (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPointPDFGaussian &) const) &mrpt::poses::CPointPDFGaussian::productIntegralWith, "Computes the \"correspondence likelihood\" of this PDF with another one:\n This is implemented as the integral from -inf to +inf of the product of\n both PDF.\n The resulting number is >=0.\n \n\n productIntegralNormalizedWith\n \n\n std::exception On errors like covariance matrix with null\n determinant, etc...\n\nC++: mrpt::poses::CPointPDFGaussian::productIntegralWith(const class mrpt::poses::CPointPDFGaussian &) const --> double", pybind11::arg("p"));
		cl.def("productIntegralWith2D", (double (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPointPDFGaussian &) const) &mrpt::poses::CPointPDFGaussian::productIntegralWith2D, "Computes the \"correspondence likelihood\" of this PDF with another one:\n This is implemented as the integral from -inf to +inf of the product of\n both PDF.\n The resulting number is >=0.\n NOTE: This version ignores the \"z\" coordinates!!\n \n\n productIntegralNormalizedWith\n \n\n std::exception On errors like covariance matrix with null\n determinant, etc...\n\nC++: mrpt::poses::CPointPDFGaussian::productIntegralWith2D(const class mrpt::poses::CPointPDFGaussian &) const --> double", pybind11::arg("p"));
		cl.def("productIntegralNormalizedWith", (double (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPointPDFGaussian &) const) &mrpt::poses::CPointPDFGaussian::productIntegralNormalizedWith, "Computes the \"correspondence likelihood\" of this PDF with another one:\n This is implemented as the integral from -inf to +inf of the product of\n both PDF.\n The resulting number is in the range [0,1]\n  Note that the resulting value is in fact\n  \n\n\n  , with \n being the square Mahalanobis distance between the\n two pdfs.\n \n\n productIntegralWith\n \n\n std::exception On errors like covariance matrix with null\n determinant, etc...\n\nC++: mrpt::poses::CPointPDFGaussian::productIntegralNormalizedWith(const class mrpt::poses::CPointPDFGaussian &) const --> double", pybind11::arg("p"));
		cl.def("productIntegralNormalizedWith2D", (double (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPointPDFGaussian &) const) &mrpt::poses::CPointPDFGaussian::productIntegralNormalizedWith2D, "Computes the \"correspondence likelihood\" of this PDF with another one:\n This is implemented as the integral from -inf to +inf of the product of\n both PDF.\n The resulting number is in the range [0,1]. This versions ignores the\n \"z\" coordinate.\n\n  Note that the resulting value is in fact\n  \n\n\n  , with \n being the square Mahalanobis distance between the\n two pdfs.\n \n\n productIntegralWith\n \n\n std::exception On errors like covariance matrix with null\n determinant, etc...\n\nC++: mrpt::poses::CPointPDFGaussian::productIntegralNormalizedWith2D(const class mrpt::poses::CPointPDFGaussian &) const --> double", pybind11::arg("p"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPointPDFGaussian::*)(class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPointPDFGaussian::drawSingleSample, "Draw a sample from the pdf \n\nC++: mrpt::poses::CPointPDFGaussian::drawSingleSample(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("outSample"));
		cl.def("bayesianFusion", [](mrpt::poses::CPointPDFGaussian &o, const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double)) &mrpt::poses::CPointPDFGaussian::bayesianFusion, "Bayesian fusion of two point distributions (product of two\n distributions->new distribution), then save the result in this object\n (WARNING: See implementing classes to see classes that can and cannot be\n mixtured!)\n \n\n The first distribution to fuse\n \n\n The second distribution to fuse\n \n\n If set to different of 0, the result of\n very separate Gaussian modes (that will result in negligible components)\n in SOGs will be dropped to reduce the number of modes in the output.\n\nC++: mrpt::poses::CPointPDFGaussian::bayesianFusion(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("mahalanobisDistanceTo", [](mrpt::poses::CPointPDFGaussian const &o, const class mrpt::poses::CPointPDFGaussian & a0) -> double { return o.mahalanobisDistanceTo(a0); }, "", pybind11::arg("other"));
		cl.def("mahalanobisDistanceTo", (double (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPointPDFGaussian &, bool) const) &mrpt::poses::CPointPDFGaussian::mahalanobisDistanceTo, "Returns the Mahalanobis distance from this PDF to another PDF, that is,\n it's evaluation at (0,0,0) \n\nC++: mrpt::poses::CPointPDFGaussian::mahalanobisDistanceTo(const class mrpt::poses::CPointPDFGaussian &, bool) const --> double", pybind11::arg("other"), pybind11::arg("only_2D"));
		cl.def("assign", (class mrpt::poses::CPointPDFGaussian & (mrpt::poses::CPointPDFGaussian::*)(const class mrpt::poses::CPointPDFGaussian &)) &mrpt::poses::CPointPDFGaussian::operator=, "C++: mrpt::poses::CPointPDFGaussian::operator=(const class mrpt::poses::CPointPDFGaussian &) --> class mrpt::poses::CPointPDFGaussian &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPointPDFParticles file:mrpt/poses/CPointPDFParticles.h line:24
		pybind11::class_<mrpt::poses::CPointPDFParticles, std::shared_ptr<mrpt::poses::CPointPDFParticles>, PyCallBack_mrpt_poses_CPointPDFParticles, mrpt::poses::CPointPDF, mrpt::bayes::CParticleFilterData<mrpt::math::TPoint3D_<float>,mrpt::bayes::particle_storage_mode::POINTER>, mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPointPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPoint3D_<float>, mrpt::bayes::particle_storage_mode::POINTER> >>> cl(M("mrpt::poses"), "CPointPDFParticles", "A probability distribution of a 2D/3D point, represented as a set of random\n samples (particles).\n \n\n CPointPDF\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPointPDFParticles(); }, [](){ return new PyCallBack_mrpt_poses_CPointPDFParticles(); } ), "doc");
		cl.def( pybind11::init<size_t>(), pybind11::arg("numParticles") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPointPDFParticles const &o){ return new PyCallBack_mrpt_poses_CPointPDFParticles(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPointPDFParticles const &o){ return new mrpt::poses::CPointPDFParticles(o); } ) );
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPointPDFParticles::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPointPDFParticles::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPointPDFParticles::*)() const) &mrpt::poses::CPointPDFParticles::GetRuntimeClass, "C++: mrpt::poses::CPointPDFParticles::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPointPDFParticles::*)() const) &mrpt::poses::CPointPDFParticles::clone, "C++: mrpt::poses::CPointPDFParticles::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPointPDFParticles::CreateObject, "C++: mrpt::poses::CPointPDFParticles::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clear", (void (mrpt::poses::CPointPDFParticles::*)()) &mrpt::poses::CPointPDFParticles::clear, "Clear all the particles (free memory) \n\nC++: mrpt::poses::CPointPDFParticles::clear() --> void");
		cl.def("setSize", [](mrpt::poses::CPointPDFParticles &o, size_t const & a0) -> void { return o.setSize(a0); }, "", pybind11::arg("numberParticles"));
		cl.def("setSize", (void (mrpt::poses::CPointPDFParticles::*)(size_t, const struct mrpt::math::TPoint3D_<float> &)) &mrpt::poses::CPointPDFParticles::setSize, "Erase all the previous particles and change the number of particles,\n with a given initial value  \n\nC++: mrpt::poses::CPointPDFParticles::setSize(size_t, const struct mrpt::math::TPoint3D_<float> &) --> void", pybind11::arg("numberParticles"), pybind11::arg("defaultValue"));
		cl.def("size", (size_t (mrpt::poses::CPointPDFParticles::*)() const) &mrpt::poses::CPointPDFParticles::size, "Returns the number of particles \n\nC++: mrpt::poses::CPointPDFParticles::size() const --> size_t");
		cl.def("getMean", (void (mrpt::poses::CPointPDFParticles::*)(class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPointPDFParticles::getMean, "C++: mrpt::poses::CPointPDFParticles::getMean(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("mean_point"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D> (mrpt::poses::CPointPDFParticles::*)() const) &mrpt::poses::CPointPDFParticles::getCovarianceAndMean, "C++: mrpt::poses::CPointPDFParticles::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPoint3D>");
		cl.def("copyFrom", (void (mrpt::poses::CPointPDFParticles::*)(const class mrpt::poses::CPointPDF &)) &mrpt::poses::CPointPDFParticles::copyFrom, "Copy operator, translating if necesary (for example, between particles\n and gaussian representations) \n\nC++: mrpt::poses::CPointPDFParticles::copyFrom(const class mrpt::poses::CPointPDF &) --> void", pybind11::arg("o"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPointPDFParticles::*)(const std::string &) const) &mrpt::poses::CPointPDFParticles::saveToTextFile, "Save PDF's particles to a text file, where each line is: X Y Z LOG_W \n\nC++: mrpt::poses::CPointPDFParticles::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPointPDFParticles::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPointPDFParticles::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. Both the mean value and the covariance matrix\n are updated correctly.  \n\nC++: mrpt::poses::CPointPDFParticles::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("computeKurtosis", (double (mrpt::poses::CPointPDFParticles::*)()) &mrpt::poses::CPointPDFParticles::computeKurtosis, "Compute the kurtosis of the distribution \n\nC++: mrpt::poses::CPointPDFParticles::computeKurtosis() --> double");
		cl.def("drawSingleSample", (void (mrpt::poses::CPointPDFParticles::*)(class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPointPDFParticles::drawSingleSample, "Draw a sample from the pdf \n\nC++: mrpt::poses::CPointPDFParticles::drawSingleSample(class mrpt::poses::CPoint3D &) const --> void", pybind11::arg("outSample"));
		cl.def("bayesianFusion", [](mrpt::poses::CPointPDFParticles &o, const class mrpt::poses::CPointPDF & a0, const class mrpt::poses::CPointPDF & a1) -> void { return o.bayesianFusion(a0, a1); }, "", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("bayesianFusion", (void (mrpt::poses::CPointPDFParticles::*)(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double)) &mrpt::poses::CPointPDFParticles::bayesianFusion, "Bayesian fusion of two point distributions (product of two\n distributions->new distribution), then save the result in this object\n (WARNING: See implementing classes to see classes that can and cannot be\n mixtured!)\n \n\n The first distribution to fuse\n \n\n The second distribution to fuse\n \n\n If set to different of 0, the result of\n very separate Gaussian modes (that will result in negligible components)\n in SOGs will be dropped to reduce the number of modes in the output.\n\nC++: mrpt::poses::CPointPDFParticles::bayesianFusion(const class mrpt::poses::CPointPDF &, const class mrpt::poses::CPointPDF &, const double) --> void", pybind11::arg("p1"), pybind11::arg("p2"), pybind11::arg("minMahalanobisDistToDrop"));
		cl.def("assign", (class mrpt::poses::CPointPDFParticles & (mrpt::poses::CPointPDFParticles::*)(const class mrpt::poses::CPointPDFParticles &)) &mrpt::poses::CPointPDFParticles::operator=, "C++: mrpt::poses::CPointPDFParticles::operator=(const class mrpt::poses::CPointPDFParticles &) --> class mrpt::poses::CPointPDFParticles &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
