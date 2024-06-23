#include <deque>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/bayes/CProbabilityParticle.h>
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
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/slam/PF_implementations_data.h>
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

// mrpt::slam::CMonteCarloLocalization3D file:mrpt/slam/CMonteCarloLocalization3D.h line:30
struct PyCallBack_mrpt_slam_CMonteCarloLocalization3D : public mrpt::slam::CMonteCarloLocalization3D {
	using mrpt::slam::CMonteCarloLocalization3D::CMonteCarloLocalization3D;

	void prediction_and_update_pfStandardProposal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "prediction_and_update_pfStandardProposal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMonteCarloLocalization3D::prediction_and_update_pfStandardProposal(a0, a1, a2);
	}
	void prediction_and_update_pfAuxiliaryPFStandard(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "prediction_and_update_pfAuxiliaryPFStandard");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMonteCarloLocalization3D::prediction_and_update_pfAuxiliaryPFStandard(a0, a1, a2);
	}
	void prediction_and_update_pfAuxiliaryPFOptimal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "prediction_and_update_pfAuxiliaryPFOptimal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMonteCarloLocalization3D::prediction_and_update_pfAuxiliaryPFOptimal(a0, a1, a2);
	}
	struct mrpt::math::TPose3D getLastPose(size_t a0, bool & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "getLastPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose3D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose3D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose3D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose3D>(std::move(o));
		}
		return CMonteCarloLocalization3D::getLastPose(a0, a1);
	}
	void PF_SLAM_implementation_custom_update_particle_with_new_pose(struct mrpt::math::TPose3D * a0, const struct mrpt::math::TPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "PF_SLAM_implementation_custom_update_particle_with_new_pose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMonteCarloLocalization3D::PF_SLAM_implementation_custom_update_particle_with_new_pose(a0, a1);
	}
	double PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a0, size_t a1, const class mrpt::obs::CSensoryFrame & a2, const class mrpt::poses::CPose3D & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "PF_SLAM_computeObservationLikelihoodForParticle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CMonteCarloLocalization3D::PF_SLAM_computeObservationLikelihoodForParticle(a0, a1, a2, a3);
	}
	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DPDFParticles::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3DPDFParticles::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3DPDFParticles::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFParticles::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFParticles::serializeFrom(a0, a1);
	}
	void copyFrom(const class mrpt::poses::CPose3DPDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFParticles::copyFrom(a0);
	}
	void getMean(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFParticles::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPose3DPDFParticles::getCovarianceAndMean();
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPose3DPDFParticles::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFParticles::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose3D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFParticles::drawSingleSample(a0);
	}
	void inverse(class mrpt::poses::CPose3DPDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFParticles::inverse(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPose3DPDF & a0, const class mrpt::poses::CPose3DPDF & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DPDFParticles::bayesianFusion(a0, a1);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPose3DPDFParticles::asString();
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "getInformationMatrix");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "getW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "setW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "particlesCount");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "normalizeWeights");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "ESS");
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
	void prediction_and_update_pfOptimalProposal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "prediction_and_update_pfOptimalProposal");
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
	bool PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> > & a0, const class mrpt::obs::CSensoryFrame * a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "PF_SLAM_implementation_doWeHaveValidObservations");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return PF_implementation::PF_SLAM_implementation_doWeHaveValidObservations(a0, a1);
	}
	bool PF_SLAM_implementation_skipRobotMovement() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization3D *>(this), "PF_SLAM_implementation_skipRobotMovement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return PF_implementation::PF_SLAM_implementation_skipRobotMovement();
	}
};

void bind_mrpt_slam_CMonteCarloLocalization3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::slam::CMonteCarloLocalization3D file:mrpt/slam/CMonteCarloLocalization3D.h line:30
		pybind11::class_<mrpt::slam::CMonteCarloLocalization3D, std::shared_ptr<mrpt::slam::CMonteCarloLocalization3D>, PyCallBack_mrpt_slam_CMonteCarloLocalization3D, mrpt::poses::CPose3DPDFParticles, mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>> cl(M("mrpt::slam"), "CMonteCarloLocalization3D", "Declares a class that represents a Probability Density Function (PDF) over a\n 3D pose (x,y,phi,yaw,pitch,roll), using a set of weighted samples.\n\n  This class also implements particle filtering for robot localization. See\n the MRPT\n   application \"app/pf-localization\" for an example of usage.\n\n \n CMonteCarloLocalization2D, CPose2D, CPosePDF, CPoseGaussianPDF,\n CParticleFilterCapable\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::CMonteCarloLocalization3D(); }, [](){ return new PyCallBack_mrpt_slam_CMonteCarloLocalization3D(); } ), "doc");
		cl.def( pybind11::init<size_t>(), pybind11::arg("M") );

		cl.def_readwrite("options", &mrpt::slam::CMonteCarloLocalization3D::options);
		cl.def("prediction_and_update_pfStandardProposal", (void (mrpt::slam::CMonteCarloLocalization3D::*)(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &)) &mrpt::slam::CMonteCarloLocalization3D::prediction_and_update_pfStandardProposal, "Update the m_particles, predicting the posterior of robot pose and map\n after a movement command.\n  This method has additional configuration parameters in \"options\".\n  Performs the update stage of the RBPF, using the sensed CSensoryFrame:\n\n   \n This is a pointer to CActionCollection, containing the\n pose change the robot has been commanded.\n   \n\n This must be a pointer to a CSensoryFrame object,\n with robot sensed observations.\n\n \n options\n\nC++: mrpt::slam::CMonteCarloLocalization3D::prediction_and_update_pfStandardProposal(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) --> void", pybind11::arg("action"), pybind11::arg("observation"), pybind11::arg("PF_options"));
		cl.def("prediction_and_update_pfAuxiliaryPFStandard", (void (mrpt::slam::CMonteCarloLocalization3D::*)(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &)) &mrpt::slam::CMonteCarloLocalization3D::prediction_and_update_pfAuxiliaryPFStandard, "Update the m_particles, predicting the posterior of robot pose and map\n after a movement command.\n  This method has additional configuration parameters in \"options\".\n  Performs the update stage of the RBPF, using the sensed CSensoryFrame:\n\n   \n This is a pointer to CActionCollection, containing the\n pose change the robot has been commanded.\n   \n\n This must be a pointer to a CSensoryFrame object,\n with robot sensed observations.\n\n \n options\n\nC++: mrpt::slam::CMonteCarloLocalization3D::prediction_and_update_pfAuxiliaryPFStandard(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) --> void", pybind11::arg("action"), pybind11::arg("observation"), pybind11::arg("PF_options"));
		cl.def("prediction_and_update_pfAuxiliaryPFOptimal", (void (mrpt::slam::CMonteCarloLocalization3D::*)(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &)) &mrpt::slam::CMonteCarloLocalization3D::prediction_and_update_pfAuxiliaryPFOptimal, "Update the m_particles, predicting the posterior of robot pose and map\n after a movement command.\n  This method has additional configuration parameters in \"options\".\n  Performs the update stage of the RBPF, using the sensed CSensoryFrame:\n\n   \n This is a pointer to CActionCollection, containing the\n pose change the robot has been commanded.\n   \n\n This must be a pointer to a CSensoryFrame object,\n with robot sensed observations.\n\n \n options\n\nC++: mrpt::slam::CMonteCarloLocalization3D::prediction_and_update_pfAuxiliaryPFOptimal(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) --> void", pybind11::arg("action"), pybind11::arg("observation"), pybind11::arg("PF_options"));
		cl.def("getVisualization", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (mrpt::slam::CMonteCarloLocalization3D::*)() const) &mrpt::slam::CMonteCarloLocalization3D::getVisualization, "Returns a 3D representation of this PDF.\n \n\n Needs the mrpt-opengl library, and using\n mrpt::opengl::CSetOfObjects::Ptr as template argument.\n\nC++: mrpt::slam::CMonteCarloLocalization3D::getVisualization() const --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");
		cl.def("getLastPose", (struct mrpt::math::TPose3D (mrpt::slam::CMonteCarloLocalization3D::*)(size_t, bool &) const) &mrpt::slam::CMonteCarloLocalization3D::getLastPose, "@{ \n\n Return the robot pose for the i'th particle. is_valid is\n always true in this class. \n\nC++: mrpt::slam::CMonteCarloLocalization3D::getLastPose(size_t, bool &) const --> struct mrpt::math::TPose3D", pybind11::arg("i"), pybind11::arg("is_valid_pose"));
		cl.def("PF_SLAM_implementation_custom_update_particle_with_new_pose", (void (mrpt::slam::CMonteCarloLocalization3D::*)(struct mrpt::math::TPose3D *, const struct mrpt::math::TPose3D &) const) &mrpt::slam::CMonteCarloLocalization3D::PF_SLAM_implementation_custom_update_particle_with_new_pose, "C++: mrpt::slam::CMonteCarloLocalization3D::PF_SLAM_implementation_custom_update_particle_with_new_pose(struct mrpt::math::TPose3D *, const struct mrpt::math::TPose3D &) const --> void", pybind11::arg("particleData"), pybind11::arg("newPose"));
		cl.def("PF_SLAM_computeObservationLikelihoodForParticle", (double (mrpt::slam::CMonteCarloLocalization3D::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const) &mrpt::slam::CMonteCarloLocalization3D::PF_SLAM_computeObservationLikelihoodForParticle, "Evaluate the observation likelihood for one particle at a given location\n\nC++: mrpt::slam::CMonteCarloLocalization3D::PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("PF_options"), pybind11::arg("particleIndexForMap"), pybind11::arg("observation"), pybind11::arg("x"));
		cl.def("assign", (class mrpt::slam::CMonteCarloLocalization3D & (mrpt::slam::CMonteCarloLocalization3D::*)(const class mrpt::slam::CMonteCarloLocalization3D &)) &mrpt::slam::CMonteCarloLocalization3D::operator=, "C++: mrpt::slam::CMonteCarloLocalization3D::operator=(const class mrpt::slam::CMonteCarloLocalization3D &) --> class mrpt::slam::CMonteCarloLocalization3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
