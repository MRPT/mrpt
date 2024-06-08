#include <deque>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/containers/yaml.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/TColor.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
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
#include <mrpt/math/TTwist3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
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
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/PF_implementations_data.h>
#include <mrpt/slam/TMonteCarloLocalizationParams.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <tuple>
#include <type_traits>
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

// mrpt::slam::CMetricMapBuilderICP file:mrpt/slam/CMetricMapBuilderICP.h line:26
struct PyCallBack_mrpt_slam_CMetricMapBuilderICP : public mrpt::slam::CMetricMapBuilderICP {
	using mrpt::slam::CMetricMapBuilderICP::CMetricMapBuilderICP;

	void initialize(const class mrpt::maps::CSimpleMap & a0, const class mrpt::poses::CPosePDF * a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderICP *>(this), "initialize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMapBuilderICP::initialize(a0, a1);
	}
	class std::shared_ptr<class mrpt::poses::CPose3DPDF> getCurrentPoseEstimation() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderICP *>(this), "getCurrentPoseEstimation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class std::shared_ptr<class mrpt::poses::CPose3DPDF>>::value) {
				static pybind11::detail::override_caster_t<class std::shared_ptr<class mrpt::poses::CPose3DPDF>> caster;
				return pybind11::detail::cast_ref<class std::shared_ptr<class mrpt::poses::CPose3DPDF>>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class std::shared_ptr<class mrpt::poses::CPose3DPDF>>(std::move(o));
		}
		return CMetricMapBuilderICP::getCurrentPoseEstimation();
	}
	void processActionObservation(class mrpt::obs::CActionCollection & a0, class mrpt::obs::CSensoryFrame & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderICP *>(this), "processActionObservation");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMapBuilderICP::processActionObservation(a0, a1);
	}
	void getCurrentlyBuiltMap(class mrpt::maps::CSimpleMap & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderICP *>(this), "getCurrentlyBuiltMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMapBuilderICP::getCurrentlyBuiltMap(a0);
	}
	const class mrpt::maps::CMultiMetricMap & getCurrentlyBuiltMetricMap() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderICP *>(this), "getCurrentlyBuiltMetricMap");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const class mrpt::maps::CMultiMetricMap &>::value) {
				static pybind11::detail::override_caster_t<const class mrpt::maps::CMultiMetricMap &> caster;
				return pybind11::detail::cast_ref<const class mrpt::maps::CMultiMetricMap &>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const class mrpt::maps::CMultiMetricMap &>(std::move(o));
		}
		return CMetricMapBuilderICP::getCurrentlyBuiltMetricMap();
	}
	unsigned int getCurrentlyBuiltMapSize() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderICP *>(this), "getCurrentlyBuiltMapSize");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<unsigned int>::value) {
				static pybind11::detail::override_caster_t<unsigned int> caster;
				return pybind11::detail::cast_ref<unsigned int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<unsigned int>(std::move(o));
		}
		return CMetricMapBuilderICP::getCurrentlyBuiltMapSize();
	}
	void saveCurrentEstimationToImage(const std::string & a0, bool a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderICP *>(this), "saveCurrentEstimationToImage");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMetricMapBuilderICP::saveCurrentEstimationToImage(a0, a1);
	}
};

// mrpt::slam::CMetricMapBuilderICP::TConfigParams file:mrpt/slam/CMetricMapBuilderICP.h line:40
struct PyCallBack_mrpt_slam_CMetricMapBuilderICP_TConfigParams : public mrpt::slam::CMetricMapBuilderICP::TConfigParams {
	using mrpt::slam::CMetricMapBuilderICP::TConfigParams::TConfigParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderICP::TConfigParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TConfigParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMetricMapBuilderICP::TConfigParams *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CLoadableOptions::saveToConfigFile(a0, a1);
	}
};

// mrpt::slam::CMonteCarloLocalization2D file:mrpt/slam/CMonteCarloLocalization2D.h line:38
struct PyCallBack_mrpt_slam_CMonteCarloLocalization2D : public mrpt::slam::CMonteCarloLocalization2D {
	using mrpt::slam::CMonteCarloLocalization2D::CMonteCarloLocalization2D;

	void prediction_and_update_pfStandardProposal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "prediction_and_update_pfStandardProposal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMonteCarloLocalization2D::prediction_and_update_pfStandardProposal(a0, a1, a2);
	}
	void prediction_and_update_pfAuxiliaryPFStandard(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "prediction_and_update_pfAuxiliaryPFStandard");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMonteCarloLocalization2D::prediction_and_update_pfAuxiliaryPFStandard(a0, a1, a2);
	}
	void prediction_and_update_pfAuxiliaryPFOptimal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "prediction_and_update_pfAuxiliaryPFOptimal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMonteCarloLocalization2D::prediction_and_update_pfAuxiliaryPFOptimal(a0, a1, a2);
	}
	struct mrpt::math::TPose3D getLastPose(size_t a0, bool & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "getLastPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose3D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose3D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose3D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose3D>(std::move(o));
		}
		return CMonteCarloLocalization2D::getLastPose(a0, a1);
	}
	void PF_SLAM_implementation_custom_update_particle_with_new_pose(struct mrpt::math::TPose2D * a0, const struct mrpt::math::TPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "PF_SLAM_implementation_custom_update_particle_with_new_pose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMonteCarloLocalization2D::PF_SLAM_implementation_custom_update_particle_with_new_pose(a0, a1);
	}
	double PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a0, size_t a1, const class mrpt::obs::CSensoryFrame & a2, const class mrpt::poses::CPose3D & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "PF_SLAM_computeObservationLikelihoodForParticle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CMonteCarloLocalization2D::PF_SLAM_computeObservationLikelihoodForParticle(a0, a1, a2, a3);
	}
	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPosePDFParticles::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPosePDFParticles::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPosePDFParticles::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFParticles::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFParticles::serializeFrom(a0, a1);
	}
	void copyFrom(const class mrpt::poses::CPosePDF & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "copyFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFParticles::copyFrom(a0);
	}
	void getMean(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "getMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFParticles::getMean(a0);
	}
	using _binder_ret_0 = class std::tuple<class mrpt::math::CMatrixFixed<double, 3, 3>, class mrpt::poses::CPose2D>;
	_binder_ret_0 getCovarianceAndMean() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "getCovarianceAndMean");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<_binder_ret_0>::value) {
				static pybind11::detail::override_caster_t<_binder_ret_0> caster;
				return pybind11::detail::cast_ref<_binder_ret_0>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<_binder_ret_0>(std::move(o));
		}
		return CPosePDFParticles::getCovarianceAndMean();
	}
	bool saveToTextFile(const std::string & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "saveToTextFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CPosePDFParticles::saveToTextFile(a0);
	}
	void changeCoordinatesReference(const class mrpt::poses::CPose3D & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "changeCoordinatesReference");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFParticles::changeCoordinatesReference(a0);
	}
	void drawSingleSample(class mrpt::poses::CPose2D & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "drawSingleSample");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFParticles::drawSingleSample(a0);
	}
	void inverse(class mrpt::poses::CPosePDF & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "inverse");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFParticles::inverse(a0);
	}
	void bayesianFusion(const class mrpt::poses::CPosePDF & a0, const class mrpt::poses::CPosePDF & a1, const double a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "bayesianFusion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPosePDFParticles::bayesianFusion(a0, a1, a2);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPosePDFParticles::asString();
	}
	bool isInfType() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "getInformationMatrix");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "getW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "setW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "particlesCount");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "normalizeWeights");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "ESS");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "prediction_and_update_pfOptimalProposal");
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
	bool PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> > & a0, const class mrpt::obs::CSensoryFrame * a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "PF_SLAM_implementation_doWeHaveValidObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::CMonteCarloLocalization2D *>(this), "PF_SLAM_implementation_skipRobotMovement");
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

void bind_mrpt_slam_CMetricMapBuilderICP(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::slam::CMetricMapBuilderICP file:mrpt/slam/CMetricMapBuilderICP.h line:26
		pybind11::class_<mrpt::slam::CMetricMapBuilderICP, std::shared_ptr<mrpt::slam::CMetricMapBuilderICP>, PyCallBack_mrpt_slam_CMetricMapBuilderICP, mrpt::slam::CMetricMapBuilder> cl(M("mrpt::slam"), "CMetricMapBuilderICP", "A class for very simple 2D SLAM based on ICP. This is a non-probabilistic\npose tracking algorithm.\n   Map are stored as in files as binary dumps of \"mrpt::maps::CSimpleMap\"\nobjects. The methods are\n	 thread-safe.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::CMetricMapBuilderICP(); }, [](){ return new PyCallBack_mrpt_slam_CMetricMapBuilderICP(); } ) );
		cl.def_readwrite("ICP_options", &mrpt::slam::CMetricMapBuilderICP::ICP_options);
		cl.def_readwrite("ICP_params", &mrpt::slam::CMetricMapBuilderICP::ICP_params);
		cl.def("initialize", [](mrpt::slam::CMetricMapBuilderICP &o) -> void { return o.initialize(); }, "");
		cl.def("initialize", [](mrpt::slam::CMetricMapBuilderICP &o, const class mrpt::maps::CSimpleMap & a0) -> void { return o.initialize(a0); }, "", pybind11::arg("initialMap"));
		cl.def("initialize", (void (mrpt::slam::CMetricMapBuilderICP::*)(const class mrpt::maps::CSimpleMap &, const class mrpt::poses::CPosePDF *)) &mrpt::slam::CMetricMapBuilderICP::initialize, "Initialize the method, starting with a known location PDF \"x0\"(if\n supplied, set to nullptr to left unmodified) and a given fixed, past map.\n  This method MUST be called if using the default constructor, after\n loading the configuration into ICP_options. In particular,\n TConfigParams::mapInitializers\n\nC++: mrpt::slam::CMetricMapBuilderICP::initialize(const class mrpt::maps::CSimpleMap &, const class mrpt::poses::CPosePDF *) --> void", pybind11::arg("initialMap"), pybind11::arg("x0"));
		cl.def("getCurrentPoseEstimation", (class std::shared_ptr<class mrpt::poses::CPose3DPDF> (mrpt::slam::CMetricMapBuilderICP::*)() const) &mrpt::slam::CMetricMapBuilderICP::getCurrentPoseEstimation, "Returns a copy of the current best pose estimation as a pose PDF.\n\nC++: mrpt::slam::CMetricMapBuilderICP::getCurrentPoseEstimation() const --> class std::shared_ptr<class mrpt::poses::CPose3DPDF>");
		cl.def("setCurrentMapFile", (void (mrpt::slam::CMetricMapBuilderICP::*)(const char *)) &mrpt::slam::CMetricMapBuilderICP::setCurrentMapFile, "Sets the \"current map file\", thus that map will be loaded if it exists\n or a new one will be created if it does not, and the updated map will be\n save to that file when destroying the object.\n\nC++: mrpt::slam::CMetricMapBuilderICP::setCurrentMapFile(const char *) --> void", pybind11::arg("mapFile"));
		cl.def("processActionObservation", (void (mrpt::slam::CMetricMapBuilderICP::*)(class mrpt::obs::CActionCollection &, class mrpt::obs::CSensoryFrame &)) &mrpt::slam::CMetricMapBuilderICP::processActionObservation, "Appends a new action and observations to update this map: See the\ndescription of the class at the top of this page to see a more complete\ndescription.\n  \n\n The estimation of the incremental pose change in the robot\npose.\n	\n\n The set of observations that robot senses at the new pose.\n See params in CMetricMapBuilder::options and\nCMetricMapBuilderICP::ICP_options\n \n\n processObservation\n\nC++: mrpt::slam::CMetricMapBuilderICP::processActionObservation(class mrpt::obs::CActionCollection &, class mrpt::obs::CSensoryFrame &) --> void", pybind11::arg("action"), pybind11::arg("in_SF"));
		cl.def("processObservation", (void (mrpt::slam::CMetricMapBuilderICP::*)(const class std::shared_ptr<class mrpt::obs::CObservation> &)) &mrpt::slam::CMetricMapBuilderICP::processObservation, "The main method of this class: Process one odometry or sensor\n   observation.\n    The new entry point of the algorithm (the old one  was\n   processActionObservation, which now is a wrapper to\n  this method).\n See params in CMetricMapBuilder::options and\n   CMetricMapBuilderICP::ICP_options\n\nC++: mrpt::slam::CMetricMapBuilderICP::processObservation(const class std::shared_ptr<class mrpt::obs::CObservation> &) --> void", pybind11::arg("obs"));
		cl.def("getCurrentlyBuiltMap", (void (mrpt::slam::CMetricMapBuilderICP::*)(class mrpt::maps::CSimpleMap &) const) &mrpt::slam::CMetricMapBuilderICP::getCurrentlyBuiltMap, "Fills \"out_map\" with the set of \"poses\"-\"sensory-frames\", thus the so\n far built map \n\nC++: mrpt::slam::CMetricMapBuilderICP::getCurrentlyBuiltMap(class mrpt::maps::CSimpleMap &) const --> void", pybind11::arg("out_map"));
		cl.def("getCurrentlyBuiltMetricMap", (const class mrpt::maps::CMultiMetricMap & (mrpt::slam::CMetricMapBuilderICP::*)() const) &mrpt::slam::CMetricMapBuilderICP::getCurrentlyBuiltMetricMap, "C++: mrpt::slam::CMetricMapBuilderICP::getCurrentlyBuiltMetricMap() const --> const class mrpt::maps::CMultiMetricMap &", pybind11::return_value_policy::automatic);
		cl.def("getCurrentlyBuiltMapSize", (unsigned int (mrpt::slam::CMetricMapBuilderICP::*)()) &mrpt::slam::CMetricMapBuilderICP::getCurrentlyBuiltMapSize, "Returns just how many sensory-frames are stored in the currently build\n map \n\nC++: mrpt::slam::CMetricMapBuilderICP::getCurrentlyBuiltMapSize() --> unsigned int");
		cl.def("saveCurrentEstimationToImage", [](mrpt::slam::CMetricMapBuilderICP &o, const std::string & a0) -> void { return o.saveCurrentEstimationToImage(a0); }, "", pybind11::arg("file"));
		cl.def("saveCurrentEstimationToImage", (void (mrpt::slam::CMetricMapBuilderICP::*)(const std::string &, bool)) &mrpt::slam::CMetricMapBuilderICP::saveCurrentEstimationToImage, "A useful method for debugging: the current map (and/or poses) estimation\n is dumped to an image file.\n \n\n The output file name\n \n\n Output format = true:EMF, false:BMP\n\nC++: mrpt::slam::CMetricMapBuilderICP::saveCurrentEstimationToImage(const std::string &, bool) --> void", pybind11::arg("file"), pybind11::arg("formatEMF_BMP"));

		{ // mrpt::slam::CMetricMapBuilderICP::TConfigParams file:mrpt/slam/CMetricMapBuilderICP.h line:40
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::slam::CMetricMapBuilderICP::TConfigParams, std::shared_ptr<mrpt::slam::CMetricMapBuilderICP::TConfigParams>, PyCallBack_mrpt_slam_CMetricMapBuilderICP_TConfigParams, mrpt::config::CLoadableOptions> cl(enclosing_class, "TConfigParams", "Algorithm configuration params");
			cl.def( pybind11::init<enum mrpt::system::VerbosityLevel &>(), pybind11::arg("parent_verbosity_level") );

			cl.def( pybind11::init( [](PyCallBack_mrpt_slam_CMetricMapBuilderICP_TConfigParams const &o){ return new PyCallBack_mrpt_slam_CMetricMapBuilderICP_TConfigParams(o); } ) );
			cl.def( pybind11::init( [](mrpt::slam::CMetricMapBuilderICP::TConfigParams const &o){ return new mrpt::slam::CMetricMapBuilderICP::TConfigParams(o); } ) );
			cl.def_readwrite("matchAgainstTheGrid", &mrpt::slam::CMetricMapBuilderICP::TConfigParams::matchAgainstTheGrid);
			cl.def_readwrite("insertionLinDistance", &mrpt::slam::CMetricMapBuilderICP::TConfigParams::insertionLinDistance);
			cl.def_readwrite("insertionAngDistance", &mrpt::slam::CMetricMapBuilderICP::TConfigParams::insertionAngDistance);
			cl.def_readwrite("localizationLinDistance", &mrpt::slam::CMetricMapBuilderICP::TConfigParams::localizationLinDistance);
			cl.def_readwrite("localizationAngDistance", &mrpt::slam::CMetricMapBuilderICP::TConfigParams::localizationAngDistance);
			cl.def_readwrite("minICPgoodnessToAccept", &mrpt::slam::CMetricMapBuilderICP::TConfigParams::minICPgoodnessToAccept);
			cl.def_readwrite("mapInitializers", &mrpt::slam::CMetricMapBuilderICP::TConfigParams::mapInitializers);
			cl.def("assign", (struct mrpt::slam::CMetricMapBuilderICP::TConfigParams & (mrpt::slam::CMetricMapBuilderICP::TConfigParams::*)(const struct mrpt::slam::CMetricMapBuilderICP::TConfigParams &)) &mrpt::slam::CMetricMapBuilderICP::TConfigParams::operator=, "C++: mrpt::slam::CMetricMapBuilderICP::TConfigParams::operator=(const struct mrpt::slam::CMetricMapBuilderICP::TConfigParams &) --> struct mrpt::slam::CMetricMapBuilderICP::TConfigParams &", pybind11::return_value_policy::automatic, pybind11::arg("other"));
			cl.def("loadFromConfigFile", (void (mrpt::slam::CMetricMapBuilderICP::TConfigParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::slam::CMetricMapBuilderICP::TConfigParams::loadFromConfigFile, "C++: mrpt::slam::CMetricMapBuilderICP::TConfigParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
		}

	}
	{ // mrpt::slam::TMonteCarloLocalizationParams file:mrpt/slam/TMonteCarloLocalizationParams.h line:20
		pybind11::class_<mrpt::slam::TMonteCarloLocalizationParams, std::shared_ptr<mrpt::slam::TMonteCarloLocalizationParams>> cl(M("mrpt::slam"), "TMonteCarloLocalizationParams", "The struct for passing extra simulation parameters to the prediction stage\n    when running a particle filter.\n   \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::TMonteCarloLocalizationParams(); } ) );
		cl.def( pybind11::init( [](mrpt::slam::TMonteCarloLocalizationParams const &o){ return new mrpt::slam::TMonteCarloLocalizationParams(o); } ) );
		cl.def_readwrite("metricMap", &mrpt::slam::TMonteCarloLocalizationParams::metricMap);
		cl.def_readwrite("metricMaps", &mrpt::slam::TMonteCarloLocalizationParams::metricMaps);
		cl.def_readwrite("KLD_params", &mrpt::slam::TMonteCarloLocalizationParams::KLD_params);
		cl.def("assign", (struct mrpt::slam::TMonteCarloLocalizationParams & (mrpt::slam::TMonteCarloLocalizationParams::*)(const struct mrpt::slam::TMonteCarloLocalizationParams &)) &mrpt::slam::TMonteCarloLocalizationParams::operator=, "C++: mrpt::slam::TMonteCarloLocalizationParams::operator=(const struct mrpt::slam::TMonteCarloLocalizationParams &) --> struct mrpt::slam::TMonteCarloLocalizationParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::slam::CMonteCarloLocalization2D file:mrpt/slam/CMonteCarloLocalization2D.h line:38
		pybind11::class_<mrpt::slam::CMonteCarloLocalization2D, std::shared_ptr<mrpt::slam::CMonteCarloLocalization2D>, PyCallBack_mrpt_slam_CMonteCarloLocalization2D, mrpt::poses::CPosePDFParticles, mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>> cl(M("mrpt::slam"), "CMonteCarloLocalization2D", "Declares a class that represents a Probability Density Function (PDF) over a\n 2D pose (x,y,phi), using a set of weighted samples.\n\n  This class also implements particle filtering for robot localization. See\n the MRPT\n   application \"app/pf-localization\" for an example of usage.\n\n \n CMonteCarloLocalization3D, CPose2D, CPosePDF, CPoseGaussianPDF,\n CParticleFilterCapable\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::CMonteCarloLocalization2D(); }, [](){ return new PyCallBack_mrpt_slam_CMonteCarloLocalization2D(); } ), "doc");
		cl.def( pybind11::init<size_t>(), pybind11::arg("M") );

		cl.def_readwrite("options", &mrpt::slam::CMonteCarloLocalization2D::options);
		cl.def("resetUniformFreeSpace", [](mrpt::slam::CMonteCarloLocalization2D &o, class mrpt::maps::COccupancyGridMap2D * a0) -> void { return o.resetUniformFreeSpace(a0); }, "", pybind11::arg("theMap"));
		cl.def("resetUniformFreeSpace", [](mrpt::slam::CMonteCarloLocalization2D &o, class mrpt::maps::COccupancyGridMap2D * a0, const double & a1) -> void { return o.resetUniformFreeSpace(a0, a1); }, "", pybind11::arg("theMap"), pybind11::arg("freeCellsThreshold"));
		cl.def("resetUniformFreeSpace", [](mrpt::slam::CMonteCarloLocalization2D &o, class mrpt::maps::COccupancyGridMap2D * a0, const double & a1, const int & a2) -> void { return o.resetUniformFreeSpace(a0, a1, a2); }, "", pybind11::arg("theMap"), pybind11::arg("freeCellsThreshold"), pybind11::arg("particlesCount"));
		cl.def("resetUniformFreeSpace", [](mrpt::slam::CMonteCarloLocalization2D &o, class mrpt::maps::COccupancyGridMap2D * a0, const double & a1, const int & a2, const double & a3) -> void { return o.resetUniformFreeSpace(a0, a1, a2, a3); }, "", pybind11::arg("theMap"), pybind11::arg("freeCellsThreshold"), pybind11::arg("particlesCount"), pybind11::arg("x_min"));
		cl.def("resetUniformFreeSpace", [](mrpt::slam::CMonteCarloLocalization2D &o, class mrpt::maps::COccupancyGridMap2D * a0, const double & a1, const int & a2, const double & a3, const double & a4) -> void { return o.resetUniformFreeSpace(a0, a1, a2, a3, a4); }, "", pybind11::arg("theMap"), pybind11::arg("freeCellsThreshold"), pybind11::arg("particlesCount"), pybind11::arg("x_min"), pybind11::arg("x_max"));
		cl.def("resetUniformFreeSpace", [](mrpt::slam::CMonteCarloLocalization2D &o, class mrpt::maps::COccupancyGridMap2D * a0, const double & a1, const int & a2, const double & a3, const double & a4, const double & a5) -> void { return o.resetUniformFreeSpace(a0, a1, a2, a3, a4, a5); }, "", pybind11::arg("theMap"), pybind11::arg("freeCellsThreshold"), pybind11::arg("particlesCount"), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"));
		cl.def("resetUniformFreeSpace", [](mrpt::slam::CMonteCarloLocalization2D &o, class mrpt::maps::COccupancyGridMap2D * a0, const double & a1, const int & a2, const double & a3, const double & a4, const double & a5, const double & a6) -> void { return o.resetUniformFreeSpace(a0, a1, a2, a3, a4, a5, a6); }, "", pybind11::arg("theMap"), pybind11::arg("freeCellsThreshold"), pybind11::arg("particlesCount"), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"));
		cl.def("resetUniformFreeSpace", [](mrpt::slam::CMonteCarloLocalization2D &o, class mrpt::maps::COccupancyGridMap2D * a0, const double & a1, const int & a2, const double & a3, const double & a4, const double & a5, const double & a6, const double & a7) -> void { return o.resetUniformFreeSpace(a0, a1, a2, a3, a4, a5, a6, a7); }, "", pybind11::arg("theMap"), pybind11::arg("freeCellsThreshold"), pybind11::arg("particlesCount"), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("phi_min"));
		cl.def("resetUniformFreeSpace", (void (mrpt::slam::CMonteCarloLocalization2D::*)(class mrpt::maps::COccupancyGridMap2D *, const double, const int, const double, const double, const double, const double, const double, const double)) &mrpt::slam::CMonteCarloLocalization2D::resetUniformFreeSpace, "Reset the PDF to an uniformly distributed one, but only in the\n free-space\n   of a given 2D occupancy-grid-map. Orientation is randomly generated in\n the whole 2*PI range.\n \n\n The occupancy grid map\n \n\n The minimum free-probability to consider a\n cell as empty (default is 0.7)\n \n\n If set to -1 the number of m_particles remains\n unchanged.\n \n\n The limits of the area to look for free cells.\n \n\n The limits of the area to look for free cells.\n \n\n The limits of the area to look for free cells.\n \n\n The limits of the area to look for free cells.\n \n\n The limits of the area to look for free cells.\n \n\n The limits of the area to look for free cells.\n  \n\n resetDeterm32inistic\n \n\n std::exception On any error (no free cell found in map,\n map=nullptr, etc...)\n\nC++: mrpt::slam::CMonteCarloLocalization2D::resetUniformFreeSpace(class mrpt::maps::COccupancyGridMap2D *, const double, const int, const double, const double, const double, const double, const double, const double) --> void", pybind11::arg("theMap"), pybind11::arg("freeCellsThreshold"), pybind11::arg("particlesCount"), pybind11::arg("x_min"), pybind11::arg("x_max"), pybind11::arg("y_min"), pybind11::arg("y_max"), pybind11::arg("phi_min"), pybind11::arg("phi_max"));
		cl.def("prediction_and_update_pfStandardProposal", (void (mrpt::slam::CMonteCarloLocalization2D::*)(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &)) &mrpt::slam::CMonteCarloLocalization2D::prediction_and_update_pfStandardProposal, "Update the m_particles, predicting the posterior of robot pose and map\n after a movement command.\n  This method has additional configuration parameters in \"options\".\n  Performs the update stage of the RBPF, using the sensed CSensoryFrame:\n\n   \n This is a pointer to CActionCollection, containing the\n pose change the robot has been commanded.\n   \n\n This must be a pointer to a CSensoryFrame object,\n with robot sensed observations.\n\n \n options\n\nC++: mrpt::slam::CMonteCarloLocalization2D::prediction_and_update_pfStandardProposal(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) --> void", pybind11::arg("action"), pybind11::arg("observation"), pybind11::arg("PF_options"));
		cl.def("prediction_and_update_pfAuxiliaryPFStandard", (void (mrpt::slam::CMonteCarloLocalization2D::*)(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &)) &mrpt::slam::CMonteCarloLocalization2D::prediction_and_update_pfAuxiliaryPFStandard, "Update the m_particles, predicting the posterior of robot pose and map\n after a movement command.\n  This method has additional configuration parameters in \"options\".\n  Performs the update stage of the RBPF, using the sensed CSensoryFrame:\n\n   \n This is a pointer to CActionCollection, containing the\n pose change the robot has been commanded.\n   \n\n This must be a pointer to a CSensoryFrame object,\n with robot sensed observations.\n\n \n options\n\nC++: mrpt::slam::CMonteCarloLocalization2D::prediction_and_update_pfAuxiliaryPFStandard(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) --> void", pybind11::arg("action"), pybind11::arg("observation"), pybind11::arg("PF_options"));
		cl.def("prediction_and_update_pfAuxiliaryPFOptimal", (void (mrpt::slam::CMonteCarloLocalization2D::*)(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &)) &mrpt::slam::CMonteCarloLocalization2D::prediction_and_update_pfAuxiliaryPFOptimal, "Update the m_particles, predicting the posterior of robot pose and map\n after a movement command.\n  This method has additional configuration parameters in \"options\".\n  Performs the update stage of the RBPF, using the sensed CSensoryFrame:\n\n   \n This is a pointer to CActionCollection, containing the\n pose change the robot has been commanded.\n   \n\n This must be a pointer to a CSensoryFrame object,\n with robot sensed observations.\n\n \n options\n\nC++: mrpt::slam::CMonteCarloLocalization2D::prediction_and_update_pfAuxiliaryPFOptimal(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) --> void", pybind11::arg("action"), pybind11::arg("observation"), pybind11::arg("PF_options"));
		cl.def("getVisualization", (class std::shared_ptr<class mrpt::opengl::CSetOfObjects> (mrpt::slam::CMonteCarloLocalization2D::*)() const) &mrpt::slam::CMonteCarloLocalization2D::getVisualization, "Returns a 3D representation of this PDF.\n \n\n Needs the mrpt-opengl library, and using\n mrpt::opengl::CSetOfObjects::Ptr as template argument.\n\nC++: mrpt::slam::CMonteCarloLocalization2D::getVisualization() const --> class std::shared_ptr<class mrpt::opengl::CSetOfObjects>");
		cl.def("getLastPose", (struct mrpt::math::TPose3D (mrpt::slam::CMonteCarloLocalization2D::*)(size_t, bool &) const) &mrpt::slam::CMonteCarloLocalization2D::getLastPose, "@{ \n\n Return the robot pose for the i'th particle. is_valid is\n always true in this class. \n\nC++: mrpt::slam::CMonteCarloLocalization2D::getLastPose(size_t, bool &) const --> struct mrpt::math::TPose3D", pybind11::arg("i"), pybind11::arg("is_valid_pose"));
		cl.def("PF_SLAM_implementation_custom_update_particle_with_new_pose", (void (mrpt::slam::CMonteCarloLocalization2D::*)(struct mrpt::math::TPose2D *, const struct mrpt::math::TPose3D &) const) &mrpt::slam::CMonteCarloLocalization2D::PF_SLAM_implementation_custom_update_particle_with_new_pose, "C++: mrpt::slam::CMonteCarloLocalization2D::PF_SLAM_implementation_custom_update_particle_with_new_pose(struct mrpt::math::TPose2D *, const struct mrpt::math::TPose3D &) const --> void", pybind11::arg("particleData"), pybind11::arg("newPose"));
		cl.def("PF_SLAM_computeObservationLikelihoodForParticle", (double (mrpt::slam::CMonteCarloLocalization2D::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const) &mrpt::slam::CMonteCarloLocalization2D::PF_SLAM_computeObservationLikelihoodForParticle, "Evaluate the observation likelihood for one particle at a given location\n\nC++: mrpt::slam::CMonteCarloLocalization2D::PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("PF_options"), pybind11::arg("particleIndexForMap"), pybind11::arg("observation"), pybind11::arg("x"));
		cl.def("assign", (class mrpt::slam::CMonteCarloLocalization2D & (mrpt::slam::CMonteCarloLocalization2D::*)(const class mrpt::slam::CMonteCarloLocalization2D &)) &mrpt::slam::CMonteCarloLocalization2D::operator=, "C++: mrpt::slam::CMonteCarloLocalization2D::operator=(const class mrpt::slam::CMonteCarloLocalization2D &) --> class mrpt::slam::CMonteCarloLocalization2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
