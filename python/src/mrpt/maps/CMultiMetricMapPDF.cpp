#include <deque>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterData.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/TMetricMapInitializer.h>
#include <mrpt/maps/metric_map_types.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
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
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
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

// mrpt::maps::CRBPFParticleData file:mrpt/maps/CMultiMetricMapPDF.h line:32
struct PyCallBack_mrpt_maps_CRBPFParticleData : public mrpt::maps::CRBPFParticleData {
	using mrpt::maps::CRBPFParticleData::CRBPFParticleData;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRBPFParticleData *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CRBPFParticleData::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRBPFParticleData *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CRBPFParticleData::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRBPFParticleData *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CRBPFParticleData::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRBPFParticleData *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRBPFParticleData::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CRBPFParticleData *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CRBPFParticleData::serializeFrom(a0, a1);
	}
};

// mrpt::maps::CMultiMetricMapPDF file:mrpt/maps/CMultiMetricMapPDF.h line:53
struct PyCallBack_mrpt_maps_CMultiMetricMapPDF : public mrpt::maps::CMultiMetricMapPDF {
	using mrpt::maps::CMultiMetricMapPDF::CMultiMetricMapPDF;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CMultiMetricMapPDF::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CMultiMetricMapPDF::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CMultiMetricMapPDF::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMapPDF::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMapPDF::serializeFrom(a0, a1);
	}
	void prediction_and_update_pfStandardProposal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "prediction_and_update_pfStandardProposal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMapPDF::prediction_and_update_pfStandardProposal(a0, a1, a2);
	}
	void prediction_and_update_pfOptimalProposal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "prediction_and_update_pfOptimalProposal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMapPDF::prediction_and_update_pfOptimalProposal(a0, a1, a2);
	}
	void prediction_and_update_pfAuxiliaryPFOptimal(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "prediction_and_update_pfAuxiliaryPFOptimal");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMapPDF::prediction_and_update_pfAuxiliaryPFOptimal(a0, a1, a2);
	}
	void prediction_and_update_pfAuxiliaryPFStandard(const class mrpt::obs::CActionCollection * a0, const class mrpt::obs::CSensoryFrame * a1, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a2) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "prediction_and_update_pfAuxiliaryPFStandard");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMapPDF::prediction_and_update_pfAuxiliaryPFStandard(a0, a1, a2);
	}
	struct mrpt::math::TPose3D getLastPose(size_t a0, bool & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "getLastPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose3D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose3D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose3D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose3D>(std::move(o));
		}
		return CMultiMetricMapPDF::getLastPose(a0, a1);
	}
	void PF_SLAM_implementation_custom_update_particle_with_new_pose(class mrpt::maps::CRBPFParticleData * a0, const struct mrpt::math::TPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "PF_SLAM_implementation_custom_update_particle_with_new_pose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CMultiMetricMapPDF::PF_SLAM_implementation_custom_update_particle_with_new_pose(a0, a1);
	}
	bool PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > & a0, const class mrpt::obs::CSensoryFrame * a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "PF_SLAM_implementation_doWeHaveValidObservations");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CMultiMetricMapPDF::PF_SLAM_implementation_doWeHaveValidObservations(a0, a1);
	}
	bool PF_SLAM_implementation_skipRobotMovement() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "PF_SLAM_implementation_skipRobotMovement");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<bool>::value) {
				static pybind11::detail::override_caster_t<bool> caster;
				return pybind11::detail::cast_ref<bool>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<bool>(std::move(o));
		}
		return CMultiMetricMapPDF::PF_SLAM_implementation_skipRobotMovement();
	}
	double PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a0, size_t a1, const class mrpt::obs::CSensoryFrame & a2, const class mrpt::poses::CPose3D & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "PF_SLAM_computeObservationLikelihoodForParticle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		return CMultiMetricMapPDF::PF_SLAM_computeObservationLikelihoodForParticle(a0, a1, a2, a3);
	}
	double getW(size_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "getW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "setW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "particlesCount");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "normalizeWeights");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF *>(this), "ESS");
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
};

// mrpt::maps::CMultiMetricMapPDF::TPredictionParams file:mrpt/maps/CMultiMetricMapPDF.h line:104
struct PyCallBack_mrpt_maps_CMultiMetricMapPDF_TPredictionParams : public mrpt::maps::CMultiMetricMapPDF::TPredictionParams {
	using mrpt::maps::CMultiMetricMapPDF::TPredictionParams::TPredictionParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF::TPredictionParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TPredictionParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::maps::CMultiMetricMapPDF::TPredictionParams *>(this), "saveToConfigFile");
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

void bind_mrpt_maps_CMultiMetricMapPDF(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CRBPFParticleData file:mrpt/maps/CMultiMetricMapPDF.h line:32
		pybind11::class_<mrpt::maps::CRBPFParticleData, std::shared_ptr<mrpt::maps::CRBPFParticleData>, PyCallBack_mrpt_maps_CRBPFParticleData, mrpt::serialization::CSerializable> cl(M("mrpt::maps"), "CRBPFParticleData", "Auxiliary class used in mrpt::maps::CMultiMetricMapPDF\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CRBPFParticleData(); }, [](){ return new PyCallBack_mrpt_maps_CRBPFParticleData(); } ) );
		cl.def( pybind11::init<const class mrpt::maps::TSetOfMetricMapInitializers &>(), pybind11::arg("mapsInit") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CRBPFParticleData const &o){ return new PyCallBack_mrpt_maps_CRBPFParticleData(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CRBPFParticleData const &o){ return new mrpt::maps::CRBPFParticleData(o); } ) );
		cl.def_readwrite("mapTillNow", &mrpt::maps::CRBPFParticleData::mapTillNow);
		cl.def_readwrite("robotPath", &mrpt::maps::CRBPFParticleData::robotPath);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CRBPFParticleData::GetRuntimeClassIdStatic, "C++: mrpt::maps::CRBPFParticleData::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CRBPFParticleData::*)() const) &mrpt::maps::CRBPFParticleData::GetRuntimeClass, "C++: mrpt::maps::CRBPFParticleData::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CRBPFParticleData::*)() const) &mrpt::maps::CRBPFParticleData::clone, "C++: mrpt::maps::CRBPFParticleData::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CRBPFParticleData::CreateObject, "C++: mrpt::maps::CRBPFParticleData::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("assign", (class mrpt::maps::CRBPFParticleData & (mrpt::maps::CRBPFParticleData::*)(const class mrpt::maps::CRBPFParticleData &)) &mrpt::maps::CRBPFParticleData::operator=, "C++: mrpt::maps::CRBPFParticleData::operator=(const class mrpt::maps::CRBPFParticleData &) --> class mrpt::maps::CRBPFParticleData &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::CMultiMetricMapPDF file:mrpt/maps/CMultiMetricMapPDF.h line:53
		pybind11::class_<mrpt::maps::CMultiMetricMapPDF, std::shared_ptr<mrpt::maps::CMultiMetricMapPDF>, PyCallBack_mrpt_maps_CMultiMetricMapPDF, mrpt::serialization::CSerializable, mrpt::bayes::CParticleFilterData<mrpt::maps::CRBPFParticleData,mrpt::bayes::particle_storage_mode::POINTER>, mrpt::bayes::CParticleFilterDataImpl<mrpt::maps::CMultiMetricMapPDF,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> >>, mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>> cl(M("mrpt::maps"), "CMultiMetricMapPDF", "Declares a class that represents a Rao-Blackwellized set of particles for\n solving the SLAM problem (This class is the base of RBPF-SLAM applications).\n   This class is used internally by the map building algorithm in\n \"mrpt::slam::CMetricMapBuilderRBPF\"\n\n \n mrpt::slam::CMetricMapBuilderRBPF\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CMultiMetricMapPDF(); }, [](){ return new PyCallBack_mrpt_maps_CMultiMetricMapPDF(); } ) );
		cl.def( pybind11::init<const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, const class mrpt::maps::TSetOfMetricMapInitializers &, const struct mrpt::maps::CMultiMetricMapPDF::TPredictionParams &>(), pybind11::arg("opts"), pybind11::arg("mapsInitializers"), pybind11::arg("predictionOptions") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CMultiMetricMapPDF const &o){ return new PyCallBack_mrpt_maps_CMultiMetricMapPDF(o); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CMultiMetricMapPDF const &o){ return new mrpt::maps::CMultiMetricMapPDF(o); } ) );
		cl.def_readwrite("options", &mrpt::maps::CMultiMetricMapPDF::options);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::maps::CMultiMetricMapPDF::GetRuntimeClassIdStatic, "C++: mrpt::maps::CMultiMetricMapPDF::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::maps::CMultiMetricMapPDF::*)() const) &mrpt::maps::CMultiMetricMapPDF::GetRuntimeClass, "C++: mrpt::maps::CMultiMetricMapPDF::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::maps::CMultiMetricMapPDF::*)() const) &mrpt::maps::CMultiMetricMapPDF::clone, "C++: mrpt::maps::CMultiMetricMapPDF::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::maps::CMultiMetricMapPDF::CreateObject, "C++: mrpt::maps::CMultiMetricMapPDF::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("clear", (void (mrpt::maps::CMultiMetricMapPDF::*)(const class mrpt::poses::CPose2D &)) &mrpt::maps::CMultiMetricMapPDF::clear, "Clear all elements of the maps, and restore all paths to a single\n starting pose \n\nC++: mrpt::maps::CMultiMetricMapPDF::clear(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("initialPose"));
		cl.def("clear", (void (mrpt::maps::CMultiMetricMapPDF::*)(const class mrpt::poses::CPose3D &)) &mrpt::maps::CMultiMetricMapPDF::clear, "C++: mrpt::maps::CMultiMetricMapPDF::clear(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("initialPose"));
		cl.def("clear", (void (mrpt::maps::CMultiMetricMapPDF::*)(const class mrpt::maps::CSimpleMap &, const class mrpt::poses::CPose3D &)) &mrpt::maps::CMultiMetricMapPDF::clear, "Resets the map by loading an already-mapped map for past poses.\n Current robot pose should be normally set to the last keyframe\n in the simplemap. \n\nC++: mrpt::maps::CMultiMetricMapPDF::clear(const class mrpt::maps::CSimpleMap &, const class mrpt::poses::CPose3D &) --> void", pybind11::arg("prevMap"), pybind11::arg("currentPose"));
		cl.def("getEstimatedPosePDFAtTime", (void (mrpt::maps::CMultiMetricMapPDF::*)(size_t, class mrpt::poses::CPose3DPDFParticles &) const) &mrpt::maps::CMultiMetricMapPDF::getEstimatedPosePDFAtTime, "Returns the estimate of the robot pose as a particles PDF for the\n instant of time \"timeStep\", from 0 to N-1.\n \n\n getEstimatedPosePDF\n\nC++: mrpt::maps::CMultiMetricMapPDF::getEstimatedPosePDFAtTime(size_t, class mrpt::poses::CPose3DPDFParticles &) const --> void", pybind11::arg("timeStep"), pybind11::arg("out_estimation"));
		cl.def("getEstimatedPosePDF", (void (mrpt::maps::CMultiMetricMapPDF::*)(class mrpt::poses::CPose3DPDFParticles &) const) &mrpt::maps::CMultiMetricMapPDF::getEstimatedPosePDF, "Returns the current estimate of the robot pose, as a particles PDF.\n \n\n getEstimatedPosePDFAtTime\n\nC++: mrpt::maps::CMultiMetricMapPDF::getEstimatedPosePDF(class mrpt::poses::CPose3DPDFParticles &) const --> void", pybind11::arg("out_estimation"));
		cl.def("getAveragedMetricMapEstimation", (const class mrpt::maps::CMultiMetricMap * (mrpt::maps::CMultiMetricMapPDF::*)()) &mrpt::maps::CMultiMetricMapPDF::getAveragedMetricMapEstimation, "Returns the weighted averaged map based on the current best estimation.\n If you need a persistent copy of this object, please use\n \"CSerializable::duplicate\" and use the copy.\n \n\n Almost 100% sure you would prefer the best current map, given by\n getCurrentMostLikelyMetricMap()\n\nC++: mrpt::maps::CMultiMetricMapPDF::getAveragedMetricMapEstimation() --> const class mrpt::maps::CMultiMetricMap *", pybind11::return_value_policy::automatic);
		cl.def("getCurrentMostLikelyMetricMap", (const class mrpt::maps::CMultiMetricMap * (mrpt::maps::CMultiMetricMapPDF::*)() const) &mrpt::maps::CMultiMetricMapPDF::getCurrentMostLikelyMetricMap, "Returns a pointer to the current most likely map (associated to the most\n likely particle) \n\nC++: mrpt::maps::CMultiMetricMapPDF::getCurrentMostLikelyMetricMap() const --> const class mrpt::maps::CMultiMetricMap *", pybind11::return_value_policy::automatic);
		cl.def("getNumberOfObservationsInSimplemap", (size_t (mrpt::maps::CMultiMetricMapPDF::*)() const) &mrpt::maps::CMultiMetricMapPDF::getNumberOfObservationsInSimplemap, "Get the number of CSensoryFrame inserted into the internal member SFs \n\nC++: mrpt::maps::CMultiMetricMapPDF::getNumberOfObservationsInSimplemap() const --> size_t");
		cl.def("insertObservation", (bool (mrpt::maps::CMultiMetricMapPDF::*)(class mrpt::obs::CSensoryFrame &)) &mrpt::maps::CMultiMetricMapPDF::insertObservation, "Insert an observation to the map, at each particle's pose and to each\n particle's metric map.\n \n\n The SF to be inserted\n \n\n true if any may was updated, false otherwise\n\nC++: mrpt::maps::CMultiMetricMapPDF::insertObservation(class mrpt::obs::CSensoryFrame &) --> bool", pybind11::arg("sf"));
		cl.def("getPath", (void (mrpt::maps::CMultiMetricMapPDF::*)(size_t, class std::deque<struct mrpt::math::TPose3D> &) const) &mrpt::maps::CMultiMetricMapPDF::getPath, "Return the path (in absolute coordinate poses) for the i'th particle.\n \n\n On index out of bounds\n\nC++: mrpt::maps::CMultiMetricMapPDF::getPath(size_t, class std::deque<struct mrpt::math::TPose3D> &) const --> void", pybind11::arg("i"), pybind11::arg("out_path"));
		cl.def("getCurrentEntropyOfPaths", (double (mrpt::maps::CMultiMetricMapPDF::*)()) &mrpt::maps::CMultiMetricMapPDF::getCurrentEntropyOfPaths, "Returns the current entropy of paths, computed as the average entropy of\n poses along the path, where entropy of each pose estimation is computed\n as the entropy of the gaussian approximation covariance.\n\nC++: mrpt::maps::CMultiMetricMapPDF::getCurrentEntropyOfPaths() --> double");
		cl.def("getCurrentJointEntropy", (double (mrpt::maps::CMultiMetricMapPDF::*)()) &mrpt::maps::CMultiMetricMapPDF::getCurrentJointEntropy, "Returns the joint entropy estimation over paths and maps, acording to\n \"Information Gain-based Exploration Using\" by C. Stachniss, G. Grissetti\n and W.Burgard.\n\nC++: mrpt::maps::CMultiMetricMapPDF::getCurrentJointEntropy() --> double");
		cl.def("updateSensoryFrameSequence", (void (mrpt::maps::CMultiMetricMapPDF::*)()) &mrpt::maps::CMultiMetricMapPDF::updateSensoryFrameSequence, "Update the poses estimation of the member \"SFs\" according to the current\n path belief.\n\nC++: mrpt::maps::CMultiMetricMapPDF::updateSensoryFrameSequence() --> void");
		cl.def("saveCurrentPathEstimationToTextFile", (void (mrpt::maps::CMultiMetricMapPDF::*)(const std::string &)) &mrpt::maps::CMultiMetricMapPDF::saveCurrentPathEstimationToTextFile, "A logging utility: saves the current path estimation for each particle\n in a text file (a row per particle, each 3-column-entry is a set\n [x,y,phi], respectively).\n\nC++: mrpt::maps::CMultiMetricMapPDF::saveCurrentPathEstimationToTextFile(const std::string &) --> void", pybind11::arg("fil"));
		cl.def("getLastPose", (struct mrpt::math::TPose3D (mrpt::maps::CMultiMetricMapPDF::*)(size_t, bool &) const) &mrpt::maps::CMultiMetricMapPDF::getLastPose, "@{ \n\nC++: mrpt::maps::CMultiMetricMapPDF::getLastPose(size_t, bool &) const --> struct mrpt::math::TPose3D", pybind11::arg("i"), pybind11::arg("pose_is_valid"));
		cl.def("PF_SLAM_implementation_custom_update_particle_with_new_pose", (void (mrpt::maps::CMultiMetricMapPDF::*)(class mrpt::maps::CRBPFParticleData *, const struct mrpt::math::TPose3D &) const) &mrpt::maps::CMultiMetricMapPDF::PF_SLAM_implementation_custom_update_particle_with_new_pose, "C++: mrpt::maps::CMultiMetricMapPDF::PF_SLAM_implementation_custom_update_particle_with_new_pose(class mrpt::maps::CRBPFParticleData *, const struct mrpt::math::TPose3D &) const --> void", pybind11::arg("particleData"), pybind11::arg("newPose"));
		cl.def("PF_SLAM_implementation_doWeHaveValidObservations", (bool (mrpt::maps::CMultiMetricMapPDF::*)(const class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &, const class mrpt::obs::CSensoryFrame *) const) &mrpt::maps::CMultiMetricMapPDF::PF_SLAM_implementation_doWeHaveValidObservations, "C++: mrpt::maps::CMultiMetricMapPDF::PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &, const class mrpt::obs::CSensoryFrame *) const --> bool", pybind11::arg("particles"), pybind11::arg("sf"));
		cl.def("PF_SLAM_implementation_skipRobotMovement", (bool (mrpt::maps::CMultiMetricMapPDF::*)() const) &mrpt::maps::CMultiMetricMapPDF::PF_SLAM_implementation_skipRobotMovement, "C++: mrpt::maps::CMultiMetricMapPDF::PF_SLAM_implementation_skipRobotMovement() const --> bool");
		cl.def("PF_SLAM_computeObservationLikelihoodForParticle", (double (mrpt::maps::CMultiMetricMapPDF::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const) &mrpt::maps::CMultiMetricMapPDF::PF_SLAM_computeObservationLikelihoodForParticle, "Evaluate the observation likelihood for one particle at a given location\n\nC++: mrpt::maps::CMultiMetricMapPDF::PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("PF_options"), pybind11::arg("particleIndexForMap"), pybind11::arg("observation"), pybind11::arg("x"));
		cl.def("assign", (class mrpt::maps::CMultiMetricMapPDF & (mrpt::maps::CMultiMetricMapPDF::*)(const class mrpt::maps::CMultiMetricMapPDF &)) &mrpt::maps::CMultiMetricMapPDF::operator=, "C++: mrpt::maps::CMultiMetricMapPDF::operator=(const class mrpt::maps::CMultiMetricMapPDF &) --> class mrpt::maps::CMultiMetricMapPDF &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::maps::CMultiMetricMapPDF::TPredictionParams file:mrpt/maps/CMultiMetricMapPDF.h line:104
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::maps::CMultiMetricMapPDF::TPredictionParams, std::shared_ptr<mrpt::maps::CMultiMetricMapPDF::TPredictionParams>, PyCallBack_mrpt_maps_CMultiMetricMapPDF_TPredictionParams, mrpt::config::CLoadableOptions> cl(enclosing_class, "TPredictionParams", "The struct for passing extra simulation parameters to the\n prediction/update stage\n    when running a particle filter.\n \n\n prediction_and_update");
			cl.def( pybind11::init( [](){ return new mrpt::maps::CMultiMetricMapPDF::TPredictionParams(); }, [](){ return new PyCallBack_mrpt_maps_CMultiMetricMapPDF_TPredictionParams(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_maps_CMultiMetricMapPDF_TPredictionParams const &o){ return new PyCallBack_mrpt_maps_CMultiMetricMapPDF_TPredictionParams(o); } ) );
			cl.def( pybind11::init( [](mrpt::maps::CMultiMetricMapPDF::TPredictionParams const &o){ return new mrpt::maps::CMultiMetricMapPDF::TPredictionParams(o); } ) );
			cl.def_readwrite("pfOptimalProposal_mapSelection", &mrpt::maps::CMultiMetricMapPDF::TPredictionParams::pfOptimalProposal_mapSelection);
			cl.def_readwrite("ICPGlobalAlign_MinQuality", &mrpt::maps::CMultiMetricMapPDF::TPredictionParams::ICPGlobalAlign_MinQuality);
			cl.def_readwrite("KLD_params", &mrpt::maps::CMultiMetricMapPDF::TPredictionParams::KLD_params);
			cl.def_readwrite("icp_params", &mrpt::maps::CMultiMetricMapPDF::TPredictionParams::icp_params);
			cl.def("loadFromConfigFile", (void (mrpt::maps::CMultiMetricMapPDF::TPredictionParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::maps::CMultiMetricMapPDF::TPredictionParams::loadFromConfigFile, "C++: mrpt::maps::CMultiMetricMapPDF::TPredictionParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::maps::CMultiMetricMapPDF::TPredictionParams & (mrpt::maps::CMultiMetricMapPDF::TPredictionParams::*)(const struct mrpt::maps::CMultiMetricMapPDF::TPredictionParams &)) &mrpt::maps::CMultiMetricMapPDF::TPredictionParams::operator=, "C++: mrpt::maps::CMultiMetricMapPDF::TPredictionParams::operator=(const struct mrpt::maps::CMultiMetricMapPDF::TPredictionParams &) --> struct mrpt::maps::CMultiMetricMapPDF::TPredictionParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
