#include <deque>
#include <ios>
#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CMultiMetricMapPDF.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
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
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/slam/PF_implementations_data.h>
#include <mrpt/slam/TKLDParams.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
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

// mrpt::slam::TKLDParams file:mrpt/slam/TKLDParams.h line:17
struct PyCallBack_mrpt_slam_TKLDParams : public mrpt::slam::TKLDParams {
	using mrpt::slam::TKLDParams::TKLDParams;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::TKLDParams *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TKLDParams::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::TKLDParams *>(this), "saveToConfigFile");
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

// mrpt::slam::PF_implementation file:mrpt/slam/PF_implementations_data.h line:36
struct PyCallBack_mrpt_slam_PF_implementation_mrpt_maps_CRBPFParticleData_mrpt_maps_CMultiMetricMapPDF_mrpt_bayes_particle_storage_mode_POINTER_t : public mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER> {
	using mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>::PF_implementation;

	struct mrpt::math::TPose3D getLastPose(size_t a0, bool & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER> *>(this), "getLastPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose3D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose3D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose3D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose3D>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PF_implementation::getLastPose\"");
	}
	void PF_SLAM_implementation_custom_update_particle_with_new_pose(class mrpt::maps::CRBPFParticleData * a0, const struct mrpt::math::TPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER> *>(this), "PF_SLAM_implementation_custom_update_particle_with_new_pose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PF_implementation::PF_SLAM_implementation_custom_update_particle_with_new_pose\"");
	}
	bool PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > & a0, const class mrpt::obs::CSensoryFrame * a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER> *>(this), "PF_SLAM_implementation_doWeHaveValidObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER> *>(this), "PF_SLAM_implementation_skipRobotMovement");
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
	double PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a0, size_t a1, const class mrpt::obs::CSensoryFrame & a2, const class mrpt::poses::CPose3D & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER> *>(this), "PF_SLAM_computeObservationLikelihoodForParticle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PF_implementation::PF_SLAM_computeObservationLikelihoodForParticle\"");
	}
};

// mrpt::slam::PF_implementation file:mrpt/slam/PF_implementations_data.h line:36
struct PyCallBack_mrpt_slam_PF_implementation_mrpt_math_TPose2D_mrpt_slam_CMonteCarloLocalization2D_mrpt_bayes_particle_storage_mode_VALUE_t : public mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE> {
	using mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>::PF_implementation;

	struct mrpt::math::TPose3D getLastPose(size_t a0, bool & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "getLastPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose3D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose3D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose3D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose3D>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PF_implementation::getLastPose\"");
	}
	void PF_SLAM_implementation_custom_update_particle_with_new_pose(struct mrpt::math::TPose2D * a0, const struct mrpt::math::TPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "PF_SLAM_implementation_custom_update_particle_with_new_pose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PF_implementation::PF_SLAM_implementation_custom_update_particle_with_new_pose\"");
	}
	bool PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> > & a0, const class mrpt::obs::CSensoryFrame * a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "PF_SLAM_implementation_doWeHaveValidObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "PF_SLAM_implementation_skipRobotMovement");
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
	double PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a0, size_t a1, const class mrpt::obs::CSensoryFrame & a2, const class mrpt::poses::CPose3D & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "PF_SLAM_computeObservationLikelihoodForParticle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PF_implementation::PF_SLAM_computeObservationLikelihoodForParticle\"");
	}
};

// mrpt::slam::PF_implementation file:mrpt/slam/PF_implementations_data.h line:36
struct PyCallBack_mrpt_slam_PF_implementation_mrpt_math_TPose3D_mrpt_slam_CMonteCarloLocalization3D_mrpt_bayes_particle_storage_mode_VALUE_t : public mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE> {
	using mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>::PF_implementation;

	struct mrpt::math::TPose3D getLastPose(size_t a0, bool & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "getLastPose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<struct mrpt::math::TPose3D>::value) {
				static pybind11::detail::override_caster_t<struct mrpt::math::TPose3D> caster;
				return pybind11::detail::cast_ref<struct mrpt::math::TPose3D>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<struct mrpt::math::TPose3D>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PF_implementation::getLastPose\"");
	}
	void PF_SLAM_implementation_custom_update_particle_with_new_pose(struct mrpt::math::TPose3D * a0, const struct mrpt::math::TPose3D & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "PF_SLAM_implementation_custom_update_particle_with_new_pose");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PF_implementation::PF_SLAM_implementation_custom_update_particle_with_new_pose\"");
	}
	bool PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> > & a0, const class mrpt::obs::CSensoryFrame * a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "PF_SLAM_implementation_doWeHaveValidObservations");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "PF_SLAM_implementation_skipRobotMovement");
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
	double PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a0, size_t a1, const class mrpt::obs::CSensoryFrame & a2, const class mrpt::poses::CPose3D & a3) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE> *>(this), "PF_SLAM_computeObservationLikelihoodForParticle");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1, a2, a3);
			if (pybind11::detail::cast_is_temporary_value_reference<double>::value) {
				static pybind11::detail::override_caster_t<double> caster;
				return pybind11::detail::cast_ref<double>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<double>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"PF_implementation::PF_SLAM_computeObservationLikelihoodForParticle\"");
	}
};

void bind_mrpt_slam_TKLDParams(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::slam::TKLDParams file:mrpt/slam/TKLDParams.h line:17
		pybind11::class_<mrpt::slam::TKLDParams, std::shared_ptr<mrpt::slam::TKLDParams>, PyCallBack_mrpt_slam_TKLDParams, mrpt::config::CLoadableOptions> cl(M("mrpt::slam"), "TKLDParams", "Option set for KLD algorithm.  \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::slam::TKLDParams(); }, [](){ return new PyCallBack_mrpt_slam_TKLDParams(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_slam_TKLDParams const &o){ return new PyCallBack_mrpt_slam_TKLDParams(o); } ) );
		cl.def( pybind11::init( [](mrpt::slam::TKLDParams const &o){ return new mrpt::slam::TKLDParams(o); } ) );
		cl.def_readwrite("KLD_binSize_XY", &mrpt::slam::TKLDParams::KLD_binSize_XY);
		cl.def_readwrite("KLD_binSize_PHI", &mrpt::slam::TKLDParams::KLD_binSize_PHI);
		cl.def_readwrite("KLD_delta", &mrpt::slam::TKLDParams::KLD_delta);
		cl.def_readwrite("KLD_epsilon", &mrpt::slam::TKLDParams::KLD_epsilon);
		cl.def_readwrite("KLD_minSampleSize", &mrpt::slam::TKLDParams::KLD_minSampleSize);
		cl.def_readwrite("KLD_maxSampleSize", &mrpt::slam::TKLDParams::KLD_maxSampleSize);
		cl.def_readwrite("KLD_minSamplesPerBin", &mrpt::slam::TKLDParams::KLD_minSamplesPerBin);
		cl.def("loadFromConfigFile", (void (mrpt::slam::TKLDParams::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::slam::TKLDParams::loadFromConfigFile, "C++: mrpt::slam::TKLDParams::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
		cl.def("assign", (class mrpt::slam::TKLDParams & (mrpt::slam::TKLDParams::*)(const class mrpt::slam::TKLDParams &)) &mrpt::slam::TKLDParams::operator=, "C++: mrpt::slam::TKLDParams::operator=(const class mrpt::slam::TKLDParams &) --> class mrpt::slam::TKLDParams &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::slam::PF_implementation file:mrpt/slam/PF_implementations_data.h line:36
		pybind11::class_<mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>, std::shared_ptr<mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>>, PyCallBack_mrpt_slam_PF_implementation_mrpt_maps_CRBPFParticleData_mrpt_maps_CMultiMetricMapPDF_mrpt_bayes_particle_storage_mode_POINTER_t> cl(M("mrpt::slam"), "PF_implementation_mrpt_maps_CRBPFParticleData_mrpt_maps_CMultiMetricMapPDF_mrpt_bayes_particle_storage_mode_POINTER_t", "");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_slam_PF_implementation_mrpt_maps_CRBPFParticleData_mrpt_maps_CMultiMetricMapPDF_mrpt_bayes_particle_storage_mode_POINTER_t(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_slam_PF_implementation_mrpt_maps_CRBPFParticleData_mrpt_maps_CMultiMetricMapPDF_mrpt_bayes_particle_storage_mode_POINTER_t const &>());
		cl.def("getLastPose", (struct mrpt::math::TPose3D (mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>::*)(size_t, bool &) const) &mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::getLastPose, "C++: mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::getLastPose(size_t, bool &) const --> struct mrpt::math::TPose3D", pybind11::arg("i"), pybind11::arg("is_valid_pose"));
		cl.def("PF_SLAM_implementation_custom_update_particle_with_new_pose", (void (mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>::*)(class mrpt::maps::CRBPFParticleData *, const struct mrpt::math::TPose3D &) const) &mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::PF_SLAM_implementation_custom_update_particle_with_new_pose, "C++: mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::PF_SLAM_implementation_custom_update_particle_with_new_pose(class mrpt::maps::CRBPFParticleData *, const struct mrpt::math::TPose3D &) const --> void", pybind11::arg("particleData"), pybind11::arg("newPose"));
		cl.def("PF_SLAM_implementation_doWeHaveValidObservations", (bool (mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>::*)(const class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &, const class mrpt::obs::CSensoryFrame *) const) &mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::PF_SLAM_implementation_doWeHaveValidObservations, "C++: mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<class mrpt::maps::CRBPFParticleData, mrpt::bayes::particle_storage_mode::POINTER> > &, const class mrpt::obs::CSensoryFrame *) const --> bool", pybind11::arg("particles"), pybind11::arg("sf"));
		cl.def("PF_SLAM_implementation_skipRobotMovement", (bool (mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>::*)() const) &mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::PF_SLAM_implementation_skipRobotMovement, "C++: mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::PF_SLAM_implementation_skipRobotMovement() const --> bool");
		cl.def("PF_SLAM_computeObservationLikelihoodForParticle", (double (mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const) &mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::PF_SLAM_computeObservationLikelihoodForParticle, "C++: mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("PF_options"), pybind11::arg("particleIndexForMap"), pybind11::arg("observation"), pybind11::arg("x"));
		cl.def("assign", (class mrpt::slam::PF_implementation<class mrpt::maps::CRBPFParticleData, class mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER> & (mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData,mrpt::maps::CMultiMetricMapPDF,mrpt::bayes::particle_storage_mode::POINTER>::*)(const class mrpt::slam::PF_implementation<class mrpt::maps::CRBPFParticleData, class mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER> &)) &mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::operator=, "C++: mrpt::slam::PF_implementation<mrpt::maps::CRBPFParticleData, mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER>::operator=(const class mrpt::slam::PF_implementation<class mrpt::maps::CRBPFParticleData, class mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER> &) --> class mrpt::slam::PF_implementation<class mrpt::maps::CRBPFParticleData, class mrpt::maps::CMultiMetricMapPDF, mrpt::bayes::particle_storage_mode::POINTER> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::slam::PF_implementation file:mrpt/slam/PF_implementations_data.h line:36
		pybind11::class_<mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>, std::shared_ptr<mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>>, PyCallBack_mrpt_slam_PF_implementation_mrpt_math_TPose2D_mrpt_slam_CMonteCarloLocalization2D_mrpt_bayes_particle_storage_mode_VALUE_t> cl(M("mrpt::slam"), "PF_implementation_mrpt_math_TPose2D_mrpt_slam_CMonteCarloLocalization2D_mrpt_bayes_particle_storage_mode_VALUE_t", "");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_slam_PF_implementation_mrpt_math_TPose2D_mrpt_slam_CMonteCarloLocalization2D_mrpt_bayes_particle_storage_mode_VALUE_t(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_slam_PF_implementation_mrpt_math_TPose2D_mrpt_slam_CMonteCarloLocalization2D_mrpt_bayes_particle_storage_mode_VALUE_t const &>());
		cl.def("getLastPose", (struct mrpt::math::TPose3D (mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>::*)(size_t, bool &) const) &mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::getLastPose, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::getLastPose(size_t, bool &) const --> struct mrpt::math::TPose3D", pybind11::arg("i"), pybind11::arg("is_valid_pose"));
		cl.def("PF_SLAM_implementation_custom_update_particle_with_new_pose", (void (mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>::*)(struct mrpt::math::TPose2D *, const struct mrpt::math::TPose3D &) const) &mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_custom_update_particle_with_new_pose, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_custom_update_particle_with_new_pose(struct mrpt::math::TPose2D *, const struct mrpt::math::TPose3D &) const --> void", pybind11::arg("particleData"), pybind11::arg("newPose"));
		cl.def("PF_SLAM_implementation_doWeHaveValidObservations", (bool (mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>::*)(const class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> > &, const class mrpt::obs::CSensoryFrame *) const) &mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_doWeHaveValidObservations, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> > &, const class mrpt::obs::CSensoryFrame *) const --> bool", pybind11::arg("particles"), pybind11::arg("sf"));
		cl.def("PF_SLAM_implementation_skipRobotMovement", (bool (mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>::*)() const) &mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_skipRobotMovement, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_skipRobotMovement() const --> bool");
		cl.def("PF_SLAM_computeObservationLikelihoodForParticle", (double (mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const) &mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_computeObservationLikelihoodForParticle, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("PF_options"), pybind11::arg("particleIndexForMap"), pybind11::arg("observation"), pybind11::arg("x"));
		cl.def("assign", (class mrpt::slam::PF_implementation<struct mrpt::math::TPose2D, class mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE> & (mrpt::slam::PF_implementation<mrpt::math::TPose2D,mrpt::slam::CMonteCarloLocalization2D,mrpt::bayes::particle_storage_mode::VALUE>::*)(const class mrpt::slam::PF_implementation<struct mrpt::math::TPose2D, class mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE> &)) &mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::operator=, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose2D, mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE>::operator=(const class mrpt::slam::PF_implementation<struct mrpt::math::TPose2D, class mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE> &) --> class mrpt::slam::PF_implementation<struct mrpt::math::TPose2D, class mrpt::slam::CMonteCarloLocalization2D, mrpt::bayes::particle_storage_mode::VALUE> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::slam::PF_implementation file:mrpt/slam/PF_implementations_data.h line:36
		pybind11::class_<mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>, std::shared_ptr<mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>>, PyCallBack_mrpt_slam_PF_implementation_mrpt_math_TPose3D_mrpt_slam_CMonteCarloLocalization3D_mrpt_bayes_particle_storage_mode_VALUE_t> cl(M("mrpt::slam"), "PF_implementation_mrpt_math_TPose3D_mrpt_slam_CMonteCarloLocalization3D_mrpt_bayes_particle_storage_mode_VALUE_t", "");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_slam_PF_implementation_mrpt_math_TPose3D_mrpt_slam_CMonteCarloLocalization3D_mrpt_bayes_particle_storage_mode_VALUE_t(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_slam_PF_implementation_mrpt_math_TPose3D_mrpt_slam_CMonteCarloLocalization3D_mrpt_bayes_particle_storage_mode_VALUE_t const &>());
		cl.def("getLastPose", (struct mrpt::math::TPose3D (mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>::*)(size_t, bool &) const) &mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::getLastPose, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::getLastPose(size_t, bool &) const --> struct mrpt::math::TPose3D", pybind11::arg("i"), pybind11::arg("is_valid_pose"));
		cl.def("PF_SLAM_implementation_custom_update_particle_with_new_pose", (void (mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>::*)(struct mrpt::math::TPose3D *, const struct mrpt::math::TPose3D &) const) &mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_custom_update_particle_with_new_pose, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_custom_update_particle_with_new_pose(struct mrpt::math::TPose3D *, const struct mrpt::math::TPose3D &) const --> void", pybind11::arg("particleData"), pybind11::arg("newPose"));
		cl.def("PF_SLAM_implementation_doWeHaveValidObservations", (bool (mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>::*)(const class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> > &, const class mrpt::obs::CSensoryFrame *) const) &mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_doWeHaveValidObservations, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_doWeHaveValidObservations(const class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> > &, const class mrpt::obs::CSensoryFrame *) const --> bool", pybind11::arg("particles"), pybind11::arg("sf"));
		cl.def("PF_SLAM_implementation_skipRobotMovement", (bool (mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>::*)() const) &mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_skipRobotMovement, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_implementation_skipRobotMovement() const --> bool");
		cl.def("PF_SLAM_computeObservationLikelihoodForParticle", (double (mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const) &mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_computeObservationLikelihoodForParticle, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::PF_SLAM_computeObservationLikelihoodForParticle(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t, const class mrpt::obs::CSensoryFrame &, const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("PF_options"), pybind11::arg("particleIndexForMap"), pybind11::arg("observation"), pybind11::arg("x"));
		cl.def("assign", (class mrpt::slam::PF_implementation<struct mrpt::math::TPose3D, class mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE> & (mrpt::slam::PF_implementation<mrpt::math::TPose3D,mrpt::slam::CMonteCarloLocalization3D,mrpt::bayes::particle_storage_mode::VALUE>::*)(const class mrpt::slam::PF_implementation<struct mrpt::math::TPose3D, class mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE> &)) &mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::operator=, "C++: mrpt::slam::PF_implementation<mrpt::math::TPose3D, mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE>::operator=(const class mrpt::slam::PF_implementation<struct mrpt::math::TPose3D, class mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE> &) --> class mrpt::slam::PF_implementation<struct mrpt::math::TPose3D, class mrpt::slam::CMonteCarloLocalization3D, mrpt::bayes::particle_storage_mode::VALUE> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
