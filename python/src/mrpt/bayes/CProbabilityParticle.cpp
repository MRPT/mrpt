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
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
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
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <string>
#include <tuple>
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

// mrpt::bayes::CParticleFilterDataImpl file:mrpt/bayes/CParticleFilterData.h line:32
struct PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPose3DPDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose3D_mrpt_bayes_particle_storage_mode_VALUE_t : public mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> {
	using mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>::CParticleFilterDataImpl;

	double getW(size_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "getW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "setW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "particlesCount");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "normalizeWeights");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "ESS");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "prediction_and_update_pfStandardProposal");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "prediction_and_update_pfAuxiliaryPFStandard");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "prediction_and_update_pfOptimalProposal");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "prediction_and_update_pfAuxiliaryPFOptimal");
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

// mrpt::bayes::CParticleFilterDataImpl file:mrpt/bayes/CParticleFilterData.h line:32
struct PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPosePDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose2D_mrpt_bayes_particle_storage_mode_VALUE_t : public mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> {
	using mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>::CParticleFilterDataImpl;

	double getW(size_t a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "getW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "setW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "particlesCount");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "normalizeWeights");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "ESS");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "prediction_and_update_pfStandardProposal");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "prediction_and_update_pfAuxiliaryPFStandard");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "prediction_and_update_pfOptimalProposal");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> *>(this), "prediction_and_update_pfAuxiliaryPFOptimal");
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

void bind_mrpt_bayes_CProbabilityParticle(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::bayes::particle_storage_mode file:mrpt/bayes/CProbabilityParticle.h line:18
	pybind11::enum_<mrpt::bayes::particle_storage_mode>(M("mrpt::bayes"), "particle_storage_mode", "use for CProbabilityParticle\n \n\n\n ")
		.value("VALUE", mrpt::bayes::particle_storage_mode::VALUE)
		.value("POINTER", mrpt::bayes::particle_storage_mode::POINTER);

;

	{ // mrpt::bayes::CProbabilityParticleBase file:mrpt/bayes/CProbabilityParticle.h line:38
		pybind11::class_<mrpt::bayes::CProbabilityParticleBase, std::shared_ptr<mrpt::bayes::CProbabilityParticleBase>> cl(M("mrpt::bayes"), "CProbabilityParticleBase", "");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CProbabilityParticleBase(); } ), "doc" );
		cl.def( pybind11::init<double>(), pybind11::arg("logw") );

		cl.def( pybind11::init( [](mrpt::bayes::CProbabilityParticleBase const &o){ return new mrpt::bayes::CProbabilityParticleBase(o); } ) );
		cl.def_readwrite("log_w", &mrpt::bayes::CProbabilityParticleBase::log_w);
		cl.def("assign", (struct mrpt::bayes::CProbabilityParticleBase & (mrpt::bayes::CProbabilityParticleBase::*)(const struct mrpt::bayes::CProbabilityParticleBase &)) &mrpt::bayes::CProbabilityParticleBase::operator=, "C++: mrpt::bayes::CProbabilityParticleBase::operator=(const struct mrpt::bayes::CProbabilityParticleBase &) --> struct mrpt::bayes::CProbabilityParticleBase &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::bayes::CParticleFilterDataImpl file:mrpt/bayes/CParticleFilterData.h line:32
		pybind11::class_<mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>, std::shared_ptr<mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>>, PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPose3DPDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose3D_mrpt_bayes_particle_storage_mode_VALUE_t, mrpt::bayes::CParticleFilterCapable> cl(M("mrpt::bayes"), "CParticleFilterDataImpl_mrpt_poses_CPose3DPDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose3D_mrpt_bayes_particle_storage_mode_VALUE_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>(); }, [](){ return new PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPose3DPDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose3D_mrpt_bayes_particle_storage_mode_VALUE_t(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPose3DPDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose3D_mrpt_bayes_particle_storage_mode_VALUE_t const &o){ return new PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPose3DPDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose3D_mrpt_bayes_particle_storage_mode_VALUE_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> const &o){ return new mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>(o); } ) );
		cl.def("derived", (class mrpt::poses::CPose3DPDFParticles & (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)()) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::derived, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::derived() --> class mrpt::poses::CPose3DPDFParticles &", pybind11::return_value_policy::automatic);
		cl.def("getW", (double (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)(size_t) const) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::getW, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::getW(size_t) const --> double", pybind11::arg("i"));
		cl.def("setW", (void (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)(size_t, double)) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::setW, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::setW(size_t, double) --> void", pybind11::arg("i"), pybind11::arg("w"));
		cl.def("particlesCount", (size_t (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)() const) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::particlesCount, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::particlesCount() const --> size_t");
		cl.def("normalizeWeights", [](mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >> &o) -> double { return o.normalizeWeights(); }, "");
		cl.def("normalizeWeights", (double (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)(double *)) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::normalizeWeights, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::normalizeWeights(double *) --> double", pybind11::arg("out_max_log_w"));
		cl.def("ESS", (double (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)() const) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::ESS, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::ESS() const --> double");
		cl.def("assign", (struct mrpt::bayes::CParticleFilterDataImpl<class mrpt::poses::CPose3DPDFParticles, class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> > > & (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)(const struct mrpt::bayes::CParticleFilterDataImpl<class mrpt::poses::CPose3DPDFParticles, class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> > > &)) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::operator=, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE>>>::operator=(const struct mrpt::bayes::CParticleFilterDataImpl<class mrpt::poses::CPose3DPDFParticles, class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> > > &) --> struct mrpt::bayes::CParticleFilterDataImpl<class mrpt::poses::CPose3DPDFParticles, class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> > > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_static("defaultEvaluator", (double (*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, const class mrpt::bayes::CParticleFilterCapable *, size_t, const void *, const void *)) &mrpt::bayes::CParticleFilterCapable::defaultEvaluator, "The default evaluator function, which simply returns the particle\n weight.\n  The action and the observation are declared as \"void*\" for a greater\n flexibility.\n \n\n prepareFastDrawSample\n\nC++: mrpt::bayes::CParticleFilterCapable::defaultEvaluator(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, const class mrpt::bayes::CParticleFilterCapable *, size_t, const void *, const void *) --> double", pybind11::arg("PF_options"), pybind11::arg("obj"), pybind11::arg("index"), pybind11::arg("action"), pybind11::arg("observation"));
		cl.def("fastDrawSample", (size_t (mrpt::bayes::CParticleFilterCapable::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) const) &mrpt::bayes::CParticleFilterCapable::fastDrawSample, "Draws a random sample from the particle filter, in such a way that each\nparticle has a probability proportional to its weight (in the standard PF\nalgorithm).\n   This method can be used to generate a variable number of m_particles\nwhen resampling: to vary the number of m_particles in the filter.\n   See prepareFastDrawSample for more information, or the \n*href=\"http://www.mrpt.org/Particle_Filters\" >Particle Filter\ntutorial.\n\n NOTES:\n		- You MUST call \"prepareFastDrawSample\" ONCE before calling this\nmethod. That method must be called after modifying the particle filter\n(executing one step, resampling, etc...)\n		- This method returns ONE index for the selected (\"drawn\") particle,\nin\nthe range [0,M-1]\n		- You do not need to call \"normalizeWeights\" before calling this.\n \n\n prepareFastDrawSample\n\nC++: mrpt::bayes::CParticleFilterCapable::fastDrawSample(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) const --> size_t", pybind11::arg("PF_options"));
		cl.def("getW", (double (mrpt::bayes::CParticleFilterCapable::*)(size_t) const) &mrpt::bayes::CParticleFilterCapable::getW, "Access to i'th particle (logarithm) weight, where first one is index 0.\n\nC++: mrpt::bayes::CParticleFilterCapable::getW(size_t) const --> double", pybind11::arg("i"));
		cl.def("setW", (void (mrpt::bayes::CParticleFilterCapable::*)(size_t, double)) &mrpt::bayes::CParticleFilterCapable::setW, "Modifies i'th particle (logarithm) weight, where first one is index 0.\n\nC++: mrpt::bayes::CParticleFilterCapable::setW(size_t, double) --> void", pybind11::arg("i"), pybind11::arg("w"));
		cl.def("particlesCount", (size_t (mrpt::bayes::CParticleFilterCapable::*)() const) &mrpt::bayes::CParticleFilterCapable::particlesCount, "Get the m_particles count.\n\nC++: mrpt::bayes::CParticleFilterCapable::particlesCount() const --> size_t");
		cl.def("prediction_and_update", (void (mrpt::bayes::CParticleFilterCapable::*)(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &)) &mrpt::bayes::CParticleFilterCapable::prediction_and_update, "Performs the prediction stage of the Particle Filter.\n  This method simply selects the appropiate protected method according to\n the particle filter algorithm to run.\n \n\n\n prediction_and_update_pfStandardProposal,prediction_and_update_pfAuxiliaryPFStandard,prediction_and_update_pfOptimalProposal,prediction_and_update_pfAuxiliaryPFOptimal\n\nC++: mrpt::bayes::CParticleFilterCapable::prediction_and_update(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) --> void", pybind11::arg("action"), pybind11::arg("observation"), pybind11::arg("PF_options"));
		cl.def("normalizeWeights", [](mrpt::bayes::CParticleFilterCapable &o) -> double { return o.normalizeWeights(); }, "");
		cl.def("normalizeWeights", (double (mrpt::bayes::CParticleFilterCapable::*)(double *)) &mrpt::bayes::CParticleFilterCapable::normalizeWeights, "Normalize the (logarithmic) weights, such as the maximum weight is zero.\n \n\n If provided, will return with the maximum log_w\n before normalizing, such as new_weights = old_weights - max_log_w.\n \n\n The max/min ratio of weights (\"dynamic range\")\n\nC++: mrpt::bayes::CParticleFilterCapable::normalizeWeights(double *) --> double", pybind11::arg("out_max_log_w"));
		cl.def("ESS", (double (mrpt::bayes::CParticleFilterCapable::*)() const) &mrpt::bayes::CParticleFilterCapable::ESS, "Returns the normalized ESS (Estimated Sample Size), in the range [0,1].\n  Note that you do NOT need to normalize the weights before calling this.\n\nC++: mrpt::bayes::CParticleFilterCapable::ESS() const --> double");
		cl.def("performResampling", [](mrpt::bayes::CParticleFilterCapable &o, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a0) -> void { return o.performResampling(a0); }, "", pybind11::arg("PF_options"));
		cl.def("performResampling", (void (mrpt::bayes::CParticleFilterCapable::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t)) &mrpt::bayes::CParticleFilterCapable::performResampling, "Performs a resample of the m_particles, using the method selected in the\n constructor.\n After computing the surviving samples, this method internally calls\n \"performSubstitution\" to actually perform the particle replacement.\n This method is called automatically by CParticleFilter::execute,\n andshould not be invoked manually normally.\n To just obtaining the sequence of resampled indexes from a sequence of\n weights, use \"resample\"\n \n\n The desired number of output particles\n after resampling; 0 means don't modify the current number.\n \n\n resample\n\nC++: mrpt::bayes::CParticleFilterCapable::performResampling(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t) --> void", pybind11::arg("PF_options"), pybind11::arg("out_particle_count"));
		cl.def("assign", (class mrpt::bayes::CParticleFilterCapable & (mrpt::bayes::CParticleFilterCapable::*)(const class mrpt::bayes::CParticleFilterCapable &)) &mrpt::bayes::CParticleFilterCapable::operator=, "C++: mrpt::bayes::CParticleFilterCapable::operator=(const class mrpt::bayes::CParticleFilterCapable &) --> class mrpt::bayes::CParticleFilterCapable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::bayes::CParticleFilterDataImpl file:mrpt/bayes/CParticleFilterData.h line:32
		pybind11::class_<mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>, std::shared_ptr<mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>>, PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPosePDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose2D_mrpt_bayes_particle_storage_mode_VALUE_t, mrpt::bayes::CParticleFilterCapable> cl(M("mrpt::bayes"), "CParticleFilterDataImpl_mrpt_poses_CPosePDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose2D_mrpt_bayes_particle_storage_mode_VALUE_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>(); }, [](){ return new PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPosePDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose2D_mrpt_bayes_particle_storage_mode_VALUE_t(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPosePDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose2D_mrpt_bayes_particle_storage_mode_VALUE_t const &o){ return new PyCallBack_mrpt_bayes_CParticleFilterDataImpl_mrpt_poses_CPosePDFParticles_std_deque_mrpt_bayes_CProbabilityParticle_mrpt_math_TPose2D_mrpt_bayes_particle_storage_mode_VALUE_t(o); } ) );
		cl.def( pybind11::init( [](mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> const &o){ return new mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>(o); } ) );
		cl.def("derived", (class mrpt::poses::CPosePDFParticles & (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)()) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::derived, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::derived() --> class mrpt::poses::CPosePDFParticles &", pybind11::return_value_policy::automatic);
		cl.def("getW", (double (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)(size_t) const) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::getW, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::getW(size_t) const --> double", pybind11::arg("i"));
		cl.def("setW", (void (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)(size_t, double)) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::setW, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::setW(size_t, double) --> void", pybind11::arg("i"), pybind11::arg("w"));
		cl.def("particlesCount", (size_t (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)() const) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::particlesCount, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::particlesCount() const --> size_t");
		cl.def("normalizeWeights", [](mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >> &o) -> double { return o.normalizeWeights(); }, "");
		cl.def("normalizeWeights", (double (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)(double *)) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::normalizeWeights, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::normalizeWeights(double *) --> double", pybind11::arg("out_max_log_w"));
		cl.def("ESS", (double (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)() const) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::ESS, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::ESS() const --> double");
		cl.def("assign", (struct mrpt::bayes::CParticleFilterDataImpl<class mrpt::poses::CPosePDFParticles, class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> > > & (mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> >>::*)(const struct mrpt::bayes::CParticleFilterDataImpl<class mrpt::poses::CPosePDFParticles, class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> > > &)) &mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::operator=, "C++: mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPosePDFParticles, std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE>>>::operator=(const struct mrpt::bayes::CParticleFilterDataImpl<class mrpt::poses::CPosePDFParticles, class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> > > &) --> struct mrpt::bayes::CParticleFilterDataImpl<class mrpt::poses::CPosePDFParticles, class std::deque<struct mrpt::bayes::CProbabilityParticle<struct mrpt::math::TPose2D, mrpt::bayes::particle_storage_mode::VALUE> > > &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_static("defaultEvaluator", (double (*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, const class mrpt::bayes::CParticleFilterCapable *, size_t, const void *, const void *)) &mrpt::bayes::CParticleFilterCapable::defaultEvaluator, "The default evaluator function, which simply returns the particle\n weight.\n  The action and the observation are declared as \"void*\" for a greater\n flexibility.\n \n\n prepareFastDrawSample\n\nC++: mrpt::bayes::CParticleFilterCapable::defaultEvaluator(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, const class mrpt::bayes::CParticleFilterCapable *, size_t, const void *, const void *) --> double", pybind11::arg("PF_options"), pybind11::arg("obj"), pybind11::arg("index"), pybind11::arg("action"), pybind11::arg("observation"));
		cl.def("fastDrawSample", (size_t (mrpt::bayes::CParticleFilterCapable::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) const) &mrpt::bayes::CParticleFilterCapable::fastDrawSample, "Draws a random sample from the particle filter, in such a way that each\nparticle has a probability proportional to its weight (in the standard PF\nalgorithm).\n   This method can be used to generate a variable number of m_particles\nwhen resampling: to vary the number of m_particles in the filter.\n   See prepareFastDrawSample for more information, or the \n*href=\"http://www.mrpt.org/Particle_Filters\" >Particle Filter\ntutorial.\n\n NOTES:\n		- You MUST call \"prepareFastDrawSample\" ONCE before calling this\nmethod. That method must be called after modifying the particle filter\n(executing one step, resampling, etc...)\n		- This method returns ONE index for the selected (\"drawn\") particle,\nin\nthe range [0,M-1]\n		- You do not need to call \"normalizeWeights\" before calling this.\n \n\n prepareFastDrawSample\n\nC++: mrpt::bayes::CParticleFilterCapable::fastDrawSample(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) const --> size_t", pybind11::arg("PF_options"));
		cl.def("getW", (double (mrpt::bayes::CParticleFilterCapable::*)(size_t) const) &mrpt::bayes::CParticleFilterCapable::getW, "Access to i'th particle (logarithm) weight, where first one is index 0.\n\nC++: mrpt::bayes::CParticleFilterCapable::getW(size_t) const --> double", pybind11::arg("i"));
		cl.def("setW", (void (mrpt::bayes::CParticleFilterCapable::*)(size_t, double)) &mrpt::bayes::CParticleFilterCapable::setW, "Modifies i'th particle (logarithm) weight, where first one is index 0.\n\nC++: mrpt::bayes::CParticleFilterCapable::setW(size_t, double) --> void", pybind11::arg("i"), pybind11::arg("w"));
		cl.def("particlesCount", (size_t (mrpt::bayes::CParticleFilterCapable::*)() const) &mrpt::bayes::CParticleFilterCapable::particlesCount, "Get the m_particles count.\n\nC++: mrpt::bayes::CParticleFilterCapable::particlesCount() const --> size_t");
		cl.def("prediction_and_update", (void (mrpt::bayes::CParticleFilterCapable::*)(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &)) &mrpt::bayes::CParticleFilterCapable::prediction_and_update, "Performs the prediction stage of the Particle Filter.\n  This method simply selects the appropiate protected method according to\n the particle filter algorithm to run.\n \n\n\n prediction_and_update_pfStandardProposal,prediction_and_update_pfAuxiliaryPFStandard,prediction_and_update_pfOptimalProposal,prediction_and_update_pfAuxiliaryPFOptimal\n\nC++: mrpt::bayes::CParticleFilterCapable::prediction_and_update(const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) --> void", pybind11::arg("action"), pybind11::arg("observation"), pybind11::arg("PF_options"));
		cl.def("normalizeWeights", [](mrpt::bayes::CParticleFilterCapable &o) -> double { return o.normalizeWeights(); }, "");
		cl.def("normalizeWeights", (double (mrpt::bayes::CParticleFilterCapable::*)(double *)) &mrpt::bayes::CParticleFilterCapable::normalizeWeights, "Normalize the (logarithmic) weights, such as the maximum weight is zero.\n \n\n If provided, will return with the maximum log_w\n before normalizing, such as new_weights = old_weights - max_log_w.\n \n\n The max/min ratio of weights (\"dynamic range\")\n\nC++: mrpt::bayes::CParticleFilterCapable::normalizeWeights(double *) --> double", pybind11::arg("out_max_log_w"));
		cl.def("ESS", (double (mrpt::bayes::CParticleFilterCapable::*)() const) &mrpt::bayes::CParticleFilterCapable::ESS, "Returns the normalized ESS (Estimated Sample Size), in the range [0,1].\n  Note that you do NOT need to normalize the weights before calling this.\n\nC++: mrpt::bayes::CParticleFilterCapable::ESS() const --> double");
		cl.def("performResampling", [](mrpt::bayes::CParticleFilterCapable &o, const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & a0) -> void { return o.performResampling(a0); }, "", pybind11::arg("PF_options"));
		cl.def("performResampling", (void (mrpt::bayes::CParticleFilterCapable::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t)) &mrpt::bayes::CParticleFilterCapable::performResampling, "Performs a resample of the m_particles, using the method selected in the\n constructor.\n After computing the surviving samples, this method internally calls\n \"performSubstitution\" to actually perform the particle replacement.\n This method is called automatically by CParticleFilter::execute,\n andshould not be invoked manually normally.\n To just obtaining the sequence of resampled indexes from a sequence of\n weights, use \"resample\"\n \n\n The desired number of output particles\n after resampling; 0 means don't modify the current number.\n \n\n resample\n\nC++: mrpt::bayes::CParticleFilterCapable::performResampling(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &, size_t) --> void", pybind11::arg("PF_options"), pybind11::arg("out_particle_count"));
		cl.def("assign", (class mrpt::bayes::CParticleFilterCapable & (mrpt::bayes::CParticleFilterCapable::*)(const class mrpt::bayes::CParticleFilterCapable &)) &mrpt::bayes::CParticleFilterCapable::operator=, "C++: mrpt::bayes::CParticleFilterCapable::operator=(const class mrpt::bayes::CParticleFilterCapable &) --> class mrpt::bayes::CParticleFilterCapable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
