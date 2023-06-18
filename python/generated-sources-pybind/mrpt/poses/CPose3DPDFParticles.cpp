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
#include <mrpt/poses/CPoseRandomSampler.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CMessage.h>
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

// mrpt::poses::CPose3DPDFParticles file:mrpt/poses/CPose3DPDFParticles.h line:33
struct PyCallBack_mrpt_poses_CPose3DPDFParticles : public mrpt::poses::CPose3DPDFParticles {
	using mrpt::poses::CPose3DPDFParticles::CPose3DPDFParticles;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "GetRuntimeClass");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "clone");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "serializeGetVersion");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "serializeTo");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "serializeFrom");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "copyFrom");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "getMean");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "getCovarianceAndMean");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "saveToTextFile");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "changeCoordinatesReference");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "drawSingleSample");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "inverse");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "bayesianFusion");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "asString");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "isInfType");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "getInformationMatrix");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "getW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "setW");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "particlesCount");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "normalizeWeights");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "ESS");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "prediction_and_update_pfStandardProposal");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "prediction_and_update_pfAuxiliaryPFStandard");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "prediction_and_update_pfOptimalProposal");
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
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DPDFParticles *>(this), "prediction_and_update_pfAuxiliaryPFOptimal");
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

void bind_mrpt_poses_CPose3DPDFParticles(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose3DPDFParticles file:mrpt/poses/CPose3DPDFParticles.h line:33
		pybind11::class_<mrpt::poses::CPose3DPDFParticles, std::shared_ptr<mrpt::poses::CPose3DPDFParticles>, PyCallBack_mrpt_poses_CPose3DPDFParticles, mrpt::poses::CPose3DPDF, mrpt::bayes::CParticleFilterData<mrpt::math::TPose3D,mrpt::bayes::particle_storage_mode::VALUE>, mrpt::bayes::CParticleFilterDataImpl<mrpt::poses::CPose3DPDFParticles,std::deque<mrpt::bayes::CProbabilityParticle<mrpt::math::TPose3D, mrpt::bayes::particle_storage_mode::VALUE> >>, mrpt::Stringifyable> cl(M("mrpt::poses"), "CPose3DPDFParticles", "Declares a class that represents a Probability Density function (PDF) of a\n 3D pose\n\n  This class is also the base for the implementation of Monte-Carlo\n Localization (MCL), in mrpt::slam::CMonteCarloLocalization2D.\n\n  See the application \"app/pf-localization\" for an example of usage.\n\n \n CPose3D, CPose3DPDF, CPoseGaussianPDF\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DPDFParticles(); }, [](){ return new PyCallBack_mrpt_poses_CPose3DPDFParticles(); } ), "doc");
		cl.def( pybind11::init<size_t>(), pybind11::arg("M") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3DPDFParticles const &o){ return new PyCallBack_mrpt_poses_CPose3DPDFParticles(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3DPDFParticles const &o){ return new mrpt::poses::CPose3DPDFParticles(o); } ) );
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<32> (*)()) &mrpt::poses::CPose3DPDFParticles::getClassName, "C++: mrpt::poses::CPose3DPDFParticles::getClassName() --> class mrpt::typemeta::string_literal<32>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DPDFParticles::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DPDFParticles::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DPDFParticles::*)() const) &mrpt::poses::CPose3DPDFParticles::GetRuntimeClass, "C++: mrpt::poses::CPose3DPDFParticles::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3DPDFParticles::*)() const) &mrpt::poses::CPose3DPDFParticles::clone, "C++: mrpt::poses::CPose3DPDFParticles::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3DPDFParticles::CreateObject, "C++: mrpt::poses::CPose3DPDFParticles::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("copyFrom", (void (mrpt::poses::CPose3DPDFParticles::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFParticles::copyFrom, "Copy operator, translating if necesary (for example, between m_particles\n and gaussian representations) \n\nC++: mrpt::poses::CPose3DPDFParticles::copyFrom(const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("o"));
		cl.def("resetDeterministic", [](mrpt::poses::CPose3DPDFParticles &o, const struct mrpt::math::TPose3D & a0) -> void { return o.resetDeterministic(a0); }, "", pybind11::arg("location"));
		cl.def("resetDeterministic", (void (mrpt::poses::CPose3DPDFParticles::*)(const struct mrpt::math::TPose3D &, size_t)) &mrpt::poses::CPose3DPDFParticles::resetDeterministic, "Reset the PDF to a single point: All m_particles will be set exactly to\n the supplied pose.\n \n\n The location to set all the m_particles.\n \n\n If this is set to 0 the number of m_particles\n remains unchanged.\n  \n\n resetUniform \n\nC++: mrpt::poses::CPose3DPDFParticles::resetDeterministic(const struct mrpt::math::TPose3D &, size_t) --> void", pybind11::arg("location"), pybind11::arg("particlesCount"));
		cl.def("resetUniform", [](mrpt::poses::CPose3DPDFParticles &o, const struct mrpt::math::TPose3D & a0, const struct mrpt::math::TPose3D & a1) -> void { return o.resetUniform(a0, a1); }, "", pybind11::arg("corner_min"), pybind11::arg("corner_max"));
		cl.def("resetUniform", (void (mrpt::poses::CPose3DPDFParticles::*)(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const int)) &mrpt::poses::CPose3DPDFParticles::resetUniform, "Reset the PDF to an uniformly distributed one, inside of the defined\n \"cube\".\n\n \n New particle count, or leave count unchanged if set\n to -1 (default).\n\n \n Orientations can be outside of the [-pi,pi] range if so desired,\n       but it must hold `phi_max>=phi_min`.\n \n\n resetDeterministic\n resetAroundSetOfPoses\n\nC++: mrpt::poses::CPose3DPDFParticles::resetUniform(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const int) --> void", pybind11::arg("corner_min"), pybind11::arg("corner_max"), pybind11::arg("particlesCount"));
		cl.def("getMean", (void (mrpt::poses::CPose3DPDFParticles::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFParticles::getMean, "Returns an estimate of the pose, (the mean, or mathematical expectation\n of the PDF), computed as a weighted average over all m_particles. \n\n\n getCovariance \n\nC++: mrpt::poses::CPose3DPDFParticles::getMean(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("mean_pose"));
		cl.def("getCovarianceAndMean", (class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D> (mrpt::poses::CPose3DPDFParticles::*)() const) &mrpt::poses::CPose3DPDFParticles::getCovarianceAndMean, "Returns an estimate of the pose covariance matrix (6x6 cov matrix) and\n the mean, both at once. \n\n getMean \n\nC++: mrpt::poses::CPose3DPDFParticles::getCovarianceAndMean() const --> class std::tuple<class mrpt::math::CMatrixFixed<double, 6, 6>, class mrpt::poses::CPose3D>");
		cl.def("getParticlePose", (struct mrpt::math::TPose3D (mrpt::poses::CPose3DPDFParticles::*)(int) const) &mrpt::poses::CPose3DPDFParticles::getParticlePose, "Returns the pose of the i'th particle \n\nC++: mrpt::poses::CPose3DPDFParticles::getParticlePose(int) const --> struct mrpt::math::TPose3D", pybind11::arg("i"));
		cl.def("saveToTextFile", (bool (mrpt::poses::CPose3DPDFParticles::*)(const std::string &) const) &mrpt::poses::CPose3DPDFParticles::saveToTextFile, "Save PDF's m_particles to a text file. In each line it will go: \"x y z\"\n\nC++: mrpt::poses::CPose3DPDFParticles::saveToTextFile(const std::string &) const --> bool", pybind11::arg("file"));
		cl.def("size", (size_t (mrpt::poses::CPose3DPDFParticles::*)() const) &mrpt::poses::CPose3DPDFParticles::size, "Get the m_particles count (equivalent to \"particlesCount\") \n\nC++: mrpt::poses::CPose3DPDFParticles::size() const --> size_t");
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3DPDFParticles::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DPDFParticles::changeCoordinatesReference, "this = p (+) this. This can be used to convert a PDF from local\n coordinates to global, providing the point (newReferenceBase) from which\n   \"to project\" the current pdf. Result PDF substituted the currently\n stored one in the object. \n\nC++: mrpt::poses::CPose3DPDFParticles::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("newReferenceBase"));
		cl.def("drawSingleSample", (void (mrpt::poses::CPose3DPDFParticles::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3DPDFParticles::drawSingleSample, "Draws a single sample from the distribution (WARNING: weights are\n assumed to be normalized!) \n\nC++: mrpt::poses::CPose3DPDFParticles::drawSingleSample(class mrpt::poses::CPose3D &) const --> void", pybind11::arg("outPart"));
		cl.def("__iadd__", (void (mrpt::poses::CPose3DPDFParticles::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3DPDFParticles::operator+=, "Appends (pose-composition) a given pose \"p\" to each particle \n\nC++: mrpt::poses::CPose3DPDFParticles::operator+=(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("Ap"));
		cl.def("append", (void (mrpt::poses::CPose3DPDFParticles::*)(class mrpt::poses::CPose3DPDFParticles &)) &mrpt::poses::CPose3DPDFParticles::append, "Appends (add to the list) a set of m_particles to the existing ones, and\n then normalize weights. \n\nC++: mrpt::poses::CPose3DPDFParticles::append(class mrpt::poses::CPose3DPDFParticles &) --> void", pybind11::arg("o"));
		cl.def("inverse", (void (mrpt::poses::CPose3DPDFParticles::*)(class mrpt::poses::CPose3DPDF &) const) &mrpt::poses::CPose3DPDFParticles::inverse, "Returns a new PDF such as: NEW_PDF = (0,0,0) - THIS_PDF \n\nC++: mrpt::poses::CPose3DPDFParticles::inverse(class mrpt::poses::CPose3DPDF &) const --> void", pybind11::arg("o"));
		cl.def("getMostLikelyParticle", (struct mrpt::math::TPose3D (mrpt::poses::CPose3DPDFParticles::*)() const) &mrpt::poses::CPose3DPDFParticles::getMostLikelyParticle, "Returns the particle with the highest weight. \n\nC++: mrpt::poses::CPose3DPDFParticles::getMostLikelyParticle() const --> struct mrpt::math::TPose3D");
		cl.def("bayesianFusion", (void (mrpt::poses::CPose3DPDFParticles::*)(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPose3DPDFParticles::bayesianFusion, "Bayesian fusion \n\nC++: mrpt::poses::CPose3DPDFParticles::bayesianFusion(const class mrpt::poses::CPose3DPDF &, const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("p1"), pybind11::arg("p2"));
		cl.def("asString", (std::string (mrpt::poses::CPose3DPDFParticles::*)() const) &mrpt::poses::CPose3DPDFParticles::asString, "C++: mrpt::poses::CPose3DPDFParticles::asString() const --> std::string");
		cl.def("assign", (class mrpt::poses::CPose3DPDFParticles & (mrpt::poses::CPose3DPDFParticles::*)(const class mrpt::poses::CPose3DPDFParticles &)) &mrpt::poses::CPose3DPDFParticles::operator=, "C++: mrpt::poses::CPose3DPDFParticles::operator=(const class mrpt::poses::CPose3DPDFParticles &) --> class mrpt::poses::CPose3DPDFParticles &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPoseRandomSampler file:mrpt/poses/CPoseRandomSampler.h line:43
		pybind11::class_<mrpt::poses::CPoseRandomSampler, std::shared_ptr<mrpt::poses::CPoseRandomSampler>> cl(M("mrpt::poses"), "CPoseRandomSampler", "An efficient generator of random samples drawn from a given 2D (CPosePDF) or\n 3D (CPose3DPDF) pose probability density function (pdf).\n This class keeps an internal state which speeds up the sequential generation\n of samples. It can manage\n  any kind of pose PDF.\n\n Use with CPoseRandomSampler::setPosePDF, then CPoseRandomSampler::drawSample\n to draw values.\n\n Notice that you can pass a 2D or 3D pose PDF, then ask for a 2D or 3D sample.\n This class always returns\n  the kind of sample you ask it for, but will skip missing terms or fill out\n with zeroes as required.\n Specifically, when sampling 3D poses from a 2D pose pdf, this class will be\n smart enough to draw only\n  the 3 required dimensions, avoiding a waste of time with the other 3 missing\n components.\n\n \n\n \n CPosePDF, CPose3DPDF");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPoseRandomSampler(); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPoseRandomSampler const &o){ return new mrpt::poses::CPoseRandomSampler(o); } ) );
		cl.def("assign", (class mrpt::poses::CPoseRandomSampler & (mrpt::poses::CPoseRandomSampler::*)(const class mrpt::poses::CPoseRandomSampler &)) &mrpt::poses::CPoseRandomSampler::operator=, "C++: mrpt::poses::CPoseRandomSampler::operator=(const class mrpt::poses::CPoseRandomSampler &) --> class mrpt::poses::CPoseRandomSampler &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("setPosePDF", (void (mrpt::poses::CPoseRandomSampler::*)(const class mrpt::poses::CPosePDF &)) &mrpt::poses::CPoseRandomSampler::setPosePDF, "This method must be called to select the PDF from which to draw samples.\n \n\n drawSample\n\nC++: mrpt::poses::CPoseRandomSampler::setPosePDF(const class mrpt::poses::CPosePDF &) --> void", pybind11::arg("pdf"));
		cl.def("setPosePDF", (void (mrpt::poses::CPoseRandomSampler::*)(const class mrpt::poses::CPose3DPDF &)) &mrpt::poses::CPoseRandomSampler::setPosePDF, "This method must be called to select the PDF from which to draw samples.\n \n\n drawSample\n\nC++: mrpt::poses::CPoseRandomSampler::setPosePDF(const class mrpt::poses::CPose3DPDF &) --> void", pybind11::arg("pdf"));
		cl.def("drawSample", (class mrpt::poses::CPose2D & (mrpt::poses::CPoseRandomSampler::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPoseRandomSampler::drawSample, "Generate a new sample from the selected PDF.\n \n\n A reference to the same object passed as argument.\n \n\n setPosePDF\n\nC++: mrpt::poses::CPoseRandomSampler::drawSample(class mrpt::poses::CPose2D &) const --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("drawSample", (class mrpt::poses::CPose3D & (mrpt::poses::CPoseRandomSampler::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPoseRandomSampler::drawSample, "Generate a new sample from the selected PDF.\n \n\n A reference to the same object passed as argument.\n \n\n setPosePDF\n\nC++: mrpt::poses::CPoseRandomSampler::drawSample(class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("isPrepared", (bool (mrpt::poses::CPoseRandomSampler::*)() const) &mrpt::poses::CPoseRandomSampler::isPrepared, "Return true if samples can be generated, which only requires a previous\n call to setPosePDF \n\nC++: mrpt::poses::CPoseRandomSampler::isPrepared() const --> bool");
		cl.def("getSamplingMean2D", (class mrpt::poses::CPose2D & (mrpt::poses::CPoseRandomSampler::*)(class mrpt::poses::CPose2D &) const) &mrpt::poses::CPoseRandomSampler::getSamplingMean2D, "If the object has been loaded with setPosePDF this method returns the 2D\n pose mean samples will be drawn around. \n\n A reference to the\n argument \n\nC++: mrpt::poses::CPoseRandomSampler::getSamplingMean2D(class mrpt::poses::CPose2D &) const --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic, pybind11::arg("out_mean"));
		cl.def("getSamplingMean3D", (class mrpt::poses::CPose3D & (mrpt::poses::CPoseRandomSampler::*)(class mrpt::poses::CPose3D &) const) &mrpt::poses::CPoseRandomSampler::getSamplingMean3D, "If the object has been loaded with setPosePDF this method returns the 3D\n pose mean samples will be drawn around. \n\n A reference to the\n argument \n\nC++: mrpt::poses::CPoseRandomSampler::getSamplingMean3D(class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("out_mean"));
		cl.def("getOriginalPDFCov2D", (void (mrpt::poses::CPoseRandomSampler::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::poses::CPoseRandomSampler::getOriginalPDFCov2D, "Retrieves the 3x3 covariance of the original PDF in \n\n. \n\nC++: mrpt::poses::CPoseRandomSampler::getOriginalPDFCov2D(class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> void", pybind11::arg("cov3x3"));
		cl.def("getOriginalPDFCov2D", (void (mrpt::poses::CPoseRandomSampler::*)(class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::poses::CPoseRandomSampler::getOriginalPDFCov2D, "Retrieves the 3x3 covariance of the original PDF in \n\n. \n\nC++: mrpt::poses::CPoseRandomSampler::getOriginalPDFCov2D(class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("cov3x3"));
		cl.def("getOriginalPDFCov3D", (void (mrpt::poses::CPoseRandomSampler::*)(class mrpt::math::CMatrixFixed<double, 6, 6> &) const) &mrpt::poses::CPoseRandomSampler::getOriginalPDFCov3D, "Retrieves the 6x6 covariance of the original PDF in \n\n\n. \n\nC++: mrpt::poses::CPoseRandomSampler::getOriginalPDFCov3D(class mrpt::math::CMatrixFixed<double, 6, 6> &) const --> void", pybind11::arg("cov6x6"));
		cl.def("getOriginalPDFCov3D", (void (mrpt::poses::CPoseRandomSampler::*)(class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::poses::CPoseRandomSampler::getOriginalPDFCov3D, "Retrieves the 6x6 covariance of the original PDF in \n\n\n. \n\nC++: mrpt::poses::CPoseRandomSampler::getOriginalPDFCov3D(class mrpt::math::CMatrixDynamic<double> &) const --> void", pybind11::arg("cov6x6"));
	}
}
