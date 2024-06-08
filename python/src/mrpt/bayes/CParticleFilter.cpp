#include <iterator>
#include <memory>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/maps/CMetricMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CAction.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
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

// mrpt::bayes::CParticleFilter::TParticleFilterOptions file:mrpt/bayes/CParticleFilter.h line:102
struct PyCallBack_mrpt_bayes_CParticleFilter_TParticleFilterOptions : public mrpt::bayes::CParticleFilter::TParticleFilterOptions {
	using mrpt::bayes::CParticleFilter::TParticleFilterOptions::TParticleFilterOptions;

	void loadFromConfigFile(const class mrpt::config::CConfigFileBase & a0, const std::string & a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilter::TParticleFilterOptions *>(this), "loadFromConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TParticleFilterOptions::loadFromConfigFile(a0, a1);
	}
	void saveToConfigFile(class mrpt::config::CConfigFileBase & a0, const std::string & a1) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::bayes::CParticleFilter::TParticleFilterOptions *>(this), "saveToConfigFile");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return TParticleFilterOptions::saveToConfigFile(a0, a1);
	}
};

void bind_mrpt_bayes_CParticleFilter(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::bayes::CParticleFilter file:mrpt/bayes/CParticleFilter.h line:51
		pybind11::class_<mrpt::bayes::CParticleFilter, std::shared_ptr<mrpt::bayes::CParticleFilter>> cl(M("mrpt::bayes"), "CParticleFilter", "This class acts as a common interface to the different interfaces (see\nCParticleFilter::TParticleFilterAlgorithm) any bayes::CParticleFilterCapable\nclass can implement: it is the invoker of particle filter algorithms.\n   The particle filter is executed on a probability density function (PDF)\ndescribed by a CParticleFilterCapable object, passed in the constructor or\nalternatively through the CParticleFilter::executeOn method.\n\n For a complete example and further details, see the \n*href=\"http://www.mrpt.org/Particle_Filter_Tutorial\" >Particle Filter\ntutorial.\n\n   The basic SIR algorithm (pfStandardProposal) consists of:\n		- Execute a prediction with the given \"action\".\n		- Update the weights of the particles using the likelihood of the\n\"observation\".\n		- Normalize weights.\n		- Perform resampling if the ESS is below the threshold options.BETA.\n\n \n\n \n mrpt::poses::CPoseParticlesPDF");
		cl.def( pybind11::init( [](){ return new mrpt::bayes::CParticleFilter(); } ) );
		cl.def( pybind11::init( [](mrpt::bayes::CParticleFilter const &o){ return new mrpt::bayes::CParticleFilter(o); } ) );

		pybind11::enum_<mrpt::bayes::CParticleFilter::TParticleFilterAlgorithm>(cl, "TParticleFilterAlgorithm", pybind11::arithmetic(), "Defines different types of particle filter algorithms.\n  The defined SIR implementations are:\n		- pfStandardProposal: Standard proposal distribution + weights\naccording to likelihood function.\n		- pfAuxiliaryPFStandard: An auxiliary PF using the standard proposal\ndistribution.\n		- pfOptimalProposal: Use the optimal proposal distribution (where\navailable!, usually this will perform approximations)\n		- pfAuxiliaryPFOptimal: Use the optimal proposal and a auxiliary\nparticle filter (see \n*href=\"http://www.mrpt.org/Paper:An_Optimal_Filtering_Algorithm_for_Non-Parametric_Observation_Models_in_Robot_Localization_(ICRA_2008)\"\n>paper).\n\n See the theoretical discussion in \n*href=\"http://www.mrpt.org/Resampling_Schemes\" >resampling schemes.")
			.value("pfStandardProposal", mrpt::bayes::CParticleFilter::pfStandardProposal)
			.value("pfAuxiliaryPFStandard", mrpt::bayes::CParticleFilter::pfAuxiliaryPFStandard)
			.value("pfOptimalProposal", mrpt::bayes::CParticleFilter::pfOptimalProposal)
			.value("pfAuxiliaryPFOptimal", mrpt::bayes::CParticleFilter::pfAuxiliaryPFOptimal)
			.export_values();


		pybind11::enum_<mrpt::bayes::CParticleFilter::TParticleResamplingAlgorithm>(cl, "TParticleResamplingAlgorithm", pybind11::arithmetic(), "Defines the different resampling algorithms.\n  The implemented resampling methods are:\n		- prMultinomial (Default): Uses standard select with replacement\n(draws\nM random uniform numbers)\n		- prResidual: The residual or \"remainder\" method.\n		- prStratified: The stratified resampling, where a uniform sample is\ndrawn for each of M subdivisions of the range (0,1].\n		- prSystematic: A single uniform sample is drawn in the range\n(0,1/M].\n\n See the theoretical discussion in \n*href=\"http://www.mrpt.org/Resampling_Schemes\" >resampling schemes.")
			.value("prMultinomial", mrpt::bayes::CParticleFilter::prMultinomial)
			.value("prResidual", mrpt::bayes::CParticleFilter::prResidual)
			.value("prStratified", mrpt::bayes::CParticleFilter::prStratified)
			.value("prSystematic", mrpt::bayes::CParticleFilter::prSystematic)
			.export_values();

		cl.def_readwrite("m_options", &mrpt::bayes::CParticleFilter::m_options);
		cl.def("executeOn", [](mrpt::bayes::CParticleFilter const &o, class mrpt::bayes::CParticleFilterCapable & a0, const class mrpt::obs::CActionCollection * a1, const class mrpt::obs::CSensoryFrame * a2) -> void { return o.executeOn(a0, a1, a2); }, "", pybind11::arg("obj"), pybind11::arg("action"), pybind11::arg("observation"));
		cl.def("executeOn", (void (mrpt::bayes::CParticleFilter::*)(class mrpt::bayes::CParticleFilterCapable &, const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, struct mrpt::bayes::CParticleFilter::TParticleFilterStats *) const) &mrpt::bayes::CParticleFilter::executeOn, "Executes a complete prediction + update step of the selected particle\n filtering algorithm.\n    The member CParticleFilter::m_options must be set before calling this\n to settle the algorithm parameters.\n\n \n           The object representing the probability distribution\n function (PDF) which apply the particle filter algorithm to.\n \n\n		A pointer to an action in the form of a\n CActionCollection,\n or nullptr if there is no action.\n \n\n	A pointer to observations in the form of a\n CSensoryFrame, or nullptr if there is no observation.\n \n\n An output structure for gathering statistics of the particle\n filter execution, or set to nullptr if you do not need it (see\n CParticleFilter::TParticleFilterStats).\n\n \n CParticleFilterCapable, executeOn\n\nC++: mrpt::bayes::CParticleFilter::executeOn(class mrpt::bayes::CParticleFilterCapable &, const class mrpt::obs::CActionCollection *, const class mrpt::obs::CSensoryFrame *, struct mrpt::bayes::CParticleFilter::TParticleFilterStats *) const --> void", pybind11::arg("obj"), pybind11::arg("action"), pybind11::arg("observation"), pybind11::arg("stats"));
		cl.def("assign", (class mrpt::bayes::CParticleFilter & (mrpt::bayes::CParticleFilter::*)(const class mrpt::bayes::CParticleFilter &)) &mrpt::bayes::CParticleFilter::operator=, "C++: mrpt::bayes::CParticleFilter::operator=(const class mrpt::bayes::CParticleFilter &) --> class mrpt::bayes::CParticleFilter &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::bayes::CParticleFilter::TParticleFilterOptions file:mrpt/bayes/CParticleFilter.h line:102
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::bayes::CParticleFilter::TParticleFilterOptions, std::shared_ptr<mrpt::bayes::CParticleFilter::TParticleFilterOptions>, PyCallBack_mrpt_bayes_CParticleFilter_TParticleFilterOptions, mrpt::config::CLoadableOptions> cl(enclosing_class, "TParticleFilterOptions", "The configuration of a particle filter.");
			cl.def( pybind11::init( [](){ return new mrpt::bayes::CParticleFilter::TParticleFilterOptions(); }, [](){ return new PyCallBack_mrpt_bayes_CParticleFilter_TParticleFilterOptions(); } ) );
			cl.def( pybind11::init( [](PyCallBack_mrpt_bayes_CParticleFilter_TParticleFilterOptions const &o){ return new PyCallBack_mrpt_bayes_CParticleFilter_TParticleFilterOptions(o); } ) );
			cl.def( pybind11::init( [](mrpt::bayes::CParticleFilter::TParticleFilterOptions const &o){ return new mrpt::bayes::CParticleFilter::TParticleFilterOptions(o); } ) );
			cl.def_readwrite("adaptiveSampleSize", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::adaptiveSampleSize);
			cl.def_readwrite("BETA", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::BETA);
			cl.def_readwrite("sampleSize", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::sampleSize);
			cl.def_readwrite("pfAuxFilterOptimal_MaximumSearchSamples", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::pfAuxFilterOptimal_MaximumSearchSamples);
			cl.def_readwrite("powFactor", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::powFactor);
			cl.def_readwrite("PF_algorithm", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::PF_algorithm);
			cl.def_readwrite("resamplingMethod", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::resamplingMethod);
			cl.def_readwrite("max_loglikelihood_dyn_range", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::max_loglikelihood_dyn_range);
			cl.def_readwrite("pfAuxFilterStandard_FirstStageWeightsMonteCarlo", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::pfAuxFilterStandard_FirstStageWeightsMonteCarlo);
			cl.def_readwrite("pfAuxFilterOptimal_MLE", &mrpt::bayes::CParticleFilter::TParticleFilterOptions::pfAuxFilterOptimal_MLE);
			cl.def("loadFromConfigFile", (void (mrpt::bayes::CParticleFilter::TParticleFilterOptions::*)(const class mrpt::config::CConfigFileBase &, const std::string &)) &mrpt::bayes::CParticleFilter::TParticleFilterOptions::loadFromConfigFile, "C++: mrpt::bayes::CParticleFilter::TParticleFilterOptions::loadFromConfigFile(const class mrpt::config::CConfigFileBase &, const std::string &) --> void", pybind11::arg("source"), pybind11::arg("section"));
			cl.def("saveToConfigFile", (void (mrpt::bayes::CParticleFilter::TParticleFilterOptions::*)(class mrpt::config::CConfigFileBase &, const std::string &) const) &mrpt::bayes::CParticleFilter::TParticleFilterOptions::saveToConfigFile, "C++: mrpt::bayes::CParticleFilter::TParticleFilterOptions::saveToConfigFile(class mrpt::config::CConfigFileBase &, const std::string &) const --> void", pybind11::arg("target"), pybind11::arg("section"));
			cl.def("assign", (struct mrpt::bayes::CParticleFilter::TParticleFilterOptions & (mrpt::bayes::CParticleFilter::TParticleFilterOptions::*)(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &)) &mrpt::bayes::CParticleFilter::TParticleFilterOptions::operator=, "C++: mrpt::bayes::CParticleFilter::TParticleFilterOptions::operator=(const struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &) --> struct mrpt::bayes::CParticleFilter::TParticleFilterOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::bayes::CParticleFilter::TParticleFilterStats file:mrpt/bayes/CParticleFilter.h line:167
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::bayes::CParticleFilter::TParticleFilterStats, std::shared_ptr<mrpt::bayes::CParticleFilter::TParticleFilterStats>> cl(enclosing_class, "TParticleFilterStats", "Statistics for being returned from the \"execute\" method. ");
			cl.def( pybind11::init( [](){ return new mrpt::bayes::CParticleFilter::TParticleFilterStats(); } ) );
			cl.def_readwrite("ESS_beforeResample", &mrpt::bayes::CParticleFilter::TParticleFilterStats::ESS_beforeResample);
			cl.def_readwrite("weightsVariance_beforeResample", &mrpt::bayes::CParticleFilter::TParticleFilterStats::weightsVariance_beforeResample);
		}

	}
}
