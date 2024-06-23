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

void bind_mrpt_bayes_CParticleFilterCapable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::bayes::CParticleFilterCapable file:mrpt/bayes/CParticleFilterCapable.h line:33
		pybind11::class_<mrpt::bayes::CParticleFilterCapable, std::shared_ptr<mrpt::bayes::CParticleFilterCapable>> cl(M("mrpt::bayes"), "CParticleFilterCapable", "This virtual class defines the interface that any particles based PDF class\n must implement in order to be executed by a mrpt::bayes::CParticleFilter.\n\n See the Particle\n Filter tutorial explaining how to use the particle filter-related\n classes.\n \n\n CParticleFilter, CParticleFilterData\n \n\n\n ");
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
