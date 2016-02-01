/* bindings */
#include "bindings.h"

/* MRPT */
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/math/lightweight_geom_data.h>

/* namespaces */
using namespace boost::python;
using namespace mrpt::bayes;
using namespace mrpt::utils;


// exporter
void export_bayes()
{
    // map namespace to be submodule of mrpt package
    MAKE_SUBMODULE(bayes)

    // CParticleFilter
    {
        scope s = class_<CParticleFilter>("CParticleFilter", init<CParticleFilter>())
            .def("executeOn", &CParticleFilter::executeOn)
            .def_readwrite("m_options", &CParticleFilter::m_options)
        ;

        // TParticleFilterAlgorithm
        enum_<CParticleFilter::TParticleFilterAlgorithm>("TParticleFilterAlgorithm")
            .value("pfStandardProposal", CParticleFilter::pfStandardProposal)
            .value("pfAuxiliaryPFStandard", CParticleFilter::pfAuxiliaryPFStandard)
            .value("pfOptimalProposal", CParticleFilter::pfOptimalProposal)
            .value("pfAuxiliaryPFOptimal", CParticleFilter::pfAuxiliaryPFOptimal)
        ;

        // TParticleResamplingAlgorithm
        enum_<CParticleFilter::TParticleResamplingAlgorithm>("TParticleResamplingAlgorithm")
            .value("prMultinomial", CParticleFilter::prMultinomial)
            .value("prResidual", CParticleFilter::prResidual)
            .value("prStratified", CParticleFilter::prStratified)
            .value("prSystematic", CParticleFilter::prSystematic)
        ;

        // TParticleFilterOptions
        class_<CParticleFilter::TParticleFilterOptions, bases<CLoadableOptions> >("TParticleFilterOptions", init<>())
            .def_readwrite("adaptiveSampleSize", &CParticleFilter::TParticleFilterOptions::adaptiveSampleSize)
            .def_readwrite("BETA", &CParticleFilter::TParticleFilterOptions::BETA)
            .def_readwrite("sampleSize", &CParticleFilter::TParticleFilterOptions::sampleSize)
            .def_readwrite("pfAuxFilterOptimal_MaximumSearchSamples", &CParticleFilter::TParticleFilterOptions::pfAuxFilterOptimal_MaximumSearchSamples)
            .def_readwrite("powFactor", &CParticleFilter::TParticleFilterOptions::powFactor)
            .def_readwrite("PF_algorithm", &CParticleFilter::TParticleFilterOptions::PF_algorithm)
            .def_readwrite("resamplingMethod", &CParticleFilter::TParticleFilterOptions::resamplingMethod)
            .def_readwrite("max_loglikelihood_dyn_range", &CParticleFilter::TParticleFilterOptions::max_loglikelihood_dyn_range)
            .def_readwrite("pfAuxFilterStandard_FirstStageWeightsMonteCarlo", &CParticleFilter::TParticleFilterOptions::pfAuxFilterStandard_FirstStageWeightsMonteCarlo)
            .def_readwrite("verbose", &CParticleFilter::TParticleFilterOptions::verbose)
            .def_readwrite("pfAuxFilterOptimal_MLE", &CParticleFilter::TParticleFilterOptions::pfAuxFilterOptimal_MLE)
        ;

        // TParticleFilterOptions
        class_<CParticleFilter::TParticleFilterStats>("TParticleFilterStats", init<>())
            .def_readwrite("ESS_beforeResample", &CParticleFilter::TParticleFilterStats::ESS_beforeResample)
            .def_readwrite("weightsVariance_beforeResample", &CParticleFilter::TParticleFilterStats::weightsVariance_beforeResample)
        ;
    }
}
