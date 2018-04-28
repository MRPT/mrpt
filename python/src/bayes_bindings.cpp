/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/config/CLoadableOptions.h>

#include <mrpt/bayes/CProbabilityParticle.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/bayes/CParticleFilterCapable.h>
#include <mrpt/bayes/CKalmanFilterCapable.h>

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

#include <mrpt/math/lightweight_geom_data.h>

/* namespaces */
using namespace boost::python;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::bayes;
using namespace mrpt::config;

// aux typedefs
namespace mrpt
{
namespace bayes
{
typedef std::deque<CProbabilityParticle<TPose2D, particle_storage_mode::VALUE>>
	CParticle2DList;
typedef std::deque<CProbabilityParticle<TPose3D, particle_storage_mode::VALUE>>
	CParticle3DList;
}  // namespace bayes
}  // namespace mrpt

// CParticleFilter
CParticleFilter::TParticleFilterStats CParticleFilter_executeOn(
	CParticleFilter& self, CParticleFilterCapable& obj,
	const mrpt::obs::CActionCollection::Ptr action,
	const mrpt::obs::CSensoryFrame::Ptr observation)
{
	CParticleFilter::TParticleFilterStats stats;
	self.executeOn(obj, action.get(), observation.get(), &stats);
	return stats;
}
// end of CParticleFilter

// exporter
void export_bayes()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(bayes)

	// CParticleFilterCapable
	{
		scope s = class_<CParticleFilterCapable, boost::noncopyable>(
			"CParticleFilterCapable", no_init);
	}

	// CParticleFilter
	{
		scope s = class_<CParticleFilter, bases<CParticleFilterCapable>>(
					  "CParticleFilter", init<>())
					  .def("executeOn", &CParticleFilter_executeOn)
					  .def_readwrite("m_options", &CParticleFilter::m_options);

		// TParticleFilterAlgorithm
		enum_<CParticleFilter::TParticleFilterAlgorithm>(
			"TParticleFilterAlgorithm")
			.value("pfStandardProposal", CParticleFilter::pfStandardProposal)
			.value(
				"pfAuxiliaryPFStandard", CParticleFilter::pfAuxiliaryPFStandard)
			.value("pfOptimalProposal", CParticleFilter::pfOptimalProposal)
			.value(
				"pfAuxiliaryPFOptimal", CParticleFilter::pfAuxiliaryPFOptimal);

		// TParticleResamplingAlgorithm
		enum_<CParticleFilter::TParticleResamplingAlgorithm>(
			"TParticleResamplingAlgorithm")
			.value("prMultinomial", CParticleFilter::prMultinomial)
			.value("prResidual", CParticleFilter::prResidual)
			.value("prStratified", CParticleFilter::prStratified)
			.value("prSystematic", CParticleFilter::prSystematic);

		// TParticleFilterOptions
		class_<
			CParticleFilter::TParticleFilterOptions, bases<CLoadableOptions>>(
			"TParticleFilterOptions", init<>())
			.def_readwrite(
				"adaptiveSampleSize",
				&CParticleFilter::TParticleFilterOptions::adaptiveSampleSize)
			.def_readwrite(
				"BETA", &CParticleFilter::TParticleFilterOptions::BETA)
			.def_readwrite(
				"sampleSize",
				&CParticleFilter::TParticleFilterOptions::sampleSize)
			.def_readwrite(
				"pfAuxFilterOptimal_MaximumSearchSamples",
				&CParticleFilter::TParticleFilterOptions::
					pfAuxFilterOptimal_MaximumSearchSamples)
			.def_readwrite(
				"powFactor",
				&CParticleFilter::TParticleFilterOptions::powFactor)
			.def_readwrite(
				"PF_algorithm",
				&CParticleFilter::TParticleFilterOptions::PF_algorithm)
			.def_readwrite(
				"resamplingMethod",
				&CParticleFilter::TParticleFilterOptions::resamplingMethod)
			.def_readwrite(
				"max_loglikelihood_dyn_range",
				&CParticleFilter::TParticleFilterOptions::
					max_loglikelihood_dyn_range)
			.def_readwrite(
				"pfAuxFilterStandard_FirstStageWeightsMonteCarlo",
				&CParticleFilter::TParticleFilterOptions::
					pfAuxFilterStandard_FirstStageWeightsMonteCarlo)
			.def_readwrite(
				"pfAuxFilterOptimal_MLE",
				&CParticleFilter::TParticleFilterOptions::
					pfAuxFilterOptimal_MLE);

		// TParticleFilterOptions
		class_<CParticleFilter::TParticleFilterStats>(
			"TParticleFilterStats", init<>())
			.def_readwrite(
				"ESS_beforeResample",
				&CParticleFilter::TParticleFilterStats::ESS_beforeResample)
			.def_readwrite(
				"weightsVariance_beforeResample",
				&CParticleFilter::TParticleFilterStats::
					weightsVariance_beforeResample);
	}

	// CParticleList
	{
#if 0
		class_<CParticle2DList>("CParticle2DList", init<>())
			.def("__len__", &CParticle2DList::size)
			.def("clear", &CParticle2DList::clear)
			.def(
				"append", &StlListLike<CParticle2DList>::add,
				with_custodian_and_ward<1, 2>())  // to let container keep value
			.def(
				"__getitem__", &StlListLike<CParticle2DList>::get,
				return_value_policy<copy_non_const_reference>())
			.def(
				"__setitem__", &StlListLike<CParticle2DList>::set,
				with_custodian_and_ward<1, 2>())  // to let container keep value
			.def("__delitem__", &StlListLike<CParticle2DList>::del);

		class_<CParticle3DList>("CParticle3DList", init<>())
			.def("__len__", &CParticle3DList::size)
			.def("clear", &CParticle3DList::clear)
			.def(
				"append", &StlListLike<CParticle3DList>::add,
				with_custodian_and_ward<1, 2>())  // to let container keep value
			.def(
				"__getitem__", &StlListLike<CParticle3DList>::get,
				return_value_policy<copy_non_const_reference>())
			.def(
				"__setitem__", &StlListLike<CParticle3DList>::set,
				with_custodian_and_ward<1, 2>())  // to let container keep value
			.def("__delitem__", &StlListLike<CParticle3DList>::del);
#endif
	}
}
