/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#ifndef CLocalMetricHypothesis_impl_H
#define CLocalMetricHypothesis_impl_H

#include <mrpt/hmtslam/CLocalMetricHypothesis.h>
#include <mrpt/slam/PF_implementations.h>
namespace mrpt
{
namespace hmtslam
{
template <class PF_ALGORITHM>
void CLocalMetricHypothesis::prediction_and_update(
	const mrpt::obs::CActionCollection* action,
	const mrpt::obs::CSensoryFrame* observation,
	const bayes::CParticleFilter::TParticleFilterOptions& PF_options)
{
	ASSERT_(m_parent.get());
	ASSERT_(m_parent->m_LSLAM_method);
	m_parent->m_LSLAM_method->prediction_and_update<PF_ALGORITHM>(
		this, action, observation, PF_options);
}

}  // namespace hmtslam
}  // namespace mrpt
#endif
