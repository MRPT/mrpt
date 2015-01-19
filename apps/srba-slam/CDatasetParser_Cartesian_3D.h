/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include "CDatasetParserBase.h"

template <>
struct CDatasetParserTempl<mrpt::srba::observations::Cartesian_3D> : public CDatasetParserBase
{
	double m_noise_std;

	CDatasetParserTempl(RBASLAM_Params &cfg) :
		CDatasetParserBase(cfg),
		m_noise_std(1e-3)
	{
		if (cfg.arg_noise.isSet())
			m_noise_std=cfg.arg_noise.getValue();
	}

	virtual void checkObsProperSize() const
	{
		// Columns: KeyframeIndex  LandmarkID | X Y Z
		ASSERT_(m_OBS.getColCount()==(2+3))
	}

	void getObs(
		size_t idx,
		mrpt::srba::observation_traits<mrpt::srba::observations::Cartesian_3D>::observation_t & o
		) const
	{
		o.feat_id = m_OBS(idx,1);
		o.obs_data.pt.x = m_OBS(idx,2) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std));
		o.obs_data.pt.y = m_OBS(idx,3) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std));
		o.obs_data.pt.z = m_OBS(idx,4) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std));
	}

	void loadNoiseParamsInto( mrpt::srba::options::observation_noise_identity::parameters_t & p )
	{
		p.std_noise_observations = m_noise_std;
	}

};
