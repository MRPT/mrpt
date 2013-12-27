/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include "CDatasetParserBase.h"

template <>
struct CDatasetParserTempl<mrpt::srba::observations::RangeBearing_2D> : public CDatasetParserBase
{
	double m_noise_std_range;
	double m_noise_std_yaw;

	CDatasetParserTempl(RBASLAM_Params &cfg) :
		CDatasetParserBase(cfg),
		m_noise_std_range(1e-4),
		m_noise_std_yaw(1e-5)
	{
		if (cfg.arg_noise.isSet())
		{
			m_noise_std_range=cfg.arg_noise.getValue();
			m_noise_std_yaw=cfg.arg_noise.getValue();
		}
	}

	virtual void checkObsProperSize() const
	{
		// Columns: KeyframeIndex  LandmarkID | Range Yaw
		ASSERT_(m_OBS.getColCount()==(2+2))
	}

	void getObs(
		size_t idx,
		mrpt::srba::observation_traits<mrpt::srba::observations::RangeBearing_2D>::observation_t & o
		) const
	{
		o.feat_id        = m_OBS(idx,1);
		o.obs_data.range = m_OBS(idx,2) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_range));
		o.obs_data.yaw   = m_OBS(idx,3) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_yaw));
	}

	void loadNoiseParamsInto( mrpt::srba::options::observation_noise_identity::parameters_t & p )
	{
		p.std_noise_observations = m_noise_std_range;
	}

	//void loadNoiseParamsInto( mrpt::srba::observation_noise_identity::parameters_t & p )
	//{
	//	//p. ... = m_noise_std_range, m_noise_std_yaw
	//}


};
