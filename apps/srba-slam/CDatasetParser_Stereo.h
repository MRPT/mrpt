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
struct CDatasetParserTempl<mrpt::srba::observations::StereoCamera> : public CDatasetParserBase
{
	double m_noise_std_px;

	CDatasetParserTempl(RBASLAM_Params &cfg) :
		CDatasetParserBase(cfg),
		m_noise_std_px(1e-4)
	{
		if (cfg.arg_noise.isSet())
			m_noise_std_px=cfg.arg_noise.getValue();
	}

	virtual void checkObsProperSize() const
	{
		// Columns: KeyframeIndex  LandmarkID | px_l.x px_l.y px_r.x px_r.y
		ASSERT_(m_OBS.getColCount()==(2+4))
	}

	void getObs(
		size_t idx,
		mrpt::srba::observation_traits<mrpt::srba::observations::StereoCamera>::observation_t & o
		) const
	{
		o.feat_id = m_OBS(idx,1);
		o.obs_data.left_px.x  = m_OBS(idx,2) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_px));
		o.obs_data.left_px.y  = m_OBS(idx,3) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_px));
		o.obs_data.right_px.x = m_OBS(idx,4) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_px));
		o.obs_data.right_px.y = m_OBS(idx,5) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_px));
	}

	void loadNoiseParamsInto( mrpt::srba::options::observation_noise_identity::parameters_t & p )
	{
		p.std_noise_observations = m_noise_std_px;
	}

	//void loadNoiseParamsInto( mrpt::srba::observation_noise_identity::parameters_t & p )
	//{
	//	//p. ... = m_noise_std_range, m_noise_std_yaw
	//}


};
