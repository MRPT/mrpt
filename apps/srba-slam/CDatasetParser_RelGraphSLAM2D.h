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
struct CDatasetParserTempl<mrpt::srba::observations::RelativePoses_2D> : public CDatasetParserBase
{
	double m_noise_std_xy,m_noise_std_yaw;

	CDatasetParserTempl(RBASLAM_Params &cfg) :
		CDatasetParserBase(cfg),
		m_noise_std_xy(0.10),
		m_noise_std_yaw(DEG2RAD(4))
	{
		if (cfg.arg_noise.isSet()) m_noise_std_xy=cfg.arg_noise.getValue();
		if (cfg.arg_noise_ang.isSet()) m_noise_std_yaw=DEG2RAD(cfg.arg_noise_ang.getValue());
	}

	virtual void checkObsProperSize() const
	{
		// Columns: KeyframeIndex  LandmarkID | X Y Z YAW PITCH ROLL  QR QX QY QZ
		ASSERT_(m_OBS.getColCount()==(2+10))
	}

	void getObs(
		size_t idx,
		mrpt::srba::observation_traits<mrpt::srba::observations::RelativePoses_2D>::observation_t & o
		) const
	{
		o.feat_id = m_OBS(idx,1);
		o.obs_data.x   = m_OBS(idx,2) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_xy));
		o.obs_data.y   = m_OBS(idx,3) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_xy));
		o.obs_data.yaw = m_OBS(idx,5) + (!m_add_noise ? .0 : mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_yaw));
	}

	void loadNoiseParamsInto( mrpt::srba::options::observation_noise_constant_matrix<mrpt::srba::observations::RelativePoses_2D>::parameters_t & p )
	{
		using mrpt::utils::square;
		p.lambda.setZero();
		p.lambda(0,0) = 1.0/square(m_noise_std_xy);
		p.lambda(1,1) = p.lambda(0,0);
		p.lambda(2,2) = 1.0/square(m_noise_std_yaw);
	}

};
