/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#pragma once

#include "CDatasetParserBase.h"

template <>
struct CDatasetParserTempl<mrpt::srba::observations::RelativePoses_2D> : public CDatasetParserBase
{
	double m_noise_std_xy,m_noise_std_yaw;

	CDatasetParserTempl(RBASLAM_Params &cfg) :
		CDatasetParserBase(cfg),
		m_noise_std_xy(1e-3),
		m_noise_std_yaw(1e-5)
	{
		if (cfg.arg_noise.isSet())
			m_noise_std_xy=cfg.arg_noise.getValue();
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
		o.obs_data.x   = m_OBS(idx,2) + mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_xy);
		o.obs_data.y   = m_OBS(idx,3) + mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_xy);
		o.obs_data.yaw = m_OBS(idx,5) + mrpt::random::randomGenerator.drawGaussian1D(0, m_noise_std_yaw);
	}

	void loadNoiseParamsInto( mrpt::srba::observation_noise_constant_matrix<mrpt::srba::observations::RelativePoses_2D>::parameters_t & p )
	{
		p.lambda.setZero();
		p.lambda(0,0) = m_noise_std_xy;
		p.lambda(1,1) = m_noise_std_xy;
		p.lambda(2,2) = m_noise_std_yaw;
	}

};
