/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef TKLDParams_H
#define TKLDParams_H

#include <mrpt/utils/CLoadableOptions.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		/** Option set for KLD algorithm.  \ingroup mrpt_slam_grp 
		  */
		class SLAM_IMPEXP TKLDParams : public utils::CLoadableOptions
		{
		public:
			TKLDParams();

			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void dumpToTextStream(mrpt::utils::CStream &out) const MRPT_OVERRIDE; // See base docs

			/** Parameters for the KLD adaptive sample size algorithm (see Dieter Fox's papers), which is used only if the CParticleFilter is created with the "adaptiveSampleSize" flag set to true.
			  */
			double	KLD_binSize_XY, KLD_binSize_PHI,
					KLD_delta, KLD_epsilon;

			/** Parameters for the KLD adaptive sample size algorithm (see Dieter Fox's papers), which is used only if the CParticleFilter is created with the "adaptiveSampleSize" flag set to true.
			  */
			unsigned int	KLD_minSampleSize, KLD_maxSampleSize;

			/** (Default: KLD_minSamplesPerBin=0) The minimum number of samples will be the maximum of KLD_minSampleSize and KLD_minSamplesPerBin * #ofBinsOccupied in the last time step */
			double	KLD_minSamplesPerBin; 

		};

	} // End of namespace
} // End of namespace

#endif
