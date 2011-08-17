/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef TKLDParams_H
#define TKLDParams_H

#include <mrpt/utils/CLoadableOptions.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		using namespace mrpt::utils;

		/** Option set for KLD algorithm.  \ingroup mrpt_slam_grp 
		  */
		class SLAM_IMPEXP TKLDParams : public utils::CLoadableOptions
		{
		public:
			TKLDParams();

			/** See utils::CLoadableOptions
			  */
			void  loadFromConfigFile(
				const mrpt::utils::CConfigFileBase  &source,
				const std::string &section);

			/** See utils::CLoadableOptions
			  */
			void  dumpToTextStream(CStream	&out) const;


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
