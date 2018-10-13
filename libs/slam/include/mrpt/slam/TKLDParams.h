/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CLoadableOptions.h>

namespace mrpt::slam
{
/** Option set for KLD algorithm.  \ingroup mrpt_slam_grp
 */
class TKLDParams : public mrpt::config::CLoadableOptions
{
   public:
	TKLDParams();

	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& source,
		const std::string& section) override;  // See base docs
	void dumpToTextStream(std::ostream& out) const override;  // See base docs

	/** Parameters for the KLD adaptive sample size algorithm (see Dieter Fox's
	 * papers), which is used only if the CParticleFilter is created with the
	 * "adaptiveSampleSize" flag set to true.
	 */
	double KLD_binSize_XY{0.2f}, KLD_binSize_PHI, KLD_delta{0.01f},
		KLD_epsilon{0.02f};

	/** Parameters for the KLD adaptive sample size algorithm (see Dieter Fox's
	 * papers), which is used only if the CParticleFilter is created with the
	 * "adaptiveSampleSize" flag set to true.
	 */
	unsigned int KLD_minSampleSize{250}, KLD_maxSampleSize{100000};

	/** (Default: KLD_minSamplesPerBin=0) The minimum number of samples will be
	 * the maximum of KLD_minSampleSize and KLD_minSamplesPerBin *
	 * #ofBinsOccupied in the last time step */
	double KLD_minSamplesPerBin{0};
};

}  // namespace mrpt::slam
