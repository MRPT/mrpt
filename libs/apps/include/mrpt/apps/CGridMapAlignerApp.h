/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/slam/CGridMapAligner.h>
#include <mrpt/system/COutputLogger.h>

namespace mrpt::apps
{
/**
 *
 * \sa mrpt::slam::CGridMapAligner
 * \ingroup mrpt_apps_grp
 */
class CGridMapAlignerApp : public mrpt::system::COutputLogger
{
   public:
	/** @name Main API
	 * @{ */

	/** Initializes the application from CLI parameters. Refer to the manpage of
	 * grid-matching. Throws on errors.
	 */
	void initialize(int argc, const char** argv);

	/** Runs with the current parameter set. Throws on errors. */
	void run();

	/** @} */

	/** @name Parameters and options. See: initialize()
	 * @{ */

	bool SAVE_SOG_3DSCENE = false;
	bool SAVE_SOG_ALL_MAPS_OVERLAP_HYPOTHESES = false;
	bool SAVE_CORR_AND_NONCORR_DISTS = false;
	bool IS_VERBOSE = false;
	bool NOSAVE = false;
	bool SKIP_ICP_STAGE = false;
	bool MOST_LIKELY_SOG_MODE_ONLY = false;

	std::string SAVE_ICP_GOODNESS_FIL = "";

	// Mode of operation
	bool is_match = false, is_detect_test = false;

	std::string RESULTS_DIR = "GRID-MATCHING_RESULTS";
	std::string fil_grid1, fil_grid2;
	std::string OUTPUT_FIL;
	std::string CONFIG_FIL;

	double STD_NOISE_XY = 0, STD_NOISE_PHI = 0;
	double STD_NOISE_LASER = 0;
	double GT_Ax = 0, GT_Ay = 0, GT_Aphi_rad = 0;

	bool NOISE_IN_LASER = false;
	bool NOISE_IN_POSE = false;

	unsigned int N_ITERS = 1;

	mrpt::slam::CGridMapAligner::TAlignerMethod aligner_method =
	    mrpt::slam::CGridMapAligner::amModifiedRANSAC;

	/** @} */
};

}  // namespace mrpt::apps
