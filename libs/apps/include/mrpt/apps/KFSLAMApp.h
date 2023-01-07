/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/system/COutputLogger.h>

#include <memory>

namespace mrpt::apps
{
/** EKF-SLAM application wrapper class.
 *
 * \sa mrpt::slam::CRangeBearingKFSLAM2D, mrpt::slam::CRangeBearingKFSLAM
 * \ingroup mrpt_apps_grp
 */
class KFSLAMApp : public mrpt::system::COutputLogger
{
   public:
	KFSLAMApp();

	/** @name Main API
	 * @{ */

	/** Initializes the application from CLI parameters. Refer to the manpage of
	 * kf-slam. Throws on errors.
	 */
	void initialize(int argc, const char** argv);

	inline void initialize(int argc, char** argv)
	{
		initialize(argc, const_cast<const char**>(argv));
	}

	/** Runs with the current parameter set. Throws on errors. */
	void run();

	/** @} */

	/** @name Parameters and options. See: initialize()
	 * @{ */

	/** Populated in initialize(). Can be replaced or manipulated by the user
	 * after that and before run() to change the parameters loaded from INI
	 * file. */
	mrpt::config::CConfigFileMemory params;

	/** rawlog to process */
	std::string rawlogFileName;

	/** @} */

	/** @name Outputs and result variables
	 * @{ */

	/** Average localization error, when supplied with a ground-truth file */
	double loc_error_wrt_gt = 0;

	/** @} */

   private:
	template <class IMPL>
	void Run_KF_SLAM();
};

}  // namespace mrpt::apps
