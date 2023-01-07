/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/apps/BaseAppDataSource.h>
#include <mrpt/apps/BaseAppInitializableCLI.h>
#include <mrpt/apps/DataSourceRawlog.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/system/COutputLogger.h>

#include <atomic>

namespace mrpt::apps
{
/** RBPF-SLAM virtual base class for application wrappers.
 *
 * This virtual base provides the common code to the application rbpf-slam.
 * It can be used by users to build their own RBPF-SLAM solution.
 *
 * \sa  mrpt::slam::CMetricMapBuilderRBPF
 * \ingroup mrpt_apps_grp
 */
class RBPF_SLAM_App_Base : virtual public mrpt::system::COutputLogger,
						   public mrpt::apps::BaseAppInitializableCLI,
						   virtual public mrpt::apps::BaseAppDataSource
{
   public:
	RBPF_SLAM_App_Base();

	/** @name Main API
	 * @{ */

	/** Initializes the application from CLI parameters. Refer to the manpage of
	 * rbpf-slam. Throws on errors.
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

	/** If enabled (default), stdin will be watched and application quits if ESC
	 * is pressed. */
	bool quits_with_esc_key = true;

	/** @} */

	/** @name Outputs and result variables
	 * @{ */
	std::shared_ptr<mrpt::slam::CMetricMapBuilderRBPF> mapBuilder;

	std::map<mrpt::system::TTimeStamp, mrpt::math::TPose3D> out_estimated_path;

	/** @} */
};

/** Instance of RBPF_SLAM_App_Base to run mapping from an offline dataset file.
 */
class RBPF_SLAM_App_Rawlog : public RBPF_SLAM_App_Base, public DataSourceRawlog
{
   public:
	RBPF_SLAM_App_Rawlog();

   protected:
	void impl_initialize(int argc, const char** argv) override;
	std::string impl_get_usage() const override
	{
		return "rbpf-slam <config_file> [dataset.rawlog]";
	}
};

}  // namespace mrpt::apps
