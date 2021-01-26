/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
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
#include <mrpt/system/COutputLogger.h>

#include <atomic>

namespace mrpt::apps
{
/** ICP-SLAM virtual base class for application wrappers.
 *
 * This virtual base provides the common code to the applications icp-slam and
 * icp-slam-live, and could be used by users to build their own ICP-SLAM
 * solution.
 *
 * \sa  mrpt::slam::CMetricMapBuilderICP
 * \ingroup mrpt_apps_grp
 */
class ICP_SLAM_App_Base : virtual public mrpt::system::COutputLogger,
						  public mrpt::apps::BaseAppInitializableCLI,
						  virtual public mrpt::apps::BaseAppDataSource
{
   public:
	ICP_SLAM_App_Base();

	/** @name Main API
	 * @{ */

	/** Initializes the application from CLI parameters. Refer to the manpage of
	 * icp-slam and icp-slam-live. Throws on errors.
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

	std::map<mrpt::system::TTimeStamp, mrpt::math::TPose3D> out_estimated_path;

	/** @} */
};

/** Instance of ICP_SLAM_App_Base to run mapping from an offline dataset file.
 */
class ICP_SLAM_App_Rawlog : public ICP_SLAM_App_Base, public DataSourceRawlog
{
   public:
	ICP_SLAM_App_Rawlog();

   protected:
	void impl_initialize(int argc, const char** argv) override;
	std::string impl_get_usage() const override
	{
		return "icp-slam <config_file> [dataset.rawlog]";
	}
};

/** Instance of ICP_SLAM_App_Base to run mapping from a live LIDAR sensor.
 */
class ICP_SLAM_App_Live : public ICP_SLAM_App_Base
{
   public:
	ICP_SLAM_App_Live();
	virtual ~ICP_SLAM_App_Live() override;

   protected:
	void impl_initialize(int argc, const char** argv) override;
	std::string impl_get_usage() const override
	{
		return "icp-slam-live <config_file>";
	}

	bool impl_get_next_observations(
		mrpt::obs::CActionCollection::Ptr& action,
		mrpt::obs::CSensoryFrame::Ptr& observations,
		mrpt::obs::CObservation::Ptr& observation) override;

	struct TThreadParams
	{
		mrpt::config::CConfigFileBase* cfgFile;
		std::string section_name;
	};

	void SensorThread(TThreadParams params);

	mrpt::hwdrivers::CGenericSensor::TListObservations m_global_list_obs;
	std::mutex m_cs_global_list_obs;
	std::atomic_bool m_allThreadsMustExit = false;
};

}  // namespace mrpt::apps
