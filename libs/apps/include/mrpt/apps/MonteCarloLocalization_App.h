/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/apps/BaseAppDataSource.h>
#include <mrpt/apps/BaseAppInitializableCLI.h>
#include <mrpt/apps/DataSourceRawlog.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/COutputLogger.h>

namespace mrpt::apps
{
/** MonteCarlo (Particle filter) localization wrapper class for CLI or custom
 * applications: virtual base class, which can be used as base for custom
 * user robotic applications.
 *
 * \ingroup mrpt_apps_grp
 */
class MonteCarloLocalization_Base : virtual public mrpt::system::COutputLogger,
									public mrpt::apps::BaseAppInitializableCLI,
									virtual public mrpt::apps::BaseAppDataSource
{
   public:
	MonteCarloLocalization_Base();

	/** Default name of the main configuration section in INI files for this app
	 */
	constexpr static auto sect = "LocalizationExperiment";

	/** @name Main API
	 * @{ */

	/** Initializes the application from CLI parameters. Refer to the manpage of
	 * pf-localization. Throws on errors.
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

	/** If true, will watch the keyboard and quit when ESC is pushed. */
	bool allow_quit_on_esc_key = true;

	/** Whether to populate out_estimated_path */
	bool fill_out_estimated_path = false;

	/** @} */

	/** @name Outputs and result variables
	 * @{ */

	/** Controlled by flag `fill_out_estimated_path` */
	mrpt::poses::CPose3DInterpolator out_estimated_path;

	/** @} */

   protected:
	template <class MONTECARLO_TYPE>
	void do_pf_localization();

	void getGroundTruth(
		mrpt::poses::CPose2D& expectedPose, size_t rawlogEntry,
		const mrpt::math::CMatrixDouble& GT, const Clock::time_point& cur_time);
	void prepareGT(const mrpt::math::CMatrixDouble& GT);

	mrpt::poses::CPose2DInterpolator GT_path;
};

/** MonteCarlo (Particle filter) localization wrapper class, reading from a
 * rawlog dataset.
 *
 * \ingroup mrpt_apps_grp
 */
class MonteCarloLocalization_Rawlog : public MonteCarloLocalization_Base,
									  public DataSourceRawlog
{
   public:
	MonteCarloLocalization_Rawlog();

   protected:
	void impl_initialize(int argc, const char** argv) override;
	std::string impl_get_usage() const override
	{
		return "pf-localization <config_file> [dataset.rawlog]";
	}
};

}  // namespace mrpt::apps
