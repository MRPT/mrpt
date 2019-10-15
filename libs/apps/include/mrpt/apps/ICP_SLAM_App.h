/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/COutputLogger.h>

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
class ICP_SLAM_App_Base : public mrpt::system::COutputLogger
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

	/** @} */

	/** @name Outputs and result variables
	 * @{ */

	std::vector<mrpt::math::TPose3D> out_estimated_path;

	/** @} */

   protected:
	virtual void impl_initialize(int argc, const char** argv) = 0;
	virtual bool impl_get_next_observations(
	    mrpt::obs::CActionCollection::Ptr& action,
	    mrpt::obs::CSensoryFrame::Ptr& observations,
	    mrpt::obs::CObservation::Ptr& observation) = 0;
	virtual std::string impl_get_usage() const = 0;
};

/** Instance of ICP_SLAM_App_Base to run mapping from an offline dataset file.
 */
class ICP_SLAM_App_Rawlog : public ICP_SLAM_App_Base
{
   public:
	ICP_SLAM_App_Rawlog();

   protected:
	void impl_initialize(int argc, const char** argv) override;
	bool impl_get_next_observations(
	    mrpt::obs::CActionCollection::Ptr& action,
	    mrpt::obs::CSensoryFrame::Ptr& observations,
	    mrpt::obs::CObservation::Ptr& observation) override;
	std::string impl_get_usage() const override
	{
		return "icp-slam <config_file> [dataset.rawlog]";
	}

   private:
	std::string m_rawlogFileName = "UNDEFINED.rawlog";
	unsigned int m_rawlog_offset;

	size_t m_rawlogEntry = 0;
	mrpt::io::CFileGZInputStream m_rawlog_io;
	mrpt::serialization::CArchive::UniquePtr m_rawlog_arch;
};

}  // namespace mrpt::apps
