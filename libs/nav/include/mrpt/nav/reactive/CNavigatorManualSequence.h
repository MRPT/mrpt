/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CAbstractNavigator.h>
#include <map>
#include <mrpt/system/CTicTac.h>

namespace mrpt::nav
{
/** "Fake navigator" for tests: it just sends out a pre-programmed sequence of
 * commands to the robot.
 * For a short discussion of the API, see CNavigatorVirtualBase
 */
class CNavigatorManualSequence : public mrpt::nav::CAbstractNavigator
{
   public:
	CNavigatorManualSequence(CRobot2NavInterface& react_iterf_impl);
	~CNavigatorManualSequence() override;

	/** @name Initialization API
	 * @{ */
	void loadConfigFile(const mrpt::config::CConfigFileBase& c)
		override;  // See base class docs!
	void saveConfigFile(mrpt::config::CConfigFileBase& c)
		const override;  // See base class docs!
	/** Must be called for loading collision grids, etc. before invoking any
	 * navigation command */
	void initialize() override;
	/** @} */

	/** Overriden in this class to ignore the cancel/pause/... commands */
	void navigationStep() override;

	struct TVelCmd
	{
		/** all with the same meaning than in
		 * CRobot2NavInterface::changeSpeeds() */
		mrpt::kinematics::CVehicleVelCmd::Ptr cmd_vel;
	};

	/** map [time_in_secs_since_beginning] -> orders. */
	std::map<double, TVelCmd> programmed_orders;

   protected:
	void onStartNewNavigation() override {}

   private:
	// Not used in this class:
	void navigate(const TNavigationParams* params) override {}
	void performNavigationStep() override {}
};
}  // namespace mrpt::nav
