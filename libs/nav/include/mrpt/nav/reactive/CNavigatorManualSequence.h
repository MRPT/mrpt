/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CAbstractNavigator.h>
#include <map>
#include <mrpt/utils/CTicTac.h>

namespace mrpt {
namespace nav 
{
	/** "Fake navigator" for tests: it just sends out a pre-programmed sequence of commands to the robot.
	 * For a short discussion of the API, see CNavigatorVirtualBase
	 */
	class NAV_IMPEXP CNavigatorManualSequence : public mrpt::nav::CAbstractNavigator
	{
	public:
		CNavigatorManualSequence( CRobot2NavInterface &react_iterf_impl );
		virtual ~CNavigatorManualSequence();

		/** @name Initialization API
		  * @{ */
		virtual void loadConfigFile(const mrpt::utils::CConfigFileBase &c) override; // See base class docs!
		virtual void saveConfigFile(mrpt::utils::CConfigFileBase &c) const override; // See base class docs!
		void initialize() override; //!< Must be called for loading collision grids, etc. before invoking any navigation command
		/** @} */

		/** Overriden in this class to ignore the cancel/pause/... commands */
		void navigationStep() override;

		struct TVelCmd
		{
			mrpt::kinematics::CVehicleVelCmd::Ptr cmd_vel; //!< all with the same meaning than in CRobot2NavInterface::changeSpeeds()
		};

		std::map<double,TVelCmd>  programmed_orders; //!< map [time_in_secs_since_beginning] -> orders.

	protected:
		virtual void onStartNewNavigation() override { }

	private:
		// Not used in this class:
		virtual void navigate( const TNavigationParams *params ) override { }
		virtual void  performNavigationStep( )  override { } 
	};
}	
}

