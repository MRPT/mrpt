/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
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
		void loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const std::string &section_prefix="") MRPT_OVERRIDE; //!< Reload the configuration from a cfg source
		void initialize() MRPT_OVERRIDE; //!< Must be called for loading collision grids, etc. before invoking any navigation command
		/** @} */

		/** Overriden in this class to ignore the cancel/pause/... commands */
		void navigationStep() MRPT_OVERRIDE;

		struct TVelCmd
		{
			mrpt::kinematics::CVehicleVelCmdPtr cmd_vel; //!< all with the same meaning than in CRobot2NavInterface::changeSpeeds()
		};

		std::map<double,TVelCmd>  programmed_orders; //!< map [time_in_secs_since_beginning] -> orders.

	protected:
		virtual void onStartNewNavigation() MRPT_OVERRIDE { }

	private:
		// Not used in this class:
		virtual void navigate( const TNavigationParams *params ) MRPT_OVERRIDE { }
		virtual void  performNavigationStep( )  MRPT_OVERRIDE { } 
	};
}	
}

