/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/nav/reactive/CNavigatorManualSequence.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>

using namespace mrpt::nav;

CNavigatorManualSequence::CNavigatorManualSequence(
	CRobot2NavInterface& robot_interface)
	: CAbstractNavigator(robot_interface)
{
}

// Dtor:
CNavigatorManualSequence::~CNavigatorManualSequence() = default;
void CNavigatorManualSequence::saveConfigFile(
	mrpt::config::CConfigFileBase& c) const
{
}

void CNavigatorManualSequence::loadConfigFile(
	const mrpt::config::CConfigFileBase& c)
{
	const std::string s = "CNavigatorManualSequence";

	programmed_orders.clear();
	std::vector<std::string> lstKeys;
	c.getAllKeys(s, lstKeys);

	for (const auto& lstKey : lstKeys)
	{
		std::string str = c.read_string(s, lstKey, "", true);
		std::vector<std::string> toks;
		mrpt::system::tokenize(str, " \t\r\n", toks);
		ASSERTMSG_(
			toks.size() > 2,
			std::string("Wrong format while parsing CNavigatorManualSequence "
						"cfg file in entry: ") +
				lstKey);

		const double t = atof(toks[0].c_str());
		TVelCmd krc;

		const size_t nComps = toks.size() - 1;
		switch (nComps)
		{
			case 2:
				krc.cmd_vel = mrpt::make_aligned_shared<
					mrpt::kinematics::CVehicleVelCmd_DiffDriven>();
				break;
			case 4:
				krc.cmd_vel = mrpt::make_aligned_shared<
					mrpt::kinematics::CVehicleVelCmd_Holo>();
				break;
			default:
				THROW_EXCEPTION("Expected 2 or 4 velocity components!");
		};

		for (size_t j = 0; j < nComps; j++)
			krc.cmd_vel->setVelCmdElement(j, atof(toks[j + 1].c_str()));

		// insert:
		programmed_orders[t] = krc;
	}
}

void CNavigatorManualSequence::initialize()
{
	ASSERT_(!programmed_orders.empty());
	m_robot.resetNavigationTimer();
}

/** Overriden in this class to ignore the cancel/pause/... commands */
void CNavigatorManualSequence::navigationStep()
{
	if (programmed_orders.empty()) return;

	const double t = m_robot.getNavigationTime();

	if (t >= programmed_orders.begin()->first)
	{
		const TVelCmd& krc = programmed_orders.begin()->second;
		// Send cmd:
		logFmt(
			mrpt::system::LVL_DEBUG,
			"[CNavigatorManualSequence] Sending cmd: t=%f\n",
			programmed_orders.begin()->first);

		if (!this->changeSpeeds(*krc.cmd_vel))
		{
			this->stop(true /*not emergency*/);
			logFmt(
				mrpt::system::LVL_ERROR,
				"[CNavigatorManualSequence] **ERROR** sending cmd to robot.");
			return;
		}
		// remove:
		programmed_orders.erase(programmed_orders.begin());
	}
}
