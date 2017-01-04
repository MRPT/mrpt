/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CNavigatorManualSequence.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/system/string_utils.h>

using namespace mrpt::nav;

CNavigatorManualSequence::CNavigatorManualSequence(CRobot2NavInterface &robot_interface) :
	CAbstractNavigator( robot_interface )
{
}

// Dtor:
CNavigatorManualSequence::~CNavigatorManualSequence()
{
}

void CNavigatorManualSequence::loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section_prefix)
{
	const std::string sSect = section_prefix+std::string("CMDS");

	programmed_orders.clear();
	mrpt::vector_string lstKeys;
	cfg.getAllKeys(sSect, lstKeys);

	for (size_t i=0;i<lstKeys.size();i++)
	{
		std::string s = cfg.read_string(sSect,lstKeys[i],"",true);
		std::vector<std::string> toks;
		mrpt::system::tokenize(s," \t\r\n",toks);
		ASSERTMSG_(toks.size()>2,std::string("Wrong format while parsing CNavigatorManualSequence cfg file in entry: ")+lstKeys[i]);
		
		const double t = atof(toks[0].c_str());
		TVelCmd krc;

		const size_t nComps = toks.size() - 1;
		switch (nComps)
		{
		case 2: krc.cmd_vel = mrpt::kinematics::CVehicleVelCmd_DiffDriven::Create(); break;
		case 4: krc.cmd_vel = mrpt::kinematics::CVehicleVelCmd_Holo::Create(); break;
		default:
			THROW_EXCEPTION("Expected 2 or 4 velocity components!");
		};

		for (size_t i=0;i<nComps;i++)
			krc.cmd_vel->setVelCmdElement(i,  atof(toks[i+1].c_str() ));

		// insert:
		programmed_orders[t] = krc;
	}
}

void CNavigatorManualSequence::initialize()
{
	ASSERT_(!programmed_orders.empty())
	m_robot.resetNavigationTimer();
}

/** Overriden in this class to ignore the cancel/pause/... commands */
void CNavigatorManualSequence::navigationStep()
{
	if (programmed_orders.empty())
		return;

	const double t = m_robot.getNavigationTime();

	if (t>=programmed_orders.begin()->first)
	{
		const TVelCmd &krc = programmed_orders.begin()->second;
		// Send cmd:
		logFmt( mrpt::utils::LVL_DEBUG, "[CNavigatorManualSequence] Sending cmd: t=%f\n",programmed_orders.begin()->first);

		if (!this->changeSpeeds(*krc.cmd_vel) )
		{
			this->stop(true /*not emergency*/);
			logFmt( mrpt::utils::LVL_ERROR, "[CNavigatorManualSequence] **ERROR** sending cmd to robot.");
			return;
		}
		// remove:
		programmed_orders.erase( programmed_orders.begin() );
	}
}


