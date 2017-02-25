/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "kinematics-precomp.h"  // Precompiled header
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::kinematics;
using namespace mrpt::utils;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CVehicleVelCmd, CSerializable, mrpt::kinematics)

CVehicleVelCmd::CVehicleVelCmd()
{
}
CVehicleVelCmd::CVehicleVelCmd(const CVehicleVelCmd &other)
{
	*this = other;
}

CVehicleVelCmd::~CVehicleVelCmd()
{
}

std::string mrpt::kinematics::CVehicleVelCmd::asString() const
{
	std::string s;
	s += "(";
	for (size_t i = 0; i < getVelCmdLength(); i++) {
		s += mrpt::format("%s=%.03f ", getVelCmdDescription(i).c_str(), getVelCmdElement(i));
	}
	s += ")";
	return s;
}

CVehicleVelCmd & CVehicleVelCmd::operator =(const CVehicleVelCmd &other)
{
	const size_t nThis = this->getVelCmdLength();
	ASSERTMSG_(typeid(*this) == typeid(other), "Trying to copy incompatible classes");
	for (size_t i = 0; i < nThis; i++)
		this->setVelCmdElement(i, other.getVelCmdElement(i) );
	return *this;
}

void CVehicleVelCmd::TVelCmdParams::loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(robotMax_V_mps, double, cfg, section);
	MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(robotMax_W_degps, double, robotMax_W_radps, cfg, section);
	robotMax_W_radps = mrpt::utils::DEG2RAD(robotMax_W_radps);
	MRPT_LOAD_CONFIG_VAR(robotMinCurvRadius, double, cfg, section);
}

CVehicleVelCmd::TVelCmdParams::TVelCmdParams() :
	robotMax_V_mps(-1.),
	robotMax_W_radps(-1.),
	robotMinCurvRadius(-1.)
{}

