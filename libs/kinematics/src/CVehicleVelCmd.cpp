/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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
	size_t nThis = this->getVelCmdLength(), nOther = other.getVelCmdLength();
	ASSERTMSG_(nThis==nOther, "Trying to copy incompatible classes");
	for (size_t i = 0; i < nThis; i++)
		this->setVelCmdElement(i, other.getVelCmdElement(i) );
	return *this;
}
