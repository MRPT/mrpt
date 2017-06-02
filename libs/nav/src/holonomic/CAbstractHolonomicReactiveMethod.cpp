/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::nav;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CAbstractHolonomicReactiveMethod, CSerializable, mrpt::nav)


CAbstractHolonomicReactiveMethod::CAbstractHolonomicReactiveMethod(const std::string &defaultCfgSectionName) :
	m_associatedPTG(NULL),
	m_enableApproachTargetSlowDown(true),
	m_cfgSectionName(defaultCfgSectionName)
{
}
CAbstractHolonomicReactiveMethod::~CAbstractHolonomicReactiveMethod()
{
}

/** Defines the name of the section (Default: "FULL_EVAL_CONFIG") */
void CAbstractHolonomicReactiveMethod::setConfigFileSectionName(const std::string &sectName)
{
	m_cfgSectionName = sectName;
}
std::string CAbstractHolonomicReactiveMethod::getConfigFileSectionName() const {
	return m_cfgSectionName;
}

void CAbstractHolonomicReactiveMethod::setAssociatedPTG(mrpt::nav::CParameterizedTrajectoryGenerator *ptg)
{
	m_associatedPTG = ptg;
}
mrpt::nav::CParameterizedTrajectoryGenerator * CAbstractHolonomicReactiveMethod::getAssociatedPTG() const
{
	return m_associatedPTG;
}

CAbstractHolonomicReactiveMethod * CAbstractHolonomicReactiveMethod::Create(const std::string &className) MRPT_NO_THROWS
{
	try 
	{
		mrpt::utils::registerAllPendingClasses();

		// Factory:
		const mrpt::utils::TRuntimeClassId *classId = mrpt::utils::findRegisteredClass( className );
		if (!classId) return NULL;

		CAbstractHolonomicReactiveMethod *holo = dynamic_cast<CAbstractHolonomicReactiveMethod*>( classId->createObject() );
		return holo;
	}
	catch (...)
	{
		return nullptr;
	}
}

CAbstractHolonomicReactiveMethod::NavInput::NavInput() :
	targets(),
	maxRobotSpeed(1.0),
	maxObstacleDist(1.0),
	clearance(nullptr)
{
}

CAbstractHolonomicReactiveMethod::NavOutput::NavOutput() :
	desiredDirection(0),
	desiredSpeed(0)
{
}
