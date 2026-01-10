/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::nav;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CAbstractHolonomicReactiveMethod, CSerializable, mrpt::nav)

CAbstractHolonomicReactiveMethod::CAbstractHolonomicReactiveMethod(
    const std::string& defaultCfgSectionName) :
    m_associatedPTG(nullptr),
    m_enableApproachTargetSlowDown(true),
    m_cfgSectionName(defaultCfgSectionName)
{
}
CAbstractHolonomicReactiveMethod::~CAbstractHolonomicReactiveMethod() = default;
/** Defines the name of the section (Default: "FULL_EVAL_CONFIG") */
void CAbstractHolonomicReactiveMethod::setConfigFileSectionName(const std::string& sectName)
{
  m_cfgSectionName = sectName;
}
std::string CAbstractHolonomicReactiveMethod::getConfigFileSectionName() const
{
  return m_cfgSectionName;
}

void CAbstractHolonomicReactiveMethod::setAssociatedPTG(
    mrpt::nav::CParameterizedTrajectoryGenerator* ptg)
{
  m_associatedPTG = ptg;
}
mrpt::nav::CParameterizedTrajectoryGenerator* CAbstractHolonomicReactiveMethod::getAssociatedPTG()
    const
{
  return m_associatedPTG;
}

CAbstractHolonomicReactiveMethod::Ptr CAbstractHolonomicReactiveMethod::Factory(
    const std::string& className) noexcept
{
  try
  {
    mrpt::rtti::registerAllPendingClasses();

    // Factory:
    const mrpt::rtti::TRuntimeClassId* classId = mrpt::rtti::findRegisteredClass(className);
    if (!classId) return nullptr;

    return mrpt::ptr_cast<CAbstractHolonomicReactiveMethod>::from(classId->createObject());
  }
  catch (...)
  {
    return CAbstractHolonomicReactiveMethod::Ptr();
  }
}
