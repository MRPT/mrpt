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

#include <mrpt/config/CConfigFilePrefixer.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/system/string_utils.h>

using namespace mrpt::nav;

/*---------------------------------------------------------------
          Class factory
  ---------------------------------------------------------------*/
CParameterizedTrajectoryGenerator::Ptr CParameterizedTrajectoryGenerator::CreatePTG(
    const std::string& ptgClassName_,
    const mrpt::config::CConfigFileBase& cfg,
    const std::string& sSection,
    const std::string& sKeyPrefix)
{
  MRPT_START

  mrpt::rtti::registerAllPendingClasses();

  // Special names for backwards compatibility with MRPT < 1.5.0
  std::string ptgClassName = mrpt::system::trim(ptgClassName_);
  if (ptgClassName.size() == 1)
  {
    switch (ptgClassName[0])
    {
      case '1':
        ptgClassName = "CPTG_DiffDrive_C";
        break;
      case '2':
        ptgClassName = "CPTG_DiffDrive_alpha";
        break;
      case '3':
        ptgClassName = "CPTG_DiffDrive_CCS";
        break;
      case '4':
        ptgClassName = "CPTG_DiffDrive_CC";
        break;
      case '5':
        ptgClassName = "CPTG_DiffDrive_CS";
        break;
    };
  }

  // Factory:
  const mrpt::rtti::TRuntimeClassId* classId = mrpt::rtti::findRegisteredClass(ptgClassName);
  if (!classId)
  {
    THROW_EXCEPTION_FMT("[CreatePTG] No PTG named `%s` is registered!", ptgClassName.c_str());
  }

  auto ptg = mrpt::ptr_cast<CParameterizedTrajectoryGenerator>::from(classId->createObject());
  if (!ptg)
  {
    THROW_EXCEPTION_FMT(
        "[CreatePTG] Object of type `%s` seems not to be a PTG!", ptgClassName.c_str());
  }

  // Wrapper to transparently add prefixes to all config keys:
  mrpt::config::CConfigFilePrefixer cfp;
  cfp.bind(cfg);
  cfp.setPrefixes("", sKeyPrefix);

  ptg->loadFromConfigFile(cfp, sSection);
  return ptg;
  MRPT_END
}
