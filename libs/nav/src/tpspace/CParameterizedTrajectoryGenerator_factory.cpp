/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>

using namespace mrpt::nav;

/*---------------------------------------------------------------
					Class factory
  ---------------------------------------------------------------*/
CParameterizedTrajectoryGenerator * CParameterizedTrajectoryGenerator::CreatePTG(const std::string &ptgClassName, mrpt::utils::TParameters<double> &params)
{
	MRPT_START

	const mrpt::utils::TRuntimeClassId *classId = mrpt::utils::findRegisteredClass( ptgClassName );
	if (!classId) {
		THROW_EXCEPTION_CUSTOM_MSG1("[CreatePTG] No PTG named `%s` is registered!",ptgClassName.c_str());
	}
	
	CParameterizedTrajectoryGenerator *ptg = dynamic_cast<CParameterizedTrajectoryGenerator*>( classId->createObject() );
	if (!ptg) {
		THROW_EXCEPTION_CUSTOM_MSG1("[CreatePTG] Object of type `%s` seems not to be a PTG!",ptgClassName.c_str());
	}

	ptg->setParams(params);
	return ptg;
	MRPT_END
}
