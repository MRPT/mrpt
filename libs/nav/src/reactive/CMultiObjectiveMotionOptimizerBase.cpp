/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h>

using namespace mrpt::nav;
using namespace mrpt::utils;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(CMultiObjectiveMotionOptimizerBase, CObject, mrpt::nav)


CMultiObjectiveMotionOptimizerBase::CMultiObjectiveMotionOptimizerBase()
{
}

int CMultiObjectiveMotionOptimizerBase::decide(const std::vector<mrpt::nav::TCandidateMovementPTG>& movs, TResultInfo & extra_info)
{
	return impl_decide(movs, extra_info);
}

CMultiObjectiveMotionOptimizerBase * CMultiObjectiveMotionOptimizerBase::Create(const std::string &className) MRPT_NO_THROWS
{
	try 
	{
		mrpt::utils::registerAllPendingClasses();

		// Factory:
		const mrpt::utils::TRuntimeClassId *classId = mrpt::utils::findRegisteredClass(className);
		if (!classId) return nullptr;

		CMultiObjectiveMotionOptimizerBase *holo = dynamic_cast<CMultiObjectiveMotionOptimizerBase*>(classId->createObject());
		return holo;
	}
	catch (...)
	{
		return nullptr;
	}
}

