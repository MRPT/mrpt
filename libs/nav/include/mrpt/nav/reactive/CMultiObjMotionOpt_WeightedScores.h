/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h>
#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
	namespace nav
	{
		DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE(CMultiObjMotionOpt_WeightedScores, NAV_IMPEXP)

			/** Virtual base class for multi-objective motion choosers, as used for reactive navigation engines.
			  *\sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
			  *  \ingroup nav_reactive
			  */
		class NAV_IMPEXP CMultiObjMotionOpt_WeightedScores: public mrpt::nav::CMultiObjectiveMotionOptimizerBase
		{
			// This must be added to any CSerializable derived class:
			DEFINE_MRPT_OBJECT(CMultiObjMotionOpt_WeightedScores)

		public:
			CMultiObjMotionOpt_WeightedScores();

			void loadConfigFile(const mrpt::utils::CConfigFileBase & c) MRPT_OVERRIDE;
			void saveConfigFile(mrpt::utils::CConfigFileBase & c) const MRPT_OVERRIDE;

		protected:
			// This virtual method is called by decide().
			int impl_decide(const std::vector<mrpt::nav::TCandidateMovementPTG> &movs, TResultInfo &extra_info) MRPT_OVERRIDE;
		};
		DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE(CMultiObjMotionOpt_WeightedScores, NAV_IMPEXP)

	}
}

