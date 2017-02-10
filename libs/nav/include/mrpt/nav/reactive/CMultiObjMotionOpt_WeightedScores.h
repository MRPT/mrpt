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

			/** Implementation of multi-objective motion chooser, using a simple rule: each score has an associated weight, 
			  * the candidate with the highest final evaluation wins.
			  * \sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
			  * \ingroup nav_reactive
			  */
		class NAV_IMPEXP CMultiObjMotionOpt_WeightedScores: public mrpt::nav::CMultiObjectiveMotionOptimizerBase
		{
			// This must be added to any CSerializable derived class:
			DEFINE_MRPT_OBJECT(CMultiObjMotionOpt_WeightedScores)

		public:
			CMultiObjMotionOpt_WeightedScores();

			void loadConfigFile(const mrpt::utils::CConfigFileBase & c) MRPT_OVERRIDE;
			void saveConfigFile(mrpt::utils::CConfigFileBase & c) const MRPT_OVERRIDE;

			struct NAV_IMPEXP TParams : public mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase
			{
				TParams();
				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source, const std::string &section) MRPT_OVERRIDE; // See base docs
				void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg, const std::string &section) const MRPT_OVERRIDE; // See base docs

			};

			TParams parameters;

		protected:
			// This virtual method is called by decide().
			int impl_decide(const std::vector<mrpt::nav::TCandidateMovementPTG> &movs, TResultInfo &extra_info) MRPT_OVERRIDE;
		};
		DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE(CMultiObjMotionOpt_WeightedScores, NAV_IMPEXP)

	}
}

