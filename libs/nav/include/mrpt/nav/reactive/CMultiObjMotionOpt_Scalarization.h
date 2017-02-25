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
		DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE(CMultiObjMotionOpt_Scalarization, NAV_IMPEXP)

			/** Implementation of multi-objective motion chooser using scalarization: a user-given formula is used to 
			  * collapse all the scores into a single scalar score. The candidate with the highest positive score is selected.
			  * Note that assert expressions are honored via the base class CMultiObjectiveMotionOptimizerBase
			  *
			  * \sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
			  * \ingroup nav_reactive
			  */
		class NAV_IMPEXP CMultiObjMotionOpt_Scalarization: public mrpt::nav::CMultiObjectiveMotionOptimizerBase
		{
			// This must be added to any CSerializable derived class:
			DEFINE_MRPT_OBJECT(CMultiObjMotionOpt_Scalarization)

		public:
			CMultiObjMotionOpt_Scalarization();

			void loadConfigFile(const mrpt::utils::CConfigFileBase & c) MRPT_OVERRIDE;
			void saveConfigFile(mrpt::utils::CConfigFileBase & c) const MRPT_OVERRIDE;

			struct NAV_IMPEXP TParams : public mrpt::nav::CMultiObjectiveMotionOptimizerBase::TParamsBase
			{
				std::string  scalar_score_formula;  //!< A formula that takes all/a subset of scores and generates a scalar global score.

				TParams();
				void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source, const std::string &section) MRPT_OVERRIDE; // See base docs
				void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg, const std::string &section) const MRPT_OVERRIDE; // See base docs
			};

			TParams parameters;

			virtual void clear() MRPT_OVERRIDE;

		protected:
			mrpt::math::CRuntimeCompiledExpression m_expr_scalar_formula;
			std::map<std::string, double>          m_expr_scalar_vars;

			// This virtual method is called by decide().
			int impl_decide(const std::vector<mrpt::nav::TCandidateMovementPTG> &movs, TResultInfo &extra_info) MRPT_OVERRIDE;
		};
		DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE(CMultiObjMotionOpt_Scalarization, NAV_IMPEXP)

	}
}

