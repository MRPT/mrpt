/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/TCandidateMovementPTG.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CObject.h>
#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
	namespace nav
	{
		DEFINE_MRPT_OBJECT_PRE_CUSTOM_LINKAGE(CMultiObjectiveMotionOptimizerBase, NAV_IMPEXP)

		/** Virtual base class for multi-objective motion choosers, as used for reactive navigation engines.
		  *\sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
		  *  \ingroup nav_reactive
		  */
		class NAV_IMPEXP CMultiObjectiveMotionOptimizerBase :
			public mrpt::utils::CObject
		{
			// This must be added to any CSerializable derived class:
			DEFINE_VIRTUAL_MRPT_OBJECT(CMultiObjectiveMotionOptimizerBase)
		public:
			CMultiObjectiveMotionOptimizerBase();

			static CMultiObjectiveMotionOptimizerBase * Create(const std::string &className) MRPT_NO_THROWS; //!< Class factory from C++ class name

			struct NAV_IMPEXP TResultInfo
			{

			};

			/** The main entry point for the class: returns the 0-based index of the best of the N motion candidates in `movs`. 
			  * If no valid one is found, `-1` will be returned.
			  */
			int decide(const std::vector<mrpt::nav::TCandidateMovementPTG> &movs, TResultInfo &extra_info);

			virtual void loadConfigFile(const mrpt::utils::CConfigFileBase & c) = 0;
			virtual void saveConfigFile(mrpt::utils::CConfigFileBase & c) const = 0;

			/** Common params for all children */
			struct NAV_IMPEXP TParamsBase : public mrpt::utils::CLoadableOptions
			{

			};

		protected:
			// This virtual method is called by decide().
			virtual int impl_decide(const std::vector<mrpt::nav::TCandidateMovementPTG> &movs, TResultInfo &extra_info) = 0;

		};
		DEFINE_MRPT_OBJECT_POST_CUSTOM_LINKAGE(CMultiObjectiveMotionOptimizerBase, NAV_IMPEXP)

	}
}

