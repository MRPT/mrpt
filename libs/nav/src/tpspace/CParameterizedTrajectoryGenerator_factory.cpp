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
#include <mrpt/nav/tpspace/CPTG1.h>
#include <mrpt/nav/tpspace/CPTG2.h>
#include <mrpt/nav/tpspace/CPTG3.h>
#include <mrpt/nav/tpspace/CPTG4.h>
#include <mrpt/nav/tpspace/CPTG5.h>
#include <mrpt/nav/tpspace/CPTG6.h>
#include <mrpt/nav/tpspace/CPTG7.h>

using namespace mrpt::nav;

/*---------------------------------------------------------------
					Class factory
  ---------------------------------------------------------------*/
MRPT_TODO("rewrite with text-based ptg names");
CParameterizedTrajectoryGenerator * CParameterizedTrajectoryGenerator::CreatePTG(const mrpt::utils::TParameters<double> &params)
{
	MRPT_START
	const int nPTG = static_cast<int>( params["PTG_type"] );
	switch(nPTG)
	{
	case 1: return new CPTG1(params);
	case 2: return new CPTG2(params);
	case 3: return new CPTG3(params);
	case 4: return new CPTG4(params);
	case 5: return new CPTG5(params);
	case 6: return new CPTG6(params);
	case 7: return new CPTG7(params);

	default:
		THROW_EXCEPTION_CUSTOM_MSG1("Unknown PTG_type=%i",nPTG)
	};
	MRPT_END
}
