/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "serialization-precomp.h"  // Precompiled headers

#include <mrpt/serialization/CSchemeArchive.h>
#include <mrpt/core/exceptions.h>

// Check if we have jsoncpp to enable those tests:
#include <mrpt/config.h>
#if MRPT_HAS_JSONCPP
#include <json/json.h>
#endif

using namespace mrpt::serialization;

CSchemeArchiveBase mrpt::serialization::archiveJSON()
{
#if MRPT_HAS_JSONCPP
	return mrpt::serialization::CSchemeArchiveBase(
		std::make_unique<CSchemeArchive<Json::Value>>());
#else
	THROW_EXCEPTION("archiveJSON() requires building MRPT against jsoncpp");
#endif
}
