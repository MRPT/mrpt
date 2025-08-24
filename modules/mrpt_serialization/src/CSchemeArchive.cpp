/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/exceptions.h>
#include <mrpt/serialization/CSchemeArchive.h>

// Check if we have jsoncpp to enable those tests:
#include <mrpt/serialization/config.h>
#if MRPT_HAS_JSONCPP
#include <json/json.h>
#endif

using namespace mrpt::serialization;

CSchemeArchiveBase mrpt::serialization::archiveJSON()
{
#if MRPT_HAS_JSONCPP
  return mrpt::serialization::CSchemeArchiveBase(std::make_unique<CSchemeArchive<Json::Value>>());
#else
  THROW_EXCEPTION("archiveJSON() requires building MRPT against jsoncpp");
#endif
}
