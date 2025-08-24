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

#include "obs-precomp.h"  // Precompiled headers
//
#include <mrpt/obs/CAction.h>
#include <mrpt/serialization/CArchive.h>

#include <iomanip>
#include <iostream>
#include <sstream>

using namespace mrpt::obs;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CAction, CSerializable, mrpt::obs)

void CAction::getDescriptionAsText(std::ostream& o) const
{
  using namespace mrpt::system;  // for the TTimeStamp << op

  o << mrpt::format(
      "Timestamp (UTC): %s\n"
      "        (local): %s\n"
      "    (as time_t): %.09f\n",
      mrpt::system::dateTimeToString(timestamp).c_str(),
      mrpt::system::dateTimeLocalToString(timestamp).c_str(), mrpt::Clock::toDouble(timestamp));

  o << "ClassName: " << this->GetRuntimeClass()->className << "\n"
    << "\n";
}

std::string CAction::getDescriptionAsTextValue() const
{
  std::stringstream ss;
  getDescriptionAsText(ss);
  return ss.str();
}
