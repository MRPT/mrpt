/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

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

	o << "Timestamp (UTC): " << mrpt::system::dateTimeToString(timestamp)
	  << std::endl;
	o << "  (as time_t): " << std::fixed << std::setprecision(5)
	  << mrpt::system::timestampTotime_t(timestamp) << std::endl;
	o << "  (as TTimestamp): " << timestamp << std::endl;
	o << std::endl;
}

std::string CAction::getDescriptionAsTextValue() const
{
	std::stringstream ss;
	getDescriptionAsText(ss);
	return ss.str();
}
