/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/gnss_messages_common.h>

using namespace std;
using namespace mrpt::obs::gnss;

void gnss_message_binary_block::writeToStream(mrpt::utils::CStream &out) const {
	out << m_content_len;
	out.WriteBuffer(m_content_ptr,m_content_len);
}
void gnss_message_binary_block::readFromStream(mrpt::utils::CStream &in) {
	uint32_t nBytesInStream; in >> nBytesInStream;
	ASSERT_EQUAL_(nBytesInStream,m_content_len);
	in.ReadBuffer(m_content_ptr,m_content_len);
}

// ---------------------------------------
UTC_time::UTC_time() :
	hour(0), minute(0), sec(0)
{
}
void UTC_time::writeToStream(mrpt::utils::CStream &out) const {
	out << hour << minute << sec;
}
void UTC_time::readFromStream(mrpt::utils::CStream &in) {
	in >> hour >> minute >> sec;
}

// Build an MRPT timestamp with the hour/minute/sec of this structure and the date from the given timestamp.
mrpt::system::TTimeStamp UTC_time::getAsTimestamp(const mrpt::system::TTimeStamp &date) const
{
	using namespace mrpt::system;

	TTimeParts parts;
	timestampToParts(date,parts, false /* UTC, not local */);

	parts.hour   = this->hour;
	parts.minute = this->minute;
	parts.second = this->sec;

	return buildTimestampFromParts(parts);
}
