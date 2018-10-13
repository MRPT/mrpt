/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/gnss_messages.h>  // Must include all message classes so we can implemente the class factory here
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>
#include <map>

using namespace std;
using namespace mrpt::obs::gnss;

#define LIST_ALL_MSGS                     \
	/* ====== NMEA ======  */             \
	DOFOR(NMEA_GGA)                       \
	DOFOR(NMEA_RMC)                       \
	DOFOR(NMEA_ZDA)                       \
	DOFOR(NMEA_VTG)                       \
	DOFOR(NMEA_GLL)                       \
	/* ====== TopCon mmGPS ====== */      \
	DOFOR(TOPCON_PZS)                     \
	DOFOR(TOPCON_SATS)                    \
	/* ====== Novatel OEM6 ======  */     \
	DOFOR(NV_OEM6_GENERIC_FRAME)          \
	DOFOR(NV_OEM6_BESTPOS)                \
	/* ====== Novatel SPAN+OEM6 ====== */ \
	DOFOR(NV_OEM6_GENERIC_SHORT_FRAME)    \
	DOFOR(NV_OEM6_INSPVAS)                \
	DOFOR(NV_OEM6_RANGECMP)               \
	DOFOR(NV_OEM6_RXSTATUS)               \
	DOFOR(NV_OEM6_RAWEPHEM)               \
	DOFOR(NV_OEM6_VERSION)                \
	DOFOR(NV_OEM6_RAWIMUS)                \
	DOFOR(NV_OEM6_MARKPOS)                \
	DOFOR(NV_OEM6_MARKTIME)               \
	DOFOR(NV_OEM6_MARK2TIME)              \
	DOFOR(NV_OEM6_IONUTC)

// Class factory:
gnss_message* gnss_message::Factory(const gnss_message_type_t msg_id)
{
#define DOFOR(_MSG_ID) \
	case _MSG_ID:      \
		return new Message_##_MSG_ID();
	switch (msg_id)
	{
		LIST_ALL_MSGS
		default:
			return nullptr;
	};
#undef DOFOR
}
bool gnss_message::FactoryKnowsMsgType(const gnss_message_type_t msg_id)
{
#define DOFOR(_MSG_ID) \
	case _MSG_ID:      \
		return true;
	switch (msg_id)
	{
		LIST_ALL_MSGS
		default:
			return false;
	};
#undef DOFOR
}

const std::string& gnss_message::getMessageTypeAsString() const
{
	static bool first_call = true;
	static std::map<gnss_message_type_t, std::string> gnss_type2str;
	if (first_call)
	{
		first_call = false;
#define DOFOR(_MSG_ID) gnss_type2str[_MSG_ID] = #_MSG_ID;
		LIST_ALL_MSGS
#undef DOFOR
	}

	return gnss_type2str[this->message_type];
}

// Save to binary stream. Launches an exception upon error
void gnss_message::writeToStream(mrpt::serialization::CArchive& out) const
{
	out.WriteAs<int32_t>(message_type);
	this->internal_writeToStream(out);
}

// Load from binary stream. Launches an exception upon error
void gnss_message::readFromStream(mrpt::serialization::CArchive& in)
{
	int32_t msg_id;
	in >> msg_id;
	ASSERT_EQUAL_((int32_t)msg_id, this->message_type);
	this->internal_readFromStream(in);
}

// Load from binary stream and creates object detecting its type (class
// factory). Launches an exception upon error
gnss_message* gnss_message::readAndBuildFromStream(
	mrpt::serialization::CArchive& in)
{
	int32_t msg_id;
	in >> msg_id;
	gnss_message* msg =
		gnss_message::Factory(static_cast<gnss_message_type_t>(msg_id));
	if (!msg)
		THROW_EXCEPTION_FMT(
			"Error deserializing gnss_message: unknown message type '%i'",
			static_cast<int>(msg_id));
	msg->internal_readFromStream(in);
	return msg;
}

// Ctor (default: nullptr pointer)
gnss_message_ptr::gnss_message_ptr() = default;
// Ctor:Makes a copy of the pointee
gnss_message_ptr::gnss_message_ptr(const gnss_message_ptr& o)
{
	if (!o.ptr)
	{
		ptr = nullptr;
	}
	else
	{
		mrpt::io::CMemoryStream buf;
		auto arch = mrpt::serialization::archiveFrom(buf);
		o->writeToStream(arch);
		buf.Seek(0);
		ptr = gnss_message::readAndBuildFromStream(arch);
	}
}
/** Assigns a pointer */
gnss_message_ptr::gnss_message_ptr(const gnss_message* p)
	: ptr(const_cast<gnss_message*>(p))
{
}
void gnss_message_ptr::set(gnss_message* p)
{
	if (ptr)
	{
		delete ptr;
		ptr = nullptr;
	}
	ptr = p;
}
// Makes a copy of the pointee
gnss_message_ptr& gnss_message_ptr::operator=(const gnss_message_ptr& o)
{
	mrpt::io::CMemoryStream buf;
	auto arch = mrpt::serialization::archiveFrom(buf);
	o->writeToStream(arch);
	buf.Seek(0);
	ptr = gnss_message::readAndBuildFromStream(arch);
	return *this;
}
gnss_message_ptr::~gnss_message_ptr()
{
	if (ptr)
	{
		delete ptr;
		ptr = nullptr;
	}
}

// ---------------------------------------
UTC_time::UTC_time() = default;
void UTC_time::writeToStream(mrpt::serialization::CArchive& out) const
{
	out << hour << minute << sec;
}
void UTC_time::readFromStream(mrpt::serialization::CArchive& in)
{
	in >> hour >> minute >> sec;
}

// Build an MRPT timestamp with the hour/minute/sec of this structure and the
// date from the given timestamp.
mrpt::system::TTimeStamp UTC_time::getAsTimestamp(
	const mrpt::system::TTimeStamp& date) const
{
	using namespace mrpt::system;

	TTimeParts parts;
	timestampToParts(date, parts, false /* UTC, not local */);

	parts.hour = this->hour;
	parts.minute = this->minute;
	parts.second = this->sec;

	return buildTimestampFromParts(parts);
}
