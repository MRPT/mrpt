/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/gnss_messages.h> // Must include all message classes so we can implemente the class factory here
#include <mrpt/utils/CMemoryStream.h>

using namespace std;
using namespace mrpt::obs::gnss;

// Class factory:
gnss_message*  gnss_message::Factory(const gnss_message_type_t msg_id)
{
#define CASE_FACTORY(_MSG_ID)  case _MSG_ID: return new Message_##_MSG_ID();
	switch (msg_id)
	{
	// ====== NMEA ====== 
	CASE_FACTORY(NMEA_GGA)
	CASE_FACTORY(NMEA_RMC)

	// ====== TopCon mmGPS ====== 
	CASE_FACTORY(TOPCON_PZS)
	CASE_FACTORY(TOPCON_SATS)

	// ====== Novatel OEM6 ====== 
	CASE_FACTORY(NV_OEM6_GENERIC_FRAME)
	CASE_FACTORY(NV_OEM6_BESTPOS)

	// ====== Novatel SPAN+OEM6 ====== 
	CASE_FACTORY(NV_OEM6_GENERIC_SHORT_FRAME)
	CASE_FACTORY(NV_OEM6_INSPVAS)
	CASE_FACTORY(NV_OEM6_RANGECMP)
	CASE_FACTORY(NV_OEM6_RXSTATUS)
	CASE_FACTORY(NV_OEM6_RAWEPHEM)
	CASE_FACTORY(NV_OEM6_VERSION)
	CASE_FACTORY(NV_OEM6_RAWIMUS)
	CASE_FACTORY(NV_OEM6_MARKPOS)
	CASE_FACTORY(NV_OEM6_MARKTIME)
	CASE_FACTORY(NV_OEM6_MARK2TIME)

	default:
		return NULL;
	};

#undef CASE_FACTORY
}

// Save to binary stream. Launches an exception upon error
void gnss_message::writeToStream(mrpt::utils::CStream &out) const
{
	const int32_t msg_id = message_type;
	out << msg_id;
	this->internal_writeToStream(out);
}

// Load from binary stream. Launches an exception upon error
void gnss_message::readFromStream(mrpt::utils::CStream &in)
{
	int32_t msg_id;
	in >> msg_id;
	ASSERT_EQUAL_((int32_t)msg_id,this->message_type);
	this->internal_readFromStream(in);
}

// Load from binary stream and creates object detecting its type (class factory). Launches an exception upon error
gnss_message* gnss_message::readAndBuildFromStream(mrpt::utils::CStream &in)
{
	int32_t msg_id;
	in >> msg_id;
	gnss_message* msg = gnss_message::Factory(static_cast<gnss_message_type_t>(msg_id) );
	MRPT_TODO("Attemp to recognize generic frames implemented in a newer version of mrpt:")
	if (!msg)
		THROW_EXCEPTION_CUSTOM_MSG1("Error deserializing gnss_message: unknown message type '%i'",static_cast<int>(msg_id));
	msg->internal_readFromStream(in);
	return msg;
}


// Ctor (default: NULL pointer)
gnss_message_ptr::gnss_message_ptr() : ptr(NULL)
{}
// Ctor:Makes a copy of the pointee
gnss_message_ptr::gnss_message_ptr(const gnss_message_ptr &o)
{
	if (!o.ptr) {
		ptr=NULL;
	}
	else {
		mrpt::utils::CMemoryStream buf;
		o->writeToStream(buf);
		buf.Seek(0);
		ptr = gnss_message::readAndBuildFromStream(buf);
	}
}
/** Assigns a pointer */
gnss_message_ptr::gnss_message_ptr(const gnss_message* p) : 
	ptr(const_cast<gnss_message*>(p)) 
{ 
}
void gnss_message_ptr::set(gnss_message* p)
{
	if (ptr) { delete ptr; ptr=NULL; }
	ptr = p;
}
// Makes a copy of the pointee
gnss_message_ptr &gnss_message_ptr::operator =(const gnss_message_ptr&o)
{
	mrpt::utils::CMemoryStream buf;
	o->writeToStream(buf);
	buf.Seek(0);
	ptr = gnss_message::readAndBuildFromStream(buf);
	return *this;
}
gnss_message_ptr::~gnss_message_ptr()
{
	if (ptr) { delete ptr; ptr=NULL; }
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
