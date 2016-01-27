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
	switch (msg_id)
	{
	// ====== NMEA ====== 
	case NMEA_GGA: return new Message_NMEA_GGA();
	case NMEA_RMC: return new Message_NMEA_RMC();
	
	// ====== TopCon mmGPS ====== 
	case TOPCON_PZS:  return new Message_TopCon_PZS();
	case TOPCON_SATS: return new Message_TopCon_SATS();

	// ====== Novatel OEM6 ====== 

	// ====== Novatel SPAN+OEM6 ====== 

	default: 
		return NULL;
	};
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
